#!/usr/bin/env python3
"""Convert JSON SMAP obstacle points to a ROS map, preserving unknown space.

Requires Pillow. Optional --free-polygon accepts a JSON array of map-frame
[x, y] vertices delimiting an explicitly confirmed traversable area.
"""
import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


def point(value):
    """Validate a finite metric coordinate without accepting boolean numbers."""
    if not isinstance(value, dict):
        raise ValueError('Expected point object with x/y')
    xy = [value.get('x'), value.get('y')]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xy):
        raise ValueError('Point coordinates must be finite numbers')
    return tuple(xy)


def convert(source, prefix, free_polygon=None, overwrite=False, assume_free=False, feature_lines=False):
    """Write a metric ROS map, with free-space assumptions explicitly selected."""
    if assume_free and free_polygon is not None:
        raise ValueError("Choose either bounds free space or a free polygon")
    source, prefix = Path(source), Path(prefix)
    paths = {name: Path(str(prefix)+suffix) for name, suffix in
             [('image', '.pgm'), ('yaml', '.yaml'), ('preview', '.png'), ('report', '.report.json')]}
    if source.resolve() in {p.resolve() for p in paths.values()}:
        raise ValueError('Output must not replace the input SMAP')
    if not overwrite and any(p.exists() for p in paths.values()):
        raise ValueError('Output already exists; choose a new prefix or pass --overwrite')
    data = json.loads(source.read_text())
    header = data['header']
    if str(header.get('mapType', '')).lower() != '2d-map':
        raise ValueError('Only 2D-Map JSON SMAP is supported')
    xmin, ymin = point(header['minPos'])
    xmax, ymax = point(header['maxPos'])
    resolution = header['resolution']
    if isinstance(resolution, bool) or not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Resolution must be finite and positive')
    if xmax <= xmin or ymax <= ymin:
        raise ValueError('Invalid map bounds')
    # Two unknown cells outside each boundary, including the maximum point cell.
    ox, oy = xmin-2*resolution, ymin-2*resolution
    width = math.ceil((xmax-xmin)/resolution)+5
    height = math.ceil((ymax-ymin)/resolution)+5
    if width*height > 25_000_000:
        raise ValueError('Map exceeds 25 million cells')
    image = Image.new('L', (width, height), 205)

    def pixel(x, y):
        col = math.floor((x-ox)/resolution+1e-9)
        row = height-1-math.floor((y-oy)/resolution+1e-9)
        return col, row

    if assume_free:
        left, bottom = pixel(xmin, ymin)
        right, top = pixel(xmax, ymax)
        ImageDraw.Draw(image).rectangle((left, top, right, bottom), fill=254)

    if free_polygon is not None:
        vertices = json.loads(Path(free_polygon).read_text())
        if not isinstance(vertices, list) or len(vertices) < 3:
            raise ValueError('Free polygon must contain at least three [x,y] vertices')
        coords = []
        for vertex in vertices:
            if not isinstance(vertex, list) or len(vertex) != 2:
                raise ValueError('Free polygon vertices must be [x,y] arrays')
            x, y = point(dict(zip(('x', 'y'), vertex)))
            if not xmin <= x <= xmax or not ymin <= y <= ymax:
                raise ValueError('Free polygon vertex is outside map bounds')
            coords.append(pixel(x, y))
        if abs(sum(coords[i][0]*coords[(i+1) % len(coords)][1]-coords[(i+1) % len(coords)][0]*coords[i][1]
                   for i in range(len(coords)))) < 1:
            raise ValueError('Free polygon must have nonzero area')
        ImageDraw.Draw(image).polygon(coords, fill=254)

    points = data.get('normalPosList', [])
    if not isinstance(points, list) or not points:
        raise ValueError('SMAP has no normalPosList obstacle points')
    for entry in points:
        x, y = point(entry)
        if not xmin-1e-6 <= x <= xmax+1e-6 or not ymin-1e-6 <= y <= ymax+1e-6:
            raise ValueError('Obstacle point is outside header bounds')
        image.putpixel(pixel(x, y), 0)

    drawn_lines = 0
    if feature_lines:
        for entry in data.get('advancedLineList', []):
            if entry.get('className') != 'FeatureLine':
                continue
            line = entry['line']
            # Protobuf JSON can omit zero-valued coordinate components.
            start = point({'x': 0.0, 'y': 0.0, **line['startPos']})
            end = point({'x': 0.0, 'y': 0.0, **line['endPos']})
            # Features may extend a few millimetres past the point-cloud bounds.
            # Clip rendering to the valid data rectangle, retaining unknown padding.
            mask = Image.new('L', image.size, 0)
            ImageDraw.Draw(mask).line([pixel(*start), pixel(*end)], fill=255, width=1)
            left, bottom = pixel(xmin, ymin)
            right, top = pixel(xmax, ymax)
            cropped = mask.crop((left, top, right+1, bottom+1))
            image.paste(0, (left, top, right+1, bottom+1), cropped)
            drawn_lines += 1

    counts = image.histogram()
    report = {
        'source': str(source.resolve()), 'map_name': header.get('mapName'),
        'resolution': resolution, 'origin': [ox, oy, 0.0], 'width': width, 'height': height,
        'obstacle_points': len(points), 'occupied_cells': counts[0],
        'free_cells': counts[254], 'unknown_cells': counts[205],
        'free_space_policy': ('user-confirmed bounds interior' if assume_free else
                              'explicit user polygon' if free_polygon else 'none; absent points remain unknown'),
        'feature_lines_drawn': drawn_lines,
        'ignored_fields': [k for k in data if k not in ('header', 'normalPosList')
                           and not (feature_lines and k == 'advancedLineList')],
        'note': 'Only explicitly selected FeatureLine geometry is rasterized; routes and semantic restrictions are not imported.',
    }
    prefix.parent.mkdir(parents=True, exist_ok=True)
    # Write P5 directly: the first image row is map north, YAML origin is southwest.
    paths['image'].write_bytes(f'P5\n{width} {height}\n255\n'.encode()+image.tobytes())
    paths['yaml'].write_text(
        f'image: {json.dumps(paths["image"].name, ensure_ascii=False)}\nmode: trinary\n'
        f'resolution: {resolution}\norigin: [{ox}, {oy}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    preview = image.convert('RGB')
    # Green outline indicates a user-confirmed area; it never alters the PGM.
    if free_polygon is not None:
        ImageDraw.Draw(preview).line(coords+[coords[0]], fill=(0, 160, 0), width=2)
    preview.save(paths['preview'])
    paths['report'].write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output-prefix', required=True, type=Path)
    parser.add_argument('--free-polygon', type=Path, help='Explicitly confirmed free-area vertices in a JSON file')
    parser.add_argument('--assume-free-within-bounds', action='store_true',
                        help='User confirms blank space inside SMAP header bounds is traversable')
    parser.add_argument('--include-feature-lines', action='store_true',
                        help='User confirms FeatureLine segments represent physical obstacles')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        report = convert(args.source, args.output_prefix, args.free_polygon, args.overwrite,
                         args.assume_free_within_bounds, args.include_feature_lines)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f'Conversion failed: {error}\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report['free_cells']:
        print('No free space inferred: this is an obstacle/unknown map, not a validated navigable map.')


if __name__ == '__main__':
    main()
