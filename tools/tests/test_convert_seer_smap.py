"""Coordinate, occupancy and overwrite regressions for the SMAP converter."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from convert_seer_smap import convert


class ConversionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/'input.smap'
        self.source.write_text(json.dumps({
            'header': {'mapType': '2D-Map', 'minPos': {'x': -1, 'y': -2},
                       'maxPos': {'x': 1, 'y': 2}, 'resolution': .5},
            'normalPosList': [{'x': -1, 'y': -2}, {'x': 1, 'y': 2}],
            'advancedLineList': [{'className': 'FeatureLine'}]}))
        self.prefix = self.root/'map'

    def test_coordinates_are_not_flipped_and_unknown_is_preserved(self):
        before = self.source.read_bytes()
        report = convert(self.source, self.prefix)
        with Image.open(self.root/'map.pgm') as im:
            self.assertEqual(im.getpixel((2, im.height-3)), 0)
            self.assertEqual(im.getpixel((6, 2)), 0)
            self.assertEqual(im.getpixel((4, 6)), 205)
        self.assertEqual(report['origin'], [-2., -3., 0.])
        self.assertEqual(report['occupied_cells'], 2)
        self.assertEqual(report['free_cells'], 0)
        self.assertEqual(self.source.read_bytes(), before)

    def test_explicit_polygon_marks_free_space_without_erasing_obstacles(self):
        polygon = self.root/'free.json'
        polygon.write_text('[[-1,-2],[1,-2],[1,2],[-1,2]]')
        report = convert(self.source, self.prefix, polygon)
        self.assertGreater(report['free_cells'], 0)
        self.assertEqual(report['occupied_cells'], 2)
        with Image.open(self.root/'map.pgm') as im:
            self.assertEqual(im.getpixel((4, 6)), 254)
            self.assertEqual(im.getpixel((0, 0)), 205)

    def test_confirmed_interior_and_feature_lines_preserve_padding(self):
        data = json.loads(self.source.read_text())
        data['advancedLineList'] = [
            {'className': 'FeatureLine', 'line': {'startPos': {'x': -1}, 'endPos': {'x': 1}}},
            {'className': 'OtherLine'}]
        self.source.write_text(json.dumps(data))
        report = convert(self.source, self.prefix, assume_free=True, feature_lines=True)
        self.assertEqual(report['feature_lines_drawn'], 1)
        self.assertGreater(report['free_cells'], 0)
        with Image.open(self.root/'map.pgm') as im:
            self.assertEqual(im.getpixel((4, 6)), 0)  # Wall at y=0.
            self.assertEqual(im.getpixel((4, 4)), 254)
            self.assertEqual(im.getpixel((0, 0)), 205)
            self.assertEqual(im.getpixel((2, im.height-3)), 0)

    def test_rejects_overwrite_and_out_of_bounds_points(self):
        convert(self.source, self.prefix)
        with self.assertRaisesRegex(ValueError, 'already exists'):
            convert(self.source, self.prefix)
        data = json.loads(self.source.read_text())
        data['normalPosList'][0]['x'] = 99
        self.source.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'outside'):
            convert(self.source, self.root/'bad')
        self.assertFalse((self.root/'bad.pgm').exists())


if __name__ == '__main__':
    unittest.main()
