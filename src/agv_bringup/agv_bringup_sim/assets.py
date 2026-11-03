"""Generate matching Gazebo geometry, occupancy map, TF model and Nav2 parameters."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml


def _element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def scene_rectangles(scene):
    """Return physical rectangles; only mapped entries go into the occupancy map."""
    xmin, xmax, ymin, ymax = scene['bounds']
    t = scene['wall_thickness']
    walls = [
        {'name': 'west_wall', 'center': [xmin, (ymin+ymax)/2], 'size': [t, ymax-ymin+t]},
        {'name': 'east_wall', 'center': [xmax, (ymin+ymax)/2], 'size': [t, ymax-ymin+t]},
        {'name': 'south_wall', 'center': [(xmin+xmax)/2, ymin], 'size': [xmax-xmin+t, t]},
        {'name': 'north_wall', 'center': [(xmin+xmax)/2, ymax], 'size': [xmax-xmin+t, t]},
    ]
    return walls + scene['obstacles'], scene.get('unmapped_obstacles', [])


def _box_model(world, rectangle):
    model = _element(world, 'model', name=rectangle['name'])
    _element(model, 'static', 'true')
    x, y = rectangle['center']
    _element(model, 'pose', f'{x} {y} 0.9 0 0 0')
    link = _element(model, 'link', name='link')
    sx, sy = rectangle['size']
    for tag in ('collision', 'visual'):
        node = _element(link, tag, name=tag)
        _element(_element(_element(node, 'geometry'), 'box'), 'size', f'{sx} {sy} 1.8')
        if tag == 'visual':
            _element(_element(node, 'material'), 'diffuse', '0.5 0.6 0.7 1')


def generate_assets(scene_file, mesh_file, params_template, output_directory):
    """Create run-local files without changing the CAD model or real hardware setup."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    scene = yaml.safe_load(Path(scene_file).read_text())
    mesh = Path(mesh_file).resolve()
    if not mesh.is_file():
        raise ValueError(f'AGV body mesh missing: {mesh}')
    mapped, unmapped = scene_rectangles(scene)
    world_sdf = ET.Element('sdf', version='1.9')
    world = _element(world_sdf, 'world', name=scene['world_name'])
    physics = _element(world, 'physics', name='physics', type='ignored')
    _element(physics, 'max_step_size', '0.002')
    _element(physics, 'real_time_factor', '1.0')
    for filename, name in (
        ('physics', 'Physics'), ('user-commands', 'UserCommands'),
        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')):
        plugin = _element(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            _element(plugin, 'render_engine', 'ogre2')
    lighting = _element(world, 'scene')
    _element(lighting, 'ambient', '0.4 0.4 0.4 1')
    _element(lighting, 'background', '0.75 0.75 0.75 1')
    light = _element(world, 'light', name='sun', type='directional')
    _element(light, 'pose', '0 0 10 0 0 0')
    _element(light, 'diffuse', '0.9 0.9 0.9 1')
    _element(light, 'direction', '-0.5 0.1 -0.9')
    floor = _element(world, 'model', name='ground_plane')
    _element(floor, 'static', 'true')
    floor_link = _element(floor, 'link', name='link')
    for tag in ('collision', 'visual'):
        node = _element(floor_link, tag, name=tag)
        plane = _element(_element(node, 'geometry'), 'plane')
        _element(plane, 'normal', '0 0 1')
        _element(plane, 'size', '30 30')
        if tag == 'visual':
            material = _element(node, 'material')
            _element(material, 'ambient', '0.7 0.7 0.7 1')
            _element(material, 'diffuse', '0.7 0.7 0.7 1')
    for rectangle in mapped + unmapped:
        _box_model(world, rectangle)
    ET.indent(world_sdf)
    ET.ElementTree(world_sdf).write(output / 'world.sdf', encoding='unicode', xml_declaration=True)

    sdf = ET.Element('sdf', version='1.9')
    model = _element(sdf, 'model', name='agv_sim')
    _element(model, 'static', 'false')
    link = _element(model, 'link', name='base_link')
    _element(link, 'gravity', 'false')
    inertia = _element(link, 'inertial')
    _element(inertia, 'mass', '40')
    tensor = _element(inertia, 'inertia')
    for key, value in {'ixx': 8, 'iyy': 4, 'izz': 8}.items():
        _element(tensor, key, value)
    sx, sy, sz = scene['body']['size']
    collision = _element(link, 'collision', name='body_collision')
    _element(collision, 'pose', f'0 0 {sz/2+0.02} 0 0 0')
    _element(_element(_element(collision, 'geometry'), 'box'), 'size', f'{sx} {sy} {sz}')
    visual = _element(link, 'visual', name='cad_body')
    _element(visual, 'pose', f"0 0 {scene['body']['mesh_z']} 0 0 0")
    _element(_element(_element(visual, 'geometry'), 'mesh'), 'uri', mesh.as_uri())
    material = _element(visual, 'material')
    _element(material, 'ambient', '0.1 0.3 0.6 1')
    _element(material, 'diffuse', '0.1 0.4 0.8 1')
    sensor = _element(link, 'sensor', name='lidar', type='gpu_lidar')
    _element(sensor, 'pose', f"0 0 {scene['body']['lidar_z']} 0 0 0")
    _element(sensor, 'topic', '/sim/scan')
    _element(sensor, 'update_rate', '10')
    _element(sensor, 'always_on', 'true')
    lidar = _element(sensor, 'lidar')
    horizontal = _element(_element(lidar, 'scan'), 'horizontal')
    for key, value in {'samples': 720, 'resolution': 1, 'min_angle': -math.pi, 'max_angle': math.pi}.items():
        _element(horizontal, key, value)
    span = _element(lidar, 'range')
    for key, value in {'min': .08, 'max': 12.0, 'resolution': .01}.items():
        _element(span, key, value)
    velocity = _element(model, 'plugin', filename='gz-sim-velocity-control-system', name='gz::sim::systems::VelocityControl')
    _element(velocity, 'topic', '/model/agv_sim/cmd_vel')
    odom = _element(model, 'plugin', filename='gz-sim-odometry-publisher-system', name='gz::sim::systems::OdometryPublisher')
    for key, value in {'odom_frame': 'odom', 'robot_base_frame': 'base_link', 'odom_topic': '/sim/odom',
                       'tf_topic': '/sim/odom_tf', 'odom_publish_frequency': 50, 'dimensions': 2}.items():
        _element(odom, key, value)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(output / 'robot.sdf', encoding='unicode', xml_declaration=True)

    urdf = ET.Element('robot', name='agv_sim')
    base = _element(urdf, 'link', name='base_link')
    visual = _element(base, 'visual')
    _element(visual, 'origin', xyz=f"0 0 {scene['body']['mesh_z']}")
    _element(_element(visual, 'geometry'), 'mesh', filename=mesh.as_uri())
    _element(_element(visual, 'material', name='agv_blue'), 'color', rgba='0.1 0.4 0.8 1')
    _element(urdf, 'link', name='laser_link')
    joint = _element(urdf, 'joint', name='laser_mount', type='fixed')
    _element(joint, 'parent', link='base_link')
    _element(joint, 'child', link='laser_link')
    _element(joint, 'origin', xyz=f"0 0 {scene['body']['lidar_z']}")
    description = ET.tostring(urdf, encoding='unicode')
    (output / 'robot.urdf').write_text(description)

    # The PGM uses top-down rows; ROS map coordinates use the lower-left origin.
    xmin, xmax, ymin, ymax = scene['bounds']
    res = scene['resolution']
    width, height = round((xmax-xmin)/res), round((ymax-ymin)/res)
    pixels = bytearray([254] * (width*height))
    for row in range(height):
        y = ymax - (row+.5)*res
        for column in range(width):
            x = xmin + (column+.5)*res
            if any(abs(x-r['center'][0]) <= r['size'][0]/2 and abs(y-r['center'][1]) <= r['size'][1]/2 for r in mapped):
                pixels[row*width+column] = 0
    (output / 'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    map_config = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': res, 'origin': [xmin, ymin, 0.0],
                  'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}
    (output / 'map.yaml').write_text(yaml.safe_dump(map_config))
    params = yaml.safe_load(Path(params_template).read_text())
    footprint = str([[-sx/2-.05, -sy/2-.05], [sx/2+.05, -sy/2-.05],
                     [sx/2+.05, sy/2+.05], [-sx/2-.05, sy/2+.05]])
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters']['footprint'] = footprint
    initial = params['amcl']['ros__parameters']['initial_pose']
    initial.update(dict(zip(('x', 'y', 'yaw'), scene['start'])))
    (output / 'nav2.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    return {'world': str(output/'world.sdf'), 'robot': str(output/'robot.sdf'),
            'map': str(output/'map.yaml'), 'params': str(output/'nav2.yaml'),
            'description': description, 'scene': scene, 'directory': str(output)}
