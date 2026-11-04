"""Fast regressions for the shared world/map and simulator navigation contract."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(sys.argv.pop(1)).resolve()
sys.path.insert(0, str(PACKAGE))
from agv_bringup_sim.assets import generate_assets
from agv_bringup_sim.initial_pose import parse_initial_pose


class InitialPoseTest(unittest.TestCase):
    def test_empty_keeps_existing_initialization_policy(self):
        self.assertIsNone(parse_initial_pose('  '))

    def test_valid_pose_retains_map_coordinates_and_radians(self):
        self.assertEqual(parse_initial_pose('[-4, -4.25, 1.5708]'), (-4.0, -4.25, 1.5708))

    def test_invalid_pose_is_rejected(self):
        for value in ('null', '{}', '[1, 2]', '[1, 2, 3, 4]', '[true, 0, 0]',
                      '["1", 2, 3]', '[NaN, 0, 0]', '[1e999, 0, 0]', 'bad'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'initial_pose'):
                parse_initial_pose(value)


class SimulationAssetsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.assets = generate_assets(PACKAGE/'config/agv_navigation_scene.yaml',
                                     PACKAGE.parent/'agv_description/meshes/base_link.STL',
                                     PACKAGE/'config/agv_nav2_sim.yaml', self.directory.name)
        self.scene = self.assets['scene']

    def test_robot_and_tf_have_no_real_hardware_and_agree_on_lidar_pose(self):
        sdf = ET.parse(self.assets['robot']).getroot()
        urdf = ET.fromstring(self.assets['description'])
        self.assertFalse(urdf.findall('ros2_control'))
        self.assertNotIn('agv_hardware', Path(self.assets['robot']).read_text())
        self.assertEqual(urdf.find('joint').find('origin').get('xyz').split()[-1],
                         sdf.find('.//sensor/pose').text.split()[2])
        names = [p.get('name') for p in sdf.findall('.//plugin')]
        self.assertIn('gz::sim::systems::VelocityControl', names)
        self.assertIn('gz::sim::systems::OdometryPublisher', names)

    def test_world_and_map_match_but_leave_the_sensor_only_obstacle_unmapped(self):
        world = ET.parse(self.assets['world']).getroot()
        names = {m.get('name') for m in world.findall('.//model')}
        pgm = (Path(self.directory.name)/'map.pgm').read_bytes()
        _, dimensions, _, pixels = pgm.split(b'\n', 3)
        width, height = map(int, dimensions.split())
        xmin, _, _, ymax = self.scene['bounds']
        res = self.scene['resolution']
        def pixel(x, y):
            column = int((x-xmin)/res)
            row = int((ymax-y)/res)
            return pixels[row*width+column]
        self.assertEqual(len(pixels), width*height)
        for obstacle in self.scene['obstacles']:
            self.assertIn(obstacle['name'], names)
            self.assertEqual(pixel(*obstacle['center']), 0)
        for obstacle in self.scene['unmapped_obstacles']:
            self.assertIn(obstacle['name'], names)
            self.assertEqual(pixel(*obstacle['center']), 254)
        self.assertEqual(pixel(*self.scene['start'][:2]), 254)
        self.assertEqual(pixel(3, 3), 254)

    def test_height_footprint_and_velocity_configuration_match_the_model(self):
        params = yaml.safe_load(Path(self.assets['params']).read_text())
        for name in ('local_costmap', 'global_costmap'):
            config = params[name][name]['ros__parameters']
            scan = config['obstacle_layer']['scan']
            self.assertGreater(scan['max_obstacle_height'], self.scene['body']['lidar_z'])
            footprint = yaml.safe_load(config['footprint'])
            self.assertGreater(max(p[0] for p in footprint), self.scene['body']['size'][0]/2)
            self.assertGreater(max(p[1] for p in footprint), self.scene['body']['size'][1]/2)
        control = params['controller_server']['ros__parameters']
        self.assertEqual(control['FollowPath']['xy_goal_tolerance'], control['goal_checker']['xy_goal_tolerance'])
        self.assertLess(control['FollowPath']['trans_stopped_velocity'], .05)
        self.assertLess(control['min_y_velocity_threshold'], .01)
        self.assertLess(control['FollowPath']['min_vel_y'], 0)
        self.assertGreater(control['FollowPath']['max_vel_y'], 0)
        self.assertEqual(params['amcl']['ros__parameters']['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        self.assertGreater(params['velocity_smoother']['ros__parameters']['max_velocity'][1], 0)

    def test_slam_frames_and_range_match_the_simulated_sensor(self):
        slam = yaml.safe_load((PACKAGE/'config/agv_slam_sim.yaml').read_text())['slam_toolbox']['ros__parameters']
        sensor = ET.parse(self.assets['robot']).find('.//sensor')
        urdf = ET.fromstring(self.assets['description'])
        frames = {link.get('name') for link in urdf.findall('link')}
        self.assertIn(slam['base_frame'], frames)
        nav = yaml.safe_load(Path(self.assets['params']).read_text())['amcl']['ros__parameters']
        self.assertEqual(slam['base_frame'], nav['base_frame_id'])
        self.assertEqual(slam['map_frame'], nav['global_frame_id'])
        self.assertEqual(slam['odom_frame'], nav['odom_frame_id'])
        self.assertEqual(slam['scan_topic'], '/scan')
        self.assertEqual(slam['min_laser_range'], float(sensor.find('lidar/range/min').text))
        self.assertEqual(slam['max_laser_range'], float(sensor.find('lidar/range/max').text))
        self.assertTrue(slam['use_sim_time'])


if __name__ == '__main__':
    unittest.main()
