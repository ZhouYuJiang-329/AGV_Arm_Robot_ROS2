"""Check shared mounting geometry and mutually exclusive launch ownership."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
import xacro
import yaml

ROOT = Path(sys.argv.pop(1)).resolve()
spec = importlib.util.spec_from_file_location('description_launch', ROOT / 'launch/robot_description.launch.py')
LAUNCH = importlib.util.module_from_spec(spec)
spec.loader.exec_module(LAUNCH)


class MountTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.mounts = Path(self.directory.name) / 'mounts.yaml'
        # Deliberately synthetic geometry, not a proposed hardware calibration.
        self.front = dict(x_m=0.13, y_m=-0.21, z_relative_m=0.0,
                          roll_deg=180.0, pitch_deg=0.0, yaw_deg=-47.0)
        self.write_mounts()

    def write_mounts(self):
        self.mounts.write_text(yaml.safe_dump({'front': self.front, 'back': dict(
            x_m=-0.13, y_m=0.21, z_relative_m=-0.24,
            roll_deg=0.0, pitch_deg=0.0, yaw_deg=45.0)}))

    def expand(self, name, enabled, back=False):
        return ET.fromstring(xacro.process_file(str(ROOT / 'urdf' / name), mappings={
            'enable_front_lidar_tf': str(enabled).lower(),
            'enable_back_lidar_tf': str(back).lower(),
            'lidar_mounts_file': str(self.mounts),
        }).toxml())

    def test_shared_joint_and_disabled_default(self):
        full = self.expand('agv_robot.urdf.xacro', True)
        only = self.expand('lidar_only.urdf.xacro', True)
        for tree in (full, only):
            joints = tree.findall("joint[@name='laser_front_mount_joint']")
            self.assertEqual(len(joints), 1)
            joint = joints[0]
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'base_link')
            self.assertEqual(joint.find('child').get('link'), 'laser_front_link')
            self.assertEqual(joint.find('origin').get('xyz'), '0.13 -0.21 0.0')
            roll, pitch, yaw = map(float, joint.find('origin').get('rpy').split())
            self.assertAlmostEqual(roll, math.pi)
            self.assertAlmostEqual(pitch, 0)
            self.assertAlmostEqual(yaw, math.radians(-47))
            names = [link.get('name') for link in tree.findall('link')]
            self.assertEqual(len(names), len(set(names)))
            self.assertNotIn('laser_back_link', names)
        disabled = self.expand('agv_robot.urdf.xacro', False)
        self.assertIsNone(disabled.find("link[@name='laser_front_link']"))

    def launch(self, model, owner, back='off'):
        context = LaunchContext()
        context.launch_configurations.update(dict(LAUNCH.HARDWARE_ARGS))
        context.launch_configurations.update(model=model, front_lidar_tf_owner=owner,
                                             back_lidar_tf_owner=back,
                                             lidar_mounts_file=str(self.mounts), use_sim_time='false')
        with patch.object(LAUNCH, 'Node') as node:
            LAUNCH._setup(context)
        return node.call_args_list

    def test_unique_front_tf_owner_in_every_mode(self):
        for model, owner, expected_nodes, expected_front in (
                ('full', 'off', 1, 0), ('full', 'description', 1, 1),
                ('full', 'standalone', 2, 1), ('front_only', 'standalone', 1, 1)):
            calls = self.launch(model, owner)
            self.assertEqual(len(calls), expected_nodes)
            descriptions = [ET.fromstring(call.kwargs['parameters'][0]['robot_description'])
                            for call in calls]
            self.assertEqual(sum(len(tree.findall("joint[@name='laser_front_mount_joint']"))
                                 for tree in descriptions), expected_front)
        with self.assertRaisesRegex(RuntimeError, 'requires'):
            self.launch('front_only', 'description')

    def test_back_geometry_and_independent_ownership(self):
        for name in ('agv_robot.urdf.xacro', 'lidar_only.urdf.xacro'):
            tree = self.expand(name, False, back=True)
            joint = tree.find("joint[@name='laser_back_mount_joint']")
            self.assertIsNotNone(joint)
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'base_link')
            self.assertEqual(joint.find('child').get('link'), 'laser_back_link')
            self.assertEqual(joint.find('origin').get('xyz'), '-0.13 0.21 -0.24')
            self.assertAlmostEqual(float(joint.find('origin').get('rpy').split()[2]), math.pi / 4)
            self.assertIsNone(tree.find("link[@name='laser_front_link']"))
        modes = ('off', 'description', 'standalone')
        for front in modes:
            for back in modes:
                calls = self.launch('full', front, back)
                descriptions = [ET.fromstring(call.kwargs['parameters'][0]['robot_description'])
                                for call in calls]
                self.assertEqual(len(calls), 1 + int('standalone' in (front, back)))
                for sensor, owner in (('front', front), ('back', back)):
                    self.assertEqual(sum(len(tree.findall(f"joint[@name='laser_{sensor}_mount_joint']"))
                                         for tree in descriptions), int(owner != 'off'))
        with self.assertRaisesRegex(RuntimeError, 'requires model=full'):
            self.launch('front_only', 'standalone', 'description')
        config = yaml.safe_load(self.mounts.read_text())
        config['back']['yaw_deg'] = None
        self.mounts.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(RuntimeError, 'back.yaw_deg'):
            self.launch('full', 'description', 'description')
        self.assertEqual(len(self.launch('full', 'description', 'off')), 1)

    def test_unconfirmed_and_invalid_values_rejected(self):
        for bad in (None, True, float('nan'), float('inf'), '0.55'):
            self.front['x_m'] = bad
            self.write_mounts()
            with self.assertRaisesRegex(RuntimeError, 'front.x_m'):
                self.launch('front_only', 'standalone')
        # Existing full bringup works with TF off, even without a config file.
        self.mounts.unlink()
        self.assertEqual(len(self.launch('full', 'off')), 1)


if __name__ == '__main__':
    unittest.main()
