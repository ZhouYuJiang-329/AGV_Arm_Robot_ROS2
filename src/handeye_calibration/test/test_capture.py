"""Exercise capture against a generated board and an in-memory ROS TF buffer."""

import numpy as np
import pytest

pytest.importorskip('rclpy')

import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger

from handeye_calibration.node import HandeyeNode


def test_capture_and_missing_input_handling(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output_directory:={tmp_path}'])
    node = None
    try:
        node = HandeyeNode()
        response = node.capture_training(Trigger.Request(), Trigger.Response())
        assert not response.success and 'No exact-time' in response.message
        response = node.solve(Trigger.Request(), Trigger.Response())
        assert not response.success and '15 training' in response.message

        pixels = np.full((480, 640, 3), 255, dtype=np.uint8)
        for row in range(9):
            for col in range(12):
                if (row + col) % 2 == 0:
                    pixels[60+row*40:100+row*40, 80+col*40:120+col*40] = 0
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'arm2_base'
        tf.child_frame_id = 'arm2_tool_link'
        tf.transform.rotation.w = 1.0
        node.buffer.set_transform(tf, 'synthetic_test')
        # Match this repository's horizontally flipped published image.
        image = node.bridge.cv2_to_imgmsg(pixels[:, ::-1].copy(), encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = 'camera_color_frame'
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = 640, 480
        info.distortion_model = 'plumb_bob'
        info.k = [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        node.receive(image, info)
        response = node.capture_training(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert (node.directory / 'session.json').exists()
        assert (node.directory / '000_training_raw.png').exists()
        assert len(node.session['samples'][0]['corners']) == 88
        assert node.session['samples'][0]['reprojection_error_px'] < 0.1
        response = node.capture_training(Trigger.Request(), Trigger.Response())
        assert not response.success and 'already captured' in response.message
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
