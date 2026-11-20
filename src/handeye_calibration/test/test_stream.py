"""Distinguish stopped delivery from bad timestamps without relaxing capture checks."""

import json
import time

import pytest

pytest.importorskip('rclpy')

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from handeye_calibration.node import HandeyeNode


def test_timestamp_offset_does_not_trigger_reconnection_loop(tmp_path, monkeypatch):
    rclpy.init()
    node = HandeyeNode([
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('image_topic', value='/handeye_stream_test/image'),
        Parameter('camera_info_topic', value='/handeye_stream_test/info'),
    ])
    try:
        assert node.image_sub.sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert node.image_sub.sub.qos_profile.depth == 1
        image = Image()
        image.header.stamp = (node.get_clock().now() - Duration(seconds=10)).to_msg()
        node.receive_image(image)
        reconnects = []
        monkeypatch.setattr(node, 'count_publishers', lambda topic: 1 if topic.endswith('/image') else 0)
        monkeypatch.setattr(node, 'reconnect_stream', lambda: reconnects.append(True))
        node.check_stream()
        assert not reconnects, 'Fresh arrival with bad timestamp is a clock issue, not a lost reader'
        status = json.loads(node.status(Trigger.Request(), Trigger.Response()).message)
        assert status['image_count'] == 1
        assert status['image_stamp_age_s'] >= 10
        assert status['image_receive_gap_s'] < 1
        node._image_received_at = time.monotonic() - 3
        node.check_stream()
        assert len(reconnects) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
