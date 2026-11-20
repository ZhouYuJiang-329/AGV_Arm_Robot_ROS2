"""Headless Qt/ROS integration checks for the actual worker and window."""

import json
import os
import time

import cv2
import numpy as np
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')

from PyQt5.QtCore import QTimer
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QFileDialog
import rclpy

from handeye_calibration.gui import CalibrationWindow
from handeye_calibration.gui_worker import CalibrationWorker


@pytest.fixture(scope='module')
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


def wait_until(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        QTest.qWait(20)
    assert predicate(), 'Timed out waiting for GUI worker'


def make_session(directory):
    rng = np.random.default_rng(8)
    camera = np.eye(4)
    camera[:3, :3] = cv2.Rodrigues(np.array([0.2, -0.6, 1.1]))[0]
    camera[:3, 3] = [0.02, 0.22, 0.14]
    board = np.eye(4)
    board[:3, 3] = [0.5, 0.1, 0.3]
    session = {
        'base_frame': 'arm2_base', 'tool_frame': 'arm2_tool_link',
        'camera_frame': 'camera_color_frame', 'board_columns': 11, 'board_rows': 8,
        'square_size_m': 0.02, 'samples': [],
    }
    for index in range(25):
        tool = np.eye(4)
        tool[:3, :3] = cv2.Rodrigues(rng.uniform(-0.8, 0.8, 3))[0]
        tool[:3, 3] = rng.uniform(-0.2, 0.2, 3)
        observed = np.linalg.inv(camera) @ np.linalg.inv(tool) @ board
        session['samples'].append({
            'split': 'training' if index < 20 else 'validation',
            'T_base_tool': tool.tolist(), 'T_camera_board': observed.tolist(),
            'stamp_ns': index + 1, 'reprojection_error_px': 0.15,
            'image_prefix': f'{index:03d}',
        })
    path = directory / 'session.json'
    path.write_text(json.dumps(session), encoding='utf-8')
    assert cv2.imwrite(str(directory / '000_corners.png'),
                       np.full((200, 320, 3), 140, np.uint8))
    return path


def test_offline_solve_exclude_export_and_shutdown(app, tmp_path, monkeypatch):
    path = make_session(tmp_path)
    window = CalibrationWindow()
    window.show()
    try:
        window.launch_worker(CalibrationWorker(session_path=path, parent=window))
        wait_until(lambda: window._ready)
        assert window.table.rowCount() == 25
        assert window.training_count.text() == '20 / 20'
        assert not window.capture_button.isEnabled()
        assert not window.settings.isEnabled()
        assert window.solve_button.isEnabled()
        window.solve_button.click()
        wait_until(lambda: not window._busy)
        assert window.candidate is not None
        assert window.export_button.isEnabled()
        assert '独立验证 · 5 组' in window.result_label.text()
        exported = tmp_path / 'export.yaml'
        monkeypatch.setattr(QFileDialog, 'getSaveFileName', lambda *a, **k: (str(exported), ''))
        window.export_button.click()
        assert exported.read_text(encoding='utf-8') == window.candidate

        window.table.selectRow(0)
        assert window._sample_view
        assert not window.image.source.isNull()
        window.toggle_button.click()
        wait_until(lambda: not window._busy)
        assert window.training_count.text() == '19 / 20'
        assert not window.export_button.isEnabled()
        assert window.candidate is None
        assert json.loads(path.read_text())['samples'][0]['excluded']
        window.solve_button.click()
        wait_until(lambda: not window._busy)
        assert window.snapshot_data['report']['training']['count'] == 19
        window.close()
        wait_until(lambda: window.worker is None)
        assert not window.isVisible()
    finally:
        if window.worker is not None:
            window.stop_session()
            wait_until(lambda: window.worker is None)
        window.close()


def test_live_worker_start_missing_camera_and_clean_stop(app, tmp_path):
    rclpy.init()
    window = CalibrationWindow()
    window.output.setText(str(tmp_path))
    window.fields['image_topic'].setText('/handeye_missing_camera_test/image')
    window.fields['camera_info_topic'].setText('/handeye_missing_camera_test/info')
    try:
        assert window.config()['square_size_m'] == 0.02
        window.start_button.click()
        wait_until(lambda: window._ready)
        wait_until(lambda: 'message' in window._health)
        assert not window.capture_button.isEnabled()
        assert not window._health['image']
        # Dispatch through the real worker even though the button is disabled.
        window.command('capture')
        wait_until(lambda: not window._busy)
        assert 'No exact-time' in window.log.toPlainText()
        window.stop_button.click()
        wait_until(lambda: window.worker is None)
        assert window.start_button.isEnabled()
    finally:
        if window.worker is not None:
            window.stop_session()
            wait_until(lambda: window.worker is None)
        window.close()
        rclpy.shutdown()


def test_bad_history_reports_error_and_restores_controls(app, tmp_path):
    invalid = tmp_path / 'session.json'
    invalid.write_text('{}', encoding='utf-8')
    window = CalibrationWindow()
    try:
        window.launch_worker(CalibrationWorker(session_path=invalid, parent=window))
        wait_until(lambda: window.worker is None)
        assert '会话缺少字段' in window.log.toPlainText()
        assert window.start_button.isEnabled()
    finally:
        window.close()


def test_stopped_video_does_not_keep_showing_old_fps(app, monkeypatch):
    from types import SimpleNamespace
    import handeye_calibration.gui as gui

    window = CalibrationWindow()
    try:
        window._render_times.extend([0.0, 0.033, 0.066])
        window._display_age = 0.02
        window._displayed_at = 0.066
        monkeypatch.setattr(gui, 'time', SimpleNamespace(monotonic=lambda: 5.066))
        window.update_performance()
        assert '0.0 FPS' in window.performance_label.text()
        assert '5020 ms' in window.performance_label.text()
    finally:
        window.close()


def test_live_ros_images_preview_and_capture(app, tmp_path):
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import StaticTransformBroadcaster
    from cv_bridge import CvBridge

    rclpy.init()
    publisher = rclpy.create_node('handeye_gui_test_camera')
    image_pub = publisher.create_publisher(Image, '/handeye_gui_test/image',
                                           QoSProfile(depth=5))
    info_pub = publisher.create_publisher(CameraInfo, '/handeye_gui_test/info',
                                          QoSProfile(depth=5))
    broadcaster = StaticTransformBroadcaster(publisher)
    tf = TransformStamped()
    tf.header.stamp = publisher.get_clock().now().to_msg()
    tf.header.frame_id = 'handeye_gui_test_base'
    tf.child_frame_id = 'handeye_gui_test_tool'
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    pixels = np.full((480, 640, 3), 255, np.uint8)
    for row in range(9):
        for col in range(12):
            if (row + col) % 2 == 0:
                pixels[60+row*40:100+row*40, 80+col*40:120+col*40] = 0
    image = CvBridge().cv2_to_imgmsg(pixels[:, ::-1].copy(), encoding='bgr8')
    image.header.frame_id = 'handeye_gui_test_camera_frame'
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.distortion_model = 'plumb_bob'
    info.k = [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    window = CalibrationWindow()
    try:
        window.output.setText(str(tmp_path))
        for key, value in [('image_topic', '/handeye_gui_test/image'),
                           ('camera_info_topic', '/handeye_gui_test/info'),
                           ('base_frame', 'handeye_gui_test_base'),
                           ('tool_frame', 'handeye_gui_test_tool'),
                           ('camera_frame', 'handeye_gui_test_camera_frame')]:
            window.fields[key].setText(value)
        window.start_button.click()
        wait_until(lambda: window._ready)
        deadline = time.monotonic() + 10
        while not window.capture_button.isEnabled() and time.monotonic() < deadline:
            image.header.stamp = publisher.get_clock().now().to_msg()
            info.header = image.header
            image_pub.publish(image)
            info_pub.publish(info)
            QTest.qWait(100)
        assert window.capture_button.isEnabled(), window._health
        wait_until(lambda: window._health.get('board'))
        assert not window.image.source.isNull()
        window.capture_button.click()
        wait_until(lambda: not window._busy)
        assert window.table.rowCount() == 1, window.log.toPlainText()
        assert window.training_count.text() == '1 / 20'
        directory = window.snapshot_data['directory']
        assert (tmp_path / directory / 'session.json').exists()
        window.close()
        wait_until(lambda: window.worker is None)
    finally:
        if window.worker is not None:
            window.stop_session()
            wait_until(lambda: window.worker is None)
        window.close()
        publisher.destroy_node()
        rclpy.shutdown()


def test_live_preview_keeps_up_when_corner_detection_is_slow(app, tmp_path, monkeypatch):
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import CameraInfo, Image

    def slow_detection(*args, **kwargs):
        time.sleep(0.3)
        return False, None

    monkeypatch.setattr(cv2, 'findChessboardCorners', slow_detection)
    rclpy.init()
    publisher = rclpy.create_node('handeye_preview_rate_test')
    image_pub = publisher.create_publisher(Image, '/handeye_rate_test/image',
                                           QoSProfile(depth=5))
    info_pub = publisher.create_publisher(CameraInfo, '/handeye_rate_test/info',
                                          QoSProfile(depth=5))
    image = CvBridge().cv2_to_imgmsg(np.full((240, 320, 3), 128, np.uint8), 'bgr8')
    image.header.frame_id = 'handeye_rate_camera'
    info = CameraInfo()
    info.width, info.height = 320, 240
    info.distortion_model = 'plumb_bob'
    info.k = [400.0, 0.0, 160.0, 0.0, 400.0, 120.0, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    window = CalibrationWindow()
    window.output.setText(str(tmp_path))
    window.fields['image_topic'].setText('/handeye_rate_test/image')
    window.fields['camera_info_topic'].setText('/handeye_rate_test/info')
    window.fields['camera_frame'].setText('handeye_rate_camera')
    rendered = []
    original = window.image.set_rgb

    def render(pixels):
        rendered.append(time.monotonic())
        original(pixels)

    window.image.set_rgb = render
    timer = QTimer()

    def publish():
        image.header.stamp = publisher.get_clock().now().to_msg()
        info.header = image.header
        image_pub.publish(image)
        info_pub.publish(info)

    timer.timeout.connect(publish)
    try:
        window.show()
        window.start_session()
        timer.start(33)
        wait_until(lambda: len(rendered) >= 2)
        rendered.clear()
        start = time.monotonic()
        QTest.qWait(1800)
        rate = len(rendered) / (time.monotonic() - start)
        assert rate >= 12, f'Preview stalled behind detector: {rate:.1f} FPS'
        assert window._health.get('age', 100) < 0.5

        # Keep publishing while breaking only this reader, as seen on hardware.
        node = window.worker.node
        old_subscription = node.image_sub.sub
        received_stamp = node.latest_image.header.stamp
        node.destroy_subscription(old_subscription)
        recovery_deadline = time.monotonic() + 7
        while time.monotonic() < recovery_deadline:
            QTest.qWait(50)
            latest = node.latest_image
            if (node.image_sub.sub is not old_subscription and latest is not None
                    and latest.header.stamp != received_stamp and window._health.get('image')):
                break
        assert node.image_sub.sub is not old_subscription, 'Stopped reader was never recreated'
        assert window._health.get('image'), 'Live preview did not recover'
        assert not window.capture_button.isEnabled(), 'No robot TF: recovery must not enable capture'
    finally:
        timer.stop()
        window.close()
        wait_until(lambda: window.worker is None)
        publisher.destroy_node()
        rclpy.shutdown()
