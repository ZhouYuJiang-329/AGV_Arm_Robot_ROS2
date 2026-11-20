"""Record stationary chessboard/TF pairs on demand, without commanding motion."""

from datetime import datetime
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from handeye_calibration.solver import quaternion_matrix, save_result


class HandeyeNode(Node):
    """Use exact image/info stamps and TF at exposure publication time."""

    def __init__(self, parameter_overrides=None, node_name='handeye_calibration'):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        defaults = {
            'base_frame': 'arm2_base',
            'tool_frame': 'arm2_tool_link',
            'camera_frame': 'camera_color_frame',
            'image_topic': '/color/image_raw',
            'camera_info_topic': '/color/camera_info',
            'board_columns': 11,
            'board_rows': 8,
            'square_size_m': 0.020,
            # Undo cv::flip(..., 1) in this repository's Astra S publisher.
            'undo_horizontal_flip': True,
            'max_reprojection_error_px': 1.0,
            'max_image_age_s': 2.0,
            'reliable_transport': True,
            'output_directory': str(Path.home() / 'handeye_sessions'),
        }
        self.cfg = {name: self.declare_parameter(name, value).value
                    for name, value in defaults.items()}
        cols, rows = self.cfg['board_columns'], self.cfg['board_rows']
        if cols < 3 or rows < 3 or self.cfg['square_size_m'] <= 0:
            raise ValueError('Invalid board dimensions or square size')
        if (self.cfg['max_reprojection_error_px'] <= 0
                or self.cfg['max_image_age_s'] <= 0):
            raise ValueError('Error and age limits must be positive')
        self.object_points = np.zeros((rows * cols, 3), np.float64)
        self.object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        self.object_points *= self.cfg['square_size_m']
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        self.latest = None
        self.latest_image = None
        self.latest_info = None
        self._stream_started_at = time.monotonic()
        self._image_received_at = None
        self._info_received_at = None
        self._image_count = 0
        self._info_count = 0
        self._pair_count = 0
        self._reconnect_count = 0
        self._last_reconnect = 0.0
        self.connect_stream()
        self.stream_watchdog = self.create_timer(0.5, self.check_stream)
        self.directory = Path(self.cfg['output_directory']).expanduser() / datetime.now().strftime(
            '%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session = {
            **self.cfg, 'units': 'metres', 'samples': [],
            'transform_convention': 'T_parent_child maps child coordinates to parent',
        }
        self.service_handles = [
            self.create_service(Trigger, '~/capture', self.capture_training),
            self.create_service(Trigger, '~/capture_validation', self.capture_validation),
            self.create_service(Trigger, '~/solve', self.solve),
            self.create_service(Trigger, '~/status', self.status),
        ]
        self.get_logger().info(
            f'Board: {cols} columns x {rows} rows of INNER corners, '
            f'{self.cfg["square_size_m"] * 1000:g} mm squares. '
            f'Stop arm before capture. Output: {self.directory}')

    def connect_stream(self):
        """Create fresh DDS readers and a fresh exact-time synchronizer."""
        qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE if self.cfg['reliable_transport']
                         else ReliabilityPolicy.BEST_EFFORT))
        self.image_sub = message_filters.Subscriber(
            self, Image, self.cfg['image_topic'], qos_profile=qos)
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, self.cfg['camera_info_topic'],
            qos_profile=qos)
        self.sync = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.sync.registerCallback(self.receive)
        # Preview does not need to wait for an exactly matching CameraInfo.
        self.image_sub.registerCallback(self.receive_image)
        self.info_sub.registerCallback(self.receive_info)

    def reconnect_stream(self):
        """Recover readers without discarding calibration samples or moving hardware."""
        self.destroy_subscription(self.image_sub.sub)
        self.destroy_subscription(self.info_sub.sub)
        self.latest = self.latest_image = self.latest_info = None
        self.connect_stream()
        self._reconnect_count += 1
        self._last_reconnect = time.monotonic()
        self.get_logger().warning(
            f'图像数据流接收中断，已重新订阅（第 {self._reconnect_count} 次）；采样记录保留。')

    def check_stream(self):
        """Use monotonic arrival times: clock offsets must not cause retry loops."""
        now = time.monotonic()
        timeout = self.cfg['max_image_age_s']
        image_gap = now - (self._image_received_at or self._stream_started_at)
        info_gap = now - (self._info_received_at or self._stream_started_at)
        image_stalled = image_gap > timeout and self.count_publishers(self.cfg['image_topic']) > 0
        info_stalled = info_gap > timeout and self.count_publishers(self.cfg['camera_info_topic']) > 0
        if (image_stalled or info_stalled) and now - self._last_reconnect >= 5.0:
            self.reconnect_stream()

    def stream_status(self):
        now = time.monotonic()
        image = self.latest_image
        age = ((self.get_clock().now() - Time.from_msg(image.header.stamp)).nanoseconds / 1e9
               if image is not None else None)
        return {
            'image_count': self._image_count,
            'info_count': self._info_count,
            'pair_count': self._pair_count,
            'image_receive_gap_s': now - (self._image_received_at or self._stream_started_at),
            'info_receive_gap_s': now - (self._info_received_at or self._stream_started_at),
            'image_stamp_age_s': age,
            'reconnect_count': self._reconnect_count,
        }

    def status(self, request, response):
        response.success = True
        response.message = json.dumps(self.stream_status())
        return response

    def receive(self, image, info):
        self.latest = (image, info)
        self._pair_count += 1

    def receive_image(self, image):
        self.latest_image = image
        self._image_received_at = time.monotonic()
        self._image_count += 1

    def receive_info(self, info):
        self.latest_info = info
        self._info_received_at = time.monotonic()
        self._info_count += 1

    def capture_training(self, request, response):
        return self.capture(response, 'training')

    def capture_validation(self, request, response):
        return self.capture(response, 'validation')

    def capture(self, response, split):
        try:
            if self.latest is None:
                raise ValueError('No exact-time Image/CameraInfo pair received')
            image, info = self.latest
            stamp = Time.from_msg(image.header.stamp)
            if stamp.nanoseconds == 0:
                raise ValueError('Image timestamp must not be zero')
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0 <= age <= self.cfg['max_image_age_s']:
                raise ValueError(f'Image is stale or in future: age={age:.3f}s')
            if image.header.frame_id != self.cfg['camera_frame']:
                raise ValueError('Image frame differs from configured camera_frame')
            if info.header.frame_id != image.header.frame_id:
                raise ValueError('Image and CameraInfo frames differ')
            if (info.width, info.height) != (image.width, image.height):
                raise ValueError('CameraInfo resolution differs from image')
            if info.distortion_model != 'plumb_bob':
                raise ValueError('Expected plumb_bob camera distortion model')
            k = np.asarray(info.k).reshape(3, 3)
            d = np.asarray(info.d)
            if (not np.isfinite(k).all() or not np.isfinite(d).all()
                    or k[0, 0] <= 0 or k[1, 1] <= 0 or d.size not in (4, 5)):
                raise ValueError('Missing or invalid camera intrinsics')
            if any(s['stamp_ns'] == stamp.nanoseconds for s in self.session['samples']):
                raise ValueError('This image was already captured')
            # Do not fall back to latest TF: a mismatched pose corrupts the solve.
            transform = self.buffer.lookup_transform(
                self.cfg['base_frame'], self.cfg['tool_frame'], stamp)
            pose = np.eye(4)
            q = transform.transform.rotation
            t = transform.transform.translation
            pose[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
            pose[:3, 3] = [t.x, t.y, t.z]
            if self.session['samples']:
                previous = np.asarray(self.session['samples'][-1]['T_base_tool'])
                angle = np.linalg.norm(cv2.Rodrigues(previous[:3, :3].T @ pose[:3, :3])[0])
                distance = np.linalg.norm(previous[:3, 3] - pose[:3, 3])
                if angle < np.deg2rad(3) and distance < 0.005:
                    raise ValueError('Pose too close to last sample; change position/orientation')
            raw = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            restored = cv2.flip(raw, 1) if self.cfg['undo_horizontal_flip'] else raw
            gray = cv2.cvtColor(restored, cv2.COLOR_BGR2GRAY)
            size = (self.cfg['board_columns'], self.cfg['board_rows'])
            found, corners = cv2.findChessboardCornersSB(
                gray, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
            if not found:
                raise ValueError(f'Could not detect full {size} inner-corner chessboard')
            ok, rvec, tvec = cv2.solvePnP(self.object_points, corners, k, d)
            if not ok:
                raise ValueError('solvePnP failed')
            board = np.eye(4)
            board[:3, :3], board[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
            depths = (board[:3, :3] @ self.object_points.T + tvec)[2]
            if np.any(depths <= 0):
                raise ValueError('Estimated board is behind camera')
            projected = cv2.projectPoints(self.object_points, rvec, tvec, k, d)[0]
            error = float(np.sqrt(np.mean(np.sum((projected - corners)**2, axis=2))))
            if not np.isfinite(error) or error > self.cfg['max_reprojection_error_px']:
                raise ValueError(f'Reprojection error too high: {error:.3f}px')
            index = len(self.session['samples'])
            prefix = self.directory / f'{index:03d}_{split}'
            annotated = restored.copy()
            cv2.drawChessboardCorners(annotated, size, corners, True)
            for suffix, pixels in [('_raw.png', raw), ('_corners.png', annotated)]:
                if not cv2.imwrite(str(prefix) + suffix, pixels):
                    raise OSError('Could not save image')
            sample = {
                'split': split, 'stamp_ns': stamp.nanoseconds,
                'T_base_tool': pose.tolist(), 'T_camera_board': board.tolist(),
                'camera_matrix': k.tolist(), 'distortion': d.tolist(),
                'corners': corners.reshape(-1, 2).tolist(),
                'reprojection_error_px': error, 'image_prefix': prefix.name,
            }
            samples = self.session['samples'] + [sample]
            updated = {**self.session, 'samples': samples}
            temporary = self.directory / 'session.json.tmp'
            temporary.write_text(json.dumps(updated, indent=2, allow_nan=False) + '\n',
                                 encoding='utf-8')
            temporary.replace(self.directory / 'session.json')
            self.session = updated
            response.success = True
            response.message = f'Saved {index}: {split}, reprojection RMS={error:.3f}px'
        except (ValueError, OSError, RuntimeError, CvBridgeError,
                TransformException, cv2.error) as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(response.message)
        return response

    def solve(self, request, response):
        try:
            report = save_result(self.session, self.directory)
            response.success = True
            response.message = (
                f'Candidate saved to {self.directory}; '
                f'training={report["training"]}; validation={report["validation"]}. '
                'Inspect validation before applying camera_tf_candidate.yaml.')
        except (ValueError, OSError, cv2.error) as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandeyeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
