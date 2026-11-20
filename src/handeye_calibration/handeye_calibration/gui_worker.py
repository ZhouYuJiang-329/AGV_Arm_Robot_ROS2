"""Qt worker: owns the ROS executor and serializes calibration operations."""

import copy
import json
from pathlib import Path
from queue import Empty, Queue
import time

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger

from handeye_calibration.node import HandeyeNode
from handeye_calibration.preview import CornerPreview, LatestFrame
from handeye_calibration.solver import rigid_transform, save_result


class CalibrationWorker(QThread):
    """No widgets or ROS objects cross the thread boundary; use copied snapshots."""

    snapshot = pyqtSignal(object)
    health = pyqtSignal(object)
    completed = pyqtSignal(str, bool, str)
    ready = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(self, config=None, session_path=None, parent=None):
        super().__init__(parent)
        self.config = config or {}
        self.session_path = Path(session_path) if session_path else None
        self.commands = Queue()
        self.node = None
        self.session = None
        self.directory = None
        self.report = None
        self.candidate = None
        self._preview_stamp = None
        self._board_found = False
        self.frames = LatestFrame()
        self.detector = None
        self._last_health = 0.0

    def submit(self, operation, value=None):
        """Called on the UI thread; only touch the thread-safe queue here."""
        self.commands.put((operation, value))

    def run(self):
        executor = None
        try:
            if self.session_path is not None:
                self.session = json.loads(self.session_path.read_text(encoding='utf-8'))
                for key in ('samples', 'tool_frame', 'camera_frame'):
                    if key not in self.session:
                        raise ValueError(f'会话缺少字段：{key}')
                for sample in self.session['samples']:
                    rigid_transform(sample['T_base_tool'])
                    rigid_transform(sample['T_camera_board'])
                    if sample['split'] not in ('training', 'validation'):
                        raise ValueError('样本类别必须是 training 或 validation')
                self.directory = self.session_path.parent
            else:
                overrides = [Parameter(name, value=value) for name, value in self.config.items()]
                self.node = HandeyeNode(overrides, node_name='handeye_calibration_gui')
                self.session = self.node.session
                self.directory = self.node.directory
                executor = SingleThreadedExecutor(context=self.node.context)
                executor.add_node(self.node)
                self.detector = CornerPreview((self.node.cfg['board_columns'],
                                               self.node.cfg['board_rows']))
            self.emit_snapshot()
            self.ready.emit()
            last_offline_health = 0.0
            while not self.isInterruptionRequested():
                if executor is not None:
                    executor.spin_once(timeout_sec=0.01)
                    if self.session is not self.node.session:
                        self.session = self.node.session
                        self.report, self.candidate = None, None
                        self.emit_snapshot()
                else:
                    self.msleep(20)
                try:
                    operation, value = self.commands.get_nowait()
                except Empty:
                    operation = None
                if operation is not None:
                    self.execute(operation, value)
                if self.node is not None:
                    self.update_preview()
                elif time.monotonic() - last_offline_health > 0.2:
                    self.health.emit({'offline': True})
                    last_offline_health = time.monotonic()
        except Exception as exc:
            self.failed.emit(f'{type(exc).__name__}: {exc}')
        finally:
            if self.detector is not None:
                self.detector.close()
            if executor is not None:
                executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
                self.node = None

    def emit_snapshot(self):
        self.snapshot.emit({
            'session': copy.deepcopy(self.session),
            'directory': str(self.directory),
            'report': copy.deepcopy(self.report),
            'candidate': self.candidate,
            'offline': self.session_path is not None,
        })

    def execute(self, operation, value):
        try:
            if operation in ('capture', 'capture_validation'):
                if self.node is None:
                    raise ValueError('历史会话模式不采集实时数据，请新建实时会话')
                callback = (self.node.capture_training if operation == 'capture'
                            else self.node.capture_validation)
                response = callback(Trigger.Request(), Trigger.Response())
                if not response.success:
                    raise ValueError(response.message)
                self.session = self.node.session
                self.report, self.candidate = None, None
                message = response.message
            elif operation == 'solve':
                self.report, self.candidate = None, None
                self.report = save_result(self.session, self.directory)
                self.candidate = (self.directory / 'camera_tf_candidate.yaml').read_text(
                    encoding='utf-8')
                message = '求解完成，请检查独立验证误差；候选外参已保存。'
            elif operation == 'toggle_sample':
                updated = copy.deepcopy(self.session)
                sample = updated['samples'][int(value)]
                sample['excluded'] = not sample.get('excluded', False)
                temporary = self.directory / 'session.json.tmp'
                temporary.write_text(json.dumps(updated, indent=2, allow_nan=False) + '\n',
                                     encoding='utf-8')
                temporary.replace(self.directory / 'session.json')
                self.session = updated
                if self.node is not None:
                    self.node.session = updated
                self.report, self.candidate = None, None
                message = '样本已排除，请重新求解。' if sample['excluded'] else '样本已恢复，请重新求解。'
            elif operation == 'reconnect':
                if self.node is None:
                    raise ValueError('历史模式无需连接相机')
                self.node.reconnect_stream()
                message = '已重新订阅相机数据，等待新图像；已有标定样本保留。'
            else:
                raise ValueError(f'未知操作：{operation}')
            self.emit_snapshot()
            self.completed.emit(operation, True, message)
        except Exception as exc:
            self.emit_snapshot()
            self.completed.emit(operation, False, str(exc))

    def update_preview(self):
        """Process new video frames without waiting for the corner detector."""
        now = time.monotonic()
        if self.node.latest_image is None:
            self.frames.take()
            self._preview_stamp = None
            self._board_found = False
            if now - self._last_health >= 0.2:
                status = self.node.stream_status()
                self.health.emit({'image': False, 'tf': False, 'intrinsics': False,
                                  'capture_ready': False, **status,
                                  'message': ('接收中断，已重新订阅，正在等待新帧…'
                                              if status['reconnect_count'] else '等待相机图像…')})
                self._last_health = now
            return
        image, info = self.node.latest_image, self.node.latest_info
        stamp = Time.from_msg(image.header.stamp)
        age = (self.node.get_clock().now() - stamp).nanoseconds / 1e9
        fresh = stamp.nanoseconds > 0 and 0 <= age <= self.node.cfg['max_image_age_s']
        paired = self.node.latest
        pair_fresh = False
        if paired is not None:
            pair_stamp = Time.from_msg(paired[0].header.stamp)
            pair_age = (self.node.get_clock().now() - pair_stamp).nanoseconds / 1e9
            pair_fresh = pair_stamp.nanoseconds > 0 and 0 <= pair_age <= self.node.cfg['max_image_age_s']
        tf_ready = pair_fresh and self.node.buffer.can_transform(
            self.node.cfg['base_frame'], self.node.cfg['tool_frame'], pair_stamp)
        intrinsics = (info is not None and info.k[0] > 0 and info.k[4] > 0
                      and np.isfinite(info.k).all() and np.isfinite(info.d).all()
                      and info.distortion_model == 'plumb_bob' and len(info.d) in (4, 5)
                      and (image.width, image.height) == (info.width, info.height)
                      and image.header.frame_id == info.header.frame_id
                      == self.node.cfg['camera_frame'])
        warning = ''
        if fresh and self._preview_stamp != stamp.nanoseconds:
            try:
                raw = self.node.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                restored = cv2.flip(raw, 1) if self.node.cfg['undo_horizontal_flip'] else raw
                scale = min(1.0, 800 / restored.shape[1])
                display = cv2.resize(restored, None, fx=scale, fy=scale)
                size = (self.node.cfg['board_columns'], self.node.cfg['board_rows'])
                corners = self.detector.update(display, now)
                self._board_found = corners is not None
                if corners is not None:
                    cv2.drawChessboardCorners(display, size, corners, True)
                    first = tuple(np.round(corners[0, 0]).astype(int))
                    cv2.circle(display, first, 9, (0, 170, 255), 2)
                    cv2.putText(display, '#0', (first[0]+12, first[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 170, 255), 2)
                self.frames.put({'pixels': cv2.cvtColor(display, cv2.COLOR_BGR2RGB),
                                 'stamp_ns': stamp.nanoseconds, 'created_at': now,
                                 'age': age})
                self._preview_stamp = stamp.nanoseconds
            except Exception as exc:
                self._board_found = False
                warning = str(exc)
        if now - self._last_health >= 0.2:
            status = self.node.stream_status()
            if fresh:
                stream_message = '停稳机械臂后采集；角点叠加仅作预览参考'
            elif status['image_receive_gap_s'] > self.node.cfg['max_image_age_s']:
                stream_message = (f'已 {status["image_receive_gap_s"]:.1f} 秒未收到新图像，'
                                  '正在自动恢复订阅…')
            elif age < 0:
                stream_message = f'收到的图像时间戳超前 {-age:.1f} 秒，请检查相机与 ROS 时钟'
            else:
                stream_message = f'仍在收帧，但时间戳落后 {age:.1f} 秒，请检查时间戳与上游缓冲'
            self.health.emit({
                **status,
                'image': fresh, 'tf': bool(tf_ready), 'intrinsics': bool(intrinsics),
                'capture_ready': bool(fresh and intrinsics and pair_fresh and tf_ready),
                'board': bool(self._board_found and fresh), 'age': age,
                'detection_fps': self.detector.fps,
                'message': warning or self.detector.error or stream_message,
            })
            self._last_health = now
