"""Desktop hand-eye calibration workbench, backed by the ROS 2 collector."""

from datetime import datetime
from collections import deque
from pathlib import Path
import signal
import sys
import time

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSpinBox, QSplitter,
    QSizePolicy, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)
import rclpy

from handeye_calibration.gui_worker import CalibrationWorker


STYLE = '''
QMainWindow, QWidget { background: #101827; color: #dce6f2; font-size: 13px; }
QGroupBox { border: 1px solid #2a3a50; border-radius: 10px; margin-top: 13px;
            padding: 18px 12px 12px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #b8cadf; }
QLineEdit, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTableWidget {
    background: #0c1320; border: 1px solid #2a3a50; border-radius: 5px;
    padding: 6px; selection-background-color: #245c70; }
QPushButton { background: #223249; border: 1px solid #344960; border-radius: 6px;
              padding: 9px 12px; font-weight: 600; }
QPushButton:hover { background: #30445e; border-color: #4d758e; }
QPushButton:disabled { background: #192334; color: #63748a; border-color: #263447; }
QPushButton[primary="true"] { background: #159a88; border-color: #24b6a0; color: white; }
QPushButton[primary="true"]:hover { background: #1aaf9b; }
QPushButton[primary="true"]:disabled { background: #19463f; color: #789b93; border-color: #28534b; }
QLabel#title { font-size: 23px; font-weight: 700; color: #eff7ff; }
QLabel#subtitle { color: #8fa4bc; font-size: 12px; }
QLabel#preview { background: #080f1b; border: 1px solid #293c55;
                 border-radius: 8px; color: #7189a5; }
QLabel#count { font-size: 23px; color: #63dfc6; font-weight: 700; }
QHeaderView::section { background: #1b2a3d; color: #adc1d8; padding: 7px; border: 0; }
QTableWidget { gridline-color: #243348; }
QTabWidget::pane { border: 1px solid #2a3a50; border-radius: 7px; }
QTabBar::tab { background: #182438; color: #8fa4bc; padding: 9px 20px; }
QTabBar::tab:selected { background: #263c51; color: #75e1cd; }
QScrollArea { border: 0; }
QCheckBox { spacing: 7px; font-weight: normal; }
QStatusBar { color: #8fa4bc; }
'''


class ImageView(QLabel):
    """Keep the source pixmap and preserve aspect ratio on every resize."""

    def __init__(self):
        super().__init__('等待相机图像\n\n启动相机与机械臂状态反馈后，点击「新建实时会话」')
        self.setObjectName('preview')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(360, 240)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.source = QPixmap()

    def set_rgb(self, pixels):
        pixels = np.ascontiguousarray(pixels)
        height, width, _ = pixels.shape
        image = QImage(pixels.data, width, height, pixels.strides[0], QImage.Format_RGB888)
        self.set_source(QPixmap.fromImage(image.copy()))

    def set_source(self, pixmap):
        self.source = pixmap
        self._scale()

    def _scale(self):
        if not self.source.isNull():
            self.setPixmap(self.source.scaled(self.size(), Qt.KeepAspectRatio,
                                             Qt.SmoothTransformation))

    def reset(self, text):
        self.source = QPixmap()
        self.clear()
        self.setText(text)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale()


class CalibrationWindow(QMainWindow):
    """UI thread only: display snapshots and submit worker commands."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('arm2 · Astra S 手眼标定')
        self.resize(1440, 960)
        self.setMinimumSize(1120, 760)
        self.worker = None
        self.snapshot_data = None
        self.candidate = None
        self._busy = False
        self._ready = False
        self._closing = False
        self._sample_view = False
        self._health = {}
        self._render_times = deque(maxlen=120)
        self._display_age = None
        self._displayed_at = None
        self._stream_reconnect_count = 0
        self.fields = {}
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(20, 16, 20, 12)
        header = QHBoxLayout()
        heading = QVBoxLayout()
        title = QLabel('手眼标定工作台')
        title.setObjectName('title')
        subtitle = QLabel('ARM 02  /  ASTRA S     ·     眼在手上  ·  固定棋盘格')
        subtitle.setObjectName('subtitle')
        heading.addWidget(title)
        heading.addWidget(subtitle)
        header.addLayout(heading)
        header.addStretch()
        self.phase = QLabel('尚未开始')
        self.phase.setStyleSheet('color: #62d9c3; font-size: 15px;')
        header.addWidget(self.phase)
        layout.addLayout(header)
        body = QSplitter(Qt.Horizontal)
        body.addWidget(self.build_settings())
        body.addWidget(self.build_preview())
        body.addWidget(self.build_actions())
        body.setSizes([260, 800, 340])
        layout.addWidget(body, 1)
        self.path_label = QLabel('数据目录：新建会话后显示。样本会自动保存。')
        self.path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.path_label.setObjectName('subtitle')
        layout.addWidget(self.path_label)
        self.setCentralWidget(root)
        self.setStyleSheet(STYLE)
        # Pull at most one latest frame per tick; never queue video Qt signals.
        self.preview_timer = QTimer(self)
        self.preview_timer.setTimerType(Qt.PreciseTimer)
        self.preview_timer.timeout.connect(self.consume_preview)
        self.preview_timer.start(33)
        self.update_controls()

    def build_settings(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 8, 0)
        self.settings = QGroupBox('01  采集配置')
        form = QFormLayout(self.settings)
        form.setLabelAlignment(Qt.AlignLeft)
        form.setRowWrapPolicy(QFormLayout.WrapAllRows)
        for key, title, default in [
            ('image_topic', '彩色图像话题', '/color/image_raw'),
            ('camera_info_topic', '相机内参话题', '/color/camera_info'),
            ('base_frame', '机械臂基座', 'arm2_base'),
            ('tool_frame', '末端坐标系', 'arm2_tool_link'),
            ('camera_frame', '彩色光学坐标系', 'camera_color_frame'),
        ]:
            self.fields[key] = QLineEdit(default)
            form.addRow(title, self.fields[key])
        dimensions = QHBoxLayout()
        for key, value, suffix in [('board_columns', 11, ' 列'), ('board_rows', 8, ' 行')]:
            spin = QSpinBox()
            spin.setRange(3, 40)
            spin.setValue(value)
            spin.setSuffix(suffix)
            self.fields[key] = spin
            dimensions.addWidget(spin)
        form.addRow('棋盘格内角点', dimensions)
        self.square = QDoubleSpinBox()
        self.square.setRange(0.1, 500)
        self.square.setDecimals(2)
        self.square.setValue(20)
        self.square.setSuffix(' mm')
        form.addRow('单格边长', self.square)
        self.flip = QCheckBox('撤销驱动的水平翻转')
        self.flip.setChecked(True)
        self.flip.setToolTip('适用于本仓库 Astra S 驱动中的 cv::flip(..., 1)')
        form.addRow(self.flip)
        self.reliable = QCheckBox('可靠传输（本工程相机驱动）')
        self.reliable.setChecked(True)
        self.reliable.setToolTip('关闭后使用 best-effort，仅用于不支持可靠传输的相机驱动')
        form.addRow(self.reliable)
        self.output = QLineEdit(str(Path.home() / 'handeye_sessions'))
        form.addRow('会话保存目录', self.output)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.settings)
        layout.addWidget(scroll, 1)
        self.start_button = QPushButton('新建实时会话')
        self.start_button.setProperty('primary', True)
        self.start_button.clicked.connect(self.start_session)
        self.open_button = QPushButton('打开历史会话…')
        self.open_button.clicked.connect(self.open_session)
        self.stop_button = QPushButton('停止采集')
        self.stop_button.clicked.connect(self.stop_session)
        self.reconnect_button = QPushButton('重新订阅相机数据')
        self.reconnect_button.clicked.connect(lambda: self.command('reconnect'))
        layout.addWidget(self.start_button)
        layout.addWidget(self.open_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.reconnect_button)
        return panel

    def build_preview(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 0, 6, 0)
        badges = QHBoxLayout()
        self.badges = {}
        for key, title in [('image', '图像'), ('intrinsics', '内参'),
                           ('tf', '机械臂 TF'), ('board', '棋盘格')]:
            badge = QLabel(f'● {title}：等待')
            badge.setStyleSheet('color: #8394ab; padding: 8px 0;')
            self.badges[key] = badge
            badges.addWidget(badge)
        layout.addLayout(badges)
        self.image = ImageView()
        layout.addWidget(self.image, 5)
        row = QHBoxLayout()
        self.preview_label = QLabel('实时预览 · 已撤销驱动水平翻转')
        self.preview_label.setObjectName('subtitle')
        row.addWidget(self.preview_label, 1)
        self.performance_label = QLabel('预览 — FPS · 角点 — Hz')
        self.performance_label.setObjectName('subtitle')
        row.addWidget(self.performance_label)
        self.live_button = QPushButton('返回实时预览')
        self.live_button.clicked.connect(self.return_live)
        row.addWidget(self.live_button)
        layout.addLayout(row)
        self.health_label = QLabel('请保持底盘与棋盘格固定；每次停稳机械臂后采样。')
        self.health_label.setWordWrap(True)
        self.health_label.setObjectName('subtitle')
        layout.addWidget(self.health_label)
        tabs = QTabWidget()
        samples = QWidget()
        sample_layout = QVBoxLayout(samples)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['编号', '用途', '重投影 RMS', '状态', '图像时间戳 ns'])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self.show_sample)
        sample_layout.addWidget(self.table)
        self.toggle_button = QPushButton('排除 / 恢复所选样本')
        self.toggle_button.clicked.connect(self.toggle_sample)
        sample_layout.addWidget(self.toggle_button)
        tabs.addTab(samples, '采集样本')
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        tabs.addTab(self.log, '操作记录')
        layout.addWidget(tabs, 3)
        return panel

    def build_actions(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 0, 0, 0)
        workflow = QGroupBox('02  采集与求解')
        actions = QVBoxLayout(workflow)
        counts = QHBoxLayout()
        self.training_count = QLabel('0 / 20')
        self.validation_count = QLabel('0 / 5')
        for label, title in [(self.training_count, '训练样本 · 推荐 20–30 组'),
                             (self.validation_count, '独立验证 · 推荐至少 5 组')]:
            box = QVBoxLayout()
            label.setObjectName('count')
            box.addWidget(label)
            caption = QLabel(title)
            caption.setObjectName('subtitle')
            caption.setWordWrap(True)
            box.addWidget(caption)
            counts.addLayout(box)
        actions.addLayout(counts)
        self.capture_button = QPushButton('采集训练样本')
        self.capture_button.setProperty('primary', True)
        self.capture_button.clicked.connect(lambda: self.command('capture'))
        self.validation_button = QPushButton('采集独立验证样本')
        self.validation_button.clicked.connect(lambda: self.command('capture_validation'))
        self.solve_button = QPushButton('计算手眼标定')
        self.solve_button.clicked.connect(lambda: self.command('solve'))
        for button in (self.capture_button, self.validation_button, self.solve_button):
            actions.addWidget(button)
        hint = QLabel('至少 15 组训练数据。请改变距离与倾角，绕不同轴旋转；避免棋盘格编号起点翻转。')
        hint.setWordWrap(True)
        hint.setObjectName('subtitle')
        actions.addWidget(hint)
        layout.addWidget(workflow)
        results = QGroupBox('03  标定结果')
        result_layout = QVBoxLayout(results)
        self.result_label = QLabel('等待求解\n\n采集训练数据和独立验证数据后，点击「计算手眼标定」。')
        self.result_label.setWordWrap(True)
        self.result_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        result_layout.addWidget(self.result_label)
        self.matrix = QPlainTextEdit()
        self.matrix.setReadOnly(True)
        self.matrix.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.matrix.setFont(QFont('monospace', 10))
        self.matrix.setPlaceholderText('T_tool_camera\n平移单位：米\n四元数顺序：xyzw')
        result_layout.addWidget(self.matrix, 1)
        self.export_button = QPushButton('导出 TF YAML…')
        self.export_button.clicked.connect(self.export_yaml)
        result_layout.addWidget(self.export_button)
        note = QLabel('导出的是候选彩色相机外参。请依据验证误差与实际任务要求判断是否采用。')
        note.setWordWrap(True)
        note.setObjectName('subtitle')
        result_layout.addWidget(note)
        layout.addWidget(results, 1)
        return panel

    def config(self):
        config = {key: field.text().strip() if isinstance(field, QLineEdit) else field.value()
                  for key, field in self.fields.items()}
        if any(isinstance(value, str) and not value for value in config.values()):
            raise ValueError('话题与坐标系名称不能为空')
        if not self.output.text().strip():
            raise ValueError('请填写会话保存目录')
        config.update(square_size_m=self.square.value() / 1000,
                      undo_horizontal_flip=self.flip.isChecked(),
                      reliable_transport=self.reliable.isChecked(),
                      output_directory=self.output.text().strip())
        return config

    def start_session(self):
        try:
            self.launch_worker(CalibrationWorker(config=self.config(), parent=self))
        except ValueError as exc:
            self.notify(str(exc), False)

    def open_session(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, '打开历史 session.json（排除样本会保存回该会话）',
            str(Path.home() / 'handeye_sessions'), '会话文件 (session.json);;JSON (*.json)')
        if filename:
            self.launch_worker(CalibrationWorker(session_path=filename, parent=self))

    def launch_worker(self, worker):
        if self.worker is not None:
            worker.deleteLater()
            return
        self.worker = worker
        self._ready = False
        self._busy = True
        self._health = {}
        self._render_times.clear()
        self._display_age = None
        self._displayed_at = None
        self._stream_reconnect_count = 0
        self._sample_view = False
        self.candidate = None
        self.image.reset('正在启动会话…')
        self.render_result(None)
        worker.snapshot.connect(self.on_snapshot)
        worker.health.connect(self.on_health)
        worker.completed.connect(self.on_completed)
        worker.ready.connect(self.on_ready)
        worker.failed.connect(lambda message: self.notify(message, False))
        worker.finished.connect(self.on_finished)
        self.phase.setText('正在启动…')
        self.update_controls()
        worker.start()

    def stop_session(self):
        if self.worker is not None:
            self._ready = False
            self.worker.requestInterruption()
            self.phase.setText('正在停止…')
            self.update_controls()

    def on_ready(self):
        self._ready = True
        self._busy = False
        offline = self.snapshot_data and self.snapshot_data['offline']
        self.phase.setText('历史会话 · 离线复算' if offline else '实时会话 · 等待采样')
        self.notify('历史会话已打开，可检查样本并重新求解。' if offline else
                    '会话已启动，请确认图像、内参和机械臂 TF。')
        self.update_controls()

    def on_finished(self):
        old_worker = self.worker
        self.worker = None
        self._ready = False
        self._busy = False
        self.phase.setText('会话已停止 · 数据已保留')
        self.on_health({})
        self.update_controls()
        if old_worker is not None:
            old_worker.deleteLater()
        if self._closing:
            self.close()

    def command(self, operation, value=None):
        if self.worker is None or not self._ready or self._busy:
            return
        self._busy = True
        self.phase.setText('正在求解…' if operation == 'solve' else '正在处理…')
        self.update_controls()
        self.worker.submit(operation, value)

    def on_completed(self, operation, success, message):
        self._busy = False
        self.phase.setText('操作完成' if success else '操作未完成 · 查看提示')
        self.notify(message, success)
        self.update_controls()

    def notify(self, message, success=True):
        self.log.appendPlainText(f'{datetime.now():%H:%M:%S}  '
                                 f'{"✓" if success else "!"}  {message}')
        self.statusBar().showMessage(message)

    def on_snapshot(self, data):
        self.snapshot_data = data
        self.candidate = data['candidate']
        self.path_label.setText(f'数据目录：{data["directory"]}')
        self.path_label.setToolTip(data['directory'])
        session = data['session']
        if data['offline']:
            for key, field in self.fields.items():
                if key in session:
                    if isinstance(field, QLineEdit):
                        field.setText(str(session[key]))
                    else:
                        field.setValue(session[key])
            self.square.setValue(session.get('square_size_m', 0.02) * 1000)
            self.flip.setChecked(session.get('undo_horizontal_flip', True))
            self.reliable.setChecked(session.get('reliable_transport', True))
        previous_row = self.table.currentRow()
        self.table.blockSignals(True)
        samples = session['samples']
        self.table.setRowCount(len(samples))
        counts = {'training': 0, 'validation': 0}
        for row, sample in enumerate(samples):
            excluded = sample.get('excluded', False)
            if not excluded:
                counts[sample['split']] += 1
            values = [f'{row:03d}', '训练' if sample['split'] == 'training' else '验证',
                      f'{sample.get("reprojection_error_px", 0):.3f} px',
                      '已排除' if excluded else '参与计算', str(sample.get('stamp_ns', '—'))]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
        if 0 <= previous_row < len(samples):
            self.table.selectRow(previous_row)
        self.table.blockSignals(False)
        self.training_count.setText(f'{counts["training"]} / 20')
        self.validation_count.setText(f'{counts["validation"]} / 5')
        self.render_result(data['report'])
        self.update_controls()

    def render_result(self, report):
        if report is None:
            self.result_label.setText('尚无当前样本的计算结果。\n修改样本后需要重新求解。')
            self.matrix.clear()
            return
        lines = ['固定标定板一致性误差（RMS / 最大值）']
        for key, title in [('training', '训练'), ('validation', '独立验证')]:
            metric = report[key]
            if metric is None:
                lines.append('\n独立验证：未采集，精度尚未验证')
                continue
            lines.append(f'\n{title} · {metric["count"]} 组\n'
                         f'位置 {metric["translation_rms_mm"]:.3f} / '
                         f'{metric["translation_max_mm"]:.3f} mm\n'
                         f'旋转 {metric["rotation_rms_deg"]:.3f} / '
                         f'{metric["rotation_max_deg"]:.3f} °')
        self.result_label.setText('\n'.join(lines))
        self.matrix.setPlainText('T_tool_camera  [m]\n' +
                                 np.array2string(np.asarray(report['T_tool_camera']),
                                                 precision=6, suppress_small=True) +
                                 '\n\n候选 TF 配置\n' + (self.candidate or ''))

    def on_health(self, health):
        self._health = health
        reconnect_count = health.get('reconnect_count', 0)
        if reconnect_count > self._stream_reconnect_count:
            self.notify(f'已重新订阅相机数据 {reconnect_count} 次，等待新帧；已有样本保留。')
            self._stream_reconnect_count = reconnect_count
        for key, badge in self.badges.items():
            title = {'image': '图像', 'intrinsics': '内参', 'tf': '机械臂 TF', 'board': '棋盘格'}[key]
            offline = health.get('offline', False)
            good = health.get(key, False)
            badge.setText(f'● {title}：' + ('离线' if offline else '就绪' if good else '等待'))
            badge.setStyleSheet(f'color: {"#63dfc6" if good else "#8a9bb2"}; padding: 8px 0;')
        self.health_label.setText('历史会话模式：选择样本可查看角点图，排除坏样本后重新求解。'
                                  if health.get('offline') else health.get('message', '采集已停止。'))
        self.update_controls()
        self.update_performance()

    def consume_preview(self):
        if self.worker is None:
            return
        frame = self.worker.frames.take()
        if frame is not None and not self._sample_view:
            self._display_age = frame['age'] + time.monotonic() - frame['created_at']
            self.on_preview(frame['pixels'])

    def on_preview(self, pixels):
        if not self._sample_view:
            self.image.set_rgb(pixels)
            now = time.monotonic()
            self._displayed_at = now
            self._render_times.append(now)
            self.update_performance()
            self.preview_label.setText('实时预览 · ' +
                                       ('已撤销驱动水平翻转' if self.flip.isChecked() else '原始发布图像'))

    def update_performance(self):
        now = time.monotonic()
        while self._render_times and now - self._render_times[0] > 2:
            self._render_times.popleft()
        fps = ((len(self._render_times) - 1) /
               (self._render_times[-1] - self._render_times[0])
               if len(self._render_times) > 1 else 0.0)
        age = (f' · {(self._display_age + now - self._displayed_at) * 1000:.0f} ms'
               if self._display_age is not None and self._displayed_at is not None else '')
        self.performance_label.setText(
            f'预览 {fps:.1f} FPS · 角点 {self._health.get("detection_fps", 0):.1f} Hz{age}')

    def show_sample(self):
        row = self.table.currentRow()
        self.update_controls()
        if row < 0 or self.snapshot_data is None:
            return
        sample = self.snapshot_data['session']['samples'][row]
        prefix = sample.get('image_prefix', '')
        filename = Path(self.snapshot_data['directory']) / f'{prefix}_corners.png'
        pixmap = QPixmap(str(filename)) if prefix else QPixmap()
        self._sample_view = True
        if pixmap.isNull():
            self.image.reset('该样本没有可读取的角点图像')
        else:
            self.image.set_source(pixmap)
        self.preview_label.setText(f'样本 {row:03d} · 已保存的角点检测结果')

    def return_live(self):
        self._sample_view = False
        self.table.clearSelection()
        self.table.setCurrentCell(-1, -1)
        self.preview_label.setText('实时预览')
        self.image.reset('等待下一帧实时图像…')

    def toggle_sample(self):
        row = self.table.currentRow()
        if row >= 0:
            self.command('toggle_sample', row)

    def update_controls(self):
        active = self.worker is not None
        available = active and self._ready and not self._busy
        offline = bool(self.snapshot_data and self.snapshot_data['offline'])
        capture_ready = available and not offline and self._health.get('capture_ready', False)
        self.settings.setEnabled(not active)
        self.start_button.setEnabled(not active)
        self.open_button.setEnabled(not active)
        self.stop_button.setEnabled(active and self._ready)
        self.capture_button.setEnabled(bool(capture_ready))
        self.validation_button.setEnabled(bool(capture_ready))
        training = sum(s['split'] == 'training' and not s.get('excluded', False)
                       for s in self.snapshot_data['session']['samples']) if self.snapshot_data else 0
        self.solve_button.setEnabled(available and training >= 15)
        self.toggle_button.setEnabled(available and self.table.currentRow() >= 0)
        self.export_button.setEnabled(bool(self.candidate) and not self._busy)
        self.live_button.setEnabled(active and not offline)
        self.reconnect_button.setEnabled(available and not offline)

    def export_yaml(self):
        if not self.candidate:
            return
        filename, _ = QFileDialog.getSaveFileName(
            self, '导出候选彩色相机外参', str(Path.home() / 'camera_tf_candidate.yaml'),
            'YAML (*.yaml *.yml)')
        if not filename:
            return
        try:
            Path(filename).write_text(self.candidate, encoding='utf-8')
            self.notify(f'候选外参已导出：{filename}')
        except OSError as exc:
            QMessageBox.warning(self, '无法保存文件', str(exc))

    def closeEvent(self, event):
        if self.worker is not None:
            self._closing = True
            self.setEnabled(False)
            self.stop_session()
            event.ignore()
        else:
            event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    app.setFont(QFont('Noto Sans CJK SC', 10))
    window = CalibrationWindow()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)
    window.show()
    try:
        return app.exec_()
    finally:
        if window.worker is not None:
            window.worker.requestInterruption()
            window.worker.wait()
        if rclpy.ok():
            rclpy.shutdown()
