import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from PyQt5.QtCore import QSignalBlocker, QTimer, Qt
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node

from learm_driver.srv import GetArmStatus, MoveJoints, MovePose, SetGripper
from std_srvs.srv import Trigger


JOINT_NAMES = ('joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6')
SERVO_NAMES = ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6')
JOINT_LIMIT_DEGREES = 90.0
GRIPPER_OPEN_PULSE_US = 500.0
GRIPPER_CLOSED_PULSE_US = 1400.0
LIVE_SEND_INTERVAL_MS = 100
STATUS_POLL_INTERVAL_MS = 500
MIN_DURATION_MS = 200
MAX_DURATION_MS = 1000
RESET_DURATION_MS = 2000
RECORDING_FORMAT_VERSION = 2
EXECUTION_MODE_SYNCHRONIZED = 'synchronized'
EXECUTION_MODE_SEQUENCED = 'sequenced'
GRIPPER_TIMING_BEFORE = 'before'
GRIPPER_TIMING_AFTER = 'after'
GRIPPER_TIMING_NONE = 'none'


@dataclass
class Keyframe:
    duration_ms: int
    opening: float
    positions_deg: list[float]
    execution_mode: str
    gripper_timing: str


@dataclass
class MotionStage:
    kind: str
    label: str
    duration_ms: int
    joint_names: tuple[str, ...] = ()
    positions_deg: tuple[float, ...] = ()
    opening: float | None = None


def pulse_to_estimated_degrees(pulse_us):
    degrees = ((float(pulse_us) - 1500.0) / 1000.0) * JOINT_LIMIT_DEGREES
    return max(-JOINT_LIMIT_DEGREES, min(JOINT_LIMIT_DEGREES, degrees))


class RecorderClient(Node):
    def __init__(self):
        super().__init__('learm_motion_recorder')
        self.move_client = self.create_client(MoveJoints, '/learm_driver/move_joints')
        self.pose_client = self.create_client(MovePose, '/learm_driver/move_pose')
        self.gripper_client = self.create_client(SetGripper, '/learm_driver/set_gripper')
        self.status_client = self.create_client(GetArmStatus, '/learm_driver/get_status')
        self.estop_client = self.create_client(Trigger, '/learm_driver/emergency_stop')


class AngleControl(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-900, 900)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(50)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(-JOINT_LIMIT_DEGREES, JOINT_LIMIT_DEGREES)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(1.0)
        self.spin.setSuffix(' deg')

        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)

    def _slider_changed(self, value):
        blocker = QSignalBlocker(self.spin)
        self.spin.setValue(value / 10.0)
        del blocker

    def _spin_changed(self, value):
        blocker = QSignalBlocker(self.slider)
        self.slider.setValue(round(value * 10.0))
        del blocker

    def value_degrees(self):
        return self.spin.value()

    def set_value_degrees(self, value):
        slider_blocker = QSignalBlocker(self.slider)
        spin_blocker = QSignalBlocker(self.spin)
        self.slider.setValue(round(value * 10.0))
        self.spin.setValue(value)
        del slider_blocker
        del spin_blocker

    def setEnabled(self, enabled):
        self.slider.setEnabled(enabled)
        self.spin.setEnabled(enabled)
        super().setEnabled(enabled)


class MotionRecorderWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.keyframes = []
        self.pending_joints = set()
        self.pending_gripper = False
        self.live_request_in_flight = False
        self.last_live_send_time = 0.0
        self.flush_after_response = False
        self.status_request_in_flight = False
        self.status_poll_request_in_flight = False
        self.playback_active = False
        self.playback_index = 0
        self.playback_generation = 0
        self.playback_stages = []
        self.playback_stage_index = 0
        self.playback_stop_after_one = False
        self.reset_active = False

        self.setWindowTitle('LeArm 实时控制与动作组')
        self.setMinimumSize(1120, 700)
        self._build_ui()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(20)

        self.live_timer = QTimer(self)
        self.live_timer.setInterval(LIVE_SEND_INTERVAL_MS)
        self.live_timer.timeout.connect(self._dispatch_live_command)

        self.status_poll_timer = QTimer(self)
        self.status_poll_timer.setInterval(STATUS_POLL_INTERVAL_MS)
        self.status_poll_timer.timeout.connect(self._poll_status)
        self.status_poll_timer.start()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        pose_box = QGroupBox('实时姿态控制')
        pose_layout = QFormLayout(pose_box)
        self.angle_controls = {}
        for joint_name in JOINT_NAMES:
            control = AngleControl()
            self.angle_controls[joint_name] = control
            control.slider.valueChanged.connect(
                lambda _value, name=joint_name: self._queue_joint(name))
            control.spin.valueChanged.connect(
                lambda _value, name=joint_name: self._queue_joint(name))
            control.slider.sliderReleased.connect(self._flush_live_command)
            pose_layout.addRow(joint_name, control)

        gripper_controls = QHBoxLayout()
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 100)
        self.gripper_slider.setValue(100)
        self.gripper_spin = QSpinBox()
        self.gripper_spin.setRange(0, 100)
        self.gripper_spin.setSuffix(' %')
        self.gripper_spin.setValue(100)
        self.gripper_slider.valueChanged.connect(self._gripper_slider_changed)
        self.gripper_spin.valueChanged.connect(self._gripper_spin_changed)
        self.gripper_slider.valueChanged.connect(self._queue_gripper)
        self.gripper_spin.valueChanged.connect(self._queue_gripper)
        self.gripper_slider.sliderReleased.connect(self._flush_live_command)
        gripper_controls.addWidget(self.gripper_slider, 1)
        gripper_controls.addWidget(self.gripper_spin)
        pose_layout.addRow('夹爪开合', gripper_controls)

        duration_controls = QHBoxLayout()
        self.duration_slider = QSlider(Qt.Horizontal)
        self.duration_slider.setRange(MIN_DURATION_MS, MAX_DURATION_MS)
        self.duration_slider.setValue(500)
        self.duration_slider.setSingleStep(10)
        self.duration_slider.setPageStep(100)
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(MIN_DURATION_MS, MAX_DURATION_MS)
        self.duration_spin.setValue(500)
        self.duration_spin.setSuffix(' ms')
        self.duration_spin.setSingleStep(10)
        self.duration_slider.valueChanged.connect(self.duration_spin.setValue)
        self.duration_spin.valueChanged.connect(self.duration_slider.setValue)
        duration_controls.addWidget(self.duration_slider, 1)
        duration_controls.addWidget(self.duration_spin)
        pose_layout.addRow('动作时间', duration_controls)

        mode_controls = QHBoxLayout()
        self.execution_mode_combo = QComboBox()
        self.execution_mode_combo.addItem('安全顺序', EXECUTION_MODE_SEQUENCED)
        self.execution_mode_combo.addItem('同步', EXECUTION_MODE_SYNCHRONIZED)
        self.gripper_timing_combo = QComboBox()
        self.gripper_timing_combo.addItem('夹爪后执行', GRIPPER_TIMING_AFTER)
        self.gripper_timing_combo.addItem('夹爪前执行', GRIPPER_TIMING_BEFORE)
        self.gripper_timing_combo.addItem('夹爪不动作', GRIPPER_TIMING_NONE)
        mode_controls.addWidget(self.execution_mode_combo)
        mode_controls.addWidget(self.gripper_timing_combo)
        pose_layout.addRow('动作组执行方式', mode_controls)

        pose_buttons = QHBoxLayout()
        self.add_button = QPushButton('添加当前姿态')
        self.reset_button = QPushButton('复位到零位')
        self.capture_button = QPushButton('读取 STM32 状态')
        self.estop_button = QPushButton('软件急停')
        pose_buttons.addWidget(self.add_button)
        pose_buttons.addWidget(self.reset_button)
        pose_buttons.addWidget(self.capture_button)
        pose_buttons.addStretch()
        pose_buttons.addWidget(self.estop_button)
        pose_layout.addRow(pose_buttons)
        layout.addWidget(pose_box)

        live_angle_box = QGroupBox('实时 PWM 估算角度')
        live_angle_layout = QGridLayout(live_angle_box)
        self.live_angle_labels = {}
        for index, servo_name in enumerate(SERVO_NAMES):
            column = (index % 3) * 2
            row = index // 3
            name_label = QLabel(servo_name)
            value_label = QLabel('--.- deg')
            self.live_angle_labels[servo_name] = value_label
            live_angle_layout.addWidget(name_label, row, column)
            live_angle_layout.addWidget(value_label, row, column + 1)
        self.live_angle_state_label = QLabel('等待 STM32 状态...')
        self.live_angle_state_label.setWordWrap(True)
        live_angle_layout.addWidget(self.live_angle_state_label, 2, 0, 1, 6)
        layout.addWidget(live_angle_box)

        recording_box = QGroupBox('动作组关键帧')
        recording_layout = QVBoxLayout(recording_box)
        self.table = QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(
            ['#', '时间', '模式', '夹爪时机', '夹爪', 'J2', 'J3', 'J4', 'J5', 'J6'])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._update_controls)
        recording_layout.addWidget(self.table)

        recording_buttons = QGridLayout()
        self.replace_button = QPushButton('替换所选')
        self.delete_button = QPushButton('删除所选')
        self.move_up_button = QPushButton('上移')
        self.move_down_button = QPushButton('下移')
        self.play_all_button = QPushButton('回放全部')
        self.play_selected_button = QPushButton('回放所选')
        self.clear_button = QPushButton('清空动作组')
        self.save_button = QPushButton('保存动作组')
        self.load_button = QPushButton('加载动作组')
        recording_buttons.addWidget(self.replace_button, 0, 0)
        recording_buttons.addWidget(self.delete_button, 0, 1)
        recording_buttons.addWidget(self.move_up_button, 0, 2)
        recording_buttons.addWidget(self.move_down_button, 0, 3)
        recording_buttons.addWidget(self.play_all_button, 1, 0)
        recording_buttons.addWidget(self.play_selected_button, 1, 1)
        recording_buttons.addWidget(self.clear_button, 1, 2)
        recording_buttons.addWidget(self.save_button, 2, 0)
        recording_buttons.addWidget(self.load_button, 2, 1)
        recording_layout.addLayout(recording_buttons)
        layout.addWidget(recording_box, 1)

        self.status_label = QLabel('滑动条会自动发送最新目标。')
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self.add_button.clicked.connect(self._add_keyframe)
        self.reset_button.clicked.connect(self._reset_to_zero)
        self.capture_button.clicked.connect(self._capture_status)
        self.estop_button.clicked.connect(self._emergency_stop)
        self.replace_button.clicked.connect(self._replace_selected)
        self.delete_button.clicked.connect(self._delete_selected)
        self.move_up_button.clicked.connect(lambda: self._move_selected(-1))
        self.move_down_button.clicked.connect(lambda: self._move_selected(1))
        self.play_all_button.clicked.connect(self._play_all)
        self.play_selected_button.clicked.connect(self._play_selected)
        self.clear_button.clicked.connect(self._clear_keyframes)
        self.save_button.clicked.connect(self._save_recording)
        self.load_button.clicked.connect(self._load_recording)
        self._update_controls()

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _gripper_slider_changed(self, value):
        blocker = QSignalBlocker(self.gripper_spin)
        self.gripper_spin.setValue(value)
        del blocker

    def _gripper_spin_changed(self, value):
        blocker = QSignalBlocker(self.gripper_slider)
        self.gripper_slider.setValue(value)
        del blocker

    def _queue_joint(self, joint_name):
        if self.playback_active:
            return
        self.pending_joints.add(joint_name)
        if not self.live_timer.isActive():
            self.live_timer.start()

    def _queue_gripper(self):
        if self.playback_active:
            return
        self.pending_gripper = True
        if not self.live_timer.isActive():
            self.live_timer.start()

    def _flush_live_command(self):
        self.flush_after_response = True
        self._dispatch_live_command(force=True)

    def _dispatch_live_command(self, force=False):
        if self.playback_active or self.live_request_in_flight:
            return
        if not force and time.monotonic() - self.last_live_send_time < LIVE_SEND_INTERVAL_MS / 1000.0:
            return
        if self.pending_joints:
            if not self.node.move_client.service_is_ready():
                self.status_label.setText('等待 /learm_driver/move_joints 服务...')
                return
            joint_names = sorted(self.pending_joints)
            self.pending_joints.clear()
            request = MoveJoints.Request()
            request.joint_names = joint_names
            request.positions_rad = [
                math.radians(self.angle_controls[name].value_degrees()) for name in joint_names]
            request.duration_ms = self.duration_spin.value()
            self.live_request_in_flight = True
            self.last_live_send_time = time.monotonic()
            self._update_controls()
            self.node.move_client.call_async(request).add_done_callback(self._live_arm_done)
            return
        if self.pending_gripper:
            if not self.node.gripper_client.service_is_ready():
                self.status_label.setText('等待 /learm_driver/set_gripper 服务...')
                return
            self.pending_gripper = False
            request = SetGripper.Request()
            request.opening = self.gripper_spin.value() / 100.0
            request.duration_ms = self.duration_spin.value()
            self.live_request_in_flight = True
            self.last_live_send_time = time.monotonic()
            self._update_controls()
            self.node.gripper_client.call_async(request).add_done_callback(self._live_gripper_done)
            return
        self.live_timer.stop()

    def _live_arm_done(self, future):
        self.live_request_in_flight = False
        self._update_controls()
        try:
            response = future.result()
        except Exception as exc:
            self.flush_after_response = False
            self.status_label.setText(f'实时关节命令失败: {exc}')
            return
        if not response.success:
            self.flush_after_response = False
            self.status_label.setText(
                f'实时关节命令被拒绝 ({response.error_code}): {response.message}')
            return
        self.status_label.setText('实时关节目标已被 STM32 接受。')
        force = self.flush_after_response
        self.flush_after_response = False
        self._dispatch_live_command(force=force)

    def _live_gripper_done(self, future):
        self.live_request_in_flight = False
        self._update_controls()
        try:
            response = future.result()
        except Exception as exc:
            self.flush_after_response = False
            self.status_label.setText(f'实时夹爪命令失败: {exc}')
            return
        if not response.success:
            self.flush_after_response = False
            self.status_label.setText(
                f'实时夹爪命令被拒绝 ({response.error_code}): {response.message}')
            return
        self.status_label.setText(f'实时夹爪目标已被 STM32 接受: {response.target_pulse_us} us。')
        force = self.flush_after_response
        self.flush_after_response = False
        self._dispatch_live_command(force=force)

    def _current_keyframe(self):
        return Keyframe(
            duration_ms=self.duration_spin.value(),
            opening=self.gripper_spin.value() / 100.0,
            positions_deg=[self.angle_controls[name].value_degrees() for name in JOINT_NAMES],
            execution_mode=self.execution_mode_combo.currentData(),
            gripper_timing=self.gripper_timing_combo.currentData(),
        )

    def _add_keyframe(self):
        if self.playback_active:
            return
        self.keyframes.append(self._current_keyframe())
        self._refresh_table(len(self.keyframes) - 1)
        self.status_label.setText(f'已添加第 {len(self.keyframes)} 个关键帧。')

    def _replace_selected(self):
        row = self.table.currentRow()
        if self.playback_active or row < 0:
            return
        self.keyframes[row] = self._current_keyframe()
        self._refresh_table(row)
        self.status_label.setText(f'已替换第 {row + 1} 个关键帧。')

    def _capture_status(self):
        if self.status_request_in_flight or self.status_poll_request_in_flight or \
          self.playback_active or self.live_request_in_flight or \
          self.pending_joints or self.pending_gripper:
            self.status_label.setText('请等待实时目标发送完成后再读取状态。')
            return
        if not self.node.status_client.service_is_ready():
            self.status_label.setText('等待 /learm_driver/get_status 服务...')
            return
        self.status_request_in_flight = True
        self._update_controls()
        self.node.status_client.call_async(GetArmStatus.Request()).add_done_callback(self._capture_done)

    def _capture_done(self, future):
        self.status_request_in_flight = False
        self._update_controls()
        try:
            response = future.result()
        except Exception as exc:
            self.status_label.setText(f'状态读取失败: {exc}')
            return
        if not response.success:
            self.status_label.setText(f'状态读取失败 ({response.error_code}): {response.message}')
            return

        current = [int(value) for value in response.current_pulse_us]
        opening = (GRIPPER_CLOSED_PULSE_US - current[0]) / (
            GRIPPER_CLOSED_PULSE_US - GRIPPER_OPEN_PULSE_US)
        positions_deg = []
        for index in range(1, len(JOINT_NAMES) + 1):
            degrees = ((current[index] - 1500.0) / 1000.0) * 90.0
            positions_deg.append(max(-JOINT_LIMIT_DEGREES, min(JOINT_LIMIT_DEGREES, degrees)))
        self._set_pose_controls(positions_deg, opening)
        moving = '运动中' if response.moving else '已停止'
        self.status_label.setText(f'已读取 STM32 PWM 估计姿态: {moving}。')
        self._update_live_angle_display(response)

    def _poll_status(self):
        if not rclpy.ok() or self.status_poll_request_in_flight or self.status_request_in_flight:
            return
        if self.live_request_in_flight or self.pending_joints or self.pending_gripper:
            return
        if not self.node.status_client.service_is_ready():
            self.live_angle_state_label.setText('等待 /learm_driver/get_status...')
            return
        self.status_poll_request_in_flight = True
        self.node.status_client.call_async(GetArmStatus.Request()).add_done_callback(self._poll_status_done)

    def _poll_status_done(self, future):
        self.status_poll_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.live_angle_state_label.setText(f'实时角度读取失败: {exc}')
            return
        if not response.success:
            self.live_angle_state_label.setText(
                f'实时角度读取失败 ({response.error_code}): {response.message}')
            return
        self._update_live_angle_display(response)

    def _update_live_angle_display(self, response):
        for index, servo_name in enumerate(SERVO_NAMES):
            degrees = pulse_to_estimated_degrees(response.current_pulse_us[index])
            target_degrees = pulse_to_estimated_degrees(response.target_pulse_us[index])
            self.live_angle_labels[servo_name].setText(
                f'{degrees:.1f} deg / 目标 {target_degrees:.1f} deg')
        moving = '运动中' if response.moving else '已停止'
        estop = '急停' if response.estop_active else '正常'
        self.live_angle_state_label.setText(f'{moving}，{estop}，PWM 估算值。')

    def _set_pose_controls(self, positions_deg, opening):
        gripper_slider_blocker = QSignalBlocker(self.gripper_slider)
        gripper_spin_blocker = QSignalBlocker(self.gripper_spin)
        self.gripper_slider.setValue(round(max(0.0, min(1.0, opening)) * 100.0))
        self.gripper_spin.setValue(self.gripper_slider.value())
        del gripper_slider_blocker
        del gripper_spin_blocker
        for joint_name, degrees in zip(JOINT_NAMES, positions_deg):
            self.angle_controls[joint_name].set_value_degrees(degrees)

    def _emergency_stop(self):
        if not self.node.estop_client.service_is_ready():
            self.status_label.setText('等待 /learm_driver/emergency_stop 服务...')
            return
        self.playback_generation += 1
        self.playback_active = False
        self.reset_active = False
        self.playback_stages = []
        self.playback_stage_index = 0
        self.pending_joints.clear()
        self.pending_gripper = False
        self.live_timer.stop()
        self._update_controls()
        self.node.estop_client.call_async(Trigger.Request()).add_done_callback(self._estop_done)

    def _estop_done(self, future):
        try:
            response = future.result()
            self.status_label.setText(response.message)
        except Exception as exc:
            self.status_label.setText(f'急停服务失败: {exc}')

    def _play_all(self):
        if not self.keyframes or not self._can_start_staged_motion():
            return
        self.playback_index = 0
        self.playback_active = True
        self.reset_active = False
        self.playback_generation += 1
        self._update_controls()
        self._play_next_keyframe(self.playback_generation, False)

    def _play_selected(self):
        row = self.table.currentRow()
        if row < 0 or not self._can_start_staged_motion():
            return
        self.playback_index = row
        self.playback_active = True
        self.reset_active = False
        self.playback_generation += 1
        self._update_controls()
        self._play_next_keyframe(self.playback_generation, True)

    def _reset_to_zero(self):
        if not self._can_start_staged_motion():
            return
        duration_ms = RESET_DURATION_MS
        self.playback_active = True
        self.reset_active = True
        self.playback_generation += 1
        self.playback_stages = [
            MotionStage('joints', '底座复位', duration_ms, ('joint_6',), (0.0,)),
            MotionStage('joints', '肩肘复位', duration_ms, ('joint_5', 'joint_4'), (0.0, 0.0)),
            MotionStage('joints', '腕部复位', duration_ms, ('joint_3', 'joint_2'), (0.0, 0.0)),
            MotionStage('gripper', '夹爪闭合', duration_ms, opening=0.0),
        ]
        self.playback_stage_index = 0
        self._update_controls()
        self._play_current_stage(self.playback_generation)

    def _can_start_staged_motion(self):
        if self.playback_active:
            return False
        if self.status_request_in_flight or self.live_request_in_flight or \
          self.pending_joints or self.pending_gripper:
            self.status_label.setText('请等待实时目标发送完成后再执行动作组或复位。')
            return False
        return True

    def _build_keyframe_stages(self, keyframe):
        if keyframe.execution_mode == EXECUTION_MODE_SYNCHRONIZED:
            return [MotionStage(
                'pose', '同步姿态', keyframe.duration_ms,
                positions_deg=tuple(keyframe.positions_deg), opening=keyframe.opening)]

        positions = dict(zip(JOINT_NAMES, keyframe.positions_deg))
        stages = []
        if keyframe.gripper_timing == GRIPPER_TIMING_BEFORE:
            stages.append(MotionStage(
                'gripper', '夹爪前执行', keyframe.duration_ms, opening=keyframe.opening))
        stages.extend([
            MotionStage('joints', '底座', keyframe.duration_ms, ('joint_6',), (positions['joint_6'],)),
            MotionStage(
                'joints', '肩部和肘部', keyframe.duration_ms,
                ('joint_5', 'joint_4'), (positions['joint_5'], positions['joint_4'])),
            MotionStage(
                'joints', '腕部', keyframe.duration_ms,
                ('joint_3', 'joint_2'), (positions['joint_3'], positions['joint_2'])),
        ])
        if keyframe.gripper_timing == GRIPPER_TIMING_AFTER:
            stages.append(MotionStage(
                'gripper', '夹爪后执行', keyframe.duration_ms, opening=keyframe.opening))
        return stages

    def _play_next_keyframe(self, generation, stop_after_one):
        if generation != self.playback_generation or not self.playback_active:
            return
        if self.playback_index >= len(self.keyframes):
            self._finish_staged_motion('动作组回放完成。')
            return

        keyframe = self.keyframes[self.playback_index]
        self.table.selectRow(self.playback_index)
        self.playback_stages = self._build_keyframe_stages(keyframe)
        self.playback_stage_index = 0
        self.playback_stop_after_one = stop_after_one
        self._play_current_stage(generation)

    def _play_current_stage(self, generation):
        if generation != self.playback_generation or not self.playback_active:
            return
        if self.playback_stage_index >= len(self.playback_stages):
            self._advance_after_keyframe(generation)
            return

        stage = self.playback_stages[self.playback_stage_index]
        self.status_label.setText(f'执行阶段：{stage.label}。')
        if stage.kind == 'pose':
            if not self.node.pose_client.service_is_ready():
                self._playback_failed('等待 /learm_driver/move_pose 服务...')
                return
            request = MovePose.Request()
            request.opening = stage.opening
            request.positions_rad = [math.radians(value) for value in stage.positions_deg]
            request.duration_ms = stage.duration_ms
            future = self.node.pose_client.call_async(request)
        elif stage.kind == 'joints':
            if not self.node.move_client.service_is_ready():
                self._playback_failed('等待 /learm_driver/move_joints 服务...')
                return
            request = MoveJoints.Request()
            request.joint_names = list(stage.joint_names)
            request.positions_rad = [math.radians(value) for value in stage.positions_deg]
            request.duration_ms = stage.duration_ms
            future = self.node.move_client.call_async(request)
        else:
            if not self.node.gripper_client.service_is_ready():
                self._playback_failed('等待 /learm_driver/set_gripper 服务...')
                return
            request = SetGripper.Request()
            request.opening = stage.opening
            request.duration_ms = stage.duration_ms
            future = self.node.gripper_client.call_async(request)
        future.add_done_callback(lambda result: self._play_stage_done(result, generation, stage))

    def _play_stage_done(self, future, generation, stage):
        if generation != self.playback_generation or not self.playback_active:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._playback_failed(f'{stage.label} 命令失败: {exc}')
            return
        if not response.success:
            self._playback_failed(f'{stage.label} 被拒绝 ({response.error_code}): {response.message}')
            return

        def advance():
            if generation != self.playback_generation or not self.playback_active:
                return
            self.playback_stage_index += 1
            self._play_current_stage(generation)

        QTimer.singleShot(stage.duration_ms + 20, advance)

    def _advance_after_keyframe(self, generation):
        if self.reset_active:
            self._set_pose_controls([0.0] * len(JOINT_NAMES), 0.0)
            self._finish_staged_motion('复位完成：joint_2 至 joint_6 已回到零位，夹爪已闭合。')
            return

        self.playback_index += 1
        if self.playback_stop_after_one:
            self._finish_staged_motion('所选关键帧回放完成。')
            return
        self._play_next_keyframe(generation, False)

    def _finish_staged_motion(self, message):
        self.playback_active = False
        self.reset_active = False
        self.playback_stages = []
        self.playback_stage_index = 0
        self._update_controls()
        self.status_label.setText(message)

    def _playback_failed(self, message):
        self._finish_staged_motion(message)

    def _refresh_table(self, selected_row=None):
        self.table.setRowCount(len(self.keyframes))
        for row, keyframe in enumerate(self.keyframes):
            mode = '同步' if keyframe.execution_mode == EXECUTION_MODE_SYNCHRONIZED else '安全顺序'
            gripper_timing = {
                GRIPPER_TIMING_BEFORE: '前执行',
                GRIPPER_TIMING_AFTER: '后执行',
                GRIPPER_TIMING_NONE: '不动作',
            }[keyframe.gripper_timing]
            values = [
                str(row + 1),
                f'{keyframe.duration_ms} ms',
                mode,
                gripper_timing,
                f'{keyframe.opening * 100.0:.0f} %',
                *[f'{value:.1f} deg' for value in keyframe.positions_deg],
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
        if selected_row is not None and selected_row >= 0:
            self.table.selectRow(selected_row)
        self._update_controls()

    def _delete_selected(self):
        row = self.table.currentRow()
        if self.playback_active or row < 0:
            return
        del self.keyframes[row]
        self._refresh_table(min(row, len(self.keyframes) - 1))

    def _move_selected(self, direction):
        row = self.table.currentRow()
        destination = row + direction
        if self.playback_active or row < 0 or destination < 0 or destination >= len(self.keyframes):
            return
        self.keyframes[row], self.keyframes[destination] = (
            self.keyframes[destination], self.keyframes[row])
        self._refresh_table(destination)

    def _clear_keyframes(self):
        if self.playback_active or not self.keyframes:
            return
        answer = QMessageBox.question(
            self, '清空动作组', '确认清空全部关键帧？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self.keyframes.clear()
            self._refresh_table()

    def _save_recording(self):
        default_path = Path.home() / 'learm_action_group.json'
        filename, _ = QFileDialog.getSaveFileName(
            self, '保存动作组', str(default_path), 'JSON (*.json)')
        if not filename:
            return
        path = Path(filename)
        if path.suffix.lower() != '.json':
            path = path.with_suffix('.json')
        document = {
            'format_version': RECORDING_FORMAT_VERSION,
            'joint_names': list(JOINT_NAMES),
            'keyframes': [asdict(keyframe) for keyframe in self.keyframes],
        }
        try:
            path.write_text(json.dumps(document, indent=2), encoding='utf-8')
        except OSError as exc:
            self.status_label.setText(f'保存失败: {exc}')
            return
        self.status_label.setText(f'已保存 {len(self.keyframes)} 个关键帧到 {path.name}。')

    def _load_recording(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, '加载动作组', str(Path.home()), 'JSON (*.json)')
        if not filename:
            return
        try:
            document = json.loads(Path(filename).read_text(encoding='utf-8'))
            format_version = document.get('format_version')
            if format_version not in (1, RECORDING_FORMAT_VERSION) or \
              document.get('joint_names') != list(JOINT_NAMES):
                raise ValueError('动作文件与当前关节配置不兼容')
            keyframes = []
            for item in document.get('keyframes', []):
                keyframe = Keyframe(
                    duration_ms=int(item['duration_ms']),
                    opening=float(item['opening']),
                    positions_deg=[float(value) for value in item['positions_deg']],
                    execution_mode=item.get('execution_mode', EXECUTION_MODE_SYNCHRONIZED),
                    gripper_timing=item.get('gripper_timing', GRIPPER_TIMING_AFTER),
                )
                if keyframe.duration_ms < MIN_DURATION_MS or keyframe.duration_ms > MAX_DURATION_MS or \
                  not 0.0 <= keyframe.opening <= 1.0 or \
                  len(keyframe.positions_deg) != len(JOINT_NAMES) or \
                  any(abs(value) > JOINT_LIMIT_DEGREES for value in keyframe.positions_deg) or \
                  keyframe.execution_mode not in (EXECUTION_MODE_SYNCHRONIZED, EXECUTION_MODE_SEQUENCED) or \
                  keyframe.gripper_timing not in (
                      GRIPPER_TIMING_BEFORE, GRIPPER_TIMING_AFTER, GRIPPER_TIMING_NONE):
                    raise ValueError('动作文件含有超出当前安全范围的关键帧')
                keyframes.append(keyframe)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.status_label.setText(f'加载失败: {exc}')
            return
        self.keyframes = keyframes
        self._refresh_table(0 if self.keyframes else None)
        self.status_label.setText(f'已加载 {len(self.keyframes)} 个关键帧。')

    def _update_controls(self):
        selected = self.table.currentRow()
        editable = not self.playback_active
        for control in self.angle_controls.values():
            control.setEnabled(editable)
        self.gripper_slider.setEnabled(editable)
        self.gripper_spin.setEnabled(editable)
        self.duration_slider.setEnabled(editable)
        self.duration_spin.setEnabled(editable)
        self.execution_mode_combo.setEnabled(editable)
        self.gripper_timing_combo.setEnabled(editable)
        self.add_button.setEnabled(editable)
        self.reset_button.setEnabled(
            editable and not self.status_request_in_flight and not self.live_request_in_flight and
            not self.pending_joints and not self.pending_gripper)
        self.capture_button.setEnabled(
            editable and not self.live_request_in_flight and not self.status_poll_request_in_flight)
        self.replace_button.setEnabled(editable and selected >= 0)
        self.delete_button.setEnabled(editable and selected >= 0)
        self.move_up_button.setEnabled(editable and selected > 0)
        self.move_down_button.setEnabled(editable and 0 <= selected < len(self.keyframes) - 1)
        self.play_all_button.setEnabled(editable and bool(self.keyframes) and not self.live_request_in_flight)
        self.play_selected_button.setEnabled(editable and selected >= 0 and not self.live_request_in_flight)
        self.clear_button.setEnabled(editable and bool(self.keyframes))
        self.save_button.setEnabled(editable and bool(self.keyframes))
        self.load_button.setEnabled(editable)

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.live_timer.stop()
        self.status_poll_timer.stop()
        event.accept()


def main():
    rclpy.init()
    node = RecorderClient()
    application = QApplication(sys.argv)
    window = MotionRecorderWindow(node)
    window.show()
    exit_code = application.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
