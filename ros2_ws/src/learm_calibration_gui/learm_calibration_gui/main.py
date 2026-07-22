import sys
import time

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node

from learm_driver.srv import CalibrationMovePwm, GetArmStatus
from std_srvs.srv import Trigger


JOINT_LIMITS = {
    'joint_1': (500, 1500),
    'joint_2': (500, 2500),
    'joint_3': (500, 2500),
    'joint_4': (500, 2500),
    'joint_5': (500, 2500),
    'joint_6': (500, 2500),
}


class CalibrationClient(Node):
    def __init__(self):
        super().__init__('learm_calibration_gui')
        self.move_client = self.create_client(
            CalibrationMovePwm, '/learm_driver/calibration_move_pwm')
        self.status_client = self.create_client(
            GetArmStatus, '/learm_driver/get_status')
        self.estop_client = self.create_client(
            Trigger, '/learm_driver/emergency_stop')
        self.clear_estop_client = self.create_client(
            Trigger, '/learm_driver/clear_estop')


class JointControl(QWidget):
    def __init__(self, joint_name, minimum, maximum, radio_group, selected):
        super().__init__()
        self.joint_name = joint_name
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        title = QHBoxLayout()
        self.radio = QRadioButton(joint_name.replace('_', ' ').title())
        self.radio.setChecked(selected)
        radio_group.addButton(self.radio)
        self.status = QLabel('当前: -- us | 目标: -- us')
        title.addWidget(self.radio)
        title.addStretch()
        title.addWidget(self.status)
        layout.addLayout(title)

        controls = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setValue((minimum + maximum) // 2)
        self.spin = QSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(1)
        self.spin.setSuffix(' us')
        self.spin.setValue(self.slider.value())
        self.range_label = QLabel(f'范围: {minimum}-{maximum} us')
        controls.addWidget(self.slider, 1)
        controls.addWidget(self.spin)
        controls.addWidget(self.range_label)
        layout.addLayout(controls)

        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)

    def set_status(self, current, target):
        self.status.setText(f'当前: {current} us | 目标: {target} us')


class CalibrationWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.controls = {}
        self.status_request_in_flight = False
        self.initial_status_received = False
        self.motion_until = 0.0

        self.setWindowTitle('LeArm PWM 舵机标定')
        self.setMinimumSize(900, 620)
        self._build_ui()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(20)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.request_status)
        self.status_timer.start(700)
        self.request_status()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        self.radio_group = QButtonGroup(self)
        servo_box = QGroupBox('舵机 PWM')
        servo_layout = QGridLayout(servo_box)
        for index, (joint_name, limits) in enumerate(JOINT_LIMITS.items()):
            control = JointControl(
                joint_name, limits[0], limits[1], self.radio_group, index == 0)
            self.controls[joint_name] = control
            servo_layout.addWidget(control, index // 2, index % 2)
        layout.addWidget(servo_box)

        move_box = QGroupBox('单关节运动')
        move_layout = QFormLayout(move_box)
        time_controls = QHBoxLayout()
        self.duration_slider = QSlider(Qt.Horizontal)
        self.duration_slider.setRange(200, 1000)
        self.duration_slider.setSingleStep(1)
        self.duration_slider.setPageStep(100)
        self.duration_slider.setValue(500)
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(200, 1000)
        self.duration_spin.setSingleStep(10)
        self.duration_spin.setSuffix(' ms')
        self.duration_spin.setValue(500)
        self.duration_slider.valueChanged.connect(self.duration_spin.setValue)
        self.duration_spin.valueChanged.connect(self.duration_slider.setValue)
        time_controls.addWidget(self.duration_slider, 1)
        time_controls.addWidget(self.duration_spin)
        move_layout.addRow('动作时间', time_controls)

        buttons = QHBoxLayout()
        self.send_button = QPushButton('发送所选关节')
        self.refresh_button = QPushButton('刷新状态')
        self.estop_button = QPushButton('软件急停')
        self.clear_estop_button = QPushButton('解除急停')
        buttons.addWidget(self.send_button)
        buttons.addWidget(self.refresh_button)
        buttons.addStretch()
        buttons.addWidget(self.estop_button)
        buttons.addWidget(self.clear_estop_button)
        move_layout.addRow(buttons)
        layout.addWidget(move_box)

        self.connection_label = QLabel('等待 /learm_driver 服务...')
        self.connection_label.setWordWrap(True)
        layout.addWidget(self.connection_label)

        self.send_button.clicked.connect(self.send_selected_joint)
        self.refresh_button.clicked.connect(self.request_status)
        self.estop_button.clicked.connect(self.emergency_stop)
        self.clear_estop_button.clicked.connect(self.clear_estop)

    def _spin_ros(self):
        if not rclpy.ok():
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.motion_until and time.monotonic() >= self.motion_until:
            self.motion_until = 0.0
            self.send_button.setEnabled(True)

    def selected_joint(self):
        for joint_name, control in self.controls.items():
            if control.radio.isChecked():
                return joint_name, control
        return None, None

    def request_status(self):
        if not rclpy.ok():
            return
        if self.status_request_in_flight:
            return
        if not self.node.status_client.service_is_ready():
            self.connection_label.setText('等待 /learm_driver/get_status...')
            return
        self.status_request_in_flight = True
        future = self.node.status_client.call_async(GetArmStatus.Request())
        future.add_done_callback(self._status_done)

    def _status_done(self, future):
        self.status_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.connection_label.setText(f'状态请求失败: {exc}')
            return
        if not response.success:
            self.connection_label.setText(f'状态错误 {response.error_code}: {response.message}')
            return

        for index, control in enumerate(self.controls.values()):
            current = int(response.current_pulse_us[index])
            target = int(response.target_pulse_us[index])
            control.set_status(current, target)
            if not self.initial_status_received:
                control.spin.setValue(max(control.spin.minimum(), min(target, control.spin.maximum())))
        self.initial_status_received = True
        moving = '运动中' if response.moving else '已就绪'
        estop = '急停已触发' if response.estop_active else '急停已解除'
        self.connection_label.setText(f'STM32 已连接: {moving}，{estop}。{response.message}')
        if not response.moving:
            self.motion_until = 0.0
            self.send_button.setEnabled(True)

    def send_selected_joint(self):
        joint_name, control = self.selected_joint()
        if joint_name is None:
            return
        if not self.node.move_client.service_is_ready():
            self.connection_label.setText('等待 /learm_driver/calibration_move_pwm...')
            return
        target = control.spin.value()
        duration = self.duration_spin.value()
        answer = QMessageBox.question(
            self,
            '确认舵机运动',
            f'将 {joint_name} 运动到 {target} us，动作时间 {duration} ms？',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return

        request = CalibrationMovePwm.Request()
        request.joint_name = joint_name
        request.target_pulse_us = target
        request.duration_ms = duration
        self.send_button.setEnabled(False)
        future = self.node.move_client.call_async(request)
        future.add_done_callback(lambda result: self._move_done(result, duration))

    def _move_done(self, future, duration):
        try:
            response = future.result()
        except Exception as exc:
            self.connection_label.setText(f'运动请求失败: {exc}')
            self.send_button.setEnabled(True)
            return
        if response.success:
            self.motion_until = time.monotonic() + (duration / 1000.0)
            self.connection_label.setText(
                f'动作已接受，发送前当前 PWM 为 {response.current_pulse_us} us，等待完成。')
        else:
            self.connection_label.setText(f'动作被拒绝 ({response.error_code}): {response.message}')
            self.send_button.setEnabled(True)

    def emergency_stop(self):
        if not self.node.estop_client.service_is_ready():
            self.connection_label.setText('等待 /learm_driver/emergency_stop...')
            return
        self.node.estop_client.call_async(Trigger.Request()).add_done_callback(self._trigger_done)
        self.send_button.setEnabled(False)

    def clear_estop(self):
        if not self.node.clear_estop_client.service_is_ready():
            self.connection_label.setText('等待 /learm_driver/clear_estop...')
            return
        self.node.clear_estop_client.call_async(Trigger.Request()).add_done_callback(self._trigger_done)

    def _trigger_done(self, future):
        try:
            response = future.result()
            self.connection_label.setText(response.message)
        except Exception as exc:
            self.connection_label.setText(f'急停服务失败: {exc}')
        self.request_status()

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.status_timer.stop()
        event.accept()


def main():
    rclpy.init()
    node = CalibrationClient()
    application = QApplication(sys.argv)
    window = CalibrationWindow(node)
    window.show()
    exit_code = application.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
