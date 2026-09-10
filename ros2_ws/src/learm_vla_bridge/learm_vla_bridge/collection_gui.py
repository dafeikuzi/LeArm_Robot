import shlex
import sys
from pathlib import Path

from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


DEFAULT_WORKSPACE = Path('/home/liuzhiwei/LeArm_Robot/ros2_ws')
DEFAULT_DATASET_ROOT = Path('/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw')
BUILD_PACKAGES = ('learm_driver', 'learm_motion_recorder', 'learm_vla_bridge')


class VlaCollectionWindow(QMainWindow):
    """One-window launcher for a manually demonstrated VLA episode."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('LeArm VLA 数据采集')
        self.setMinimumSize(900, 680)

        self.driver_process = None
        self.control_process = None
        self.vla_process = None
        self.build_process = None
        self.service_process = None
        self.service_output = ''
        self.service_action = None
        self.recording_active = False

        self._build_ui()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        settings_box = QGroupBox('采集配置')
        settings = QFormLayout(settings_box)
        self.workspace_edit = QLineEdit(str(DEFAULT_WORKSPACE))
        self.serial_port_edit = QLineEdit('/dev/ttyUSB0')
        self.camera_device_edit = QLineEdit('/dev/video0')
        self.task_edit = QLineEdit('抓取棕色小狗')

        dataset_row = QHBoxLayout()
        self.dataset_root_edit = QLineEdit(str(DEFAULT_DATASET_ROOT))
        select_dataset_button = QPushButton('选择目录')
        select_dataset_button.clicked.connect(self._select_dataset_root)
        dataset_row.addWidget(self.dataset_root_edit, 1)
        dataset_row.addWidget(select_dataset_button)

        self.camera_fps_spin = self._rate_spin(30)
        self.publish_rate_spin = self._rate_spin(15.0)
        self.status_rate_spin = self._rate_spin(15.0)
        self.record_rate_spin = self._rate_spin(15.0)

        settings.addRow('ROS 2 工作区', self.workspace_edit)
        settings.addRow('机械臂串口', self.serial_port_edit)
        settings.addRow('摄像头设备', self.camera_device_edit)
        settings.addRow('任务描述', self.task_edit)
        settings.addRow('数据目录', dataset_row)
        settings.addRow('摄像头帧率', self.camera_fps_spin)
        settings.addRow('观察发布频率', self.publish_rate_spin)
        settings.addRow('STM32 状态频率', self.status_rate_spin)
        settings.addRow('记录频率', self.record_rate_spin)
        layout.addWidget(settings_box)

        process_box = QGroupBox('启动流程')
        process_layout = QGridLayout(process_box)
        self.build_button = QPushButton('1. 构建工作区')
        self.start_driver_button = QPushButton('2. 启动机械臂驱动')
        self.open_control_button = QPushButton('3. 打开控制 GUI')
        self.start_vla_button = QPushButton('4. 启动观察和记录节点')
        self.shutdown_button = QPushButton('关闭本窗口启动的节点')

        self.build_button.clicked.connect(self._build_workspace)
        self.start_driver_button.clicked.connect(self._start_driver)
        self.open_control_button.clicked.connect(self._open_control_gui)
        self.start_vla_button.clicked.connect(self._start_vla_nodes)
        self.shutdown_button.clicked.connect(self._shutdown_processes)

        process_layout.addWidget(self.build_button, 0, 0)
        process_layout.addWidget(self.start_driver_button, 0, 1)
        process_layout.addWidget(self.open_control_button, 1, 0)
        process_layout.addWidget(self.start_vla_button, 1, 1)
        process_layout.addWidget(self.shutdown_button, 2, 0, 1, 2)
        layout.addWidget(process_box)

        episode_box = QGroupBox('Episode 记录')
        episode_layout = QHBoxLayout(episode_box)
        self.start_episode_button = QPushButton('开始记录')
        self.finish_episode_button = QPushButton('成功结束')
        self.abort_episode_button = QPushButton('放弃记录')
        self.start_episode_button.clicked.connect(lambda: self._call_episode_service('start_episode'))
        self.finish_episode_button.clicked.connect(lambda: self._call_episode_service('stop_episode'))
        self.abort_episode_button.clicked.connect(lambda: self._call_episode_service('abort_episode'))
        self.start_episode_button.setEnabled(False)
        self.finish_episode_button.setEnabled(False)
        self.abort_episode_button.setEnabled(False)
        episode_layout.addWidget(self.start_episode_button)
        episode_layout.addWidget(self.finish_episode_button)
        episode_layout.addWidget(self.abort_episode_button)
        layout.addWidget(episode_box)

        self.process_status = QLabel('进程状态：未启动')
        self.episode_status = QLabel('记录状态：未开始')
        self.process_status.setWordWrap(True)
        self.episode_status.setWordWrap(True)
        layout.addWidget(self.process_status)
        layout.addWidget(self.episode_status)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setPlaceholderText('进程输出将显示在这里。')
        layout.addWidget(self.log, 1)

    @staticmethod
    def _rate_spin(value):
        spin = QDoubleSpinBox()
        spin.setRange(0.1, 30.0)
        spin.setDecimals(1)
        spin.setSingleStep(1.0)
        spin.setValue(value)
        spin.setSuffix(' Hz')
        return spin

    def _select_dataset_root(self):
        selected = QFileDialog.getExistingDirectory(
            self,
            '选择 VLA 数据目录',
            self.dataset_root_edit.text(),
        )
        if selected:
            self.dataset_root_edit.setText(selected)

    def _workspace(self):
        return Path(self.workspace_edit.text()).expanduser().resolve()

    def _validate_workspace(self):
        workspace = self._workspace()
        if not (workspace / 'src').is_dir():
            self._set_process_status(f'工作区无效：{workspace}')
            return None
        if not (workspace / 'install' / 'setup.bash').is_file():
            self._set_process_status('未找到 install/setup.bash，请先点击“构建工作区”。')
            return None
        return workspace

    def _validate_collection_settings(self):
        workspace = self._validate_workspace()
        if workspace is None:
            return None
        task = self.task_edit.text().strip()
        if not task:
            self._set_episode_status('任务描述不能为空。')
            return None
        if not self.serial_port_edit.text().strip() or not self.camera_device_edit.text().strip():
            self._set_process_status('串口设备和摄像头设备不能为空。')
            return None
        return workspace, task

    def _build_workspace(self):
        workspace = self._workspace()
        if not (workspace / 'src').is_dir():
            self._set_process_status(f'工作区无效：{workspace}')
            return
        if self._is_running(self.build_process):
            self._set_process_status('工作区正在构建。')
            return

        command = (
            'source /opt/ros/jazzy/setup.bash && '
            f'cd {shlex.quote(str(workspace))} && '
            'colcon build --packages-select ' + ' '.join(BUILD_PACKAGES) +
            ' --symlink-install --allow-overriding learm_driver'
        )
        self.build_process = self._start_process('构建', command)
        self._set_process_status('工作区正在构建。')

    def _start_driver(self):
        settings = self._validate_collection_settings()
        if settings is None:
            return
        if self._is_running(self.driver_process):
            self._set_process_status('机械臂驱动已经由本窗口启动。')
            return

        workspace, _task = settings
        command = self._ros_command(
            workspace,
            [
                'ros2', 'launch', 'learm_driver', 'learm_bringup.launch.py',
                f'serial_port:={self.serial_port_edit.text().strip()}',
            ],
        )
        self.driver_process = self._start_process('机械臂驱动', command)
        self._set_process_status('机械臂驱动正在启动。')

    def _open_control_gui(self):
        workspace = self._validate_workspace()
        if workspace is None:
            return
        if self._is_running(self.control_process):
            self._set_process_status('机械臂控制 GUI 已打开。')
            return

        command = self._ros_command(
            workspace,
            ['ros2', 'run', 'learm_motion_recorder', 'learm_motion_recorder'],
        )
        self.control_process = self._start_process('机械臂控制 GUI', command)
        self._set_process_status('机械臂控制 GUI 正在打开。')

    def _start_vla_nodes(self):
        settings = self._validate_collection_settings()
        if settings is None:
            return
        if self._is_running(self.vla_process):
            self._set_process_status('VLA 观察和记录节点已经由本窗口启动。')
            self.start_episode_button.setEnabled(not self.recording_active)
            return

        workspace, task = settings
        command = self._ros_command(
            workspace,
            [
                'ros2', 'launch', 'learm_vla_bridge', 'recording.launch.py',
                f'camera_device:={self.camera_device_edit.text().strip()}',
                f'dataset_root:={self.dataset_root_edit.text().strip()}',
                f'camera_fps:={int(self.camera_fps_spin.value())}',
                f'publish_rate_hz:={self.publish_rate_spin.value():.1f}',
                f'status_poll_rate_hz:={self.status_rate_spin.value():.1f}',
                f'record_rate_hz:={self.record_rate_spin.value():.1f}',
                f'task:={task}',
            ],
        )
        self.vla_process = self._start_process('VLA 观察和记录', command)
        self._set_process_status('VLA 观察和记录节点正在启动。')
        QTimer.singleShot(1500, self._enable_episode_start_if_available)

    def _enable_episode_start_if_available(self):
        if self._is_running(self.vla_process) and not self.recording_active:
            self.start_episode_button.setEnabled(True)
            self._set_episode_status('记录器已启动。确认图像和机械臂状态正常后，点击“开始记录”。')

    def _call_episode_service(self, action):
        if not self._is_running(self.vla_process):
            self._set_episode_status('请先启动 VLA 观察和记录节点。')
            return
        if self._is_running(self.service_process):
            self._set_episode_status('正在等待上一条记录服务响应。')
            return

        workspace = self._validate_workspace()
        if workspace is None:
            return
        self.service_action = action
        self.service_output = ''
        self.start_episode_button.setEnabled(False)
        self.finish_episode_button.setEnabled(False)
        self.abort_episode_button.setEnabled(False)
        command = self._ros_command(
            workspace,
            ['ros2', 'service', 'call', f'/learm_vla_recorder/{action}', 'std_srvs/srv/Trigger', '{}'],
        )
        self.service_process = self._start_process('记录服务', command, self._handle_service_finished)
        self._set_episode_status('正在调用记录服务。')

    def _handle_service_finished(self, exit_code, _exit_status):
        self._read_process_output('记录服务', self.service_process)
        succeeded = self._service_succeeded(exit_code, self.service_output)
        action = self.service_action
        if action == 'start_episode' and succeeded:
            self.recording_active = True
            self.finish_episode_button.setEnabled(True)
            self.abort_episode_button.setEnabled(True)
            self._set_episode_status('正在记录。请使用机械臂控制 GUI 完成抓取。')
        elif action == 'stop_episode' and succeeded:
            self.recording_active = False
            self.start_episode_button.setEnabled(True)
            self._set_episode_status('记录成功结束。请检查 metadata.json 中 outcome 是否为 success。')
        elif action == 'abort_episode' and succeeded:
            self.recording_active = False
            self.start_episode_button.setEnabled(True)
            self._set_episode_status('当前 episode 已放弃，不会作为成功样本使用。')
        else:
            self.start_episode_button.setEnabled(self._is_running(self.vla_process) and not self.recording_active)
            self.finish_episode_button.setEnabled(self.recording_active)
            self.abort_episode_button.setEnabled(self.recording_active)
            self._set_episode_status('记录服务未成功，请查看日志并确认图像、STM32 状态和任务描述。')

    def _ros_command(self, workspace, arguments):
        setup_file = workspace / 'install' / 'setup.bash'
        return (
            'source /opt/ros/jazzy/setup.bash && '
            f'source {shlex.quote(str(setup_file))} && '
            f'exec {shlex.join(arguments)}'
        )

    def _start_process(self, label, command, finished_callback=None):
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda name=label, current=process: self._read_process_output(name, current))
        process.errorOccurred.connect(
            lambda _error, name=label: self._append_log(f'[{name}] 无法启动或运行异常。'))
        if finished_callback is not None:
            process.finished.connect(finished_callback)
        else:
            process.finished.connect(
                lambda code, _status, name=label: self._handle_long_process_finished(name, code))
        self._append_log(f'$ {command}')
        process.start('/bin/bash', ['-lc', command])
        return process

    def _read_process_output(self, label, process):
        output = bytes(process.readAllStandardOutput()).decode(errors='replace')
        if not output:
            return
        if process is self.service_process:
            self.service_output += output
        for line in output.rstrip().splitlines():
            self._append_log(f'[{label}] {line}')
            if process is self.vla_process:
                normalized = line.lower()
                if 'status request failed' in normalized or 'cannot read arm status' in normalized:
                    self._set_process_status('STM32 状态查询出现告警，请查看日志；当前记录继续使用最近状态。')

    def _handle_long_process_finished(self, label, exit_code):
        self._append_log(f'[{label}] 已退出，返回码 {exit_code}。')
        if label == 'VLA 观察和记录':
            self.recording_active = False
            self.start_episode_button.setEnabled(False)
            self.finish_episode_button.setEnabled(False)
            self.abort_episode_button.setEnabled(False)
            self._set_episode_status('记录器已退出。未调用“成功结束”的 episode 会被标记为 interrupted。')

    @staticmethod
    def _is_running(process):
        return process is not None and process.state() != QProcess.NotRunning

    @staticmethod
    def _service_succeeded(exit_code, output):
        normalized_output = output.lower()
        return exit_code == 0 and (
            'success: true' in normalized_output or 'success=true' in normalized_output)

    def _shutdown_processes(self):
        if self.recording_active:
            self._set_episode_status('请先点击“成功结束”或“放弃记录”，再关闭节点。')
            return
        self._stop_process(self.vla_process, 'VLA 观察和记录')
        self._stop_process(self.control_process, '机械臂控制 GUI')
        self._stop_process(self.driver_process, '机械臂驱动')
        self.start_episode_button.setEnabled(False)
        self.finish_episode_button.setEnabled(False)
        self.abort_episode_button.setEnabled(False)
        self._set_process_status('已请求关闭本窗口启动的节点。')

    def _stop_process(self, process, label):
        if not self._is_running(process):
            return
        self._append_log(f'[{label}] 正在关闭。')
        process.terminate()
        QTimer.singleShot(2000, lambda current=process: self._kill_if_running(current))

    @staticmethod
    def _kill_if_running(process):
        if process is not None and process.state() != QProcess.NotRunning:
            process.kill()

    def _set_process_status(self, text):
        self.process_status.setText(f'进程状态：{text}')

    def _set_episode_status(self, text):
        self.episode_status.setText(f'记录状态：{text}')

    def _append_log(self, text):
        self.log.appendPlainText(text)

    def closeEvent(self, event):
        if self.recording_active:
            QMessageBox.warning(self, '仍在记录', '请先成功结束或放弃当前 episode。')
            event.ignore()
            return
        self._shutdown_processes()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = VlaCollectionWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
