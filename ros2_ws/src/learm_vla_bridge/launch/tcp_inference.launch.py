import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def default_windows_host():
    try:
        fields = subprocess.check_output(
            ['ip', 'route', 'show', 'default'], text=True, timeout=2).split()
        return fields[fields.index('via') + 1]
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return '127.0.0.1'


def generate_launch_description():
    windows_host = LaunchConfiguration('windows_host')
    serial_config = PathJoinSubstitution([
        FindPackageShare('arm_serial_control'), 'config', 'serial_controller.yaml'])
    driver_config = PathJoinSubstitution([
        FindPackageShare('learm_driver'), 'config', 'learm_driver.yaml'])
    camera_config = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'), 'config', 'network_camera.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('windows_host', default_value=default_windows_host()),
        DeclareLaunchArgument('serial_tcp_port', default_value='8766'),
        DeclareLaunchArgument('camera_tcp_port', default_value='8767'),
        Node(
            package='arm_serial_control', executable='serial_controller',
            name='serial_controller', output='screen',
            parameters=[serial_config, {
                'transport': 'tcp',
                'tcp_host': windows_host,
                'tcp_port': ParameterValue(
                    LaunchConfiguration('serial_tcp_port'), value_type=int),
            }],
        ),
        Node(
            package='learm_driver', executable='learm_driver_node',
            name='learm_driver', output='screen', parameters=[driver_config],
        ),
        Node(
            package='learm_vla_bridge', executable='network_camera_node',
            name='learm_network_camera', output='screen',
            parameters=[camera_config, {
                'host': windows_host,
                'port': ParameterValue(
                    LaunchConfiguration('camera_tcp_port'), value_type=int),
            }],
        ),
    ])
