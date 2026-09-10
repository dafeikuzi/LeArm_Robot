import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def default_windows_host():
    try:
        fields = subprocess.check_output(
            ['ip', 'route', 'show', 'default'], text=True, timeout=2).split()
        return fields[fields.index('via') + 1]
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return '127.0.0.1'


def generate_launch_description():
    driver_launch = PathJoinSubstitution([
        FindPackageShare('learm_driver'), 'launch', 'learm_bringup.launch.py'])
    recording_launch = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'), 'launch', 'recording.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('windows_host', default_value=default_windows_host()),
        DeclareLaunchArgument('serial_tcp_port', default_value='8766'),
        DeclareLaunchArgument('camera_tcp_port', default_value='8767'),
        DeclareLaunchArgument(
            'dataset_root', default_value='/root/LeArm_Robot/datasets/learm_vla/raw'),
        DeclareLaunchArgument('task', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'serial_transport': 'tcp',
                'tcp_host': LaunchConfiguration('windows_host'),
                'tcp_port': LaunchConfiguration('serial_tcp_port'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recording_launch),
            launch_arguments={
                'camera_source': 'topic',
                'windows_host': LaunchConfiguration('windows_host'),
                'camera_tcp_port': LaunchConfiguration('camera_tcp_port'),
                'dataset_root': LaunchConfiguration('dataset_root'),
                'task': LaunchConfiguration('task'),
            }.items(),
        ),
    ])
