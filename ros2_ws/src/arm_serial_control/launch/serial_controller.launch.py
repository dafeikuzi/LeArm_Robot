from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    transport = LaunchConfiguration('transport')
    port = LaunchConfiguration('port')
    tcp_host = LaunchConfiguration('tcp_host')
    tcp_port = LaunchConfiguration('tcp_port')
    config_file = PathJoinSubstitution([
        FindPackageShare('arm_serial_control'),
        'config',
        'serial_controller.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='tcp'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='8766'),
        Node(
            package='arm_serial_control',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[config_file, {
                'transport': transport,
                'port': port,
                'tcp_host': tcp_host,
                'tcp_port': ParameterValue(tcp_port, value_type=int),
            }],
        ),
    ])
