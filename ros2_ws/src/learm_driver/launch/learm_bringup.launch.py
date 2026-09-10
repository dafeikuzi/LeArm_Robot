from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_mode_enabled = LaunchConfiguration('calibration_mode_enabled')
    serial_transport = LaunchConfiguration('serial_transport')
    serial_port = LaunchConfiguration('serial_port')
    tcp_host = LaunchConfiguration('tcp_host')
    tcp_port = LaunchConfiguration('tcp_port')
    serial_config = PathJoinSubstitution([
        FindPackageShare('arm_serial_control'),
        'config',
        'serial_controller.yaml',
    ])
    driver_config = PathJoinSubstitution([
        FindPackageShare('learm_driver'),
        'config',
        'learm_driver.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_mode_enabled',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'serial_transport',
            default_value='tcp',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='8766'),
        Node(
            package='arm_serial_control',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[serial_config, {
                'transport': serial_transport,
                'port': serial_port,
                'tcp_host': tcp_host,
                'tcp_port': ParameterValue(tcp_port, value_type=int),
            }],
        ),
        Node(
            package='learm_driver',
            executable='learm_driver_node',
            name='learm_driver',
            output='screen',
            parameters=[driver_config, {
                'calibration_mode_enabled': calibration_mode_enabled,
            }],
        ),
    ])
