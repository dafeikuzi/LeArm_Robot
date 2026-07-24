from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_mode_enabled = LaunchConfiguration('calibration_mode_enabled')
    serial_port = LaunchConfiguration('serial_port')
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
            'serial_port',
            default_value='/dev/ttyUSB0',
        ),
        Node(
            package='arm_serial_control',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[serial_config, {
                'port': serial_port,
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
