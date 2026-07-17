from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('arm_serial_control'),
        'config',
        'serial_controller.yaml',
    ])
    return LaunchDescription([
        Node(
            package='arm_serial_control',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[config_file],
        ),
    ])
