from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        Node(
            package='arm_serial_control',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[serial_config],
        ),
        Node(
            package='learm_driver',
            executable='learm_driver_node',
            name='learm_driver',
            output='screen',
            parameters=[driver_config],
        ),
    ])
