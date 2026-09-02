from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'),
        'config',
        'observation.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('status_service', default_value='/learm_driver/get_status'),
        DeclareLaunchArgument('task', default_value=''),
        Node(
            package='learm_vla_bridge',
            executable='observation_node',
            name='learm_vla_observation',
            output='screen',
            parameters=[config_file, {
                'camera_device': LaunchConfiguration('camera_device'),
                'status_service': LaunchConfiguration('status_service'),
                'task': LaunchConfiguration('task'),
            }],
        ),
    ])
