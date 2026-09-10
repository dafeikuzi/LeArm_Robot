from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'), 'config', 'network_camera.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8767'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        Node(
            package='learm_vla_bridge',
            executable='network_camera_node',
            name='learm_network_camera',
            output='screen',
            parameters=[config_file, {
                'host': LaunchConfiguration('host'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'), value_type=float),
            }],
        ),
    ])
