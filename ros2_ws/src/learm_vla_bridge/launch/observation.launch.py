from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    observation_config = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'),
        'config',
        'observation.yaml',
    ])
    camera_config = PathJoinSubstitution([
        FindPackageShare('learm_vla_bridge'), 'config', 'network_camera.yaml'])
    camera_source = LaunchConfiguration('camera_source')
    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='topic'),
        DeclareLaunchArgument('windows_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('camera_tcp_port', default_value='8767'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('status_service', default_value='/learm_driver/get_status'),
        DeclareLaunchArgument('task', default_value=''),
        Node(
            package='learm_vla_bridge',
            executable='network_camera_node',
            name='learm_network_camera',
            output='screen',
            condition=IfCondition(PythonExpression(["'", camera_source, "' == 'topic'"])),
            parameters=[camera_config, {
                'host': LaunchConfiguration('windows_host'),
                'port': ParameterValue(LaunchConfiguration('camera_tcp_port'), value_type=int),
            }],
        ),
        Node(
            package='learm_vla_bridge',
            executable='observation_node',
            name='learm_vla_observation',
            output='screen',
            parameters=[observation_config, {
                'camera_source': camera_source,
                'camera_device': LaunchConfiguration('camera_device'),
                'status_service': LaunchConfiguration('status_service'),
                'task': LaunchConfiguration('task'),
            }],
        ),
    ])
