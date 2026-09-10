from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('learm_vla_bridge')
    observation_config = PathJoinSubstitution([package_share, 'config', 'observation.yaml'])
    recorder_config = PathJoinSubstitution([package_share, 'config', 'recorder.yaml'])
    camera_config = PathJoinSubstitution([package_share, 'config', 'network_camera.yaml'])
    camera_source = LaunchConfiguration('camera_source')
    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='topic'),
        DeclareLaunchArgument('windows_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('camera_tcp_port', default_value='8767'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('dataset_root', default_value='/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw'),
        DeclareLaunchArgument('task', default_value=''),
        DeclareLaunchArgument('camera_fps', default_value='15'),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('status_poll_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('status_freshness_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('record_rate_hz', default_value='5.0'),
        Node(
            package='learm_vla_bridge',
            executable='network_camera_node',
            name='learm_network_camera',
            output='screen',
            condition=IfCondition(PythonExpression(["'", camera_source, "' == 'topic'"])),
            parameters=[camera_config, {
                'host': LaunchConfiguration('windows_host'),
                'port': ParameterValue(LaunchConfiguration('camera_tcp_port'), value_type=int),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=float),
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
                'task': LaunchConfiguration('task'),
                'camera_fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'), value_type=float),
                'status_poll_rate_hz': ParameterValue(
                    LaunchConfiguration('status_poll_rate_hz'), value_type=float),
                'status_freshness_timeout_s': ParameterValue(
                    LaunchConfiguration('status_freshness_timeout_s'), value_type=float),
            }],
        ),
        Node(
            package='learm_vla_bridge',
            executable='episode_recorder',
            name='learm_vla_recorder',
            output='screen',
            parameters=[recorder_config, {
                'dataset_root': LaunchConfiguration('dataset_root'),
                'task_override': LaunchConfiguration('task'),
                'record_rate_hz': ParameterValue(
                    LaunchConfiguration('record_rate_hz'), value_type=float),
            }],
        ),
    ])
