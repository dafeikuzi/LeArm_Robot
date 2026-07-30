from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_pose_file = PathJoinSubstitution([
        FindPackageShare('learm_drawing'),
        'config',
        'drawing_poses.template.yaml',
    ])
    default_trajectory_file = PathJoinSubstitution([
        FindPackageShare('learm_drawing'),
        'trajectories',
        'phase2_pen_tap.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('pose_file', default_value=default_pose_file),
        DeclareLaunchArgument('trajectory_file', default_value=default_trajectory_file),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('repeat', default_value='1'),
        DeclareLaunchArgument('pose_service', default_value='/learm_driver/move_pose'),
        DeclareLaunchArgument('service_wait_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('service_call_timeout_s', default_value='5.0'),
        Node(
            package='learm_drawing',
            executable='trajectory_player',
            name='learm_trajectory_player',
            output='screen',
            parameters=[{
                'pose_file': LaunchConfiguration('pose_file'),
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'dry_run': LaunchConfiguration('dry_run'),
                'repeat': LaunchConfiguration('repeat'),
                'pose_service': LaunchConfiguration('pose_service'),
                'service_wait_timeout_s': LaunchConfiguration('service_wait_timeout_s'),
                'service_call_timeout_s': LaunchConfiguration('service_call_timeout_s'),
            }],
        ),
    ])
