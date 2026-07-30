from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('input_source', default_value=''),
        DeclareLaunchArgument('camera_backend', default_value='v4l2'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),
        DeclareLaunchArgument('frame_width', default_value='320'),
        DeclareLaunchArgument('frame_height', default_value='240'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('show_image', default_value='false'),
        DeclareLaunchArgument('loop_video', default_value='true'),
        DeclareLaunchArgument('log_detections', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('target_device', default_value='cpu'),
        Node(
            package='learm_vision',
            executable='model_detector',
            name='learm_model_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'input_source': LaunchConfiguration('input_source'),
                'camera_backend': LaunchConfiguration('camera_backend'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'image_size': LaunchConfiguration('image_size'),
                'show_image': LaunchConfiguration('show_image'),
                'loop_video': LaunchConfiguration('loop_video'),
                'log_detections': LaunchConfiguration('log_detections'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'device': LaunchConfiguration('target_device'),
            }],
        ),
    ])
