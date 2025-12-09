from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
                'camera_frame_id': 'camera_link',
                'video_device': '/dev/video0',
                'pixel_format': 'YUYV' # Use 'YUYV' for uncompressed or 'MJPG' for compressed (lower CPU load if camera supports it)
            }]
        )
    ])
