from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. ArUco Tracker
        Node(
            package='xpi_vision',
            executable='aruco_tracker_node',
            name='aruco_tracker',
            output='screen',
            parameters=[{
                'marker_size': 0.1,
                'dictionary_name': 'DICT_4X4_50'
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/camera_info', '/camera/camera_info')
            ]
        )
    ])
