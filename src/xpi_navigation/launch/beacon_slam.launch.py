from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Ranging Node (Hardware Interface)
        Node(
            package='xpi_navigation',
            executable='uwb_ranging_node',
            name='uwb_ranging',
            parameters=[{'simulate': True}] # Testing by default
        ),
        
        # 2. SLAM Node (Math Engine)
        Node(
            package='xpi_navigation',
            executable='beacon_slam_node',
            name='beacon_slam',
            output='screen',
            remappings=[('~/raw_ranges', '/uwb_ranging/raw_ranges')]
        )
    ])
