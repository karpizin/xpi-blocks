from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. BLE Ranging Node
        Node(
            package='xpi_navigation',
            executable='ble_ranging_node',
            name='ble_ranging',
            parameters=[{
                'scan_duration': 1.0,
                'name_prefix': 'BEACON' # Look for devices named BEACON_1, BEACON_2 etc.
            }]
        ),
        
        # 2. SLAM Node (Shared with UWB)
        Node(
            package='xpi_navigation',
            executable='beacon_slam_node',
            name='ble_slam',
            output='screen',
            remappings=[('~/raw_ranges', '/ble_ranging/raw_ranges')]
        )
    ])
