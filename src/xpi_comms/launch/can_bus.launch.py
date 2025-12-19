from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_comms',
            executable='can_bridge_node.py',
            name='can_bridge',
            output='screen',
            parameters=[{
                'interface': 'can0',
                'bitrate': 500000
            }]
        )
    ])
