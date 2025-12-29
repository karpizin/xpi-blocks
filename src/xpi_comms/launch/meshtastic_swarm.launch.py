from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Meshtastic Bridge Node
        Node(
            package='xpi_comms',
            executable='meshtastic_bridge',
            name='meshtastic_bridge',
            parameters=[{
                'interface': 'serial',
                'address': '/dev/ttyUSB0', # Change to your port
                'node_name': 'drone_01'
            }],
            output='screen'
        ),

        # 2. Swarm Controller Node
        Node(
            package='xpi_comms',
            executable='swarm_controller',
            name='swarm_controller',
            parameters=[{
                'safe_distance': 10.0 # meters
            }],
            output='screen'
        )
    ])
