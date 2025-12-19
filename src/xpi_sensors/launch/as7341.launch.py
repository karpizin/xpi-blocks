from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='as7341_node.py',
            name='as7341',
            output='screen',
            parameters=[{
                'polling_rate': 10.0,
                'gain': 3,
                'integration_time': 29
            }]
        )
    ])
