from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='ccs811_node.py',
            name='ccs811',
            output='screen',
            parameters=[{
                'polling_rate': 1.0
            }]
        )
    ])
