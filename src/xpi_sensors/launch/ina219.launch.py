from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='ina219_node.py',
            name='ina219',
            output='screen',
            parameters=[{
                'polling_rate': 10.0,
                'shunt_ohms': 0.1
            }]
        )
    ])
