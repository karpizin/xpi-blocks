from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='apds9960_node.py',
            name='apds9960',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'polling_rate': 20.0,
                'enable_color': True,
                'enable_prox': True,
                'enable_gesture': True
            }]
        )
    ])
