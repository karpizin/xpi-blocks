from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='max44009_node.py',
            name='max44009',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x4A,
                'polling_rate': 5.0
            }]
        )
    ])
