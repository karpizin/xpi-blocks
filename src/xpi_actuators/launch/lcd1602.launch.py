from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_actuators',
            executable='lcd1602_node.py',
            name='lcd1602',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x27,
                'cols': 16,
                'rows': 2
            }]
        )
    ])
