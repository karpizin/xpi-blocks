from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='opt3001_node',
            name='opt3001',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x44, # Check if ADDR pin is connected to GND (0x44), VDD (0x45), SDA (0x46), SCL (0x47)
                'poll_rate': 1.0
            }]
        )
    ])
