from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='tcs34725_node',
            name='tcs34725',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x29,
                'poll_rate': 2.0,
                'gain': 1,            # 1, 4, 16, 60
                'integration_time_ms': 154 # 2.4, 24, 50, 101, 154, 700
            }]
        )
    ])
