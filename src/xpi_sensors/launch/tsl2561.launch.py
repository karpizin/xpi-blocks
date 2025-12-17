from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='tsl2561_node',
            name='tsl2561',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x39, # Check your module: Float=0x39, GND=0x29, VCC=0x49
                'poll_rate': 1.0,
                'gain_16x': False,   # Set True for low light
                'integration_time': 2 # 0=13ms, 1=101ms, 2=402ms
            }]
        )
    ])
