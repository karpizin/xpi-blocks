from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Motor Driver (TB6612)
        Node(
            package='xpi_actuators',
            executable='tb6612_driver',
            name='motor_driver',
            parameters=[{
                'pin_pwma': 18,
                'pin_ain1': 23,
                'pin_ain2': 24,
                'pin_stby': 25
            }]
        ),
        
        # 2. Encoder
        Node(
            package='xpi_inputs',
            executable='rotary_encoder_node',
            name='motor_encoder',
            parameters=[{
                'pin_a': 17,
                'pin_b': 27,
                'publish_rate': 50.0
            }]
        ),
        
        # 3. Power Monitor
        Node(
            package='xpi_sensors',
            executable='ina219_node',
            name='motor_power',
            parameters=[{
                'i2c_bus': 1,
                'polling_rate': 10.0
            }]
        )
    ])
