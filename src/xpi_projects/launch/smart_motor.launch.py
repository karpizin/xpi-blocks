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
                'motor_a_pwm_pin': 18,
                'motor_a_in1_pin': 23,
                'motor_a_in2_pin': 24,
                'motor_b_pwm_pin': -1,   # single motor: leave motor B's default GPIOs alone
                'stby_pin': 25
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
                'polling_rate': 10.0     # INA219 is on the board's default I2C bus (bus 1)
            }]
        )
    ])
