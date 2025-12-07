from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor A Pins
        DeclareLaunchArgument('motor_a_pwm_pin', default_value='12', description='GPIO BCM for Motor A PWM'),
        DeclareLaunchArgument('motor_a_in1_pin', default_value='5', description='GPIO BCM for Motor A IN1'),
        DeclareLaunchArgument('motor_a_in2_pin', default_value='6', description='GPIO BCM for Motor A IN2'),
        # Motor B Pins
        DeclareLaunchArgument('motor_b_pwm_pin', default_value='13', description='GPIO BCM for Motor B PWM'),
        DeclareLaunchArgument('motor_b_in1_pin', default_value='26', description='GPIO BCM for Motor B IN1'),
        DeclareLaunchArgument('motor_b_in2_pin', default_value='19', description='GPIO BCM for Motor B IN2'),

        DeclareLaunchArgument('pwm_frequency', default_value='1000', description='PWM Frequency in Hz'),
        DeclareLaunchArgument('mock_hardware', default_value='false', description='Run in mock mode without real GPIO'),

        Node(
            package='xpi_actuators',
            executable='tb6612_driver_node',
            name='motor_driver',
            parameters=[{
                'motor_a_pwm_pin': LaunchConfiguration('motor_a_pwm_pin'),
                'motor_a_in1_pin': LaunchConfiguration('motor_a_in1_pin'),
                'motor_a_in2_pin': LaunchConfiguration('motor_a_in2_pin'),
                'motor_b_pwm_pin': LaunchConfiguration('motor_b_pwm_pin'),
                'motor_b_in1_pin': LaunchConfiguration('motor_b_in1_pin'),
                'motor_b_in2_pin': LaunchConfiguration('motor_b_in2_pin'),
                'pwm_frequency': LaunchConfiguration('pwm_frequency'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
