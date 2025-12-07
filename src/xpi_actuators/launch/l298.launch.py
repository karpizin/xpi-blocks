from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import json
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_mode',
            default_value='stepper', # 'stepper' or 'dc_dual'
            description='Operating mode of the L298 driver (stepper or dc_dual)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real L298 driver'
        ),

        # Stepper Parameters
        DeclareLaunchArgument(
            'stepper_in1_pin',
            default_value='17',
            description='GPIO BCM pin for Stepper IN1'
        ),
        DeclareLaunchArgument(
            'stepper_in2_pin',
            default_value='27',
            description='GPIO BCM pin for Stepper IN2'
        ),
        DeclareLaunchArgument(
            'stepper_in3_pin',
            default_value='22',
            description='GPIO BCM pin for Stepper IN3'
        ),
        DeclareLaunchArgument(
            'stepper_in4_pin',
            default_value='23',
            description='GPIO BCM pin for Stepper IN4'
        ),
        DeclareLaunchArgument(
            'stepper_enable_pin',
            default_value='None', # Set to actual GPIO if used, e.g., '5'
            description='Optional: GPIO BCM pin for Stepper ENABLE (active high)'
        ),
        DeclareLaunchArgument(
            'stepper_steps_per_revolution',
            default_value='200',
            description='Number of full steps per motor revolution for stepper mode'
        ),
        DeclareLaunchArgument(
            'stepper_step_delay_s',
            default_value='0.005',
            description='Minimum delay in seconds between each stepper step'
        ),
        DeclareLaunchArgument(
            'stepper_motor_max_rpm',
            default_value='60',
            description='Maximum RPM of the stepper motor for continuous speed calculation'
        ),

        # DC Dual Motor Parameters
        DeclareLaunchArgument(
            'motor_a_in1_pin',
            default_value='17',
            description='GPIO BCM pin for Motor A IN1 (DC mode)'
        ),
        DeclareLaunchArgument(
            'motor_a_in2_pin',
            default_value='27',
            description='GPIO BCM pin for Motor A IN2 (DC mode)'
        ),
        DeclareLaunchArgument(
            'motor_a_pwm_pin',
            default_value='None', # Set to actual GPIO if using L298EN for speed
            description='Optional: GPIO BCM pin for Motor A PWM (DC mode)'
        ),
        DeclareLaunchArgument(
            'motor_b_in1_pin',
            default_value='22',
            description='GPIO BCM pin for Motor B IN1 (DC mode)'
        ),
        DeclareLaunchArgument(
            'motor_b_in2_pin',
            default_value='23',
            description='GPIO BCM pin for Motor B IN2 (DC mode)'
        ),
        DeclareLaunchArgument(
            'motor_b_pwm_pin',
            default_value='None', # Set to actual GPIO if using L298EN for speed
            description='Optional: GPIO BCM pin for Motor B PWM (DC mode)'
        ),
        DeclareLaunchArgument(
            'dc_pwm_frequency',
            default_value='1000',
            description='PWM Frequency in Hz for DC motor speed control'
        ),
        
        Node(
            package='xpi_actuators',
            executable='l298_driver_node',
            name='l298_motor_driver',
            parameters=[{
                'driver_mode': LaunchConfiguration('driver_mode'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                
                # Stepper parameters
                'stepper_in1_pin': LaunchConfiguration('stepper_in1_pin'),
                'stepper_in2_pin': LaunchConfiguration('stepper_in2_pin'),
                'stepper_in3_pin': LaunchConfiguration('stepper_in3_pin'),
                'stepper_in4_pin': LaunchConfiguration('stepper_in4_pin'),
                'stepper_enable_pin': LaunchConfiguration('stepper_enable_pin'),
                'stepper_steps_per_revolution': LaunchConfiguration('stepper_steps_per_revolution'),
                'stepper_step_delay_s': LaunchConfiguration('stepper_step_delay_s'),
                'stepper_motor_max_rpm': LaunchConfiguration('stepper_motor_max_rpm'),

                # DC Dual Motor parameters
                'motor_a_in1_pin': LaunchConfiguration('motor_a_in1_pin'),
                'motor_a_in2_pin': LaunchConfiguration('motor_a_in2_pin'),
                'motor_a_pwm_pin': LaunchConfiguration('motor_a_pwm_pin'),
                'motor_b_in1_pin': LaunchConfiguration('motor_b_in1_pin'),
                'motor_b_in2_pin': LaunchConfiguration('motor_b_in2_pin'),
                'motor_b_pwm_pin': LaunchConfiguration('motor_b_pwm_pin'),
                'dc_pwm_frequency': LaunchConfiguration('dc_pwm_frequency'),
            }],
            output='screen'
        )
    ])
