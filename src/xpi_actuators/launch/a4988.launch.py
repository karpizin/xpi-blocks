from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json
import os

def generate_launch_description():
    # Microstep pins, e.g., for A4988: MS1 (GPIO22), MS2 (GPIO23), MS3 (GPIO24)
    microstep_pins_env_var = os.environ.get('XPI_A4988_MS_PINS', '[]')
    microstep_pins_list = json.loads(microstep_pins_env_var)

    return LaunchDescription([
        DeclareLaunchArgument(
            'step_pin',
            default_value='17',
            description='GPIO BCM pin for STEP signal'
        ),
        DeclareLaunchArgument(
            'dir_pin',
            default_value='27',
            description='GPIO BCM pin for DIR signal'
        ),
        DeclareLaunchArgument(
            'enable_pin',
            default_value='None', # Set to actual GPIO if used, e.g., '22'
            description='Optional: GPIO BCM pin for ENABLE signal (active low)'
        ),
        DeclareLaunchArgument(
            'microstep_pins',
            default_value=microstep_pins_env_var, # Pass as string
            description='List of BCM GPIO pins for microstep selection (e.g., "[22, 23, 24]")'
        ),
        DeclareLaunchArgument(
            'microstep_setting',
            default_value='16', # Common setting (1, 2, 4, 8, 16 for A4988; up to 32 for DRV8825)
            description='Microstep setting (e.g., 1 for full step, 16 for 1/16 step)'
        ),
        DeclareLaunchArgument(
            'steps_per_revolution_fullstep',
            default_value='200', # Typical for 1.8 degree stepper
            description='Number of full steps per motor revolution'
        ),
        DeclareLaunchArgument(
            'step_delay_s',
            default_value='0.001', # Minimum delay in seconds between steps
            description='Minimum delay in seconds between steps (lower = faster, be careful not to lose steps)'
        ),
        DeclareLaunchArgument(
            'motor_max_rpm',
            default_value='60', # Max RPM of the motor with load
            description='Maximum RPM of the motor for continuous speed calculation'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real stepper motor'
        ),
        
        Node(
            package='xpi_actuators',
            executable='a4988_driver_node',
            name='a4988_stepper_driver',
            parameters=[{
                'step_pin': LaunchConfiguration('step_pin'),
                'dir_pin': LaunchConfiguration('dir_pin'),
                'enable_pin': LaunchConfiguration('enable_pin'),
                'microstep_pins': microstep_pins_list, # Pass as list of integers
                'microstep_setting': LaunchConfiguration('microstep_setting'),
                'steps_per_revolution_fullstep': LaunchConfiguration('steps_per_revolution_fullstep'),
                'step_delay_s': LaunchConfiguration('step_delay_s'),
                'motor_max_rpm': LaunchConfiguration('motor_max_rpm'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
