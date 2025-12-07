from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json
import os

def generate_launch_description():
    # Convert string representation of list to actual list
    # Use environment variable to pass list of integers
    # Example: export PINS_LIST="[2, 3, 4, 17, 27, 22, 10, 9]"
    pins_env_var = os.environ.get('XPI_LEDBAR_PINS', '[2, 3, 4, 17, 27, 22, 10, 9]')
    pins_list = json.loads(pins_env_var)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pins',
            default_value=pins_env_var, # Pass as string
            description='List of BCM GPIO pins connected to the LEDs (e.g., "[2, 3, 4, 17]")'
        ),
        DeclareLaunchArgument(
            'invert_logic',
            default_value='false',
            description='Set to true if LEDs are active low (e.g., connected to GND with pull-up)'
        ),
        DeclareLaunchArgument(
            'initial_value',
            default_value='0',
            description='Number of LEDs to light up initially (0 to count of pins)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real GPIO'
        ),
        
        Node(
            package='xpi_actuators',
            executable='led_bar_node',
            name='led_bar_display',
            parameters=[{
                'pins': pins_list, # Pass as list of integers
                'invert_logic': LaunchConfiguration('invert_logic'),
                'initial_value': LaunchConfiguration('initial_value'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
