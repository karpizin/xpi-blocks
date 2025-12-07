from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json

def generate_launch_description():
    # Convert string representation of list to actual list
    # Use environment variable to pass list of integers
    # Example: export PINS_LIST="[18, 23, 24]"
    pins_env_var = os.environ.get('XPI_PINS_LIST', '[18, 23]')
    pins_list = json.loads(pins_env_var)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pins',
            default_value=pins_env_var, # Pass as string
            description='List of BCM GPIO pins to monitor (e.g., "[18, 23, 24]")'
        ),
        DeclareLaunchArgument(
            'pull_up',
            default_value='true',
            description='Use internal pull-up resistor for input pins'
        ),
        DeclareLaunchArgument(
            'pull_down',
            default_value='false',
            description='Use internal pull-down resistor for input pins'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Rate in Hz to publish pin states'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real GPIO'
        ),
        
        Node(
            package='xpi_sensors',
            executable='gpio_digital_input_node',
            name='digital_input_monitor',
            parameters=[{
                # Pass list of integers for pins
                'pins': pins_list,
                'pull_up': LaunchConfiguration('pull_up'),
                'pull_down': LaunchConfiguration('pull_down'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
