from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='keyboard_to_joy_node',
            name='keyboard_joy',
            output='screen',
            prefix='xterm -e', # Run in separate terminal to capture keys
        )
    ])