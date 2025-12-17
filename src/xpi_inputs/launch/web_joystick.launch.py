from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='web_joystick_node',
            name='web_joy',
            output='screen',
            parameters=[{
                'port': 8080,
                'scale_linear': 1.0,
                'scale_angular': 1.5
            }]
        )
    ])
