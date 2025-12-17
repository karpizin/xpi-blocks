from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='mouse_node',
            name='mouse_driver',
            output='screen',
            parameters=[{
                'device_name': 'Mouse', # Or 'Touchpad'
                'mode': 'velocity',     # 'velocity' or 'position'
                'sensitivity': 0.005,
                'decay': 0.2
            }]
        )
    ])
