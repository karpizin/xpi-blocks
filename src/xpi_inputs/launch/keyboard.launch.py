from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen', # Show output in terminal
            prefix='xterm -e', # For running in a separate terminal
            parameters=[{
                'speed': 0.5, # Linear speed in m/s
                'turn': 1.0   # Angular speed in rad/s
            }]
        )
    ])
