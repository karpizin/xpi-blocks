from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='rotary_encoder_node.py',
            name='left_encoder',
            output='screen',
            parameters=[{
                'pin_a': 17,
                'pin_b': 27,
                'reverse': False,
                'publish_rate': 20.0
            }]
        )
    ])
