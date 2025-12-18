from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='gesture_control_node',
            name='gesture_ctl',
            output='screen',
            parameters=[{
                'mode': 'proportional', # proportional, discrete, joy
                'activation_gesture': 'FIST',
                'image_topic': '/image_raw'
            }]
        )
    ])
