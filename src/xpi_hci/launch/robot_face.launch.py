from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The Brain: Procedural Eyes Generator
        Node(
            package='xpi_hci',
            executable='expression_engine',
            name='expression_engine',
            output='screen'
        ),
        
        # The Adapter: For 16x16 Matrix
        Node(
            package='xpi_hci',
            executable='led_matrix_face',
            name='led_matrix_face',
            parameters=[{'width': 16, 'height': 16}]
        )
    ])
