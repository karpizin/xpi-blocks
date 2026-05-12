from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The Brain: Procedural Eyes Generator
        Node(
            package='xpi_hci',
            executable='expression_engine',
            name='robot_face',
            output='screen'
        ),
        
        # The Adapter: For 16x16 Matrix
        Node(
            package='xpi_hci',
            executable='led_matrix_face',
            name='robot_face_matrix',
            parameters=[{
                'width': 16,
                'height': 16,
                'source_image_topic': '/robot_face/eyes_image',
            }]
        )
    ])
