from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='gps_node.py',
            name='gps',
            output='screen',
            parameters=[{
                'port': '/dev/ttyS0',
                'baudrate': 9600,
                'frame_id': 'gps_link'
            }]
        )
    ])
