from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='max17048_node.py',
            name='battery_gauge',
            output='screen',
            parameters=[{'polling_rate': 1.0}]
        )
    ])
