from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_comms',
            executable='rtc_monitor_node',
            name='rtc_status',
            output='screen'
        )
    ])
