from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_comms',
            executable='mqtt_bridge_node.py',
            name='mqtt_bridge',
            output='screen',
            parameters=[{
                'broker_address': 'localhost',
                'broker_port': 1883,
                'mqtt_topic_sub': 'robot/rx',
                'mqtt_topic_pub': 'robot/tx'
            }]
        )
    ])
