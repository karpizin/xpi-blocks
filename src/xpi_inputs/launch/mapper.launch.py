import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('xpi_inputs'),
        'config',
        'mapper_example.yaml'
    )

    return LaunchDescription([
        Node(
            package='xpi_inputs',
            executable='joy_mapper_node',
            name='joy_mapper',
            output='screen',
            parameters=[config]
        )
    ])
