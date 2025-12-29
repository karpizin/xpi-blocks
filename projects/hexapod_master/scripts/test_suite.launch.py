import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master'
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    nodes_dir = os.path.join(pkg_dir, 'nodes')

    return LaunchDescription([
        # 1. Start the Full Hexapod System
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scripts_dir, 'hexapod_full.launch.py')
            )
        ),

        # 2. Add the Test Scenario Node
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'test_scenario_node.py'),
            name='test_scenario_node',
            output='screen'
        )
    ])
