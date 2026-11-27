import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    project_dir = Path(__file__).resolve().parents[1]
    scripts_dir = project_dir / 'scripts'
    nodes_dir = project_dir / 'nodes'

    return LaunchDescription([
        # 1. Start the Full Hexapod System
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(scripts_dir / 'hexapod_full.launch.py')
            )
        ),

        # 2. Add the Test Scenario Node
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'test_scenario_node.py'), '--ros-args', '-r', '__node:=test_scenario_node'],
            output='screen'
        )
    ])
