import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    project_dir = Path(__file__).resolve().parents[1]
    urdf_file = project_dir / 'urdf' / 'leg.urdf'
    viewer_script = project_dir / 'nodes' / 'leg_ik_viewer_node.py'
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        ExecuteProcess(
            cmd=[sys.executable, str(viewer_script), '--ros-args', '-r', '__node:=leg_ik_viewer'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_config_dir] # Config can be added later
        ),
    ])
