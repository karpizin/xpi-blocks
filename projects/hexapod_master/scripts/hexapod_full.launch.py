import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    nodes_dir = Path(__file__).resolve().parents[1] / 'nodes'
    
    return LaunchDescription([
        # 1. Global Controller Node (Body Kinematics)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'hexapod_body_kinematics_node.py'), '--ros-args', '-r', '__node:=body_node'],
            output='screen'
        ),

        # 2. Gait Generator (Gait Engine)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'hexapod_gait_node.py'), '--ros-args', '-r', '__node:=gait_node'],
            output='screen'
        ),

        # 3. Automatic Stabilization (Auto-Leveler)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'auto_leveler_node.py'), '--ros-args', '-r', '__node:=leveler_node'],
            output='screen'
        ),

        # 5. Heading Controller (Turn to Angle)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'heading_controller_node.py'), '--ros-args', '-r', '__node:=heading_node'],
            output='screen'
        ),

        # 6. Circular Motion Controller
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'circular_motion_node.py'), '--ros-args', '-r', '__node:=circular_motion_node'],
            output='screen'
        ),

        # 7. Contact Bridge (for Gazebo feedback)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'contact_bridge_node.py'), '--ros-args', '-r', '__node:=contact_bridge_node'],
            output='screen'
        ),

        # 4. Viewer (link with Rviz2/Gazebo)
        ExecuteProcess(
            cmd=[sys.executable, str(nodes_dir / 'leg_ik_viewer_node.py'), '--ros-args', '-r', '__node:=ik_bridge'],
            output='screen'
        ),
    ])
