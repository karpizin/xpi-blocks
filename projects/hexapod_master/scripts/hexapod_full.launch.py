import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes_dir = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/nodes'
    
    return LaunchDescription([
        # 1. Global Controller Node (Body Kinematics)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'hexapod_body_kinematics_node.py'),
            name='body_node',
            output='screen'
        ),

        # 2. Gait Generator (Gait Engine)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'hexapod_gait_node.py'),
            name='gait_node',
            output='screen'
        ),

        # 3. Automatic Stabilization (Auto-Leveler)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'auto_leveler_node.py'),
            name='leveler_node',
            output='screen'
        ),

        # 5. Heading Controller (Turn to Angle)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'heading_controller_node.py'),
            name='heading_node',
            output='screen'
        ),

        # 6. Circular Motion Controller
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'circular_motion_node.py'),
            name='circular_motion_node',
            output='screen'
        ),

        # 7. Contact Bridge (for Gazebo feedback)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'contact_bridge_node.py'),
            name='contact_bridge_node',
            output='screen'
        ),

        # 4. Viewer (link with Rviz2/Gazebo)
        Node(
            package='xpi_actuators',
            executable=os.path.join(nodes_dir, 'leg_ik_viewer_node.py'),
            name='ik_bridge',
            output='screen'
        ),
    ])
