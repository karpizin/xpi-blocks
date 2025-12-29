import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF file (use absolute path for simplicity in this context)
    urdf_file = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/urdf/leg.urdf'
    
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
        Node(
            package='xpi_actuators', # Use an existing package for launch
            executable='/Users/slava/Documents/xpi-blocks/projects/hexapod_master/nodes/leg_ik_viewer_node.py',
            name='leg_ik_viewer',
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
