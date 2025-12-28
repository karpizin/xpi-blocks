import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к полной URDF модели
    urdf_file = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/urdf/hexapod.urdf'
    
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
        # Используем GUI для ручного теста всех суставов одновременно (все 18 серв)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
