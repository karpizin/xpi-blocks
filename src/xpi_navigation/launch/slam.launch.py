import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    config_file = '/Users/slava/Documents/xpi-blocks/src/xpi_navigation/config/mapper_params_online_async.yaml'
    
    return LaunchDescription([
        # 1. LiDAR Driver (LD19)
        Node(
            package='xpi_sensors',
            executable='/Users/slava/Documents/xpi-blocks/src/xpi_sensors/xpi_sensors/ld19_node.py',
            name='ld19_node',
            parameters=[{'port': '/dev/ttyUSB0'}],
            remappings=[('~/scan', '/scan')]
        ),

        # 2. Static Transform: base_footprint -> base_link -> lidar_link
        # For mapping we need a hierarchy. 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link']
        ),

        # 3. Slam Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file]
        ),
        
        # 4. Rviz2 (Optional, for desktop)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
