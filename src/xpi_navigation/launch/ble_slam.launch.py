from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('xpi_navigation')
    # Default config path (users should override or place file here)
    default_map = os.path.join(pkg_share, 'config', 'known_beacons.yaml')

    return LaunchDescription([
        # --- Ranging Node ---
        Node(
            package='xpi_navigation',
            executable='ble_ranging_node',
            name='ble_ranging_node',
            parameters=[{
                'name_prefix': 'ROBOT_BEACON'
            }]
        ),
        
        # --- SLAM / Localization Node ---
        DeclareLaunchArgument(
            'mode',
            default_value='LOCALIZATION',
            description='Operation mode: LOCALIZATION or MAPPING'
        ),
        DeclareLaunchArgument(
            'beacons_file',
            default_value=default_map,
            description='Path to YAML file with known beacon coordinates'
        ),
        
        Node(
            package='xpi_navigation',
            executable='ble_slam_node',
            name='ble_slam_node',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'beacons_file': LaunchConfiguration('beacons_file')
            }]
        )
    ])