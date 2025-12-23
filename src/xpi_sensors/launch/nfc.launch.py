from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'polling_rate',
            default_value='5.0',
            description='Scanning frequency in Hz'
        ),
        
        Node(
            package='xpi_sensors',
            executable='nfc_reader_node',
            name='nfc_scanner',
            parameters=[{
                'polling_rate': LaunchConfiguration('polling_rate')
            }],
            output='screen'
        )
    ])
