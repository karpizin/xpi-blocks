from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('ntrip_mount', default_value=''),

        # 1. RTK GPS Node
        Node(
            package='xpi_sensors',
            executable='gps_rtk_node.py',
            name='gps_rtk',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': 38400
            }],
            output='screen'
        ),

        # 2. NTRIP Client (Only if mountpoint is provided)
        Node(
            package='xpi_sensors',
            executable='ntrip_client_node.py',
            name='ntrip_client',
            parameters=[{
                'mountpoint': LaunchConfiguration('ntrip_mount'),
                'host': 'rtk2go.com'
            }],
            condition=lambda context: LaunchConfiguration('ntrip_mount').perform(context) != '',
            output='screen'
        )
    ])
