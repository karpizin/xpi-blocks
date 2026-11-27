from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frequency', default_value='5', description='GPS Update Rate (Hz)'),
        DeclareLaunchArgument('ntrip_mount', default_value=''),

        # 1. RTK GPS Node
        Node(
            package='xpi_sensors',
            executable='gps_rtk_node',
            name='gps_rtk',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'frequency': LaunchConfiguration('frequency'),
                'baudrate': 38400
            }],
            output='screen'
        ),

        # 2. NTRIP Client (Only if mountpoint is provided)
        Node(
            package='xpi_sensors',
            executable='ntrip_client_node',
            name='ntrip_client',
            parameters=[{
                'mountpoint': LaunchConfiguration('ntrip_mount'),
                'host': 'rtk2go.com'
            }],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ntrip_mount'), "' != ''"])),
            output='screen'
        )
    ])
