from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB2',
            description='AT command port of the modem'
        ),
        DeclareLaunchArgument(
            'enable_gps',
            default_value='true',
            description='Whether to enable and publish GNSS data'
        ),
        
        Node(
            package='xpi_comms',
            executable='lte_modem_node',
            name='lte_management',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'enable_gps': LaunchConfiguration('enable_gps')
            }],
            output='screen'
        )
    ])
