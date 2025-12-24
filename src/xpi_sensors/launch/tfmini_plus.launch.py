from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='tfmini_plus_node',
            name='tfmini_plus',
            namespace='tfmini_plus',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
