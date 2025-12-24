from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS0', description='Serial port for HC-12'),
        DeclareLaunchArgument('baudrate', default_value='9600', description='Baud rate for HC-12'),
        DeclareLaunchArgument('set_pin', default_value='17', description='GPIO pin for SET (Active LOW)'),
        DeclareLaunchArgument('use_mock', default_value='false', description='Use mock GPIO'),

        Node(
            package='xpi_comms',
            executable='hc12_node',
            name='hc12',
            namespace='hc12',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'set_pin': LaunchConfiguration('set_pin'),
                'use_mock': LaunchConfiguration('use_mock'),
            }],
            output='screen'
        )
    ])
