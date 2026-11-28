from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='Address to listen on. 127.0.0.1: this machine only; '
                        '0.0.0.0: any device on the network (a token is always required).'),
        DeclareLaunchArgument('port', default_value='8080', description='Web server port'),
        # The token is not a launch argument on purpose (it would show in `ps` and
        # `ros2 param get`): set XPI_WEB_JOYSTICK_TOKEN, or let the node generate one and
        # print the URL to open.
        Node(
            package='xpi_inputs',
            executable='web_joystick_node',
            name='web_joy',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'scale_linear': 1.0,
                'scale_angular': 1.5
            }]
        )
    ])
