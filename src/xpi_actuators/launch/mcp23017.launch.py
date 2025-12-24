from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_address', default_value='0x20'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_actuators',
            executable='mcp23017',
            name='mcp23017',
            namespace='mcp23017',
            parameters=[{
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
