from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spi_port', default_value='0'),
        DeclareLaunchArgument('spi_cs', default_value='0'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('default_height', default_value='0.1'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_navigation',
            executable='optical_flow_node',
            name='optical_flow',
            namespace='optical_flow',
            parameters=[{
                'spi_port': LaunchConfiguration('spi_port'),
                'spi_cs': LaunchConfiguration('spi_cs'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'default_height': LaunchConfiguration('default_height'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
