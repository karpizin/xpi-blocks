from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='bno055_node',
            name='bno055',
            namespace='bno055',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
