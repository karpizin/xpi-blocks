from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gpio_pin', default_value='26'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='pir_node',
            name='pir',
            namespace='pir',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
