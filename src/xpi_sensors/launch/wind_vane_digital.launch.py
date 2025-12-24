from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='1'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('offset_degrees', default_value='0.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='wind_vane_digital_node',
            name='wind_vane_digital',
            namespace='wind_vane_digital',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'offset_degrees': LaunchConfiguration('offset_degrees'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
