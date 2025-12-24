from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('data_pin', default_value='5'),
        DeclareLaunchArgument('clock_pin', default_value='6'),
        DeclareLaunchArgument('reference_unit', default_value='1.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='hx711_node',
            name='hx711',
            namespace='hx711',
            parameters=[{
                'data_pin': LaunchConfiguration('data_pin'),
                'clock_pin': LaunchConfiguration('clock_pin'),
                'reference_unit': LaunchConfiguration('reference_unit'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
