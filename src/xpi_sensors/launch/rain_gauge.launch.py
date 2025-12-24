from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gpio_pin', default_value='22'),
        DeclareLaunchArgument('mm_per_pulse', default_value='0.2794'),
        DeclareLaunchArgument('use_mock', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='rain_gauge_node',
            name='rain_gauge',
            namespace='rain_gauge',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'mm_per_pulse': LaunchConfiguration('mm_per_pulse'),
                'use_mock': LaunchConfiguration('use_mock'),
            }],
            output='screen'
        )
    ])
