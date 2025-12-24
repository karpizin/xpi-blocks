from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gpio_pin', default_value='18'),
        DeclareLaunchArgument('is_passive', default_value='true'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_actuators',
            executable='buzzer_node',
            name='buzzer',
            namespace='buzzer',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'is_passive': LaunchConfiguration('is_passive'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
