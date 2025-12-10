from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('led_count', default_value='30', description='Number of LEDs'),
        DeclareLaunchArgument('led_pin', default_value='18', description='GPIO Pin'),
        DeclareLaunchArgument('initial_effect', default_value='solid', description='Startup Effect'),
        
        Node(
            package='xpi_actuators',
            executable='ws2812_driver',
            name='ws2812_driver',
            parameters=[{
                'led_count': LaunchConfiguration('led_count'),
                'led_pin': LaunchConfiguration('led_pin'),
                'initial_effect': LaunchConfiguration('initial_effect')
            }],
            output='screen'
        )
    ])
