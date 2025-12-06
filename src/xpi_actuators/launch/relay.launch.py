from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='17',
            description='GPIO pin number (BCM)'
        ),
        DeclareLaunchArgument(
            'active_high',
            default_value='true',
            description='Set true for NO relay, false for NC relay'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in simulation mode without real GPIO'
        ),
        
        Node(
            package='xpi_actuators',
            executable='relay_node',
            name='my_relay',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'active_high': LaunchConfiguration('active_high'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }]
        )
    ])
