from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='5',
            description='BCM GPIO pin connected to the anemometer'
        ),
        DeclareLaunchArgument(
            'update_interval',
            default_value='5.0',
            description='Calculation window in seconds'
        ),
        DeclareLaunchArgument(
            'factor',
            default_value='0.666',
            description='Conversion factor (1 Hz = X m/s). 0.666 is common for 2.4 km/h per Hz.'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode'
        ),
        
        Node(
            package='xpi_sensors',
            executable='anemometer_node',
            name='wind_speed_sensor',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'update_interval': LaunchConfiguration('update_interval'),
                'factor': LaunchConfiguration('factor'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
