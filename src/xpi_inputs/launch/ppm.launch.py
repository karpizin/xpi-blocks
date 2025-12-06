from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='17',
            description='GPIO pin for PPM input (BCM numbering)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Rate at which Joy messages are published (Hz)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real GPIO'
        ),
        DeclareLaunchArgument(
            'num_channels',
            default_value='8',
            description='Number of PPM channels to decode'
        ),
        DeclareLaunchArgument(
            'frame_timeout_us',
            default_value='5000',
            description='Microseconds, pulse width threshold for end of frame'
        ),
        DeclareLaunchArgument(
            'min_pulse_us',
            default_value='900',
            description='Microseconds, typical minimum pulse width'
        ),
        DeclareLaunchArgument(
            'max_pulse_us',
            default_value='2100',
            description='Microseconds, typical maximum pulse width'
        ),
        DeclareLaunchArgument(
            'center_pulse_us',
            default_value='1500',
            description='Microseconds, typical center pulse width'
        ),
        
        Node(
            package='xpi_inputs',
            executable='ppm_receiver_node',
            name='ppm_receiver',
            parameters=[{
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                'num_channels': LaunchConfiguration('num_channels'),
                'frame_timeout_us': LaunchConfiguration('frame_timeout_us'),
                'min_pulse_us': LaunchConfiguration('min_pulse_us'),
                'max_pulse_us': LaunchConfiguration('max_pulse_us'),
                'center_pulse_us': LaunchConfiguration('center_pulse_us')
            }]
        )
    ])
