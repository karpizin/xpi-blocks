from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'led_count',
            default_value='30',
            description='Number of LED pixels in the strip'
        ),
        DeclareLaunchArgument(
            'led_pin',
            default_value='18', # BCM GPIO pin (18 uses PWM, 10 uses PCM)
            description='GPIO pin connected to the pixels (BCM numbering)'
        ),
        DeclareLaunchArgument(
            'led_freq_hz',
            default_value='800000', # 800kHz
            description='LED signal frequency in hertz (usually 800khz)'
        ),
        DeclareLaunchArgument(
            'led_dma',
            default_value='10', # DMA channel to use
            description='DMA channel to use for generating signal'
        ),
        DeclareLaunchArgument(
            'led_brightness',
            default_value='255', # 0-255
            description='Brightness (0 for darkest and 255 for brightest)'
        ),
        DeclareLaunchArgument(
            'led_invert',
            default_value='false',
            description='Invert the signal (when using NPN transistor level shift)'
        ),
        DeclareLaunchArgument(
            'led_channel',
            default_value='0', # Set to '1' for GPIOs 13, 19, 40, 52.
            description='PWM channel to use (0 or 1)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real WS2812 hardware'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='30.0',
            description='Max rate in Hz to update LEDs (internal driver processing rate)'
        ),
        
        Node(
            package='xpi_actuators',
            executable='ws2812_driver_node',
            name='ws2812_strip_driver',
            parameters=[{
                'led_count': LaunchConfiguration('led_count'),
                'led_pin': LaunchConfiguration('led_pin'),
                'led_freq_hz': LaunchConfiguration('led_freq_hz'),
                'led_dma': LaunchConfiguration('led_dma'),
                'led_brightness': LaunchConfiguration('led_brightness'),
                'led_invert': LaunchConfiguration('led_invert'),
                'led_channel': LaunchConfiguration('led_channel'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                'update_rate': LaunchConfiguration('update_rate')
            }],
            output='screen'
        )
    ])
