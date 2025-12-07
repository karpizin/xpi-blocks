from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'clk_pin',
            default_value='23', # BCM GPIO pin for CLK
            description='GPIO pin connected to the TM1637 CLK line'
        ),
        DeclareLaunchArgument(
            'dio_pin',
            default_value='24', # BCM GPIO pin for DIO
            description='GPIO pin connected to the TM1637 DIO line'
        ),
        DeclareLaunchArgument(
            'brightness',
            default_value='7', # 0-7
            description='Display brightness (0 for dimmest, 7 for brightest)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real TM1637 hardware'
        ),
        DeclareLaunchArgument(
            'display_digits',
            default_value='4', # 4 or 6
            description='Number of digits on your TM1637 display (e.g., 4 or 6)'
        ),
        DeclareLaunchArgument(
            'default_text',
            default_value='----',
            description='Text to display on the TM1637 at startup'
        ),
        
        Node(
            package='xpi_actuators',
            executable='tm1637_node',
            name='tm1637_display',
            parameters=[{
                'clk_pin': LaunchConfiguration('clk_pin'),
                'dio_pin': LaunchConfiguration('dio_pin'),
                'brightness': LaunchConfiguration('brightness'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                'display_digits': LaunchConfiguration('display_digits'),
                'default_text': LaunchConfiguration('default_text')
            }],
            output='screen'
        )
    ])
