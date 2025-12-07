from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C Bus Number (usually 1 for Raspberry Pi)'
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x3C', # Common SSD1306 address
            description='I2C Address of the SSD1306 OLED display'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='128', # Display width
            description='Display width in pixels (e.g., 128)'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='64', # Display height
            description='Display height in pixels (e.g., 64, 32)'
        ),
        DeclareLaunchArgument(
            'rotation',
            default_value='0', # Rotation (0, 90, 180, 270)
            description='Display rotation in degrees (0, 90, 180, 270)'
        ),
        DeclareLaunchArgument(
            'font_size',
            default_value='10',
            description='Default font size for text display'
        ),
        DeclareLaunchArgument(
            'default_text',
            default_value='Hello XPI!',
            description='Text to display on the OLED at startup'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real SSD1306 hardware'
        ),
        
        Node(
            package='xpi_actuators',
            executable='ssd1306_node',
            name='oled_display',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'rotation': LaunchConfiguration('rotation'),
                'font_size': LaunchConfiguration('font_size'),
                'default_text': LaunchConfiguration('default_text'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
