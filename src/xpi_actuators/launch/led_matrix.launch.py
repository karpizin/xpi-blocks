from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cascaded_devices',
            default_value='1',
            description='Number of MAX7219 devices cascaded (for larger displays)'
        ),
        DeclareLaunchArgument(
            'gpio_port',
            default_value='0', # SPI Port (0 or 1)
            description='SPI port number (0 for CE0, 1 for CE1)'
        ),
        DeclareLaunchArgument(
            'gpio_device',
            default_value='0', # SPI Device (CS0 or CS1)
            description='SPI device number (chip select)'
        ),
        DeclareLaunchArgument(
            'block_orientation',
            default_value='0', # 0, 90, -90, or 180 degrees.
            description='Orientation of individual 8x8 blocks'
        ),
        DeclareLaunchArgument(
            'rotate',
            default_value='0', # Rotation for each device (0, 1, 2, 3)
            description='Rotation of the entire display (0=0, 1=90, 2=180, 3=270)'
        ),
        DeclareLaunchArgument(
            'brightness',
            default_value='50', # 1 to 255
            description='Brightness of the display (1 to 255)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real MAX7219 hardware'
        ),
        
        Node(
            package='xpi_actuators',
            executable='led_matrix_node',
            name='led_matrix_display',
            parameters=[{
                'cascaded_devices': LaunchConfiguration('cascaded_devices'),
                'gpio_port': LaunchConfiguration('gpio_port'),
                'gpio_device': LaunchConfiguration('gpio_device'),
                'block_orientation': LaunchConfiguration('block_orientation'),
                'rotate': LaunchConfiguration('rotate'),
                'brightness': LaunchConfiguration('brightness'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
