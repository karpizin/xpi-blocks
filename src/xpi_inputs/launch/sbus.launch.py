from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_port',
            default_value='/dev/ttyS0',
            description='UART port for SBUS receiver'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='100000', # Standard SBUS baud rate
            description='Baud rate for UART communication'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Rate at which Joy messages are published (Hz)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real UART/SBUS receiver'
        ),
        DeclareLaunchArgument(
            'invert_sbus',
            default_value='false',
            description='Invert SBUS data if receiver needs it (e.g. some inverters)'
        ),
        
        Node(
            package='xpi_inputs',
            executable='sbus_receiver_node',
            name='sbus_receiver',
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                'invert_sbus': LaunchConfiguration('invert_sbus')
            }]
        )
    ])
