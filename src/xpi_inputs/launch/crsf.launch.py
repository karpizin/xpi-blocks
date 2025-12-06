from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_port',
            default_value='/dev/ttyS0',
            description='UART port for CRSF receiver'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='420000', # Standard CRSF baud rate
            description='Baud rate for UART communication'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='100.0',
            description='Rate at which Joy messages are published (Hz)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real UART/CRSF receiver'
        ),
        
        Node(
            package='xpi_inputs',
            executable='crsf_receiver_node',
            name='crsf_receiver',
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }]
        )
    ])
