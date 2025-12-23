from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA0',
            description='Serial port (e.g., /dev/ttyAMA0 for Pi UART, /dev/ttyUSB0 for USB adapter)'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='9600',
            description='Baudrate (PMS5003 uses 9600)'
        ),
        DeclareLaunchArgument(
            'polling_rate',
            default_value='1.0',
            description='Polling rate in Hz'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='pms5003_link',
            description='Frame ID for TF'
        ),
        
        Node(
            package='xpi_sensors',
            executable='pms5003_node',
            name='pms5003_air_quality',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'polling_rate': LaunchConfiguration('polling_rate'),
                'frame_id': LaunchConfiguration('frame_id')
            }],
            output='screen'
        )
    ])
