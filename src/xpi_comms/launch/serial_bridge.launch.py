from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='text',
        description='Operation mode: "text" (lines) or "binary" (raw bytes)'
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        mode_arg,
        Node(
            package='xpi_comms',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'mode': LaunchConfiguration('mode')
            }],
            output='screen'
        )
    ])
