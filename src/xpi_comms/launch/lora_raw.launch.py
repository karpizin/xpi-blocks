from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frequency', default_value='433.0'),
        DeclareLaunchArgument('spreading_factor', default_value='7'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_comms',
            executable='lora_raw_node',
            name='lora_raw',
            namespace='lora',
            parameters=[{
                'frequency': LaunchConfiguration('frequency'),
                'spreading_factor': LaunchConfiguration('spreading_factor'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
