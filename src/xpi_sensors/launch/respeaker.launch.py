from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'update_rate',
            default_value='0.1',
            description='Polling rate for DOA/VAD in seconds'
        ),
        Node(
            package='xpi_sensors',
            executable='respeaker_node',
            name='respeaker',
            namespace='respeaker',
            parameters=[{
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            output='screen'
        )
    ])
