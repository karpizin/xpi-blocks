from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='base'),
        DeclareLaunchArgument('threshold', default_value='0.01'),

        Node(
            package='xpi_audio',
            executable='whisper_stt_node.py',
            name='whisper_stt',
            parameters=[{
                'model_size': LaunchConfiguration('model'),
                'energy_threshold': LaunchConfiguration('threshold'),
            }],
            output='screen'
        )
    ])
