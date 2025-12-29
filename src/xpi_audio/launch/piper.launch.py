from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='/usr/share/piper/voices/en_US-amy-low.onnx'),
        DeclareLaunchArgument('speed', default_value='1.0'),

        Node(
            package='xpi_audio',
            executable='piper_tts_node.py',
            name='piper_tts',
            parameters=[{
                'model_path': LaunchConfiguration('model'),
                'speed': LaunchConfiguration('speed'),
            }],
            output='screen'
        )
    ])
