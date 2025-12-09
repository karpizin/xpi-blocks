import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'provider',
            default_value='gemini',
            description='VLM Provider: gemini, openrouter, ollama'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='gemini-1.5-flash',
            description='Model name (e.g. gemini-1.5-flash, gpt-4o)'
        ),
        DeclareLaunchArgument(
            'interval',
            default_value='30.0',
            description='Analysis interval in seconds'
        ),
        DeclareLaunchArgument(
            'api_key',
            default_value=os.environ.get('GEMINI_API_KEY', ''),
            description='API Key (defaults to env var)'
        ),

        Node(
            package='xpi_llm',
            executable='vlm_observer_node',
            name='vlm_observer',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'analysis_interval': LaunchConfiguration('interval'),
                'provider': LaunchConfiguration('provider'),
                'model': LaunchConfiguration('model'),
                'api_key': LaunchConfiguration('api_key'),
                'prompt': 'Describe the scene in front of the robot. List key objects and any potential hazards.'
            }]
        )
    ])
