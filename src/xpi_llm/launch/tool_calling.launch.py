import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'llm_client_type',
            default_value='gemini',
            description='Type of LLM client (openrouter, gemini, ollama)'
        ),
        DeclareLaunchArgument(
            'llm_api_key',
            default_value=os.environ.get('GEMINI_API_KEY', ''), # Default to env var for Gemini
            description='API Key for the chosen LLM service (e.g., OPENROUTER_API_KEY or GEMINI_API_KEY)'
        ),
        DeclareLaunchArgument(
            'llm_model',
            default_value='gemini-pro',
            description='Specific LLM model to use (e.g., gemini-pro, openai/gpt-3.5-turbo, llama2)'
        ),
        DeclareLaunchArgument(
            'ollama_host',
            default_value='http://localhost:11434',
            description='Host for Ollama server (if using ollama client)'
        ),
        DeclareLaunchArgument(
            'llm_temperature',
            default_value='0.1', # Keep low for reliable tool calls
            description='Temperature for LLM generation (creativity)'
        ),
        DeclareLaunchArgument(
            'llm_max_tokens',
            default_value='200',
            description='Maximum tokens for LLM response'
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='~/command',
            description='Input topic for user text commands'
        ),
        DeclareLaunchArgument(
            'response_topic',
            default_value='~/response',
            description='Output topic for LLM textual responses and tool execution results'
        ),
        
        Node(
            package='xpi_llm',
            executable='tool_calling_node',
            name='llm_controller',
            parameters=[{
                'llm_client_type': LaunchConfiguration('llm_client_type'),
                'llm_api_key': LaunchConfiguration('llm_api_key'),
                'llm_model': LaunchConfiguration('llm_model'),
                'ollama_host': LaunchConfiguration('ollama_host'),
                'llm_temperature': LaunchConfiguration('llm_temperature'),
                'llm_max_tokens': LaunchConfiguration('llm_max_tokens'),
                'command_topic': LaunchConfiguration('command_topic'),
                'response_topic': LaunchConfiguration('response_topic')
            }],
            output='screen',
            emulate_tty=True
        )
    ])
