from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'llm_client_type',
            default_value='gemini',
            description='LLM backend: gemini, ollama, llamacpp, openai (any OpenAI-compatible server), qwen (DashScope), openrouter'
        ),
        DeclareLaunchArgument(
            'llm_api_key',
            default_value='',
            description='API key. Empty: the provider environment variable (GEMINI_API_KEY, DASHSCOPE_API_KEY, OPENROUTER_API_KEY...); prefer it, a parameter is visible in ros2 param get'
        ),
        DeclareLaunchArgument(
            'llm_model',
            default_value='',
            description='Model. Empty: the backend default (gemini-3.6-flash, Ollama qwen3.5:4b). E.g. gemma4:e4b, qwen3.5:9b, gemma-4-26b-a4b-it, qwen-plus'
        ),
        DeclareLaunchArgument(
            'ollama_host',
            default_value='http://localhost:11434',
            description='Host for Ollama server (if using ollama client)'
        ),
        DeclareLaunchArgument(
            'llm_base_url',
            default_value='',
            description='Server URL. Empty: the backend default (llama.cpp http://localhost:8080/v1). '
                        'E.g. http://192.168.1.20:8080/v1 for llama-server on another machine'
        ),
        DeclareLaunchArgument(
            'llm_think',
            default_value='off',
            description='Reasoning of Qwen 3.x / Gemma 4 (ollama, llamacpp, openai, qwen): auto, on, off (off is faster)'
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
                'llm_base_url': LaunchConfiguration('llm_base_url'),
                'llm_think': LaunchConfiguration('llm_think'),
                'llm_temperature': LaunchConfiguration('llm_temperature'),
                'llm_max_tokens': LaunchConfiguration('llm_max_tokens'),
                'command_topic': LaunchConfiguration('command_topic'),
                'response_topic': LaunchConfiguration('response_topic')
            }],
            output='screen',
            emulate_tty=True
        )
    ])
