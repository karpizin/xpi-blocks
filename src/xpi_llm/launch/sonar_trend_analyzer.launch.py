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
            'history_size',
            default_value='20',
            description='Number of sonar readings to keep in history for analysis'
        ),
        DeclareLaunchArgument(
            'analysis_interval_sec',
            default_value='5.0',
            description='Interval in seconds to perform LLM analysis'
        ),
        DeclareLaunchArgument(
            'llm_base_url',
            default_value='',
            description='Server URL. Empty: the backend default (llama.cpp http://localhost:8080/v1). '
                        'E.g. http://192.168.1.20:8080/v1 for llama-server on another machine'
        ),
        DeclareLaunchArgument(
            'llm_think',
            default_value='auto',
            description='Reasoning of Qwen 3.x / Gemma 4 (ollama, llamacpp, openai, qwen): auto, on, off (off is faster)'
        ),
        DeclareLaunchArgument(
            'llm_temperature',
            default_value='0.5',
            description='Temperature for LLM generation (creativity)'
        ),
        DeclareLaunchArgument(
            'llm_max_tokens',
            default_value='100',
            description='Maximum tokens for LLM response'
        ),
        DeclareLaunchArgument(
            'sonar_topic',
            default_value='/sonar_front/range',
            description='Input topic for sonar range data'
        ),
        
        Node(
            package='xpi_llm',
            executable='sonar_trend_analyzer_node',
            name='sonar_trend_analyzer',
            parameters=[{
                'llm_client_type': LaunchConfiguration('llm_client_type'),
                'llm_api_key': LaunchConfiguration('llm_api_key'),
                'llm_model': LaunchConfiguration('llm_model'),
                'ollama_host': LaunchConfiguration('ollama_host'),
                'llm_base_url': LaunchConfiguration('llm_base_url'),
                'llm_think': LaunchConfiguration('llm_think'),
                'history_size': LaunchConfiguration('history_size'),
                'analysis_interval_sec': LaunchConfiguration('analysis_interval_sec'),
                'llm_temperature': LaunchConfiguration('llm_temperature'),
                'llm_max_tokens': LaunchConfiguration('llm_max_tokens'),
                'sonar_topic': LaunchConfiguration('sonar_topic')
            }],
            # Required to pass environment variables like API keys from current shell to node
            emulate_tty=True
        )
    ])
