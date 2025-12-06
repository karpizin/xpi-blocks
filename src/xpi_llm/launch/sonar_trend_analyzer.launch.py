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
