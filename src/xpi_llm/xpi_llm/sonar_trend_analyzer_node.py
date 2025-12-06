import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from xpi_llm.llm_clients import llm_client_factory, LLMClient
import numpy as np
import os
import threading
import time
import math

class SonarTrendAnalyzerNode(Node):
    """
    ROS2 Node that analyzes trends in sonar data using an LLM.
    Subscribes to sensor_msgs/Range and publishes std_msgs/String with LLM analysis.
    """

    def __init__(self):
        super().__init__('sonar_trend_analyzer_node')

        # 1. Declare Parameters
        self.declare_parameter('llm_client_type', 'gemini') # openrouter, gemini, ollama
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('llm_model', 'gemini-pro')
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('history_size', 20) # Number of sonar readings to keep
        self.declare_parameter('analysis_interval_sec', 5.0) # How often to analyze
        self.declare_parameter('llm_temperature', 0.5)
        self.declare_parameter('llm_max_tokens', 100)
        self.declare_parameter('sonar_topic', '/sonar_front/range')

        # 2. Read Parameters
        self.llm_client_type = self.get_parameter('llm_client_type').value
        self.llm_api_key = self.get_parameter('llm_api_key').value
        self.llm_model = self.get_parameter('llm_model').value
        self.ollama_host = self.get_parameter('ollama_host').value
        self.history_size = self.get_parameter('history_size').value
        self.analysis_interval = self.get_parameter('analysis_interval_sec').value
        self.llm_temperature = self.get_parameter('llm_temperature').value
        self.llm_max_tokens = self.get_parameter('llm_max_tokens').value
        self.sonar_topic = self.get_parameter('sonar_topic').value

        # 3. Initialize LLM Client
        self.llm_client: LLMClient = None
        try:
            if self.llm_client_type == 'ollama':
                self.llm_client = llm_client_factory(
                    self.llm_client_type, host=self.ollama_host, model=self.llm_model
                )
            else:
                self.llm_client = llm_client_factory(
                    self.llm_client_type, api_key=self.llm_api_key, model=self.llm_model
                )
            self.get_logger().info(f'LLM Client initialized: {self.llm_client.get_model_name()}')
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize LLM client: {e}. ')
            self.get_logger().error('Please check client type, API key, and model parameters.')
            self.llm_client = None
        except ImportError as e:
            self.get_logger().error(f'Missing LLM library: {e}. ')
            self.get_logger().error('Please install it (e.g., pip install google-generativeai or openai).')
            self.llm_client = None

        self.sonar_history = []
        self.history_lock = threading.Lock() # Protect shared history access

        # 4. Subscribers and Publishers
        self.range_subscription = self.create_subscription(
            Range,
            self.sonar_topic,
            self.range_callback,
            10
        )
        self.analysis_publisher = self.create_publisher(String, '~/sonar_trend', 10)

        # 5. Analysis Timer
        self.analysis_timer = self.create_timer(self.analysis_interval, self.analyze_trend_callback)
        self.get_logger().info(f'Subscribing to {self.sonar_topic} for sonar data.')

    def range_callback(self, msg: Range):
        """Callback for incoming sonar range data."""
        if math.isinf(msg.range): # Ignore infinite range readings
            return

        with self.history_lock:
            self.sonar_history.append((self.get_clock().now().nanoseconds / 1e9, msg.range))
            if len(self.sonar_history) > self.history_size:
                self.sonar_history.pop(0) # Keep history size limited

    def analyze_trend_callback(self):
        """Callback to trigger LLM analysis of sonar trends."""
        if not self.llm_client:
            self.get_logger().warn('LLM client not initialized, skipping trend analysis.')
            return

        with self.history_lock:
            if len(self.sonar_history) < 2:
                # self.get_logger().info('Not enough sonar data for trend analysis.')
                return
            
            # Create a copy to avoid holding the lock during LLM call
            current_history = list(self.sonar_history) 
        
        # Format history for LLM prompt
        # Convert nanoseconds timestamp to relative seconds
        start_time = current_history[0][0]
        formatted_readings = [f"at {t - start_time:.2f}s: {r:.2f}m" for t, r in current_history]
        
        prompt = (
            f"You are an AI assistant analyzing robot sonar data. "
            f"The robot measures distance to objects in meters. "
            f"Analyze the following sequence of sonar readings and describe the trend. "
            f"Example trends: 'approaching an object', 'moving away', 'stationary', 'moving parallel to a wall', 'object appeared/disappeared'.\n"
            f"Sonar readings ({len(formatted_readings)} points):\n"
            f"{'; '.join(formatted_readings)}\n"
            f"What is the trend observed? Be concise and do not include probabilities or confidence scores. Start directly with the trend."
        )

        # Offload LLM call to a separate thread to not block ROS2 spin
        thread = threading.Thread(target=self._call_llm_for_analysis, args=(prompt,))
        thread.start()

    def _call_llm_for_analysis(self, prompt: str):
        """Internal method to call LLM, run in a separate thread."""
        try:
            self.get_logger().debug(f"Sending prompt to LLM: {prompt}")
            llm_response = self.llm_client.generate(
                prompt,
                temperature=self.llm_temperature,
                max_tokens=self.llm_max_tokens
            )
            if llm_response:
                self.get_logger().info(f'LLM Analysis: {llm_response}')
                msg = String()
                msg.data = llm_response
                self.analysis_publisher.publish(msg)
            else:
                self.get_logger().warn('LLM returned an empty response.')
        except Exception as e:
            self.get_logger().error(f'Error during LLM analysis: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SonarTrendAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
