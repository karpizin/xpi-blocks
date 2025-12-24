import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from .llm_clients import llm_client_factory
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import io
import os
import threading

class AudioPatternAnalyzer(Node):
    def __init__(self):
        super().__init__('audio_pattern_analyzer')
        
        # Parameters
        self.declare_parameter('llm_provider', 'gemini')
        self.declare_parameter('llm_model', 'gemini-1.5-flash')
        self.declare_parameter('api_key', '')
        self.declare_parameter('db_threshold', -30.0) # dB level to trigger analysis
        self.declare_parameter('record_duration', 3.0) # seconds
        self.declare_parameter('cooldown', 10.0) # seconds between analysis
        self.declare_parameter('sample_rate', 16000)

        provider = self.get_parameter('llm_provider').value
        model = self.get_parameter('llm_model').value
        api_key = self.get_parameter('api_key').value or os.environ.get('GEMINI_API_KEY') or os.environ.get('OPENROUTER_API_KEY')
        
        self.threshold = self.get_parameter('db_threshold').value
        self.record_duration = self.get_parameter('record_duration').value
        self.cooldown = self.get_parameter('cooldown').value
        self.fs = self.get_parameter('sample_rate').value

        if not api_key:
            self.get_logger().error("API Key not found. Analysis will fail.")

        # LLM Client
        self.llm = llm_client_factory(provider, api_key=api_key, model=model)

        # State
        self.is_recording = False
        self.last_analysis_time = 0.0
        
        # Publishers
        self.class_pub = self.create_publisher(String, '~/classification', 10)
        self.summary_pub = self.create_publisher(String, '~/summary', 10)

        # Subscriber to real-time dB level
        self.db_sub = self.create_subscription(Float32, 'audio_level/db', self.db_callback, 10)

        self.get_logger().info(f"Audio Pattern Analyzer initialized using {provider}:{model}")

    def db_callback(self, msg):
        """Triggers recording if threshold is exceeded and cooldown passed."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if msg.data > self.threshold and not self.is_recording:
            if current_time - self.last_analysis_time > self.cooldown:
                self.get_logger().info(f"Noise threshold exceeded ({msg.data:.1f} dB). Starting recording...")
                threading.Thread(target=self.record_and_analyze).start()

    def record_and_analyze(self):
        """Records a snippet of audio and sends it to LLM."""
        self.is_recording = True
        try:
            # Record audio
            recording = sd.rec(int(self.record_duration * self.fs), samplerate=self.fs, channels=1)
            sd.wait() # Wait until recording is finished
            
            # Convert to WAV in memory
            buffer = io.BytesIO()
            wav.write(buffer, self.fs, recording)
            audio_bytes = buffer.getvalue()

            self.get_logger().info("Recording finished. Sending to LLM for analysis...")

            prompt = (
                "Identify the main sound in this audio clip. Categorize it (e.g., Speech, Music, Dog, Alarm, Background Noise). "
                "Provide a very short category name followed by a brief description. "
                "Respond in format: Category: [Name] | Description: [Text]"
            )

            # Use LLM to analyze (MIME type for WAV is audio/wav)
            response, _ = self.llm.generate(prompt, media_data=audio_bytes, mime_type="audio/wav")

            if response:
                # Publish Classification
                class_msg = String()
                if "|" in response:
                    class_msg.data = response.split("|")[0].replace("Category:", "").strip()
                else:
                    class_msg.data = response[:50]
                self.class_pub.publish(class_msg)

                # Publish Full Summary
                summary_msg = String()
                summary_msg.data = response
                self.summary_pub.publish(summary_msg)
                
                self.get_logger().info(f"Analysis Complete: {response}")

        except Exception as e:
            self.get_logger().error(f"Failed to record or analyze: {e}")
        
        finally:
            self.is_recording = False
            self.last_analysis_time = self.get_clock().now().nanoseconds / 1e9

def main(args=None):
    rclpy.init(args=args)
    node = AudioPatternAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
