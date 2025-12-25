#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge
import cv2
import threading
import time
import os
from xpi_llm.llm_clients import llm_client_factory

class VLMObserverNode(Node):
    """
    ROS2 Node that periodically captures an image from the camera topic,
    sends it to a Vision-Language Model (VLM), and publishes the analysis.
    Also supports manual triggers via a topic.
    """

    def __init__(self):
        super().__init__('vlm_observer_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('analysis_interval', 60.0) # Seconds (default long for periodic)
        self.declare_parameter('provider', 'gemini') # gemini, openrouter, ollama
        self.declare_parameter('model', 'gemini-1.5-flash')
        self.declare_parameter('default_prompt', 'Describe the scene in front of the robot. Be concise.')
        self.declare_parameter('api_key', '')

        self.image_topic = self.get_parameter('image_topic').value
        self.interval = self.get_parameter('analysis_interval').value
        self.provider = self.get_parameter('provider').value
        self.model_name = self.get_parameter('model').value
        self.default_prompt = self.get_parameter('default_prompt').value
        
        # Get API Key
        self.api_key = self.get_parameter('api_key').value or os.environ.get('GEMINI_API_KEY')
        
        # Initialize LLM Client
        try:
            self.llm_client = llm_client_factory(
                self.provider, 
                api_key=self.api_key, 
                model=self.model_name
            )
            self.get_logger().info(f"VLM Client initialized: {self.provider} ({self.model_name})")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize VLM client: {e}")
            return

        # ROS Interfaces
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(String, '~/trigger', self.trigger_callback, 10)
        self.publisher_ = self.create_publisher(String, '/vlm/scene_description', 10)
        self.bridge = CvBridge()

        # State
        self.latest_image = None
        self.lock = threading.Lock()
        self.is_processing = False

        # Timer for periodic analysis
        if self.interval > 0:
            self.timer = self.create_timer(self.interval, self.timer_callback)
            self.get_logger().info(f"VLM Observer periodic analysis every {self.interval}s")

        self.get_logger().info(f"VLM Observer started. Listening on {self.image_topic}")

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = msg

    def trigger_callback(self, msg):
        """Handle manual trigger with optional custom prompt."""
        prompt = msg.data if msg.data else self.default_prompt
        self.get_logger().info(f"Manual VLM trigger received with prompt: '{prompt}'")
        self._request_analysis(prompt)

    def timer_callback(self):
        """Handle periodic analysis."""
        self._request_analysis(self.default_prompt)

    def _request_analysis(self, prompt):
        if self.is_processing:
            self.get_logger().warn("Skipping analysis: Previous request still processing.")
            return

        image_msg = None
        with self.lock:
            if self.latest_image:
                image_msg = self.latest_image
        
        if image_msg:
            threading.Thread(target=self.process_image, args=(image_msg, prompt)).start()
        else:
            self.get_logger().debug("No image received yet.")

    def process_image(self, msg, prompt):
        self.is_processing = True
        try:
            # Convert and compress
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded_img = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                self.get_logger().error("Failed to encode image to JPEG")
                return
            
            image_bytes = encoded_img.tobytes()

            # Call VLM
            self.get_logger().info(f"Sending image to VLM...")
            response, _ = self.llm_client.generate(prompt, image_data=image_bytes)

            self.get_logger().info(f"VLM Response: {response}")

            # Publish result
            result_msg = String()
            result_msg.data = response
            self.publisher_.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"Error during VLM processing: {e}")
        finally:
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = VLMObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()