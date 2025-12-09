#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
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
    """

    def __init__(self):
        super().__init__('vlm_observer_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('analysis_interval', 30.0) # Seconds
        self.declare_parameter('provider', 'gemini') # gemini, openrouter, ollama
        self.declare_parameter('model', 'gemini-1.5-flash')
        self.declare_parameter('prompt', 'List the main objects visible in this image. Return a comma-separated list.')
        self.declare_parameter('api_key', '') # Optional, preferably set via env var

        self.image_topic = self.get_parameter('image_topic').value
        self.interval = self.get_parameter('analysis_interval').value
        self.provider = self.get_parameter('provider').value
        self.model_name = self.get_parameter('model').value
        self.prompt = self.get_parameter('prompt').value
        
        # Get API Key from param or env
        self.api_key = self.get_parameter('api_key').value
        if not self.api_key:
            if self.provider == 'gemini':
                self.api_key = os.environ.get('GEMINI_API_KEY')
            elif self.provider == 'openrouter':
                self.api_key = os.environ.get('OPENROUTER_API_KEY')
        
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
            self.destroy_node()
            return

        # ROS Interfaces
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/vlm/detected_objects', 10)
        self.bridge = CvBridge()

        # State
        self.latest_image = None
        self.lock = threading.Lock()
        self.is_processing = False

        # Timer
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.get_logger().info(f"VLM Observer started. Listening on {self.image_topic}, analyzing every {self.interval}s")

    def image_callback(self, msg):
        """Buffer the latest image."""
        with self.lock:
            self.latest_image = msg

    def timer_callback(self):
        """Process the latest image."""
        if self.is_processing:
            self.get_logger().warn("Skipping analysis: Previous request still processing.")
            return

        image_msg = None
        with self.lock:
            if self.latest_image:
                image_msg = self.latest_image
                self.latest_image = None # Clear buffer to ensure freshness
        
        if image_msg:
            # Process in a separate thread to avoid blocking ROS spin
            thread = threading.Thread(target=self.process_image, args=(image_msg,))
            thread.start()
        else:
            self.get_logger().debug("No image received yet.")

    def process_image(self, msg):
        self.is_processing = True
        try:
            # Convert ROS Image to OpenCV -> JPEG -> Bytes
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Compress to JPEG to reduce size/tokens
            success, encoded_img = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                self.get_logger().error("Failed to encode image to JPEG")
                return
            
            image_bytes = encoded_img.tobytes()

            # Call VLM
            self.get_logger().info(f"Sending image to VLM ({self.provider})...")
            start_time = time.time()
            response, _ = self.llm_client.generate(self.prompt, image_data=image_bytes)
            latency = time.time() - start_time

            self.get_logger().info(f"VLM Response ({latency:.2f}s): {response}")

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
