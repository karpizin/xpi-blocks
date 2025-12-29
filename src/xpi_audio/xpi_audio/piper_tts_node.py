#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import tempfile

class PiperTtsNode(Node):
    """
    Local Text-to-Speech node using the Piper neural TTS engine.
    Subscribes to ~/say (String) and plays audio via system's default output.
    """
    def __init__(self):
        super().__init__('piper_tts_node')

        # 1. Parameters
        self.declare_parameter('model_path', '/usr/share/piper/voices/en_US-amy-low.onnx')
        self.declare_parameter('config_path', '') # Usually auto-detected if next to .onnx
        self.declare_parameter('piper_bin', 'piper')
        self.declare_parameter('speed', 1.0)
        
        self.model_path = self.get_parameter('model_path').value
        self.piper_bin = self.get_parameter('piper_bin').value
        self.speed = self.get_parameter('speed').value

        # 2. Subscription
        self.create_subscription(String, '~/say', self.say_callback, 10)

        # 3. Check for piper availability
        if not self.check_piper():
            self.get_logger().error(f"Piper binary '{self.piper_bin}' not found! Install it or set 'piper_bin' parameter.")

        self.get_logger().info(f"Piper TTS Node initialized. Using model: {os.path.basename(self.model_path)}")

    def check_piper(self):
        try:
            subprocess.run([self.piper_bin, '--version'], capture_output=True)
            return True
        except FileNotFoundError:
            return False

    def say_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Speaking: '{text}'")
        
        try:
            # We pipe text to piper, and its output to aplay
            # Command: echo "text" | piper --model model.onnx --output_raw | aplay -r 22050 -f S16_LE -t raw
            
            # Using Popen for streaming performance
            echo_process = subprocess.Popen(['echo', text], stdout=subprocess.PIPE)
            
            piper_cmd = [
                self.piper_bin, 
                '--model', self.model_path,
                '--length_scale', str(1.0 / self.speed),
                '--output_raw'
            ]
            
            piper_process = subprocess.Popen(piper_cmd, stdin=echo_process.stdout, stdout=subprocess.PIPE)
            
            # aplay settings depend on the Piper model (usually 22050Hz)
            aplay_cmd = ['aplay', '-r', '22050', '-f', 'S16_LE', '-t', 'raw']
            aplay_process = subprocess.Popen(aplay_cmd, stdin=piper_process.stdout)
            
            aplay_process.wait()
            
        except Exception as e:
            self.get_logger().error(f"Failed to play TTS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PiperTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
