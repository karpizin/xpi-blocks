import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import numpy as np
import pyaudio
try:
    import aubio
except ImportError:
    aubio = None

class BeatDetectorNode(Node):
    def __init__(self):
        super().__init__('beat_detector_node')
        
        # Parameters
        self.declare_parameter('device_index', -1) # -1 for default
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('buffer_size', 512)
        self.declare_parameter('hop_size', 256)
        
        rate = self.get_parameter('sample_rate').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.hop_size = self.get_parameter('hop_size').value
        
        # Publishers
        self.beat_pub = self.create_publisher(Float32, '/audio/beat', 10)
        self.is_beat_pub = self.create_publisher(Bool, '/audio/is_beat', 10)
        
        # Audio Setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=rate,
            input=True,
            input_device_index=self.get_parameter('device_index').value,
            frames_per_buffer=self.hop_size
        )
        
        # Aubio Setup
        if aubio:
            self.tempo = aubio.tempo("default", self.buffer_size, self.hop_size, rate)
            self.get_logger().info("Using Aubio for high-precision beat tracking.")
        else:
            self.get_logger().warn("Aubio not found. Falling back to energy-based detection.")
            self.prev_energy = 0
            
        # Timer for processing
        self.create_timer(0.001, self.process_audio)
        self.get_logger().info("Beat Detector Node started.")

    def process_audio(self):
        try:
            data = self.stream.read(self.hop_size, exception_on_overflow=False)
            samples = np.frombuffer(data, dtype=np.float32)
            
            is_beat = False
            confidence = 0.0
            
            if aubio:
                if self.tempo(samples):
                    is_beat = True
                    confidence = self.tempo.get_confidence()
            else:
                # Simple energy-based detection (fallback)
                energy = np.sum(samples**2) / len(samples)
                if energy > self.prev_energy * 1.5 and energy > 0.001:
                    is_beat = True
                    confidence = min(1.0, energy * 100)
                self.prev_energy = energy

            if is_beat:
                msg = Float32()
                msg.data = float(confidence)
                self.beat_pub.publish(msg)
                
                bool_msg = Bool()
                bool_msg.data = True
                self.is_beat_pub.publish(bool_msg)
                
        except Exception as e:
            self.get_logger().error(f"Audio error: {e}")

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BeatDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
