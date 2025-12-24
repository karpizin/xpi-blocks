import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sounddevice as sd
import numpy as np

class AudioLevelNode(Node):
    def __init__(self):
        super().__init__('audio_level_node')
        
        # Parameters
        self.declare_parameter('device_index', -1) # Default device
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('block_size', 1600) # 100ms blocks
        
        device = self.get_parameter('device_index').value
        if device == -1: device = None
        
        self.fs = self.get_parameter('sample_rate').value
        self.block_size = self.get_parameter('block_size').value

        # Publishers
        self.db_pub = self.create_publisher(Float32, '~/db', 10)
        self.rms_pub = self.create_publisher(Float32, '~/rms', 10)

        # Start Stream
        try:
            self.stream = sd.InputStream(
                device=device,
                channels=1,
                samplerate=self.fs,
                blocksize=self.block_size,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info(f"Audio stream started on device {device if device else 'default'}")
        except Exception as e:
            self.get_logger().error(f"Failed to start audio stream: {e}")

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(str(status))
        
        # Calculate RMS
        rms = np.sqrt(np.mean(indata**2))
        
        # Calculate dB (relative to 1.0)
        # We add a tiny epsilon to avoid log10(0)
        db = 20 * np.log10(max(rms, 1e-6))

        # Publish
        msg_rms = Float32()
        msg_rms.data = float(rms)
        self.rms_pub.publish(msg_rms)

        msg_db = Float32()
        msg_db.data = float(db)
        self.db_pub.publish(msg_db)

def main(args=None):
    rclpy.init(args=args)
    node = AudioLevelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
