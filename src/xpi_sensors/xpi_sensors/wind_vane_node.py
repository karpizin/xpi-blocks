import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import bisect

class WindVaneNode(Node):
    """
    ROS2 Node for an analog Wind Vane.
    Subscribes to voltage from an ADC (ADS1115) and converts it to degrees and cardinal directions.
    Default mapping is based on Misol/Fine Offset sensors with a 10k pull-up to 3.3V.
    """

    def __init__(self):
        super().__init__('wind_vane_node')

        # Parameters
        self.declare_parameter('input_voltage_topic', '/ads1115/voltage_ch0')
        self.declare_parameter('v_ref', 3.3) # Supply voltage
        
        topic = self.get_parameter('input_voltage_topic').value

        # Mapping: Voltage Threshold -> (Degrees, Name)
        # We use mid-points between ideal voltages for robust detection
        self.mapping = [
            (0.24, 112.5, "ESE"),
            (0.28, 67.5,  "ENE"),
            (0.35, 90.0,  "E"),
            (0.51, 157.5, "SSE"),
            (0.74, 135.0, "SE"),
            (0.94, 202.5, "SSW"),
            (1.16, 180.0, "S"),
            (1.40, 22.5,  "NNE"),
            (1.71, 45.0,  "NE"),
            (1.98, 247.5, "WSW"),
            (2.15, 225.0, "SW"),
            (2.39, 337.5, "NNW"),
            (2.60, 0.0,   "N"),
            (2.78, 292.5, "WNW"),
            (2.96, 315.0, "NW"),
            (3.17, 270.0, "W"),
        ]
        # Sort by voltage for bisect
        self.mapping.sort()
        self.thresholds = [m[0] for m in self.mapping]

        # Publishers
        self.dir_pub = self.create_publisher(Float32, '~/direction', 10)
        self.card_pub = self.create_publisher(String, '~/cardinal', 10)

        # Subscriber
        self.sub = self.create_subscription(Float32, topic, self.voltage_callback, 10)
        
        self.get_logger().info(f"Wind Vane initialized. Subscribing to {topic}")

    def voltage_callback(self, msg):
        v = msg.data
        
        # Find the closest mapping
        idx = bisect.bisect_left(self.thresholds, v)
        
        # Clip index to bounds
        if idx >= len(self.mapping):
            idx = len(self.mapping) - 1
        
        angle, name = self.mapping[idx][1], self.mapping[idx][2]

        # Publish
        msg_dir = Float32()
        msg_dir.data = float(angle)
        self.dir_pub.publish(msg_dir)

        msg_card = String()
        msg_card.data = name
        self.card_pub.publish(msg_card)

        self.get_logger().debug(f"V={v:.3f} -> {name} ({angle} deg)")

def main(args=None):
    rclpy.init(args=args)
    node = WindVaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
