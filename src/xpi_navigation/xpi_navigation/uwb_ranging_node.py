#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random # For fallback simulator

class UWBRangingNode(Node):
    def __init__(self):
        super().__init__('uwb_ranging_node')
        
        self.declare_parameter('port', 'SPI0')
        self.declare_parameter('simulate', False)
        self.simulate = self.get_parameter('simulate').value
        
        self.pub = self.create_publisher(String, '~/raw_ranges', 10)
        self.timer = self.create_timer(0.5, self.poll_uwb)
        self.get_logger().info(f"UWB Ranging Node Started (Simulate={self.simulate})")

    def poll_uwb(self):
        # In real hardware, here we talk to DWM1000 via SPI
        # result = {"beacon_1": 2.45, "beacon_2": 5.12}
        
        if self.simulate:
            # Generate dummy data for testing the SLAM engine
            # 3 beacons at [0,5], [5,0], [5,5]
            beacons = {
                "1": [0, 5],
                "2": [5, 0],
                "3": [5, 5]
            }
            # Simulating robot moving? Let's just output distances from real 0,0
            # with some noise
            ranges = {}
            for b_id, b_pos in beacons.items():
                dist = np.linalg.norm(np.array([0,0]) - np.array(b_pos)) + random.uniform(-0.1, 0.1)
                ranges[b_id] = dist
        else:
            # Placeholder for SPI logic
            ranges = {} # Fill with real data
            
        msg = String()
        msg.data = json.dumps(ranges)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UWBRangingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
