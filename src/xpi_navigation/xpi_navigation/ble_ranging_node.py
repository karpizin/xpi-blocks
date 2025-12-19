#!/usr/bin/env python3
import asyncio
import json
import math
from bleak import BleakScanner
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BLERangingNode(Node):
    def __init__(self):
        super().__init__('ble_ranging_node')
        
        # Parameters
        self.declare_parameter('scan_duration', 1.0)
        self.declare_parameter('measured_power', -59) # RSSI at 1 meter
        self.declare_parameter('environmental_factor', 2.5) # N (2.0-4.0)
        self.declare_parameter('name_prefix', 'ROBOT_BEACON') # Only listen to these
        
        self.scan_duration = self.get_parameter('scan_duration').value
        self.tx_power = self.get_parameter('measured_power').value
        self.n = self.get_parameter('environmental_factor').value
        self.prefix = self.get_parameter('name_prefix').value

        self.pub = self.create_publisher(String, '~/raw_ranges', 10)
        
        # Start the async loop in a separate thread or use a timer to trigger async task
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info(f"BLE Ranging Node Started (Filter: {self.prefix}*)")

    async def perform_scan(self):
        try:
            # Discover devices
            devices = await BleakScanner.discover(timeout=self.scan_duration)
            
            ranges = {}
            for d in devices:
                # Filter by name or other criteria
                if d.name and d.name.startswith(self.prefix):
                    # RSSI to Distance
                    dist = 10**((self.tx_power - d.rssi) / (10 * self.n))
                    ranges[d.address] = round(dist, 2)
            
            if ranges:
                msg = String()
                msg.data = json.dumps(ranges)
                self.pub.publish(msg)
                # self.get_logger().info(f"Found {len(ranges)} beacons")
                
        except Exception as e:
            self.get_logger().warn(f"BLE Scan failed: {e}")

    def timer_callback(self):
        # Trigger the async scan
        asyncio.run(self.perform_scan())

def main(args=None):
    rclpy.init(args=args)
    node = BLERangingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
