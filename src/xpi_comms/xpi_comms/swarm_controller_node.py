#!/usr/bin/env python3
"""
Simple Swarm Controller.
Adjusts local robot instructions based on neighbor telemetry.
"""
import rclpy
from rclpy.node import Node
import json
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist # For velocity/heading control

class SwarmControllerNode(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # 1. Parameters
        self.declare_parameter('safe_distance', 5.0) # meters
        self.safe_distance = self.get_parameter('safe_distance').value

        # 2. Subscriptions
        self.sub_neighbors = self.create_subscription(String, '/meshtastic_bridge/neighbors', self._neighbors_callback, 10)
        self.sub_local_gps = self.create_subscription(String, '/gps/state', self._local_gps_callback, 10) # Simplified

        # 3. Control Publication
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.local_pos = None
        self.get_logger().info("Swarm Controller started. Monitoring neighbors for safety...")

    def _local_gps_callback(self, msg):
        self.local_pos = json.loads(msg.data)

    def _neighbors_callback(self, msg):
        if not self.local_pos:
            return

        neighbor_data = json.loads(msg.data)
        n_pos = neighbor_data.get('data', {})
        
        # Calculate distance (simplified, converted from degrees to meters)
        # In a real system, the Haversine formula should be used
        dist = self._calculate_distance(
            self.local_pos.get('lat'), self.local_pos.get('lon'),
            n_pos.get('lat'), n_pos.get('lon')
        )

        if dist < self.safe_distance:
            self.get_logger().warn(f"COLLISION ALERT! Neighbor {neighbor_data['node_id']} is too close ({dist:.2f}m)")
            self._apply_avoidance_logic()

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculates distance between two points in meters using Haversine formula."""
        if None in [lat1, lon1, lat2, lon2]: return 1000.0
        R = 6371000 # Earth radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def _apply_avoidance_logic(self):
        """Sends a stop or turn command."""
        msg = Twist()
        msg.linear.x = -0.5 # Move backward
        msg.angular.z = 1.0 # Turn
        self.pub_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
