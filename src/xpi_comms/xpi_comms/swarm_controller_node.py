#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import math

class SwarmControllerNode(Node):
    """
    Example node for swarm coordination.
    Listens to neighbors and ensures safe distances.
    """
    def __init__(self):
        super().__init__('swarm_controller_node')
        
        # 1. Parameters
        self.declare_parameter('safe_distance', 5.0) # meters
        self.safe_dist = self.get_parameter('safe_distance').value
        
        # 2. Subscriptions
        # Listens to neighbor data from the bridge
        self.create_subscription(String, '~/neighbors', self._neighbors_callback, 10)
        # Listens to local GPS state
        self.sub_local_gps = self.create_subscription(String, '/gps/state', self._local_gps_callback, 10)
        
        self.local_pos = None
        
        # 3. Control Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Swarm Controller started.')

    def _local_gps_callback(self, msg):
        try:
            self.local_pos = json.loads(msg.data)
        except: pass

    def _neighbors_callback(self, msg):
        if not self.local_pos: return
        
        try:
            neighbors = json.loads(msg.data)
            for nid, data in neighbors.items():
                dist = self.calculate_distance(
                    self.local_pos['lat'], self.local_pos['lon'],
                    data['lat'], data['lon']
                )
                
                if dist < self.safe_dist:
                    self.get_logger().warn(f'Collision risk with {nid}! Distance: {dist:.2f}m')
                    self.avoid_collision()
        except Exception as e:
            self.get_logger().error(f'Error in swarm logic: {e}')

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Earth radius in meters
        R = 6371000 
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def avoid_collision(self):
        """Sends a stop or turn command."""
        msg = Twist()
        msg.linear.x = -0.5 # Backward
        msg.angular.z = 1.0 # Turn
        self.cmd_pub.publish(msg)

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