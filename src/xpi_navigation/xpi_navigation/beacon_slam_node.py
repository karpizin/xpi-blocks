#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.optimize import minimize
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import String # For simplified map output in v1
import json

class BeaconSLAMNode(Node):
    def __init__(self):
        super().__init__('beacon_slam_node')
        
        # State: Beacon positions {ID: [x, y]}
        self.beacons = {}
        # History of measurements [(robot_x, robot_y, beacon_id, distance)]
        self.history = []
        
        # Robot position (estimated)
        self.robot_pos = np.array([0.0, 0.0])
        
        # Sub/Pub
        self.create_subscription(String, '~/raw_ranges', self.cb_ranges, 10)
        # We also need odom to know how robot moves between pings
        self.create_subscription(PoseWithCovarianceStamped, '/odom', self.cb_odom, 10)
        
        self.map_pub = self.create_publisher(String, '~/beacon_map', 10)
        
        # Timer for optimization (every 5 seconds)
        self.timer = self.create_timer(5.0, self.optimize_map)
        self.get_logger().info("Beacon SLAM Engine Started (Auto-Discovery Mode)")

    def cb_odom(self, msg):
        # Update current robot position guess from odometry
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y

    def cb_ranges(self, msg):
        # Expected input: JSON string '{"1": 2.5, "2": 4.1}' (ID: Distance)
        try:
            ranges = json.loads(msg.data)
            for b_id, dist in ranges.items():
                b_id = str(b_id)
                
                # If new beacon, initialize its position roughly
                if b_id not in self.beacons:
                    # Place it north of robot for now (initial guess)
                    self.beacons[b_id] = self.robot_pos + np.array([0.0, float(dist)])
                    self.get_logger().info(f"Discovered new beacon {b_id}")
                
                # Save to history for optimization
                self.history.append((self.robot_pos.copy(), b_id, float(dist)))
                
                # Limit history size to prevent slow down
                if len(self.history) > 500:
                    self.history.pop(0)
                    
        except Exception as e:
            self.get_logger().warn(f"Error parsing ranges: {e}")

    def optimize_map(self):
        if not self.history or len(self.beacons) < 2:
            return

        # Problem: Find [x1, y1, x2, y2...] for all beacons 
        # that minimizes SUM( (dist_calc - dist_meas)^2 )
        
        beacon_ids = list(self.beacons.keys())
        initial_guess = []
        for bid in beacon_ids:
            initial_guess.extend(self.beacons[bid])
            
        def error_func(coords):
            # unpack coords
            b_pos = {}
            for i, bid in enumerate(beacon_ids):
                b_pos[bid] = coords[i*2:i*2+2]
            
            total_error = 0
            for r_pos, bid, dist_meas in self.history:
                dist_calc = np.linalg.norm(r_pos - b_pos[bid])
                total_error += (dist_calc - dist_meas)**2
            return total_error

        # Run Optimization (Nelder-Mead is robust)
        res = minimize(error_func, initial_guess, method='Nelder-Mead')
        
        if res.success:
            # Update beacon positions
            new_coords = res.x
            for i, bid in enumerate(beacon_ids):
                self.beacons[bid] = new_coords[i*2:i*2+2]
            
            # Publish updated map
            map_msg = String()
            map_msg.data = json.dumps({k: list(v) for k, v in self.beacons.items()})
            self.map_pub.publish(map_msg)
            # self.get_logger().info("Map Optimized")

def main(args=None):
    rclpy.init(args=args)
    node = BeaconSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
