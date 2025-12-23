#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
import json
import yaml
import numpy as np
from scipy.optimize import least_squares
import os

class BLESLAMNode(Node):
    """
    ROS2 Node for BLE Localization and Mapping.
    Modes:
    1. LOCALIZATION: Uses known beacon positions from YAML to find robot pose.
    2. MAPPING (Experimental): Estimates beacon positions relative to robot start (requires Odometry).
    """
    def __init__(self):
        super().__init__('ble_slam_node')

        # Parameters
        self.declare_parameter('mode', 'LOCALIZATION') # LOCALIZATION or MAPPING
        self.declare_parameter('beacons_file', '') # Path to YAML
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.mode = self.get_parameter('mode').value
        beacons_file = self.get_parameter('beacons_file').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # State
        self.known_beacons = {} # MAC -> [x, y, z]
        self.robot_pose = np.array([0.0, 0.0, 0.0]) # x, y, z
        self.last_ranges = {} # MAC -> dist
        
        # MAPPING State
        self.mapped_beacons = {} # MAC -> [x, y, z] estimated
        self.robot_trajectory = [] # List of (x,y) from odom

        # Load Map
        if self.mode == 'LOCALIZATION':
            if os.path.exists(beacons_file):
                with open(beacons_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if 'beacons' in data:
                        self.known_beacons = data['beacons']
                        self.get_logger().info(f"Loaded {len(self.known_beacons)} beacons from map.")
            else:
                self.get_logger().error(f"Beacons file not found: {beacons_file}")

        # Subs/Pubs
        self.create_subscription(String, '/ble_ranging_node/raw_ranges', self._ranges_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '~/pose', 10)
        
        # For visualization of solved beacons in mapping mode
        self.beacons_pub = self.create_publisher(String, '~/mapped_beacons', 10) 

    def _odom_callback(self, msg):
        # Update internal odometry state (simplified)
        # In a real SLAM, this would drive the prediction step of an EKF
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.robot_trajectory.append((px, py))

    def _ranges_callback(self, msg):
        try:
            current_ranges = json.loads(msg.data)
            self.last_ranges = current_ranges
            
            if self.mode == 'LOCALIZATION':
                self._solve_trilateration(current_ranges)
            elif self.mode == 'MAPPING':
                self._update_mapping(current_ranges)

        except json.JSONDecodeError:
            pass

    def _solve_trilateration(self, ranges):
        # Collect active beacons that we know
        anchors = []
        distances = []
        
        for mac, dist in ranges.items():
            if mac in self.known_beacons:
                anchors.append(self.known_beacons[mac][:2]) # Use X,Y only for 2D solving
                distances.append(dist)
        
        if len(anchors) < 3:
            self.get_logger().debug("Not enough beacons for trilateration (need 3+)")
            return

        anchors = np.array(anchors)
        distances = np.array(distances)

        # Objective function for Least Squares: sum((x-ax)^2 + (y-ay)^2 - d^2)^2
        def residuals(x, anchors, distances):
            return np.linalg.norm(anchors - x, axis=1) - distances

        # Initial guess: previous pose or centroid of anchors
        x0 = self.robot_pose[:2]
        
        # Optimize
        result = least_squares(residuals, x0, args=(anchors, distances), loss='soft_l1')
        
        if result.success:
            self.robot_pose[:2] = result.x
            self._publish_pose(result.x, 0.0) # Z=0 for now
            self.get_logger().debug(f"Solved Pose: {result.x}")

    def _update_mapping(self, ranges):
        """
        Experimental: Naive mapping.
        If we see a beacon and we have moved enough since last update, try to triangulate it.
        This is a placeholder for full GraphSLAM logic.
        """
        # For now, just log that we see them. Real mapping needs EKF.
        for mac, dist in ranges.items():
            if mac not in self.mapped_beacons:
                # Initialize potentially
                pass
        
        # To truly map, we need to solve the inverse problem of trilateration over time.
        # We would accumulate constraints: (RobotPose_t, Beacon_i, Distance_ti)
        # And solve a large graph optimization.

    def _publish_pose(self, xy, z):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        
        msg.pose.pose.position.x = xy[0]
        msg.pose.pose.position.y = xy[1]
        msg.pose.pose.position.z = float(z)
        
        # Identity orientation (unknown)
        msg.pose.pose.orientation.w = 1.0
        
        # Covariance (Static for now, implies some uncertainty)
        msg.pose.covariance[0] = 0.5 # X variance
        msg.pose.covariance[7] = 0.5 # Y variance
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BLESLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
