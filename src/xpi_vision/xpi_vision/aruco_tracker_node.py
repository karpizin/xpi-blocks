#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_matrix

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        
        # Parameters
        self.declare_parameter('marker_size', 0.1) # Meters
        self.declare_parameter('dictionary_name', 'DICT_4X4_50')
        
        self.marker_size = self.get_parameter('marker_size').value
        
        # ArUco Setup
        dict_id = getattr(cv2.aruco, self.get_parameter('dictionary_name').value)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

        # Camera calibration (Defaults for RPi Camera v2 if no info received)
        self.mtx = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=float)
        self.dist = np.zeros((5,1))

        # Sub/Pub
        self.create_subscription(Image, '~/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '~/camera_info', self.info_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseArray, '~/poses', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 10)
        
        self.get_logger().info("ArUco Tracker Node Ready")

    def info_callback(self, msg):
        self.mtx = np.array(msg.k).reshape((3,3))
        self.dist = np.array(msg.d)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        
        # Detect
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            # Estimate Pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.mtx, self.dist)
            
            for i in range(len(ids)):
                # Draw on debug image
                cv2.drawFrameAxes(cv_img, self.mtx, self.dist, rvecs[i], tvecs[i], 0.05)
                
                # Convert to ROS Pose
                p = Pose()
                p.position.x = float(tvecs[i][0][0])
                p.position.y = float(tvecs[i][0][1])
                p.position.z = float(tvecs[i][0][2])
                
                # Rotation: rvec to quaternion
                rmat, _ = cv2.Rodrigues(rvecs[i])
                # Expand to 4x4 matrix for tf_transformations
                full_mat = np.eye(4)
                full_mat[:3, :3] = rmat
                q = quaternion_from_matrix(full_mat)
                p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                
                pose_array.poses.append(p)
            
            self.pose_pub.publish(pose_array)
            
            # Label markers
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)

        # Publish debug image
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
