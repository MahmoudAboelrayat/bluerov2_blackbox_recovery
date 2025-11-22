#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations

class ORBVO(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        # parameters
        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('calibration_file', 'bluerov2_localization/param/camera_calibration_11_11.npz')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        # Load camera calibration
        data = np.load(self.calib_file)
        self.K = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, self.camera_topic, self.vo_callback, 10)

        self.pub = self.create_publisher(PoseStamped, 'vo/pose', 10)

        self.orb = cv2.ORB_create(2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None

        # Initialize pose
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        cv2.namedWindow("ORB Feature Tracking", cv2.WINDOW_NORMAL)

    def vo_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # Undistort
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist_coeffs, (w, h), 1)
        frame = cv2.undistort(frame, self.K, self.dist_coeffs, None, new_K)

        # Extract features
        kp, des = self.orb.detectAndCompute(frame, None)

        if self.prev_frame is None:
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_des = des
            return

        # Match features
        matches = self.bf.match(self.prev_des, des)
        matches = sorted(matches, key=lambda m: m.distance)

        # Draw visualization
        match_img = cv2.drawMatches(
            self.prev_frame, self.prev_kp,
            frame, kp,
            matches[:200], None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        cv2.imshow("ORB Feature Tracking", match_img)
        cv2.waitKey(1)

        # Extract matched points
        pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

        if len(pts1) < 8:
            self.get_logger().warn("Not enough matches")
            return

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K, method=cv2.RANSAC, threshold=1.0)

        if E is None:
            return

        # Recover pose
        _, R_est, t_est, _ = cv2.recoverPose(E, pts1, pts2, self.K)

        # Integrate motion
        self.t += self.R @ t_est
        self.R = R_est @ self.R

        self.publish_pose(msg)

        # Update for next iteration
        self.prev_frame = frame
        self.prev_kp = kp
        self.prev_des = des

    def publish_pose(self, msg):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "camera"

        # Convert rotation matrix
        quat = self.rot_to_quat(self.R)

        pose.pose.position.x = float(self.t[0])
        pose.pose.position.y = float(self.t[1])
        pose.pose.position.z = float(self.t[2])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.pub.publish(pose)

    def rot_to_quat(self, R):
        T = np.eye(4)
        T[:3, :3] = R
        return tf_transformations.quaternion_from_matrix(T)

def main():
    rclpy.init()
    node = ORBVO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
