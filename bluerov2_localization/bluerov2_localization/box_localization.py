#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class BoxArucoNode(Node):
    def __init__(self):
        super().__init__('box_localization_node')

        # Parameters
        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('marker_length', 0.075)  # meters
        self.declare_parameter('calibration_file', 'bluerov2_localization/param/camera_calibration_11_11.npz')
        self.declare_parameter('marker_id', 42)
        self.declare_parameter('parent_frame', 'camera')
        self.declare_parameter('child_frame', 'blackbox_center')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # Load camera calibration
        data = np.load(self.calib_file)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

        # Publisher for box pose
        self.pose_pub = self.create_publisher(PoseStamped, 'box_pose', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Box ArUco detector node started, listening to {self.camera_topic}")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.marker_id in ids:
            idx = np.where(ids.flatten() == self.marker_id)[0][0]
            marker_corners = [corners[idx]]

            # Estimate pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # Draw axes for visualization
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec.reshape(3,1), tvec.reshape(3,1), self.marker_length/2)

            # Convert to quaternion
            rot_matrix, _ = cv2.Rodrigues(rvec)
            quat = self.rotation_matrix_to_quaternion(rot_matrix)

            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(pose_msg)

            # Publish TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = float(tvec[0])
            t.transform.translation.y = float(tvec[1])
            t.transform.translation.z = float(tvec[2])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(t)

        # Optional visualization
        cv2.aruco.drawDetectedMarkers(cv_image, corners)
        cv2.imshow("Box ArUco Detection", cv_image)
        cv2.waitKey(1)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert rotation matrix to quaternion (x, y, z, w)"""
        q = np.empty((4,))
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * t
            q[0] = (R[2, 1] - R[1, 2]) / t
            q[1] = (R[0, 2] - R[2, 0]) / t
            q[2] = (R[1, 0] - R[0, 1]) / t
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                t = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[3] = (R[2, 1] - R[1, 2]) / t
                q[0] = 0.25 * t
                q[1] = (R[0, 1] + R[1, 0]) / t
                q[2] = (R[0, 2] + R[2, 0]) / t
            elif R[1, 1] > R[2, 2]:
                t = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[3] = (R[0, 2] - R[2, 0]) / t
                q[0] = (R[0, 1] + R[1, 0]) / t
                q[1] = 0.25 * t
                q[2] = (R[1, 2] + R[2, 1]) / t
            else:
                t = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[3] = (R[1, 0] - R[0, 1]) / t
                q[0] = (R[0, 2] + R[2, 0]) / t
                q[1] = (R[1, 2] + R[2, 1]) / t
                q[2] = 0.25 * t
        return q.tolist()


def main(args=None):
    rclpy.init(args=args)
    node = BoxArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
