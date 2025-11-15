#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np

class CameraPoseFromMarkers(Node):
    def __init__(self):
        super().__init__('camera_pose_from_markers')
        self.subscription = self.create_subscription(
            PoseArray,
            'aruco_poses',  # camera -> marker poses
            self.pose_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cam_pose_pub = self.create_publisher(PoseStamped, 'aruco_odom', 10)
        self.get_logger().info('Camera Pose from Markers Node started.')

    def pose_callback(self, msg: PoseArray):
        cam_positions = []
        cam_rotations = []

        for i, pose in enumerate(msg.poses):
            if pose.position.x == -999.99:
                continue  # marker not found

            # Camera -> Marker (from PoseArray)
            T_cam_marker = quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                             pose.orientation.z, pose.orientation.w])
            T_cam_marker[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

            # Invert to get Marker -> Camera
            T_marker_cam = np.linalg.inv(T_cam_marker)

            try:
                # Lookup NED -> Marker_i
                marker_frame = f'aruco_{i}'  # adjust naming to match your TF tree
                t = self.tf_buffer.lookup_transform('ned', marker_frame, rclpy.time.Time())
                T_ned_marker = quaternion_matrix([t.transform.rotation.x,
                                                 t.transform.rotation.y,
                                                 t.transform.rotation.z,
                                                 t.transform.rotation.w])
                T_ned_marker[:3, 3] = [t.transform.translation.x,
                                        t.transform.translation.y,
                                        t.transform.translation.z]

                # Chain: NED -> Marker -> Camera
                T_ned_cam = np.dot(T_ned_marker, T_marker_cam)
                cam_positions.append(T_ned_cam[:3, 3])
                cam_rotations.append(quaternion_from_matrix(T_ned_cam))

            except Exception as e:
                self.get_logger().warn(f"Could not lookup TF for {marker_frame}: {e}")
                continue

        if len(cam_positions) == 0:
            return  # no markers visible

        # Average positions and rotations
        cam_pos_avg = np.mean(cam_positions, axis=0)
        q_avg = np.mean(cam_rotations, axis=0)
        q_avg /= np.linalg.norm(q_avg)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'ned'
        pose_msg.pose.position.x = cam_pos_avg[0]
        pose_msg.pose.position.y = cam_pos_avg[1]
        pose_msg.pose.position.z = cam_pos_avg[2]
        pose_msg.pose.orientation.x = q_avg[0]
        pose_msg.pose.orientation.y = q_avg[1]
        pose_msg.pose.orientation.z = q_avg[2]
        pose_msg.pose.orientation.w = q_avg[3]
        self.cam_pose_pub.publish(pose_msg)

        # Publish TF from NED -> camera
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'ned'
        tf_msg.child_frame_id = 'camera'
        tf_msg.transform.translation.x = cam_pos_avg[0]
        tf_msg.transform.translation.y = cam_pos_avg[1]
        tf_msg.transform.translation.z = cam_pos_avg[2]
        tf_msg.transform.rotation.x = q_avg[0]
        tf_msg.transform.rotation.y = q_avg[1]
        tf_msg.transform.rotation.z = q_avg[2]
        tf_msg.transform.rotation.w = q_avg[3]
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = CameraPoseFromMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
