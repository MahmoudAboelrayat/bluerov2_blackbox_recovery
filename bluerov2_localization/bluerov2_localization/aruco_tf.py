#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations 
import tf2_ros

class ArucoStaticTFs(Node):
    def __init__(self):
        super().__init__('aruco_static_tfs')

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        marker_poses = [
            (7.0, 4.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 0
            (1.0, 1.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 1
            (4.0, 7.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 2
            (7.0, 7.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 3
            (4.0, 4.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 4
            (7.0, 1.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 5
            (4.0, 1.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 6
            (1.0, 4.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 7
            (1.0, 7.5, 4.8, 0.7068252, 0.7073883, 0.0000327, 0.0000328),  # Aruco 8
        ]
        transforms = []

        # Define placeholders for 8 ArUco markers
        for i in range(0, 9):
            x, y, z, qx, qy, qz, qw = marker_poses[i]  # i ranges from 1 to 8
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'ned'       # ned frame
            t.child_frame_id = f'aruco_{i}' # marker frame

            t.transform.translation.x = x  # placeholder
            t.transform.translation.y = y  # placeholder
            t.transform.translation.z = z  # placeholder

            # If marker is aligned with world axes, use no rotation (0,0,0,1)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            transforms.append(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'       # world frame
        t.child_frame_id = f'ned' # ned frame

        t.transform.translation.x = 0.0  # placeholder
        t.transform.translation.y = 0.0  # placeholder
        t.transform.translation.z = 0.0  # placeholder

        # If marker is aligned with world axes, use no rotation (0,0,0,1)
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0
        transforms.append(t)
        # Send all static transforms at once
        
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms for 8 ArUco markers.')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoStaticTFs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
