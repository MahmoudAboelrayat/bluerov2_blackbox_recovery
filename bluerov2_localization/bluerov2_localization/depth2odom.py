#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

class DepthToOdom(Node):
    def __init__(self):
        super().__init__("depth_to_odom")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter("rel_alt_topic", "global_position/rel_alt")
        self.declare_parameter("output_topic", "depth_odom")

        rel_alt_topic = self.get_parameter("rel_alt_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.sub = self.create_subscription(
            Float64,
            rel_alt_topic,
            self.rel_alt_callback,
            qos_profile=qos_profile
        )

        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        self.get_logger().info(
            f"Depth-to-Odometry converter started.\n"
            f"Listening to: {rel_alt_topic}\n"
            f"Publishing:   {output_topic}"
        )

    def rel_alt_callback(self, msg: Float64):
        rel_alt = msg.data  # in meters

        depth = -rel_alt   

        odom = PoseWithCovarianceStamped()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "ned"

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = depth

        odom.pose.covariance = [
            9999.9, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 9999.9, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.9, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.9, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 9999.9
        ]

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = DepthToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
