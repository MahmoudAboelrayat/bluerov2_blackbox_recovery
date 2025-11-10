#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor,MultiThreadedExecutor
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from mavros_msgs.msg import MountControl
from std_msgs.msg import Float64

class Bluerov2Gimbal(Node):

    def __init__(self):
        super().__init__("bluerov2_gimbal")
        self.get_logger().info("Initializing Bluerov2 Gimbal Node...")
        self.sub_tilt_angle = None
        self.sub_imu = None
        self.pub_camera_angle = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() called.")
        self.tilt_init = None
        self.angle = 0.0
       
        self.pub_camera_angle = self.create_publisher(MountControl, 'mount_control/command',10)
        self.pub_current_angle = self.create_publisher(Float64,"camera/angle",10)
       
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Camera Gimbal activate.")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_tilt_angle = self.create_subscription(
            Float64, 
            'camera/angle',
            self.get_camera_angle,
            qos_profile= qos_profile
        )
        self.sub_imu = self.create_subscription(Imu, "imu/data", self.get_angle, qos_profile=qos_profile)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Camera Gimbal deactivate.")
        if hasattr(self, "sub_tilt_angle") and self.sub_tilt_angle is not None:
            self.destroy_subscription(self._sub_tilt_angle)
            self.sub_tilt_angle = None

        if hasattr(self, "sub_imu") and self.sub_imu is not None:
            self.destroy_subscription(self.sub_imu)
            self.sub_imu = None

        self.stop_msg()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() called.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() called.")
        return TransitionCallbackReturn.SUCCESS
    
    def get_angle(self, data):
        
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        sinp = 2.0 * (w * y - z * x)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        pitch = np.arcsin(sinp)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if self.tilt_init is not None:
            tilt_angle =  pitch - self.tilt_init 
            self.get_logger().info(f"tilt init: {np.degrees(tilt_angle)}, pitch: {np.degrees(pitch)}")
            msg = MountControl()
            msg.pitch = np.degrees(tilt_angle)
            msg.roll = 0.0
            msg.yaw = 0.0
            msg.mode = 2        # MAVLINK_MOUNT_MODE_MAVLINK_TARGETING
            self.pub_camera_angle.publish(msg)

            angle_msg = Float64()
            angle_msg.data = tilt_angle
            self.pub_current_angle(angle_msg)
            
            # self.get_logger().info(f"tilt angle: {np.degrees(tilt_angle)}")
        # self.get_logger().info(f"roll: {np.degrees(roll)}, pitch: {np.degrees(pitch)}, yaw: {np.degrees(yaw)}")
    def get_camera_angle(self, data):
        if self.tilt_init is None:
            self.tilt_init = data


    def stop_msg(self):
        tilt_angle =  self.tilt_init 
        msg = MountControl()
        msg.pitch = np.degrees(tilt_angle)
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.mode = 2        # MAVLINK_MOUNT_MODE_MAVLINK_TARGETING
        self.pub_camera_angle.publish(msg)

        angle_msg = Float64()
        angle_msg.data = tilt_angle
        self.pub_current_angle(angle_msg)
def main(args=None):
    rclpy.init(args=args)

    bluerov2_gimbal = Bluerov2Gimbal()
    executor = MultiThreadedExecutor()
    executor.add_node(bluerov2_gimbal)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    bluerov2_gimbal.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
