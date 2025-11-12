#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16,Float64
import numpy as np
from bluerov2_util.PID import PID  # your reusable PID class
from rcl_interfaces.msg import SetParametersResult

class Bluerov2DepthControl(LifecycleNode):

    def __init__(self):
        super().__init__("depth_controller")
        self.get_logger().info("Initializing Bluerov2 Depth Control Node...")

        # Declare parameters (flattened dot notation for nested PID)
        self.declare_parameter('pid.kp', 0.0)
        self.declare_parameter('pid.ki', 0.0)
        self.declare_parameter('pid.kd', 0.0)
        self.declare_parameter('target_depth', 0.0)
        self.declare_parameter('flotability', 0.0)
        self.declare_parameter('input_topic', '/input')
        self.declare_parameter('setpoint_topic', '/setpoint')
        self.declare_parameter('output_topic', 'controller/pwm_depth')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('pid.i_lim',0.0)
        self.declare_parameter('enable', True)
        self.declare_parameter("hold",False)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Runtime attributes
        self._pid = None
        self.goal_depth = 0.0
        self._flotability = 0.0
        self._current_depth = 0.0
        self._input_sub = None
        self._output_pub = None

    # Lifecycle callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring Depth Controller...")

        # Load parameters
        kp = self.get_parameter('pid.kp').value
        ki = self.get_parameter('pid.ki').value
        kd = self.get_parameter('pid.kd').value
        i_lim = self.get_parameter('pid.i_lim').value
        self.goal_depth = self.get_parameter('target_depth').value
        self.flotability = self.get_parameter('flotability').value
        self.get_logger().info(f"flotability:{self.flotability}")
        input_topic = self.get_parameter('input_topic').value
        self.get_logger().info(f"input_topic:{input_topic}")
        output_topic = self.get_parameter('output_topic').value
        setpoint_topic = self.get_parameter('setpoint_topic').value
        self.rate = self.get_parameter('control_rate').value
        self.hold = self.get_parameter('hold').value
        self.get_logger().info(f"rate:{self.rate}")

        self._pid = PID(kp, ki, kd, i_lim)  # optional limits
        self.get_logger().info(f"output topic: {output_topic}")

        self._output_pub = self.create_publisher(UInt16, output_topic, 10)

        self._input_topic = input_topic
        self.setpoint_topic = setpoint_topic
        
        self.init_control = True
        self.target = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Depth Controller...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._input_sub = self.create_subscription(
            Float64,
            self._input_topic,
            self._on_depth_measurement,
            qos_profile=qos_profile
        )

        self.setpoint_sub = self.create_subscription(Float64,
                            self.setpoint_topic,
                            self.get_target_depth,
                            qos_profile=qos_profile)
        # Control timer
        self._pid.reset()
        self.init_control = True
        self._timer = self.create_timer(1.0 / self.rate, self._control_loop)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Depth Controller...")
        # Stop the control loop
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        # Optionally destroy subscriptions to avoid receiving messages
        if hasattr(self, "_input_sub") and self._input_sub is not None:
            self.destroy_subscription(self._input_sub)
            self._input_sub = None

        if hasattr(self, "setpoint_sub") and self.setpoint_sub is not None:
            self.destroy_subscription(self.setpoint_sub)
            self.setpoint_sub = None

        self.send_stop_pwm()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Depth Controller...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down Depth Controller...")
        return TransitionCallbackReturn.SUCCESS

    # Parameter update callback for runtime tuning
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'pid.kp':
                self._pid.kp = p.value
                self.get_logger().info(f"kp updated to: {self._pid.kp}")
            elif p.name == 'pid.ki':
                self._pid.ki = p.value
                self.get_logger().info(f"ki updated to: {self._pid.ki}")
            elif p.name == 'pid.kd':
                self._pid.kd = p.value
                self.get_logger().info(f"kd updated to: {self._pid.kd}")
            elif p.name == 'pid.i_lim':
                self.get_logger().info(f"integration limit updated to: {self._pid.i_lim}")
                self._pid.i_lim = p.value
            elif p.name == 'hold':
                self.hold = p.value
                self.get_logger().info(f"hold mode updated: {self.hold}")
        return SetParametersResult(successful=True)


    # Subscriber callback
    def get_target_depth(self,msg):
        self.goal_depth = msg.data
    def _on_depth_measurement(self, msg):
        self._current_depth = msg.data

    # Timer callback: computes PID output and publishes RC override
    def _control_loop(self):
        if self._pid is None:
            return
        if self.hold:
            if self.init_control:
                self.init_control = False
                self.target = self._current_depth
        else:
            self.target = self.goal_depth

        out,dic = self._pid.update(self.target, self._current_depth,feedforward=self.flotability/10) 
        force = -out
        self.get_logger().info(f"PID debug: {dic}")
        pwm = self._force_to_pwm(force*10 / 4)

        msg_pwm = UInt16()
        msg_pwm.data= int(pwm)  # heave
        self._output_pub.publish(msg_pwm)

    # Convert force to PWM
    def _force_to_pwm(self, force):
        if force > 0:
            pwm = 1536 + 9.7 * force
        else:
            pwm = 1464 + 12.781 * force
        return np.clip(pwm, 1300, 1700)

    def send_stop_pwm(self):
        pwm = 1500
        msg_pwm = UInt16()
        msg_pwm.data= int(pwm)
        self._output_pub.publish(msg_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = Bluerov2DepthControl()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
