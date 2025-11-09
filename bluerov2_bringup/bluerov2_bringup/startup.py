import rclpy
from rclpy.node import Node
from time import sleep
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import MountControl
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.msg import Transition,State
from lifecycle_msgs.srv import ChangeState, GetState
class bluerov2_bringup(Node):
    def __init__(self):
        super().__init__("bluerov2_bringup")

        self.pub_camera_anlge = self.create_publisher(MountControl, 'mount_control/command', 10)
        self.declare_parameter("initialize", False)
        self.declare_parameter("lights_available", True)
        self.declare_parameter("gripper_available", True)

        self.declare_parameter("gripper_pin", 11.0)
        self.declare_parameter("light_pin", 13.0)
        self.declare_parameter("camera_pen", 16.0)

        self.declare_parameter("light_min", 1100.0)
        self.declare_parameter("light_max", 1900.0)
        self.declare_parameter("gripper_open", 1600.0)
        self.declare_parameter("gripper_close", 1300.0)
        self.declare_parameter("tilt_int", 0.0)
        self.declare_parameter("tilt_max", 45.0)
        self.declare_parameter("tilt_min", -45.0)
        self.declare_parameter("mode", "manual")

        self.light_pin = self.get_parameter("light_pin").value
        self.camera_servo_pin = self.get_parameter("camera_pen").value
        self.gripper_pin = self.get_parameter("gripper_pin").value
        self.light_min = self.get_parameter("light_min").value
        self.light_max = self.get_parameter("light_max").value
        self.gripper_open = self.get_parameter("gripper_open").value
        self.gripper_close = self.get_parameter("gripper_close").value
        self.tilt_int = self.get_parameter("tilt_int").value
        self.camera_tilt_max = self.get_parameter("tilt_max").value
        self.camera_tilt_min = self.get_parameter("tilt_min").value
        self.lights_available = self.get_parameter("lights_available").value
        self.gripper_available = self.get_parameter("gripper_available").value
        self.mode = self.get_parameter("mode").value
        self.initialization_test_flag = self.get_parameter("initialize").value

        self.mode_change = False
        if self.initialization_test_flag:
            self.initialization_test()

        self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        self.get_mode("mode")

        if self.mode_change:
            self.get_logger().info(f"Mode changed to {self.mode}")
            if self.mode == "correction":
                self.activate_lifecycle('bluerov2_gimbal')
            elif self.mode == "manual":
                self.deactivate_gimbal('bluerov2_gimbal')
               
        

    def initialization_test(self):
        """Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem."""
        self.get_logger().info("Starting initialization test for lights, camera servo and gripper.")
        # Flash the light
        if self.lights_available:
            self.light = self.light_min
            self.get_logger().info(f"ligth pin:{self.light_pin}, light value:{self.light}")
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)
            self.light = self.light_max
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)
            self.light = self.light_min
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)

        # Move the camera servo to max and min
        if self.gripper_available:
            self.gripper = self.gripper_close
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)
            self.gripper = self.gripper_open
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)
            self.gripper = self.gripper_close  
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)

         # Move the camera servo to max and min
        self.tilt = self.camera_tilt_max
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        self.tilt = self.camera_tilt_min
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        self.tilt = self.tilt_int
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        
        
        self.get_logger().info("Light and camera servo test completed.")  
        
    def send_servo_comand(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board (13 for lights and 15 for camera servo)
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
                result = client.wait_for_service(timeout_sec=4.0)
        # Create a request object for CommandLong service
        request = CommandLong.Request()


        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0             


        # Send the service request and wait for the response
        future = client.call_async(request)

    def set_camera_tilt(self, tilt_angle):
        ''' Set the camera tilt angle in degrees'''
        msg = MountControl()
        msg.pitch = tilt_angle# tilt down
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.mode = 2        # MAVLINK_MOUNT_MODE_MAVLINK_TARGETING
        self.pub_camera_anlge.publish(msg)


    def get_mode(self, name):
        target_node = 'bluerov2_teleop'
        self.client = self.create_client(GetParameters, f'{target_node}/get_parameters')

        if not self.client.service_is_ready():
            # self.get_logger().warn(f'Parameter service not ready on {target_node}, skipping request.')
            return

        request = GetParameters.Request()
        request.names = [name]  # List of parameter names

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_callback)

    def parameter_callback(self, future):
        try:
            response = future.result()
            value = response.values[0].string_value
            if getattr(self, "mode", None) != value:
                self.mode_change = True
            else:
                self.mode_change = False
            self.mode = value
        except Exception as e:
            self.get_logger().error(f'Failed to get parameter: {e}')

    def activate_lifecycle(self, node_name):
        ns = self.get_namespace().strip('/')
        self.get_logger().info(f'Activating lifecycle node: {node_name} in namespace: {ns}')
        full_name = f'/{ns}/{node_name}/change_state' if ns else f'/{node_name}/change_state'
        client = self.create_client(ChangeState, full_name)
        if not client.service_is_ready():
            self.get_logger().warn(f'Lifecycle service not ready for {node_name}')
            return

        # Step 1: Configure the node
        configure_req = ChangeState.Request()
        configure_req.transition.id = Transition.TRANSITION_CONFIGURE

        future_config = client.call_async(configure_req)

        # Callback after configure attempt
        def after_config(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.get_logger().info(f'{node_name} configured successfully.')

                    # Step 2: Activate after successful configure
                    activate_req = ChangeState.Request()
                    activate_req.transition.id = Transition.TRANSITION_ACTIVATE
                    future_activate = client.call_async(activate_req)

                    def after_activate(fut_act):
                        try:
                            resp_act = fut_act.result()
                            if resp_act.success:
                                self.get_logger().info(f'{node_name} activated successfully.')
                            else:
                                self.get_logger().warn(f'Failed to activate {node_name}')
                        except Exception as e:
                            self.get_logger().error(f'Error activating {node_name}: {e}')

                    future_activate.add_done_callback(after_activate)

                else:
                    self.get_logger().warn(f'Failed to configure {node_name}')
            except Exception as e:
                self.get_logger().error(f'Error configuring {node_name}: {e}')

        future_config.add_done_callback(after_config)


    def deactivate_gimbal(self, node_name):
        """
        Attempts to deactivate and then cleanup a ROS 2 lifecycle node.
        """
        # 🌟 Setup Client Names
        ns = self.get_namespace().strip('/')
        client_name = f'/{ns}/{node_name}/change_state' if ns else f'/{node_name}/change_state'
        state_name = f'/{ns}/{node_name}/get_state' if ns else f'/{node_name}/get_state'
        self.get_logger().info(f"client {client_name}")
        # 🌟 Create Clients
        change_client = self.create_client(ChangeState, client_name)
        state_client = self.create_client(GetState, state_name)

        # 🌟 Wait for Services
        if not change_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"ChangeState service not available for **{node_name}**")
            return
        if not state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"GetState service not available for **{node_name}**")
            return

        # 🌟 Get Current State
        current_state = self._get_node_state(node_name, state_client)
        if current_state is None:
            return

        # 🌟 Deactivate if Active
        if current_state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f"**Deactivating** {node_name}...")
            if not self._send_transition(node_name, change_client, Transition.TRANSITION_DEACTIVATE):
                return # Exit on failed deactivation
        else:
            self.get_logger().info(f"{node_name} is not active (state id={current_state}), skipping deactivate.")
        
        # ---
        
        # 🌟 Check State and Cleanup if Inactive
        # Refresh state after attempted deactivation
        current_state = self._get_node_state(node_name, state_client)
        if current_state is None:
            return

        if current_state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f"**Cleaning up** {node_name}...")
            if not self._send_transition(node_name, change_client, Transition.TRANSITION_CLEANUP):
                return # Exit on failed cleanup
        elif current_state == State.PRIMARY_STATE_FINALIZED:
            self.get_logger().info(f"{node_name} is already **Finalized**, no further action.")
        else:
            self.get_logger().warn(f"Cannot cleanup {node_name}. Current state is **{current_state}**.")


    # Helper functions added for cleaner, reusable logic and error checking

    def _get_node_state(self, node_name, state_client):
        """Internal helper to get the current state of the node."""
        state_req = GetState.Request()
        future_state = state_client.call_async(state_req)
        rclpy.spin_until_future_complete(self, future_state, timeout_sec=5.0)
        
        if future_state.result() is None or not future_state.done():
            self.get_logger().warn(f"Failed to get state for **{node_name}** or call timed out.")
            return None
        
        return future_state.result().current_state.id

    def _send_transition(self, node_name, change_client, transition_id):
        """Internal helper to send a ChangeState transition and check the result."""
        transition_name = Transition.TRANSITION_DEACTIVATE if transition_id == Transition.TRANSITION_DEACTIVATE else Transition.TRANSITION_CLEANUP
        
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = change_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) # Increased timeout
        
        result = future.result()
        if result is None or not future.done():
            self.get_logger().error(f"Transition **{transition_name}** for {node_name} failed (call timed out or no response).")
            return False
        
        if not result.success:
            self.get_logger().error(f"Transition **{transition_name}** for {node_name} failed (service returned failure).")
            return False
            
        self.get_logger().info(f"Transition **{transition_name}** successfully requested for {node_name}.")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = bluerov2_bringup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
