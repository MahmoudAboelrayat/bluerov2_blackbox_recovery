#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from mavros_msgs.msg import OverrideRCIn, Mavlink
from bluerov2_interface.msg import Detection
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


# camera parameters
u0 = 480
v0 = 270
lx = 455
ly = 455
kud = 0.00683 
kdu = -0.01424     
    
# convert a pixel coordinate to meters given linear calibration parameters
def convert2meter(pt, u0, v0, lx, ly):
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a pixel coordinate to meters using default calibration parameters
def convertOnePoint2meter(pt):
    global u0, v0, lx, ly
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a list of pixels coordinates to meters using default calibration parameters
def convertListPoint2meter(points):
    global u0, v0, lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n, 2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt, u0, v0, lx, ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter



class Controller(Node):
    def __init__(self):
        super().__init__('Vision_Controller')

        # Initialize with None to be set from image dimensions
        self.desired_point = None
        self.image_center_set = False

        # ============== DECLARE DYNAMIC PARAMETERS ==============
        
        # Desired point parameters (will be auto-set from image center on first frame)
        self.declare_parameter('desired_point_x', -1.0,
            ParameterDescriptor(description='Desired X pixel coordinate (-1 = auto image center)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_y', -1.0,
            ParameterDescriptor(description='Desired Y pixel coordinate (-1 = auto image center)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_z', 0.5,
            ParameterDescriptor(description='Desired distance (depth) in meters',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Control gains for [surge, sway, heave, yaw]
        self.declare_parameter('gain_surge', 0.6,
            ParameterDescriptor(description='Visual servoing gain for surge (forward/back)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_sway', 0.5,
            ParameterDescriptor(description='Visual servoing gain for sway (left/right)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_heave', 0.5,
            ParameterDescriptor(description='Visual servoing gain for heave (up/down)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_yaw', 0.5,
            ParameterDescriptor(description='Visual servoing gain for yaw (rotation)',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Velocity limits
        self.declare_parameter('v_linear_max', 0.3,
            ParameterDescriptor(description='Maximum linear velocity (m/s)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('v_angular_max', 0.5,
            ParameterDescriptor(description='Maximum angular velocity (rad/s)',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Floatability compensation
        self.declare_parameter('floatability', -0.18,
            ParameterDescriptor(description='Heave compensation for buoyancy/floatability',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Sign inversion flags for each DOF
        self.declare_parameter('invert_surge', False,
            ParameterDescriptor(description='Invert surge control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_sway', False,
            ParameterDescriptor(description='Invert sway control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_heave', True,
            ParameterDescriptor(description='Invert heave control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_yaw', True,
            ParameterDescriptor(description='Invert yaw control direction',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Control enable/disable
        self.declare_parameter('enable_visual_servoing', False,
            ParameterDescriptor(description='Enable/disable visual servoing control',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Track handle center instead of box center
        self.declare_parameter('track_handle', False,
            ParameterDescriptor(description='Track handle instead of box center',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self._on_set_parameters)
        
        # Initialize parameters
        self.update_parameters()

        self.distance = 1.0
        self.blackbox_xmin = 0
        self.blackbox_xmax = 0
        self.blackbox_ymin = 0
        self.blackbox_ymax = 0
        self.handle_xmin = 0

        # Visual servo parameters
        self.Camera_cooriction_surge = 1500
        self.Camera_cooriction_sway = 1500
        self.Camera_cooriction_heave = 1500
        self.Camera_cooriction_yaw = 1500
        self.Camera_cooriction_roll = 1500
        self.Camera_cooriction_pitch = 1500

        # Initialize error tracking
        self.s_error = np.zeros(2)
        self.z_error = 0.0

        # ---------------- SUBSCRIBERS ----------------

        #sim topic:   /bluerov/camera/image_color
        # real robot topic: camera/image
        self.subscription = self.create_subscription(
            Image,
            '/bluerov/camera/image_color',
            self.image_callback,
            10
        )
        # OpenCV Bridge
        self.bridge = CvBridge()

        self.boxDetections = self.create_subscription(
            Detection,
            'detections',
            self.color_video_tracking_callback,
            10
        )
        self.boxDetections 

        # ---------------- PUBLISHER ----------------
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)

        timer_period = 0.05  # 50 msec - 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Color Tracker Controller Active with Dynamic Parameters")
        self.log_current_parameters()

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'desired_point_x':
                self.desired_point[0] = p.value
            elif p.name == 'desired_point_y':
                self.desired_point[1] = p.value
            elif p.name == 'desired_point_z':
                self.desired_point[2] = p.value
            elif p.name == 'gain_surge':
                self.gains[0] = p.value
            elif p.name == 'gain_sway':
                self.gains[1] = p.value
            elif p.name == 'gain_heave':
                self.gains[2] = p.value
            elif p.name == 'gain_yaw':
                self.gains[3] = p.value
            elif p.name == 'v_linear_max':
                self.v_linear_max = p.value
            elif p.name == 'v_angular_max':
                self.v_angular_max = p.value
            elif p.name == 'floatability':
                self.floatability = p.value
            elif p.name == 'invert_surge':
                self.invert_surge = p.value
            elif p.name == 'invert_sway':
                self.invert_sway = p.value
            elif p.name == 'invert_heave':
                self.invert_heave = p.value
            elif p.name == 'invert_yaw':
                self.invert_yaw = p.value
            elif p.name == 'enable_visual_servoing':
                self.enable_visual_servoing = p.value
                self.get_logger().info(f"Visual servoing {'ENABLED' if p.value else 'DISABLED'}")
            elif p.name == 'track_handle':
                self.track_handle = p.value
                self.get_logger().info(f"Target switched to {'HANDLE' if p.value else 'BOX CENTER'}")
        return SetParametersResult(successful=True)

    def update_parameters(self):
        """Update internal variables from ROS2 parameters"""
        # Get desired point
        desired_x = self.get_parameter('desired_point_x').value
        desired_y = self.get_parameter('desired_point_y').value
        desired_z = self.get_parameter('desired_point_z').value
        
        # Initialize desired_point array if needed
        if self.desired_point is None:
            self.desired_point = np.array([desired_x, desired_y, desired_z])
        else:
            # Only update from parameters if they're not -1 (auto mode)
            if desired_x >= 0:
                self.desired_point[0] = desired_x
            if desired_y >= 0:
                self.desired_point[1] = desired_y
            self.desired_point[2] = desired_z
        
        # Get control gains
        self.gains = np.array([
            self.get_parameter('gain_surge').value,
            self.get_parameter('gain_sway').value,
            self.get_parameter('gain_heave').value,
            self.get_parameter('gain_yaw').value
        ])
        
        # Get velocity limits
        self.v_linear_max = self.get_parameter('v_linear_max').value
        self.v_angular_max = self.get_parameter('v_angular_max').value
        
        # Get floatability compensation
        self.floatability = self.get_parameter('floatability').value
        
        # Get inversion flags
        self.invert_surge = self.get_parameter('invert_surge').value
        self.invert_sway = self.get_parameter('invert_sway').value
        self.invert_heave = self.get_parameter('invert_heave').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        
        # Get control enable flag
        self.enable_visual_servoing = self.get_parameter('enable_visual_servoing').value

        # Get target selection
        self.track_handle = self.get_parameter('track_handle').value

    def log_current_parameters(self):
        """Log all current parameter values"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Current Parameters:")
        self.get_logger().info(f"  Desired Point: [{self.desired_point[0]:.1f}, {self.desired_point[1]:.1f}, {self.desired_point[2]:.2f}]")
        self.get_logger().info(f"  Control Gains: [{self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[2]:.2f}, {self.gains[3]:.2f}]")
        self.get_logger().info(f"  Linear Max Vel: {self.v_linear_max:.2f} m/s")
        self.get_logger().info(f"  Angular Max Vel: {self.v_angular_max:.2f} rad/s")
        self.get_logger().info(f"  Floatability: {self.floatability:.2f}")
        self.get_logger().info(f"  Inversions: Surge={self.invert_surge}, Sway={self.invert_sway}, "
                             f"Heave={self.invert_heave}, Yaw={self.invert_yaw}")
        self.get_logger().info(f"  Visual Servoing: {'ENABLED' if self.enable_visual_servoing else 'DISABLED'}")
        try:
            use_sim = self.get_parameter('use_sim_time').value
            self.get_logger().info(f"  Use Sim Time: {use_sim}")
        except:
            pass
        self.get_logger().info(f"  Track Handle: {self.track_handle}")
        self.get_logger().info("=" * 60)

    def timer_callback(self):
        '''
        Time step at a fixed rate (1 / timer_period = 20 Hz) to execute control logic.
        '''
        self.setOverrideRCIN(self.Camera_cooriction_pitch, self.Camera_cooriction_roll, 
                            self.Camera_cooriction_heave, self.Camera_cooriction_yaw, 
                            self.Camera_cooriction_surge, self.Camera_cooriction_sway)

    def image_callback(self, msg):
        try:
            self.image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')    
            image_height, image_width, image_channels = self.image_np.shape
            
            # Set desired point to image center on first frame if not manually set
            if not self.image_center_set:
                param_x = self.get_parameter('desired_point_x').value
                param_y = self.get_parameter('desired_point_y').value
                
                # If parameters are still -1 (auto mode), set to image center
                if param_x < 0 or param_y < 0:
                    self.desired_point = np.array([
                        image_width // 2,
                        image_height // 2,
                        self.get_parameter('desired_point_z').value
                    ])
                    self.get_logger().info(
                        f"Auto-set desired point to image center: "
                        f"[{self.desired_point[0]:.1f}, {self.desired_point[1]:.1f}]"
                    )
                self.image_center_set = True
            
            # Extract width and height of the blackbox from the detected bounding box
            (width_px, height_px) = (self.blackbox_xmax - self.blackbox_xmin, 
                                    self.blackbox_ymax - self.blackbox_ymin)

            # Determine which dimension is the width ( choose max or min?)
            blackbox_width_px = max(width_px, height_px)
            
            if blackbox_width_px > 0:
                known_width_m = 0.3
                focal_length = 455
                self.distance = (known_width_m * focal_length) / blackbox_width_px
            else:
                # No valid detection - stop and rotate to search (only if servoing is enabled)
                if self.enable_visual_servoing:
                    self.Camera_cooriction_surge = self.mapValueScalSat(0.0)
                    self.Camera_cooriction_sway = self.mapValueScalSat(0.0)
                    self.Camera_cooriction_heave = self.mapValueScalSat(-0.25)
                    
                    search_rotation_velocity = 0.1  
                    self.Camera_cooriction_yaw = self.mapValueScalSat(-search_rotation_velocity)
                    
                    self.get_logger().warn("No detection - searching (rotating)...", 
                                        throttle_duration_sec=1.0)
                else:
                    # Visual servoing disabled - maintain neutral position
                    self.Camera_cooriction_surge = 1500
                    self.Camera_cooriction_sway = 1500
                    self.Camera_cooriction_heave = 1500
                    self.Camera_cooriction_yaw = 1500

            self.draw_visualization()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def mapValueScalSat(self, value):
        '''Map the value of the joystick analog from -1 to 1 to a pwm value from 1100 to 1900'''
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)
    
    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, 
                        channel_forward, channel_lateral):
        '''Override RC channels for motor control'''
        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)
        msg_override.channels[1] = np.uint(channel_roll)
        msg_override.channels[2] = np.uint(channel_throttle)
        msg_override.channels[3] = np.uint(channel_yaw)
        msg_override.channels[4] = np.uint(channel_forward)
        msg_override.channels[5] = np.uint(channel_lateral)
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500

        self.pub_msg_override.publish(msg_override)

    def color_video_tracking_callback(self, msg):
        """
        Visual servoing callback for BlueROV control using surge, sway, heave, and yaw.
        """
        # If visual servoing is disabled, set neutral commands and return
        if not self.enable_visual_servoing:
            self.Camera_cooriction_surge = 1500
            self.Camera_cooriction_sway = 1500
            self.Camera_cooriction_heave = 1500
            self.Camera_cooriction_yaw = 1500
            return
        
        # Extract detection data
        self.blackbox_xmin = msg.blackbox_xmin
        self.blackbox_xmax = msg.blackbox_xmax
        self.blackbox_xcenter = msg.blackbox_xcenter
        self.blackbox_ymin = msg.blackbox_ymin
        self.blackbox_ymax = msg.blackbox_ymax
        self.blackbox_ycenter = msg.blackbox_ycenter
        
        self.handle_xmin = msg.handle_xmin
        self.handle_xmax = msg.handle_xmax
        self.handle_xcenter = msg.handle_xcenter
        self.handle_ymin = msg.handle_ymin
        self.handle_ymax = msg.handle_ymax
        self.handle_ycenter = msg.handle_ycenter
        
        # Depth estimation
        Z = max(self.distance, 0.1)
        
        # Convert to normalized coordinates
        #x, y = convertOnePoint2meter([self.blackbox_xcenter, self.blackbox_ycenter])
        
        # Select target point based on tracking mode
        if self.track_handle and self.handle_xcenter > 0 and self.handle_xmax > 0:
            # Track handle if enabled and handle is detected
            x, y = convertOnePoint2meter([self.handle_xcenter, self.handle_ycenter])
        else:
            # Track box center (default or fallback)
            x, y = convertOnePoint2meter([self.blackbox_xcenter, self.blackbox_ycenter])
                
        xd, yd = convertOnePoint2meter(self.desired_point[:2])
        zd = self.desired_point[2]
        
        # Feature error
        self.s_error = np.array([x - xd, y - yd])
        self.z_error = Z - zd
        
        # Interaction matrix
        fx = 1.0
        fy = 1.0
        
        L_full = np.array([
            [-fx/Z, 0, fx*x/Z, fx*x*y, -fx*(1+x**2), fx*y],
            [0, -fy/Z, fy*y/Z, fy*(1+y**2), -fy*x*y, -fy*x]
        ])
        
        L_reduced = L_full[:, [0, 1, 2, 5]]
        L_depth = np.array([[0, 0, -1, 0]])
        L = np.vstack([L_reduced, L_depth])
        
        error_combined = np.array([self.s_error[0], self.s_error[1], self.z_error])
        
        # Compute velocity using DYNAMIC GAINS
        L_inv = np.linalg.pinv(L)
        v_cam_4d = -self.gains * (L_inv @ error_combined)
        
        # Transform to ROV frame
        H = self.camera_to_rov_transform()
        v_rov_4d = self.transform_velocity_4dof(v_cam_4d, H)
        
        # Apply DYNAMIC VELOCITY LIMITS
        v_rov_4d[0:3] = np.clip(v_rov_4d[0:3], -self.v_linear_max, self.v_linear_max)
        v_rov_4d[3] = np.clip(v_rov_4d[3], -self.v_angular_max, self.v_angular_max)
        
        # Apply inversion flags
        surge_sign = -1.0 if self.invert_surge else 1.0
        sway_sign = -1.0 if self.invert_sway else 1.0
        heave_sign = -1.0 if self.invert_heave else 1.0
        yaw_sign = -1.0 if self.invert_yaw else 1.0
        
        # Publish
        msg_twist = Twist()
        msg_twist.linear.x = float(surge_sign * v_rov_4d[0])
        msg_twist.linear.y = float(sway_sign * v_rov_4d[1])
        msg_twist.linear.z = float(heave_sign * v_rov_4d[2])
        msg_twist.angular.x = 0.0
        msg_twist.angular.y = 0.0
        msg_twist.angular.z = float(yaw_sign * v_rov_4d[3])
        
        # Store for RC override with inversion and floatability
        self.Camera_cooriction_surge = self.mapValueScalSat(surge_sign * v_rov_4d[0])
        self.Camera_cooriction_sway = self.mapValueScalSat(sway_sign * v_rov_4d[1])
        self.Camera_cooriction_heave = self.mapValueScalSat(heave_sign * v_rov_4d[2] + self.floatability)
        self.Camera_cooriction_yaw = self.mapValueScalSat(yaw_sign * v_rov_4d[3])

    def transform_velocity_4dof(self, v_cam, H):
        """Transform 4DOF velocity from camera frame to ROV body frame"""
        R = H[0:3, 0:3]
        v_linear_cam = v_cam[0:3]
        v_linear_rov = R @ v_linear_cam
        yaw_rov = v_cam[3]
        return np.array([v_linear_rov[0], v_linear_rov[1], v_linear_rov[2], yaw_rov])

    def camera_to_rov_transform(self):
        """Returns transformation matrix from camera frame to ROV body frame"""
        R = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        t = np.array([0.2, 0.0, 0.0])
        H = np.eye(4)
        H[0:3, 0:3] = R
        H[0:3, 3] = t
        return H
    
    def overlay_points(self, image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
        cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
        position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

    def draw_visualization(self):
        """Draw detection visualization with bounding boxes and tracking points"""
        try:
            if not hasattr(self, 'image_np') or self.image_np is None:
                return
            
            desired_x = int(self.desired_point[0])
            desired_y = int(self.desired_point[1])
            
            self.overlay_points(self.image_np, [desired_x, desired_y], 255, 0, 0, 
                               'desired point', scale=0.7, offsetx=10, offsety=-10)
            
            if self.blackbox_xmin > 0 and self.blackbox_xmax > 0:
                cv2.rectangle(self.image_np,
                            (int(self.blackbox_xmin), int(self.blackbox_ymin)),
                            (int(self.blackbox_xmax), int(self.blackbox_ymax)),
                            (0, 255, 255), 2)
                
                detected_x = int(self.blackbox_xcenter)
                detected_y = int(self.blackbox_ycenter)
                
                self.overlay_points(self.image_np, [detected_x, detected_y], 0, 255, 0,
                                   'tracked point', scale=0.7, offsetx=10, offsety=10)
                
                error_x = detected_x - desired_x
                error_y = detected_y - desired_y
                
                info_y = 30
                line_height = 30
                
                error_text = f"Pixel Error in x,y: ({error_x:.0f}, {error_y:.0f})"
                cv2.putText(self.image_np, error_text,
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, error_text,
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                depth_text = f"Estimated Distance to box: {self.distance:.2f} m"
                cv2.putText(self.image_np, depth_text,
                           (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, depth_text,
                           (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                box_width = self.blackbox_xmax - self.blackbox_xmin
                box_height = self.blackbox_ymax - self.blackbox_ymin
                box_text = f"Box size: {box_width:.0f} x {box_height:.0f} px"
                cv2.putText(self.image_np, box_text,
                           (10, info_y + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, box_text,
                           (10, info_y + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                vel_text = (f"PWM - Surge: {self.Camera_cooriction_surge}, "
                            f"Sway: {self.Camera_cooriction_sway}, "
                            f"Heave: {self.Camera_cooriction_heave}, "
                            f"Yaw: {self.Camera_cooriction_yaw}")
                cv2.putText(self.image_np, vel_text,
                           (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, vel_text,
                           (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1) 
                
                img_error_text = f"Image Error: ({self.s_error[0]:.4f}, {self.s_error[1]:.4f}), Depth Error: {self.z_error:.4f}"
                cv2.putText(self.image_np, img_error_text,
                           (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, img_error_text,
                           (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Display current gains and floatability
                gains_text = f"Gains [S,Sw,H,Y]: [{self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[2]:.2f}, {self.gains[3]:.2f}] | Float: {self.floatability:.2f}"
                cv2.putText(self.image_np, gains_text,
                           (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, gains_text,
                           (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Display control status
                #status_text = f"VS: {'ON' if self.enable_visual_servoing else 'OFF'} | Inv: S={self.invert_surge} Sw={self.invert_sway} H={self.invert_heave} Y={self.invert_yaw}"
                status_text = f"VS: {'ON' if self.enable_visual_servoing else 'OFF'} | Target: {'HANDLE' if self.track_handle else 'BOX'} | Inv: S={self.invert_surge} Sw={self.invert_sway} H={self.invert_heave} Y={self.invert_yaw}"
                cv2.putText(self.image_np, status_text,
                           (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, status_text,
                           (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if self.handle_xmin > 0 and self.handle_xmax > 0:
                cv2.rectangle(self.image_np,
                            (int(self.handle_xmin), int(self.handle_ymin)),
                            (int(self.handle_xmax), int(self.handle_ymax)),
                            (255, 0, 255), 2)
                
                handle_x = int(self.handle_xcenter)
                handle_y = int(self.handle_ycenter)
                self.overlay_points(self.image_np, [handle_x, handle_y], 255, 0, 255,
                                   'handle', scale=0.6, offsetx=10, offsety=10)
            
            cv2.imshow("Visual Servoing", self.image_np)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()