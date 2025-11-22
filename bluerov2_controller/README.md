# BlueROV2 Vision Controller

This package implements an **Image-Based Visual Servoing (IBVS)** controller for the BlueROV2. It subscribes to object detections and camera images to autonomously control the ROV's thrusters.

## Main Goals
1.  **Error Calculation:** Determines the difference between the detected object's center and the desired setpoint.
2.  **Visual Servoing:** Uses an interaction matrix to calculate necessary velocities (Surge, Sway, Heave, Yaw).
3.  **PWM Output:** Converts velocities into RC override signals (PWM 1100-1900) to drive the thrusters directly via MAVROS.

## How to Launch

You can launch the controller using the provided launch file. 

**Basic Launch:**
```bash
ros2 launch bluerov2_controller bluerov2_Vision_Controller.launch.py
```

**Launch with Custom Namespace:**
```bash
ros2 launch bluerov2_controller bluerov2_Vision_Controller.launch.py namespace:=my_bluerov
```

## Editable Parameters

All parameters can be modified in real-time (dynamically) without restarting the node.

### 1. Control Gains & Logic
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `enable_visual_servoing` | `False` | **Master Switch.** Set to `True` to start autonomous movement. |
| `gain_surge` | `0.6` | Aggressiveness of forward/backward movement. |
| `gain_sway` | `0.5` | Aggressiveness of lateral movement. |
| `gain_heave` | `0.5` | Aggressiveness of vertical movement. |
| `gain_yaw` | `0.5` | Aggressiveness of rotation. |
| `floatability` | `-0.18` | Constant heave offset to compensate for buoyancy. |

### 2. Target Settings
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `track_handle` | `False` | `False` = Track Box Center. `True` = Track Handle Center. |
| `desired_point_x` | `-1.0` | Target pixel X. Set to `-1` to auto-lock to image center. |
| `desired_point_y` | `-1.0` | Target pixel Y. Set to `-1` to auto-lock to image center. |
| `desired_point_z` | `0.5` | Desired distance from the target (in meters). |

### 3. Safety & Hardware Limits
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `v_linear_max` | `0.3` | Max linear velocity limit (m/s). |
| `v_angular_max` | `0.5` | Max angular velocity limit (rad/s). |
| `invert_surge` | `False` | Invert surge thruster direction. |
| `invert_sway` | `False` | Invert sway thruster direction. |
| `invert_heave` | `True` | Invert heave thruster direction. |
| `invert_yaw` | `True` | Invert yaw thruster direction. |

## GUI Parameter Tuning (rqt)

You can use the ROS 2 GUI tools to adjust gains and switches visually using sliders and checkboxes.

1.  Open a terminal and run:
    ```bash
    rqt
    ```
2.  In the top menu bar, go to **Plugins** > **Configuration** > **Dynamic Reconfigure** > **Parameter Reconfigure**.
3.  In the left-hand tree view, find and click on: `/bluerov2/bluerov2_Vision_Controller`.
4.  The panel will populate with the parameters.
5.  **Note:** Changes made here take effect immediately.

## Topics

| Type | Topic | Direction | Description |
| :--- | :--- | :--- | :--- |
| `sensor_msgs/Image` | `/bluerov/camera/image_color` | Subscriber | Simulation Camera stream for visual feedback. |
| `sensor_msgs/Image` | `camera/image` | Subscriber | Real Camera stream for visual feedback. |
| `bluerov2_interface/Detection` | `detections` | Subscriber | Bounding box coordinates of the target. |
| `mavros_msgs/OverrideRCIn` | `rc/override` | Publisher | PWM commands to flight controller (Channels 1-6). |

## 🔧 CLI Runtime Usage Example

If you prefer the command line over `rqt`, use these commands:

```bash
# Enable control
ros2 param set /bluerov2/bluerov2_Vision_Controller enable_visual_servoing True

# Change target to Handle
ros2 param set /bluerov2/bluerov2_Vision_Controller track_handle True
```
