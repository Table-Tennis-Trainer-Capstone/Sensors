# TABLE TENNIS TRAINER
## Spring 2026 Capstone Project

A high-speed, vision-guided robotic system designed to autonomously engage in a Table Tennis match with a human opponent. This project utilizes a distributed ROS2 architecture across two NVIDIA Jetson Orin Nanos with GPU-accelerated vision processing, interfaced via Ethernet to an STM32 microcontroller for real-time motor control.

## Contributors

### Sensors Team
- Liam Dabelstein
- Jake VanEssendelft
- Lucian Bracy
- John Duplain

### Mechanical Team
- Nathaniel LeBlanc
- Ben Pumphrey
- Grant Monroe
- Zane Peeler
- Tae Lee
- Theodore Williamson

---

## System Architecture & Hardware Integration

The project utilizes a distributed hardware stack to minimize processing latency. High-level perception is handled by NVIDIA Jetson Orin Nanos with GPU acceleration, while low-level motor execution is managed by an STM32 microcontroller.

### Hardware Components
* **Compute:** 2x NVIDIA Jetson Orin Nano Super (8GB RAM, 1024-core CUDA GPU)
* **Vision:** 2x Arducam OV9281 (1MP Global Shutter, Monochrome, MIPI CSI-2)
* **Microcontroller:** STM32 (connected via Ethernet UDP)
* **Actuation:** [Add mechanical details here @mech team]
* **Networking:** Gigabit Ethernet Switch

### Camera Specifications
| Specification | Value |
| :--- | :--- |
| **Sensor** | OV9281 (1MP Global Shutter) |
| **Resolution** | 640×400 pixels |
| **Frame Rate** | 240 FPS |
| **Interface** | MIPI CSI-2 (4-lane) |
| **Format** | Monochrome (GREY/Y10/Y16) |
| **Shutter** | Global (no rolling shutter artifacts) |

### Hardware Mapping
| Component | Connection | Purpose |
| :--- | :--- | :--- |
| **Arducam OV9281** | MIPI CSI-2 (CAM1 port) | 240 FPS global shutter capture at 640x400 |
| **Jetson A <-> B** | Ethernet (Gigabit, Static IPs) | ROS2 DDS for sharing 2D ball detections |
| **Jetson A <-> STM32** | Ethernet (UDP, 192.168.1.100) | Sending joint angles at high frequency |
| **STM32 <-> Motors** | PWM / Step-Dir | Direct motor control |

### Network Configuration
- **Jetson A IP:** `192.168.1.10`
- **Jetson B IP:** `192.168.1.20`
- **STM32 IP:** `192.168.1.100`
- **ROS_DOMAIN_ID:** `42`

---

## ROS 2 Package Glossary

Each package in the `src/` directory is designed with **modularity** in mind.

### 1. ttt_msgs
* **Type:** Interface Package
* **Function:** Custom message definitions
* **Messages:**
  - `BallDetection.msg` - 2D ball position (x, y, radius, confidence)
  - `BallTrack.msg` - 3D tracked ball (position, velocity, tracked status)

### 2. ttt_camera
* **Type:** Hardware Driver
* **Function:** V4L2 interface to Arducam OV9281 via MIPI CSI-2
* **Output:** `sensor_msgs/Image` (MONO8, 240 FPS)
* **Features:** 
  - Hardware-accelerated capture
  - Configurable exposure/gain via v4l2-ctl
  - Optional live preview window

### 3. ttt_vision
* **Type:** GPU-Accelerated Perception Engine
* **Function:** Ball detection using CUDA OpenCV
* **Pipeline:**
  1. GPU upload (cv::cuda::GpuMat)
  2. GPU Gaussian blur (cv::cuda::createGaussianFilter)
  3. GPU Hough circles detection
  4. CPU brightness filtering (select brightest candidate)
* **Output:** `ttt_msgs/BallDetection` (x, y, radius, confidence)

### 4. ttt_stereo
* **Type:** 3D Reconstruction
* **Function:** Stereo triangulation from dual 2D detections
* **Algorithm:**
  - Time-synchronized detection matching (ApproximateTime policy)
  - Disparity calculation: `disparity = left_x - right_x`
  - Depth: `Z = (fx × baseline) / disparity`
  - 3D position: `(X, Y, Z)` in camera frame
* **Output:** `geometry_msgs/PointStamped` (3D ball position)

### 5. ttt_trajectory
* **Type:** Predictive Tracking
* **Function:** Physics-based trajectory prediction with bounce handling
* **Features:**
  - Rolling buffer of 3D samples
  - Linear fit for X, Z motion
  - Parabolic fit for Y (gravity-affected)
  - Bounce physics (table collision with restitution coefficient)
  - Landing point prediction
* **Output:** 
  - `/ball_trajectory/predicted` - Future position
  - `/ball_trajectory/landing` - Table landing point

### 6. ttt_control
* **Type:** Motion Planning
* **Function:** Inverse kinematics (IK) solver
* **Features:** Utilizes MoveIt2 and OMPL (RRTConnect) to calculate and execute joint trajectories. Implements custom joint constraints to maintain optimal paddle orientation and avoid singularities or self-collisions.

### 7. ttt_hardware
* **Type:** Hardware Bridge
* **Function:** UDP communication to STM32
* **Protocol:** UDP stream parsing joint states
* **Target:** `192.168.1.100:5000`
* **Status:** Implemented. Bridges ROS2 `/joint_states` to the STM32 via high-frequency UDP messages.

### 8. ttt_calibration
* **Type:** Spatial Configuration
* **Function:** Camera pose estimation and TF2 transforms
* **Output:** TF2 transforms (table -> root -> robot_base)

### 9. ttt_bringup
* **Type:** System Orchestrator
* **Function:** Launch scripts for coordinated startup
* **Launch Files:**
  - `calibration.py` - Central configuration dictionary for all nodes
  - `marty_gui.py` - Flask application acting as the master control interface

---

## System Topology

### Jetson A (`192.168.1.10`)
**Nodes:** 
- `camera_left` - OV9281 capture (240 FPS)
- `vision_node` (left) - GPU ball detection
- `stereo_node` - 3D triangulation
- `trajectory_node` - Physics prediction
- `tf_broadcaster` - Coordinate transforms

**Hardware:** 
- Left Arducam OV9281 (MIPI CSI-2)
- Ethernet to Jetson B (ROS2 DDS)
- Ethernet to STM32 (UDP)

### Jetson B (`192.168.1.20`)
**Nodes:**
- `camera_right` - OV9281 capture (240 FPS)
- `vision_node` (right) - GPU ball detection
- `tf_broadcaster` - Coordinate transforms

**Hardware:**
- Right Arducam OV9281 (MIPI CSI-2)
- Ethernet to Jetson A (ROS2 DDS)

### Data Flow
```
Camera Feed (240 FPS)
    ↓
GPU Detection (~2-5ms)
    ↓
2D Ball Position (left + right)
    ↓
Stereo Triangulation
    ↓
3D Ball Position
    ↓
Trajectory Prediction
    ↓
Landing Point Estimation
    ↓
Inverse Kinematics
    ↓
STM32 Joint Commands
```

---

## M.A.R.T.Y. Control Center (GUI)

A PyQt5-based dashboard (`marty_gui.py`) provides:

### Features
- **Dual camera feeds** - Live video from both Jetsons with synchronized display
- **3D ball position display** - Real-time X, Y, Z coordinates
- **Built-in ArUco calibration mode** - Visual marker detection overlay
- **One-click calibration** - Automatic camera pose estimation and config update
- **System orchestration** - SSH-based launch of both Jetsons
- **Status monitoring** - Real-time system health display

### Calibration Mode
1. Click **"Calibration Mode"** button
2. Position ArUco markers (IDs 0-3) at table corners
3. Markers detected and labeled in real-time
4. Click **"Save Calibration"** when 3+ markers visible
5. System automatically:
   - Runs calibration on both Jetsons
   - Parses results
   - Updates `stereo_extrinsic.yaml`
   - Rebuilds calibration package
6. Restart M.A.R.T.Y. to apply changes

### Usage
```bash
cd ~/TTT-Capstone-Sensors/tabletennistrainer_ws
source install/setup.bash
python3 src/ttt_bringup/scripts/marty_gui.py
```

---

## Camera Calibration Guide

### Overview
Two cameras mounted at **45° downward angles** above opposite table sides enable stereo vision while minimizing occlusion.

### Calibration Types

#### 1. Intrinsic Calibration (Camera Properties)
- **What:** Focal length, principal point, lens distortion
- **Method:** Default values provided (500px focal length for OV9281 @ 640×400)
- **When to calibrate:** When sub-centimeter accuracy needed

**Default intrinsics:**
```yaml
camera_matrix:
  data: [500.0, 0.0, 320.0,
         0.0, 500.0, 200.0,
         0.0, 0.0, 1.0]
distortion_coefficients:
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
```

#### 2. Extrinsic Calibration (Camera Position)

##### A. ArUco Marker Calibration (Recommended)

**Advantages:**
- ✅ Automatic - no manual measurements
- ✅ Accurate to ±1-2mm
- ✅ Fast - calibrates both cameras in <30 seconds
- ✅ Repeatable - easy to recalibrate if cameras move
- ✅ Integrated into GUI

**Requirements:**
- 4× ArUco markers (DICT_6X6_250, IDs 0-3)
- Printed at 10cm × 10cm each
- Placed at table corners

**Print Markers:**
1. Visit: https://chev.me/arucogen/
2. Settings:
   - Dictionary: **6x6 (250 markers)**
   - Marker ID: **0, 1, 2, 3** (generate separately)
   - Marker size: **100mm**
3. Print at **100% scale** (no scaling!)
4. Verify: Measure with ruler (should be exactly 10cm × 10cm)
5. Mount on cardboard for rigidity

**Marker Placement:**
```
                                    Table (Top View)
                              ┌─────────────────────────┐
                              │    [0]         [2]      │ 
        Front (your side) ->  │                         │ <- Back (opponent)
                              │                         │
                              │    [1]         [3]      │  
                              └─────────────────────────┘

ID 0: Front-left corner
ID 1: Front-right corner
ID 2: Back-right corner
ID 3: Back-left corner
```

**Calibration Steps:**

**Method 1: Using GUI (Easiest)**
```bash
# Launch M.A.R.T.Y. dashboard
python3 src/ttt_bringup/scripts/marty_gui.py

# Steps:
# 1. Click "Calibration Mode"
# 2. Position markers at table corners
# 3. Wait for 3+ markers detected in each camera
# 4. Click "Save Calibration"
# 5. Restart GUI
```

**Method 2: Manual Launch**
```bash
# Launch calibration on each Jetson
# Jetson A (left camera):
ros2 launch ttt_bringup calibrate_table_launch.py

# Jetson B (right camera):
ros2 launch ttt_bringup calibrate_table_launch.py camera_id:=right

# Press 'c' to calibrate, 's' to save
# Results saved to /tmp/camera_left_calibration.yaml
```

##### B. Manual Measurement (Quick but Less Accurate)

**Tools:** Tape measure, phone level app

**Steps:**
1. Measure camera positions from table center
2. Use level app to verify 45° angle
3. Update `stereo_extrinsic.yaml` manually

---

### Calibration File Structure
```
ttt_calibration/
├── config/
│   ├── camera_left_intrinsic.yaml     # Left camera lens parameters
│   ├── camera_right_intrinsic.yaml    # Right camera lens parameters
│   ├── stereo_extrinsic.yaml          # Camera poses (auto-updated by ArUco)
│   └── table_markers.yaml             # ArUco marker positions
├── launch/
│   ├── calibration.launch.py          # TF broadcaster
│   └── calibrate_table_launch.py      # ArUco calibration mode
└── src/
    ├── tf_broadcaster_node.cpp        # Publishes TF transforms
    └── aruco_calibrate_camera.cpp     # ArUco-based pose estimation
```

### Example Calibrated Values

After ArUco calibration, `stereo_extrinsic.yaml` will contain:
```yaml
/**:
  ros__parameters:
    camera_baseline: 1.524  # Auto-calibrated distance between cameras

    left_camera:
      x: -0.762      # 76.2cm left of table center
      y: 0.143       # 14.3cm toward opponent
      z: 0.987       # 98.7cm above table
      roll: 0.012    # 0.7° roll
      pitch: -0.798  # 45.7° downward
      yaw: -0.023    # 1.3° yaw correction

    right_camera:
      x: 0.762       # 76.2cm right of table center
      y: 0.138       # Similar forward position
      z: 0.991       # Similar height
      roll: -0.008
      pitch: -0.792
      yaw: 0.018
```

### Verification
```bash
# Launch TF broadcaster
ros2 launch ttt_calibration calibration.launch.py

# Verify transforms
ros2 run tf2_tools view_frames  # Generates frames.pdf
ros2 run tf2_ros tf2_echo table camera_left_optical_frame

# Expected output:
# Translation: [-0.762, 0.143, 0.987]
# Rotation (RPY): [0.012, -0.798, -0.023]
#         (degrees): [0.7°, -45.7°, -1.3°]
```

---

## Building the Project

### Standard Build
```bash
cd ~/TTT-Capstone-Sensors/tabletennistrainer_ws
./build.sh
source install/setup.bash
```

### Build Script Contents
```bash
#!/bin/bash
if [ "$1" == "clean" ]; then
    rm -rf build install log
fi
colcon build --symlink-install
```

### Memory-Constrained Build
If Jetson freezes during build (RAM exhaustion):
```bash
colcon build --symlink-install --parallel-workers 1
```

### Build Single Package
```bash
colcon build --packages-select ttt_vision
source install/setup.bash
```

---

## Running the System

### Full System Launch

**On Jetson A:**
```bash
cd ~/TTT-Capstone-Sensors/tabletennistrainer_ws
source install/setup.bash
ros2 launch ttt_bringup jetsonA.launch.py
```

**On Jetson B:**
```bash
cd ~/TTT-Capstone-Sensors/tabletennistrainer_ws
source install/setup.bash
ros2 launch ttt_bringup jetsonB.launch.py
```

**Or use GUI (launches both automatically):**
```bash
python3 src/ttt_bringup/scripts/marty_gui.py
```

### Testing Individual Components

**Camera only:**
```bash
ros2 launch ttt_bringup camera_left.launch.py
ros2 topic hz /camera/left/image_raw  # Should show ~240 Hz
```

**Vision with detection overlay:**
```bash
ros2 launch ttt_bringup jetsonA.launch.py
# Window shows green circles on detected balls
```

**Monitor 3D positions:**
```bash
ros2 topic echo /ball_position_3d
# Output: x, y, z in meters relative to table
```

**Monitor trajectory predictions:**
```bash
ros2 topic echo /ball_trajectory/landing
# Shows predicted table landing point
```

---

## Camera Tuning

### V4L2 Controls
```bash
# List available controls
v4l2-ctl -d /dev/video0 --list-ctrls

# Set exposure and gain for bright ball tracking
v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200

# Auto-exposure off (manual control)
v4l2-ctl -d /dev/video0 -c auto_exposure=1
```

### Optimal Settings (for white ping pong ball)
```bash
v4l2-ctl -d /dev/video0 \
  -c exposure=800 \
  -c analogue_gain=1200 \
  -c auto_exposure=1
```

These settings are automatically applied by `marty_gui.py` on startup.

---

## Troubleshooting

### Camera Not Detected
```bash
# Check camera connection
ls /dev/video*  # Should show /dev/video0

# Check formats
v4l2-ctl -d /dev/video0 --list-formats-ext

# Verify MIPI connection
dmesg | grep ov9281
```

### Network Issues (Jetsons Can't See Each Other)
```bash
# Check ROS_DOMAIN_ID matches on both
echo $ROS_DOMAIN_ID  # Should be 42

# Test connectivity
ping 192.168.1.10  # From Jetson B to A
ping 192.168.1.20  # From Jetson A to B

# Check topics visible
ros2 topic list | grep ball_detection
# Should see /ball_detection/left and /ball_detection/right
```

### GPU Not Working
```bash
# Verify CUDA OpenCV installation
python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())"
# Should output: 1

# Check GPU usage during vision
tegrastats  # Monitor GPU utilization

# Rebuild OpenCV with CUDA if needed
~/OpenCV-4-10-0.sh
```

### Calibration Not Saving
```bash
# Check file permissions
ls -l /tmp/camera_*_calibration.yaml

# Verify ArUco detection
ros2 topic echo /camera/left/image_raw  # Camera running?
ros2 run ttt_calibration aruco_calibrate_camera  # Manual test
```

---

## Performance Metrics

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 4.2ms | 240 FPS = 4.17ms period |
| GPU ball detection | 2-5ms | CUDA-accelerated pipeline |
| Stereo triangulation | <1ms | Simple math |
| Trajectory prediction | <1ms | 5-20 sample buffer |
| **Total perception** | **~10ms** | Camera → 3D prediction |

### Bandwidth
- **Image data:** 640×400×1 byte × 240 FPS = **60 MB/s per camera**
- **ROS2 detections:** ~100 bytes × 240 Hz = **24 KB/s per camera**
- **3D positions:** ~50 bytes × 240 Hz = **12 KB/s**

---

## Package Structure Example

Using `ttt_vision` as reference:
```text
ttt_vision/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Dependencies and metadata
├── include/
│   └── ttt_vision/         # Public headers (if needed)
├── src/
│   └── vision_node.cpp     # GPU-accelerated ball detector
└── launch/                 # Launch file (optional)
```

### Adding a New Node

1. **Create source file:**
```bash
nano src/ttt_vision/src/my_node.cpp
```

2. **Register in CMakeLists.txt:**
```cmake
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
  ttt_msgs
  OpenCV
)
target_link_libraries(my_node ${OpenCV_LIBS})

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

3. **Build and run:**
```bash
colcon build --packages-select ttt_vision
source install/setup.bash
ros2 run ttt_vision my_node
```

---

## Quick Reference Commands

### System Control
```bash
# Launch full system (GUI)
python3 src/ttt_bringup/scripts/marty_gui.py

# Launch manually
ros2 launch ttt_bringup jetsonA.launch.py  # Jetson A
ros2 launch ttt_bringup jetsonB.launch.py  # Jetson B

# Stop all nodes
pkill -f ttt_bringup
```

### Monitoring
```bash
# List active nodes
ros2 node list

# List topics
ros2 topic list

# Monitor ball detections
ros2 topic echo /ball_detection/left
ros2 topic hz /ball_detection/left  # Check rate

# Monitor 3D positions
ros2 topic echo /ball_position_3d

# View coordinate transforms
ros2 run tf2_tools view_frames
```

### Calibration
```bash
# ArUco calibration (GUI)
python3 src/ttt_bringup/scripts/marty_gui.py
# → Click "Calibration Mode"

# ArUco calibration (manual)
ros2 launch ttt_bringup calibrate_table_launch.py

# Verify transforms
ros2 run tf2_ros tf2_echo table camera_left_optical_frame
```

### Camera Control
```bash
# Tune exposure/gain
v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200

# Test camera
ros2 launch ttt_bringup camera_left.launch.py
ros2 topic hz /camera/left/image_raw  # Should be ~240 Hz
```

---

## Resources

### Documentation
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [OpenCV Module](https://docs.opencv.org/4.x/)
- [Jetson Orin Nano](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [Arducam OV9281](https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/OV9281/)

### Calibration Resources
- [Camera Calibration (ROS Wiki)](http://wiki.ros.org/camera_calibration)
- [Stereo Vision Tutorial](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [Online ArUco Generator](https://chev.me/arucogen/)

### Team Resources
- [Project Repository](https://github.com/jrv11706/TTT-Capstone-Sensors/tree/main)
- [Mechanical CAD Files](link-to-cad)

---

---

## Acknowledgments
