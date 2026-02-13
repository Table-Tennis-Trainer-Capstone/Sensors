# TABLE TENNIS TRAINER
## Spring 2026 Capstone Project

A high-speed, vision-guided robotic system designed to autonomously engage in a Table Tennis match with a human opponent. This project utilizes a distributed ROS2 architecture across two NVIDIA Jetson Nanos to handle GPU-accelerated vision processing, interfaced via Ethernet communication to an STM32 microcontroller which will command the joint positions on the arm.

## Contributors

### Sensors Team

Liam Dabelstein

Jake VanEssendelft

Lucian Bracy

John Duplain

### Mechanical Team

Nathaniel LeBlanc

Ben Pumphrey

Grant Monroe

Zane Peeler

Tae Lee

Theodore Williamson

## System Architecture & Hardware Integration

The project utilizes a distributed hardware stack to minimize processing latency. High-level perception is handled by NVIDIA Jetsons, while low-level motor execution is managed by an STM32 microcontroller.

### **Hardware Components**
* **Compute:** 2x NVIDIA Jetson Nano (Developer Kit).
* **Vision:** 2x Arducam OV2311 (Global Shutter, MIPI CSI-2).
* **Microcontroller:** STM32 (connected via Ethernet).
* **Actuation:** add section here @mech team.
* **Networking:** Gigabit Ethernet Switch.


### **Hardware Mapping**
| Component | Connection | Purpose |
| :--- | :--- | :--- |
| **Arducams** | MIPI CSI-2 | 120 FPS Global Shutter image capture at 1280x720 resolution. |
| **Jetson A ↔ B** | Ethernet (Static IP) | Sharing 2D ball coordinates for stereo fusion. |
| **Jetson A ↔ STM32** | Ethernet (UDP) | Sending joint angles to the arm at high frequency. |
| **STM32 ↔ Motors** | PWM / Step-Dir | Direct electrical control of the arm. |

---

### ROS 2 Package Glossary

Each package in the `src/` directory is designed with **Modularity** in mind and serves a specific purpose in the system.

### **1. ttt_msgs**
* **Type:** Interface Package.
* **Function:** Defines the custom "language" of the robot. It contains the `.msg` files for ball detection, tracking, and arm status.

### **2. ttt_camera**
* **Type:** Hardware Driver.
* **Function:** Interfaces with the MIPI CSI-2 ports on the Jetson. It uses a hardware-accelerated GStreamer pipeline to provide raw grayscale frames.
* **Output:** `sensor_msgs/Image` (120 FPS).

### **3. ttt_vision**
* **Type:** Perception Engine.
* **Function:** The "eyes and brain" of the system. It handles:
    * **Detection:** Finding the ball in 2D pixels (using GPU/VPI).
    * **Stereo Fusion:** Triangulating two 2D points into a 3D (X, Y, Z) coordinate.
    * **Prediction:** Estimating where the ball will be using a Kalman Filter.

### **4. ttt_control**
* **Type:** Decision Maker.
* **Function:** Performs **Inverse Kinematics (IK)**. It takes the predicted ball destination and calculates the exact joint angles required for the paddle to hit the ball.

### **5. ttt_hardware**
* **Type:** Communication Bridge.
* **Function:** The "translator." It takes ROS 2 joint commands and packs them into a binary UDP format that the **STM32** can understand with minimal overhead.

### **6. ttt_calibration**
* **Type:** Configuration.
* **Function:** Stores the physical offsets of the cameras (e.g., the 45° tilt). It publishes the **TF2 (Transform)** tree so the vision system knows where the table is relative to the cameras.

### **7. ttt_bringup**
* **Type:** System Orchestrator.
* **Function:** Contains Python launch scripts. Allows for start of the entire system with a single command.

## 🌐 System Topology

### Jetson A (`insert IP address here`)
- **Nodes:** `camera_left`, `ball_detector_left`, `stereo_fusion`, `predictor`, `ik_solver`, `stm32_bridge`.
- **Hardware:** Left Arducam, Ethernet to STM32, Ethernet to Slave Jetson.

### Jetson B (`insert IP address here`)
- **Nodes:** `camera_right`, `ball_detector_right`.
- **Hardware:** Right Arducam.

#### Package Structure

All packages in this workspace follow the standard ROS 2 C++ layout.

Using `ttt_vision` as an example:
```text
ttt_vision/
├── CMakeLists.txt          # Build instructions
├── package.xml             # Metadata and dependencies
├── include/
│   └── ttt_vision/         # Header files
│       └── ball_math.hpp   # Declarations and shared logic
├── src/
│   ├── ball_detector.cpp   # Node implementation
│   └── ball_tracker.cpp    # Node implementation
└── msg/                    # (Only in ttt_msgs) Custom data types

```

### How to Edit and Add Features
#### **1. Editing Headers (.hpp)**
* **Location:** Always place headers in `include/<package_name>/`.
* If you add a new header, ensure ament_export_include_directories(include) is in CMakeLists.txt so other packages can see it.

#### **2. Adding a New Node (.cpp)**
* **Location:** Place the implementation file in the `src/` directory.
* **Registration:** You must register the new node in the `CMakeLists.txt` file so the compiler knows to build it. Add the following lines:

```cmake
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp ttt_msgs)

# This part ensures the executable is installed to the lib folder
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

#### 3. **Building and Running a Single Package**
* **Build:** `colcon build --symlink-install --packages-select <package_name>`
* **Source:** `source install/setup.bash`
* **Run:** `ros2 run <package_name> <executable_name>`


### Building the Project
We use `colcon` with symlink installation to avoid redundant builds.

```bash
# Standard Build
./build.sh

# If the Jetson freezes (RAM conservation mode)
colcon build --symlink-install --parallel-workers 1

```
