import sys
import cv2
import numpy as np
import subprocess
import os
import signal
import time

# Must be set before rclpy/DDS initializes
os.environ.setdefault('ROS_DOMAIN_ID', '42')
# Ensure workspace messages are findable
import sys as _sys
_ws_python = '/home/capstone-nano2/TTT-Capstone-Sensors/tabletennistrainer_ws/install/ttt_msgs/local/lib/python3.10/dist-packages'
if _ws_python not in _sys.path:
    _sys.path.insert(0, _ws_python)
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI', 'file:///home/capstone-nano2/cyclonedds.xml')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from ttt_msgs.msg import BallDetection
from PyQt5.QtWidgets import (QApplication, QLabel, QWidget, QGridLayout, 
                             QVBoxLayout, QHBoxLayout, QFrame, QMessageBox, 
                             QPushButton, QTabWidget)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

# --- CONFIGURATION ---
JETSON_A_IP = "192.168.1.10"
JETSON_A_USER = "capstone-nano1"

LOCAL_WS = "/home/capstone-nano2/TTT-Capstone-Sensors/tabletennistrainer_ws"
REMOTE_WS = "/home/capstone-nano1/TTT-Capstone-Sensors/tabletennistrainer_ws"

class SystemLauncher(QThread):
    """Handles the background shell commands for startup and shutdown"""
    status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.processes = []

    def run(self):
        try:
            self.status_signal.emit("Tuning Local V4L2 Settings (Jetson B)...")
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure=800", "-c", "analogue_gain=1200"])
            
            self.status_signal.emit("Launching Jetson B (Local)...")
            cmd_b = f"export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///home/capstone-nano2/cyclonedds.xml && source /opt/ros/humble/setup.bash && source {LOCAL_WS}/install/setup.bash && ros2 launch ttt_bringup jetsonB.launch.py"
            proc_b = subprocess.Popen(cmd_b, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            self.processes.append(proc_b)

            self.status_signal.emit("Connecting to Jetson A (Remote)...")
            cmd_a = (f"ssh -tt {JETSON_A_USER}@{JETSON_A_IP} "
                     f"'v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200; "
                     f"export ROS_DOMAIN_ID=42; "
                     f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
                     f"export CYCLONEDDS_URI=file:///home/capstone-nano1/cyclonedds.xml; "
                     f"source /opt/ros/humble/setup.bash; source {REMOTE_WS}/install/setup.bash; "
                     f"ros2 launch ttt_bringup jetsonA.launch.py'")
            proc_a = subprocess.Popen(cmd_a, shell=True, preexec_fn=os.setsid)
            self.processes.append(proc_a)

            self.status_signal.emit("System Online. Waiting for ROS stabilization...")
            time.sleep(2)
        except Exception as e:
            self.status_signal.emit(f"Startup Error: {str(e)}")

    def stop(self):
        self.status_signal.emit("Shutting down nodes...")
        for p in self.processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except:
                pass
        
        subprocess.run(["ssh", f"{JETSON_A_USER}@{JETSON_A_IP}", "pkill -f ttt_bringup"])
        self.status_signal.emit("System Offline.")


class CalibrationWorker(QThread):
    """Handles ArUco marker detection for calibration"""
    status_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        try:
            # ArUco setup
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.parameters = cv2.aruco.DetectorParameters()
            self.parameters.adaptiveThreshWinSizeMin = 3
            self.parameters.adaptiveThreshWinSizeMax = 53
            self.parameters.adaptiveThreshWinSizeStep = 4
            self.parameters.minMarkerPerimeterRate = 0.01
            self.parameters.maxMarkerPerimeterRate = 4.0
            self.parameters.polygonalApproxAccuracyRate = 0.1
            
            # Marker positions on table (standard table tennis table)
            self.marker_positions = {
                0: (-0.7625, -1.37, 0.0),   # Front-left
                1: (0.7625, -1.37, 0.0),    # Front-right
                2: (0.7625, 1.37, 0.0),     # Back-right
                3: (-0.7625, 1.37, 0.0),    # Back-left
            }
            
            # Camera intrinsics (approximate)
            self.camera_matrix = np.array([
                [320.0, 0.0, 320.0],
                [0.0, 320.0, 200.0],
                [0.0, 0.0, 1.0]
            ])
            self.dist_coeffs = np.zeros((1, 5))
            
            self.status_signal.emit("Calibration mode ready")
            
        except Exception as e:
            self.status_signal.emit(f"Calibration init error: {str(e)}")
    
    def detect_markers(self, frame):
        """Detect ArUco markers and draw on frame"""
        if frame is None or len(frame.shape) != 3:
            return frame, []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        
        detected_ids = []
        
        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i, marker_id in enumerate(ids.flatten()):
                detected_ids.append(int(marker_id))
                
                # Get marker corners
                corner = corners[i][0]
                center = corner.mean(axis=0).astype(int)
                
                # Draw marker info
                label = f"ID {marker_id}"
                cv2.putText(frame, label, (center[0] - 30, center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw marker name
                if marker_id in [0, 1, 2, 3]:
                    names = {0: "Front-Left", 1: "Front-Right", 
                            2: "Back-Right", 3: "Back-Left"}
                    cv2.putText(frame, names[marker_id], 
                               (center[0] - 40, center[1] + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return frame, detected_ids
    
    def stop(self):
        self.running = False


class ROSWorker(QThread):
    image_signal = pyqtSignal(np.ndarray, str)
    coords_signal = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.calibration_mode = False
        self.calibration_worker = None

    def run(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('marty_dashboard_node')
        self.latest_detection = {'left': None, 'right': None}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.node.create_subscription(CompressedImage, '/camera/left/compressed', self.left_callback, qos)
        self.node.create_subscription(CompressedImage, '/camera/right/compressed', self.right_callback, qos)
        self.node.create_subscription(PointStamped, '/ball_position_3d', self.stereo_callback, qos)
        self.node.create_subscription(BallDetection, '/ball_detection/left', lambda m: self.detection_callback(m, 'left'), qos)
        self.node.create_subscription(BallDetection, '/ball_detection/right', lambda m: self.detection_callback(m, 'right'), qos)

        rclpy.spin(self.node)

    def left_callback(self, msg): self.process_frame(msg, "left")
    def right_callback(self, msg): self.process_frame(msg, "right")
    def stereo_callback(self, msg): self.coords_signal.emit(msg.point.x, msg.point.y, msg.point.z)
    def detection_callback(self, msg, side): self.latest_detection[side] = msg

    def process_frame(self, msg, side):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is not None:
            # Draw ball detection overlay
            det = self.latest_detection.get(side)
            if det is not None and det.confidence > 0:
                cx, cy, r = int(det.x), int(det.y), max(int(det.radius), 5)
                cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

            # Apply ArUco detection if in calibration mode
            if self.calibration_mode and self.calibration_worker:
                frame, detected = self.calibration_worker.detect_markers(frame)
                
                # Draw status on frame
                status_text = f"Markers: {len(detected)}/4 detected"
                cv2.putText(frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                if len(detected) >= 3:
                    cv2.putText(frame, "Ready to calibrate!", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, "Need 3+ markers", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            
            self.image_signal.emit(frame, side)
    
    def enable_calibration(self):
        self.calibration_mode = True
        self.calibration_worker = CalibrationWorker()
        self.calibration_worker.start()
    
    def disable_calibration(self):
        self.calibration_mode = False
        if self.calibration_worker:
            self.calibration_worker.stop()
            self.calibration_worker = None


class MartyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("M.A.R.T.Y. Control Center")
        self.setFixedSize(1400, 900)
        self.calibration_mode = False
        self.init_ui()
        
        # Start Launcher
        self.launcher = SystemLauncher()
        self.launcher.status_signal.connect(self.update_status)
        self.launcher.start()

        # Start ROS
        self.worker = ROSWorker()
        self.worker.image_signal.connect(self.update_image)
        self.worker.coords_signal.connect(self.update_stats)
        self.worker.start()

    def init_ui(self):
        self.setStyleSheet("background-color: #121212; color: #e0e0e0;")
        main_layout = QVBoxLayout()

        # Status Bar
        self.status_bar = QLabel("Initializing...")
        self.status_bar.setStyleSheet("background-color: #333; padding: 5px; color: #00FF00; font-family: Consolas;")
        main_layout.addWidget(self.status_bar)

        # Control Buttons
        button_layout = QHBoxLayout()
        
        self.calib_button = QPushButton("📐 Calibration Mode")
        self.calib_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 10px;
                font-size: 14px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:checked {
                background-color: #FF9800;
            }
        """)
        self.calib_button.setCheckable(True)
        self.calib_button.clicked.connect(self.toggle_calibration)
        button_layout.addWidget(self.calib_button)
        
        self.save_calib_button = QPushButton("💾 Save Calibration")
        self.save_calib_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 10px;
                font-size: 14px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #555;
                color: #999;
            }
        """)
        self.save_calib_button.setEnabled(False)
        self.save_calib_button.clicked.connect(self.save_calibration)
        button_layout.addWidget(self.save_calib_button)
        
        button_layout.addStretch()
        main_layout.addLayout(button_layout)

        # Video Feeds
        video_layout = QHBoxLayout()
        self.left_feed = self.create_video_label("LEFT CAMERA (A)")
        self.right_feed = self.create_video_label("RIGHT CAMERA (B)")
        video_layout.addWidget(self.left_feed)
        video_layout.addWidget(self.right_feed)
        main_layout.addLayout(video_layout)

        # Data Frame
        data_frame = QFrame()
        data_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; border: 1px solid #333;")
        data_layout = QGridLayout(data_frame)
        val_font = QFont("Consolas", 28, QFont.Bold)

        self.lbl_x = self.create_stat_label("X: 0.00", "#ff5555", val_font)
        self.lbl_y = self.create_stat_label("Y: 0.00", "#55ff55", val_font)
        self.lbl_z = self.create_stat_label("Z: 0.00", "#5555ff", val_font)

        data_layout.addWidget(QLabel("3D BALL POSITION (METERS)"), 0, 0, 1, 3, Qt.AlignCenter)
        data_layout.addWidget(self.lbl_x, 1, 0)
        data_layout.addWidget(self.lbl_y, 1, 1)
        data_layout.addWidget(self.lbl_z, 1, 2)

        main_layout.addWidget(data_frame)
        
        # Calibration Instructions (hidden by default)
        self.calib_instructions = QLabel()
        self.calib_instructions.setStyleSheet("""
            background-color: #FF9800;
            color: white;
            padding: 10px;
            border-radius: 5px;
            font-size: 12px;
        """)
        self.calib_instructions.setWordWrap(True)
        self.calib_instructions.setText(
            "CALIBRATION MODE:\n"
            "1. Place 10cm ArUco markers at table corners (IDs 0-3)\n"
            "2. Ensure 3+ markers are visible in each camera\n"
            "3. Click 'Save Calibration' when markers are detected\n"
            "4. Results will be saved to /tmp/ - update config files manually"
        )
        self.calib_instructions.hide()
        main_layout.addWidget(self.calib_instructions)
        
        self.setLayout(main_layout)

    def create_video_label(self, text):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("border: 2px solid #444; background-color: #000;")
        lbl.setMinimumSize(640, 400)
        return lbl

    def create_stat_label(self, text, color, font):
        lbl = QLabel(text)
        lbl.setFont(font)
        lbl.setStyleSheet(f"color: {color};")
        lbl.setAlignment(Qt.AlignCenter)
        return lbl

    def update_status(self, text):
        self.status_bar.setText(f"SYSTEM STATUS: {text}")

    def update_image(self, frame, side):
        h, w, ch = frame.shape
        q_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img).scaled(640, 400, Qt.KeepAspectRatio)
        if side == "left":
            self.left_feed.setPixmap(pixmap)
        else:
            self.right_feed.setPixmap(pixmap)

    def update_stats(self, x, y, z):
        self.lbl_x.setText(f"X: {x:.2f}")
        self.lbl_y.setText(f"Y: {y:.2f}")
        self.lbl_z.setText(f"Z: {z:.2f}")

    def toggle_calibration(self):
        self.calibration_mode = self.calib_button.isChecked()
        
        if self.calibration_mode:
            self.calib_button.setText("🔍 Calibrating...")
            self.calib_instructions.show()
            self.save_calib_button.setEnabled(True)
            self.worker.enable_calibration()
            self.update_status("Calibration Mode Active - Position ArUco markers")
        else:
            self.calib_button.setText("📐 Calibration Mode")
            self.calib_instructions.hide()
            self.save_calib_button.setEnabled(False)
            self.worker.disable_calibration()
            self.update_status("Normal Operation")

    def save_calibration(self):
        reply = QMessageBox.question(
            self, 'Save Calibration',
            "Save current camera calibration?\n\n"
            "This will trigger calibration on both Jetsons.\n"
            "Make sure markers are visible!",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                self.update_status("Calibrating left camera...")
                
                # Run ArUco calibration for left camera
                left_result = subprocess.run([
                    "bash", "-c",
                    f"ssh {JETSON_A_USER}@{JETSON_A_IP} "
                    f"'source {REMOTE_WS}/install/setup.bash && "
                    f"timeout 10 ros2 run ttt_calibration aruco_calibrate_camera "
                    f"--ros-args -p camera_id:=left -p auto_calibrate:=true'"
                ], capture_output=True, text=True, timeout=15)
                
                self.update_status("Calibrating right camera...")
                
                # Run ArUco calibration for right camera  
                right_result = subprocess.run([
                    "bash", "-c",
                    f"source {LOCAL_WS}/install/setup.bash && "
                    f"timeout 10 ros2 run ttt_calibration aruco_calibrate_camera "
                    f"--ros-args -p camera_id:=right -p auto_calibrate:=true"
                ], capture_output=True, text=True, timeout=15)
                
                # Copy left calibration from remote
                subprocess.run([
                    "scp",
                    f"{JETSON_A_USER}@{JETSON_A_IP}:/tmp/camera_left_calibration.yaml",
                    "/tmp/"
                ])
                
                # Parse calibration results
                left_calib = self.parse_calibration_file("/tmp/camera_left_calibration.yaml")
                right_calib = self.parse_calibration_file("/tmp/camera_right_calibration.yaml")
                
                if not left_calib or not right_calib:
                    raise Exception("Failed to parse calibration files - ensure markers were visible")
                
                # Update config
                config_path = os.path.join(LOCAL_WS, "src/ttt_calibration/config/stereo_extrinsic.yaml")
                self.update_stereo_config(config_path, left_calib, right_calib)
                
                # Rebuild
                self.update_status("Rebuilding calibration package...")
                build_result = subprocess.run([
                    "bash", "-c",
                    f"cd {LOCAL_WS} && colcon build --packages-select ttt_calibration"
                ], capture_output=True, text=True, timeout=30)
                
                if build_result.returncode != 0:
                    raise Exception(f"Build failed: {build_result.stderr}")
                
                self.update_status("✅ Calibration complete!")
                
                # Show success dialog
                baseline = abs(left_calib['x'] - right_calib['x'])
                QMessageBox.information(
                    self, 'Calibration Complete',
                    "Calibration successful!\n\n"
                    f"Updated: {config_path}\n\n"
                    "Camera positions:\n"
                    f"Left:  ({left_calib['x']:+.3f}, {left_calib['y']:+.3f}, {left_calib['z']:.3f}) m\n"
                    f"Right: ({right_calib['x']:+.3f}, {right_calib['y']:+.3f}, {right_calib['z']:.3f}) m\n"
                    f"Baseline: {baseline:.3f} m\n\n"
                    "Restart M.A.R.T.Y. for changes to take effect"
                )
                
            except subprocess.TimeoutExpired:
                QMessageBox.warning(
                    self, 'Calibration Timeout',
                    "Calibration took too long.\n\n"
                    "Ensure:\n"
                    "  • Markers are visible in both cameras\n"
                    "  • Cameras are running\n"
                    "  • Network connection is stable"
                )
                self.update_status("Calibration timed out")
            
            except Exception as e:
                QMessageBox.warning(
                    self, 'Calibration Error',
                    f"Calibration failed:\n{str(e)}\n\n"
                    "Check that:\n"
                    "  • 3+ markers visible in each camera\n"
                    "  • aruco_calibrate_camera node is built\n"
                    "  • Cameras are running"
                )
            self.update_status("Calibration failed")

    def parse_calibration_file(self, filepath):
        """Parse calibration YAML file and extract camera pose"""
        try:
            if not os.path.exists(filepath):
                print(f"Calibration file not found: {filepath}")
                return None
            
            with open(filepath, 'r') as f:
                lines = f.readlines()
            
            calib = {}
            in_position = False
            in_orientation = False
            
            for line in lines:
                line = line.strip()
                
                # Track which section we're in
                if 'position:' in line:
                    in_position = True
                    in_orientation = False
                elif 'orientation_radians:' in line:
                    in_position = False
                    in_orientation = True
                
                # Parse position values
                if in_position:
                    if 'x:' in line:
                        calib['x'] = float(line.split(':')[1].strip())
                    elif 'y:' in line:
                        calib['y'] = float(line.split(':')[1].strip())
                    elif 'z:' in line:
                        calib['z'] = float(line.split(':')[1].strip())
                
                # Parse orientation values
                if in_orientation:
                    if 'roll:' in line:
                        calib['roll'] = float(line.split(':')[1].strip())
                    elif 'pitch:' in line:
                        calib['pitch'] = float(line.split(':')[1].strip())
                    elif 'yaw:' in line:
                        calib['yaw'] = float(line.split(':')[1].strip())
            
            # Verify we got all values
            required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            if all(key in calib for key in required_keys):
                print(f"Parsed {filepath}: x={calib['x']:.3f}, y={calib['y']:.3f}, z={calib['z']:.3f}")
                return calib
            else:
                print(f"Missing keys in {filepath}: {[k for k in required_keys if k not in calib]}")
                return None
            
        except Exception as e:
            print(f"Error parsing {filepath}: {e}")
            return None

    def update_stereo_config(self, config_path, left_calib, right_calib):
        """Update stereo_extrinsic.yaml with calibrated values"""
        import shutil
        from datetime import datetime
        
        # Calculate baseline (distance between cameras)
        baseline = abs(left_calib['x'] - right_calib['x'])
        
        # Create backup with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = config_path + f".backup_{timestamp}"
        shutil.copy(config_path, backup_path)
        print(f"Backup saved: {backup_path}")
        
        # Generate new YAML content
        yaml_content = f"""/**:
        ros__parameters:
        camera_baseline: {baseline:.6f}  # Auto-calibrated {timestamp}

        left_camera:
            x: {left_calib['x']:.6f}      # Auto-calibrated
            y: {left_calib['y']:.6f}      # Auto-calibrated
            z: {left_calib['z']:.6f}      # Auto-calibrated
            roll: {left_calib['roll']:.6f}    # Auto-calibrated
            pitch: {left_calib['pitch']:.6f}  # Auto-calibrated
            yaw: {left_calib['yaw']:.6f}      # Auto-calibrated

        right_camera:
            x: {right_calib['x']:.6f}     # Auto-calibrated
            y: {right_calib['y']:.6f}     # Auto-calibrated
            z: {right_calib['z']:.6f}     # Auto-calibrated
            roll: {right_calib['roll']:.6f}   # Auto-calibrated
            pitch: {right_calib['pitch']:.6f} # Auto-calibrated
            yaw: {right_calib['yaw']:.6f}     # Auto-calibrated

        robot_base:
            x: 0.0
            y: -1.0
            z: 0.0
            roll: 0.0
            pitch: 0.0
            yaw: 0.0
        """
        
        # Write updated config
        with open(config_path, 'w') as f:
            f.write(yaml_content)
        
        print(f"Updated: {config_path}")
        print(f"  Left camera:  ({left_calib['x']:+.3f}, {left_calib['y']:+.3f}, {left_calib['z']:.3f})")
        print(f"  Right camera: ({right_calib['x']:+.3f}, {right_calib['y']:+.3f}, {right_calib['z']:.3f})")
        print(f"  Baseline: {baseline:.3f} m")

    def closeEvent(self, event):
        """Overrides window close button to ensure processes die"""
        reply = QMessageBox.question(
            self, 'Quit',
            "Shut down all M.A.R.T.Y. nodes?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.launcher.stop()
            event.accept()
        else:
            event.ignore()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MartyDashboard()
    window.show()
    sys.exit(app.exec_())