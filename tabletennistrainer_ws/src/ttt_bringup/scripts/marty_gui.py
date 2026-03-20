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
        # Kill local process groups (launch files + all children)
        for p in self.processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except:
                pass
        # Kill any remaining local ROS2 processes
        subprocess.run("pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; pkill -f ttt_", shell=True)
        # Kill all ROS2 processes on nano1
        subprocess.run(["ssh", f"{JETSON_A_USER}@{JETSON_A_IP}",
                        "pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; pkill -f ttt_"])
        self.status_signal.emit("System Offline.")


class CalibrationWorker(QThread):
    """Runs ArUco detection in its own thread at ~10 fps.
    process_frame submits frames; the thread stores the latest result for overlay."""
    status_signal = pyqtSignal(str)

    NAMES = {0: "Front-Left", 1: "Front-Right", 2: "Back-Right", 3: "Back-Left"}

    def __init__(self):
        super().__init__()
        self.running = True
        self._pending = {}        # side -> latest raw frame
        self._lock = __import__('threading').Lock()
        # last detection result per side: list of (corners_array, marker_id)
        self.last_result = {'left': [], 'right': []}

    def submit_frame(self, frame, side):
        """Called from ROS thread — just stores the latest frame, non-blocking."""
        with self._lock:
            self._pending[side] = frame.copy()

    def run(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 3
        params.minMarkerPerimeterRate = 0.005
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.15
        params.maxErroneousBitsInBorderRate = 0.5
        params.errorCorrectionRate = 0.6
        params.minCornerDistanceRate = 0.01
        params.minDistanceToBorder = 1

        self.status_signal.emit("Calibration mode ready")

        import time
        while self.running:
            with self._lock:
                pending = dict(self._pending)
                self._pending.clear()

            for side, frame in pending.items():
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
                result = []
                if ids is not None:
                    for i, mid in enumerate(ids.flatten()):
                        result.append((corners[i][0], int(mid)))
                self.last_result[side] = result

            total = sum(len(v) for v in self.last_result.values())
            if total > 0:
                self.status_signal.emit(f"Markers detected: {total}/4")

            time.sleep(0.1)  # ~10 fps detection rate

    def draw_overlay(self, frame, side):
        """Draw cached detection result onto frame — called from ROS thread, fast."""
        results = self.last_result.get(side, [])
        count = 0
        for corner, mid in results:
            pts = corner.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            center = pts.mean(axis=0).astype(int)
            cv2.putText(frame, f"ID {mid}", (center[0]-30, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if mid in self.NAMES:
                cv2.putText(frame, self.NAMES[mid], (center[0]-40, center[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            count += 1

        status = f"Markers: {count}/4"
        color = (0, 255, 0) if count >= 3 else (0, 165, 255)
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if count >= 3:
            cv2.putText(frame, "Ready to calibrate!", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    def stop(self):
        self.running = False


def draw_axis_indicator(frame):
    """Draw XYZ axis arrows in the bottom-left corner of the frame."""
    h, w = frame.shape[:2]
    origin = (55, h - 55)
    length = 40

    # X = red  → right
    # Y = green → up (height above table)
    # Z = blue  → up-right diagonal (depth, into scene)
    axes = [
        ((length, 0),        (0, 0, 255),   'X'),   # X right  (BGR: red)
        ((0, -length),       (0, 255, 0),   'Y'),   # Y up     (BGR: green)
        ((length//2, -length//2), (255, 0, 0), 'Z'),# Z depth  (BGR: blue)
    ]

    # Semi-transparent background box
    box_tl = (origin[0] - 15, origin[1] - length - 20)
    box_br = (origin[0] + length + 20, origin[1] + 15)
    overlay = frame.copy()
    cv2.rectangle(overlay, box_tl, box_br, (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

    for (dx, dy), color, label in axes:
        tip = (origin[0] + dx, origin[1] + dy)
        cv2.arrowedLine(frame, origin, tip, color, 2, tipLength=0.3)
        label_pos = (tip[0] + 4, tip[1] + 4)
        cv2.putText(frame, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


class ROSWorker(QThread):
    image_signal = pyqtSignal(np.ndarray, str)
    coords_signal = pyqtSignal(float, float, float)
    predicted_signal = pyqtSignal(float, float, float)
    landing_signal = pyqtSignal(float, float)

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
        self.node.create_subscription(PointStamped, '/ball_trajectory/predicted', self.predicted_callback, qos)
        self.node.create_subscription(PointStamped, '/ball_trajectory/landing', self.landing_callback, qos)
        self.node.create_subscription(BallDetection, '/ball_detection/left', lambda m: self.detection_callback(m, 'left'), qos)
        self.node.create_subscription(BallDetection, '/ball_detection/right', lambda m: self.detection_callback(m, 'right'), qos)

        rclpy.spin(self.node)

    def left_callback(self, msg): self.process_frame(msg, "left")
    def right_callback(self, msg): self.process_frame(msg, "right")
    def stereo_callback(self, msg): self.coords_signal.emit(msg.point.x, msg.point.y, msg.point.z)
    def predicted_callback(self, msg): self.predicted_signal.emit(msg.point.x, msg.point.y, msg.point.z)
    def landing_callback(self, msg): self.landing_signal.emit(msg.point.x, msg.point.z)
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
                self.calibration_worker.submit_frame(frame, side)   # non-blocking
                self.calibration_worker.draw_overlay(frame, side)   # draws cached result
            
            draw_axis_indicator(frame)
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
        self.worker.predicted_signal.connect(self.update_predicted)
        self.worker.landing_signal.connect(self.update_landing)
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
        val_font  = QFont("Consolas", 22, QFont.Bold)
        pred_font = QFont("Consolas", 16)

        self.lbl_x = self.create_stat_label("X: 0.00", "#ff5555", val_font)
        self.lbl_y = self.create_stat_label("Y: 0.00", "#55ff55", val_font)
        self.lbl_z = self.create_stat_label("Z: 0.00", "#5555ff", val_font)

        self.lbl_px = self.create_stat_label("pX: --", "#ff9999", pred_font)
        self.lbl_py = self.create_stat_label("pY: --", "#99ff99", pred_font)
        self.lbl_pz = self.create_stat_label("pZ: --", "#9999ff", pred_font)

        self.lbl_land = self.create_stat_label("Landing: --", "#ffcc00", pred_font)

        hdr_current = QLabel("CURRENT POSITION (m)")
        hdr_current.setStyleSheet("color: #aaa; font-size: 11px;")
        hdr_predicted = QLabel("PREDICTED +250ms (m)")
        hdr_predicted.setStyleSheet("color: #aaa; font-size: 11px;")

        data_layout.addWidget(hdr_current,  0, 0, 1, 3, Qt.AlignCenter)
        data_layout.addWidget(self.lbl_x,   1, 0)
        data_layout.addWidget(self.lbl_y,   1, 1)
        data_layout.addWidget(self.lbl_z,   1, 2)
        data_layout.addWidget(hdr_predicted, 2, 0, 1, 3, Qt.AlignCenter)
        data_layout.addWidget(self.lbl_px,  3, 0)
        data_layout.addWidget(self.lbl_py,  3, 1)
        data_layout.addWidget(self.lbl_pz,  3, 2)
        data_layout.addWidget(self.lbl_land, 4, 0, 1, 3, Qt.AlignCenter)

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

    def update_predicted(self, x, y, z):
        self.lbl_px.setText(f"pX: {x:.2f}")
        self.lbl_py.setText(f"pY: {y:.2f}")
        self.lbl_pz.setText(f"pZ: {z:.2f}")

    def update_landing(self, x, z):
        self.lbl_land.setText(f"Landing  X: {x:.2f}  Z: {z:.2f}")

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


def _emergency_cleanup():
    """Called on Ctrl+C or SIGTERM to ensure nodes are killed."""
    subprocess.run("pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; pkill -f ttt_", shell=True)
    subprocess.run(["ssh", f"{JETSON_A_USER}@{JETSON_A_IP}",
                    "pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; pkill -f ttt_"])


if __name__ == "__main__":
    import atexit
    atexit.register(_emergency_cleanup)
    signal.signal(signal.SIGTERM, lambda *_: (_emergency_cleanup(), sys.exit(0)))

    app = QApplication(sys.argv)
    window = MartyDashboard()
    window.show()
    sys.exit(app.exec_())