import sys
import cv2
import numpy as np
import rclpy
import subprocess
import os
import signal
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from PyQt5.QtWidgets import (QApplication, QLabel, QWidget, QGridLayout, 
                             QVBoxLayout, QHBoxLayout, QFrame, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

# --- CONFIGURATION ---
JETSON_B_IP = "192.168.1.20"
JETSON_B_USER = "capstone-nano2"
LOCAL_WS = "/home/capstone-nano1/TTT-Capstone-Sensors/tabletennistrainer_ws"
REMOTE_WS = "/home/capstone-nano2/Desktop/TTT-Capstone-Sensors/tabletennistrainer_ws"

class SystemLauncher(QThread):
    """Handles the background shell commands for startup and shutdown"""
    status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.processes = []

    def run(self):
        try:
            self.status_signal.emit("Tuning V4L2 Settings...")
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure=800", "-c", "analogue_gain=1200"])
            
            self.status_signal.emit("Launching Jetson A (Local)...")
            cmd_a = f"source /opt/ros/humble/setup.bash && source {LOCAL_WS}/install/setup.bash && ros2 launch ttt_bringup jetsonA.launch.py"
            proc_a = subprocess.Popen(cmd_a, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            self.processes.append(proc_a)

            self.status_signal.emit("Connecting to Jetson B (Remote)...")
            # We use SSH to trigger the remote launch file
            cmd_b = (f"ssh -tt {JETSON_B_USER}@{JETSON_B_IP} "
                     f"'v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200; "
                     f"source /opt/ros/humble/setup.bash; source {REMOTE_WS}/install/setup.bash; "
                     f"ros2 launch ttt_bringup jetsonB.launch.py'")
            proc_b = subprocess.Popen(cmd_b, shell=True, preexec_fn=os.setsid)
            self.processes.append(proc_b)

            self.status_signal.emit("System Online. Waiting for ROS stabilization...")
            time.sleep(2)
        except Exception as e:
            self.status_signal.emit(f"Startup Error: {str(e)}")

    def stop(self):
        self.status_signal.emit("Shutting down nodes...")
        # Kill local processes
        for p in self.processes:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        
        # Kill remote processes on Jetson B
        subprocess.run(["ssh", f"{JETSON_B_USER}@{JETSON_B_IP}", "pkill -f ttt_bringup"])
        self.status_signal.emit("System Offline.")

class ROSWorker(QThread):
    image_signal = pyqtSignal(np.ndarray, str)
    coords_signal = pyqtSignal(float, float, float)

    def run(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('marty_dashboard_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Matched to your C++ CameraNode topics
        self.node.create_subscription(CompressedImage, '/camera/left/compressed', self.left_callback, qos)
        self.node.create_subscription(CompressedImage, '/camera/right/compressed', self.right_callback, qos)
        
        # Matched to your StereoNode output
        self.node.create_subscription(PointStamped, '/ball_position_3d', self.stereo_callback, qos)
        
        rclpy.spin(self.node)

    def left_callback(self, msg): self.process_frame(msg, "left")
    def right_callback(self, msg): self.process_frame(msg, "right")
    def stereo_callback(self, msg): self.coords_signal.emit(msg.point.x, msg.point.y, msg.point.z)

    def process_frame(self, msg, side):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.image_signal.emit(frame, side)

class MartyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("M.A.R.T.Y. Control Center")
        self.setFixedSize(1400, 850)
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

        video_layout = QHBoxLayout()
        self.left_feed = self.create_video_label("LEFT CAMERA (A)")
        self.right_feed = self.create_video_label("RIGHT CAMERA (B)")
        video_layout.addWidget(self.left_feed)
        video_layout.addWidget(self.right_feed)
        main_layout.addLayout(video_layout)

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
        self.setLayout(main_layout)

    def create_video_label(self, text):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("border: 2px solid #444; background-color: #000;")
        lbl.setMinimumSize(640, 400)
        return lbl

    def create_stat_label(self, text, color, font):
        lbl = QLabel(text); lbl.setFont(font); lbl.setStyleSheet(f"color: {color};"); lbl.setAlignment(Qt.AlignCenter)
        return lbl

    def update_status(self, text): self.status_bar.setText(f"SYSTEM STATUS: {text}")

    def update_image(self, frame, side):
        h, w, ch = frame.shape
        q_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img).scaled(640, 400, Qt.KeepAspectRatio)
        if side == "left": self.left_feed.setPixmap(pixmap)
        else: self.right_feed.setPixmap(pixmap)

    def update_stats(self, x, y, z):
        self.lbl_x.setText(f"X: {x:.2f}"); self.lbl_y.setText(f"Y: {y:.2f}"); self.lbl_z.setText(f"Z: {z:.2f}")

    def closeEvent(self, event):
        """Overrides window close button to ensure processes die"""
        reply = QMessageBox.question(self, 'Quit', "Shut down all M.A.R.T.Y. nodes?", 
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
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