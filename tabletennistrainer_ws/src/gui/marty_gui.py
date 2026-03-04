import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point  
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap
import numpy as np

class ROSWorker(QThread):
    # Signals to update the UI from the background thread
    image_signal = pyqtSignal(np.ndarray, str) # frame, camera_name
    data_signal = pyqtSignal(float, float)     # x, y

    def run(self):
        rclpy.init()
        self.node = Node('gui_subscriber')
        
        # Subscribers
        self.node.create_subscription(CompressedImage, '/camera_local/image/compressed', 
                                      self.local_callback, 10)
        self.node.create_subscription(CompressedImage, '/camera_remote/image/compressed', 
                                      self.remote_callback, 10)
        self.node.create_subscription(Point, '/ball_tracker/coords', 
                                      self.data_callback, 10)
        
        self.count = 0
        rclpy.spin(self.node)

    def data_callback(self, msg):
        self.data_signal.emit(msg.x, msg.y)

    def process_frame(self, msg, name):
        self.count += 1
        # Throttle to 60 FPS (240 / 4)
        if self.count % 4 == 0:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image_signal.emit(frame, name)

    def local_callback(self, msg): self.process_frame(msg, "local")
    def remote_callback(self, msg): self.process_frame(msg, "remote")

class MartyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("M.A.R.T.Y. Vision Control")
        self.init_ui()
        
        self.worker = ROSWorker()
        self.worker.image_signal.connect(self.update_image)
        self.worker.data_signal.connect(self.update_data)
        self.worker.start()

    def init_ui(self):
        layout = QGridLayout()
        
        # Camera Feed Labels
        self.local_view = QLabel("Local Feed (Jetson B)")
        self.remote_view = QLabel("Remote Feed (Jetson A)")
        
        # Data Labels
        self.coord_label = QLabel("Ball Coords: (0, 0)")
        self.coord_label.setStyleSheet("font-size: 18px; font-weight: bold; color: green;")
        
        layout.addWidget(self.local_view, 0, 0)
        layout.addWidget(self.remote_view, 0, 1)
        layout.addWidget(self.coord_label, 1, 0, 1, 2, alignment=Qt.AlignmentFlag.AlignCenter)
        
        self.setLayout(layout)

    def update_image(self, frame, name):
        # Convert OpenCV to QImage
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(q_img).scaled(640, 360, Qt.AspectRatioMode.KeepAspectRatio)
        
        if name == "local": self.local_view.setPixmap(pixmap)
        else: self.remote_view.setPixmap(pixmap)

    def update_data(self, x, y):
        self.coord_label.setText(f"Ball Coords: X={x:.2f}, Y={y:.2f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MartyDashboard()
    window.show()
    sys.exit(app.exec())