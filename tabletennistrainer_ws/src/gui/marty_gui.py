import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QGridLayout, QVBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
import numpy as np

class ROSWorker(QThread):
    image_signal = pyqtSignal(np.ndarray, str)
    data_signal = pyqtSignal(float, float)

    def run(self):
        rclpy.init()
        self.node = Node('marty_gui_node')
        
        # Subscribers
        self.node.create_subscription(CompressedImage, '/camera_local/compressed', 
                                      self.local_callback, 10)
        self.node.create_subscription(CompressedImage, '/camera_remote/compressed', 
                                      self.remote_callback, 10)
        self.node.create_subscription(Point, '/ball_tracker/coords', 
                                      self.data_callback, 10)
        
        self.count = 0
        rclpy.spin(self.node)

    def data_callback(self, msg):
        self.data_signal.emit(msg.x, msg.y)

    def process_frame(self, msg, name):
        self.count += 1
        # Throttling to 60 FPS (Assuming 240 FPS input)
        if self.count % 4 == 0:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.image_signal.emit(frame, name)

    def local_callback(self, msg): self.process_frame(msg, "local")
    def remote_callback(self, msg): self.process_frame(msg, "remote")

class MartyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("M.A.R.T.Y. Control Center (PyQt5)")
        self.init_ui()
        
        self.worker = ROSWorker()
        self.worker.image_signal.connect(self.update_image)
        self.worker.data_signal.connect(self.update_data)
        self.worker.start()

    def init_ui(self):
        layout = QGridLayout()
        
        # Display Labels
        self.local_view = QLabel("Local Feed")
        self.remote_view = QLabel("Remote Feed")
        self.coord_label = QLabel("Ball Coords: (0, 0)")
        
        # Styling
        self.coord_label.setStyleSheet("font-size: 20pt; color: #00FF00; font-weight: bold;")
        self.local_view.setAlignment(Qt.AlignCenter)
        self.remote_view.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.local_view, 0, 0)
        layout.addWidget(self.remote_view, 0, 1)
        layout.addWidget(self.coord_label, 1, 0, 1, 2, Qt.AlignCenter)
        
        self.setLayout(layout)
        self.setStyleSheet("background-color: #121212; color: white;")

    def update_image(self, frame, name):
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        # PyQt5 uses QImage.Format_RGB888 (We use BGR order from OpenCV)
        q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img).scaled(640, 360, Qt.KeepAspectRatio)
        
        if name == "local": self.local_view.setPixmap(pixmap)
        else: self.remote_view.setPixmap(pixmap)

    def update_data(self, x, y):
        self.coord_label.setText(f"Ball Tracking: X={x:.1f} Y={y:.1f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MartyDashboard()
    window.resize(1300, 500)
    window.show()
    sys.exit(app.exec_())