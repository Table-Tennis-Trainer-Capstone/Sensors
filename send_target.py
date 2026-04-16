#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class TargetSender(Node):
    def __init__(self):
        super().__init__('target_sender')
        self.pub = self.create_publisher(PointStamped, '/ball_trajectory/predicted', 10)
        self.get_logger().info('Target Sender ready. Sending target...')

    def send(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'table'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.pub.publish(msg)
        self.get_logger().info(f'Sent prediction to table (X:{x}, Y:{y}, Z:{z})')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 4:
        print("Usage: ros2 run ttt_tests send_target.py <X> <Y> <Z>")
        print("Example: ros2 run ttt_tests send_target.py 0.5 0.2 -0.8")
        sys.exit(1)

    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    
    node = TargetSender()
    time.sleep(0.5) # allow ROS DDS discovery to connect to ttt_control
    node.send(x, y, z)
    time.sleep(0.1) # allow packet to leave before killing node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()