from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_right',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'right',
                'width': 1280,
                'height': 800,
                'fps': 120,
                'exposure': 7000,
                'analogue_gain': 1500,
            }],
            output='screen'
        ),
    ])