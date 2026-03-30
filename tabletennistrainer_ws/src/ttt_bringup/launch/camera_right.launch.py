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
                'width': 640,
                'height': 400,
                'fps': 240,
                'exposure': 1500,
                'analogue_gain': 400,
            }],
            output='screen'
        ),
    ])