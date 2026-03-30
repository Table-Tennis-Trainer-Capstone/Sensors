from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Right Camera (physically on Jetson B)
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
                'exposure': 7000,
                'analogue_gain': 1200,
            }],
            output='screen'
        ),

        # Right Vision
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_right',
            parameters=[{
                'camera_id': 'right',
                'min_area': 1,
                'max_area': 2000,
                'motion_threshold': 5,
                'min_contrast': 5,
                'dilate_iters': 1,
                'edge_margin': 10,
            }],
            output='screen'
        ),
    ])
