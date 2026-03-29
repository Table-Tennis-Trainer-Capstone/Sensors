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
                'width': 1280,
                'height': 800,
                'fps': 120,
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
                'min_radius': 2,
                'max_radius': 60,
                'min_brightness': 50,
                'min_contrast': 1,
                'min_circularity': 0.15,
                'max_aspect_ratio': 3.0,
                'edge_margin': 5,
                'show_window': False,
            }],
            output='screen'
        ),
    ])
