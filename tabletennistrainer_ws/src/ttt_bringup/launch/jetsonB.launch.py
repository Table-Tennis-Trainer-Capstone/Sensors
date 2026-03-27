from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Left Camera (physically on Jetson B)
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'width': 640,
                'height': 400,
                'fps': 60,
                'exposure': 15000,
                'analogue_gain': 32000,
            }],
            output='screen'
        ),

        # Left Vision
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_left',
            parameters=[{
                'camera_id': 'left',
                'min_radius': 2,
                'max_radius': 25,
                'min_brightness': 8,
                'min_contrast': 4,
                'min_circularity': 0.25,
                'max_aspect_ratio': 3.5,
                'diff_threshold': 15,
                'noise_cell_size': 8,
                'noise_suppress_thresh': 4.0,
                'noise_decay_per_sec': 0.3,
                'edge_margin': 20,
                'show_window': False,
            }],
            output='screen'
        ),
    ])
