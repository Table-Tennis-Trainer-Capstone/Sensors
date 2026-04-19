import sys, os; sys.path.insert(0, os.path.dirname(__file__)); from calibration import PARAMS
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
                'width': PARAMS['width'],
                'height': PARAMS['height'],
                'fps': PARAMS['fps'],
                'exposure': PARAMS['exposure'],
                'analogue_gain': PARAMS['analogue_gain'],
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
                'min_area': PARAMS['min_area'],
                'max_area': PARAMS['max_area'],
                'min_contrast': PARAMS['min_contrast'],
                'dilate_iters': PARAMS['dilate_iters'],
                'edge_margin': PARAMS['edge_margin'],
                'kf_gate_px': PARAMS.get('kf_gate_px', 250.0),
                'kf_process_noise': PARAMS.get('kf_process_noise', 0.05),
                'consistency_min': PARAMS.get('consistency_min', 1),
                'frame_delay': PARAMS.get('frame_delay', 5),
                'max_aspect_ratio': PARAMS.get('max_aspect_ratio', 3.5),
                **({'table_roi': PARAMS['table_roi_right']} if PARAMS['table_roi_right'] else {}),
            }],
            output='screen'
        ),
    ])
