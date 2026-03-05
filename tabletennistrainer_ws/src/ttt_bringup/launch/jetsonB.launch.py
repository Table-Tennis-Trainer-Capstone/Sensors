from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    calibration_dir = get_package_share_directory('ttt_calibration')
    
    return LaunchDescription([
        # Launch calibration (same transforms on both Jetsons)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_dir, 'launch', 'calibration.launch.py')
            )
        ),
        
        # Right Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_right',
            parameters=[{
                'device': '/dev/video0',  # Still video0 on this Jetson
                'camera_id': 'right',
                'width': 640,
                'height': 400,
                'fps': 240,
                'show_window': False
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
                'min_brightness': 25,
                'min_radius': 5,
                'max_radius': 50,
                'blur_size': 5,
                'show_window': True
            }],
            output='screen'
        )
    ])
