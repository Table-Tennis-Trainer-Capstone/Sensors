from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    calibration_dir = get_package_share_directory('ttt_calibration')

    return LaunchDescription([
        # Launch calibration first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_dir, 'launch', 'calibration.launch.py')
            )
        ),

        # right Camera
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
                'exposure': 6000,
                'analogue_gain': 8000,
            }],
            output='screen'
        ),

        # right Vision
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_right',
            parameters=[{
                'camera_id': 'right',
                'min_brightness': 10,
                'min_radius': 5,
                'max_radius': 50,
                'show_window': False,   # Draws detection overlay on screen
            }],
            output='screen'
        ),

        # Stereo Vision
       Node (
           package='ttt_stereo',
           executable='stereo_node',
           parameters=[{
               'baseline_m': 1.5,
               'fx': 224.1,
               'fy': 200.0,
               'cx': 320.0,
               'cy': 200.0,
               'max_sync_age_ms': 100,
           }],
       ),

       Node(
           package='ttt_trajectory',
           executable='trajectory_node',
           name='trajectory_node',
           parameters=[{
               'lookahead_ms': 250,
               'min_samples': 5,
               'max_samples': 20,
               'gravity': 9.81,
               'table_y': -0.5,    # tune this to your camera mount height
               'restitution': 0.85,
           }],
           output='screen'
       ),
    ])
