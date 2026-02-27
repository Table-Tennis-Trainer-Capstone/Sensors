from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    calibration_dir = get_package_share_directory('ttt_calibration')
    bringup_dir = get_package_share_directory('ttt_bringup')
    
    return LaunchDescription([
        # Launch Calibration (TF transforms)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_dir, 'launch', 'calibration.launch.py')
            )
        ),
        
        # Jetson A - Left Camera + Vision
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'width': 640,
                'height': 400,
                'fps': 240,
                'show_window': False
            }],
            output='screen'
        ),
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_left',
            parameters=[{
                'camera_id': 'left',
                'min_brightness': 200,
                'min_radius': 5,
                'max_radius': 50,
                'blur_size': 5,
                'show_window': True
            }],
            output='screen'
        ),
        
        # Jetson B - Right Camera + Vision 
        # Node(
        #     package='ttt_camera',
        #     executable='camera_node',
        #     name='camera_right',
        #     parameters=[{
        #         'device': '/dev/video1',  # Different camera
        #         'camera_id': 'right',
        #         'width': 640,
        #         'height': 400,
        #         'fps': 240,
        #         'show_window': False
        #     }],
        #     output='screen'
        # ),
        
        # 4. Future nodes (commented out for now)
        # Node(
        #     package='ttt_vision',
        #     executable='stereo_fusion_node',
        #     name='stereo_fusion',
        #     output='screen'
        # ),
        # Node(
        #     package='ttt_control',
        #     executable='arm_controller_node',
        #     name='arm_controller',
        #     output='screen'
        # ),
        # Node(
        #     package='ttt_hardware',
        #     executable='stm32_bridge_node',
        #     name='stm32_bridge',
        #     output='screen'
        # ),
    ])
