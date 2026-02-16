from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Left Camera (High-Speed MIPI CSI-2)
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'sensor_id': 0,
                'camera_id': 'left',
                'width': 1280,           # Updated for high-speed HD
                'height': 720,           # Updated for high-speed HD
                'fps': 120
            }],
            output='screen'
        ),

        # Left Vision (Ball Detector with VPI GPU)
        Node(
            package='ttt_vision',
           executable='ball_detector_node',
            name='ball_detector_left',
            parameters=[{
                'camera_id': 'left',
                'min_brightness': 200,
                'min_radius': 3,         # Shrunk slightly for 720p scale
                'max_radius': 30,
                'blur_size': 3           # Reduced blur for sharper fast edges
            }],
            output='screen'
        )
    ]) 
