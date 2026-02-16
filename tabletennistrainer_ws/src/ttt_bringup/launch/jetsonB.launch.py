from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Right Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_right',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'right',
                'width': 640,
                'height': 400,
                'fps': 60
            }],
            output='screen'
        ),
        
        # Right Vision (Ball Detector with VPI GPU)
        Node(
            package='ttt_vision',
            executable='ball_detector_node',
            name='ball_detector_right',
            parameters=[{
                'camera_id': 'right',
                'min_brightness': 200,
                'min_radius': 5,
                'max_radius': 50,
                'blur_size': 5
            }],
            output='screen'
        )
    ])

