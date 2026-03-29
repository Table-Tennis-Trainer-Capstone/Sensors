from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=""),
        LogInfo(msg="========================================"),
        LogInfo(msg="  TABLE CORNER CALIBRATION"),
        LogInfo(msg="========================================"),
        LogInfo(msg=""),
        LogInfo(msg="Place ArUco markers at table corners:"),
        LogInfo(msg("  ID 0: Front-left"),
        LogInfo(msg="  ID 1: Front-right"),
        LogInfo(msg="  ID 2: Back-right"),
        LogInfo(msg="  ID 3: Back-left"),
        LogInfo(msg=""),
        LogInfo(msg="Press 's' to save when all 4 detected"),
        LogInfo(msg="========================================"),
        
        # Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'width': 1280,
                'height': 800,
                'fps': 60,
                'show_window': False
            }],
            output='screen'
        ),
        
        # Table corner calibration
        Node(
            package='ttt_calibration',
            executable='table_corner_calibration_node',
            name='table_corner_calibration',
            parameters=[{
                'camera_id': 'left',
                'marker_size': 0.10,  # 10cm markers
                'dictionary_id': 10,
                'show_detection': True
            }],
            output='screen'
        )
    ])
