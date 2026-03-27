from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    calibration_dir = get_package_share_directory('ttt_calibration')
    moveit_config_dir = get_package_share_directory('ttt_control')

    return LaunchDescription([
        # Launch calibration first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_dir, 'launch', 'calibration.launch.py')
            )
        ),

        # Robot State Publisher (publishes URDF + joint TF transforms)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py')
            )
        ),

        # MoveIt move_group (IK solver + motion planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
            )
        ),

        # ros2_control + arm_controller + joint_state_broadcaster
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'controllers.launch.py')
            )
        ),

        # Bridge URDF root link to TF tree (robot_base → root, identity transform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_base_to_root',
            arguments=['0', '0', '0', '0', '0', '0', 'robot_base', 'root'],
        ),

        # Left Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'right',
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
            name='ball_detector_right',
            parameters=[{
                'camera_id': 'right',
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
                'edge_margin': 30,
                'show_window': False,
            }],
            output='screen'
        ),

        # Stereo Vision (receives right detection from Jetson B via DDS)
        Node(
            package='ttt_stereo',
            executable='stereo_node',
            parameters=[{
                'baseline_m': 1.5,
                'fx': 224.1,
                'fy': 200.0,
                'cx': 320.0,
                'cy': 200.0,
                'max_sync_age_ms': 50,
            }],
        ),

        # Trajectory Prediction
        Node(
            package='ttt_trajectory',
            executable='trajectory_node',
            name='trajectory_node',
            parameters=[{
                'lookahead_ms': 100,
                'min_samples': 3,
                'max_samples': 12,
                'gravity': 9.81,
                'camera_tilt_deg': 31.0,
                'table_y': 4.5,    # MEASURE: place ball on table, read /ball_position_3d Y
                'restitution': 0.85,
            }],
            output='screen'
        ),

        # IK Control (sends pose goals to MoveIt, receives /joint_states)
        Node(
            package='ttt_control',
            executable='control_node',
            name='ttt_control_node',
            parameters=[{
                'update_rate_hz': 10.0,
                'planning_time_s': 0.05,
            }],
            output='screen'
        ),

        # Hardware Bridge (UDP to STM32 at 192.168.1.100)
        Node(
            package='ttt_hardware',
            executable='hardware_node',
            parameters=[{
                'stm_ip': '192.168.1.100',
                'stm_port': 5000,
                'joint_topic': '/joint_states',
            }],
            output='screen'
        ),
    ])
