import sys, os; sys.path.insert(0, os.path.dirname(__file__)); from calibration import PARAMS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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

        # Bridge table frame to robot_base (robot is mounted 1.4732m behind net, level with table)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='table_to_robot_base',
            arguments=['0', '0', '-1.4732', '0', '0', '0', 'table', 'robot_base'],
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
                'camera_id': 'left',
                'width': PARAMS['width'],
                'height': PARAMS['height'],
                'fps': PARAMS['fps'],
                'exposure': PARAMS['exposure'],
                'analogue_gain': PARAMS['analogue_gain'],
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
                'min_area': PARAMS['min_area'],
                'max_area': PARAMS['max_area'],
                'min_contrast': PARAMS['min_contrast'],
                'dilate_iters': PARAMS['dilate_iters'],
                'edge_margin': PARAMS['edge_margin'],
                'kf_gate_px': PARAMS.get('kf_gate_px', 250.0),
                'kf_process_noise': PARAMS.get('kf_process_noise', 0.05),
                **({'table_roi': PARAMS['table_roi_left']} if PARAMS['table_roi_left'] else {}),
            }],
            output='screen'
        ),

        # Stereo Vision (receives right detection from Jetson B via DDS)
        Node(
            package='ttt_stereo',
            executable='stereo_node',
            parameters=[{
                'fx': PARAMS['fx'],
                'fy': PARAMS['fy'],
                'cx': PARAMS['cx'],
                'cy': PARAMS['cy'],
                'baseline_m': PARAMS['baseline_m'],
                'max_sync_age_ms': PARAMS['max_sync_age_ms'],
                'limit_x_m': PARAMS.get('limit_x_m', 1.5),
                'limit_y_top_m': PARAMS.get('limit_y_top_m', 2.0),
                'limit_y_bottom_m': PARAMS.get('limit_y_bottom_m', -0.2),
                'limit_z_m': PARAMS.get('limit_z_m', 2.5),
            }],
        ),

        # Trajectory Prediction
        Node(
            package='ttt_trajectory',
            executable='trajectory_node',
            name='trajectory_node',
            parameters=[{
                'lookahead_ms': PARAMS['lookahead_ms'],
            'stage1_min_samples': PARAMS.get('stage1_min_samples', 3),
            'stage2_min_samples': PARAMS.get('stage2_min_samples', 8),
                'max_samples': PARAMS['max_samples'],
                'gravity': PARAMS['gravity'],
                'camera_tilt_deg': PARAMS['camera_tilt_deg'],
                'table_y': PARAMS['table_y'],
                'restitution': PARAMS['restitution'],
                'net_z': PARAMS.get('net_z', 0.0),
                'min_incoming_speed': PARAMS.get('min_incoming_speed', 0.5),
                'net_margin_z': PARAMS.get('net_margin_z', -0.2),
                'max_track_z': PARAMS.get('max_track_z', 1.15),
                'max_velocity': PARAMS.get('max_velocity', 25.0),
            }],
            output='screen'
        ),

        # IK Control (sends pose goals to MoveIt, receives /joint_states)
        Node(
            package='ttt_control',
            executable='control_node',
            name='ttt_control_node',
            parameters=[{
                'update_rate_hz':  PARAMS['update_rate_hz'],
                'planning_time_s': PARAMS['planning_time_s'],
                'return_delay_ms': PARAMS['return_delay_ms'],
                'speed_multiplier': PARAMS.get('speed_multiplier', 4.0),
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
