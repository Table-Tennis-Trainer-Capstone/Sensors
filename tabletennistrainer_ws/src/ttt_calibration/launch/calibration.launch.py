from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = get_package_share_directory('ttt_calibration')
    
    return LaunchDescription([
        # TF Broadcaster - publishes camera and robot transforms
        Node(
            package='ttt_calibration',
            executable='tf_broadcaster_node',
            name='tf_broadcaster',
            parameters=[
                os.path.join(config_dir, 'config', 'stereo_extrinsic.yaml')
            ],
            output='screen'
        )
    ])
