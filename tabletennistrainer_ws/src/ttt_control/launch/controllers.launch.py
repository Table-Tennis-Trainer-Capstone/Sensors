from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mainarmass", package_name="ttt_control")
        .to_moveit_configs()
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("ttt_control"), "config", "ros2_controllers.yaml"
    )

    return LaunchDescription([
        # Controller manager with mock hardware interface
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                controllers_yaml,
            ],
            output="screen",
        ),

        # Publishes /joint_states from controller feedback
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        # Receives MoveIt trajectory goals and drives mock joints
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["armgroup_controller", "--controller-manager", "/controller_manager"],
        ),
    ])
