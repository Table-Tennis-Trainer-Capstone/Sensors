from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mainarmass", package_name="ttt_control")
        .robot_description_kinematics()
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                # Allow wrist to start at hardware home (0°) even though URDF
                # lower limit is 2.09 rad. MoveIt clamps to the nearest valid
                # value instead of rejecting the start state entirely.
                "ompl.start_state_max_bounds_error": 3.15,
            },
        ],
    )

    return LaunchDescription([move_group_node])
