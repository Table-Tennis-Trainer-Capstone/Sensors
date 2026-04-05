from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mainarmass", package_name="ttt_control")
        .planning_pipelines(
            # OMPL handles position-only Cartesian goals (ball tracking).
            # Pilz is kept available so named-state joint goals remain fast.
            # Default is OMPL — control_node switches to Pilz only for setNamedTarget.
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
