#!/bin/bash
# Launch MoveIt2 + RViz2 to visualize the MARTY robot arm.
# Usage:  ./run_moveit_demo.sh
# Needs micromamba marty_env (or ttt_env) activated first.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
WS="$SCRIPT_DIR"

# -- 0. Pre-Flight Cleanup & Build ---------------------------------------------
echo "[0/5] Cleaning up old processes & Rebuilding workspace..."

# Force-kill any lingering background ROS processes to free up ports
killall -9 rviz2 robot_state_publisher ros2_control_node python3 2>/dev/null || true

# Build the control package
colcon build --symlink-install --packages-select ttt_control
if [ $? -ne 0 ]; then
    echo "Build failed! Aborting launch to prevent running broken code."
    exit 1
fi

# Source your reliable custom environment script
if [ -f "$WS/ros2_env.sh" ]; then
    source "$WS/ros2_env.sh"
else
    echo "ros2_env.sh not found. Proceeding with default sourcing."
    if [ -f "$WS/install/local_setup.bash" ]; then
        source "$WS/install/local_setup.bash" 2>/dev/null || true
    fi
fi

# -- Environment ---------------------------------------------------------------
CONDA_PFX="${CONDA_PREFIX:-$HOME/micromamba/envs/marty_env}"

# Add workspace packages to AMENT_PREFIX_PATH (skips broken Jetson setup.bash)
for PKG in ttt_msgs ttt_control; do
    D="$WS/install/$PKG"
    if [ -d "$D" ]; then
        export AMENT_PREFIX_PATH="$D:${AMENT_PREFIX_PATH:-}"
    fi
done

export ROS_DOMAIN_ID=43
export DYLD_LIBRARY_PATH="$CONDA_PFX/lib:${DYLD_LIBRARY_PATH:-}"

unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1

echo "AMENT_PREFIX_PATH = $AMENT_PREFIX_PATH"
echo "ROS_DOMAIN_ID     = $ROS_DOMAIN_ID"

# -- Cleanup on exit -----------------------------------------------------------
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null
    sleep 1
}
trap cleanup EXIT INT TERM

# -- Launch sequence -----------------------------------------------------------
echo "[1/5] robot_state_publisher..."
ros2 launch ttt_control rsp.launch.py &
sleep 3

echo "[2/5] ros2_control fake hardware + controllers..."
ros2 launch ttt_control controllers.launch.py &
sleep 3

echo "[3/5] MoveIt move_group (takes ~8s)..."
ros2 launch ttt_control move_group.launch.py &
sleep 9

echo "[4/5] Control Node..."
ros2 run ttt_control control_node &
sleep 2

echo "[5/5] RViz2 via MoveIt launch so kinematics params are loaded..."
RVIZ_CFG="$SCRIPT_DIR/ttt_demo.rviz"
if [ -f "$RVIZ_CFG" ]; then
    ros2 launch ttt_control moveit_rviz.launch.py rviz_config:="$RVIZ_CFG"
else
    echo "WARNING: ttt_demo.rviz not found, opening blank RViz"
    ros2 launch ttt_control moveit_rviz.launch.py
fi