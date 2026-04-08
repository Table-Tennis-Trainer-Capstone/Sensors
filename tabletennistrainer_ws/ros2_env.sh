#!/bin/zsh
# Source this in any terminal to connect to the running ROS2 stack.
# Usage: source ./ros2_env.sh

WS="$(cd "$(dirname "$0")" && pwd)"

if [ -f "$WS/install/local_setup.zsh" ]; then
    source "$WS/install/local_setup.zsh"
fi

export ROS_DOMAIN_ID=43
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1

echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY = 1 (Local loopback only)"
echo "AMENT_PREFIX_PATH = $AMENT_PREFIX_PATH"
