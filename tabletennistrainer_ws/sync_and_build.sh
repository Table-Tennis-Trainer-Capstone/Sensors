#!/bin/bash

# Target Configuration
JETSON_A_IP="192.168.1.10"
JETSON_A_USER="capstone-nano1"
LOCAL_WORKSPACE_DIR="/home/capstone-nano2/Sensors/tabletennistrainer_ws/"
REMOTE_WORKSPACE_DIR="/home/capstone-nano1/Sensors/tabletennistrainer_ws/"

echo "Building locally on Jetson B..."
cd $LOCAL_WORKSPACE_DIR
colcon build --symlink-install

echo "Syncing workspace to Jetson A ($JETSON_A_IP)..."
# Ensure the remote directory exists before syncing
ssh $JETSON_A_USER@$JETSON_A_IP "mkdir -p $REMOTE_WORKSPACE_DIR"

# Rsync the workspace, explicitly ignoring local build artifacts
rsync -avz --delete \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude '.git/' \
    $LOCAL_WORKSPACE_DIR $JETSON_A_USER@$JETSON_A_IP:$REMOTE_WORKSPACE_DIR

echo " Building remotely on Jetson A..."
ssh $JETSON_A_USER@$JETSON_A_IP "cd $REMOTE_WORKSPACE_DIR && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

echo "Sync and build complete on both Jetsons!"