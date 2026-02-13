#!/bin/bash
set -e

if [ "$1" == "clean" ]; then
    echo "Cleaning workspace..."
    rm -rf build/ install/ log/
fi

echo "Building Table Tennis Trainer Workspace..."

colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ -f build/compile_commands.json ]; then
    ln -sf $(pwd)/build/compile_commands.json $(pwd)/compile_commands.json
    echo "LSP database updated."
fi

echo "Build Complete. Run 'source install/setup.bash' to update your environment."