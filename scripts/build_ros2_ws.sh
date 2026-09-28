#!/usr/bin/env bash
# ==============================================================================
# build_ros2_ws.sh: Clean Colcon Build Script for Autonomous UAV ROS 2 Workspace
# Author: Member 3 (ROS 2 & System Integration Engineer)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/../drone_ws"

echo "================================================================================"
echo "  BUILDING AUTONOMOUS UAV ROS 2 WORKSPACE (Jazzy / Humble / Iron)"
echo "================================================================================"

cd "${WS_DIR}"

echo " -> Sourcing ROS 2 base environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: Standard ROS 2 install not found in /opt/ros/. Ensure ROS 2 is sourced."
fi

echo " -> Cleaning previous build cache..."
# rm -rf build install log

echo " -> Running Colcon build with symlink install & parallel compilation..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --parallel-workers $(nproc)

echo " -> Sourcing workspace overlay..."
source "${WS_DIR}/install/setup.bash"

echo "================================================================================"
echo "  BUILD COMPLETE! All packages (interfaces, orchestrator, mavros, cpp) ready."
echo "================================================================================"
