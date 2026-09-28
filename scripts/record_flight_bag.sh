#!/usr/bin/env bash
# ==============================================================================
# record_flight_bag.sh: ROS 2 Bag Recorder for Flight Data & System Benchmarking
# ==============================================================================
set -e

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="uav_flight_log_${TIMESTAMP}"

echo "Starting ROS 2 bag recording: ${BAG_NAME}..."
ros2 bag record \
    -o "${BAG_NAME}" \
    /mavros/setpoint_position/local \
    /mavros/local_position/pose \
    /mavros/battery \
    /slam/pose \
    /slam/odometry \
    /orchestrator/drone_state \
    /ai/semantic_goal \
    /safety/failsafe_trigger \
    /tf \
    /tf_static
