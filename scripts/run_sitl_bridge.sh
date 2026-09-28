#!/usr/bin/env bash
# ==============================================================================
# run_sitl_bridge.sh: Launches ArduPilot SITL / Gazebo Harmonic MAVROS Connection
# ==============================================================================
set -e

FCU_URL=${1:-"udp://127.0.0.1:14550@14555"}
GCS_URL=${2:-"udp://@127.0.0.1:14550"}

echo "Connecting MAVROS to Autopilot SITL via: ${FCU_URL}..."
ros2 launch mavros apm.launch \
    fcu_url:="${FCU_URL}" \
    gcs_url:="${GCS_URL}" \
    tgt_system:=1 \
    tgt_component:=1
