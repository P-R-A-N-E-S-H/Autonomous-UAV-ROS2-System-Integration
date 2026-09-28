#!/usr/bin/env bash
# ==============================================================================
# launch_ardupilot_sitl.sh: Launch ArduPilot Software-In-The-Loop Simulation
# Author: Member 2 (Simulation & Flight Stack Engineer)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM_FILE="${SCRIPT_DIR}/../params/ardupilot_sitl_quad.parm"

echo "================================================================================"
echo "  STARTING ARDUPILOT 4.5+ SITL (X-QUADROTOR FOR GAZEBO HARMONIC)"
echo "================================================================================"

sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --add-param-file="${PARAM_FILE}" \
    --console \
    --map \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14555
