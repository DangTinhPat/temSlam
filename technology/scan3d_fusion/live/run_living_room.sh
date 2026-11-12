#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
exec 9>/tmp/rb-live-slam.lock
flock -n 9 || { echo 'Live SLAM đang mở. Hãy đóng phiên hiện tại trước.'; exit 1; }
export RB1_LIVE_SLAM=1 RB1_SIM_PORT=37685 RB1_SIM_IMU_UNIT=648 RB1_AUTO_SCAN=1
export RB1_SIM_MODEL="$root/simulation/scenarios/indoor_tiny_house_living_room.xml"
export RB1_SCAN_ROUTE="$root/technology/scan3d_fusion/mapping/native/living_room_scan_route.json"
export RB1_SIM_SPEED_FACTOR=5
export PYTHONPATH="${PYTHONPATH:-}"
exec /home/dvt/micromamba/envs/wheel-leg-ros/bin/python gui/sim_gui.py
