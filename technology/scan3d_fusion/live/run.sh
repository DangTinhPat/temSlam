#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
exec 9>/tmp/rb-live-slam.lock
flock -n 9 || { echo 'Live SLAM đang mở. Hãy dùng cửa sổ hiện có.'; exit 1; }
export RB1_LIVE_SLAM=1 RB1_SIM_PORT=37685 RB1_SIM_IMU_UNIT=648
export RB1_SIM_MODEL="$root/simulation/scenarios/indoor_tiny_house.xml"
export PYTHONPATH="${PYTHONPATH:-}"
exec /home/dvt/micromamba/envs/wheel-leg-ros/bin/python gui/sim_gui.py
