#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
ros_env=/home/dvt/micromamba/envs/wheel-leg-ros
source "$ros_env/setup.bash"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=44
python "$root/technology/scan3d_fusion/live/rviz.py" "$1" &
bridge=$!
trap 'kill "$bridge" 2>/dev/null || true; wait "$bridge" 2>/dev/null || true' EXIT
rviz2 -d "$root/technology/scan3d_fusion/live/live.rviz"
