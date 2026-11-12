#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../../../" && pwd)"
ros_env="/home/dvt/micromamba/envs/wheel-leg-ros"
source "$ros_env/setup.bash"
export CONDA_PREFIX="$ros_env"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ $# -gt 0 ]]; then
  session="$1"
else
  session="$(find "$root/datasets/raw" -mindepth 1 -maxdepth 1 -type d -name 'tof-*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
fi
config="$root/technology/scan3d_fusion/mapping/native/rviz_native.rviz"
if [[ "${2:-}" == "--house-only" ]]; then
  config="$root/technology/scan3d_fusion/mapping/native/rviz_house.rviz"
elif [[ $# -gt 1 ]]; then
  echo "Usage: $0 [datasets/raw/tof-SESSION] [--house-only]" >&2
  exit 2
fi
if [[ -z "$session" || ! -f "$session/state.json" ]]; then
  echo "Usage: $0 [datasets/raw/tof-SESSION]" >&2
  exit 2
fi
[[ "$session" = /* ]] || session="$root/$session"
python "$root/technology/scan3d_fusion/mapping/native/rviz_bridge.py" "$session" --scene "$root/simulation/scenarios/indoor_tiny_house.xml" &
bridge_pid=$!
cleanup() { kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
rviz2 -d "$config"
