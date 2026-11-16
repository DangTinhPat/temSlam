#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
ros_env="/home/dvt/micromamba/envs/wheel-leg-ros"
source "$ros_env/setup.bash"
export CONDA_PREFIX="$ros_env"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$root"
exec 9>/tmp/rb-rviz-navigation.lock
if ! flock -n 9; then
  echo "RViz navigation is already running. Use the existing window." >&2
  exit 1
fi
config="$(mktemp --suffix=.rviz)"
cp "${RB_NAV_RVIZ_CONFIG:-$root/technology/scan3d_fusion/navigation/navigation.rviz}" "$config"
bridge_pid="";rviz_pid=""
cleanup() {
  [[ -z "$rviz_pid" ]] || kill "$rviz_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || kill "$bridge_pid" 2>/dev/null || true
  [[ -z "$rviz_pid" ]] || wait "$rviz_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
  rm -f "$config"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
"$ros_env/bin/python" "$root/technology/scan3d_fusion/navigation/rviz_navigation.py" "$@" &
bridge_pid=$!
"$ros_env/bin/rviz2" -d "$config" &
rviz_pid=$!
wait -n "$bridge_pid" "$rviz_pid"
