#!/usr/bin/env bash
set -eo pipefail
if [[ $# -ne 1 || ! -d "$1" ]]; then
  echo "Usage: $0 classified-scan-directory" >&2
  exit 2
fi
root="$(cd "$(dirname "$0")/../../.." && pwd)"
ros_env=/home/dvt/micromamba/envs/wheel-leg-ros
unset PYTHONPATH
source "$ros_env/setup.bash"
export CONDA_PREFIX="$ros_env"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
config="$(mktemp --suffix=.rviz)"
bridge_pid=''
cleanup() {
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  rm -f "$config"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$ros_env/bin/python" "$root/technology/scan3d_fusion/viewer/classified_scan.py" "$1" --write-config "$config"
"$ros_env/bin/python" "$root/technology/scan3d_fusion/viewer/classified_scan.py" "$1" &
bridge_pid=$!
"$ros_env/bin/rviz2" -d "$config"
