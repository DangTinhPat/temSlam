#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
ros_env="/home/dvt/micromamba/envs/wheel-leg-ros"
source "$ros_env/setup.bash"
export CONDA_PREFIX="$ros_env"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ $# != 1 || ! -f "$1" ]]; then
  echo "Usage: $0 MESH.stl" >&2
  exit 2
fi
mesh="$1"
config="$(mktemp --suffix=.rviz)"
python - "$root/simulation/demo/viewer/clay.rviz" "$mesh" "$config" <<'PY'
import json, sys
from pathlib import Path
import yaml
config = yaml.safe_load(Path(sys.argv[1]).read_text())
info = Path(sys.argv[2]).with_suffix('.json')
if info.is_file():
    bounds = json.loads(info.read_text()).get('bounds_m')
    if bounds:
        view = config['Visualization Manager']['Views']['Current']
        view['Focal Point'] = dict(zip('XYZ', [(a+b)/2 for a, b in zip(*bounds)]))
        view['Distance'] = max(.3, 2*sum((b-a)**2 for a, b in zip(*bounds))**.5)
        view['Name'] = 'Reconstructed surface'
Path(sys.argv[3]).write_text(yaml.safe_dump(config))
PY
python "$root/simulation/demo/viewer/rviz_bridge.py" "$mesh" &
bridge_pid=$!
cleanup() { kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; rm -f "$config"; }
trap cleanup EXIT INT TERM
rviz2 -d "$config"
