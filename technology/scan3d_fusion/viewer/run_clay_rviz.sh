#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
ros_env="/home/dvt/micromamba/envs/wheel-leg-ros"
source "$ros_env/setup.bash"
export CONDA_PREFIX="$ros_env"
export LD_LIBRARY_PATH="$ros_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ $# -gt 0 ]]; then
  mesh="$1"
else
  mesh="$(find "$root/datasets/processed/sofa_clay" -mindepth 3 -maxdepth 3 \
    -path '*/mesh/clay.stl' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
if [[ -z "$mesh" || ! -f "$mesh" ]]; then
  echo "Usage: $0 path/to/mesh/clay.stl [--compare-sofa]" >&2
  exit 2
fi
compare="${2:-}"
if [[ -n "$compare" && "$compare" != "--compare-sofa" && "$compare" != "--floor-completion" ]]; then
  echo "Usage: $0 [mesh.stl] [--compare-sofa | --floor-completion]" >&2
  exit 2
fi
config="$(mktemp --suffix=.rviz)"
python - "$root/technology/scan3d_fusion/viewer/clay.rviz" "$mesh" "$config" "$compare" <<'PY'
import json, sys
from pathlib import Path
import xml.etree.ElementTree as ET
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
if sys.argv[4] == '--compare-sofa':
    scene = Path(sys.argv[1]).resolve().parents[3] / 'simulation/scenarios/indoor_tiny_house.xml'
    sofa = next(g for g in ET.parse(scene).getroot().findall('worldbody/geom')
                if g.get('name') == 'env_mesh_sofa')
    x, y, _ = map(float, sofa.get('pos').split())
    view = config['Visualization Manager']['Views']['Current']
    view['Focal Point'] = {'X': x+1.3, 'Y': y, 'Z': .65}
    view['Distance'] = 6.5
    view['Yaw'] = -1.57079632679
    view['Name'] = 'Scan and original sofa, same scale'
    config['Window Geometry']['Width'] = 1600
if sys.argv[4] == '--floor-completion':
    layers = json.loads((Path(sys.argv[2]).parent/'floor_completion/manifest.json').read_text())['layers']
    displays = config['Visualization Manager']['Displays']
    displays[0]['Enabled'] = False
    displays[0]['Name'] = 'Mesh goc - doi chieu'
    for name, title in [('objects','Vat the va phan mesh con lai'),
                        ('floor','San lien mach - gom ca phan suy ra'),
                        ('observed','San da quan sat'),('inferred','San suy ra - mau xanh')]:
        if name not in layers:
            continue
        enabled = name in ('objects','floor') or 'floor' not in layers
        displays.append({'Class':'rviz_default_plugins/Marker', 'Name':title, 'Enabled':enabled,
                         'Topic':{'Value':'/scan3d/'+name,'Reliability Policy':'Reliable',
                                  'Durability Policy':'Transient Local'}})
    view = config['Visualization Manager']['Views']['Current']
    view['Distance'] *= .65
    view['Pitch'] = 1.0
    view['Name'] = 'Observed and inferred floor'
Path(sys.argv[3]).write_text(yaml.safe_dump(config))
PY
bridge_args=("$mesh")
[[ -z "$compare" ]] || bridge_args+=("$compare")
python "$root/technology/scan3d_fusion/viewer/rviz_bridge.py" "${bridge_args[@]}" &
bridge_pid=$!
cleanup() { kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; rm -f "$config"; }
trap cleanup EXIT INT TERM
rviz2 -d "$config"
