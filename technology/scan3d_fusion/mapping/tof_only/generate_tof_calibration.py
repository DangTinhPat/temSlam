"""Generate shared native calibration from the vision configuration."""
import pathlib
import sys
import yaml
root = pathlib.Path(__file__).resolve().parents[4]
c = yaml.safe_load((root / 'technology/scan3d_fusion/config/tof.yaml').read_text())['sensor']
p = c['position_body_m']; axes = c['xyaxes_body']; n = c['noise']
out = pathlib.Path(sys.argv[1]); out.parent.mkdir(parents=True, exist_ok=True)
out.write_text('#pragma once\nnamespace tof {\n' +
    'inline constexpr double position[3] = {' + ','.join(map(str,p)) + '};\n' +
    'inline constexpr double axes[6] = {' + ','.join(map(str,axes)) + '};\n' +
    '\n'.join(f'inline constexpr double {key} = {value};' for key,value in dict(
        window_offset=c['board_size_m'][0]/2+c['window_size_m'][0],
        fov=c['fovy_deg'], period=1/c['frame_rate_hz'], minimum=c['min_range_m'],
        maximum=c['max_range_m'], near_noise=n['near_accuracy_m'],
        far_noise=n['far_accuracy_fraction'], corner_noise=n['corner_extra_fraction'],
        sigma=n['sigma_fraction_of_bound']).items()) + '\n}\n')
