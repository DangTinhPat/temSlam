"""Check owned MuJoCo startup and sensor gates; never send a navigation goal."""
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/navigation'))
from runtime import NativeNavigation


def main():
    runtime = NativeNavigation((-.15, 0, 0))
    started = time.monotonic()
    result = dict(ready=False, scope='MuJoCo startup/observation only; no DRIVE goal')
    try:
        runtime.start()
        result['session'] = str(runtime.backend.map_session)
        print('Session:', result['session'], flush=True)
        while time.monotonic() - started < 65:
            pose, guard = runtime.observe(time.monotonic())
            if runtime.ready and pose and guard is not None:
                result.update(ready=True, pose=pose, guard=guard)
                break
            time.sleep(.04)
        else:
            result['message'] = runtime.message
    finally:
        runtime.close()
    (ROOT / 'evaluation/navigation_native_smoke.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    assert result['ready'], 'Startup/observation did not complete; inspect session logs'
    assert result['pose']['source'] == 'wheel_imu_short_run'
    assert isinstance(result['guard']['clear'], bool)


if __name__ == '__main__':
    main()
