"""Owned native simulation startup/inference check; no drive goal is sent."""
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/navigation'))
from runtime import NativeNavigation
from ground_segmentation import read_labels, SOURCE


def main():
    runtime = NativeNavigation((-.15, 0, 0))
    frames = []
    last = -1
    started = time.monotonic()
    try:
        runtime.start()
        session = runtime.backend.map_session
        print(str(session), flush=True)
        while time.monotonic()-started<80:
            runtime.observe(time.monotonic())
            try:
                with np.load(session/'rgbd.npz', allow_pickle=False) as sample:
                    stamp = float(sample['t'])
                    if stamp>last:
                        labels = read_labels(sample, sample['depth'].shape)
                        assert labels is not None
                        assert str(sample['depth_source'])=='deltar_tof_fusion'
                        assert sample['depth'].dtype==np.uint16
                        frames.append(dict(t=stamp, counts=np.bincount(labels.ravel(), minlength=3).tolist(),
                                           confirmations=int(sample['ground_confirmations'])))
                        last = stamp
            except FileNotFoundError:
                pass
            if len(frames)>=8:
                break
            time.sleep(.04)
    finally:
        runtime.close()
    report = dict(session=str(session), scope='Native balance + inference only; no navigation goal',
                  source=SOURCE, frames=frames, seconds=time.monotonic()-started)
    (ROOT/'evaluation/ground_live_smoke.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    assert len(frames)>=8, 'Live labelled inference did not produce eight fresh frames'
    assert any(frame['counts'][1]>0 and frame['confirmations']==3 for frame in frames), 'Live floor was never confirmed'


if __name__=='__main__':
    main()
