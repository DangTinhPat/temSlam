"""Localize in a fixed RGB–ToF map, from an explicit approximate body pose.

Replay consumes localization_input.json; live mode observes an existing rgbd.npz
buffer. This process never builds a map or sends motor commands.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'technology/scan3d_fusion/live'), str(Path(__file__).parent)]
from workers import fusion_depth, write_json
from fixed_map import FixedReference, FixedMapLocalizer


def load_measurement(path):
    with np.load(path, allow_pickle=False) as data:
        batch = {key: data[key] for key in data.files}
    fusion_depth(batch)  # Keep the same fusion-only boundary as reconstruction.
    return batch


def live_status(record, received_at, now, stopped=False):
    result = dict(record or dict(state='initializing', usable=False, reason='waiting_for_rgbd'))
    age = max(0., now-received_at)
    if age > 2:
        result.update(state='lost', usable=False, reason='stale_rgbd')
    if stopped:
        result.update(state='stopped', usable=False, reason='worker_stopped')
    wall = time.time()
    result.update(wall_time=wall, input_age_s=age, valid_until_wall_time=wall+max(0., 2-age))
    return result


def prepare_output(reference, source, output, initial_pose, mode):
    reference, source, output = (Path(p).resolve() for p in (reference, source, output))
    if not np.isfinite(initial_pose).all() or np.asarray(initial_pose).shape != (3,):
        raise ValueError('Initial pose requires finite body x, y, yaw')
    if any(output == p or p in output.parents for p in (reference, source)):
        raise ValueError('Output must be outside the reference and input session')
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'run.json', dict(reference=str(reference), source=str(source), mode=mode,
        initial_body_pose_xyyaw_rad=list(initial_pose), started_wall_time=time.time(),
        pose_source='wheel/IMU + dynamic camera mount + fixed-map planar matching',
        limitation='Level floor; approximate initial pose required; no global recovery or navigation.'))
    write_json(output/'status.json', live_status(None, time.monotonic(), time.monotonic()))
    return reference, source, output


def process(localizer, batch, timestamp, packet, mount, intrinsics):
    begin = time.perf_counter()
    result = localizer.update(round(timestamp*1e9), packet, mount, batch['depth'], intrinsics)
    result.update(time_s=timestamp, processing_ms=(time.perf_counter()-begin)*1000)
    return result


def summarize(records):
    latency = [r['processing_ms'] for r in records]
    return dict(frames=len(records), accepted_frames=sum(bool(r['accepted']) for r in records),
        usable_frames=sum(bool(r['usable']) for r in records),
        usable_fraction=sum(bool(r['usable']) for r in records)/len(records) if records else 0.,
        states=dict(Counter(r['state'] for r in records)),
        rejection_reasons=dict(Counter(r['reason'] for r in records if not r['accepted'])),
        processing_ms_p50=float(np.median(latency)) if latency else None,
        processing_ms_p95=float(np.quantile(latency,.95)) if latency else None,
        limitation='Coverage and matching diagnostics, not absolute accuracy. Inference time excluded.')


def mark_error(output, exc):
    write_json(output/'status.json', dict(state='lost', usable=False, reason='worker_error',
        error=str(exc), wall_time=time.time(), valid_until_wall_time=time.time()))


def run_replay(reference, source, output, initial_pose):
    reference, source, output = prepare_output(reference, source, output, initial_pose, 'replay')
    records = []
    try:
        manifest = json.loads((source/'localization_input.json').read_text())
        if not manifest['frames']:
            raise ValueError('Replay has no measurements')
        localizer = FixedMapLocalizer(FixedReference.from_session(reference), initial_pose=initial_pose)
        with (output/'poses.jsonl').open('x') as stream:
            for frame in manifest['frames']:
                relative = Path(frame['file'])
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError('Replay measurement must be a relative session path')
                batch = load_measurement(source/relative)
                # Mapping poses and evaluator truth are deliberately not consumed.
                result = process(localizer, batch, frame['time_s'], frame['odometry'],
                                 frame['body_from_camera'], manifest['intrinsics'])
                stream.write(json.dumps(result, allow_nan=False)+'\n'); stream.flush()
                records.append(result)
                now = time.monotonic()
                write_json(output/'status.json', live_status(result, now, now))
        summary = summarize(records)
        write_json(output/'summary.json', summary)
        now = time.monotonic()
        write_json(output/'status.json', live_status(records[-1], now, now, stopped=True))
        return summary
    except Exception as exc:
        mark_error(output, exc)
        raise


def run_live(reference, source, output, initial_pose):
    reference, source, output = prepare_output(reference, source, output, initial_pose, 'live')
    records = []; last = None; received = time.monotonic(); last_report = 0.; stamp = -1.
    try:
        localizer = FixedMapLocalizer(FixedReference.from_session(reference), initial_pose=initial_pose)
        with (output/'poses.jsonl').open('x') as stream:
            while not (source/'STOP').exists():
                now = time.monotonic()
                try:
                    path = source/'rgbd.npz'
                    # An old buffer at process startup is not a fresh observation.
                    file_age = max(0., time.time()-path.stat().st_mtime)
                    if file_age <= 2:
                        batch = load_measurement(path)
                        t = float(batch['t'])
                        if not np.isfinite(t) or t < stamp:
                            raise ValueError('Sensor timestamp reset; start a new localization session')
                        if t > stamp:
                            packet = json.loads(str(batch['odometry']))
                            last = process(localizer, batch, t, packet, batch['mount'], batch['K'])
                            stamp = t; received = now-file_age
                            stream.write(json.dumps(last, allow_nan=False)+'\n'); stream.flush()
                            records.append(last)
                except FileNotFoundError:
                    pass
                if now-last_report >= .1:
                    write_json(output/'status.json', live_status(last, received, time.monotonic()))
                    last_report = now
                time.sleep(.02)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        mark_error(output, exc)
        raise
    summary = summarize(records)
    write_json(output/'summary.json', summary)
    write_json(output/'status.json', live_status(last, received, time.monotonic(), stopped=True))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path, dest='reference')
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--replay', type=Path)
    inputs.add_argument('--live', type=Path)
    parser.add_argument('--initial-pose', nargs=3, type=float, required=True, metavar=('X_M','Y_M','YAW_DEG'))
    parser.add_argument('--output', required=True, type=Path, help='New directory outside map and input session')
    args = parser.parse_args()
    x, y, yaw_deg = args.initial_pose
    run = run_replay if args.replay else run_live
    result = run(args.reference, args.replay or args.live, args.output, (x,y,np.deg2rad(yaw_deg)))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
