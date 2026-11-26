"""Evaluate finalized camera poses against simulation capture timestamps."""
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation


def reference_cameras(session, frames):
    # The demo rig has a fixed camera. The saved mount is body_from_camera.
    with np.load(session / 'rgbd.npz', allow_pickle=False) as latest:
        mount = latest['mount']
    targets = [float(frame['time_s']) for frame in frames]
    nearest = [None] * len(targets)
    gaps = [math.inf] * len(targets)
    with (session / 'live_capture.jsonl').open() as stream:
        for line in stream:
            capture = json.loads(line)
            timestamp = float(capture['t'])
            for i, target in enumerate(targets):
                gap = abs(timestamp - target)
                if gap < gaps[i]:
                    nearest[i], gaps[i] = capture, gap
    if max(gaps) > .02:
        raise ValueError('Thiếu capture tham chiếu đồng bộ')
    result = []
    for capture in nearest:
        body = np.eye(4)
        body[:3, :3] = Rotation.from_quat(
            [float(capture[k]) for k in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
        body[:3, 3] = [float(capture[k]) for k in ('x', 'y', 'z')]
        result.append(body @ mount)
    return result


def relative_motion_error(reference, estimated):
    relative_reference = np.linalg.solve(*reference)
    relative_estimate = np.linalg.solve(*estimated)
    residual = np.linalg.solve(relative_reference, relative_estimate)
    return (float(np.linalg.norm(residual[:3, 3])),
            math.degrees(Rotation.from_matrix(residual[:3, :3]).magnitude()))


def loop_closure_check(route, session, *, require_return_edge=False):
    points = route.get('waypoints', [])
    if len(points) < 2 or math.hypot(points[-1]['x']-points[0]['x'],
                                     points[-1]['y']-points[0]['y']) > .1:
        return {'passed': False, 'reason': 'Tuyến chưa khép vòng'}
    try:
        quality = json.loads((session / 'pose_quality.json').read_text())
        frames = json.loads((session / 'live_sequence.json').read_text())['frames']
        endpoints = [frames[0], frames[-1]]
        reference = reference_cameras(session, endpoints)
        estimated = [np.asarray(frame['camera_to_world']) for frame in endpoints]
        translation, rotation = relative_motion_error(reference, estimated)
        pgo = int(quality.get('pgo_events', 0))
        closures = int(quality.get('loop_closure_events', 0))
    except (OSError, ValueError, KeyError, IndexError, TypeError, np.linalg.LinAlgError):
        return {'passed': False, 'reason': 'Thiếu pose cuối hoặc capture tham chiếu hợp lệ'}
    passed = closures > 0 and pgo > 0 and translation <= .1 and rotation <= 5.
    reason = 'Đạt kiểm tra đầu–cuối' if passed else 'Thiếu sự kiện SDK hoặc sai số điểm cuối vượt ngưỡng'
    verified = None
    if require_return_edge:
        from graph_capture import assess_return_edges
        try:
            evidence = assess_return_edges(session)
            arrival = evidence.get('return_ns', [])
            last_ts = round(frames[-1]['time_s'] * 1e9)
            verified = bool(evidence.get('verified_in_final_graph') and len(arrival) == 2 and arrival[0] <= last_ts <= arrival[1]
                            and not quality.get('tracking_interrupted', False))
        except (OSError, ValueError, KeyError, IndexError, TypeError):
            verified = False
        passed = passed and verified
        if not verified:
            reason = 'Chưa xác minh cạnh SDK nối lượt đi với lượt về trên quỹ đạo cuối'
        elif passed:
            reason = 'Đạt sai số đầu–cuối và xác minh được cạnh đóng vòng'
    return {'passed': passed, 'return_edge_verified': verified, 'pgo_events': pgo, 'loop_closure_events': closures,
            'endpoint_translation_error_m': translation, 'endpoint_rotation_error_deg': rotation,
            'reason': reason}
