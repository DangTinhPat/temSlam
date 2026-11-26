"""Rebuild bmap1 A/B meshes from one recorded session and score common observations."""
import argparse
from itertools import product
import json
from pathlib import Path

import numpy as np
from scipy.stats import theilslopes
from scipy.spatial.transform import Rotation

from pose_check import reference_cameras, relative_motion_error
from worker_common import write_json
from vslam.localization import floor_camera_pose


# Inner faces of the bmap1 middle room, in the MuJoCo evaluation frame.
WALLS = {'south': (1, -2.5), 'north': (1, 2.5),
         'left': (0, -2.46), 'right': (0, 2.46)}
WALL_PAIRS = [('north', 'south'), ('left', 'right')]


def common_runs(stamps, candidate, epochs, approved):
    """Indices into the SDK snapshot, breaking at every missing timestamp or epoch."""
    if (np.shape(candidate) != (len(stamps), 4, 4) or len(epochs) != len(stamps)
            or np.any(np.diff(stamps) <= 0)):
        raise ValueError('Invalid final graph snapshot')
    approved = set(approved)
    runs = []
    for i, stamp in enumerate(stamps):
        if stamp not in approved or not np.isfinite(candidate[i]).all():
            continue
        if not runs or runs[-1][-1] != i - 1 or epochs[i] != epochs[i - 1]:
            runs.append([])
        runs[-1].append(i)
    return runs


def shared_bounds(bounds, approved, candidate, voxel):
    """Enclose the live volume and its image under every A-to-B pose correction."""
    bounds = np.asarray(bounds)
    corners = np.array(list(product(*bounds.T)))
    deltas = candidate @ np.linalg.inv(approved)
    mapped = corners @ deltas[:, :3, :3].transpose(0, 2, 1) + deltas[:, None, :3, 3]
    padding = np.array([bounds[0] - mapped.min(axis=(0, 1)), mapped.max(axis=(0, 1)) - bounds[1]])
    steps = np.maximum(0, np.ceil(padding / voxel - 1e-7))
    return bounds + np.array([[-1], [1]]) * steps * voxel


def revisit_pair(bodies):
    xy = bodies[:, :2, 3]
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    yaw = np.arctan2(bodies[:, 1, 0], bodies[:, 0, 0])
    best = None
    for j in range(1, len(xy)):
        gaps = np.linalg.norm(xy[:j] - xy[j], axis=1)
        eligible = (gaps <= .4) & (distance[j] - distance[:j] > max(5., .65 * distance[-1]))
        # Use an independent pair of observations for the revisit score.
        if j == len(xy) - 1:
            eligible[0] = False
        for i in np.flatnonzero(eligible):
            dyaw = yaw[j] - yaw[i]
            angle = abs(np.arctan2(np.sin(dyaw), np.cos(dyaw)))
            rank = (angle >= np.deg2rad(15), gaps[i], int(i), j)
            if best is None or rank < best:
                best = rank
    return list(best[2:]) if best is not None else None


def wall_strips(mesh):
    xyz, normals = mesh.triangles_center, mesh.face_normals
    result = {}
    for wall, (axis, plane) in WALLS.items():
        along = 1 - axis
        mask = ((abs(xyz[:, axis] - plane) < .85) & (abs(xyz[:, along]) < 2.1)
                & (xyz[:, 2] > .12) & (xyz[:, 2] < .80) & (abs(normals[:, axis]) > .70))
        u, v = xyz[mask, along], xyz[mask, axis]
        bins = np.floor((u + 2.1) / .15).astype(int)
        result[wall] = {int(i): (np.median(u[bins == i]), np.median(v[bins == i]))
                        for i in np.unique(bins) if np.sum(bins == i) >= 3}
    return result


def wall_fit(strips, bins, axis):
    bins = sorted(bins)
    result = dict(bin_ids=bins, span_m=0., axis_angle_deg=None, median_fit_residual_m=None)
    if bins:
        u, v = np.array([strips[i] for i in bins]).T
        result['span_m'] = float(np.ptp(u))
    if len(bins) >= 4:
        slope, intercept, _, _ = theilslopes(v, u)
        result.update(axis_angle_deg=float(np.rad2deg(np.arctan(slope))) * (1 if axis == 1 else -1),
                      median_fit_residual_m=float(np.median(abs(v - intercept - slope * u))))
    return result


def wall_metrics(strips, support):
    fits = {name: wall_fit(strips[name], support[name], axis) for name, (axis, _) in WALLS.items()}
    angles = {}
    for a, b in WALL_PAIRS:
        left, right = fits[a]['axis_angle_deg'], fits[b]['axis_angle_deg']
        angles[a + '_' + b] = (None if left is None or right is None
                               else abs((left - right + 90) % 180 - 90))
    return dict(walls=fits, pair_angle_deg=angles)


def bmap1_wall_check(strips):
    # Number of shared 15 cm bins on the two wall pairs in the reference scan.
    pairs = {}
    support = {}
    for a, b in WALL_PAIRS:
        support[a] = support[b] = strips[a].keys() & strips[b].keys()
    angles = wall_metrics(strips, support)['pair_angle_deg']
    for (a, b), minimum in zip(WALL_PAIRS, (15, 24)):
        name = a + '_' + b
        angle = angles[name]
        count = len(support[a])
        pairs[name] = dict(angle_deg=angle, shared_bins=count, required_bins=minimum,
                           passed=count >= minimum and angle is not None and angle <= 2.)
    return dict(passed=all(pair['passed'] for pair in pairs.values()), limit_deg=2., pairs=pairs)


def final_wall_strips(session, world_from_map):
    import trimesh
    direct = trimesh.load(session / 'final_mesh.ply', force='mesh')
    direct.apply_transform(world_from_map)
    return wall_strips(direct)


def check_final_walls(session):
    first = json.loads((session / 'live_sequence.json').read_text())['frames'][0]
    reference = reference_cameras(session, [first])[0]
    world_from_map = reference @ np.linalg.inv(first['visual_camera_to_world'])
    return bmap1_wall_check(final_wall_strips(session, world_from_map))


def pair_scores(pair, frames, truth, poses):
    if pair is None:
        return None
    reference = truth[pair]
    relative_reference = np.linalg.solve(*reference)
    result = dict(timestamp_ns=[frames[i]['timestamp_ns'] for i in pair],
                  reference_translation_m=float(np.linalg.norm(relative_reference[:3, 3])),
                  reference_rotation_deg=float(np.rad2deg(Rotation.from_matrix(relative_reference[:3, :3]).magnitude())))
    for name, variant in poses.items():
        translation, rotation = relative_motion_error(reference, variant[pair])
        result[name] = dict(translation_error_m=translation, rotation_error_deg=rotation)
    return result


def compare(session):
    from worker_reconstruction import build_surface
    import torch
    torch.set_num_threads(2)

    seq = json.loads((session / 'live_sequence.json').read_text())
    if seq.get('pose_stage') != 'final':
        raise ValueError('A final live_sequence.json is required')
    with np.load(session / 'graph_snapshots/final.npz', allow_pickle=False) as snapshot:
        stamps, candidate, epochs = [snapshot[k] for k in ('timestamp_ns', 'candidate', 'tracker_epoch')]
    approved = seq['frames']
    by_stamp = {f['timestamp_ns']: i for i, f in enumerate(approved)}
    if len(by_stamp) != len(approved) or not by_stamp.keys() <= set(stamps):
        raise ValueError('Approved timestamps cannot be joined to the final graph snapshot')
    runs = common_runs(stamps, candidate, epochs, by_stamp)
    headless = json.loads((session / 'headless_result.json').read_text())
    whole_route = (headless.get('route_completed') is True and not seq.get('tracking_interrupted')
                   and len(runs) == 1 and len(runs[0]) == len(stamps))
    route = json.loads((session / 'scan_route.json').read_text())['waypoints']
    closed = len(route) > 1 and np.hypot(route[-1]['x'] - route[0]['x'], route[-1]['y'] - route[0]['y']) <= .1
    report = dict(session=session.name, approved_frames=len(approved), mesh_keyframes=len(stamps),
                  matched_frames=sum(map(len, runs)), whole_route_comparable=bool(whole_route),
                  whole_loop_comparable=bool(whole_route and closed),
                  unpublished_timestamp_ns=[int(t) for t in stamps if t not in by_stamp],
                  missing_sdk_timestamp_ns=stamps[~np.isfinite(candidate).all(axis=(1, 2))].tolist(),
                  live_bounds_m=seq['bounds_m'], segments=[])
    out = session / 'loop_ab'
    out.mkdir(exist_ok=True)
    mesh_metadata = session / 'final_mesh.json'
    if mesh_metadata.exists():
        live_voxel = json.loads(mesh_metadata.read_text())['voxel_m']
    else:
        from worker_reconstruction import new_volume
        live_voxel = new_volume(np.asarray(seq['bounds_m'])).voxel
    if approved:
        truth = np.array(reference_cameras(session, approved))
        # Align only the extracted surfaces for evaluation, exactly once for both variants.
        world_from_map = truth[0] @ np.linalg.inv(approved[0]['visual_camera_to_world'])
        report['world_from_map_for_wall_scoring'] = world_from_map.tolist()
        if (session / 'final_mesh.ply').exists():
            live_strips = final_wall_strips(session, world_from_map)
            report['live_mesh'] = wall_metrics(live_strips, live_strips)
            report['bmap1_wall_check'] = bmap1_wall_check(live_strips)
        live_poses = {'A': np.asarray([f['camera_to_world'] for f in approved])}
        body = truth @ np.linalg.inv([f['body_from_camera'] for f in approved])
        report['approved_endpoint'] = (pair_scores([0, len(approved) - 1], approved, truth, live_poses)
                                       if len(approved) > 1 else None)
        report['approved_revisit'] = pair_scores(revisit_pair(body), approved, truth, live_poses)

    for n, indices in enumerate(runs):
        rows = [by_stamp[stamps[i]] for i in indices]
        frames = [approved[r] for r in rows]
        reference = truth[rows]
        poses = dict(A=np.asarray([f['camera_to_world'] for f in frames]),
                     B=np.array([floor_camera_pose(candidate[i], f['odometry_body_pose'], f['body_from_camera'])
                                 for i, f in zip(indices, frames)]))
        bounds = shared_bounds(seq['bounds_m'], poses['A'], poses['B'], live_voxel).tolist()
        bodies = reference @ np.linalg.inv([f['body_from_camera'] for f in frames])
        segment = dict(epoch=int(epochs[indices[0]]), keyframes=len(indices), bounds_m=bounds,
                       file_range=[frames[0]['file'], frames[-1]['file']],
                       timestamp_ns=[int(stamps[indices[0]]), int(stamps[indices[-1]])],
                       endpoint=pair_scores([0, len(rows) - 1], frames, reference, poses) if len(rows) > 1 else None,
                       revisit=pair_scores(revisit_pair(bodies), frames, reference, poses), meshes={})
        report['segments'].append(segment)
        strips = {}
        for label, variant in poses.items():
            sequence = dict(seq, bounds_m=bounds, frames=[dict(f, camera_to_world=m.tolist()) for f, m in zip(frames, variant)])
            if len(frames) < 3:
                segment['meshes'][label] = dict(reason='Fewer than three views')
                continue
            try:
                surface, voxel = build_surface(session, sequence)
            except RuntimeError as exc:
                if str(exc) not in ('insufficient observed zero crossings for a surface', 'no supported surface components'):
                    raise
                segment['meshes'][label] = dict(reason=str(exc))
                continue
            target = out / ('%03d_%s.ply' % (n, label))
            surface.export(target)
            surface.apply_transform(world_from_map)
            strips[label] = wall_strips(surface)
            segment['meshes'][label] = dict(file=target.name, faces=len(surface.faces), voxel_m=float(voxel),
                                            observed=wall_metrics(strips[label], strips[label]))
        if len(strips) == 2:
            # Fit opposite walls and both variants on identical longitudinal bins.
            support = {}
            for a, b in WALL_PAIRS:
                support[a] = support[b] = set.intersection(*(set(s[wall]) for s in strips.values() for wall in (a, b)))
            for label in strips:
                segment['meshes'][label]['common'] = wall_metrics(strips[label], support)
    write_json(out / 'metrics.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--walls-only', action='store_true', help='Score the finalized mesh without rebuilding it')
    args = parser.parse_args()
    session = args.session
    if args.walls_only:
        print(json.dumps(check_final_walls(session)))
        raise SystemExit(0)
    result = compare(session)
    print('%s (%d/%d common keyframes)' % (session / 'loop_ab/metrics.json',
                                            result['matched_frames'], result['mesh_keyframes']))
