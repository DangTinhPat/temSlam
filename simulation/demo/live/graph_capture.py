"""Record SDK graph evidence, keeping SDK node IDs separate from mesh timestamps."""
from collections import Counter
import json
import sys

import numpy as np

from worker_common import write_json, write_npz
from vslam import tracker as sdk


def native_pose(pose):
    return [*pose.translation, *pose.rotation]  # SDK xyz + quaternion xyzw


def observable_errors(previous, candidate, times, epochs, original):
    valid = np.flatnonzero(np.isfinite(candidate).all(axis=(1, 2)))
    if original and len(valid) and len(valid) == len(times):
        return sdk.graph_diagnostics(previous, candidate, times)
    details = dict(consistent=None, origin=None, violations=[])
    breaks = np.flatnonzero((np.diff(valid) != 1) | (np.diff(epochs[valid]) != 0)) + 1
    for run in np.split(valid, breaks):
        if not len(run):
            continue
        residuals = sdk.graph_diagnostics(previous[run], candidate[run], np.asarray(times)[run])
        if run[0] == 0:
            details['origin'] = residuals['origin']
        details['violations'].extend(v for v in residuals['violations']
                                     if run[0] == 0 or v['from_ns'] != v['to_ns'])
    return details


def isolate_lc_edge(old_nodes, head, new_nodes, added):
    if not new_nodes and len(added) == 1:
        return added[0]
    if len(new_nodes) != 1 or len(added) != 2 or head is None:
        return None
    new = new_nodes[0]
    # cuVSLAM 17 adds predecessor -> new head and matched old node -> new head.
    sources = [a for a, b in added if b == new and a in old_nodes]
    if len(set(sources)) == 2 and head in sources:
        return next(edge for edge in added if edge[0] != head)
    return None


class GraphCapture:
    def __init__(self, session):
        self.folder = session / 'graph_snapshots'
        self.folder.mkdir(exist_ok=True)
        self.epoch = 0
        self.alignment = None
        self.archived = {}
        self.reset_topology()
        self.errors = set()

    def reset_topology(self):
        self.seen_events = set()
        self.nodes = set()
        self.edges = Counter()
        self.head = None
        self.last_read = None
        self.last_track = None

    def log(self, kind, ts, **data):
        with (self.folder/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(kind=kind, timestamp_ns=int(ts), epoch=self.epoch, **data))+'\n')

    def error(self, where, exc, ts):
        message = f'{where}: {exc}'
        if message not in self.errors:
            self.errors.add(message)
            print(f'Graph capture: {message}', file=sys.stderr, flush=True)
            try:
                self.log('capture_error', ts, operation=where, error=str(exc))
            except OSError:
                pass

    def observe(self, tracker, ts, frames):
        """Read each tracked sample so node first sightings have a bounded time interval."""
        try:
            metrics = tracker.get_slam_metrics()
            event = None
            if metrics and (metrics.lc_status or metrics.pgo_status):
                key = (metrics.timestamp_ns, metrics.lc_status, metrics.pgo_status)
                if key not in self.seen_events:
                    self.seen_events.add(key)
                    event = {k: int(getattr(metrics, k)) for k in
                             ('timestamp_ns', 'lc_status', 'pgo_status', 'lc_selected_landmarks_count',
                              'lc_tracked_landmarks_count', 'lc_pnp_landmarks_count', 'lc_good_landmarks_count')
                             if hasattr(metrics, k)}
            if event is not None:
                self.snapshot(tracker, ts, frames, f'event_{ts}', 'observed')
                self.log('sdk_event', ts, metrics=event)
            graph = tracker.get_pose_graph()
            if graph is None:
                return
            nodes, edges = graph.nodes, graph.edges
            ids = {n.id for n in nodes}
            pairs = Counter((e.node_from, e.node_to) for e in edges)
            added = sorted((pairs - self.edges).elements())
            removed = sorted((self.edges - pairs).elements())
            new_nodes = sorted(ids - self.nodes)
            lost_nodes = sorted(self.nodes - ids)
            if event is not None or added or removed or new_nodes or lost_nodes:
                synchronous = (event is not None and event['lc_status'] and event['timestamp_ns'] == ts
                               and self.last_read is not None and self.last_read == self.last_track)
                edge = (isolate_lc_edge(self.nodes, self.head, new_nodes, added)
                        if synchronous and not removed and not lost_nodes else None)
                self.log('topology', ts, previous_read_ns=self.last_read, new_nodes=new_nodes,
                         removed_nodes=lost_nodes, new_edges=added, removed_edges=removed,
                         isolated_lc_edge=edge)
            if new_nodes or lost_nodes:
                self.head = new_nodes[0] if len(new_nodes) == 1 and not lost_nodes else None
            self.nodes, self.edges, self.last_read = ids, pairs, ts
            if event is not None:
                write_npz(self.folder/f'topology_{ts}.npz',
                          node_id=np.array([n.id for n in nodes], np.uint64),
                          node_pose_xyz_xyzw=np.array([native_pose(n.node_pose) for n in nodes]).reshape(-1, 7),
                          endpoints=np.array([(e.node_from, e.node_to) for e in edges], np.uint64).reshape(-1, 2),
                          transform_xyz_xyzw=np.array([native_pose(e.transform) for e in edges]).reshape(-1, 7),
                          covariance=np.array([e.covariance for e in edges]).reshape(-1, 36))
        except Exception as exc:  # An optional SDK internal or its export must never change tracking.
            self.error('observe', exc, ts)
        finally:
            self.last_track = ts

    def candidates(self, tracker, frames, optimized=None):
        stamps = {f['timestamp_ns'] for f in frames if f['tracker_epoch'] == self.epoch}
        result = self.archived.copy()
        if stamps and self.alignment is not None:
            if optimized is None:
                optimized = {p.timestamp_ns: sdk.pose_matrix(p.pose) for p in tracker.get_all_slam_poses()
                             if p.timestamp_ns in stamps}
            result.update({ts: self.alignment @ matrix for ts, matrix in optimized.items() if ts in stamps})
        return result

    def snapshot(self, tracker, ts, frames, name, decision, optimized=None):
        try:
            try:
                poses = self.candidates(tracker, frames, optimized)
            except Exception as exc:
                self.error('pose_history', exc, ts)
                poses = self.archived.copy()
            times = [f['timestamp_ns'] for f in frames]
            epochs = np.asarray([f['tracker_epoch'] for f in frames], dtype=np.int64)
            previous = np.asarray([f['visual_camera_to_world'] for f in frames]).reshape(-1, 4, 4)
            mapped = np.asarray([f['camera_to_world'] for f in frames]).reshape(-1, 4, 4)
            missing = [t for t in times if t not in poses]
            candidate = np.full(previous.shape, np.nan)
            for i, t in enumerate(times):
                if t in poses:
                    candidate[i] = poses[t]
            write_npz(self.folder/f'{name}.npz', timestamp_ns=np.array(times, np.int64), previous=previous,
                      candidate=candidate, mapped=mapped, tracker_epoch=epochs)
            details = observable_errors(previous, candidate, times, epochs, self.epoch == 0)
            self.log('snapshot', ts, snapshot=f'{name}.npz', decision=decision, missing_ns=missing, **details)
            return poses
        except Exception as exc:
            self.error('snapshot', exc, ts)
            return self.archived.copy()

    def restart(self, tracker, ts, frames):
        self.archived = self.snapshot(tracker, ts, frames, f'before_restart_{ts}', 'restart')
        self.epoch += 1
        self.alignment = None
        self.reset_topology()
        try:
            self.log('restart', ts)
        except OSError as exc:
            self.error('restart', exc, ts)


def return_edge_evidence(rows, outbound_ns, return_ns):
    """Compare SDK endpoint first-sighting intervals against independently observed visit windows."""
    if not outbound_ns[0] <= outbound_ns[1] < return_ns[0] <= return_ns[1]:
        raise ValueError('Expected disjoint, ordered outbound and return time windows')
    first_visit = {}
    result = dict(verified=[], ambiguous=[])
    for row in rows:
        if row['kind'] != 'topology':
            continue
        epoch, ts, prev = row['epoch'], row['timestamp_ns'], row['previous_read_ns']
        for node in row.get('removed_nodes', ()):
            first_visit.pop((epoch, node), None)
        for node in row['new_nodes']:
            for phase, (start, end) in enumerate((outbound_ns, return_ns)):
                if prev is not None and start <= prev <= ts <= end:
                    first_visit[epoch, node] = phase
        if not return_ns[0] <= ts <= return_ns[1]:
            continue
        for edge in row['new_edges']:
            if {first_visit.get((epoch, node)) for node in edge} == {0, 1}:
                category = 'verified' if edge == row['isolated_lc_edge'] else 'ambiguous'
                result[category].append(dict(epoch=epoch, timestamp_ns=ts, edge=edge))
    return result


def attributed_topology(rows, track_stamps):
    """Join synchronous or next-sample metrics by their measurement stamp."""
    prior = dict(zip(track_stamps[1:], track_stamps[:-1]))
    closures = Counter()
    for row in rows:
        if row['kind'] == 'sdk_event' and row['metrics']['lc_status']:
            measured, read = row['metrics']['timestamp_ns'], row['timestamp_ns']
            if measured == read or measured == prior.get(read):
                closures[row['epoch'], measured] += 1
    epoch, old_nodes, head = None, set(), None
    for row in rows:
        if row['kind'] != 'topology':
            continue
        if row['epoch'] != epoch:
            epoch, old_nodes, head = row['epoch'], set(), None
        ts, nodes, added = row['timestamp_ns'], row['new_nodes'], row['new_edges']
        removed = row['removed_nodes']
        edge = None
        if (closures[epoch, ts] == 1 and row['previous_read_ns'] is not None
                and row['previous_read_ns'] == prior.get(ts) and not removed and not row['removed_edges']):
            edge = isolate_lc_edge(old_nodes, head, nodes, added)
        yield dict(row, isolated_lc_edge=edge)
        old_nodes.difference_update(removed)
        old_nodes.update(nodes)
        if nodes or removed:
            head = nodes[0] if len(nodes) == 1 and not removed else None


def assess_return_edges(session, windows_ns=None, *, region=None):
    """Use simulator captures only in offline scoring, independent of the candidate graph."""
    with (session/'graph_snapshots/events.jsonl').open() as stream:
        rows = [json.loads(line) for line in stream]
    track_stamps = sorted(int(f.stem) for f in (session/'tracker_input').glob('*.npz'))
    topology = list(attributed_topology(rows, track_stamps))
    with (session/'live_capture.jsonl').open() as stream:
        captures = [json.loads(line) for line in stream]
    times = np.array([round(float(c['t']) * 1e9) for c in captures], dtype=np.int64)
    xy = np.array([[float(c['x']), float(c['y'])] for c in captures])
    result = dict(verified=[], ambiguous=[], verified_in_final_graph=False, sdk_lc_measurements=sum(
        r['kind']=='sdk_event' and bool(r['metrics']['lc_status']) for r in rows), attributable_edges=[])
    if not len(times) or np.any(np.diff(times) <= 0):
        return dict(result, reason='Missing or unordered reference captures')
    seen = {}
    for row in topology:
        epoch, ts = row['epoch'], row['timestamp_ns']
        for node in row['removed_nodes']:
            seen.pop((epoch, node), None)
        for node in row['new_nodes']:
            seen[epoch, node] = ts
        edge = row['isolated_lc_edge']
        if edge is not None:
            first = [seen[epoch, n] for n in edge]
            indices = [int(np.argmin(abs(times - stamp))) for stamp in first]
            gap = (float(np.linalg.norm(np.diff(xy[indices], axis=0)))
                   if max(abs(times[indices] - first)) <= 20_000_000 else None)
            result['attributable_edges'].append(dict(epoch=epoch, timestamp_ns=ts, edge=edge,
                                                    first_seen_ns=first, reference_xy_gap_m=gap))
    if windows_ns is None:
        waypoints = json.loads((session/'scan_route.json').read_text())['waypoints']
        route = np.array([[point['x'], point['y']] for point in waypoints])
        completed = json.loads((session/'headless_result.json').read_text()).get('route_completed')
        if not completed or len(route) < 2 or np.linalg.norm(route[-1] - route[0]) > .1:
            return dict(result, reason='Route incomplete or open')
        center, radius = (route[0], .4) if region is None else (np.asarray(region[:2]), region[2])
        if not np.isfinite(center).all() or not np.isfinite(radius) or radius <= 0:
            raise ValueError('Expected finite region coordinates and a positive radius')
        result.update(region_xy_m=center.tolist(), region_radius_m=radius)
        home = np.linalg.norm(xy - center, axis=1) <= radius
        far = route[np.argmax(np.linalg.norm(route - center, axis=1))]
        far_distances = np.linalg.norm(xy - far, axis=1)
        pivot = int(np.argmin(far_distances))
        if ((region is None and not (home[0] and home[-1])) or home[pivot]
                or far_distances[pivot] > radius):
            return dict(result, reason='Reference captures do not show departure, far waypoint and return')
        # Stand-up can briefly leave the start region before the robot settles for the scan.
        boundaries = np.flatnonzero(np.diff(home.astype(int))) + 1
        visits = [(r[0], r[-1]) for r in np.split(np.arange(len(home)), boundaries) if home[r[0]]]
        early = [r for r in visits if r[1] < pivot]
        late = [r for r in visits if r[0] > pivot]
        if not early or not late:
            return dict(result, reason='Missing visits on opposite sides of the far waypoint')
        departure, arrival = early[-1], late[-1]
        windows_ns = ([int(times[i]) for i in departure], [int(times[i]) for i in arrival])
    outbound, returning = windows_ns
    result.update(outbound_ns=outbound, return_ns=returning)
    result.update(return_edge_evidence(topology, outbound, returning))
    final = next((r for r in reversed(rows) if r.get('snapshot') == 'final.npz'), None)
    if final and final['decision'] == 'applied' and result['verified']:
        retained = Counter()
        for row in topology:
            if row['epoch'] == final['epoch'] and row['timestamp_ns'] <= final['timestamp_ns']:
                retained.update(map(tuple, row['new_edges']))
                retained.subtract(map(tuple, row['removed_edges']))
        result['verified_in_final_graph'] = any(
            r['epoch'] == final['epoch'] and r['timestamp_ns'] <= final['timestamp_ns']
            and retained[tuple(r['edge'])] > 0 for r in result['verified'])
    return result


if __name__ == '__main__':
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description='Check SDK edges between two measured visits to a region.')
    parser.add_argument('session', type=Path)
    parser.add_argument('--outbound', type=float, nargs=2, metavar=('START_S', 'END_S'))
    parser.add_argument('--return-visit', type=float, nargs=2, metavar=('START_S', 'END_S'))
    parser.add_argument('--region', type=float, nargs=3, metavar=('X_M', 'Y_M', 'RADIUS_M'),
                        help='Assess visits to this landmark before and after the farthest waypoint')
    args = parser.parse_args()
    if bool(args.outbound) != bool(args.return_visit):
        parser.error('Specify both visit windows or use measured start-region visits')
    if args.region and args.outbound:
        parser.error('Choose either a reference region or explicit time windows')
    windows = ([round(s*1e9) for s in args.outbound], [round(s*1e9) for s in args.return_visit]) if args.outbound else None
    report = assess_return_edges(args.session, windows, region=args.region)
    name = 'visit_assessment.json' if windows or args.region else 'return_assessment.json'
    write_json(args.session/'graph_snapshots'/name, report)
    print(json.dumps(report, indent=2))
