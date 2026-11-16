"""Scan-derived navigation grid; coordinates are row=y, column=x, origin is an edge.

Floor is assumed horizontal at world z=0. Classified scans use ground support,
obstacle hits and ambiguous/dropoff endpoint vetoes. Legacy scans use floor hits
within ±2.5cm and obstacle endpoints from 2.5–40cm. Free space requires
two views of floor and two measured-depth observations at every 2.5cm height
from 2.5–40cm. Invalid/no-return pixels never clear space. This sampled volume
is a prototype approximation, not a collision certificate for thin unseen objects.
"""
import heapq
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import binary_dilation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live.workers import fusion_depth
from mapping.odometry.localization import rigid
from reconstruction.ground_segmentation import read_labels,GROUND,OBSTACLE,UNCERTAIN

FREE, OCCUPIED, UNKNOWN = 0, 1, -1


class NavigationGrid:
    def __init__(self, cells, origin_xy, resolution, clearance_m=.25, *, demo_only=False,
                 elevation=None, roughness=None, slope=None):
        cells, origin = np.asarray(cells), np.asarray(origin_xy, dtype=float)
        if (cells.ndim != 2 or not cells.size or not np.isin(cells, [FREE, OCCUPIED, UNKNOWN]).all()
                or origin.shape != (2,) or not np.isfinite(origin).all()
                or not np.isfinite(resolution) or resolution <= 0
                or not np.isfinite(clearance_m) or clearance_m < 0
                or not isinstance(demo_only, (bool, np.bool_))):
            raise ValueError('Invalid navigation grid geometry or cell states')
        shape = cells.shape
        layers = []
        for index, layer in enumerate((elevation, roughness, slope)):
            values = np.full(shape, np.nan) if layer is None else np.asarray(layer, dtype=float)
            if (values.shape != shape or np.isinf(values).any()
                    or (index > 0 and np.any(values[np.isfinite(values)] < 0))):
                raise ValueError('Terrain layers must match the grid and contain no infinities')
            layers.append(values.copy())
        self.cells, self.origin = cells.astype(np.int8, copy=True), origin.copy()
        self.resolution, self.clearance_m = float(resolution), float(clearance_m)
        self.demo_only = bool(demo_only)
        self.elevation, self.roughness, self.slope = layers
        measured = np.isfinite(self.elevation)
        characterized = measured & np.isfinite(self.slope) & np.isfinite(self.roughness)
        self.terrain_cost = np.where(characterized,
                                     np.minimum(self.slope/np.deg2rad(10), 1.) +
                                     np.minimum(self.roughness/.02, 1.),
                                     measured.astype(float))
        self.blocked = self.cells != FREE
        if clearance_m > 0:
            radius = int(np.ceil(clearance_m/resolution))+1
            y, x = np.mgrid[-radius:radius+1, -radius:radius+1]
            # Inflate entire cell squares, so exact endpoint positions remain safe.
            footprint = np.hypot(np.maximum(abs(x)-1, 0), np.maximum(abs(y)-1, 0))*resolution <= clearance_m+1e-12
            self.blocked = binary_dilation(self.blocked, footprint, border_value=1)
        for array in (self.cells, self.origin, self.blocked, self.elevation,
                      self.roughness, self.slope, self.terrain_cost):
            array.flags.writeable = False

    def world_to_cell(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('Require finite x/y coordinates')
        col, row = np.floor((xy-self.origin)/self.resolution).astype(np.int64)
        return int(row), int(col)

    def cell_to_world(self, cell):
        cell = np.asarray(cell)
        if cell.shape != (2,) or not np.isfinite(cell).all() or not np.equal(cell, np.floor(cell)).all():
            raise ValueError('Require integer row/column coordinates')
        return self.origin+(cell[::-1]+.5)*self.resolution

    def segment_clear(self, a_xy, b_xy):
        a, b = np.asarray(a_xy, dtype=float), np.asarray(b_xy, dtype=float)
        if a.shape != (2,) or b.shape != (2,) or not np.isfinite([a, b]).all():
            return False
        a, b = (a-self.origin)/self.resolution, (b-self.origin)/self.resolution
        if np.any(np.minimum(a, b) <= 0) or np.any(np.maximum(a, b) >= self.cells.shape[::-1]):
            return False
        lo = np.maximum(np.floor(np.minimum(a, b)).astype(int)-1, 0)
        hi = np.minimum(np.floor(np.maximum(a, b)).astype(int)+2, self.cells.shape[::-1])
        rows, cols = np.nonzero(self.blocked[lo[1]:hi[1], lo[0]:hi[0]])
        if not len(rows):
            return True
        lower = np.column_stack((cols+lo[0], rows+lo[1]))
        enter, leave = np.zeros(len(rows)), np.ones(len(rows))
        for axis in range(2):
            delta = b[axis]-a[axis]
            if abs(delta) < 1e-12:
                outside = (a[axis] < lower[:, axis]-1e-12) | (a[axis] > lower[:, axis]+1+1e-12)
                enter[outside] = np.inf
            else:
                t0, t1 = (lower[:, axis]-a[axis])/delta, (lower[:, axis]+1-a[axis])/delta
                enter = np.maximum(enter, np.minimum(t0, t1))
                leave = np.minimum(leave, np.maximum(t0, t1))
        return not np.any(enter <= leave+1e-12)

    def plan(self, start_xy, goal_xy):
        start, goal = np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float)
        if not self.segment_clear(start, start) or not self.segment_clear(goal, goal):
            raise ValueError('Start or goal is outside observed traversable space')
        source, target = self.world_to_cell(start), self.world_to_cell(goal)
        if source == target and self.segment_clear(start, goal):
            return [start.tolist(), goal.tolist()]
        queue, costs, previous = [(0., source)], {source: 0.}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == target:
                chain = [target]
                while chain[-1] != source:
                    chain.append(previous[chain[-1]])
                path = [start.tolist()]+[self.cell_to_world(c).tolist() for c in reversed(chain)]+[goal.tolist()]
                if not all(self.segment_clear(a, b) for a, b in zip(path, path[1:])):
                    raise ValueError('Exact endpoint connection is blocked')
                return path
            row, col = current
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
                r, c = row+dr, col+dc
                if not (0 <= r < self.cells.shape[0] and 0 <= c < self.cells.shape[1]) or self.blocked[r, c]:
                    continue
                if dr and dc and (self.blocked[row, c] or self.blocked[r, col]):
                    continue
                distance = np.hypot(dr, dc)
                cost = costs[current]+distance*(1.+self.terrain_cost[r, c])
                neighbor = (r, c)
                if cost >= costs.get(neighbor, np.inf):
                    continue
                costs[neighbor], previous[neighbor] = cost, current
                heuristic = np.hypot(r-target[0], c-target[1])
                heapq.heappush(queue, (cost+heuristic, neighbor))
        raise ValueError('No path through observed traversable space')

    def save(self, path):
        with Path(path).open('xb') as stream:
            np.savez_compressed(stream, cells=self.cells, origin=self.origin,
                                resolution=self.resolution, clearance_m=self.clearance_m, demo_only=self.demo_only,
                                elevation=self.elevation, roughness=self.roughness, slope=self.slope)

    @classmethod
    def load(cls, path):
        try:
            with np.load(path, allow_pickle=False) as data:
                return cls(data['cells'], data['origin'], float(data['resolution']), float(data['clearance_m']),
                           demo_only=data['demo_only'].item() if 'demo_only' in data else False,
                           elevation=data['elevation'] if 'elevation' in data else None,
                           roughness=data['roughness'] if 'roughness' in data else None,
                           slope=data['slope'] if 'slope' in data else None)
        except (KeyError, TypeError) as error:
            raise ValueError('Invalid saved navigation grid') from error


def _depth(path, *, classified=False):
    with np.load(path, allow_pickle=False) as sample:
        depth = sample['depth']
        if depth.ndim != 2 or min(depth.shape) < 2 or depth.dtype != np.uint16:
            raise ValueError('Require a 2D uint16 millimeter fusion depth image')
        result = fusion_depth(sample)
        labels = read_labels(sample, depth.shape) if classified else None
    # The fusion model is configured for a maximum depth of ten meters.
    result[(depth == 0) | (depth > 10000)] = np.nan
    return (result, labels) if classified else result


def _xy_cells(points, resolution):
    cells = np.floor(points[:, :2]/resolution).astype(np.int64)
    if not len(cells):
        return cells
    shifted = cells-cells.min(0)
    keys = np.ravel_multi_index(shifted.T, shifted.max(0)+1)
    return cells[np.unique(keys, return_index=True)[1]]


def build_grid(session, resolution=.05, clearance_m=.25):
    """Carve measured ray free-space, using only saved poses and fusion depth.

    Projective carving checks all four neighboring valid depth rays and leaves
    3cm before their measured endpoints. It never extrapolates no-return rays.
    shortcut: 2.5cm vertical samples with horizontal floor assumption; upgrade
    to calibrated volumetric occupancy before hardware navigation.
    """
    NavigationGrid([[UNKNOWN]], [0, 0], resolution, clearance_m)
    session = Path(session)
    sequence_path = session/'surface_sequence.json'
    if not sequence_path.exists():
        sequence_path = session/'live_sequence.json'
    sequence = json.loads(sequence_path.read_text())
    intrinsics = np.asarray(sequence['intrinsics'], dtype=float)
    if intrinsics.shape != (4,) or not np.isfinite(intrinsics).all() or min(intrinsics[:2]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    fx, fy, cx, cy = intrinsics
    frames = sequence['frames']
    if not frames:
        raise ValueError('Scan has no frames')
    floor_counts, floor_heights, obstacles, origins, ambiguous = {}, {}, set(), [], set()
    classification_mode = None
    poses = []
    for frame in frames:
        pose = rigid(frame['camera_to_world'])
        poses.append(pose)
        origins.append(np.floor(pose[:2, 3]/resolution).astype(int))
        filename = Path(frame['file'])
        if filename.is_absolute() or '..' in filename.parts:
            raise ValueError('Frame paths must be relative to the scan session')
        depth, labels = _depth(session/filename, classified=True)
        if classification_mode is not None and classification_mode != (labels is not None):
            raise ValueError('Do not mix classified and unclassified frames; replay the whole sequence')
        classification_mode = labels is not None
        y, x = np.nonzero(np.isfinite(depth))
        d = depth[y, x]
        points = np.column_stack(((x-cx)*d/fx, (y-cy)*d/fy, d)) @ pose[:3, :3].T+pose[:3, 3]
        if labels is None:
            floor = points[np.abs(points[:, 2]) <= .025]
            occupied = points[(points[:, 2] > .025) & (points[:, 2] <= .4)]
        else:
            classes = labels[y, x]
            floor = points[classes == GROUND]
            envelope = (points[:, 2] > -.1) & (points[:, 2] <= .4)
            occupied = points[(classes == OBSTACLE) & envelope]
            unsure = points[(classes == UNCERTAIN) & (points[:, 2] <= .4) & (np.abs(points[:, 2]) > .012)]
            ambiguous.update(map(tuple, _xy_cells(unsure, resolution)))
        for xy in _xy_cells(floor, resolution):
            cell = tuple(xy)
            floor_counts[cell] = floor_counts.get(cell, 0)+1
        if len(floor):
            floor_cells = np.floor(floor[:, :2]/resolution).astype(np.int64)
            unique_cells, inverse = np.unique(floor_cells, axis=0, return_inverse=True)
            sums = np.bincount(inverse, weights=floor[:, 2])
            counts = np.bincount(inverse)
            for cell, height in zip(unique_cells, sums/counts):
                floor_heights.setdefault(tuple(cell), []).append(float(height))
        obstacles.update(map(tuple, _xy_cells(occupied, resolution)))
    if not floor_counts:
        raise ValueError('Scan has no measured floor support near z=0')
    observed = np.asarray(list(floor_counts)+list(obstacles)+list(ambiguous)+origins)
    lower, upper = observed.min(0)-1, observed.max(0)+2
    shape = (upper-lower)[::-1]
    if np.prod(shape) > 4_000_000:
        raise ValueError('Scan bounds exceed four million navigation cells')
    cells = np.full(shape, UNKNOWN, np.int8)
    for col, row in obstacles:
        cells[row-lower[1], col-lower[0]] = OCCUPIED
    candidates = np.asarray([xy for xy, count in floor_counts.items()
                             if count >= 2 and xy not in obstacles and xy not in ambiguous], dtype=int).reshape(-1, 2)
    heights = np.arange(.025, .4001, .025)
    coverage = np.zeros((len(candidates), len(heights)), np.uint8)
    xy = (candidates+.5)*resolution
    for frame, pose in zip(frames, poses):
        missing = np.argwhere(coverage < 2)
        if not len(missing):
            break
        points = np.column_stack((xy[missing[:, 0]], heights[missing[:, 1]]))
        camera = (points-pose[:3, 3]) @ pose[:3, :3]
        forward = camera[:, 2] > .03
        indices = np.flatnonzero(forward)
        camera = camera[forward]
        pixels = np.floor(camera[:, :2]/camera[:, 2, None]*[fx, fy]+[cx, cy]).astype(int)
        depth = _depth(session/frame['file'])
        inside = (pixels[:, 0] >= 0) & (pixels[:, 1] >= 0) & (pixels[:, 0]+1 < depth.shape[1]) & (pixels[:, 1]+1 < depth.shape[0])
        pixels, camera, indices = pixels[inside], camera[inside], indices[inside]
        u, v = pixels.T
        rays = np.column_stack((depth[v, u], depth[v+1, u], depth[v, u+1], depth[v+1, u+1]))
        clear = np.all(np.isfinite(rays) & (rays > camera[:, 2, None]+.03), axis=1)
        selected = missing[indices[clear]]
        coverage[selected[:, 0], selected[:, 1]] += 1
    traversable = candidates[np.all(coverage >= 2, axis=1)]
    cells[traversable[:, 1]-lower[1], traversable[:, 0]-lower[0]] = FREE
    elevation = np.full(shape, np.nan)
    for xy in traversable:
        heights_at_cell = floor_heights.get(tuple(xy), ())
        if len(heights_at_cell) >= 2:
            elevation[xy[1]-lower[1], xy[0]-lower[0]] = np.median(heights_at_cell)
    roughness = np.full(shape, np.nan)
    slope = np.full(shape, np.nan)
    known = np.isfinite(elevation)
    for row, col in zip(*np.nonzero(known)):
        y0, y1 = max(0, row-1), min(shape[0], row+2)
        x0, x1 = max(0, col-1), min(shape[1], col+2)
        neighborhood = elevation[y0:y1, x0:x1]
        local_rows, local_cols = np.nonzero(np.isfinite(neighborhood))
        if len(local_rows) >= 3:
            z = neighborhood[local_rows, local_cols]
            xy = np.column_stack(((local_cols+ x0-col)*resolution,
                                  (local_rows+ y0-row)*resolution))
            design = np.column_stack((xy, np.ones(len(xy))))
            if np.linalg.matrix_rank(design) == 3:
                plane, *_ = np.linalg.lstsq(design, z, rcond=None)
                roughness[row, col] = np.median(np.abs(z-design@plane))
                slope[row, col] = np.arctan(np.linalg.norm(plane[:2]))
    return NavigationGrid(cells, lower*resolution, resolution, clearance_m,
                          elevation=elevation, roughness=roughness, slope=slope)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Build a conservative navigation grid from fusion keyframes.')
    parser.add_argument('session', type=Path)
    parser.add_argument('output', type=Path, help='New NPZ outside the source map directory')
    args = parser.parse_args()
    source, output = args.session.resolve(), args.output.resolve()
    if output.exists() or source == output or source in output.parents:
        parser.error('Output must be new and outside the source map directory')
    grid = build_grid(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    grid.save(output)
    print(json.dumps(dict(file=str(output), free=int((grid.cells == FREE).sum()),
                          occupied=int((grid.cells == OCCUPIED).sum()),
                          unknown=int((grid.cells == UNKNOWN).sum()),
                          traversable=int((~grid.blocked).sum())), indent=2))
