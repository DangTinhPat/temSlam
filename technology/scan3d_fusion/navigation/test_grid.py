import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from grid import FREE, OCCUPIED, UNKNOWN, NavigationGrid, build_grid, _xy_cells
from reconstruction.ground_segmentation import SOURCE


class GridTests(unittest.TestCase):
    def test_fast_projection_preserves_negative_coordinates_and_duplicate_cells(self):
        points = np.random.default_rng(51).uniform(-4, 4, (10000, 3))
        expected = np.unique(np.floor(points[:, :2]/.05).astype(np.int64), axis=0)
        np.testing.assert_array_equal(_xy_cells(points, .05), expected)
        self.assertEqual(_xy_cells(np.empty((0, 3)), .05).shape, (0, 2))

    def test_unknown_obstacles_and_boundary_are_inflated(self):
        cells = np.zeros((20, 20), np.int8)
        cells[10, 10], cells[5, 5] = OCCUPIED, UNKNOWN
        grid = NavigationGrid(cells, [0, 0], .1, .2)
        self.assertTrue(grid.blocked[10, 12])
        self.assertTrue(grid.blocked[5, 7])
        self.assertTrue(grid.blocked[0, 15])
        self.assertFalse(grid.blocked[15, 15])

    def test_astar_detours_and_preserves_exact_endpoints(self):
        cells = np.zeros((12, 12), np.int8)
        cells[:8, 6] = OCCUPIED
        grid = NavigationGrid(cells, [-1, -1], 1., 0.)
        start, goal = [1.2, 2.1], [8.3, 2.4]
        path = grid.plan(start, goal)
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_array_equal(path[-1], goal)
        self.assertGreater(max(p[1] for p in path), 7.)
        self.assertTrue(all(grid.segment_clear(a, b) for a, b in zip(path, path[1:])))

    def test_same_cell_goal_has_no_spurious_trip_to_cell_center(self):
        grid = NavigationGrid(np.zeros((8, 8), np.int8), [0, 0], 1., 0.)
        self.assertEqual(grid.plan([3.1, 3.1], [3.2, 3.2]), [[3.1, 3.1], [3.2, 3.2]])

    def test_supercover_checks_corner_and_gridline_contact(self):
        cells = np.zeros((8, 8), np.int8)
        cells[3, 4] = OCCUPIED
        grid = NavigationGrid(cells, [0, 0], 1., 0.)
        self.assertFalse(grid.segment_clear([3.5, 3.5], [4.5, 4.5]))
        self.assertFalse(grid.segment_clear([4., 2.5], [4., 4.5]))
        self.assertFalse(grid.segment_clear([0, 2], [0, 3]))
        self.assertFalse(grid.segment_clear([np.nan, 0], [1, 1]))

    def test_decimal_gridline_endpoint_cannot_hide_touching_obstacle(self):
        cells = np.zeros((8, 8), np.int8)
        cells[3, 3] = OCCUPIED
        grid = NavigationGrid(cells, [0, 0], .1, 0.)
        self.assertFalse(grid.segment_clear([.15, .35], [.3, .35]))

    def test_invalid_blocked_and_unreachable_endpoints_fail(self):
        cells = np.zeros((9, 9), np.int8)
        cells[:, 4] = UNKNOWN
        grid = NavigationGrid(cells, [0, 0], 1., 0.)
        for start, end in [([2.5, 2.5], [6.5, 2.5]), ([4.5, 2.5], [2.5, 2.5]),
                           ([-1, 0], [2.5, 2.5]), ([2.5, 2.5], [np.inf, 1])]:
            with self.assertRaises(ValueError):
                grid.plan(start, end)

    def test_save_load_validates_and_never_overwrites(self):
        grid = NavigationGrid(np.zeros((10, 10), np.int8), [-2., 1.], .2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'grid.npz'
            grid.save(path)
            loaded = NavigationGrid.load(path)
            np.testing.assert_array_equal(loaded.cells, grid.cells)
            np.testing.assert_array_equal(loaded.blocked, grid.blocked)
            self.assertEqual(loaded.world_to_cell([-1.5, 1.7]), (3, 2))
            np.testing.assert_allclose(loaded.cell_to_world((3, 2)), [-1.5, 1.7])
            with self.assertRaises(FileExistsError):
                grid.save(path)
            np.savez(path, cells=[[2]], origin=[0, 0], resolution=.1, clearance_m=.25)
            with self.assertRaises(ValueError):
                NavigationGrid.load(path)

    def test_constructor_rejects_invalid_geometry_and_state(self):
        for cells, origin, resolution, clearance in [([[9]], [0, 0], .1, .25),
                ([[0]], [0, np.nan], .1, .25), ([[0]], [0, 0], 0., .25),
                ([[0]], [0, 0], .1, -.1), ([], [0, 0], .1, .25),
                ([[.5]], [0, 0], .1, .25)]:
            with self.assertRaises(ValueError):
                NavigationGrid(cells, origin, resolution, clearance)


def write_session(path, depths, poses=None):
    frames = []
    for i, depth in enumerate(depths):
        pose = np.eye(4)
        pose[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        pose[2, 3] = .3
        if poses is not None:
            pose = poses[i]
        np.savez(path/f'{i}.npz', depth=depth, depth_source='deltar_tof_fusion')
        frames.append(dict(file=f'{i}.npz', camera_to_world=pose.tolist()))
    (path/'live_sequence.json').write_text(json.dumps(dict(intrinsics=[80, 80, 79.5, 59.5], frames=frames)))


def room_depth():
    y, x = np.mgrid[:120, :160]
    with np.errstate(divide='ignore'):
        floor = .3/((y-59.5)/80)
    floor[floor <= 0] = np.inf
    return np.rint(np.minimum(floor, 2.)*1000).astype(np.uint16)


class BuildTests(unittest.TestCase):
    def test_measured_drop_cannot_be_cleared_by_older_floor_views(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            write_session(path, [room_depth()]*3)
            for i in range(3):
                depth = room_depth()
                labels = np.where(np.indices(depth.shape)[0]>60, 1, 2).astype(np.uint8)
                if i==2:
                    rows = np.indices(depth.shape)[0]
                    floor = rows>59.5
                    depth[floor] = np.minimum(.5/((rows[floor]-59.5)/80), 2.)*1000
                    labels.fill(0)
                np.savez(path/f'{i}.npz', depth=depth, depth_source='deltar_tof_fusion',
                         ground_labels=labels, ground_source=SOURCE, ground_confirmations=3,
                         ground_plane_camera=[0., -1., 0., .3])
            grid = build_grid(path, resolution=.1, clearance_m=0.)
            self.assertEqual(grid.cells[grid.world_to_cell([1.05, .05])], UNKNOWN)

    def test_labels_remove_only_confirmed_ground_and_uncertain_endpoints_block(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            write_session(path, [room_depth()]*2)
            for i in range(2):
                depth = room_depth()
                # Biased endpoints at z=3cm were obstacles in the old height cut.
                depth[80:84, 78:82] = 1000
                labels = np.where(np.indices(depth.shape)[0]>60, 1, 2).astype(np.uint8)
                # This unit fixture supplies classification; geometric tests cover fitting.
                labels[80:84, 78:82] = 1
                np.savez(path/f'{i}.npz', depth=depth, depth_source='deltar_tof_fusion',
                         ground_labels=labels, ground_source=SOURCE, ground_confirmations=3,
                         ground_plane_camera=[0., -1., 0., .3])
            grid = build_grid(path, resolution=.1, clearance_m=0.)
            self.assertNotEqual(grid.cells[grid.world_to_cell([1.01, .01])], OCCUPIED)
            with np.load(path/'1.npz') as sample:
                values = dict(sample)
            values['ground_labels'][80:84, 78:82] = 0
            np.savez(path/'1.npz', **values)
            grid = build_grid(path, resolution=.1, clearance_m=0.)
            self.assertEqual(grid.cells[grid.world_to_cell([1.01, .01])], UNKNOWN)

    def test_floor_and_whole_height_evidence_required_and_wall_not_cleared(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            write_session(path, [room_depth()]*2)
            grid = build_grid(path, resolution=.1, clearance_m=0.)
            self.assertEqual(grid.cells[grid.world_to_cell([1.05, .05])], FREE)
            self.assertEqual(grid.cells[grid.world_to_cell([2.01, .05])], OCCUPIED)
            # Very near the camera, floor is outside the field of view.
            self.assertEqual(grid.cells[grid.world_to_cell([.15, .05])], UNKNOWN)
            for invalid in (0, 10001, 65535):
                depths = [np.where(np.indices(d.shape)[0] < 60, invalid, d).astype(np.uint16)
                          for d in [room_depth()]*2]
                write_session(path, depths)
                grid = build_grid(path, resolution=.1, clearance_m=0.)
                self.assertEqual(grid.cells[grid.world_to_cell([1.05, .05])], UNKNOWN)

    def test_low_and_overhanging_obstacle_hits_win_over_clear_rays(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for rows in (slice(79, 82), slice(54, 57)):
                depth = room_depth()
                depth[rows, 78:82] = 1000  # Separate low (.05m) and overhanging (.35m) hits.
                write_session(path, [room_depth(), room_depth(), depth])
                grid = build_grid(path, resolution=.1, clearance_m=0.)
                self.assertEqual(grid.cells[grid.world_to_cell([1.01, .01])], OCCUPIED)

    def test_bad_provenance_no_returns_and_bad_transform_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            write_session(path, [room_depth()])
            np.savez(path/'0.npz', depth=room_depth())
            with self.assertRaises(ValueError):
                build_grid(path)
            write_session(path, [np.zeros((120, 160), np.uint16)])
            with self.assertRaises(ValueError):
                build_grid(path)
            write_session(path, [room_depth()], [np.zeros((4, 4))])
            with self.assertRaises(ValueError):
                build_grid(path)


if __name__ == '__main__':
    unittest.main()
