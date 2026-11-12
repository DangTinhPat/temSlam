"""Geometry regressions for projection, missing data and surface extraction."""
import unittest
from pathlib import Path
import sys

import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "capture"))
from capture_mujoco import zone_rectangles, zone_statistics, set_handheld_pose
from reconstruct import TSDF, tof_support


class GeometryTests(unittest.TestCase):
    def test_handheld_motion_preserves_camera_tof_mount(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body>
          <freejoint/><geom size=".01" mass="1"/>
          <camera name="rgb" pos=".0979 0 .0918" xyaxes="0 -1 0 0 0 1"/>
          <camera name="tof" pos=".089 .02 .0918" xyaxes="-.069756 -.997564 0 0 0 1"/>
        </body></worldbody></mujoco>''')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        R0 = data.cam_xmat[0].reshape(3, 3).copy()
        offset = data.cam_xpos[0].copy()
        relative = R0.T@(data.cam_xpos[1]-offset)
        relative_R = R0.T@data.cam_xmat[1].reshape(3, 3)
        target = np.array([-2.3, -2.3, .5])
        for position in ([-.65, -2.3, .5], [-3.2, -3.1, 1.8], [-2.3, -2.3, 2.]):
            set_handheld_pose(data, offset, R0, position, target)
            mujoco.mj_forward(model, data)
            R = data.cam_xmat[0].reshape(3, 3)
            np.testing.assert_allclose(data.cam_xpos[0], position, atol=1e-10)
            direction = target-position
            np.testing.assert_allclose(-R[:, 2], direction/np.linalg.norm(direction), atol=1e-10)
            np.testing.assert_allclose(R.T@(data.cam_xpos[1]-data.cam_xpos[0]), relative, atol=1e-10)
            np.testing.assert_allclose(R.T@data.cam_xmat[1].reshape(3, 3), relative_R, atol=1e-10)

    def test_calibrated_projection_without_truth_depth(self):
        hist = np.tile([1., .01], (64, 1))
        with self.assertRaises(ValueError):
            zone_rectangles(hist, np.eye(3), np.zeros(3), 45, 640, 1.8, 45)
        rect = zone_rectangles(hist, np.eye(3), np.zeros(3), 45, 640, 480, 45)
        np.testing.assert_allclose(rect[0], [0, 80, 60, 140], atol=1)
        np.testing.assert_allclose(rect[-1], [420, 500, 480, 560], atol=1)
        f = 480/(2*np.tan(np.deg2rad(22.5)))
        calibration = {"rotation_matrix": np.eye(3), "translation_m": np.zeros(3)}
        supported = tof_support(np.ones((480, 640)), hist, rect, np.ones(64, bool),
                                [f, f, 319.5, 239.5], calibration)
        self.assertEqual(supported[240, 320], 1)
        self.assertEqual(supported[240, 0], 0)
        rejected = tof_support(np.full((480, 640), 2), hist, rect, np.ones(64, bool),
                               [f, f, 319.5, 239.5], calibration)
        self.assertFalse(rejected.any())
        anchored = tof_support(np.full((480, 640), 1.08, np.float32), hist, rect,
                               np.ones(64, bool), [f, f, 319.5, 239.5], calibration)
        self.assertAlmostEqual(float(anchored[240, 320]), 1., places=5)
        empty = np.zeros((480, 640), np.float32)
        empty[:20] = np.nan
        rejected = tof_support(empty, hist, rect, np.ones(64, bool),
                               [f, f, 319.5, 239.5], calibration)
        self.assertTrue(np.isfinite(rejected).all())
        self.assertFalse(rejected.any())

    def test_two_returns_choose_closest_cluster(self):
        depth = np.ones((128, 128), np.float32)
        depth[:, 8::16] = 2
        sensor = {"min_range_m": .02, "max_range_m": 4,
                  "noise": {"near_accuracy_m": 0, "far_accuracy_fraction": 0,
                            "sigma_fraction_of_bound": 0}}
        hist, mask = zone_statistics(depth, sensor, np.random.default_rng(1))
        self.assertTrue(mask.all())
        np.testing.assert_allclose(hist[:, 0], 1)
        self.assertTrue(np.all(hist[:, 1] > 0))

    def test_tsdf_metric_plane_and_invalid_returns(self):
        volume = TSDF([[-.2, -.2, .8], [.2, .2, 1.2]], voxel=.02, device="cpu")
        K = [50, 50, 31.5, 23.5]
        volume.integrate(np.full((48, 64), np.nan), np.eye(4), K)
        volume.integrate(np.zeros((48, 64)), np.eye(4), K)
        self.assertEqual(volume.weights.sum().item(), 0)
        for _ in range(6):
            volume.integrate(np.ones((48, 64)), np.eye(4), K)
        mesh = volume.mesh()
        self.assertGreater(len(mesh.faces), 100)
        self.assertLess(np.max(np.abs(mesh.vertices[:, 2]-1)), .01)
        pose = np.eye(4)
        pose[0, 0] = -1
        with self.assertRaises(ValueError):
            volume.integrate(np.ones((48, 64)), pose, K)


if __name__ == "__main__":
    unittest.main()
