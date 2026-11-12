"""Frame conversion and independent trajectory evaluation regressions."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from evaluate import metrics
from track_rgbd import body_pose, pose_matrix, MAP_FROM_START


class AdapterTests(unittest.TestCase):
    def test_optical_axes(self):
        class Pose:
            rotation = [0, 0, 0, 1]
            translation = [0, 0, 2]
        np.testing.assert_allclose(pose_matrix(Pose)[:3, 3], [2, 0, 0])
        np.testing.assert_allclose(MAP_FROM_START[:3, :3].T @ MAP_FROM_START[:3, :3], np.eye(3))

    def test_stationary_body_with_turning_head_and_lever_arm(self):
        world_body = np.eye(4)
        world_body[:3, 3] = [1, 2, .3]
        for yaw in [-180, -40, 0, 90, 180]:
            mount = np.eye(4)
            mount[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
            mount[:3, 3] = mount[:3, :3] @ [.1, .02, .09]
            np.testing.assert_allclose(body_pose(world_body @ mount, mount), world_body, atol=1e-12)
        with self.assertRaises(ValueError):
            body_pose(world_body, np.zeros((4, 4)))

    def test_rigid_alignment_preserves_scale_error(self):
        truth = np.tile(np.eye(4), (5, 1, 1))
        truth[:, :3, 3] = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, .3], [0, 0, .5]]
        offset = np.eye(4)
        offset[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .7]).as_matrix()
        offset[:3, 3] = [5, 2, 1]
        report, _ = metrics(offset @ truth, truth)
        self.assertLess(report['ate_se3_rmse_m'], 1e-12)
        self.assertLess(report['rpe_adjacent_translation_rmse_m'], 1e-12)
        scaled = truth.copy()
        scaled[:, :3, 3] *= 2
        report, _ = metrics(scaled, truth)
        self.assertGreater(report['ate_se3_rmse_m'], .5)


if __name__ == '__main__':
    unittest.main()
