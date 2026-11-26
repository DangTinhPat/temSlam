import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from vslam.localization import floor_camera_pose


def transform(angles, translation):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('ZYX', angles, degrees=True).as_matrix()
    pose[:3, 3] = translation
    return pose


class FloorCameraPoseTests(unittest.TestCase):
    def test_preserves_visual_body_xy_yaw_with_an_offset_tilted_camera(self):
        visual = transform([179, 8, -15], [2, 3, .64])
        measured = transform([-173, -3, 4], [12, -8, .145])
        mount = transform([88, 12, 91], [.10, .02, .09])
        expected = transform([179, -3, 4], [2, 3, .145]) @ mount
        np.testing.assert_allclose(floor_camera_pose(visual @ mount, measured, mount), expected, atol=1e-12)
