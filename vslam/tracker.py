"""Core cuVSLAM adapter shared by simulation and the fixed camera demo."""
import importlib.metadata

import numpy as np
from scipy.spatial.transform import Rotation

from .localization import rigid

SLAM_MAP_SOURCE = 'cuVSLAM accepted poses with verified RGB-D recovery and graph corrections'

# OpenCV camera axes (right/down/forward) to initial horizontal map axes.
MAP_FROM_START = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                           [0., -1., 0., 0.], [0., 0., 0., 1.]])


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose.rotation).as_matrix()
    transform[:3, 3] = pose.translation
    if not np.isfinite(transform).all():
        raise ValueError('cuVSLAM returned a non-finite pose')
    return MAP_FROM_START @ transform


def optimized_map_poses(optimized, timestamps, map_from_slam):
    """Use the same map frame for every snapshot of the SDK pose graph."""
    if (not timestamps or any(b <= a for a, b in zip(timestamps, timestamps[1:]))
            or any(t not in optimized for t in timestamps)):
        raise ValueError('Pose graph is missing one or more retained keyframes')
    alignment = rigid(map_from_slam)
    return [alignment @ rigid(optimized[t]) for t in timestamps]


def local_motion_error(previous_visual, current_visual, previous_sensor, current_sensor):
    """Compare increments in the two local frames, independently of graph updates."""
    visual_delta = np.linalg.solve(previous_visual, current_visual)
    sensor_delta = np.linalg.solve(previous_sensor, current_sensor)
    residual = np.linalg.solve(sensor_delta, visual_delta)
    return (float(np.linalg.norm(residual[:3, 3])),
            float(Rotation.from_matrix(residual[:3, :3]).magnitude()))


def _graph_errors(previous, proposed):
    previous, proposed = np.asarray(previous), np.asarray(proposed)
    origin_error = np.linalg.solve(previous[:1], proposed[:1])
    old_edges = np.linalg.solve(previous[:-1], previous[1:])
    new_edges = np.linalg.solve(proposed[:-1], proposed[1:])
    errors = np.concatenate((origin_error, np.linalg.solve(old_edges, new_edges)))
    distance = np.linalg.norm(errors[:, :3, 3], axis=1)
    angle = Rotation.from_matrix(errors[:, :3, :3]).magnitude()
    # shortcut: map commit limits for the simulated room; calibrate on longer, verified loops.
    return distance, angle, (distance <= .10) & (angle <= .15)


def graph_is_consistent(previous, proposed):
    """Allow distributed corrections while rejecting jumps at the origin or between views."""
    return bool(_graph_errors(previous, proposed)[2].all())


def graph_diagnostics(previous, proposed, timestamps):
    distance, angle, valid = _graph_errors(previous, proposed)

    def residual(i):
        return dict(from_ns=int(timestamps[max(0, i - 1)]), to_ns=int(timestamps[i]),
                    distance_m=float(distance[i]), angle_rad=float(angle[i]))

    return dict(consistent=bool(valid.all()), origin=residual(0),
                violations=[residual(i) for i in np.flatnonzero(~valid)])


class RgbdSlam:
    """Reusable image/depth interface; no simulator, ground-truth or ROS inputs."""
    def __init__(self, image_size, intrinsics, *, use_motion_model=True):
        import cuvslam
        version = importlib.metadata.version('cuvslam')
        if version.split('+')[0] != '17.0.1':
            raise RuntimeError(f'This adapter targets cuVSLAM 17.0.1, found {version}')
        self.size = tuple(map(int, image_size))
        self.last_timestamp = -1
        self.last_odometry_pose = None
        camera = cuvslam.Camera()
        camera.size = self.size
        camera.focal = list(intrinsics[:2])
        camera.principal = list(intrinsics[2:])
        if len(intrinsics) != 4 or not np.isfinite(intrinsics).all() or min(intrinsics[:2]) <= 0:
            raise ValueError('Invalid pinhole intrinsics')
        settings = cuvslam.Tracker.OdometryRGBDSettings()
        settings.depth_camera_id = 0
        settings.depth_scale_factor = 1000.
        odom = cuvslam.Tracker.OdometryConfig(
            use_gpu=True, async_sba=False, use_motion_model=use_motion_model,
            odometry_mode=cuvslam.Tracker.OdometryMode.RGBD, rgbd_settings=settings)
        slam = cuvslam.Tracker.SlamConfig(
            use_gpu=True, sync_mode=True, enable_reading_internals=True,
            planar_constraints=False, gt_align_mode=False, max_map_size=0)
        self.tracker = cuvslam.Tracker(cuvslam.Rig([camera]), odom, slam)

    def track(self, timestamp_ns, rgb, depth_mm):
        width, height = self.size
        if (timestamp_ns <= self.last_timestamp or rgb.shape != (height, width, 3)
                or rgb.dtype != np.uint8 or depth_mm.shape != (height, width)
                or depth_mm.dtype != np.uint16):
            raise ValueError('Require increasing timestamps, RGB uint8 and aligned depth uint16 mm')
        if not np.any(depth_mm):
            self.last_odometry_pose = None
            raise ValueError('Require nonzero measured depth; stop mapping')
        self.last_timestamp = timestamp_ns
        estimate, slam = self.tracker.track(
            timestamp_ns, images=[np.ascontiguousarray(rgb)], depths=[np.ascontiguousarray(depth_mm)])
        if estimate.world_from_rig is None or slam is None:
            self.last_odometry_pose = None
            return None
        self.last_odometry_pose = pose_matrix(estimate.world_from_rig.pose)
        return pose_matrix(slam)

    def localize(self, folder, timestamp_ns, rgb, guess):
        import cuvslam
        native = np.linalg.solve(MAP_FROM_START, guess)
        hint = cuvslam.Pose(Rotation.from_matrix(native[:3, :3]).as_quat(), native[:3, 3])
        result = []
        self.tracker.localize_in_map(
            str(folder), timestamp_ns, hint, [np.ascontiguousarray(rgb[..., ::-1])],
            cuvslam.Tracker.SlamLocalizationSettings(horizontal_search_radius=.4, vertical_search_radius=.12,
                horizontal_step=.1, vertical_step=.06, angular_step_rads=.15), lambda: None,
            lambda pose, error: result.append(pose))
        return pose_matrix(result[0]) if result and result[0] is not None else None
