"""Continuous wheel/IMU odometry with a separate visual map correction; no ROS."""
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(t[:3, :3]) < .999):
        raise ValueError('Expected a finite rigid transform')
    return t


def odom_body(packet):
    fields = ['time_s', 'x', 'y', 'yaw', 'pitch', 'roll', 'axle_body_x',
              'axle_body_z', 'wheel_radius']
    if (not np.isfinite([packet[k] for k in fields]).all() or packet['wheel_radius'] <= 0
            or packet['valid'] not in (0, 1)):
        raise ValueError('Invalid wheel/IMU packet')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('ZYX', [packet['yaw'], packet['pitch'], packet['roll']]).as_matrix()
    transform[:3, 3] = ([packet['x'], packet['y'], packet['wheel_radius']]
                        - transform[:3, :3] @ [packet['axle_body_x'], 0, packet['axle_body_z']])
    return transform


class BodyLocalization:
    def __init__(self, level_floor=False):
        self.map_from_odom = None
        self.last_timestamp = -1
        self.level_floor = level_floor
        self.gravity_alignment = None

    def update(self, timestamp_ns, packet, body_from_camera, map_from_camera=None):
        if (not isinstance(timestamp_ns, (int, np.integer)) or timestamp_ns < 0
                or timestamp_ns <= self.last_timestamp
                or abs(timestamp_ns*1e-9-packet['time_s']) > .02):
            raise ValueError('Require increasing camera timestamps and odometry within 20 ms')
        mount = rigid(body_from_camera)
        local = odom_body(packet)
        self.last_timestamp = timestamp_ns
        if not packet['valid']:
            return None
        visual_accepted = False
        if map_from_camera is not None:
            visual_body = rigid(map_from_camera) @ np.linalg.inv(mount)
            if self.level_floor:
                if self.gravity_alignment is None:
                    self.gravity_alignment = local @ np.linalg.inv(visual_body)
                visual_body = self.gravity_alignment @ visual_body
            candidate = visual_body @ np.linalg.inv(local)
            if self.map_from_odom is None:
                visual_accepted = True
            else:
                residual = np.linalg.inv(self.map_from_odom @ local) @ visual_body
                # shortcut: conservative jump gate, tune with synchronized real RGB-D/encoder/IMU logs.
                visual_accepted = (np.linalg.norm(residual[:3, 3]) < .5
                                   and Rotation.from_matrix(residual[:3, :3]).magnitude() < .35)
            if visual_accepted:
                if self.level_floor:
                    yaw = np.arctan2(candidate[1, 0], candidate[0, 0])
                    candidate[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
                    candidate[2, 3] = 0.
                self.map_from_odom = candidate
        mapped = local if self.map_from_odom is None else self.map_from_odom @ local
        return {'timestamp_ns': timestamp_ns, 'frame': 'odom' if self.map_from_odom is None else 'map',
                'odom_from_body': local.tolist(), 'world_from_body': mapped.tolist(),
                'world_from_camera': (mapped @ mount).tolist(),
                'visual_accepted': bool(visual_accepted), 'slip': bool(packet['slip']),
                'pose_source': 'wheel/IMU local odometry + external cuVSLAM map correction'}
