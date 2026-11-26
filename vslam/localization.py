"""Wheel/IMU body pose for initial alignment and motion diagnostics."""
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


def floor_camera_pose(visual_camera, measured_body, body_from_camera):
    """Keep visual body XY and heading; use the IMU tilt and wheel/leg height on a level floor."""
    body = visual_camera @ np.linalg.inv(body_from_camera)
    measured = np.asarray(measured_body)
    yaw = np.arctan2(body[1, 0], body[0, 0])
    measured_yaw = np.arctan2(measured[1, 0], measured[0, 0])
    body[:3, :3] = Rotation.from_euler('z', yaw - measured_yaw).as_matrix() @ measured[:3, :3]
    body[2, 3] = measured[2, 3]
    return body @ body_from_camera
