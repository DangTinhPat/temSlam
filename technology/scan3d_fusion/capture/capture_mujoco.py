#!/usr/bin/env python3
"""Create DELTAR-format synthetic RGB/ToF captures from the MuJoCo house."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import tempfile

import h5py
import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = ROOT / "datasets/processed/scan3d_fusion/rb_sim_input"
POSES = (
    ("east", 4.0, 0.0, 0.14, 0.0),
    ("north", 0.0, 3.0, 0.14, math.pi / 2),
    ("west", -4.0, 0.0, 0.14, math.pi),
    ("south", 0.0, -3.0, 0.14, -math.pi / 2),
)


def set_pose(data, x, y, z, yaw):
    data.qpos[:7] = [x, y, z, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def set_handheld_pose(data, camera_offset, camera_rotation, position, target):
    """Move the complete rigid RGB/ToF rig so its optical center looks at target."""
    position, target = np.asarray(position, float), np.asarray(target, float)
    forward = target-position
    length = np.linalg.norm(forward)
    if not np.isfinite([position, target]).all() or length < 1e-6:
        raise ValueError("invalid handheld camera position/target")
    forward /= length
    up = np.array([0., 0., 1.]) if abs(forward[2]) < .99 else np.array([0., 1., 0.])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    desired = np.column_stack((right, np.cross(right, forward), -forward))
    rotation = desired @ camera_rotation.T
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    data.qpos[:3] = position-rotation@camera_offset
    data.qpos[3:7] = quaternion


def zone_statistics(depth, sensor, rng):
    """Approximate a zone return with many rays, in ToF optical-Z metres."""
    if depth.shape != (128, 128):
        raise ValueError("ToF sampling image must be 128 x 128")
    hist = np.zeros((64, 2), np.float32)
    valid = np.zeros(64, bool)
    for row in range(8):
        for col in range(8):
            values = depth[row*16:(row+1)*16, col*16:(col+1)*16].ravel()
            values = values[np.isfinite(values) & (values >= sensor["min_range_m"]) &
                            (values <= sensor["max_range_m"])]
            if len(values) < 32:
                continue
            # shortcut: equal-area rays approximate a return distribution; no photon/reflectance model.
            counts, edges = np.histogram(values, bins=np.arange(0, 4.04, .04))
            occupied = np.flatnonzero(counts >= max(3, counts.max() * .1))
            if not len(occupied):
                continue
            clusters = np.split(occupied, np.flatnonzero(np.diff(occupied) > 1) + 1)
            # The RB sensor is configured for the closest significant target.
            cluster = next((c for c in clusters if counts[c].sum() >= len(values)*.2), None)
            if cluster is None:
                continue
            selected = values[(values >= edges[cluster[0]]) & (values <= edges[cluster[-1]+1])]
            mean = float(selected.mean())
            noise = sensor["noise"]
            bound = float(noise["near_accuracy_m"]) if mean <= .2 else mean*float(noise["far_accuracy_fraction"])
            sigma_noise = bound * float(noise["sigma_fraction_of_bound"])
            index = row*8+col
            hist[index] = [np.clip(mean+rng.normal(0, sigma_noise), .02, 4),
                           max(.005, math.hypot(float(selected.std()), sigma_noise))]
            valid[index] = True
    return hist, valid


def zone_rectangles(hist, rotation, translation, fovy, width, height, tof_fovy):
    """Project measured ToF zone corners through calibration, without RGB ground-truth depth."""
    if width != int(width) or height != int(height) or min(width, height) < 16:
        raise ValueError("image width/height must be pixel dimensions, not camera height in metres")
    rect = np.zeros((64, 4), np.float32)
    f = height / (2*math.tan(math.radians(fovy)/2))
    h = math.tan(math.radians(tof_fovy)/2)
    for i in range(64):
        row, col = divmod(i, 8)
        z = max(.1, float(hist[i, 0]))
        corners = np.array([[(2*c/8-1)*h*z, (1-2*r/8)*h*z, -z]
                            for r in (row, row+1) for c in (col, col+1)])
        camera = (corners - translation) @ rotation
        if np.any(camera[:, 2] >= 0):
            raise ValueError("zone projected behind camera; check extrinsic calibration")
        x = camera[:, 0] / -camera[:, 2]*f + width/2
        y = -camera[:, 1] / -camera[:, 2]*f + height/2
        rect[i] = [math.floor(y.min()), math.floor(x.min()),
                   math.ceil(y.max()), math.ceil(x.max())]
    return rect


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--scan", choices=("cardinal", "sofa", "ball", "sofa-handheld"), default="cardinal")
    parser.add_argument("--frames", type=int, default=72)
    parser.add_argument("--samples-per-pose", type=int, default=3)
    args = parser.parse_args()
    if args.frames < 8:
        parser.error("--frames must be at least 8")
    if args.scan == "sofa-handheld" and args.frames < 72:
        parser.error("--scan sofa-handheld needs at least 72 frames across its three passes")
    if args.samples_per_pose < 1:
        parser.error("--samples-per-pose must be positive")

    model_path = ROOT / "simulation/scenarios/indoor_tiny_house.xml"
    # Mesh `inertia="shell"` is a newer MuJoCo attribute than the pinned DELTAR
    # renderer wheel; it affects inertial compilation, not this static capture.
    xml_text = re.sub(r'\s+inertia="[^"]+"', "", model_path.read_text())
    with tempfile.NamedTemporaryFile("w", suffix=".xml", dir=model_path.parent,
                                     delete=False) as xml_file:
        xml_file.write(xml_text)
        compat_xml = Path(xml_file.name)
    try:
        model = mujoco.MjModel.from_xml_path(str(compat_xml))
    finally:
        compat_xml.unlink(missing_ok=True)
    data = mujoco.MjData(model)
    sensor = yaml.safe_load((ROOT / "technology/scan3d_fusion/config/tof.yaml").read_text())["sensor"]
    camera_config = yaml.safe_load((ROOT / "technology/scan3d_fusion/config/camera.yaml").read_text())["camera"]
    width, height = map(int, camera_config["resolution_px"])
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    tof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "tof_camera")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body_link")
    sofa_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "env_mesh_sofa")
    if min(camera_id, tof_id, body_id, sofa_id) < 0:
        raise RuntimeError("model must contain body_link, head_camera, tof_camera and env_mesh_sofa")
    set_pose(data, 0, 0, 0, 0)
    mujoco.mj_forward(model, data)
    sofa_center = data.geom_xpos[sofa_id].copy()
    camera_offset = data.cam_xpos[camera_id].copy()
    camera_rotation_local = data.cam_xmat[camera_id].reshape(3, 3).copy()

    # Keep the robot out of its own range measurements as the simulator mapper does.
    original_groups = model.geom_group.copy()
    for geom_id in range(model.ngeom):
        parent = int(model.geom_bodyid[geom_id])
        while parent > 0 and parent != body_id:
            parent = int(model.body_parentid[parent])
        if parent == body_id:
            model.geom_group[geom_id] = 5

    calibration_path = ROOT / "technology/scan3d_fusion/config/sim_calibration.json"
    calibration = json.loads(calibration_path.read_text())["camera_to_tof"]
    cam_to_tof_R = np.asarray(calibration["rotation_matrix"], dtype=np.float64)
    cam_to_tof_t = np.asarray(calibration["translation_m"], dtype=np.float64)
    rng = np.random.default_rng(args.seed)
    args.output.mkdir(parents=True, exist_ok=True)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = camera_id
    render_options = mujoco.MjvOption()
    render_options.geomgroup[3] = 1
    render_options.geomgroup[5] = 0
    model.light_ambient[:] = .3
    renderer = mujoco.Renderer(model, height=height, width=width)
    tof_renderer = mujoco.Renderer(model, height=128, width=128)
    manifest = {"train": [], "test": []}
    f = height / (2*math.tan(math.radians(float(model.cam_fovy[camera_id]))/2))
    sequence = {"pose_source": "simulator ground truth; static kinematic capture, not SLAM",
                "scan": args.scan,
                "sensor_model": "256 area rays per zone, closest significant cluster, optical-Z mean/std",
                "intrinsics": [f, f, (width-1)/2, (height-1)/2], "image_size": [width, height],
                "frames": []}
    poses = POSES
    if args.scan in ("sofa", "ball"):
        center_x, center_y, radius = (*sofa_center[:2], 1.5) if args.scan == "sofa" else (-.65, -3.55, .8)
        poses = []
        for i in range(args.frames):
            angle = 2*math.pi*i/args.frames
            x, y = center_x+radius*math.cos(angle), center_y+radius*math.sin(angle)
            poses.append((f"{args.scan}_{i:04d}", x, y, .219775157, math.atan2(center_y-y, center_x-x)))
        sequence["bounds_m"] = ([[center_x-1.3, center_y-1.35, .025],
                                  [center_x+1.3, center_y+1.35, 1.15]] if args.scan == "sofa"
                                else [[-.9, -3.8, .02], [-.4, -3.3, .25]])
    elif args.scan == "sofa-handheld":
        poses = []
        center_x, center_y = sofa_center[:2]
        for i in range(args.frames):
            phase = 3*i/args.frames
            angle = 2*math.pi*phase
            optical_height = np.interp(phase, [0, .9, 1.1, 1.9, 2.1, 3], [.5, .5, 1.1, 1.1, 1.8, 1.8])
            # Entries are camera optical centers; the rigid rig is positioned below.
            poses.append((f"handheld_{i:04d}", center_x+1.5*math.cos(angle),
                          center_y+1.5*math.sin(angle), float(optical_height), 0.))
        sequence["bounds_m"] = [[center_x-1.3, center_y-1.35, .025],
                                 [center_x+1.3, center_y+1.35, 1.15]]
        sequence["trajectory"] = {"type": "three handheld orbits with vertical transitions",
            "camera_heights_m": [.5, 1.1, 1.8], "look_at_m": [center_x, center_y, .5],
            "radius_m": 1.5, "robot_control": False}
    count = 0
    try:
        for pose_index, (name, x, y, z, yaw) in enumerate(poses):
            if args.scan == "sofa-handheld":
                set_handheld_pose(data, camera_offset, camera_rotation_local,
                                  [x, y, z], [center_x, center_y, .5])
            else:
                set_pose(data, x, y, z, yaw)
            mujoco.mj_forward(model, data)
            camera_rotation = data.cam_xmat[camera_id].reshape(3, 3).copy()
            renderer.update_scene(data, camera=camera, scene_option=render_options)
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera, scene_option=render_options)
            depth = renderer.render().copy().astype(np.float32)
            renderer.disable_depth_rendering()

            depth[~np.isfinite(depth) | (depth < .02) | (depth > 4)] = 0
            tof_renderer.enable_depth_rendering()
            tof_renderer.update_scene(data, camera="tof_camera", scene_option=render_options)
            tof_depth = tof_renderer.render().copy()
            camera_to_world = np.eye(4)
            # OpenGL camera (right/up/back) -> CV camera (right/down/forward).
            camera_to_world[:3, :3] = camera_rotation @ np.diag([1., -1., -1.])
            camera_to_world[:3, 3] = data.cam_xpos[camera_id]
            body_to_world = np.eye(4)
            body_to_world[:3, :3] = data.xmat[body_id].reshape(3, 3)
            body_to_world[:3, 3] = data.xpos[body_id]
            body_from_camera = np.linalg.inv(body_to_world) @ camera_to_world
            for sample_index in range(args.samples_per_pose):
                sample_hist, mask = zone_statistics(tof_depth, sensor, rng)
                rect_data = zone_rectangles(sample_hist, cam_to_tof_R, cam_to_tof_t,
                                            float(model.cam_fovy[camera_id]), width, height,
                                            float(sensor["fovy_deg"]))
                filename = f"{name}_{sample_index:02d}.h5"
                with h5py.File(args.output / filename, "w") as sample:
                    sample.create_dataset("rgb", data=rgb, compression="gzip", compression_opts=1)
                    sample.create_dataset("depth", data=depth, compression="gzip", compression_opts=1)
                    sample.create_dataset("hist_data", data=sample_hist)
                    sample.create_dataset("fr", data=rect_data)
                    sample.create_dataset("mask", data=mask)
                manifest["test"].append({"filename": filename})
                sequence["frames"].append({"file": filename,
                    "time_s": pose_index*8/15 + sample_index/15,
                    "body_from_camera": body_from_camera.tolist(),
                    "camera_to_world": camera_to_world.tolist()})
                count += 1
            if pose_index % 12 == 0:
                print(f"capture {pose_index+1}/{len(poses)}", flush=True)
    finally:
        renderer.close()
        tof_renderer.close()
        model.geom_group[:] = original_groups

    (args.output / "data.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "sequence.json").write_text(json.dumps(sequence, indent=2) + "\n")
    print(f"wrote {count} simulator captures to {args.output}")
    print("camera image and ToF range are model inputs; depth is retained only for offline evaluation")


if __name__ == "__main__":
    main()
