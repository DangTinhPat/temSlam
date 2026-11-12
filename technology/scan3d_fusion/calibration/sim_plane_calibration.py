#!/usr/bin/env python3
"""Estimate the camera-to-ToF rigid transform from synthetic MuJoCo plane views."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[3]
POSES = (
    ("east_wall_front", "east_wall", 4.0, 0.0, 0.14, 0.0, 0.0),
    ("east_wall_oblique_left", "east_wall", 4.0, 0.0, 0.14, -0.30, 0.0),
    ("east_wall_oblique_right", "east_wall", 4.0, 0.0, 0.14, 0.30, 0.0),
    ("west_wall", "west_wall", -4.0, 0.0, 0.14, math.pi, 0.0),
    ("north_wall_front", "north_wall", 0.0, 3.0, 0.14, math.pi / 2, 0.0),
    ("north_wall_oblique", "north_wall", 0.0, 3.0, 0.14, math.pi / 2 + 0.30, 0.0),
    ("south_wall_west", "south_wall_west", 0.0, -3.0, 0.14, -math.pi / 2, 0.0),
    ("ground", "ground", 0.0, 0.0, 0.35, 0.0, 0.72),
)


def raycast(model, data, camera_id, body_id, width, height, fovy_deg, groups):
    rotation = data.cam_xmat[camera_id].reshape(3, 3)
    origin = data.cam_xpos[camera_id].copy()
    tan_y = math.tan(math.radians(fovy_deg) / 2)
    tan_x = tan_y * width / height
    points, geom_ids = [], []
    for row in range(height):
        y = (1 - 2 * (row + 0.5) / height) * tan_y
        for col in range(width):
            x = (2 * (col + 0.5) / width - 1) * tan_x
            direction_camera = np.array([x, y, -1.0])
            direction_camera /= np.linalg.norm(direction_camera)
            direction_world = rotation @ direction_camera
            geom_id = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(
                model, data, origin, direction_world, groups, 1, body_id, geom_id
            )
            if distance > 0:
                points.append(direction_camera * distance)
                geom_ids.append(int(geom_id[0]))
    return np.asarray(points), np.asarray(geom_ids, dtype=np.int32)


def pose_quaternion(yaw, pitch):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    # q_world_body = q_z(yaw) * q_y(pitch)
    return np.array([cy * cp, -sy * sp, cy * sp, sy * cp])


def fit_plane(points):
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    offset = -normal @ center
    return np.r_[normal, offset]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=ROOT / "simulation/scenarios/indoor_tiny_house.xml")
    parser.add_argument("--output", type=Path, default=ROOT / "datasets/processed/sim_rgb_tof_calibration")
    parser.add_argument("--noise-scale", type=float, default=1.0)
    parser.add_argument("--samples-per-pose", type=int, default=6)
    parser.add_argument("--seed", type=int, default=21)
    args = parser.parse_args()
    if args.noise_scale < 0 or args.samples_per_pose < 1:
        parser.error("noise scale must be non-negative and samples per pose must be positive")

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body_link")
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    tof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "tof_camera")
    if min(body_id, camera_id, tof_id) < 0:
        raise RuntimeError("model must contain body_link, head_camera and tof_camera")

    sensor = yaml.safe_load((ROOT / "technology/scan3d_fusion/config/tof.yaml").read_text())["sensor"]
    fovy = float(sensor["fovy_deg"])
    min_range, max_range = float(sensor["min_range_m"]), float(sensor["max_range_m"])
    noise = sensor["noise"]
    rng = np.random.default_rng(args.seed)
    original_groups = model.geom_group.copy()
    for geom_id in range(model.ngeom):
        parent = int(model.geom_bodyid[geom_id])
        while parent > 0 and parent != body_id:
            parent = int(model.body_parentid[parent])
        if parent == body_id:
            model.geom_group[geom_id] = 5
    groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)

    observations = []
    ground_truth = None
    try:
        for label, target_name, x, y, z, yaw, pitch in POSES:
            data.qpos[:3] = (x, y, z)
            data.qpos[3:7] = pose_quaternion(yaw, pitch)
            mujoco.mj_forward(model, data)

            camera_points, camera_geom = raycast(
                model, data, camera_id, body_id, 64, 48,
                float(model.cam_fovy[camera_id]), groups,
            )
            camera_points = camera_points[camera_geom == mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, target_name)]
            if len(camera_points) < 100:
                raise RuntimeError(f"too few camera depth samples on {target_name}: {len(camera_points)}")

            camera_rotation = data.cam_xmat[camera_id].reshape(3, 3).copy()
            camera_origin = data.cam_xpos[camera_id].copy()
            tof_rotation = data.cam_xmat[tof_id].reshape(3, 3).copy()
            tof_origin = data.cam_xpos[tof_id].copy()
            ground_truth = (
                tof_rotation.T @ camera_rotation,
                tof_rotation.T @ (camera_origin - tof_origin),
            )

            target_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, target_name)
            for sample_index in range(args.samples_per_pose):
                tof_local_points, tof_geom = raycast(
                    model, data, tof_id, body_id, 8, 8, fovy, groups,
                )
                target_indices = np.flatnonzero(tof_geom == target_geom)
                if len(target_indices) < 8:
                    raise RuntimeError(f"too few ToF zones on {target_name}: {len(target_indices)}")

                # Apply the configured simulator range noise to these 8x8 returns.
                clean_ranges = np.linalg.norm(tof_local_points[target_indices], axis=1)
                accuracy = np.where(clean_ranges <= 0.2,
                                    float(noise["near_accuracy_m"]),
                                    clean_ranges * float(noise["far_accuracy_fraction"]))
                corner = ((target_indices // 8 == 0) | (target_indices // 8 == 7)) & (
                    (target_indices % 8 == 0) | (target_indices % 8 == 7)
                )
                accuracy += clean_ranges * float(noise["corner_extra_fraction"]) * corner
                sigma = accuracy * float(noise["sigma_fraction_of_bound"])
                measured_ranges = clean_ranges + np.clip(
                    rng.normal(0.0, sigma) * args.noise_scale,
                    -accuracy * args.noise_scale, accuracy * args.noise_scale,
                )
                measured_ranges = np.clip(measured_ranges, min_range, max_range)
                tof_points = (tof_local_points[target_indices] / clean_ranges[:, None]
                              * measured_ranges[:, None])
                observations.append({
                    "target": f"{label}_{sample_index + 1}",
                    "camera_points": camera_points,
                    "plane": fit_plane(tof_points),
                    "tof_zone_count": int(len(target_indices)),
                })
    finally:
        model.geom_group[:] = original_groups

    all_points = np.concatenate([item["camera_points"] for item in observations])
    all_planes = np.concatenate([
        np.repeat(item["plane"][None, :], len(item["camera_points"]), axis=0)
        for item in observations
    ])

    def residual(parameters):
        rotation = Rotation.from_rotvec(parameters[:3]).as_matrix()
        points_tof = all_points @ rotation.T + parameters[3:]
        return np.einsum("ij,ij->i", all_planes[:, :3], points_tof) + all_planes[:, 3]

    result = least_squares(residual, np.zeros(6), loss="soft_l1", f_scale=0.02, max_nfev=500)
    estimated_rotation = Rotation.from_rotvec(result.x[:3]).as_matrix()
    estimated_translation = result.x[3:]
    true_rotation, true_translation = ground_truth
    rotation_error = Rotation.from_matrix(estimated_rotation @ true_rotation.T).magnitude()
    translation_error = np.linalg.norm(estimated_translation - true_translation)
    before = residual(np.zeros(6))
    after = residual(result.x)

    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output / "calibration_observations.npz",
        camera_points=all_points,
        tof_planes=all_planes,
    )
    summary = {
        "method": "simulated RGB-D raycast points to ToF fitted planes; robust SE(3) least squares",
        "model": str(args.model.relative_to(ROOT) if args.model.is_relative_to(ROOT) else args.model),
        "seed": args.seed,
        "tof_noise_scale": args.noise_scale,
        "samples_per_pose": args.samples_per_pose,
        "observations": [{"target": item["target"], "tof_zones": item["tof_zone_count"],
                          "camera_points": len(item["camera_points"])} for item in observations],
        "camera_to_tof": {
            "rotation_matrix": estimated_rotation.tolist(),
            "translation_m": estimated_translation.tolist(),
        },
        "simulation_truth_for_validation_only": {
            "rotation_matrix": true_rotation.tolist(),
            "translation_m": true_translation.tolist(),
        },
        "errors": {
            "rotation_deg": math.degrees(rotation_error),
            "translation_mm": translation_error * 1000,
            "point_to_plane_abs_median_before_m": float(np.median(np.abs(before))),
            "point_to_plane_abs_median_after_m": float(np.median(np.abs(after))),
            "point_to_plane_rmse_after_m": float(np.sqrt(np.mean(after**2))),
            "optimizer_success": bool(result.success),
            "optimizer_message": result.message,
        },
        "limitations": [
            "Uses synthetic local camera depth points and ToF planes; does not validate visual odometry.",
            "Known MuJoCo extrinsics are used only as a hidden validation reference, not as optimizer input.",
            "Scene geom IDs provide plane correspondence labels; real captures need manual or automatic plane association.",
            "ToF is one noisy ray per zone, not the sensor's measured depth distribution.",
        ],
    }
    (args.output / "calibration_result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))

    # Self-check the calibration result against the simulator's hidden mount transform.
    if not result.success or rotation_error > math.radians(5) or translation_error > 0.03:
        raise RuntimeError("calibration did not recover the known simulated sensor transform")


if __name__ == "__main__":
    main()
