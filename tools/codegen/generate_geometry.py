#!/usr/bin/env python3
"""Generate the controller's robot geometry headers for RB v1.3 from the project YAML files.

Writes firmware/core/include/rb1/core/GeneratedGeometry.hpp and GeneratedBodyEnvelope.hpp.
The controller models the robot as 7 planar bodies (body, then thigh/shin/wheel per side) with the legs in the
x-z plane; every joint turns about +Y and the knee angle is the shin angle relative to the thigh, which is the
same coordinate the parallelogram linkage gives (shin absolute angle = hip + knee).

Choices that are assumptions, not measurements, are listed in ASSUMPTIONS and printed in the header:
 - the head is rigid with the body for balance;
 - the 150 g wheel assembly is split into the BL4012 stator (132 g, fixed on the shin) and wheel + tyre (18 g,
   spinning), with the spinning part treated as a disc;
 - left and right legs are averaged, since the CAD differs from mirror symmetry by 0.5 mm or less.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "model"
OUT = ROOT / "firmware/core/include/rb1/core"

HIP_SOFT_RAD = (math.radians(-88.0), math.radians(133.0))   # CAD hard stops are about -90 and +135 degrees
KNEE_SOFT_RAD = (0.0, math.radians(110.0))                  # CAD hard stops are about 0 and 111.6 degrees
WHEEL_RADIUS_M = 0.035                                      # 70 mm tyre in the CAD
# Homing pose of the real robot (option B): knee folded to its stop and hip turned to its
# positive mechanical stop. The legs tuck up and the robot lies on its belly, which is also the direction
# StandUp expects (positive hip home, fold towards negative). The CAD gives +135.2 deg on the left and
# +134.9 deg on the right, so the mean is taken.
HOME_HIP_STOP_CAD_RAD = math.radians(135.05)
HOME_KNEE_CAD_RAD = 0.0
ENVELOPE_PRODUCTS = ("Body 1 v1.3.2-1", "Body 2 v1.3.2", "Robot Head POC")
ENVELOPE_PAD_M = 1.0e-4
# Placeholder leg inertias (hip, knee) [kg m^2]: RB has no measured data for them yet.
INHERITED_LEG_INERTIA = (7.5e-4, 2.5e-3)

ASSUMPTIONS = [
    "Head (0.22 kg) is rigid with the body for balance.",
    "Wheel assembly 0.150 kg = BL4012 stator 0.132 kg (on the shin) + wheel and tyre 0.018 kg (spinning, disc).",
    "Left and right legs are averaged; CAD asymmetry is 0.5 mm or less.",
    "Soft joint limits sit inside the CAD hard stops: hip -88..+133 deg, knee 0..110 deg.",
    "leg_*_inertia_kgm2 are placeholder values, not yet measured for RB.",
]

def f(value: float) -> str:
    text = f"{value:.7g}"
    if "e" not in text and "." not in text:
        text += ".0"
    return text + "F"

def load(name: str) -> dict:
    return yaml.safe_load((MODEL / name).read_text(encoding="utf-8"))

def link(inertials: dict, name: str) -> dict:
    row = inertials["links"][name]
    tensor = np.asarray(row["inertia_com_link_kg_m2"], float)
    pos = row["inertial_pos_link_m"]
    return {"m": float(row["mass_kg"]), "x": float(pos[0]), "z": float(pos[2]), "iyy": float(tensor[1, 1])}

def combine(parts: list[dict]) -> dict:
    """Merge rigid parts given as mass, COM (x, z) and Iyy about their own COM, all in one frame."""
    mass = sum(p["m"] for p in parts)
    x = sum(p["m"] * p["x"] for p in parts) / mass
    z = sum(p["m"] * p["z"] for p in parts) / mass
    iyy = sum(p["iyy"] + p["m"] * ((p["x"] - x) ** 2 + (p["z"] - z) ** 2) for p in parts)
    return {"m": mass, "x": x, "z": z, "iyy": iyy}

def build() -> dict:
    inertials = load("inertials.yaml")
    frames = {j["name"]: j for j in load("joint_frames.yaml")["joints"]}
    actuators = load("actuators.yaml")
    imu = load("imu.yaml")["imu"]

    # Body: body_link plus the head carried at its yaw-joint origin.
    head = link(inertials, "head")
    head_origin = frames["head_yaw"]["origin_body_m"]
    head_in_body = {"m": head["m"], "x": head_origin[0] + head["x"], "z": head_origin[2] + head["z"], "iyy": head["iyy"]}
    body = combine([link(inertials, "body_link"), head_in_body])

    thigh = {"m": link(inertials, "left_thigh")["m"],
             "x": (link(inertials, "left_thigh")["x"] + link(inertials, "right_thigh")["x"]) / 2,
             "z": (link(inertials, "left_thigh")["z"] + link(inertials, "right_thigh")["z"]) / 2,
             "iyy": (link(inertials, "left_thigh")["iyy"] + link(inertials, "right_thigh")["iyy"]) / 2}

    def rel(child: str, parent: str, side: str) -> np.ndarray:
        return np.asarray(frames[f"{side}_{child}"]["origin_body_m"], float) - np.asarray(frames[f"{side}_{parent}"]["origin_body_m"], float)

    hip = {s: np.asarray(frames[f"{s}_hip"]["origin_body_m"], float) for s in ("left", "right")}
    knee_rel = {s: rel("knee", "hip", s) for s in ("left", "right")}
    wheel_rel = {s: rel("wheel", "knee", s) for s in ("left", "right")}
    mean_knee = (knee_rel["left"] + np.array([knee_rel["right"][0], -knee_rel["right"][1], knee_rel["right"][2]])) / 2
    mean_wheel = (wheel_rel["left"] + np.array([wheel_rel["right"][0], -wheel_rel["right"][1], wheel_rel["right"][2]])) / 2

    # Shin carries the BL4012 stator; the spinning part is the wheel and tyre.
    wa = actuators["wheel_assembly"]
    stator_m, assembly_m = float(wa["motor_mass_kg"]), float(wa["mass_kg"])
    spin_m = assembly_m - stator_m
    stator_r = float(actuators["motors"]["kpower_bl4012"]["dimensions_mm"]["outer_diameter"]) / 2000.0
    left_shin, right_shin = link(inertials, "left_shin"), link(inertials, "right_shin")
    shin_link = {"m": left_shin["m"], "x": (left_shin["x"] + right_shin["x"]) / 2,
                 "z": (left_shin["z"] + right_shin["z"]) / 2, "iyy": (left_shin["iyy"] + right_shin["iyy"]) / 2}
    stator = {"m": stator_m, "x": float(mean_wheel[0]), "z": float(mean_wheel[2]), "iyy": 0.5 * stator_m * stator_r ** 2}
    shin = combine([shin_link, stator])
    wheel = {"m": spin_m, "x": 0.0, "z": 0.0, "iyy": 0.5 * spin_m * WHEEL_RADIUS_M ** 2}

    total = body["m"] + 2 * (thigh["m"] + shin["m"] + wheel["m"])
    hip_y = abs(hip["left"][1])
    axle_y = hip_y + abs(mean_knee[1]) + abs(mean_wheel[1])
    parts = [body] + [thigh, shin, wheel] * 2
    origins = [(0.0, hip_y, float(hip["left"][2])), (float(mean_knee[0]), abs(mean_knee[1]), float(mean_knee[2])),
               (float(mean_wheel[0]), abs(mean_wheel[1]), float(mean_wheel[2]))]
    right = [(0.0, -hip_y, float(hip["left"][2])), (float(mean_knee[0]), -abs(mean_knee[1]), float(mean_knee[2])),
             (float(mean_wheel[0]), -abs(mean_wheel[1]), float(mean_wheel[2]))]
    axle_below_hip = -(origins[1][2] + origins[2][2])
    spawn_z = axle_below_hip + WHEEL_RADIUS_M - origins[0][2]

    blocks = load("cad_blocks.yaml")["components"]
    boxes = []
    for product in ENVELOPE_PRODUCTS:
        row = next(c for c in blocks if c["product"] == product and len(c["assembly_path"]) == 1)
        lo = [v - ENVELOPE_PAD_M for v in row["aabb_min_body_m"]]
        hi = [v + ENVELOPE_PAD_M for v in row["aabb_max_body_m"]]
        boxes.append((product, lo, hi))
    # Controller coordinates: RobotController adds hang_hip_cad to a measured hip angle before using the geometry,
    # where hang_hip_cad puts the axle straight below the hip with the knee at 0.
    knee_x, knee_z = origins[1][0], origins[1][2]
    wheel_x, wheel_z = origins[2][0], origins[2][2]
    hang_hip_cad = math.atan2(knee_x + wheel_x, -(knee_z + wheel_z))

    def axle_x(hip, knee):
        c, s = math.cos(hip), math.sin(hip)
        c2, s2 = math.cos(hip + knee), math.sin(hip + knee)
        return c * knee_x + s * knee_z + c2 * wheel_x + s2 * wheel_z

    lo, hi = -2.0, 0.0                                     # hip that puts the axle under the hip at the soft knee limit
    for _ in range(80):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if axle_x(mid, KNEE_SOFT_RAD[1]) > 0 else (lo, mid)
    straight_hip_cad = (lo + hi) / 2
    poses = dict(hang_hip_cad=hang_hip_cad, straight_hip=straight_hip_cad - hang_hip_cad, straight_knee=KNEE_SOFT_RAD[1],
                 home_hip=HOME_HIP_STOP_CAD_RAD - hang_hip_cad, home_knee=HOME_KNEE_CAD_RAD)
    return dict(poses=poses, parts=parts, origins=origins + right, total=total, axle_spacing=2 * axle_y, spawn_z=spawn_z,
                boxes=boxes, imu=imu["position_body_m"], axle_below_hip=axle_below_hip, spin_m=spin_m, stator_m=stator_m)

def write_geometry(g: dict, path: Path) -> None:
    names = ["body (body_link + head)", "left thigh", "left shin + BL4012 stator", "left wheel + tyre",
             "right thigh", "right shin + BL4012 stator", "right wheel + tyre"]
    lines = [
        "// Generated by tools/codegen/generate_geometry.py from the RB v1.3 CAD, inertial, joint-frame,",
        "// actuator and IMU YAML files in model/. Do not edit by hand.",
        "// +X forward, +Y left, +Z up. Body frame = body_link, origin at the hip axis height.",
        "// Part order: body, left thigh/shin/wheel, right thigh/shin/wheel. Link COM (x, z) is relative to",
        "// that link's own joint origin; iyy is about the link COM.",
        "// Assumptions:",
        *[f"//  - {a}" for a in ASSUMPTIONS],
        "#pragma once",
        "#include <array>",
        "namespace rb1::core::geometry {",
        "struct Part { float mass, x, z, iyy; };",
        "struct Origin { float x, y, z; };",
        "inline constexpr std::array<Part, 7> parts{{",
    ]
    for i, (p, n) in enumerate(zip(g["parts"], names)):
        comma = "," if i < 6 else ""
        lines.append(f"    {{{f(p['m'])}, {f(p['x'])}, {f(p['z'])}, {f(p['iyy'])}}}{comma}  // {n}")
    lines += ["}};", "// Joint origins: hip in the body frame, knee relative to the hip, wheel axle relative to the knee.",
              "inline constexpr std::array<Origin, 6> origins{{"]
    for i, o in enumerate(g["origins"]):
        comma = "," if i < 5 else ""
        lines.append(f"    {{{f(o[0])}, {f(o[1])}, {f(o[2])}}}{comma}")
    lines += [
        "}};",
        f"inline constexpr float total_mass_kg = {f(g['total'])};",
        f"inline constexpr float wheel_radius_m = {f(WHEEL_RADIUS_M)};",
        f"inline constexpr float axle_spacing_m = {f(g['axle_spacing'])};",
        "inline constexpr float neutral_hip_rad = 0.0F;",
        "inline constexpr float neutral_knee_rad = 0.0F;  // CAD pose: knee fully folded.",
        "inline constexpr float knee_zero_rebase_rad = 0.0F;",
        f"inline constexpr float neutral_spawn_z_m = {f(g['spawn_z'])};",
        f"inline constexpr float native_hip_min_rad = {f(HIP_SOFT_RAD[0])};",
        f"inline constexpr float native_hip_max_rad = {f(HIP_SOFT_RAD[1])};",
        f"inline constexpr float native_knee_min_rad = {f(KNEE_SOFT_RAD[0])};",
        f"inline constexpr float native_knee_max_rad = {f(KNEE_SOFT_RAD[1])};",
        "inline constexpr float native_knee_travel_rad = native_knee_max_rad;",
        "// Encoder-zero poses in controller coordinates (hip measured from hang_hip_cad). The real robot is homed in",
        "// home_*: knee folded to its stop, hip at its positive mechanical stop. straight_* is the extended pose",
        "// (knee at the soft limit, axle straight below the hip, 184.8 mm).",
        f"inline constexpr float hang_hip_cad_rad = {f(g['poses']['hang_hip_cad'])};",
        f"inline constexpr float home_hip_rad = {f(g['poses']['home_hip'])};",
        f"inline constexpr float home_knee_rad = {f(g['poses']['home_knee'])};",
        f"inline constexpr float straight_hip_rad = {f(g['poses']['straight_hip'])};",
        f"inline constexpr float straight_knee_rad = {f(g['poses']['straight_knee'])};",
        f"inline constexpr float leg_hip_inertia_kgm2 = {f(INHERITED_LEG_INERTIA[0])};  // inherited, not measured for RB",
        f"inline constexpr float leg_knee_inertia_kgm2 = {f(INHERITED_LEG_INERTIA[1])};  // inherited, not measured for RB",
        "}  // namespace rb1::core::geometry",
        ""]
    path.write_text("\n".join(lines), encoding="utf-8")

def write_envelope(g: dict, path: Path) -> None:
    lines = [
        "// Generated by tools/codegen/generate_geometry.py from model/cad_blocks.yaml.",
        "// Fixed body shell and head AABBs in body_link with 0.1 mm padding, not calibrated contact.",
        "// +X forward, +Y left, +Z up.",
        "#pragma once",
        "#include <array>",
        "namespace rb1::core::geometry {",
        "struct BodyBox { std::array<float, 3> lo, hi; };",
        f"inline constexpr std::array<BodyBox, {len(g['boxes'])}> body_envelope{{{{"]
    for i, (name, lo, hi) in enumerate(g["boxes"]):
        comma = "," if i < len(g["boxes"]) - 1 else ""
        lines.append(f"    {{{{{', '.join(f(v) for v in lo)}}}, {{{', '.join(f(v) for v in hi)}}}}}{comma}  // {name}")
    lines += ["}};", "}  // namespace rb1::core::geometry", ""]
    path.write_text("\n".join(lines), encoding="utf-8")

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    g = build()
    write_geometry(g, args.out / "GeneratedGeometry.hpp")
    write_envelope(g, args.out / "GeneratedBodyEnvelope.hpp")
    print(f"total mass {g['total']:.4f} kg, axle spacing {g['axle_spacing']:.4f} m, spawn z {g['spawn_z']:.5f} m")
    print(f"wheel+tyre {g['spin_m']*1000:.0f} g spinning, stator {g['stator_m']*1000:.0f} g on the shin")
    print(f"IMU position in body frame (for WholeBodyModel.hpp): {g['imu']}")
    print(f"wrote headers to {args.out}")

if __name__ == "__main__":
    main()
