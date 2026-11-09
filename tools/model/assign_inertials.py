#!/usr/bin/env python3
"""Build RB v1.3 per-link inertials from audited CAD geometry and mass inputs.

CAD solid inertia is scaled to the user-supplied mass for each mass group,
then combined into rigid links with the parallel-axis theorem. The assembly
STEP has no kinematic mates, so link membership is an explicit, reviewable
mapping rather than inferred from CAD constraints.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
BLOCKS = ROOT / "model/cad_blocks.yaml"
FRAMES = ROOT / "model/joint_frames.yaml"
OUTPUT = ROOT / "model/inertials.yaml"
MJCF_OUTPUT = ROOT / "model/inertial_skeleton.xml"
ACTUATORS = ROOT / "model/actuators.yaml"
IMU = ROOT / "model/imu.yaml"

def component(blocks: dict, path: tuple[str, ...], mass_kg: float,
              label: str, instance: str | None = None) -> dict:
    matches = [row for row in blocks["components"]
               if tuple(row["assembly_path"]) == path
               and (instance is None or row["instance"] == instance)]
    if len(matches) != 1:
        raise ValueError(f"Expected one CAD record for {path} instance={instance}, found {len(matches)}")
    row = matches[0]
    volume_mm3 = float(row["solid_volume_m3"]) * 1e9
    volume_inertia_mm5 = np.asarray(
        row["uniform_volume_central_inertia_mm5"], dtype=float
    )
    if volume_mm3 <= 0 or not np.all(np.isfinite(volume_inertia_mm5)):
        raise ValueError(f"Invalid volume/inertia for {path}")
    # Volume inertia has units mm^5. Divide by CAD volume (mm^3), multiply
    # by supplied mass (kg), then convert mm^2 to m^2.
    inertia_kg_m2 = volume_inertia_mm5 * (mass_kg / volume_mm3) * 1e-6
    return {
        "label": label,
        "mass_kg": mass_kg,
        "com_body_m": np.asarray(row["uniform_volume_centroid_body_m"], dtype=float),
        "inertia_com_body_kg_m2": inertia_kg_m2,
        "cad_path": list(path),
        "cad_instance": row["instance"],
        "mass_method": "user mass scaled over CAD volume; uniform-density approximation",
    }

def box_lump(mass_kg: float, center_m: tuple[float, float, float],
             dimensions_m: tuple[float, float, float], label: str) -> dict:
    x, y, z = dimensions_m
    inertia = np.diag([
        mass_kg * (y*y + z*z) / 12.0,
        mass_kg * (x*x + z*z) / 12.0,
        mass_kg * (x*x + y*y) / 12.0,
    ])
    return {
        "label": label,
        "mass_kg": mass_kg,
        "com_body_m": np.asarray(center_m, dtype=float),
        "inertia_com_body_kg_m2": inertia,
        "cad_path": None,
        "cad_instance": None,
        "mass_method": "estimated uniform cuboid; location/dimensions are an explicit lumped-misc assumption",
    }

def combine(parts: list[dict], origin_body_m: list[float]) -> dict:
    total_mass = sum(part["mass_kg"] for part in parts)
    origin = np.asarray(origin_body_m, dtype=float)
    com_body = sum(part["mass_kg"] * part["com_body_m"] for part in parts) / total_mass
    inertia = np.zeros((3, 3), dtype=float)
    eye = np.eye(3)
    for part in parts:
        displacement = part["com_body_m"] - com_body
        inertia += part["inertia_com_body_kg_m2"] + part["mass_kg"] * (
            np.dot(displacement, displacement) * eye - np.outer(displacement, displacement)
        )
    local_com = com_body - origin
    eigenvalues = np.linalg.eigvalsh(inertia)
    if eigenvalues[0] <= 0 or not np.all(np.isfinite(eigenvalues)):
        raise ValueError(f"Invalid link inertia eigenvalues: {eigenvalues}")
    # A physical rigid-body inertia must also satisfy the principal-moment
    # triangle inequalities (largest <= sum of the other two).
    if eigenvalues[-1] > eigenvalues[0] + eigenvalues[1] + 1e-12:
        raise ValueError(f"Inertia violates principal-moment triangle inequality: {eigenvalues}")
    return {
        "mass_kg": total_mass,
        "com_body_m": com_body,
        "inertial_pos_link_m": local_com,
        "inertia_com_link_kg_m2": inertia,
        "principal_inertia_kg_m2": eigenvalues,
        "components": [part["label"] for part in parts],
    }

def rounded(values, digits=10):
    return np.round(np.asarray(values, dtype=float), digits).tolist()

def build(blocks_path: Path, frames_path: Path) -> dict:
    blocks = yaml.safe_load(blocks_path.read_text(encoding="utf-8"))
    frames = yaml.safe_load(frames_path.read_text(encoding="utf-8"))
    if blocks["source"]["sha256"] != frames["source"]["sha256"]:
        raise ValueError("CAD block report and frame file refer to different STEP sources")

    def p(path, mass, label, instance=None):
        return component(blocks, tuple(path), mass, label, instance)

    body_parts = [
        p(["Body 1 v1.3.2-1"], .175, "upper_body_175g"),
        p(["Body 2 v1.3.2"], .092, "lower_body_92g"),
        p(["PCBA"], .028, "pcb_28g"),
        p(["battery 14.4V-1"], .210, "battery_210g"),
        p(["battery Cover v1.3"], .012, "battery_cover_12g"),
        p(["Ga dong co dui"], .038, "thigh_motor_bracket_38g"),
        p(["dong co dui"], .130, "left_thigh_motor_130g", "NAUO8"),
        p(["dong co dui"], .130, "right_thigh_motor_130g", "NAUO9"),
        p(["Left LEG v1.3", "Dong co chan cho"], .410, "left_knee_motor_410g_body", "NAUO87"),
        p(["Right LEG v1.3", "Dong co chan cho"], .410, "right_knee_motor_410g_body", "NAUO53"),
        # Generic wire/fastener mass has no measured CAD COM. Approximate it
        # as a body-mounted cuboid and keep this assumption prominent.
        # Moved to the front block (x 50..110 mm) on request: the real robot tips its head down, which the
        # shell-centred lump did not reproduce. Placement stays an assumption until the COM is measured.
        box_lump(.100, (.080, 0.0, .00375), (.060, .1608, .0925),
                 "wiring_fasteners_misc_100g_estimated"),
    ]

    left_thigh = [
        p(["Left LEG v1.3", "Left Hip v1.3-1"], .085, "left_hip_shoulder_85g"),
    ]
    right_thigh = [
        p(["Right LEG v1.3", "Right Hip v1.3"], .085, "right_hip_shoulder_85g"),
    ]
    left_shin = [p(["Left LEG v1.3", "Left Knee v1.3-1"], .035, "left_knee_shin_35g")]
    right_shin = [p(["Right LEG v1.3", "Right Knee v1.3"], .035, "right_knee_shin_35g")]
    left_wheel = [p(["Left LEG v1.3", "Wheel"], .150, "left_wheel_assembly_150g_including_XGO")]
    right_wheel = [p(["Right LEG v1.3", "Wheel-1"], .150, "right_wheel_assembly_150g_including_XGO")]
    head = [p(["Robot Head POC"], .220, "head_assembly_220g_estimate")]

    joint_origins = {row["name"]: row["origin_body_m"] for row in frames["joints"]}
    links = {
        "body_link": combine(body_parts, [0, 0, 0]),
        "left_thigh": combine(left_thigh, joint_origins["left_hip"]),
        "right_thigh": combine(right_thigh, joint_origins["right_hip"]),
        "left_shin": combine(left_shin, joint_origins["left_knee"]),
        "right_shin": combine(right_shin, joint_origins["right_knee"]),
        "left_wheel": combine(left_wheel, joint_origins["left_wheel"]),
        "right_wheel": combine(right_wheel, joint_origins["right_wheel"]),
        "head": combine(head, joint_origins["head_yaw"]),
    }
    whole_robot = combine(
        body_parts + left_thigh + right_thigh + left_shin + right_shin
        + left_wheel + right_wheel + head,
        [0, 0, 0],
    )
    mass_total = sum(row["mass_kg"] for row in links.values())
    expected_mass = frames["mass_budget"]["listed_subtotal_kg"]
    if abs(mass_total - expected_mass) > 1e-9:
        raise ValueError(f"Link masses sum to {mass_total}, expected {expected_mass}")

    result_links = {}
    for name, row in links.items():
        result_links[name] = {
            "mass_kg": round(row["mass_kg"], 9),
            "inertial_pos_link_m": rounded(row["inertial_pos_link_m"], 8),
            "inertia_com_link_kg_m2": rounded(row["inertia_com_link_kg_m2"], 12),
            "principal_inertia_kg_m2": rounded(row["principal_inertia_kg_m2"], 12),
            "components": row["components"],
        }
    return {
        "schema_version": 1,
        "model": "RB v1.3",
        "source": {
            "step": blocks["source"]["step"],
            "sha256": blocks["source"]["sha256"],
            "coordinate_frame": "body_link; +X forward, +Y left, +Z up; metres and kg",
        },
        "mass_total_kg": round(mass_total, 9),
        "mass_status": "matches supplied row sum; completeness is not independently scale-verified",
        "assembled_reference_pose": {
            "mass_kg": round(whole_robot["mass_kg"], 9),
            "com_body_m": rounded(whole_robot["com_body_m"], 8),
            "inertia_com_body_kg_m2": rounded(whole_robot["inertia_com_link_kg_m2"], 12),
            "principal_inertia_kg_m2": rounded(whole_robot["principal_inertia_kg_m2"], 12),
            "note": "Aggregate diagnostic at the supplied STEP pose; not a substitute for articulated per-link inertials.",
        },
        "method": {
            "cad_tensor": "exact BREP volume tensor about the uniform-volume centroid, scaled by supplied mass/solid volume",
            "aggregation": "parallel-axis theorem in the STEP assembly pose; link frames use the joint origins in joint_frames.yaml",
            "material_limit": "CAD does not carry verified material densities or measured component COMs; internal mass is approximated as uniform within each named mass group",
            "kinematic_limit": "STEP has no joint mates; rigid-link membership is the explicit mapping listed below",
        },
        "link_membership": {
            "body_link": "body shells, PCB, battery, cover, thigh-motor bracket, all four 130/410 g motors, plus 100 g misc lump",
            "left_thigh": "Left Hip CAD segment, 85 g; motor-free long link between shoulder and knee axes",
            "right_thigh": "Right Hip CAD segment, 85 g; motor-free long link between shoulder and knee axes",
            "left_shin": "Left Knee 35 g",
            "right_shin": "Right Knee 35 g",
            "wheels": "150 g per complete Wheel assembly, including XGO motor as confirmed by user",
            "head": "Robot Head POC, 220 g user estimate",
        },
        "links": result_links,
        "validation": {
            "link_mass_sum_kg": round(mass_total, 9),
            "input_row_sum_kg": round(expected_mass, 9),
            "all_link_inertia_eigenvalues_positive": True,
            "principal_moment_triangle_inequality": True,
            "misc_100g_placement": "estimated as a 60 mm front block (x 50..110 mm) of the body-shell AABB; replace with measured allocation when available",
        },
    }

def actuator_lines(spec: dict) -> list[str]:
    """Tendon + motor elements: control range is joint-output torque (motor peak x ratio)."""
    lines = ['  <tendon>']
    for t in spec["crank_tendons"]:
        lines.append(f'    <fixed name="{t["name"]}">')
        for joint_name in t["joints"]:
            lines.append(f'      <joint joint="{joint_name}" coef="1"/>')
        lines.append('    </fixed>')
    lines += ['  </tendon>', '  <actuator>']
    for a in spec["joint_actuators"]:
        peak = float(spec["motors"][a["motor"]]["peak_torque_nm"]) * float(a["ratio"])
        target = ("joint" if "joint" in a["target"] else "tendon")
        lines.append(f'    <motor name="{a["name"]}" {target}="{a["target"][target]}" '
                     f'ctrlrange="{-peak:.12g} {peak:.12g}"/>')
    lines.append('  </actuator>')
    return lines

def imu_site_line(spec: dict) -> str:
    """IMU site on the body, axes aligned with body_link (identity orientation)."""
    imu = spec["imu"]
    axes = (imu["axes_in_body"]["x"], imu["axes_in_body"]["y"], imu["axes_in_body"]["z"])
    if axes != ("+X", "+Y", "+Z"):
        raise ValueError(f"IMU axes {axes}: only the identity mounting is wired into the MJCF so far")
    pos = " ".join(f"{float(c):.12g}" for c in imu["position_body_m"])
    return f'      <site name="imu" pos="{pos}" size="0.003"/>'

def imu_sensor_lines() -> list[str]:
    return ['  <sensor>',
            '    <gyro name="imu_gyro" site="imu"/>',
            '    <accelerometer name="imu_accel" site="imu"/>',
            '    <framequat name="imu_quat" objtype="site" objname="imu"/>',
            '  </sensor>']

def write_mjcf(inertials: dict, frames_path: Path, output: Path) -> None:
    frames = yaml.safe_load(frames_path.read_text(encoding="utf-8"))
    actuator_spec = yaml.safe_load(ACTUATORS.read_text(encoding="utf-8"))
    imu_spec = yaml.safe_load(IMU.read_text(encoding="utf-8"))
    # Dry joint friction per joint group, defined once in actuators.yaml (head_yaw has none: frictionless).
    group_friction = actuator_spec["joint_friction_nm"]
    friction_nm = {f"{side}_{group}": group_friction[group]
                   for side in ("left", "right") for group in ("hip", "knee", "wheel")}
    link_data = inertials["links"]
    joint = {row["name"]: row for row in frames["joints"]}

    def v(values):
        return " ".join(f"{float(value):.12g}" for value in values)

    def inertial_element(name: str) -> str:
        row = link_data[name]
        tensor = np.asarray(row["inertia_com_link_kg_m2"], dtype=float)
        full = [tensor[0, 0], tensor[1, 1], tensor[2, 2],
                tensor[0, 1], tensor[0, 2], tensor[1, 2]]
        return (f'<inertial pos="{v(row["inertial_pos_link_m"])}" '
                f'mass="{row["mass_kg"]:.12g}" fullinertia="{v(full)}"/>')

    def joint_element(name: str, axis: str) -> str:
        friction = friction_nm.get(name)
        extra = f' frictionloss="{friction:.12g}"' if friction is not None else ""
        return (f'<joint name="{name}" type="hinge" axis="{axis}" '
                f'limited="false"{extra}/>')

    hip_l = joint["left_hip"]["origin_body_m"]
    hip_r = joint["right_hip"]["origin_body_m"]
    knee_l = joint["left_knee"]["origin_body_m"]
    knee_r = joint["right_knee"]["origin_body_m"]
    wheel_l = joint["left_wheel"]["origin_body_m"]
    wheel_r = joint["right_wheel"]["origin_body_m"]
    head = joint["head_yaw"]["origin_body_m"]

    knee_l_rel = [knee_l[i] - hip_l[i] for i in range(3)]
    knee_r_rel = [knee_r[i] - hip_r[i] for i in range(3)]
    wheel_l_rel = [wheel_l[i] - knee_l[i] for i in range(3)]
    wheel_r_rel = [wheel_r[i] - knee_r[i] for i in range(3)]
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<mujoco model="rb_v1_3_inertial_skeleton">',
        '  <compiler angle="radian" autolimits="true"/>',
        '  <option timestep="0.002" gravity="0 0 -9.81"/>',
        '  <worldbody>',
        '    <body name="body_link">',
        '      <freejoint name="root_free"/>',
        f'      {inertial_element("body_link")}',
        imu_site_line(imu_spec),
        f'      <body name="left_thigh" pos="{v(hip_l)}">',
        f'        {joint_element("left_hip", "0 1 0")}',
        f'        {inertial_element("left_thigh")}',
        f'        <body name="left_shin" pos="{v(knee_l_rel)}">',
        f'          {joint_element("left_knee", "0 1 0")}',
        f'          {inertial_element("left_shin")}',
        f'          <body name="left_wheel" pos="{v(wheel_l_rel)}">',
        f'            {joint_element("left_wheel", "0 1 0")}',
        f'            {inertial_element("left_wheel")}',
        '          </body>',
        '        </body>',
        '      </body>',
        f'      <body name="right_thigh" pos="{v(hip_r)}">',
        f'        {joint_element("right_hip", "0 1 0")}',
        f'        {inertial_element("right_thigh")}',
        f'        <body name="right_shin" pos="{v(knee_r_rel)}">',
        f'          {joint_element("right_knee", "0 1 0")}',
        f'          {inertial_element("right_shin")}',
        f'          <body name="right_wheel" pos="{v(wheel_r_rel)}">',
        f'            {joint_element("right_wheel", "0 1 0")}',
        f'            {inertial_element("right_wheel")}',
        '          </body>',
        '        </body>',
        '      </body>',
        f'      <body name="head" pos="{v(head)}">',
        f'        {joint_element("head_yaw", "0 0 1")}',
        f'        {inertial_element("head")}',
        '      </body>',
        '    </body>',
        '  </worldbody>',
        *actuator_lines(actuator_spec),
        *imu_sensor_lines(),
        '</mujoco>',
        '',
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blocks", type=Path, default=BLOCKS)
    parser.add_argument("--frames", type=Path, default=FRAMES)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = build(args.blocks.resolve(), args.frames.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
                           encoding="utf-8")
    write_mjcf(report, args.frames.resolve(), MJCF_OUTPUT)
    print(f"Wrote {args.output.resolve()}")
    print(f"Wrote {MJCF_OUTPUT.resolve()}")
    print(f"Total link mass: {report['mass_total_kg']:.6f} kg")
    for name, item in report["links"].items():
        print(f"{name}: {item['mass_kg']:.6f} kg; COM={item['inertial_pos_link_m']}")

if __name__ == "__main__":
    main()
