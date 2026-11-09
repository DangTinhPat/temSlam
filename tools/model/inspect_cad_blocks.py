#!/usr/bin/env python3
"""Inspect named RB v1.3 STEP blocks in the body_link coordinate frame."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import yaml

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.GProp import GProp_GProps
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Trsf, gp_Vec

ROOT = Path(__file__).resolve().parents[2]
STEP_DEFAULT = ROOT / "cad/robot_v1_3.step"
FRAMES_DEFAULT = ROOT / "model/joint_frames.yaml"
REPORT_DEFAULT = ROOT / "model/cad_blocks.yaml"

INTERESTING = {
    "Left LEG v1.3", "Right LEG v1.3", "Wheel", "Wheel-1",
    "Wheel 70mm", "XGO 000 ____ A 2024.2.21", "Spring assembly",
    "Join Link-1", "Join Link", "Left Link v1.2-1", "Right Link v1.2",
    "Link 2-1", "Link 2", "Left Hip v1.3-1", "Right Hip v1.3",
    "Left Knee v1.3-1", "Right Knee v1.3", "Dong co chan cho",
    "PCBA", "Robot Head POC", "Body 1 v1.3.2-1", "Body 2 v1.3.2",
    "dong co dui", "Ga dong co dui", "battery 14.4V-1",
    "battery Cover v1.3",
}

def label_name(label: TDF_Label) -> str:
    attribute = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
        return attribute.Get().ToExtString()
    return ""

def inspect(step: Path, frame_file: Path) -> dict:
    frame_data = yaml.safe_load(frame_file.read_text(encoding="utf-8"))
    datum = frame_data["source"]["body_link_datum"]
    origin_mm = datum["step_translation_mm"]
    source_hash = hashlib.sha256(step.read_bytes()).hexdigest()

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(False)
    reader.SetMatMode(False)
    reader.SetLayerMode(False)
    reader.SetPropsMode(False)
    reader.SetGDTMode(False)
    status = reader.ReadFile(str(step))
    if str(status).split(".")[-1] != "IFSelect_RetDone":
        raise RuntimeError(f"STEP read failed: {status}")
    document = TDocStd_Document(TCollection_ExtendedString("rb-v1.3-inspect"))
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer failed")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free_shapes = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free_shapes)
    if free_shapes.Size() != 1:
        raise RuntimeError(f"expected one root assembly, found {free_shapes.Size()}")
    root = free_shapes.Value(1)
    root_components = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(root, root_components)

    inverse_datum = gp_Trsf()
    inverse_datum.SetTranslation(gp_Vec(*(-float(value) for value in origin_mm)))

    records = []
    joint_by_name = {joint["name"]: joint for joint in frame_data["joints"]}
    axis_source_paths = {
        "left_hip": ("Left LEG v1.3", "Left Hip v1.3-1"),
        "right_hip": ("Right LEG v1.3", "Right Hip v1.3"),
        "left_knee": ("Left LEG v1.3", "Left Knee v1.3-1"),
        "right_knee": ("Right LEG v1.3", "Right Knee v1.3"),
        "left_wheel": ("Left LEG v1.3", "Wheel", "Wheel 70mm"),
        "right_wheel": ("Right LEG v1.3", "Wheel-1", "Wheel 70mm"),
        "head_yaw": ("Robot Head POC", "JC2804 BLDC Motor", "JC2804 BLDC Motor"),
    }
    axis_evidence = {}

    def inspect_axis_geometry(joint_name, shape, total_location):
        joint = joint_by_name[joint_name]
        origin = joint["origin_body_m"]
        expected = joint["axis_body"]
        norm = sum(value * value for value in expected) ** 0.5
        expected = [value / norm for value in expected]
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        offsets_mm = []
        radii_mm = set()
        while explorer.More():
            surface = BRepAdaptor_Surface(TopoDS.Face(explorer.Current()), True)
            if surface.GetType() == GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                axis = cylinder.Axis().Transformed(total_location.Transformation())
                point = axis.Location()
                direction = axis.Direction()
                body_point = [
                    (point.X() - origin_mm[0]) * .001,
                    (point.Y() - origin_mm[1]) * .001,
                    (point.Z() - origin_mm[2]) * .001,
                ]
                body_direction = [direction.X(), direction.Y(), direction.Z()]
                alignment = abs(sum(a * b for a, b in zip(body_direction, expected)))
                delta = [a - b for a, b in zip(body_point, origin)]
                along = sum(a * b for a, b in zip(delta, expected))
                offset = sum((a - along * b) ** 2
                             for a, b in zip(delta, expected)) ** .5
                if alignment >= 0.9999 and offset <= 0.001:
                    offsets_mm.append(offset * 1000)
                    radii_mm.add(round(cylinder.Radius(), 4))
            explorer.Next()
        axis_evidence[joint_name] = {
            "source_product_path": list(axis_source_paths[joint_name]),
            "matching_cylindrical_faces": len(offsets_mm),
            "supporting_radii_mm": sorted(radii_mm),
            "max_axis_line_offset_mm": (round(max(offsets_mm), 5)
                                         if offsets_mm else None),
            "status": "matched" if offsets_mm else "no_cylindrical_match",
        }

    def visit(label, parent_location: TopLoc_Location, parent_path: tuple[str, ...]):
        total_location = parent_location.Multiplied(
            XCAFDoc_ShapeTool.GetLocation_s(label)
        )
        definition = TDF_Label()
        is_reference = XCAFDoc_ShapeTool.GetReferredShape_s(label, definition)
        base = definition if is_reference else label
        product_name = label_name(base) or label_name(label)
        path = parent_path + (product_name,)
        children = Sequence_TDF_Label()
        XCAFDoc_ShapeTool.GetComponents_s(base, children)

        if product_name in INTERESTING:
            shape = shape_tool.GetShape_s(base)
            world_shape = BRepBuilderAPI_Transform(
                shape, total_location.Transformation(), True
            ).Shape()
            body_shape = BRepBuilderAPI_Transform(
                world_shape, inverse_datum, True
            ).Shape()
            box = Bnd_Box()
            BRepBndLib.Add_s(body_shape, box)
            mins_mm = (box.GetXMin(), box.GetYMin(), box.GetZMin())
            maxs_mm = (box.GetXMax(), box.GetYMax(), box.GetZMax())
            minimum = [round(value * .001, 6) for value in mins_mm]
            maximum = [round(value * .001, 6) for value in maxs_mm]
            bbox_center = [round((a + b) * .5, 6)
                           for a, b in zip(minimum, maximum)]
            volume_properties = GProp_GProps()
            BRepGProp.VolumeProperties_s(body_shape, volume_properties)
            volume = volume_properties.Mass()
            volume_center = None
            central_volume_inertia = None
            if volume > 0.0:
                center = volume_properties.CentreOfMass()
                volume_center = [round(center.X() * .001, 6),
                                 round(center.Y() * .001, 6),
                                 round(center.Z() * .001, 6)]
                # OpenCascade's MatrixOfInertia is central (about the
                # framework's centre of mass). Keep its axes in body_link;
                # it can be scaled by mass/volume and combined later with
                # the parallel-axis theorem.
                matrix = volume_properties.MatrixOfInertia()
                central_volume_inertia = []
                for row in range(1, 4):
                    central_volume_inertia.append([
                        round(matrix.Value(row, column), 3)
                        for column in range(1, 4)
                    ])
            records.append({
                "product": product_name,
                "assembly_path": list(path),
                "instance": label_name(label),
                "aabb_min_body_m": minimum,
                "aabb_max_body_m": maximum,
                "aabb_center_body_m": bbox_center,
                "uniform_volume_centroid_body_m": volume_center,
                "solid_volume_m3": round(volume * 1e-9, 9),
                "uniform_volume_central_inertia_mm5": central_volume_inertia,
            })

        for joint_name, source_path in axis_source_paths.items():
            if path == source_path and children.Size() == 0:
                inspect_axis_geometry(joint_name, shape_tool.GetShape_s(base),
                                      total_location)

        for index in range(1, children.Size() + 1):
            visit(children.Value(index), total_location, path)

    for index in range(1, root_components.Size() + 1):
        visit(root_components.Value(index), TopLoc_Location(), ())

    # The root assembly box is also reported relative to the same body datum.
    root_shape = shape_tool.GetShape_s(root)
    root_shape = BRepBuilderAPI_Transform(
        root_shape, inverse_datum, True
    ).Shape()
    root_box = Bnd_Box()
    BRepBndLib.Add_s(root_shape, root_box)
    root_min = [round(value * .001, 6) for value in
                (root_box.GetXMin(), root_box.GetYMin(), root_box.GetZMin())]
    root_max = [round(value * .001, 6) for value in
                (root_box.GetXMax(), root_box.GetYMax(), root_box.GetZMax())]

    def find_record(path):
        return next(record for record in records
                    if record["assembly_path"] == path)

    left_leg = find_record(["Left LEG v1.3"])
    right_leg = find_record(["Right LEG v1.3"])
    left_wheel = find_record(["Left LEG v1.3", "Wheel", "Wheel 70mm"])
    right_wheel = find_record(["Right LEG v1.3", "Wheel-1", "Wheel 70mm"])

    def mirror_aabb_error(left, right):
        # X and Z should match; Y should reflect around the body sagittal plane.
        errors = [
            abs(left["aabb_min_body_m"][0] - right["aabb_min_body_m"][0]),
            abs(left["aabb_max_body_m"][0] - right["aabb_max_body_m"][0]),
            abs(left["aabb_min_body_m"][2] - right["aabb_min_body_m"][2]),
            abs(left["aabb_max_body_m"][2] - right["aabb_max_body_m"][2]),
            abs(left["aabb_min_body_m"][1] + right["aabb_max_body_m"][1]),
            abs(left["aabb_max_body_m"][1] + right["aabb_min_body_m"][1]),
        ]
        return round(max(errors), 6)

    axis_positions = {
        joint["name"]: joint["origin_body_m"]
        for joint in frame_data["joints"]
    }
    hip_mirror_error = max(
        abs(axis_positions["left_hip"][0] - axis_positions["right_hip"][0]),
        abs(axis_positions["left_hip"][2] - axis_positions["right_hip"][2]),
        abs(axis_positions["left_hip"][1] + axis_positions["right_hip"][1]),
    )
    knee_mirror_error = max(
        abs(axis_positions["left_knee"][0] - axis_positions["right_knee"][0]),
        abs(axis_positions["left_knee"][2] - axis_positions["right_knee"][2]),
        abs(axis_positions["left_knee"][1] + axis_positions["right_knee"][1]),
    )
    wheel_mirror_error = max(
        abs(axis_positions["left_wheel"][0] - axis_positions["right_wheel"][0]),
        abs(axis_positions["left_wheel"][2] - axis_positions["right_wheel"][2]),
        abs(axis_positions["left_wheel"][1] + axis_positions["right_wheel"][1]),
    )

    return {
        "source": {
            "step": step.name,
            "sha256": source_hash,
            "length_unit": "mm",
            "top_level_component_count": root_components.Size(),
        },
        "body_frame": {
            "name": "body_link",
            "datum_product": datum["product"],
            "step_origin_mm": origin_mm,
            "axes": ["x_forward", "y_left", "z_up"],
            "robot_aabb_min_m": root_min,
            "robot_aabb_max_m": root_max,
        },
        "position_semantics": {
            "aabb_center": "axis-aligned bounding-box center; not a mass COM",
            "uniform_volume_centroid": "geometry volume centroid assuming uniform density; not a measured mass COM",
            "uniform_volume_central_inertia": "CAD volume inertia about uniform-volume centroid in body_link axes; scale by mass/volume only as a uniform-density approximation",
        },
        "mass_budget_review": {
            "user_row_total_kg": frame_data["mass_budget"]["listed_subtotal_kg"],
            "verified_full_robot_mass_kg": None,
            "per_link_inertial_manifest_created": True,
            "per_link_inertials_embedded_in_articulated_mjcf_skeleton": True,
            "full_visual_contact_model_available": False,
            "direct_product_matches": {
                "upper_body": ["Body 1 v1.3.2-1"],
                "lower_body": ["Body 2 v1.3.2"],
                "pcb": ["PCBA"],
                "battery": ["battery 14.4V-1"],
                "battery_cover": ["battery Cover v1.3"],
                "thigh_motor_bracket": ["Ga dong co dui"],
                "thigh_motor_pair": ["dong co dui"],
                "knee_motor_pair": ["Dong co chan cho"],
                "head_estimate": ["Robot Head POC"],
                "wheel_pair": ["Wheel", "Wheel-1"],
            },
            "unresolved_mass_blocks": {
                "wiring_fasteners_misc": "100 g lump has no measured COM or CAD solid; manifest uses a body-centered cuboid estimate.",
            },
            "review_required": [
                "Measure complete assembled robot mass to check whether supplied rows cover every component.",
                "Measure or allocate the 100 g wiring/fasteners mass across rigid links.",
                "Replace uniform-density CAD tensor approximations with measured COM/inertia where sim-to-real accuracy requires it.",
            ],
        },
        "symmetry_checks": {
            "left_right_leg_aabb_max_mirror_error_m": mirror_aabb_error(left_leg, right_leg),
            "left_right_wheel_aabb_max_mirror_error_m": mirror_aabb_error(left_wheel, right_wheel),
            "hip_axis_max_mirror_error_m": round(hip_mirror_error, 6),
            "knee_axis_max_mirror_error_m": round(knee_mirror_error, 6),
            "wheel_axis_max_mirror_error_m": round(wheel_mirror_error, 6),
            "interpretation": "Geometric symmetry check only; does not validate mass or joint zero/sign.",
        },
        "joint_axes": frame_data["joints"],
        "axis_geometry_evidence": axis_evidence,
        "components": records,
    }

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, default=STEP_DEFAULT)
    parser.add_argument("--frames", type=Path, default=FRAMES_DEFAULT)
    parser.add_argument("--output", type=Path, default=REPORT_DEFAULT)
    args = parser.parse_args()
    report = inspect(args.step.resolve(), args.frames.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
                           encoding="utf-8")
    print(f"Wrote {args.output.resolve()}")
    print(f"Source SHA-256: {report['source']['sha256']}")
    print(f"Components inspected: {len(report['components'])}")
    print(f"Robot AABB: {report['body_frame']['robot_aabb_min_m']} .. "
          f"{report['body_frame']['robot_aabb_max_m']} m")

if __name__ == "__main__":
    main()
