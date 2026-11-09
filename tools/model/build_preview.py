#!/usr/bin/env python3
"""Build a static MuJoCo visual preview of cad/robot_v1_3.step.

Requires cadquery-ocp (OCP), PyYAML, and MuJoCo. The STEP contains component
placements but no mechanical joint mates, so this preview renders the assembled
CAD geometry and marks the seven joint-axis frames from the sidecar YAML. It is
not a dynamic or physics-calibrated simulation.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import struct
import tempfile

import yaml

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Trsf, gp_Vec

ROOT = Path(__file__).resolve().parents[2]
FRAME_FILE = ROOT / "model/joint_frames.yaml"

# Muted module palette (RGBA). Keep structural colors distinct without using
# saturated primary colors that wash out under MuJoCo's headlight.
MATERIALS = {
    "body": (0.76, 0.68, 0.48, 1.0),
    "left_leg": (0.35, 0.52, 0.49, 1.0),
    "right_leg": (0.39, 0.49, 0.58, 1.0),
    "head": (0.70, 0.54, 0.38, 1.0),
    "electronics": (0.55, 0.48, 0.62, 1.0),
    "battery": (0.62, 0.60, 0.40, 1.0),
    "motor": (0.58, 0.37, 0.36, 1.0),
    "belt": (0.50, 0.44, 0.32, 1.0),
    "metal": (0.42, 0.47, 0.51, 1.0),
    "axis_y": (0.70, 0.34, 0.31, 1.0),
    "axis_z": (0.28, 0.44, 0.62, 1.0),
}

def module_material(name: str) -> str:
    """Map each top-level STEP component to its visual module material."""
    lowered = name.lower().replace("_", " ").replace("-", " ")
    if "left leg" in lowered:
        return "left_leg"
    if "right leg" in lowered:
        return "right_leg"
    if "pcba" in lowered:
        return "electronics"
    if "head" in lowered:
        return "head"
    if "battery 14" in lowered:
        return "battery"
    if any(token in lowered for token in ("bearing", "m3x", "ga dong")):
        return "metal"
    if "dong co" in lowered:
        return "motor"
    if "timing belt" in lowered:
        return "belt"
    return "body"

def rgba_text(color: tuple[float, float, float, float]) -> str:
    return " ".join(f"{value:.3f}" for value in color)

def label_name(label: TDF_Label) -> str:
    name = TDataStd_Name()
    return name.Get().ToExtString() if label.FindAttribute(
        TDataStd_Name.GetID_s(), name
    ) else ""

def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]+", "_", value).strip("_") or "part"

def ascii_stl_triangles(path: Path):
    """Yield binary-packed triangles from OCP's ASCII STL output."""
    with path.open("r", encoding="ascii", errors="ignore") as stream:
        lines = iter(stream)
        for line in lines:
            fields = line.strip().split()
            if len(fields) != 5 or fields[:2] != ["facet", "normal"]:
                continue
            normal = tuple(float(value) for value in fields[2:5])
            vertices = []
            while len(vertices) < 3:
                fields = next(lines).strip().split()
                if fields and fields[0] == "vertex":
                    vertices.append(tuple(float(value) for value in fields[1:4]))
            yield struct.pack(
                "<12fH", *(normal + vertices[0] + vertices[1] + vertices[2]), 0
            )

def write_binary_chunks(
    source: Path, assets: Path, component_index: int, max_faces: int
) -> list[tuple[str, int]]:
    chunks: list[tuple[str, int]] = []
    data = bytearray()
    face_count = 0
    chunk_index = 0

    def flush() -> None:
        nonlocal data, face_count, chunk_index
        if face_count == 0:
            return
        filename = f"mesh_{component_index:02d}_{chunk_index:02d}.stl"
        header = f"RB v1.3 visual {component_index}:{chunk_index}".encode("ascii")
        header = header[:80].ljust(80, b" ")
        with (assets / filename).open("wb") as stream:
            stream.write(header)
            stream.write(struct.pack("<I", face_count))
            stream.write(data)
        chunks.append((filename, face_count))
        data = bytearray()
        face_count = 0
        chunk_index += 1

    for triangle in ascii_stl_triangles(source):
        data.extend(triangle)
        face_count += 1
        if face_count == max_faces:
            flush()
    flush()
    return chunks

def write_preview_xml(
    output: Path,
    parts: list[tuple[int, str, list[tuple[str, int]]]],
    frames: dict,
) -> None:
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<mujoco model="rb_v1_3_cad_preview">',
        '  <compiler angle="radian" meshdir="assets" autolimits="true"/>',
        '  <option timestep="0.002" gravity="0 0 0"/>',
        '  <visual>',
        '    <global azimuth="0" elevation="-18" fovy="45"/>',
        '    <headlight ambient="0.65 0.65 0.65" diffuse="0.80 0.80 0.80" specular="0.30 0.30 0.30"/>',
        '    <map znear="0.01" zfar="5"/>',
        '  </visual>',
        '  <asset>',
    ]
    for index, _, chunks in parts:
        for chunk_index, (filename, _) in enumerate(chunks):
            lines.append(
                f'    <mesh name="mesh_{index:02d}_{chunk_index:02d}" '
                f'file="{filename}" scale="0.001 0.001 0.001"/>'
            )
    for name in ("body", "left_leg", "right_leg", "head", "electronics",
                 "battery", "motor", "belt", "metal", "axis_y", "axis_z"):
        lines.append(
            f'    <material name="{name}" rgba="{rgba_text(MATERIALS[name])}" '
            'specular="0.22" shininess="0.28"/>'
        )
    lines.extend([
        '  </asset>',
        '  <worldbody>',
        '    <geom name="ground" type="plane" pos="0 0 -0.078" size="2 2 0.01" rgba="0.88 0.90 0.93 1" contype="0" conaffinity="0"/>',
        '    <body name="rb_v1_3_cad">',
    ])
    for index, name, chunks in parts:
        material = module_material(name)
        for chunk_index, _ in enumerate(chunks):
            lines.append(
                f'      <geom name="cad_{index:02d}_{chunk_index:02d}_{name}" '
                f'type="mesh" mesh="mesh_{index:02d}_{chunk_index:02d}" '
                f'material="{material}" contype="0" conaffinity="0"/>'
            )

    for joint in frames["joints"]:
        name = joint["name"]
        x, y, z = joint["origin_body_m"]
        axis = joint["axis_body"]
        if abs(axis[1]) > 0.5:
            direction = "y"
            start, end = (x, y - 0.018, z), (x, y + 0.018, z)
        else:
            direction = "z"
            start, end = (x, y, z - 0.025), (x, y, z + 0.025)
        fromto = " ".join(f"{value:.8f}" for value in (*start, *end))
        material = "axis_y" if direction == "y" else "axis_z"
        lines.append(
            f'      <geom name="axis_{name}" type="capsule" fromto="{fromto}" '
            f'size="0.0018" material="{material}" contype="0" conaffinity="0"/>'
        )
        lines.append(
            f'      <geom name="pivot_{name}" type="sphere" '
            f'pos="{x:.8f} {y:.8f} {z:.8f}" size="0.004" '
            f'material="{material}" contype="0" conaffinity="0"/>'
        )
    lines.extend(['    </body>', '  </worldbody>', '</mujoco>', ''])
    output.write_text("\n".join(lines), encoding="utf-8")

def build(step: Path, output: Path, deflection_mm: float, max_faces: int) -> Path:
    if not step.is_file():
        raise FileNotFoundError(step)
    if not FRAME_FILE.is_file():
        raise FileNotFoundError(FRAME_FILE)
    output.mkdir(parents=True, exist_ok=True)
    assets = output / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    for old_mesh in assets.glob("mesh_*.stl"):
        old_mesh.unlink()

    frames = yaml.safe_load(FRAME_FILE.read_text(encoding="utf-8"))
    origin = frames["source"]["body_link_datum"]["step_translation_mm"]
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(*(-float(value) for value in origin)))

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
    document = TDocStd_Document(TCollection_ExtendedString("rb-v1.3-preview"))
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free_shapes = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free_shapes)
    if free_shapes.Size() != 1:
        raise RuntimeError(f"expected one root assembly, found {free_shapes.Size()}")
    components = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(free_shapes.Value(1), components)

    parts: list[tuple[int, str, list[tuple[str, int]]]] = []
    with tempfile.TemporaryDirectory(prefix="rb-v1-3-") as scratch:
        scratch_path = Path(scratch)
        for index in range(1, components.Size() + 1):
            label = components.Value(index)
            definition = TDF_Label()
            base = (definition if XCAFDoc_ShapeTool.GetReferredShape_s(label, definition)
                    else label)
            name = label_name(base) or label_name(label) or f"component_{index:02d}"
            shape = shape_tool.GetShape_s(label)
            translated = BRepBuilderAPI_Transform(shape, transform, True).Shape()
            BRepMesh_IncrementalMesh(
                translated, deflection_mm, False, 0.45, True
            )
            ascii_stl = scratch_path / f"component_{index:02d}.stl"
            if not StlAPI_Writer().Write(translated, str(ascii_stl)):
                raise RuntimeError(f"failed to export component {index}: {name}")
            chunks = write_binary_chunks(ascii_stl, assets, index, max_faces)
            if not chunks:
                raise RuntimeError(f"component {index} exported no triangles: {name}")
            parts.append((index, safe_name(name), chunks))
            triangle_count = sum(count for _, count in chunks)
            print(
                f"{index:02d} {name}: {triangle_count:,} triangles "
                f"in {len(chunks)} mesh(es)", flush=True
            )

    model_path = output / "preview.xml"
    write_preview_xml(model_path, parts, frames)
    return model_path

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, default=ROOT / "cad/robot_v1_3.step")
    parser.add_argument("--output", type=Path, default=ROOT / "model")
    parser.add_argument("--deflection-mm", type=float, default=0.45)
    parser.add_argument("--max-faces-per-mesh", type=int, default=150_000)
    parser.add_argument("--launch", action="store_true",
                        help="open the generated static preview in MuJoCo")
    args = parser.parse_args()
    if args.deflection_mm <= 0 or not 0 < args.max_faces_per_mesh < 200_000:
        parser.error("deflection must be positive and mesh face limit must be 1..199999")
    model_path = build(args.step.resolve(), args.output.resolve(),
                       args.deflection_mm, args.max_faces_per_mesh)
    print(f"Wrote {model_path}", flush=True)
    if args.launch:
        import mujoco
        import mujoco.viewer

        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        print(
            f"MuJoCo {mujoco.__version__}: {model.nmesh} meshes, "
            f"{model.ngeom} geoms; static CAD preview",
            flush=True,
        )
        mujoco.viewer.launch(model, data)

if __name__ == "__main__":
    main()
