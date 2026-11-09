#!/usr/bin/env python3
"""Extract the convex hull of each RB v1.3 thigh (for the floor contact of the dynamic model).

The leg sub-assemblies of cad/robot_v1_3.step are split into rigid groups by part name; the thigh group (Left/Right Hip, bearing
bracket, GT2 pulley, hip cover, screws, wire cover 2) is meshed and its convex hull is written to
model/thigh_hull.json in the thigh frame (origin on the hip axis, +X forward, metres).
Requires cadquery-ocp (OCP), SciPy and PyYAML.
"""
from __future__ import annotations

import json
import struct
import tempfile
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import ConvexHull

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Builder
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Trsf, gp_Vec

ROOT = Path(__file__).resolve().parents[2]
FRAMES = ROOT / "model/joint_frames.yaml"
OUT = ROOT / "model/thigh_hull.json"
OTHER_GROUPS = ("dong co chan cho", "wire roller", "bearing 6809", "wheel 70mm", "rubber ring", "xgo", "wheel washer",
                "left knee", "right knee", "wire cover 1", "wire cover 3", "link 2", "left link", "right link",
                "m3x12 iso 4762", "join link", "spring")

def name_of(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

def is_thigh(name: str, xmax_mm: float) -> bool:
    low = name.lower()
    if low.startswith("link 2") or any(low.startswith(g) or g in low for g in OTHER_GROUPS):
        return False
    return not ("m2" in low)          # the M2 screws belong to the crank / coupler

def leaves(label, location, path, out):
    total = location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
    definition = TDF_Label()
    base = definition if XCAFDoc_ShapeTool.GetReferredShape_s(label, definition) else label
    children = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(base, children)
    here = path + (name_of(base) or name_of(label),)
    if children.Size() == 0:
        out.append((here, BRepBuilderAPI_Transform(XCAFDoc_ShapeTool.GetShape_s(base), total.Transformation(), True).Shape()))
    for index in range(1, children.Size() + 1):
        leaves(children.Value(index), total, here, out)

def stl_vertices(path: str) -> np.ndarray:
    pts = []
    for line in open(path, encoding="ascii", errors="ignore"):
        fields = line.split()
        if fields and fields[0] == "vertex":
            pts.append([float(v) for v in fields[1:4]])
    return np.asarray(pts)

def main() -> None:
    frames = yaml.safe_load(FRAMES.read_text(encoding="utf-8"))
    origin = np.asarray(frames["source"]["body_link_datum"]["step_translation_mm"], dtype=float)
    hips = {j["name"]: np.asarray(j["origin_body_m"], dtype=float) * 1000.0 for j in frames["joints"]}
    reader = STEPCAFControl_Reader()
    for setter in ("SetNameMode",):
        getattr(reader, setter)(True)
    for setter in ("SetColorMode", "SetMatMode", "SetLayerMode", "SetPropsMode", "SetGDTMode"):
        getattr(reader, setter)(False)
    reader.ReadFile(str(ROOT / "cad/robot_v1_3.step"))
    document = TDocStd_Document(TCollection_ExtendedString("thigh-hull"))
    reader.Transfer(document)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free = Sequence_TDF_Label()
    shapes.GetFreeShapes(free)
    components = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(free.Value(1), components)
    to_body = gp_Trsf()
    to_body.SetTranslation(gp_Vec(*(-origin)))
    result = {"frame": "thigh frame: origin on the hip axis, +X forward, +Y left, +Z up; metres, convex hull vertices",
              "source": "cad/robot_v1_3.step via tools/model/extract_thigh_hull.py"}
    for side, index in (("left", 1), ("right", 2)):
        parts: list = []
        leaves(components.Value(index), TopLoc_Location(), (), parts)
        compound, builder = TopoDS_Compound(), BRep_Builder()
        builder.MakeCompound(compound)
        kept = []
        for path, shape in parts:
            if is_thigh(path[-1], 0.0):
                builder.Add(compound, BRepBuilderAPI_Transform(shape, to_body, True).Shape())
                kept.append(path[-1])
        BRepMesh_IncrementalMesh(compound, 0.5, False, 0.45, True)
        with tempfile.TemporaryDirectory() as tmp:
            stl = str(Path(tmp) / "thigh.stl")
            StlAPI_Writer().Write(compound, stl)
            points = stl_vertices(stl)
        points -= hips[f"{side}_hip"] * np.array([1, 1, 1])
        hull = ConvexHull(points)
        verts = np.round(points[hull.vertices], 1) * 0.001
        verts = np.unique(np.round(verts, 4), axis=0)
        result[side] = {"parts": kept, "vertices_m": verts.tolist()}
        print(side, len(kept), "parts,", len(verts), "hull vertices; x", verts[:, 0].min().round(4), verts[:, 0].max().round(4),
              "y", verts[:, 1].min().round(4), verts[:, 1].max().round(4), "z", verts[:, 2].min().round(4), verts[:, 2].max().round(4))
    OUT.write_text(json.dumps(result, separators=(",", ":")), encoding="utf-8")
    print("wrote", OUT)

if __name__ == "__main__":
    main()
