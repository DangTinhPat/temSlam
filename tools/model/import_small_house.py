#!/usr/bin/env python3
"""Convert the AWS RoboMaker Small House SDF asset pack to an RB MuJoCo scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import re
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import trimesh


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = ROOT / "simulation/assets/aws_robomaker_small_house/source"
DEFAULT_BASE_MODEL = ROOT / "model/dynamic.xml"
DEFAULT_OUTPUT = ROOT / "simulation/scenarios/indoor_small_house.xml"
CONVERTED = ROOT / "simulation/assets/aws_robomaker_small_house/converted"
COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
METER_PER_COLLADA_UNIT = "meter"


def child_text(parent: ET.Element, path: str, default: str = "") -> str:
    node = parent.find(path)
    return node.text.strip() if node is not None and node.text else default


def floats(text: str, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    values = tuple(float(v) for v in text.split()) if text.strip() else default
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {text!r}")
    return values


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_").lower()


def quat_from_rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = q[1:]
    return v + 2 * np.cross(qv, np.cross(qv, v) + q[0] * v)


def world_pose(instance_pose: tuple[float, ...], local_pose: tuple[float, ...]) -> tuple[np.ndarray, np.ndarray]:
    p = np.asarray(instance_pose[:3], dtype=float)
    q = quat_from_rpy(instance_pose[3:])
    lp = np.asarray(local_pose[:3], dtype=float)
    lq = quat_from_rpy(local_pose[3:])
    return p + rotate(q, lp), quat_multiply(q, lq)


def pose_attrs(pos: np.ndarray, quat: np.ndarray) -> dict[str, str]:
    return {"pos": " ".join(f"{x:.9g}" for x in pos),
            "quat": " ".join(f"{x:.9g}" for x in quat)}


def dae_unit(dae_path: Path) -> float:
    root = ET.parse(dae_path).getroot()
    asset = root.find("c:asset", COLLADA_NS)
    unit = asset.find("c:unit", COLLADA_NS) if asset is not None else None
    return float(unit.get(METER_PER_COLLADA_UNIT, "1")) if unit is not None else 1.0


def dae_textures(dae_path: Path) -> list[Path]:
    root = ET.parse(dae_path).getroot()
    images = root.findall("c:library_images/c:image/c:init_from", COLLADA_NS)
    return [(dae_path.parent / (image.text or "")).resolve() for image in images]


def write_obj(mesh: trimesh.Trimesh, path: Path) -> None:
    text = mesh.export(file_type="obj")
    # MuJoCo gets the UV coordinates from OBJ; material definitions are supplied in MJCF.
    text = "\n".join(line for line in text.splitlines()
                      if not line.startswith(("mtllib ", "usemtl "))) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def collada_parts(dae_path: Path, scale: tuple[float, float, float]) -> list[tuple[trimesh.Trimesh, Path | None]]:
    scene = trimesh.load(dae_path, process=False)
    if not isinstance(scene, trimesh.Scene):
        scene = trimesh.Scene(scene)
    nodes = list(scene.graph.nodes_geometry)
    textures = dae_textures(dae_path)
    if textures and len(textures) != len(nodes):
        raise ValueError(f"{dae_path}: {len(nodes)} mesh parts, {len(textures)} textures; cannot map reliably")
    meter = dae_unit(dae_path)
    parts = []
    for index, node in enumerate(nodes):
        transform, geometry_name = scene.graph.get(node)
        mesh = scene.geometry[geometry_name].copy()
        mesh.apply_transform(transform)
        mesh.apply_scale(np.asarray(scale) * meter)
        texture = textures[index] if textures else None
        parts.append((mesh, texture))
    return parts


def scene_mesh(path: Path, scale: tuple[float, float, float]) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = mesh.to_geometry()
    mesh.apply_scale(np.asarray(scale) * dae_unit(path))
    return mesh


def diffuse_rgba(visual: ET.Element) -> str:
    raw = child_text(visual, "material/diffuse", "0.72 0.74 0.76 1")
    values = [float(x) for x in raw.split()]
    if len(values) == 3:
        values.append(1.0)
    return " ".join(f"{min(1.0, max(0.0, v)):.5g}" for v in values[:4])


def source_model_paths(source: Path, world_path: Path) -> tuple[ET.Element, list[tuple[ET.Element, str]]]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"no <world> in {world_path}")
    instances = []
    for model in world.findall("model"):
        include = model.find("include")
        uri = child_text(include, "uri") if include is not None else ""
        if not uri.startswith("model://"):
            continue
        model_name = uri[len("model://"):].rstrip("/")
        instances.append((model, model_name))
    return world, instances


def wall_collision_boxes(path: Path, scale: tuple[float, float, float]) -> list[tuple[np.ndarray, np.ndarray]]:
    mesh = scene_mesh(path, scale)
    panels = []
    for part in mesh.split(only_watertight=False):
        bounds = part.bounds
        size = bounds[1] - bounds[0]
        horizontal = size[:2]
        thin_axis = int(np.argmin(horizontal))
        long_axis = 1 - thin_axis
        if size[2] < 1.0 or size[long_axis] < 0.55 or size[thin_axis] > 0.08:
            continue
        panels.append([bounds[0].copy(), bounds[1].copy(), thin_axis, long_axis])

    # The source mesh models wall faces as separate thin surfaces. Pair close, overlapping faces
    # into one finite-thickness box so MuJoCo has stable collisions without filling room interiors.
    groups: list[list] = []
    for panel in sorted(panels, key=lambda p: (p[2], p[3], p[0][p[2]])):
        lo, hi, thin_axis, long_axis = panel
        match = None
        for group in groups:
            glo, ghi, gthin, glong = group[0], group[1], group[2], group[3]
            overlap = min(hi[long_axis], ghi[long_axis]) - max(lo[long_axis], glo[long_axis])
            z_overlap = min(hi[2], ghi[2]) - max(lo[2], glo[2])
            if (thin_axis == gthin and long_axis == glong and
                    abs((lo[thin_axis] + hi[thin_axis] - glo[thin_axis] - ghi[thin_axis]) / 2) < 0.28 and
                    overlap > 0.5 * min(size_of(lo, hi, long_axis), size_of(glo, ghi, long_axis)) and
                    z_overlap > 0.5 * min(hi[2] - lo[2], ghi[2] - glo[2])):
                match = group
                break
        if match is None:
            groups.append([lo.copy(), hi.copy(), thin_axis, long_axis])
        else:
            match[0] = np.minimum(match[0], lo)
            match[1] = np.maximum(match[1], hi)

    boxes = []
    for lo, hi, thin_axis, _ in groups:
        size = hi - lo
        size[thin_axis] = max(size[thin_axis], 0.08)
        boxes.append(((lo + hi) / 2, size / 2))
    return boxes


def size_of(lo: np.ndarray, hi: np.ndarray, axis: int) -> float:
    return float(hi[axis] - lo[axis])


def add_texture_asset(root: ET.Element, texture_path: Path, cache: dict[Path, tuple[str, str]]) -> tuple[str, str]:
    texture_path = texture_path.resolve()
    if texture_path in cache:
        return cache[texture_path]
    if texture_path.is_file():
        digest = hashlib.sha256(texture_path.read_bytes()).hexdigest()[:12]
    else:
        digest = hashlib.sha256(f"placeholder:{texture_path.name}".encode()).hexdigest()[:12]
    texture_name = f"house_tex_{digest}"
    material_name = f"house_mat_{digest}"
    asset = root.find("asset")
    if asset is not None and any(node.get("name") == texture_name for node in asset.findall("texture")):
        cache[texture_path] = texture_name, material_name
        return texture_name, material_name
    out = CONVERTED / "textures" / f"{digest}_{texture_path.stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    if texture_path.is_file():
        with Image.open(texture_path) as image:
            image = image.convert("RGB")
            image.thumbnail((1024, 1024), Image.Resampling.LANCZOS)
            image.save(out, format="PNG", optimize=True)
    else:
        write_placeholder_poster(out, texture_path.stem)
    rel = Path("../assets/aws_robomaker_small_house/converted/textures") / out.name
    ET.SubElement(asset, "texture", {"name": texture_name, "type": "2d", "file": rel.as_posix()})
    ET.SubElement(asset, "material", {"name": material_name, "texture": texture_name,
                                       "texuniform": "false", "specular": "0.1", "shininess": "0.1"})
    cache[texture_path] = texture_name, material_name
    return texture_name, material_name


def write_placeholder_poster(path: Path, seed_text: str) -> None:
    """Make a reproducible, high-feature poster where upstream references a portrait image."""
    seed = int(hashlib.sha256(seed_text.encode()).hexdigest()[:8], 16)
    rng = random.Random(seed)
    colors = [(35, 74, 112), (185, 89, 55), (229, 184, 89), (61, 132, 118),
              (119, 84, 146), (216, 214, 194)]
    image = Image.new("RGB", (256, 384), colors[rng.randrange(len(colors))])
    draw = ImageDraw.Draw(image)
    for _ in range(14):
        color = colors[rng.randrange(len(colors))]
        x0, y0 = rng.randrange(256), rng.randrange(384)
        x1, y1 = min(255, x0 + rng.randrange(20, 150)), min(383, y0 + rng.randrange(20, 150))
        if rng.random() < 0.5:
            draw.rectangle((x0, y0, x1, y1), fill=color)
        else:
            draw.ellipse((x0, y0, x1, y1), fill=color)
    for _ in range(18):
        color = colors[rng.randrange(len(colors))]
        points = [(rng.randrange(256), rng.randrange(384)) for _ in range(3)]
        draw.line(points, fill=color, width=rng.randrange(2, 8))
    image.save(path)


def build(source: Path, base_model: Path, output: Path) -> dict:
    source = source.resolve()
    world_path = source / "worlds/small_house.world"
    if not world_path.is_file():
        raise FileNotFoundError(f"expected upstream world at {world_path}")
    if not base_model.is_file():
        raise FileNotFoundError(f"base MuJoCo robot model not found: {base_model}")

    if CONVERTED.exists():
        shutil.rmtree(CONVERTED)
    (CONVERTED / "meshes").mkdir(parents=True)
    root = ET.parse(base_model).getroot()
    root.set("model", "rb_v1_3_aws_small_house")
    asset = root.find("asset")
    worldbody = root.find("worldbody")
    if asset is None or worldbody is None:
        raise ValueError("base model must contain <asset> and <worldbody>")

    visual = root.find("visual")
    if visual is None:
        visual = ET.Element("visual")
        root.insert(list(root).index(worldbody), visual)
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "1280")
    global_visual.set("offheight", "720")
    headlight = visual.find("headlight")
    if headlight is None:
        ET.SubElement(visual, "headlight", {"ambient": "0.32 0.32 0.32", "diffuse": "0.65 0.65 0.65",
                                             "specular": "0.1 0.1 0.1"})

    ground = next((g for g in worldbody.findall("geom") if g.get("name") == "ground"), None)
    if ground is not None:
        ground.set("rgba", "0.2 0.22 0.24 0")  # keep the contact plane, show the source floor mesh instead

    light_positions = [(-7.0, -3.5, 2.8), (0.0, -3.5, 2.8), (7.0, -3.5, 2.8),
                       (-7.0, 3.5, 2.8), (0.0, 3.5, 2.8), (7.0, 3.5, 2.8)]
    for i, pos in enumerate(light_positions):
        ET.SubElement(worldbody, "light", {"name": f"house_light_{i}",
                                           "pos": " ".join(f"{x:g}" for x in pos),
                                           "directional": "false", "diffuse": "0.55 0.55 0.55",
                                           "ambient": "0.06 0.06 0.06", "cutoff": "9", "exponent": "1"})

    world, instances = source_model_paths(source, world_path)
    models: dict[str, ET.Element] = {}
    for name in sorted({n for _, n in instances}):
        model_sdf = source / "models" / name / "model.sdf"
        if not model_sdf.is_file():
            raise FileNotFoundError(f"missing included model: {model_sdf}")
        models[name] = ET.parse(model_sdf).getroot().find("model")
        if models[name] is None:
            raise ValueError(f"no <model> in {model_sdf}")

    texture_cache: dict[Path, tuple[str, str]] = {}
    mesh_count = 0
    visual_count = 0
    collision_count = 0
    collision_geom_names: set[str] = set()
    collision_mesh_assets: set[str] = set()
    added_visual_groups: dict[str, list[list[tuple[str, str | None]]]] = {}
    for model_name, sdf_model in models.items():
        links = sdf_model.findall("link")
        if len(links) != 1:
            raise ValueError(f"{model_name}: expected one static link, found {len(links)}")
        model_visuals: list[list[tuple[str, str | None]]] = []
        for visual_index, item in enumerate(links[0].findall("visual")):
            geometry = item.find("geometry")
            mesh_node = geometry.find("mesh") if geometry is not None else None
            if mesh_node is None:
                model_visuals.append([])
                continue
            uri = child_text(mesh_node, "uri")
            dae_path = source / "models" / model_name / "meshes" / Path(uri).name
            mesh_scale = floats(child_text(mesh_node, "scale"), 3, (1.0, 1.0, 1.0))
            visual_parts = []
            for part_index, (mesh, texture_path) in enumerate(collada_parts(dae_path, mesh_scale)):
                mesh_name = f"house_{safe_name(model_name)}_v{visual_index}_p{part_index}"
                rel_mesh = Path("../assets/aws_robomaker_small_house/converted/meshes") / f"{mesh_name}.obj"
                write_obj(mesh, CONVERTED / "meshes" / f"{mesh_name}.obj")
                ET.SubElement(asset, "mesh", {"name": mesh_name, "file": rel_mesh.as_posix(), "inertia": "shell"})
                material_name = None
                if texture_path is not None:
                    _, material_name = add_texture_asset(root, texture_path, texture_cache)
                visual_parts.append((mesh_name, material_name))
                mesh_count += 1
            model_visuals.append(visual_parts)
        added_visual_groups[model_name] = model_visuals

    instances_added = 0
    for instance_index, (instance, model_name) in enumerate(instances):
        sdf_model = models[model_name]
        instance_pose = floats(child_text(instance, "pose"), 6, (0, 0, 0, 0, 0, 0))
        link = sdf_model.find("link")
        link_pose = floats(child_text(link, "pose"), 6, (0, 0, 0, 0, 0, 0))
        base_pose = tuple(np.asarray(instance_pose) + np.asarray(link_pose))
        visual_items = link.findall("visual")
        for visual_index, item in enumerate(visual_items):
            visual_meshes = added_visual_groups[model_name][visual_index]
            local_pose = floats(child_text(item, "pose"), 6, (0, 0, 0, 0, 0, 0))
            if model_name == "aws_robomaker_residential_FloorB_01":
                local_pose = (local_pose[0], local_pose[1], local_pose[2] + 0.004, *local_pose[3:])
            pos, quat = world_pose(base_pose, local_pose)
            for part_index, (mesh_name, material_name) in enumerate(visual_meshes):
                attrs = {"name": f"house_visual_{instance_index}_{visual_index}_{part_index}", "type": "mesh",
                         "mesh": mesh_name, "contype": "0", "conaffinity": "0", "group": "0"}
                attrs.update(pose_attrs(pos, quat))
                if material_name:
                    attrs["material"] = material_name
                else:
                    attrs["rgba"] = diffuse_rgba(item)
                ET.SubElement(worldbody, "geom", attrs)
                visual_count += 1

        for collision_index, item in enumerate(link.findall("collision")):
            geometry = item.find("geometry")
            if geometry is None:
                continue
            local_pose = floats(child_text(item, "pose"), 6, (0, 0, 0, 0, 0, 0))
            pos, quat = world_pose(base_pose, local_pose)
            mesh_node = geometry.find("mesh")
            if mesh_node is not None:
                uri = child_text(mesh_node, "uri")
                dae_path = source / "models" / model_name / "meshes" / Path(uri).name
                mesh_scale = floats(child_text(mesh_node, "scale"), 3, (1.0, 1.0, 1.0))
                if model_name == "aws_robomaker_residential_HouseWallB_01":
                    for box_index, (box_pos, box_halfsize) in enumerate(wall_collision_boxes(dae_path, mesh_scale)):
                        wall_pos = pos + rotate(quat, box_pos)
                        wall_quat = quat
                        attrs = {"name": f"house_wall_collision_{instance_index}_{box_index}", "type": "box",
                                 "size": " ".join(f"{x:.8g}" for x in box_halfsize),
                                 "contype": "2", "conaffinity": "1", "rgba": "0.1 0.1 0.1 0",
                                 "group": "3", "friction": "0.8 0.02 0.0001"}
                        attrs.update(pose_attrs(wall_pos, wall_quat))
                        ET.SubElement(worldbody, "geom", attrs)
                        collision_count += 1
                    continue
                if model_name == "aws_robomaker_residential_FloorB_01":
                    continue  # the RB model already has a contact plane; this mesh is visual only
                mesh = scene_mesh(dae_path, mesh_scale)
                mesh_name = f"house_{safe_name(model_name)}_c{collision_index}"
                if mesh_name not in collision_mesh_assets:
                    rel_mesh = Path("../assets/aws_robomaker_small_house/converted/meshes") / f"{mesh_name}.obj"
                    write_obj(mesh, CONVERTED / "meshes" / f"{mesh_name}.obj")
                    ET.SubElement(asset, "mesh", {"name": mesh_name, "file": rel_mesh.as_posix(), "inertia": "shell"})
                    collision_mesh_assets.add(mesh_name)
                geom_name = f"house_collision_{instance_index}_{collision_index}"
                if geom_name in collision_geom_names:
                    raise ValueError(f"duplicate collision geom name: {geom_name}")
                collision_geom_names.add(geom_name)
                attrs = {"name": geom_name, "type": "mesh", "mesh": mesh_name, "contype": "2",
                         "conaffinity": "1", "rgba": "0.1 0.1 0.1 0", "group": "3",
                         "friction": "0.8 0.02 0.0001"}
                attrs.update(pose_attrs(pos, quat))
                ET.SubElement(worldbody, "geom", attrs)
                collision_count += 1
            else:
                sphere = geometry.find("sphere")
                if sphere is not None:
                    radius = float(child_text(sphere, "radius", "0.1"))
                    attrs = {"name": f"house_collision_{instance_index}_{collision_index}", "type": "sphere",
                             "size": f"{radius:g}", "contype": "2", "conaffinity": "1",
                             "rgba": "0.1 0.1 0.1 0", "group": "3", "friction": "0.8 0.02 0.0001"}
                    attrs.update(pose_attrs(pos, quat))
                    ET.SubElement(worldbody, "geom", attrs)
                    collision_count += 1
        instances_added += 1

    # Robot geometry can contact house obstacles without enabling self-collision.
    for geom in worldbody.iter("geom"):
        if geom.get("name") in ("ground",) or geom.get("name", "").startswith("house_"):
            continue
        geom.set("contype", "1")
        geom.set("conaffinity", "0")

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    manifest = {"source": "aws-robotics/aws-robomaker-small-house-world@ros1",
                "world": "small_house.world", "model_instances": instances_added,
                "visual_mesh_parts": mesh_count, "visual_geoms": visual_count,
                "collision_geoms": collision_count,
                "textures": len({name for name, _ in texture_cache.values()}),
                "base_robot_model": str(base_model.relative_to(ROOT))}
    (CONVERTED / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help="unpacked upstream source directory")
    parser.add_argument("--base-model", type=Path, default=DEFAULT_BASE_MODEL, help="RB MuJoCo XML model")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="output MuJoCo scene XML")
    args = parser.parse_args()
    result = build(args.source, args.base_model, args.output)
    print(json.dumps({"output": str(args.output), **result}, indent=2))


if __name__ == "__main__":
    main()
