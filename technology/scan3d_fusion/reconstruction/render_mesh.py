#!/usr/bin/env python3
"""Render only the reconstructed mesh with matte clay shading."""
import argparse
import json
from pathlib import Path
from xml.sax.saxutils import quoteattr

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--elevation", type=float, default=-25, help="View elevation in degrees; -60 exposes room interiors")
    cli = parser.parse_args()
    metadata = cli.mesh.with_suffix('.json')
    pose_source = json.loads(metadata.read_text()).get('pose_source', '') if metadata.is_file() else ''
    planar_floor = 'observed/inferred floor' in pose_source
    mesh = trimesh.load(cli.mesh, force="mesh")
    center = mesh.bounds.mean(axis=0)
    extent = np.linalg.norm(mesh.extents)
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <visual><global offwidth="1000" offheight="760"/><quality shadowsize="2048"/>
        <headlight ambient=".35 .35 .35" diffuse=".65 .65 .65" specular="0 0 0"/></visual>
      <asset><texture type="skybox" builtin="gradient" rgb1=".12 .14 .17" rgb2=".07 .08 .10" width="128" height="128"/>
        <mesh name="scan" file={quoteattr(str(cli.mesh.resolve()))}/>
        <material name="clay" rgba=".72 .69 .63 1" specular=".05" shininess=".1"/></asset>
      <worldbody><light pos="0 0 6" directional="true" dir="-.4 -.6 -1" diffuse=".5 .5 .5"/>
        <geom type="mesh" mesh="scan" material="clay" contype="0" conaffinity="0"/></worldbody>
    </mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = center
    camera.distance = extent*1.05
    camera.elevation = cli.elevation
    renderer = mujoco.Renderer(model, height=760, width=1000)
    frames = []
    try:
        for angle in (135, -45, -120):
            camera.azimuth = angle
            renderer.update_scene(data, camera=camera)
            # Coplanar floor triangles cause shadow-map acne; RViz also displays these layers without shadows.
            if planar_floor:
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            frames.append(Image.fromarray(renderer.render().copy()))
    finally:
        renderer.close()
    canvas = Image.new("RGB", (1600, 840), "#171c23")
    canvas.paste(frames[0], (0, 55))
    canvas.paste(frames[1].resize((600, 456)), (1000, 20))
    canvas.paste(frames[2].resize((600, 456)), (1000, 405))
    draw = ImageDraw.Draw(canvas)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 24)
    small = ImageFont.truetype(font_path, 17)
    reference = cli.mesh.stem == "reference"
    estimated = pose_source.startswith('cuVSLAM')
    surface_matched = 'surface matching' in pose_source
    label = "DEPTH CHUẨN MÔ PHỎNG — ĐỐI CHIẾU" if reference else "RGB + ToF → DELTAR → MESH ĐẤT SÉT"
    if estimated:
        label = "RGB + ToF → cuVSLAM → MESH ĐẤT SÉT"
    if surface_matched:
        label = "RGB + ToF → GHÉP BỀ MẶT → MESH ĐẤT SÉT"
    draw.text((30, 23), label, font=font, fill="#e4e7ec")
    pose_label = "Pose ước lượng cuVSLAM" if estimated else "Pose mô phỏng đã biết"
    if surface_matched:
        pose_label = "Pose ghép bề mặt + wheel/IMU"
    surface_label = 'Sàn phẳng gồm cả vùng suy diễn; vật thể giữ mesh đo' if planar_floor else 'Phần khuất có thể còn hở'
    draw.text((30, 810), f"Tái dựng từ nhiều góc nhìn • {pose_label} • {surface_label}", font=small, fill="#b7c0cc")
    cli.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(cli.output)
    print(cli.output)


if __name__ == "__main__":
    main()
