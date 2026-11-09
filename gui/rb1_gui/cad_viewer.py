"""Render-only MuJoCo viewer: the RB v1.3 CAD meshes riding on the dynamic.xml model.

The C++ runner owns the physics and the control loop. This process loads the same MJCF, swaps the collision
primitives for the CAD meshes (visual only) and renders the latest VIEW snapshot it reads on stdin. It never
publishes anything back to the runner.

Meshes: model/assets/mesh_NN_00.stl (body, head, PCB, battery, motors, ...) and
model/assets/rb_parts/{left,right}_{body,thigh,crank,coupler,shin,wheel}.stl (the leg parts, split from
the STEP assembly by part name). The crank and the coupler of the parallelogram are not bodies of the dynamic model;
they are drawn on mocap bodies from the 4-bar kinematics (the serial hip/knee model is exact for the other parts).
Without the part meshes the viewer falls back to the collision primitives.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import sys

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "model/assets"
PARTS = ASSETS / "rb_parts"
HIP_Y = 0.0812                          # |hip y| in the body frame
CRANK_PIN = np.array([-0.0155, 0.0125])  # crank pin relative to the hip axis (x, z), from the CAD link ends
# CAD part origin offsets (the STL parts are exported in the assembly frame)
OFFSETS = {"left": {"thigh": (0, .0812, 0), "shin": (-.0999987, .1024, -.0005138), "wheel": (-.0091523, .1201, -.0423091)},
           "right": {"thigh": (0, -.0812, 0), "shin": (-.1, -.1024, 0), "wheel": (-.0093693, -.1200998, -.0422621)}}
COLORS = {"body": (.76, .68, .48), "head": (.70, .54, .38), "thigh": (.35, .52, .49), "crank": (.80, .45, .20),
          "coupler": (.85, .75, .25), "shin": (.30, .45, .62), "wheel": (.10, .10, .11), "pcb": (.55, .48, .62),
          "battery": (.62, .60, .40), "motor": (.58, .37, .36), "metal": (.42, .47, .51), "belt": (.5, .44, .32)}
BODY_MESHES = {3: "pcb", 4: "head", 5: "body", 6: "metal", 7: "metal", 8: "motor", 9: "motor", 10: "belt", 11: "belt",
               12: "metal", 13: "battery", 14: "body", 15: "metal", 16: "body", 17: "body"}

def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1] for part in line.split("|")[1:] if "=" in part}

def latest_view(buffer):
    lines = buffer.split(b"\n")
    partial = lines.pop()
    latest = next((line for line in reversed(lines) if line.startswith(b"VIEW|")), None)
    return latest, partial if len(partial) <= 65536 else b""

def ry(a):
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])

class RbModel:
    def __init__(self, model_path: Path, start_key: str, meshes: bool = True):
        model_path = Path(model_path).resolve()
        spec = mujoco.MjSpec.from_file(str(model_path))
        self.house_scene = any(geom.name.startswith("house_visual_") for geom in spec.geoms)
        # In the house scene, simple robot collision geoms render faster than the full STEP-derived CAD overlay.
        self.meshes = (meshes and not self.house_scene and PARTS.is_dir()
                       and (PARTS / "left_thigh.stl").is_file())
        if self.meshes:
            # The CAD parts below use model/assets as meshdir. Preserve the scene's own mesh/texture paths first.
            for mesh in spec.meshes:
                if mesh.name.startswith("house_") and mesh.file:
                    mesh_path = Path(mesh.file)
                    if not mesh_path.is_absolute():
                        mesh.file = str((model_path.parent / mesh_path).resolve())
            for texture in spec.textures:
                if texture.name.startswith("house_tex_") and texture.file:
                    texture_path = Path(texture.file)
                    if not texture_path.is_absolute():
                        texture.file = str((model_path.parent / texture_path).resolve())
            spec.meshdir = str(ASSETS)
            for geom in spec.geoms:        # hide collision proxies; keep house visuals and CAD meshes visible
                if geom.name == "ground":
                    continue
                geom.group = 1 if geom.name.startswith("house_visual_") else 3
            self._add_cad(spec)
        self._ground_grid(spec)
        self._lighting(spec, shadows=not self.house_scene)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, start_key)
        if key >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        self.apply({})

    @staticmethod
    def _ground_grid(spec):
        """A checkered floor: with the camera following the robot a plain floor gives no sense of motion."""
        texture = spec.add_texture(name="rb_grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                                   builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[.64, .68, .74], rgb2=[.78, .81, .87],
                                   width=512, height=512)
        material = spec.add_material(name="rb_grid")
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture.name
        material.texrepeat = [40, 40]
        material.reflectance = 0.0
        for geom in spec.geoms:
            if geom.name == "ground":
                geom.material = "rb_grid"

    @staticmethod
    def _lighting(spec, shadows=True):
        """Bright, even light: a strong headlight, a sun with a shadow, and a sky gradient instead of black."""
        headlight = spec.visual.headlight
        headlight.ambient, headlight.diffuse, headlight.specular = [.45, .45, .45], [.70, .70, .70], [.2, .2, .2]
        sun = spec.worldbody.add_light()
        sun.pos, sun.dir = [0.3, -0.5, 2.0], [-0.1, 0.25, -1.0]
        sun.diffuse, sun.specular = [.85, .85, .85], [.3, .3, .3]
        sun.castshadow = shadows
        sky = spec.add_texture(name="rb_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                               builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[.72, .82, .95], rgb2=[.35, .42, .55],
                               width=512, height=3072)
        del sky

    @staticmethod
    def _add_cad(spec):
        def add_mesh(name, rel):
            mesh = spec.add_mesh()
            mesh.name, mesh.file, mesh.scale = name, rel, [.001] * 3

        def add_geom(body, mesh, color, offset):
            geom = body.add_geom()
            geom.type, geom.meshname = mujoco.mjtGeom.mjGEOM_MESH, mesh
            geom.pos = [-o for o in offset]
            geom.rgba = list(color) + [1]
            geom.contype = geom.conaffinity = 0
            geom.group = 1

        link = spec.body("body_link")
        for index, color in BODY_MESHES.items():
            add_mesh(f"m{index}", f"mesh_{index:02d}_00.stl")
            add_geom(link, f"m{index}", COLORS[color], (0, 0, 0))
        for side in ("left", "right"):
            add_mesh(f"{side}_body", f"rb_parts/{side}_body.stl")
            add_geom(link, f"{side}_body", COLORS["motor"], (0, 0, 0))
            for part in ("thigh", "shin", "wheel"):
                add_mesh(f"{side}_{part}", f"rb_parts/{side}_{part}.stl")
                add_geom(spec.body(f"{side}_{part}"), f"{side}_{part}", COLORS[part], OFFSETS[side][part])
            for part in ("crank", "coupler"):
                add_mesh(f"{side}_{part}", f"rb_parts/{side}_{part}.stl")
                body = spec.worldbody.add_body()
                body.name, body.mocap = f"{side}_{part}_vis", True
                add_geom(body, f"{side}_{part}", COLORS[part], (0, 0, 0))

    def _set_mocap(self, name, rotation, position):
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        mocap = self.model.body_mocapid[body]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
        self.data.mocap_quat[mocap], self.data.mocap_pos[mocap] = quat, position

    def _linkage(self):
        m, d = self.model, self.data
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body_link")
        p, r = d.xpos[body], d.xmat[body].reshape(3, 3)
        pin = np.array([CRANK_PIN[0], 0, CRANK_PIN[1]])
        for side, y in (("left", HIP_Y), ("right", -HIP_Y)):
            theta = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_hip")]]
            knee = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_knee")]]
            phi = theta + knee
            hip = np.array([0, y, 0])
            self._set_mocap(f"{side}_crank_vis", r @ ry(phi), p + r @ (hip - ry(phi) @ hip))      # rigid with the crank angle
            shift = (ry(phi) - ry(theta)) @ pin                                                     # coupler: parallel to the thigh
            self._set_mocap(f"{side}_coupler_vis", r @ ry(theta), p + r @ (hip - ry(theta) @ hip + shift))

    def apply(self, values):
        q = self.data.qpos
        for index, key in enumerate(("x", "y", "z", "qw", "qx", "qy", "qz", "lh", "lk", "lw", "rh", "rk", "rw")):
            q[index] = float(values.get(key, q[index]))
        mujoco.mj_forward(self.model, self.data)
        if self.meshes:
            self._linkage()
            mujoco.mj_kinematics(self.model, self.data)

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--start-key", default="home")
    parser.add_argument("--primitives", action="store_true", help="draw the collision primitives instead of the CAD")
    args = parser.parse_args(argv)
    rb = RbModel(args.model, args.start_key, meshes=not args.primitives)
    handle = mujoco.viewer.launch_passive(rb.model, rb.data, show_left_ui=False, show_right_ui=False)
    with handle.lock():
        # Tracking camera: it follows the body in x, y and z, and the mouse still orbits and zooms around it.
        handle.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        handle.cam.trackbodyid = mujoco.mj_name2id(rb.model, mujoco.mjtObj.mjOBJ_BODY, "body_link")
        if rb.house_scene:
            handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = 8.0, 135, -35
        else:
            handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = 0.9, 90, -10
    handle.sync()
    pending = b""
    fd = sys.stdin.fileno()
    try:
        while handle.is_running():
            if not select.select([fd], [], [], .05)[0]:
                continue
            chunk = os.read(fd, 65536)
            if not chunk:
                break
            line, pending = latest_view(pending + chunk)
            if line is None:
                continue
            try:
                with handle.lock():
                    values = fields(line.decode("ascii"))
                    rb.apply(values)
                handle.sync()
            except (TypeError, ValueError, RuntimeError):
                continue                                       # a malformed snapshot only skips a frame
    finally:
        handle.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
