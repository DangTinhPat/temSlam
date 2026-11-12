"""Render-only RGB and ToF previews from the simulator's VIEW snapshots."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import sys
import tkinter as tk

from PIL import Image, ImageDraw, ImageTk
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]


def fields(line: bytes) -> dict[str, str]:
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.decode("ascii").split("|")[1:] if "=" in part}


def latest_view(buffer: bytes) -> tuple[bytes | None, bytes]:
    lines = buffer.split(b"\n")
    partial = lines.pop()
    latest = next((line for line in reversed(lines) if line.startswith(b"VIEW|")), None)
    return latest, partial if len(partial) <= 65536 else b""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    model = mujoco.MjModel.from_xml_path(str(parser.parse_args().model))
    rgb_config = yaml.safe_load((ROOT / "technology/scan3d_fusion/config/camera.yaml").read_text(encoding="utf-8"))["camera"]
    rgb_width, rgb_height = map(int, rgb_config["resolution_px"])
    depth_width, depth_height = rgb_width // 2, rgb_height // 2
    tof_config = yaml.safe_load((ROOT / "technology/scan3d_fusion/config/tof.yaml").read_text(encoding="utf-8"))["sensor"]
    tof_min_range = float(tof_config["min_range_m"])
    tof_max_range = float(tof_config["max_range_m"])
    tof_period = 1.0 / float(tof_config["frame_rate_hz"])
    tof_noise = tof_config["noise"]
    noise_rng = np.random.default_rng()
    data = mujoco.MjData(model)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    tof_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "tof_camera")
    if camera_id < 0 or tof_camera_id < 0:
        raise RuntimeError("model must contain head_camera and tof_camera")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body_link")

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = camera_id
    renderer = mujoco.Renderer(model, height=rgb_height, width=rgb_width)
    tof_fovy = float(model.cam_fovy[tof_camera_id])

    root = tk.Tk()
    root.title(f"RB · RGB {rgb_width}x{rgb_height}")
    root.geometry(f"{rgb_width}x{rgb_height + 30}+12+12")
    root.attributes("-topmost", True)
    root.resizable(False, False)
    image_label = tk.Label(root, bg="#111318")
    image_label.pack()
    status = tk.Label(root, text="Waiting for simulator…", anchor="w", padx=8)
    status.pack(fill="x")

    tof_window = tk.Toplevel(root)
    tof_window.title("RB - ToF 8x8")
    tof_window.geometry(f"344x360+{rgb_width + 70}+12")
    tof_window.attributes("-topmost", True)
    tof_window.resizable(False, False)
    tof_label = tk.Label(tof_window, bg="#111318")
    tof_label.pack(padx=8, pady=(8, 0))
    tof_status = tk.Label(tof_window, text="8x8 | waiting for simulator...", anchor="w", padx=8)
    tof_status.pack(fill="x")
    rgbd_window = tk.Toplevel(root)
    rgbd_window.title("RB · RGB-D (simulated) · RGB | depth")
    rgbd_window.geometry(f"640x280+12+{rgb_height + 70}")
    rgbd_window.attributes("-topmost", True)
    rgbd_window.resizable(False, False)
    rgbd_label = tk.Label(rgbd_window, bg="#111318")
    rgbd_label.pack(padx=4, pady=(4, 0))
    rgbd_status = tk.Label(rgbd_window, text="RGB-D | waiting for simulator...", anchor="w", padx=8)
    rgbd_status.pack(fill="x")
    rgbd_window.protocol("WM_DELETE_WINDOW", rgbd_window.withdraw)

    open_window = True
    root.protocol("WM_DELETE_WINDOW", lambda: stop())

    def stop() -> None:
        nonlocal open_window
        open_window = False

    pending = b""
    input_fd = sys.stdin.fileno()
    measured_tof = None
    last_tof_update = float("-inf")
    while open_window:
        if select.select([input_fd], [], [], 0.02)[0]:
            chunk = os.read(input_fd, 65536)
            if not chunk:
                break
            complete = (pending + chunk).split(b"\n")
            pending = complete.pop()
            if len(pending) > 65536:
                pending = b""
            for record in complete:
                if record.startswith(b"TOFMAP "):
                    parts = record.split()
                    if len(parts) in (76, 77):
                        try:
                            offset = 13 if len(parts) == 77 else 12
                            measured_tof = (float(parts[1]), [float(v) or None for v in parts[offset:]])
                        except ValueError:
                            pass
            line = next((v for v in reversed(complete) if v.startswith(b"VIEW|")), None)
            if line is not None:
                values = fields(line)
                try:
                    for index, key in enumerate(("x", "y", "z", "qw", "qx", "qy", "qz")):
                        data.qpos[index] = float(values[key])
                    for key, name in (("lh", "left_hip"), ("lk", "left_knee"), ("lw", "left_wheel"),
                                      ("rh", "right_hip"), ("rk", "right_knee"), ("rw", "right_wheel"),
                                      ("hj", "head_yaw")):
                        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                        if joint >= 0:
                            address = model.jnt_qposadr[joint]
                            data.qpos[address] = float(values.get(key, data.qpos[address]))
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=camera)
                    rgb_frame = renderer.render()
                    rgb = ImageTk.PhotoImage(Image.fromarray(rgb_frame))
                    image_label.configure(image=rgb)
                    image_label.image = rgb
                    sim_time = float(values["t"])
                    status.configure(text=f"RGB {rgb_width}x{rgb_height} · t={sim_time:.2f} s")

                    if sim_time - last_tof_update >= tof_period and measured_tof is not None:
                        renderer.enable_depth_rendering()
                        renderer.update_scene(data, camera=camera)
                        depth = renderer.render().copy()
                        renderer.disable_depth_rendering()
                        depth = np.asarray(
                            Image.fromarray(depth.astype(np.float32), mode="F").resize(
                                (depth_width, depth_height), Image.Resampling.NEAREST)).copy()
                        valid_depth = (np.isfinite(depth) & (depth >= tof_min_range) &
                                       (depth <= tof_max_range))
                        measured_depth = depth[valid_depth]
                        accuracy = np.where(
                            measured_depth <= 0.2,
                            float(tof_noise["near_accuracy_m"]),
                            measured_depth * float(tof_noise["far_accuracy_fraction"]))
                        sigma = accuracy * float(tof_noise["sigma_fraction_of_bound"])
                        # shortcut: independent pixel noise ignores fusion correlation; replace when measured data exists.
                        error = np.clip(noise_rng.normal(0.0, sigma), -accuracy, accuracy)
                        depth[valid_depth] = np.clip(measured_depth + error,
                                                     tof_min_range, tof_max_range)
                        gray = np.zeros(depth.shape, dtype=np.uint8)
                        gray[valid_depth] = (
                            255.0 * (1.0 - depth[valid_depth] / tof_max_range)).astype(np.uint8)
                        panel_size = (rgb_width // 2, rgb_height // 2)
                        rgbd_panel = Image.new("RGB", (panel_size[0] * 2, panel_size[1]), "#111318")
                        rgbd_panel.paste(Image.fromarray(rgb_frame).resize(panel_size), (0, 0))
                        depth_image = Image.fromarray(gray, mode="L").convert("RGB")
                        rgbd_panel.paste(depth_image, (panel_size[0], 0))
                        panel_draw = ImageDraw.Draw(rgbd_panel)
                        panel_draw.line((panel_size[0], 0, panel_size[0], panel_size[1]), fill="#77808d")
                        panel_draw.text((8, 6), "RGB", fill="white")
                        panel_draw.text((panel_size[0] + 8, 6), "Depth (m)", fill="white")
                        rgbd_photo = ImageTk.PhotoImage(rgbd_panel)
                        rgbd_label.configure(image=rgbd_photo)
                        rgbd_label.image = rgbd_photo
                        rgbd_status.configure(
                            text=f"Simulated RGB-D · RGB {rgb_width}×{rgb_height} · "
                                 f"depth {depth_width}×{depth_height} · "
                                 f"valid {int(np.count_nonzero(valid_depth))}/{depth.size} px · "
                                 f"t={sim_time:.2f} s")
                        tof_time, depths = measured_tof

                        cell = 40
                        tof_image = Image.new("RGB", (8 * cell, 8 * cell), "#111318")
                        draw = ImageDraw.Draw(tof_image)
                        for index, distance in enumerate(depths):
                            row, column = divmod(index, 8)
                            x0, y0 = column * cell, row * cell
                            if distance is None:
                                color, label = (24, 28, 34), "--"
                            else:
                                fraction = (distance - tof_min_range) / (tof_max_range - tof_min_range)
                                color = (int(255 * (1 - fraction)),
                                         int(255 * (1 - abs(2 * fraction - 1))),
                                         int(255 * fraction))
                                label = f"{distance:.1f}"
                            draw.rectangle((x0, y0, x0 + cell - 1, y0 + cell - 1),
                                           fill=color, outline="#252a31")
                            draw.text((x0 + 8, y0 + 15), label, fill="white")
                        photo = ImageTk.PhotoImage(tof_image)
                        tof_label.configure(image=photo)
                        tof_label.image = photo
                        tof_status.configure(
                            text=f"ToF 8x8 | {tof_min_range:g}-{tof_max_range:g} m | "
                                 f"FOV {tof_fovy:g} deg | 15 Hz | t={tof_time:.2f} s")
                        last_tof_update = sim_time
                except (KeyError, TypeError, ValueError, RuntimeError):
                    pass
        try:
            root.update_idletasks()
            root.update()
        except tk.TclError:
            break
    renderer.close()
    root.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
