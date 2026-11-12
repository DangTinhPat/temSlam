"""Process and IPC boundary for the native MuJoCo runner."""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import datetime
from typing import Dict, Optional

def workspace_root() -> Path:
    here = Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / "firmware").is_dir() and (candidate / "model").is_dir():
            return candidate
    raise RuntimeError("cannot locate the lumi-quantum workspace")

# Start poses: the lying homing pose (hip at its +135 deg stop, knee folded) needs STANDUP; the extended spawn pose
# is balanced by ENABLE.
START_POSES = {"Nằm (homing)": "home", "Đứng (spawn)": "spawn_extended"}

class NativeBackend:
    """Slow UI client; the native child owns the kHz control loop."""

    def __init__(self, root: Optional[Path] = None, rate_hz: int = 3000, start_key: str = "home"):
        self.root = Path(root or workspace_root())
        self.rate_hz = int(rate_hz)
        self.start_key = start_key
        self.process: Optional[subprocess.Popen[str]] = None
        self.lines: queue.Queue[str] = queue.Queue(maxsize=256)
        self.telemetry: Dict[str, str] = {}
        # RB1_SIM_LOG=<file>: append the telemetry (STATE) and the commands sent, for offline analysis.
        log_path = os.environ.get("RB1_SIM_LOG")
        self._sim_log = open(log_path, "a", buffering=1) if log_path else None
        self.telemetry_received_monotonic_s = 0.0
        self.view: Dict[str, str] = {}
        self.log = deque(maxlen=200)
        self._reader: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._viewer_process: Optional[subprocess.Popen[str]] = None
        self._camera_process: Optional[subprocess.Popen[str]] = None
        self._map_process = None
        self.map_session: Optional[Path] = None
        self._map_dropped = 0
        self._command_socket: Optional[socket.socket] = None
        self._command_port = int(os.environ.get("RB1_SIM_PORT", "37681"))
        self.live_scan = None
        self._ready = threading.Event()

    def runner_path(self) -> Path:
        candidates = []
        if os.environ.get("RB1_SIM_RUNNER"):
            candidates.append(Path(os.environ["RB1_SIM_RUNNER"]))
        candidates += [
            self.root / "firmware/build/bin/rb1_sim",
            self.root / "build/bin/rb1_sim",
        ]
        for path in candidates:
            if path.is_file() and path.stat().st_mode & 0o111:
                return path
        raise RuntimeError("rb1_sim is not built: cmake -S firmware -B firmware/build && "
                           "cmake --build firmware/build --target rb1_sim (or set RB1_SIM_RUNNER)")

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    @property
    def viewer_running(self) -> bool:
        return (self._viewer_process is not None and
                self._viewer_process.poll() is None)

    def start(self) -> None:
        with self._lock:
            if self.running:
                return
            runner = self.runner_path()
            # RB1_SIM_MODEL=<xml> overrides the official SLAM house scene.
            default_model = self.root / "simulation/scenarios/indoor_tiny_house.xml"
            if not default_model.is_file():
                default_model = self.root / "model/dynamic.xml"
            model = Path(os.environ.get("RB1_SIM_MODEL") or default_model)
            if not model.is_absolute():
                model = self.root / model
            if not model.is_file():
                raise RuntimeError(f"RB dynamic model is missing: {model} (python3 tools/model/generate_dynamic_model.py)")
            self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._ready.clear()
            command = [str(runner), "--model", str(model), "--rate", str(self.rate_hz),
                       "--command-port", str(self._command_port)]
            speed_factor = os.environ.get("RB1_SIM_SPEED_FACTOR")
            command += ["--speed-factor", speed_factor] if speed_factor else ["--realtime"]
            # Start keyframe: "home" (lying, limp until STANDUP) or "spawn_extended" (ENABLE balances it).
            command += ["--start-key", self.start_key]
            if os.environ.get("RB1_LIVE_SLAM") == "1":
                command += ["--capture-sync"]
            # RB1_SIM_NOISE=<scale>: synthetic sensor noise (the runner default is none: a noiseless robot sits
            # unnaturally still).
            if os.environ.get("RB1_SIM_NOISE"):
                command += ["--sensor-noise", os.environ["RB1_SIM_NOISE"]]
            # The MPU-6500 imperfections (model/noise.yaml) are on: a random chip per launch, or
            # RB1_SIM_IMU_UNIT=<n> to repeat one; RB1_SIM_IMU=0 gives the ideal IMU.
            if os.environ.get("RB1_SIM_IMU") != "0":
                import random
                unit = os.environ.get("RB1_SIM_IMU_UNIT") or str(random.randint(1, 1000))
                command += ["--imu-model", "1", "--imu-unit", unit]
                print(f"IMU: MPU-6500 model, unit {unit} (RB1_SIM_IMU_UNIT={unit} repeats it)", flush=True)
            # RB1_SIM_ARGS="--standup-set cagain=10": extra runner options for A/B trials (shell-split).
            if os.environ.get("RB1_SIM_ARGS"):
                import shlex
                command += shlex.split(os.environ["RB1_SIM_ARGS"])
            self.process = subprocess.Popen(
                command,
                cwd=self.root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
            self._reader = threading.Thread(target=self._read_output, daemon=True)
            self._reader.start()
            try:
                if not self._ready.wait(timeout=2.0):
                    raise RuntimeError("native runner did not open command transport")
                self._open_map_view()
                if os.environ.get("RB1_LIVE_SLAM") == "1":
                    sys.path.insert(0, str(self.root / "technology/scan3d_fusion/live"))
                    from session import LiveSession
                    self.live_scan = LiveSession(self.root, self.map_session)
                    if os.environ.get("RB1_HEADLESS") != "1":
                        self._open_mujoco_viewer(model)
            except Exception:
                self.stop()
                raise

    def _open_mujoco_viewer(self, model_path: Path) -> None:
        """Open the real MuJoCo 3-D renderer used by the native model.

        The C++ child remains the sole physics/control owner.  This Python
        viewer loads the identical MJCF and mirrors only the debug VIEW
        snapshot, so no viewer pose/contact data can enter RobotState or CMF.
        """
        viewer_script = self.root / "gui/rb1_gui/cad_viewer.py"
        if not viewer_script.is_file():
            raise RuntimeError(f"RB CAD viewer is missing: {viewer_script}")
        viewer_env = os.environ.copy()
        # This workspace runs on a hybrid-GPU laptop; DRI_PRIME selects the GTX 1650 for MuJoCo rendering.
        viewer_env.setdefault("DRI_PRIME", "1")
        self._viewer_process = subprocess.Popen(
            [sys.executable, str(viewer_script), "--model", str(model_path), "--start-key", self.start_key],
            cwd=self.root, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT, text=True, bufsize=1, env=viewer_env)
        os.set_blocking(self._viewer_process.stdin.fileno(), False)

    def _open_map_view(self) -> None:
        executable = self.runner_path().parent / "rb_tof_mapper"
        if not executable.is_file():
            raise RuntimeError("Native ToF mapper missing; rebuild rb1_sim")
        self.map_session = self.root / "datasets/raw" / (
            "tof-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
        self.map_session.mkdir(parents=True, exist_ok=True)
        self._map_process = subprocess.Popen(
            [sys.executable, str(self.root / "technology/scan3d_fusion/viewer/tof_map_viewer.py"),
             "--mapper", str(executable), "--runner-command", json.dumps(self.process.args),
             "--session", str(self.map_session), "--headless"], cwd=self.root, stdin=subprocess.PIPE,
            text=True, bufsize=1)
        os.set_blocking(self._map_process.stdin.fileno(), False)

    def _close_map_view(self) -> None:
        process = self._map_process
        self._map_process = None
        if process is None:
            return
        try:
            process.stdin.close()
            process.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def _open_camera_view(self, model_path: Path) -> None:
        script = self.root / "technology/scan3d_fusion/sensor/camera_viewer.py"
        self._camera_process = subprocess.Popen(
            [sys.executable, str(script), "--model", str(model_path)],
            cwd=self.root, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        os.set_blocking(self._camera_process.stdin.fileno(), False)

    def _sync_mujoco_viewer(self) -> None:
        if not self.viewer_running or self._viewer_process.stdin is None:
            return
        try:
            line = "VIEW|" + "|".join(
                f"{key}={value}" for key, value in self.view.items()) + "\n"
            packet = line.encode("ascii")
            if len(packet) <= 4096:
                os.write(self._viewer_process.stdin.fileno(), packet)
        except (BrokenPipeError, OSError):
            return

    def _sync_camera_view(self) -> None:
        process = self._camera_process
        if process is None or process.poll() is not None or process.stdin is None:
            return
        try:
            line = "VIEW|" + "|".join(
                f"{key}={value}" for key, value in self.view.items()) + "\n"
            packet = line.encode("ascii")
            if len(packet) <= 4096:
                os.write(process.stdin.fileno(), packet)
        except (BrokenPipeError, OSError):
            return

    def _close_camera_view(self) -> None:
        process = self._camera_process
        self._camera_process = None
        if process is None:
            return
        if process.stdin is not None:
            process.stdin.close()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def _close_mujoco_viewer(self) -> None:
        process = self._viewer_process
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
            process.wait(timeout=2.0)
        except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        finally:
            self._viewer_process = None

    def _read_output(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            if line.startswith("CAPTURE|"):
                if self.map_session is not None and self.map_session.is_dir():
                    packet = self._parse(line.strip())
                    with (self.map_session / "live_capture.jsonl").open("a") as stream:
                        stream.write(json.dumps(packet) + "\n")
                    target = self.map_session / "live_capture.json"
                    tmp = target.with_suffix(".tmp")
                    tmp.write_text(json.dumps(packet))
                    tmp.replace(target)
                continue
            if line.startswith(("TOFMAP ", "MAPRESET")):
                camera = self._camera_process
                if camera is not None and camera.poll() is None:
                    try:
                        packet = line.encode("ascii")
                        if len(packet) <= 4096:
                            os.write(camera.stdin.fileno(), packet)
                    except (OSError, ValueError):
                        pass
                mapper = self._map_process
                if mapper is not None and mapper.poll() is None:
                    try:
                        packet = line.encode("ascii")
                        if len(packet) > 4096:
                            raise OSError("ToF packet exceeds atomic pipe size")
                        os.write(mapper.stdin.fileno(), packet)
                    except (OSError, ValueError):
                        self._map_dropped += 1
                        if self._map_dropped % 15 == 1:
                            self.log.append(f"ToF map stream dropped {self._map_dropped} frames")
                continue
            if line.startswith("VIEW|"):
                mapper = self._map_process
                if mapper is not None and mapper.poll() is None:
                    try:
                        os.write(mapper.stdin.fileno(), line.encode("ascii"))
                    except (OSError, ValueError):
                        pass
            if line.startswith("READY|"):
                self._ready.set()
            line = line.rstrip("\n")
            try:
                self.lines.put_nowait(line)
            except queue.Full:
                try:
                    self.lines.get_nowait()
                except queue.Empty:
                    pass
                self.lines.put_nowait(line)

    @staticmethod
    def _parse(line: str) -> Dict[str, str]:
        return {part.split("=", 1)[0]: part.split("=", 1)[1]
                for part in line.split("|")[1:] if "=" in part}

    def drain(self) -> int:
        count = 0
        view_changed = False
        for _ in range(self.lines.qsize()):
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            count += 1
            if line.startswith("STATE|"):
                if self._sim_log is not None:
                    self._sim_log.write(line + "\n")
                self.telemetry = self._parse(line)
                self.telemetry_received_monotonic_s = time.monotonic()
            elif line.startswith("VIEW|"):
                self.view = self._parse(line)
                view_changed = True
            else:
                self.log.append(line)
        if view_changed:
            self._sync_mujoco_viewer()
            self._sync_camera_view()
        return count

    def send(self, command: str) -> None:
        if self._sim_log is not None:
            self._sim_log.write(f"CMD|{command}|t={self.telemetry.get('t', '')}\n")
        process = self.process
        if process is None or process.poll() is not None:
            return
        try:
            packet = (command.rstrip("\n") + "\n").encode("ascii")
            if self._command_socket is not None:
                # One datagram is one BLE-like command write; no controller
                # response is required on this path.
                self._command_socket.sendto(
                    packet, ("127.0.0.1", self._command_port))
            elif process.stdin is not None:
                process.stdin.write(packet.decode("ascii"))
                process.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def stop(self) -> None:
        self.send("STOP")
        if self.live_scan is not None:
            self.live_scan.close()
            self.live_scan = None
        self._close_map_view()
        self._close_mujoco_viewer()
        self._close_camera_view()
        process = self.process
        if process is None:
            return
        self.send("QUIT")
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.process = None
        if self._command_socket is not None:
            self._command_socket.close()
            self._command_socket = None

    def command_drive(self, velocity: float, yaw_rate: float) -> None:
        if math.isfinite(velocity) and math.isfinite(yaw_rate):
            self.send(f"DRIVE {velocity:.7g} {yaw_rate:.7g}")

    def command_stop(self) -> None:
        self.send("STOP")

    def command_pose(self, hip: float, knee: float,
                     right_hip: Optional[float] = None,
                     right_knee: Optional[float] = None) -> None:
        if right_hip is None:
            right_hip = hip
        if right_knee is None:
            right_knee = knee
        values = (hip, knee, right_hip, right_knee)
        if all(math.isfinite(value) for value in values):
            self.send("POSE " + " ".join(f"{value:.7g}" for value in values))

    def command_height(self, body_height_m: float) -> None:
        """Request a symmetric FK height target; C++ owns the inverse FK."""
        if math.isfinite(body_height_m):
            self.send(f"HEIGHT {body_height_m:.7g}")

    @property
    def starts_lying(self) -> bool:
        """The sim starts lying ("home"): ENABLE is not the way to start it, STANDUP is."""
        return self.start_key != "spawn_extended"

    def command_standup(self) -> None:
        """Start the autonomous fold / head drop / kneel / catch from the lying pose."""
        self.send("STANDUP")

    def command_push(self, force_n: float, duration_s: float = 0.1) -> None:
        """External push on the body (runner PUSH: |F| <= 50 N, 0 < t <= 2 s)."""
        if math.isfinite(force_n) and math.isfinite(duration_s):
            self.send(f"PUSH {force_n:.4g} {duration_s:.4g}")
