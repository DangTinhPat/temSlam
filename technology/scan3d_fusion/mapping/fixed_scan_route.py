"""Follow the precomputed indoor-house scan path; no online route planning."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time


class FixedScanRoute:
    def __init__(self, backend, route_file: Path):
        self.backend = backend
        self.route = json.loads(Path(route_file).read_text(encoding="utf-8"))
        self.waypoints = self.route["waypoints"]
        self.index = 0
        self.phase = "idle"
        self.started = 0.0
        self.phase_started = 0.0
        self.head_commanded = 0.0
        self.head_command_at = 0.0
        self.still_since = None
        self.last_pose = None
        self.status = "Sẵn sàng quét map"
        self.wait_ready_since = None
        self.last_checkpoint = None
        self._last_diagnostic = 0.0
        self._turning = False
        self._drive_index = None
        self._sweep_progress_at = 0.0
        self._sweep_best_angle = -math.pi

    def _now(self):
        return float(self.backend.view.get("t", time.monotonic()))

    def _record(self):
        session = getattr(self.backend, "map_session", None)
        now = time.monotonic()
        if session is None or now-self._last_diagnostic < .5:
            return
        self._last_diagnostic = now
        packet = {"phase": self.phase, "index": self.index, "status": self.status,
                  "sim_time": self._now(), "controller": dict(self.backend.telemetry),
                  "view": dict(self.backend.view)}
        target = Path(session)/"route_state.json"
        try:
            tmp = target.with_suffix(".tmp")
            tmp.write_text(json.dumps(packet), encoding="utf-8")
            tmp.replace(target)
        except OSError as exc:
            self.backend.log.append(f"ROUTE_LOG_ERROR|{exc}")

    @property
    def active(self):
        return self.phase != "idle"

    def start(self, enabled: bool, estop: bool) -> bool:
        args = self.backend.process.args if self.backend.process is not None else []
        try:
            model_path = Path(args[args.index("--model") + 1]).resolve()
            route_scene = (self.backend.root / self.route["scene"]).resolve()
        except (ValueError, IndexError):
            self.status = "Không xác định được scene của mô phỏng"
            return False
        if model_path != route_scene:
            self.status = "Tuyến này chỉ dành cho indoor_tiny_house.xml"
            return False
        view = self.backend.view
        try:
            x, y = float(view["x"]), float(view["y"])
            yaw = self._yaw(view)
        except (KeyError, TypeError, ValueError):
            self.status = "Chưa có pose mô phỏng"
            return False
        fields = self.backend.telemetry
        if (not self.backend.running or not enabled or estop or
                fields.get("drive_ready") != "1"):
            self.status = "Cần bật RB và chờ trạng thái cân bằng ổn định"
            return False
        checkpoint = self.last_checkpoint
        if checkpoint and checkpoint['session'] == str(getattr(self.backend, 'map_session', '')):
            old = checkpoint['view']
            if 'x' not in old or 'y' not in old:
                self.status = "Thiếu pose tại điểm dừng; không thể nối tuyến an toàn"
                return False
            if math.hypot(x-float(old['x']), y-float(old['y'])) > .3:
                self.status = "RB đã rời vị trí dừng; không thể nối tuyến an toàn"
                return False
            self.index = checkpoint['index']
            self.phase = 'drive' if checkpoint['phase'] in ('drive', 'wait_ready') else 'settle_base'
            self.started = self.phase_started = self._now()
            self.still_since = self.wait_ready_since = None
            self.head_commanded = float(view.get('hj', 0))
            self.head_command_at = self._now()
            self.last_checkpoint = None
            self._drive_index = None
            self._turning = False
            self._progress("Tiếp tục tuyến quét")
            return True
        if math.hypot(x, y) > .6 or abs(yaw) > .20:
            self.status = "Tuyến cố định cần RB gần điểm đầu (0, 0), hướng +X"
            return False
        self.index = 0
        self.last_checkpoint = None
        self._drive_index = None
        self._turning = False
        self.phase, self.started = "drive", self._now()
        self.head_commanded = float(view.get("hj", "0"))
        self.head_command_at = self.started
        self.status = "Đang chạy tuyến quét cố định"
        return True

    def stop(self, message="Đã dừng tuyến quét"):
        if self.active:
            self.last_checkpoint = {"phase": self.phase, "index": self.index,
                                    "session": str(getattr(self.backend, "map_session", "")),
                                    "view": dict(self.backend.view)}
        if self.active and self.backend.running:
            self.backend.send("STOP")
            try:
                self.backend.send(f"HEAD {float(self.backend.view['hj']):.7f}")
            except (KeyError, TypeError, ValueError):
                pass
        self.phase = "idle"
        self.status = message
        self._last_diagnostic = 0
        self._record()

    def update(self, enabled: bool, estop: bool):
        if not self.active:
            return self.status
        live = getattr(self.backend, 'live_scan', None)
        if live is not None:
            failure = live.failure_reason()
            if failure:
                self.stop("Tuyến quét dừng: " + failure)
                return self.status
        fields, view = self.backend.telemetry, self.backend.view
        if (not self.backend.running or not enabled or estop or
                fields.get("mode") not in ("Balance", "Balancing", "Drive", "PoseChange")):
            self.stop("Tuyến quét dừng: RB mất trạng thái cân bằng/điều khiển")
            return self.status
        if self.phase == "drive" and fields.get("drive_ready") != "1":
            self.backend.send("STOP")
            self.phase = "wait_ready"
            self.phase_started = self._now()
            self.wait_ready_since = None
        if self.phase == "wait_ready":
            self.backend.send("STOP")
            if fields.get("drive_ready") == "1":
                if self.wait_ready_since is None:
                    self.wait_ready_since = self._now()
                elif self._now()-self.wait_ready_since >= .3:
                    self.phase = "drive"
                    self._drive_index = None
            else:
                self.wait_ready_since = None
            if self._now()-self.phase_started > 30:
                self.stop("Tuyến quét dừng: controller chưa sẵn sàng sau 30 giây mô phỏng")
                return self.status
            if self.phase == "wait_ready":
                return self._progress("Tạm chờ controller sẵn sàng; giữ nguyên điểm tuyến")
        try:
            x, y = float(view["x"]), float(view["y"])
            yaw = self._yaw(view)
            head = float(view.get("hj", "0"))
        except (KeyError, TypeError, ValueError):
            self.stop("Tuyến quét dừng: mất pose RB")
            return self.status

        if self.phase == "position_head":
            target = -math.radians(180)
            self._ramp_head(target, math.radians(45), head)
            if abs(head - target) < .06:
                self.phase = "sweep_head"
                self.phase_started = self._now()
                self._sweep_progress_at = time.monotonic()
                self._sweep_best_angle = -math.pi
            elif self._now() - self.phase_started > 8:
                self.stop("Tuyến quét dừng: đầu không tới được góc bắt đầu")
                return self.status
            return self._progress("Đặt đầu về −180°")
        if self.phase == "settle_base":
            velocity = abs(float(fields.get("ground_velocity", "inf")))
            pitch_rate = abs(float(fields.get("pitch_rate", "inf")))
            if velocity < .025 and pitch_rate < .12:
                if self.still_since is None:
                    self.still_since = self._now()
                elif self._now() - self.still_since >= .30:
                    self.still_since = None
                    self.phase = "position_head"
                    self.phase_started = self._now()
                    self.head_commanded = head
                    self.head_command_at = self.phase_started
            else:
                self.still_since = None
            if self._now() - self.started > 5:
                self.stop("Tuyến quét dừng: RB không đứng yên được tại điểm quét")
                return self.status
            return self._progress("Chờ RB đứng yên")
        if self.phase == "sweep_head":
            target = math.radians(180)
            step = self.waypoints[self.index].get('scan_step_deg')
            captured = True
            limit = target
            if step and live is not None:
                feedback = live.scan_feedback()
                if feedback is None or feedback['sensor_time'] < self.phase_started:
                    if time.monotonic()-self._sweep_progress_at > 180:
                        self.stop('Tuyến quét dừng: quá hạn chờ góc fusion mới tại điểm quét dày')
                    return self._progress('Chờ ảnh fusion trước khi lia tiếp')
                processed=float(feedback['head_yaw_rad'])
                if processed>self._sweep_best_angle+math.radians(1):
                    self._sweep_best_angle=processed
                    self._sweep_progress_at=time.monotonic()
                limit = min(target,processed+math.radians(step))
                captured = abs(processed-target)<math.radians(step)
            self._ramp_head(limit, math.radians(20), head)
            if abs(head - target) < .06 and captured:
                self.phase = "return_head"
                self.phase_started = self._now()
            elif ((not step or live is None) and self._now() - self.phase_started > 28) or (
                    step and live is not None and time.monotonic()-self._sweep_progress_at>180):
                self.stop("Tuyến quét dừng: đầu không hoàn thành lượt lia")
                return self.status
            else:
                return self._progress(f"Đang lia ToF {math.degrees(head):.0f}°")

        if self.phase == "return_head":
            self._ramp_head(0.0, math.radians(45), head)
            if abs(head) < .06:
                self.index += 1
                self.phase = "drive"
            elif self._now() - self.phase_started > 8:
                self.stop("Tuyến quét dừng: đầu không hồi về hướng thẳng")
                return self.status
            else:
                return self._progress("Đưa đầu về hướng thẳng")

        if self.index >= len(self.waypoints):
            self.backend.send("STOP")
            self._send_head(0.0)
            self.phase = "idle"
            self.status = "Đã hoàn tất tuyến quét map"
            return self.status

        while self.index < len(self.waypoints):
            point = self.waypoints[self.index]
            distance = math.hypot(point["x"] - x, point["y"] - y)
            if distance >= .09:
                break
            if point.get("scan"):
                self.backend.send("STOP")
                self.phase = "settle_base"
                self.started = self._now()
                self.still_since = None
                return self._progress("Dừng quét ToF")
            self.index += 1
        if self.index >= len(self.waypoints):
            self.backend.send("STOP")
            self.phase = "idle"
            self.status = "Đã hoàn tất tuyến quét map"
            self._record()
            return self.status

        desired = math.atan2(point["y"] - y, point["x"] - x)
        error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
        now = self._now()
        if self._drive_index != self.index:
            self._drive_index = self.index
            self._best_distance, self._best_heading = distance, abs(error)
            self._progress_at = now
        if distance < self._best_distance - .03 or abs(error) < self._best_heading - .08:
            self._best_distance, self._best_heading = distance, abs(error)
            self._progress_at = now
        if now - self._progress_at > 12:
            self.stop("Tuyến quét dừng: không tiến triển trong 12 giây; kiểm tra vật cản/controller")
            return self.status
        # Hysteresis prevents alternating zero/forward commands near a single angle threshold.
        if abs(error) > .65:
            self._turning = True
        elif abs(error) < .15:
            self._turning = False
        speed_limit = max(0.0,min(.20,float(point.get('speed_m_s',.20))))
        speed = 0.0 if self._turning else min(speed_limit, .85 * distance) * max(0.0, 1 - (error / .65)**2)
        self.backend.command_drive(speed, max(-.55, min(.55, 1.5 * error)))
        return self._progress(f"{'Căn hướng' if self._turning else 'Di chuyển tuyến'} · {distance:.1f} m tới điểm kế")

    def _send_head(self, angle):
        self.backend.send(f"HEAD {angle:.7f}")
        self.head_command_at = self._now()
        self.head_commanded = angle

    def _ramp_head(self, target, speed, measured):
        now = self._now()
        dt = now - self.head_command_at
        if dt < 1 / 30 or abs(self.head_commanded - measured) > math.radians(8):
            return
        step = min(speed * min(dt, .5), math.radians(8))
        delta = max(-step, min(step, target - self.head_commanded))
        if abs(delta) > 1e-6:
            self._send_head(self.head_commanded + delta)

    def _progress(self, action):
        if not self.active:
            return self.status
        self.status = f"{action} · điểm {min(self.index + 1, len(self.waypoints))}/{len(self.waypoints)}"
        self._record()
        return self.status

    @staticmethod
    def _yaw(view):
        qw, qx, qy, qz = (float(view[k]) for k in ("qw", "qx", "qy", "qz"))
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
