"""Follow the precomputed indoor-house scan path; no online route planning."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

SCAN_ANGULAR_SPEED = .04  # rad/s for body turns and every head phase.


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
        self._head_progress_phase = None
        self._final_heading_since = None

    def _now(self):
        return float(self.backend.view.get("t", time.monotonic()))

    def _scan_angles(self, index=None):
        point = self.waypoints[self.index if index is None else index]
        start, end = point.get("scan_range_deg", self.route.get(
            "scan_range_deg", self.route["head_range_deg"]))
        low, high = self.route["head_range_deg"]
        if (not math.isfinite(start) or not math.isfinite(end) or start >= end or
                start < low or end > high):
            raise ValueError(f"Invalid scan range {start, end}; head limits are {low, high}")
        return math.radians(start), math.radians(end)

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
            self.status = "Tuyến quét không khớp scene đang chạy"
            return False
        view = self.backend.view
        try:
            x, y = float(view["x"]), float(view["y"])
            yaw = self._yaw(view)
        except (KeyError, TypeError, ValueError):
            self.status = "Chưa có pose mô phỏng"
            return False
        fields = self.backend.telemetry
        try:
            for index, point in enumerate(self.waypoints):
                if point.get("scan"):
                    self._scan_angles(index)
        except (KeyError, TypeError, ValueError) as exc:
            self.status = f"Tuyến quét có góc lia không hợp lệ: {exc}"
            return False
        if not self.backend.running or not enabled or estop:
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
            self.phase = 'start_hold'
            self._start_hold_wall = time.monotonic()
            self.started = self.phase_started = self._now()
            self.still_since = self.wait_ready_since = None
            self.head_commanded = float(view.get('hj', 0))
            self.head_command_at = self._now()
            self.last_checkpoint = None
            self._drive_index = None
            self._turning = False
            self.backend.send("STOP")
            self._progress("Chờ RB đứng yên trước khi tiếp tục tuyến quét")
            return True
        start_x, start_y = self.route.get("start_xy_m", (0.0, 0.0))
        if math.hypot(x-start_x, y-start_y) > .6 or abs(yaw) > .20:
            self.status = f"Tuyến cố định cần RB gần điểm đầu ({start_x:g}, {start_y:g}), hướng +X"
            return False
        self.index = 0
        self._final_heading_since = None
        self.last_checkpoint = None
        self._drive_index = None
        self._turning = False
        self.phase = "start_hold"
        self._start_hold_wall = time.monotonic()
        self.started = self.phase_started = self._now()
        self.still_since = None
        self.backend.send("STOP")
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
        if self.phase == "start_hold":
            self.backend.send("STOP")
            now = self._now()
            wall_now = time.monotonic()
            if now-self.phase_started >= 30 or wall_now-self._start_hold_wall >= 30:
                self.stop("Tuyến quét dừng: RB chưa đứng yên/sẵn sàng sau 30 giây mô phỏng/thực")
                return self.status
            try:
                received = float(getattr(self.backend, "telemetry_received_monotonic_s", 0))
                fresh = received > 0 and 0 <= wall_now-received <= 1
                settled = (fresh and fields.get("drive_ready") == "1" and
                           fields.get("height_settled") == "1" and fields.get("height_motion") == "0" and
                           abs(float(fields.get("ground_velocity", "inf"))) < .025 and
                           abs(float(fields.get("pitch_rate", "inf"))) < .12)
            except (TypeError, ValueError):
                settled = False
            if settled:
                if self.still_since is None:
                    self.still_since = now
                elif now-self.still_since >= .3:
                    self.phase = "drive"
                    self.still_since = None
                    self._drive_index = None
            else:
                self.still_since = None
            if self.phase == "start_hold":
                return self._progress("Chờ RB đứng yên trước khi chạy tuyến quét")
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
            target, _ = self._scan_angles()
            paced = bool(self.waypoints[self.index].get('scan_step_deg') and live is not None)
            limit, captured = self._processed_head_limit(target, live) if paced else (target, True)
            if limit is None:
                return self._progress('Chờ ảnh fusion trước khi đặt đầu tiếp')
            self._ramp_head(limit, SCAN_ANGULAR_SPEED, head)
            if abs(head - target) < .06 and captured:
                self.phase = "sweep_head"
                self.phase_started = self._now()
                self._sweep_progress_at = time.monotonic()
                self._sweep_best_angle = target
            elif not paced and self._now() - self.phase_started > self._head_travel_timeout():
                self.stop("Tuyến quét dừng: đầu không tới được góc bắt đầu")
                return self.status
            return self._progress(f"Đặt đầu về {math.degrees(target):.0f}°")
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
            _, target = self._scan_angles()
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
            self._ramp_head(limit, SCAN_ANGULAR_SPEED, head)
            if abs(head - target) < .06 and captured:
                self.phase = "return_head"
                self.phase_started = self._now()
            elif ((not step or live is None) and self._now() - self.phase_started > self._head_travel_timeout()) or (
                    step and live is not None and time.monotonic()-self._sweep_progress_at>180):
                self.stop("Tuyến quét dừng: đầu không hoàn thành lượt lia")
                return self.status
            else:
                return self._progress(f"Đang lia ToF {math.degrees(head):.0f}°")

        if self.phase == "return_head":
            paced = bool(self.waypoints[self.index].get('scan_step_deg') and live is not None)
            limit, captured = self._processed_head_limit(0.0, live) if paced else (0.0, True)
            if limit is None:
                return self._progress('Chờ ảnh fusion trước khi hồi đầu tiếp')
            self._ramp_head(limit, SCAN_ANGULAR_SPEED, head)
            if abs(head) < .06 and captured:
                self.index += 1
                self.phase = "drive"
            elif not paced and self._now() - self.phase_started > self._head_travel_timeout():
                self.stop("Tuyến quét dừng: đầu không hồi về hướng thẳng")
                return self.status
            else:
                return self._progress("Đưa đầu về hướng thẳng")

        if self.phase == "drive" and live is not None:
            feedback = live.scan_feedback() if hasattr(live, 'scan_feedback') else None
            try:
                sensor_time = float(feedback['sensor_time']) if feedback else float('nan')
            except (KeyError, TypeError, ValueError):
                sensor_time = float('nan')
            # shortcut: cap allowable lag at 2.5 sim seconds; revisit for factors above 5x.
            max_lag = min(2.5, .5*max(1., float(os.environ.get('RB1_SIM_SPEED_FACTOR', '1'))))
            if not math.isfinite(sensor_time) or self._now()-sensor_time > max_lag:
                self.backend.send("STOP")
                self._progress_at = self._now()
                return self._progress("Chờ SLAM theo kịp ảnh trước khi di chuyển")

        if self.index >= len(self.waypoints):
            self.backend.send("STOP")
            self.phase = "idle"
            self.status = "Đã hoàn tất tuyến quét map"
            return self.status

        while self.index < len(self.waypoints):
            point = self.waypoints[self.index]
            distance = math.hypot(point["x"] - x, point["y"] - y)
            returning = self.index == len(self.waypoints) - 1 and 'finish_yaw_deg' in self.route
            if distance >= .09 and not (returning and self._final_heading_since is not None):
                break
            if returning:
                now = self._now()
                if self._final_heading_since is None:
                    self._final_heading_since = now
                error = math.remainder(math.radians(self.route['finish_yaw_deg']) - yaw, 2 * math.pi)
                if abs(error) > .05:
                    self.still_since = None
                    if now - self._final_heading_since > 120:
                        self.stop("Tuyến quét dừng: RB chưa quay về hướng ban đầu")
                        return self.status
                    self.backend.command_drive(0., max(-SCAN_ANGULAR_SPEED, min(SCAN_ANGULAR_SPEED, error)))
                    return self._progress("Quay thân về hướng quét ban đầu")
                self.backend.send("STOP")
                if self.still_since is None:
                    self.still_since = now
                if now - self.still_since < 2:
                    return self._progress("Giữ hướng ban đầu để cuVSLAM quan sát lại")
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

        reverse = bool(point.get('reverse', False))
        desired = math.atan2(point["y"] - y, point["x"] - x) + (math.pi if reverse else 0.0)
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
        speed = 0.0 if self._turning else (-1 if reverse else 1) * min(speed_limit, .85 * distance) * max(0.0, 1 - (error / .65)**2)
        self.backend.command_drive(speed, max(-SCAN_ANGULAR_SPEED, min(SCAN_ANGULAR_SPEED, 1.5 * error)))
        return self._progress(f"{'Căn hướng' if self._turning else 'Di chuyển tuyến'} · {distance:.1f} m tới điểm kế")

    def _head_travel_timeout(self):
        low, high = self.route['head_range_deg']
        return math.radians(high-low)/SCAN_ANGULAR_SPEED + 8

    def _processed_head_limit(self, target, live):
        phase = (self.phase, self.phase_started)
        now = time.monotonic()
        if self._head_progress_phase != phase:
            self._head_progress_phase = phase
            self._head_progress_at = now
            self._head_best_error = math.inf
        feedback = live.scan_feedback()
        fresh = feedback is not None and feedback['sensor_time'] >= self.phase_started
        if fresh:
            processed = float(feedback['head_yaw_rad'])
            error = abs(processed-target)
            if error < self._head_best_error-math.radians(1):
                self._head_best_error = error
                self._head_progress_at = now
        if now-self._head_progress_at > 180:
            self.stop('Tuyến quét dừng: quá hạn chờ góc fusion mới khi đặt/hồi đầu')
            return None, False
        if not fresh:
            return None, False
        step = math.radians(self.waypoints[self.index]['scan_step_deg'])
        return max(processed-step, min(processed+step, target)), error < step

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
