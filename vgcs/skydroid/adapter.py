from __future__ import annotations

import queue
import re
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from vgcs.skydroid.command_map import SkydroidCommandProfile, get_profile
from vgcs.skydroid.protocol import (
    build_gac_query,
    build_gimbal_speed,
    build_got_target,
    build_slr_query,
    build_slr_trigger,
    build_sum_track,
    build_top_frame,
    build_zoom_command_burst,
    decode_slr_distance_m,
    extract_attitude_deg,
    parse_slr_distance_from_payload,
    slr_raw_hex,
    parse_top_frame,
    _LRF_FRAME_H,
    _LRF_FRAME_W,
)
from vgcs.skydroid.transport import TopUdpTransport

# PROTOCAL.doc: TOP UDP port 5000; lens/system also on 9003 on some C13 builds.
_C13_PROBE_PORTS = (5000, 9003, 19856)
_ZOOM_EXTRA_PORTS = (9003, 19853)

# LRF lock: SLR reads along laser boresight — wait for tracker slew before accepting range.
_LRF_LOCK_MIN_WAIT_S = 1.25
_LRF_LOCK_MAX_WAIT_S = 12.0
_LRF_LOCK_POLL_S = 0.25
_LRF_BASELINE_EPS_M = 1.5
_LRF_STABLE_EPS_M = 0.6
_LRF_MOVED_MIN_M = 2.0
_LRF_GIMBAL_SLR_MIN_M = 0.5
_LRF_GIMBAL_ACCEPT_MIN_DEG = 1.5
_LRF_GIMBAL_SLR_SETTLE_S = 2.5
_LRF_CONVERGE_SAMPLES = 5
_LRF_REFINE_SAMPLES = 8
_LRF_REFINE_HOLD_S = 2.75
_LRF_REFINE_EXTEND_S = 2.5
_LRF_CLIMB_EPS_M = 0.35
_LRF_SETTLE_WINDOW = 8
_LRF_SETTLE_MIN_DRIFT_M = 2.0
_LRF_POST_MOVE_MIN_WAIT_S = 2.5
_LRF_POST_MOVE_SETTLE_S = 4.0
_LRF_GOT_RESEND_INTERVAL_S = 2.0
_LRF_BASELINE_RETRIES = 8
_LRF_BASELINE_GAP_S = 0.2
_LRF_POST_GOT_WAIT_S = 2.5
_LRF_SLR_SHOT_SETTLE_S = 0.45
_LRF_GAM_APPROACH_DPS = 20.0
_LRF_GAM_SETTLE_S = 2.0
_LRF_GAM_AIM_TOL_DEG = 2.5
_LRF_GAM_WAIT_S = 4.5
_LRF_GIMBAL_MOVE_MIN_DEG = 0.35
_LRF_FOV_H_DEG = 83.4
_LRF_FOV_V_DEG = 46.9
_LRF_REPOINT_AFTER_S = 4.0
_LRF_GIMBAL_SLEW_SPEED_DPS = 3.0
_LRF_GIMBAL_HOLD_REFRESH_S = 0.08
_LRF_GIMBAL_AXIS_MAX_S = 1.8
_LRF_GIMBAL_SLEW_SCALE = 0.88
_LRF_C13_INVERT_GSY = True
_LRF_LOCK_MOVE_GIMBAL = False
_LRF_GIMBAL_OVERSHOOT_FACTOR = 1.45
_LRF_GIMBAL_SLEW_SETTLE_S = 0.25
_LRF_MAX_REPOINTS = 0
_PROBE_TIMEOUT_S = 0.12

# Motion commands: no UDP reply wait (C13 often has no timely ACK for GSY/GSP/GSM).
_MOTION_COMMANDS = frozenset(
    {
        "GSY",
        "GSP",
        "GSM",
        "GAY",
        "GAP",
        "GAM",
        "PTZ",
        "PT_UP",
        "PT_DOWN",
        "PT_LEFT",
        "PT_RIGHT",
        "PT_CENTER",
        "PT_STOP",
        "PTZ_UP",
        "PTZ_DOWN",
        "PTZ_LEFT",
        "PTZ_RIGHT",
        "PTZ_CENTER",
        "PTZ_STOP",
        "ZMC",
        "FCC",
    }
)


@dataclass(frozen=True)
class GimbalStatus:
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    supported: bool = False
    updated_mono: float = 0.0


class SkydroidTopUdpAdapter:
    def __init__(
        self,
        *,
        host: str = "",
        hosts: list[str] | None = None,
        port: int = 5000,
        timeout_s: float = 0.25,
        retries: int = 1,
        rate_limit_hz: float = 25.0,
        log_path: str = "",
        profile_id: str = "c13_default",
    ) -> None:
        merged: list[str] = []
        for h in [str(host or "").strip(), *(hosts or [])]:
            if h and h not in merged:
                merged.append(h)
        if not merged:
            merged.append("192.168.144.108")
        self._hosts = merged
        self._active_host = merged[0]
        self._transport = TopUdpTransport(
            self._active_host,
            port,
            timeout_s=timeout_s,
            retries=retries,
            log_path=log_path,
        )
        self._queue: queue.Queue[tuple[list[str], dict[str, object], bool]] = queue.Queue(maxsize=128)
        self._status = GimbalStatus()
        self._status_lock = threading.Lock()
        self._profile: SkydroidCommandProfile = get_profile(profile_id)
        self._profile_id = str(profile_id or "c13_default")
        self._running = False
        self._worker: threading.Thread | None = None
        self._status_poller: threading.Thread | None = None
        self._probe_thread: threading.Thread | None = None
        self._probe_finished = threading.Event()
        self._probe_finished.set()
        self._gaa_enabled = False
        self._min_dt = 1.0 / max(1.0, float(rate_limit_hz))
        self._last_send_mono = 0.0
        self._laser_range_m: float | None = None
        self._laser_range_mono: float = 0.0
        self._last_slr_poll_mono: float = 0.0
        self._lrf_locked = False
        self._lrf_lock_x = 0
        self._lrf_lock_y = 0
        self._lrf_locked_distance_m: float | None = None
        self._active_port = int(port)
        self._transport.set_datagram_handler(self._maybe_update_status)

    def active_endpoint(self) -> tuple[str, int, str]:
        return (str(self._active_host), int(self._active_port), str(self._profile_id))

    def get_laser_range_m(self, *, max_age_s: float = 4.0) -> float | None:
        """Frozen C13 SLR distance after video lock; None when not locked."""
        with self._status_lock:
            if not self._lrf_locked:
                return None
            dist = self._lrf_locked_distance_m
            ts = float(self._laser_range_mono or 0.0)
        if dist is None or ts <= 0.0:
            return None
        if time.monotonic() - ts > max(0.5, float(max_age_s)):
            return None
        return float(dist)

    def is_lrf_locked(self) -> bool:
        with self._status_lock:
            return bool(self._lrf_locked)

    def _send_got_track(self, x_px: int, y_px: int, *, frame_w: int, frame_h: int) -> None:
        got = build_got_target(x_px, y_px, frame_w=frame_w, frame_h=frame_h)
        self._transport.send_and_receive(
            got, expect_reply=False, log=True, timeout_s=0.08
        )
        time.sleep(0.12)
        confirm = build_sum_track(confirm=True)
        self._transport.send_and_receive(
            confirm, expect_reply=False, log=True, timeout_s=0.08
        )

    @staticmethod
    def _pixel_boresight_offset_deg(
        x_px: int, y_px: int, *, fw: int = _LRF_FRAME_W, fh: int = _LRF_FRAME_H
    ) -> tuple[float, float]:
        """Yaw/pitch delta (degrees) from frame centre to pixel (companion 83.4° HFOV)."""
        ox = (float(x_px) - fw / 2.0) / max(1.0, fw / 2.0)
        oy = (float(y_px) - fh / 2.0) / max(1.0, fh / 2.0)
        dyaw = ox * (_LRF_FOV_H_DEG / 2.0)
        dpitch = oy * (_LRF_FOV_V_DEG / 2.0)
        return float(dyaw), float(dpitch)

    def _ensure_active_transport(self) -> None:
        host = str(self._active_host or self._hosts[0] or "").strip()
        port = int(self._active_port or self._transport._port or 5000)
        if host:
            self._transport.set_route_host(host)
            self._transport._host = host
        self._transport._port = port

    @staticmethod
    def _angle_err_deg(target: float, current: float) -> float:
        """Signed shortest delta on a gimbal axis (approx ±90°)."""
        d = float(target) - float(current)
        if d > 90.0:
            d -= 180.0
        elif d < -90.0:
            d += 180.0
        return float(d)

    @staticmethod
    def _expected_offset_deg(dyaw: float, dpitch: float) -> float:
        return float((float(dyaw) ** 2 + float(dpitch) ** 2) ** 0.5)

    def _gimbal_aim_ok(
        self,
        att_start: tuple[float, float],
        att_end: tuple[float, float] | None,
        *,
        yaw_tgt: float,
        pitch_tgt: float,
        dyaw: float,
        dpitch: float,
    ) -> bool:
        if att_end is None:
            return False
        expected = self._expected_offset_deg(dyaw, dpitch)
        if expected < 0.25:
            return True
        move = self._gimbal_total_move_deg(att_start, att_end)
        if move > expected * _LRF_GIMBAL_OVERSHOOT_FACTOR + 2.0:
            print(
                f"[VGCS:lrf] gimbal overshoot move={move:.1f}° "
                f"expected≈{expected:.1f}° att={att_start}->{att_end}"
            )
            return False
        if move < 0.25 and expected > 2.0:
            print(
                f"[VGCS:lrf] gimbal did not move ({move:.1f}°) "
                f"expected≈{expected:.1f}° att={att_start}->{att_end}"
            )
            return False
        yaw_err = abs(self._angle_err_deg(yaw_tgt, att_end[0]))
        yaw_delta = abs(float(att_end[0]) - float(att_start[0]))
        pitch_gac_stuck = (
            abs(float(att_end[1])) < 0.05
            and abs(float(att_start[1])) < 0.05
            and abs(float(dpitch)) > 1.0
        )
        if pitch_gac_stuck:
            return yaw_err <= 2.5 or yaw_delta >= abs(float(dyaw)) * 0.35
        pitch_err = abs(self._angle_err_deg(pitch_tgt, att_end[1]))
        if yaw_err > 2.5 and yaw_delta < abs(float(dyaw)) * 0.35:
            print(
                f"[VGCS:lrf] gimbal missed aim yaw_err={yaw_err:.1f}° "
                f"target={yaw_tgt:.1f} att={att_end}"
            )
            return False
        if pitch_err > 2.5:
            print(
                f"[VGCS:lrf] gimbal missed aim pitch_err={pitch_err:.1f}° "
                f"target={pitch_tgt:.1f} att={att_end}"
            )
            return False
        return True

    def _gimbal_stop_hard(self) -> None:
        """Multiple GSM stops — C13 can coast after a single stop packet."""
        self._ensure_active_transport()
        self._drop_pending_motion_commands()
        tick = max(0.05, float(_LRF_GIMBAL_HOLD_REFRESH_S))
        stop = build_gimbal_speed(0.0, 0.0)
        for _ in range(12):
            try:
                self._transport.send_and_receive(
                    stop, expect_reply=False, log=False, timeout_s=0.04
                )
            except Exception:
                pass
            try:
                self.set_speed(0.0, 0.0)
            except Exception:
                pass
            time.sleep(tick)

    @staticmethod
    def _axis_burst_duration_s(angle_deg: float, speed_dps: float) -> float:
        spd = max(2.0, float(speed_dps))
        return min(
            float(_LRF_GIMBAL_AXIS_MAX_S),
            abs(float(angle_deg)) / spd * float(_LRF_GIMBAL_SLEW_SCALE) + 0.12,
        )

    @staticmethod
    def _gsy_yaw_rate_for_offset(dyaw: float, spd: float) -> float:
        """C13 field firmware: GSY sign is opposite PROTOCAL §3.2.1 (yaw right = positive)."""
        rate = float(spd) if float(dyaw) > 0.0 else -float(spd)
        if _LRF_C13_INVERT_GSY:
            rate = -rate
        return rate

    def _hold_speed_direct_burst(
        self,
        yaw_rate: float,
        pitch_rate: float,
        duration_s: float,
    ) -> None:
        """GSY/GSP refresh without GAC reads (GAC polling blocks the transport lock)."""
        self._ensure_active_transport()
        deadline = time.monotonic() + max(0.12, min(_LRF_GIMBAL_AXIS_MAX_S, float(duration_s)))
        tick = max(0.05, float(_LRF_GIMBAL_HOLD_REFRESH_S))
        while time.monotonic() < deadline:
            y = float(yaw_rate)
            p = float(pitch_rate)
            if abs(y) >= 0.05:
                self._transport.send_and_receive(
                    build_top_frame("GSY", {"yaw": y}),
                    expect_reply=False,
                    log=False,
                    timeout_s=0.05,
                )
            if abs(p) >= 0.05:
                self._transport.send_and_receive(
                    build_top_frame("GSP", {"pitch": p}),
                    expect_reply=False,
                    log=False,
                    timeout_s=0.05,
                )
            time.sleep(tick)
        stop = build_gimbal_speed(0.0, 0.0)
        for _ in range(6):
            self._transport.send_and_receive(
                stop, expect_reply=False, log=False, timeout_s=0.05
            )
            time.sleep(tick)

    def _slew_gimbal_open_loop_offset(
        self,
        dyaw: float,
        dpitch: float,
        att_start: tuple[float, float],
        *,
        speed_dps: float = _LRF_GIMBAL_SLEW_SPEED_DPS,
    ) -> tuple[float, float] | None:
        """Short undershoot bursts: yaw first, optional brief pitch (no GAC correction chase)."""
        spd = max(2.0, float(speed_dps))
        expected = self._expected_offset_deg(dyaw, dpitch)
        att: tuple[float, float] | None = att_start
        if abs(float(dyaw)) >= 0.3:
            yaw_r = self._gsy_yaw_rate_for_offset(dyaw, spd)
            dur_y = self._axis_burst_duration_s(dyaw, spd)
            print(f"[VGCS:lrf] yaw burst {yaw_r:+.1f}°/s dur={dur_y:.2f}s")
            self._hold_speed_direct_burst(yaw_r, 0.0, dur_y)
            self._gimbal_stop_hard()
            time.sleep(_LRF_GIMBAL_SLEW_SETTLE_S)
            att = self._read_gimbal_attitude_deg() or att
            move = self._gimbal_total_move_deg(att_start, att)
            if move > expected * _LRF_GIMBAL_OVERSHOOT_FACTOR + 1.5:
                print(
                    f"[VGCS:lrf] yaw burst overshoot {move:.1f}° "
                    f"(expected≈{expected:.1f}°) — continuing pitch"
                )
        if att is not None and abs(float(dpitch)) >= 0.5:
            pitch_r = -spd if float(dpitch) > 0.0 else spd
            dur_p = min(1.0, self._axis_burst_duration_s(dpitch, spd))
            print(f"[VGCS:lrf] pitch burst {pitch_r:+.1f}°/s dur={dur_p:.2f}s")
            self._hold_speed_direct_burst(0.0, pitch_r, dur_p)
        self._gimbal_stop_hard()
        time.sleep(_LRF_GIMBAL_SLEW_SETTLE_S)
        return self._read_gimbal_attitude_deg() or att

    def _wait_gimbal_at_angles(
        self,
        yaw_tgt: float,
        pitch_tgt: float,
        *,
        timeout_s: float = _LRF_GAM_WAIT_S,
    ) -> tuple[float, float] | None:
        deadline = time.monotonic() + max(0.5, float(timeout_s))
        last: tuple[float, float] | None = None
        while time.monotonic() < deadline:
            time.sleep(0.2)
            att = self._read_gimbal_attitude_deg()
            if att is None:
                continue
            last = att
            yaw_err = abs(self._angle_err_deg(float(yaw_tgt), att[0]))
            pitch_err = abs(self._angle_err_deg(float(pitch_tgt), att[1]))
            if (
                yaw_err <= _LRF_GAM_AIM_TOL_DEG
                and pitch_err <= _LRF_GAM_AIM_TOL_DEG
            ):
                return att
        return last

    def _point_gimbal_at_pixel_gam(
        self,
        x_px: int,
        y_px: int,
        att_now: tuple[float, float] | None,
    ) -> tuple[tuple[float, float] | None, bool]:
        """Point gimbal at click via GAM absolute angles (PROTOCAL §3.3.1)."""
        self._ensure_active_transport()
        if att_now is None:
            att_now = self._read_gimbal_attitude_deg()
        if att_now is None:
            print("[VGCS:lrf] GAM point skipped — no GAC attitude")
            return None, False
        dyaw, dpitch_img = self._pixel_boresight_offset_deg(x_px, y_px)
        yaw_tgt = float(att_now[0]) + dyaw
        pitch_tgt = max(-90.0, min(90.0, float(att_now[1]) - dpitch_img))
        if abs(dyaw) < 0.2 and abs(dpitch_img) < 0.2:
            return att_now, True
        print(
            f"[VGCS:lrf] GAM point px=({x_px},{y_px}) "
            f"att={att_now} -> target=({yaw_tgt:.2f},{pitch_tgt:.2f}) "
            f"offset=({dyaw:.1f}°,{dpitch_img:.1f}°)"
        )
        self._drop_pending_motion_commands()
        spd = float(_LRF_GAM_APPROACH_DPS)
        try:
            frame = build_top_frame(
                "GAM",
                {
                    "yaw": yaw_tgt,
                    "pitch": pitch_tgt,
                    "speed": spd,
                    "yaw_speed": spd,
                    "pitch_speed": spd,
                },
            )
            self._transport.send_and_receive(
                frame, expect_reply=False, log=True, timeout_s=0.12
            )
        except Exception as exc:
            print(f"[VGCS:lrf] GAM point failed: {exc}")
            return att_now, False
        time.sleep(_LRF_GAM_SETTLE_S)
        att_after = self._wait_gimbal_at_angles(yaw_tgt, pitch_tgt)
        aim_ok = self._gimbal_aim_ok(
            att_now,
            att_after,
            yaw_tgt=yaw_tgt,
            pitch_tgt=pitch_tgt,
            dyaw=dyaw,
            dpitch=dpitch_img,
        )
        move_deg = (
            self._gimbal_total_move_deg(att_now, att_after)
            if att_after is not None
            else 0.0
        )
        print(
            f"[VGCS:lrf] GAM point done att={att_now}->{att_after} "
            f"move={move_deg:.1f}° ok={aim_ok}"
        )
        return att_after, aim_ok

    def _send_gimbal_point_at_pixel(
        self,
        x_px: int,
        y_px: int,
        att_now: tuple[float, float] | None,
    ) -> tuple[tuple[float, float] | None, bool]:
        """Closed-loop point at click; returns (attitude, aim_ok). Not used for LRF lock."""
        if not _LRF_LOCK_MOVE_GIMBAL:
            if att_now is None:
                att_now = self._read_gimbal_attitude_deg()
            return att_now, True
        self._ensure_active_transport()
        if att_now is None:
            att_now = self._read_gimbal_attitude_deg()
        if att_now is None:
            print("[VGCS:lrf] gimbal slew skipped — no GAC attitude")
            return None, False
        dyaw, dpitch_img = self._pixel_boresight_offset_deg(x_px, y_px)
        yaw_tgt = float(att_now[0]) + dyaw
        pitch_tgt = max(-90.0, min(90.0, float(att_now[1]) - dpitch_img))
        if abs(dyaw) < 0.2 and abs(dpitch_img) < 0.2:
            return att_now, True
        print(
            f"[VGCS:lrf] gimbal slew px=({x_px},{y_px}) "
            f"att={att_now} -> target=({yaw_tgt:.2f},{pitch_tgt:.2f}) "
            f"offset=({dyaw:.1f}°,{dpitch_img:.1f}°) "
            f"endpoint={self._active_host}:{self._active_port}"
        )
        self._drop_pending_motion_commands()
        att_after = self._slew_gimbal_open_loop_offset(
            dyaw, dpitch_img, att_now
        )
        self._gimbal_stop_hard()
        aim_ok = self._gimbal_aim_ok(
            att_now,
            att_after,
            yaw_tgt=yaw_tgt,
            pitch_tgt=pitch_tgt,
            dyaw=dyaw,
            dpitch=dpitch_img,
        )
        move_deg = (
            self._gimbal_total_move_deg(att_now, att_after)
            if att_after is not None
            else 0.0
        )
        print(
            f"[VGCS:lrf] gimbal slew done att={att_now}->{att_after} "
            f"move={move_deg:.1f}° ok={aim_ok}"
        )
        return att_after, aim_ok

    @staticmethod
    def _slr_median(values: list[float]) -> float:
        s = sorted(float(v) for v in values)
        n = len(s)
        if n <= 0:
            return 0.0
        mid = n // 2
        if n % 2:
            return float(s[mid])
        return (float(s[mid - 1]) + float(s[mid])) / 2.0

    @staticmethod
    def _slr_still_climbing(samples: list[float], eps: float = _LRF_CLIMB_EPS_M) -> bool:
        if len(samples) < 2:
            return False
        return float(samples[-1]) > float(samples[-2]) + float(eps)

    @staticmethod
    def _slr_converged(samples: list[float], n: int = _LRF_CONVERGE_SAMPLES) -> float | None:
        if len(samples) < n:
            return None
        tail = [float(v) for v in samples[-n:]]
        if max(tail) - min(tail) > _LRF_STABLE_EPS_M:
            return None
        if SkydroidTopUdpAdapter._slr_still_climbing(tail):
            return None
        return SkydroidTopUdpAdapter._slr_median(tail)

    @staticmethod
    def _slr_still_settling(samples: list[float], elapsed: float) -> bool:
        """True while SLR drift after tracker slew suggests readings have not plateaued."""
        if len(samples) < 4:
            return elapsed < _LRF_POST_MOVE_MIN_WAIT_S
        if elapsed < _LRF_POST_MOVE_MIN_WAIT_S:
            return True
        n = min(len(samples), _LRF_SETTLE_WINDOW)
        window = [float(v) for v in samples[-n:]]
        drift = abs(window[-1] - window[0])
        if drift >= _LRF_SETTLE_MIN_DRIFT_M and elapsed < _LRF_POST_MOVE_SETTLE_S:
            return True
        if len(samples) >= 8 and elapsed < _LRF_POST_MOVE_SETTLE_S:
            mid = SkydroidTopUdpAdapter._slr_median(samples[-8:-4])
            recent = SkydroidTopUdpAdapter._slr_median(samples[-4:])
            if abs(recent - mid) > 0.8:
                return True
        if len(window) >= 6 and elapsed < _LRF_POST_MOVE_SETTLE_S:
            half = max(2, n // 2)
            first = SkydroidTopUdpAdapter._slr_median(window[:half])
            second = SkydroidTopUdpAdapter._slr_median(window[half:])
            if abs(second - first) > 1.0:
                return True
        return False

    @staticmethod
    def _slr_samples_moved_from_baseline(
        samples: list[float], baseline_m: float | None
    ) -> bool:
        if baseline_m is None:
            return False
        base = float(baseline_m)
        return any(abs(float(s) - base) >= _LRF_MOVED_MIN_M for s in samples)

    @staticmethod
    def _slr_post_move_samples(
        samples: list[float], baseline_m: float | None
    ) -> list[float]:
        if baseline_m is None:
            return []
        base = float(baseline_m)
        for i, s in enumerate(samples):
            if abs(float(s) - base) >= _LRF_MOVED_MIN_M:
                return [float(v) for v in samples[i:]]
        return []

    @staticmethod
    def _gimbal_attitude_moved(
        before: tuple[float, float] | None,
        after: tuple[float, float] | None,
        *,
        min_deg: float = _LRF_GIMBAL_MOVE_MIN_DEG,
    ) -> bool | None:
        if before is None or after is None:
            return None
        dy = abs(float(after[0]) - float(before[0]))
        dp = abs(float(after[1]) - float(before[1]))
        return dy >= float(min_deg) or dp >= float(min_deg)

    @staticmethod
    def _gimbal_total_move_deg(
        before: tuple[float, float] | None,
        after: tuple[float, float] | None,
    ) -> float:
        if before is None or after is None:
            return 0.0
        dy = float(after[0]) - float(before[0])
        dp = float(after[1]) - float(before[1])
        return float((dy * dy + dp * dp) ** 0.5)

    @staticmethod
    def _slr_moved_enough(
        value_m: float,
        baseline_m: float | None,
        *,
        min_m: float = _LRF_MOVED_MIN_M,
    ) -> bool:
        if baseline_m is None:
            return False
        return abs(float(value_m) - float(baseline_m)) >= float(min_m)

    def _try_accept_stable_slr(
        self,
        samples: list[float],
        *,
        elapsed: float,
    ) -> float | None:
        """Accept a stable post-GOT SLR median (no stale-baseline gate)."""
        if len(samples) < _LRF_CONVERGE_SAMPLES:
            return None
        if float(elapsed) < _LRF_LOCK_MIN_WAIT_S:
            return None
        tail = [float(v) for v in samples[-_LRF_CONVERGE_SAMPLES:]]
        converged = self._slr_converged(tail, _LRF_CONVERGE_SAMPLES)
        if converged is None:
            return None
        if self._slr_still_climbing(tail[-3:]):
            return None
        return float(converged)

    def _try_accept_gimbal_slew_slr(
        self,
        samples: list[float],
        baseline_m: float | None,
        att_before: tuple[float, float] | None,
        att_now: tuple[float, float] | None,
        *,
        gimbal_slew_mono: float | None,
        yaw_tgt: float | None = None,
        pitch_tgt: float | None = None,
        dyaw: float = 0.0,
        dpitch: float = 0.0,
    ) -> float | None:
        """Accept stable SLR only when gimbal aimed correctly and range moved from baseline."""
        if gimbal_slew_mono is None or len(samples) < _LRF_CONVERGE_SAMPLES:
            return None
        if time.monotonic() - float(gimbal_slew_mono) < _LRF_GIMBAL_SLR_SETTLE_S:
            return None
        if att_before is None or att_now is None:
            return None
        if yaw_tgt is not None and pitch_tgt is not None:
            if not self._gimbal_aim_ok(
                att_before,
                att_now,
                yaw_tgt=float(yaw_tgt),
                pitch_tgt=float(pitch_tgt),
                dyaw=float(dyaw),
                dpitch=float(dpitch),
            ):
                return None
        tail = [float(v) for v in samples[-_LRF_CONVERGE_SAMPLES:]]
        converged = self._slr_converged(tail, _LRF_CONVERGE_SAMPLES)
        if converged is None:
            return None
        if self._slr_still_climbing(tail[-3:]):
            return None
        if not self._slr_moved_from_baseline(converged, baseline_m):
            return None
        return float(converged)

    @staticmethod
    def _slr_tail_stable(samples: list[float], n: int = 3) -> tuple[float, float] | None:
        if len(samples) < n:
            return None
        tail = samples[-n:]
        spread = max(tail) - min(tail)
        if spread > _LRF_STABLE_EPS_M:
            return None
        return SkydroidTopUdpAdapter._slr_median(tail), spread

    def _note_slr_sample(self, value_m: float) -> None:
        # Distance is frozen after lock — ignore live SLR datagrams while locked.
        with self._status_lock:
            if self._lrf_locked:
                return
            self._laser_range_m = float(value_m)
            self._laser_range_mono = time.monotonic()

    def _refine_slr_estimate(
        self,
        on_sample: Callable[[float], None] | None,
    ) -> float | None:
        """Extra SLR polls after coarse converge — median filters jitter / late slew."""
        buf: list[float] = []
        start_mono = time.monotonic()
        deadline = start_mono + _LRF_REFINE_HOLD_S
        max_deadline = start_mono + _LRF_REFINE_HOLD_S + _LRF_REFINE_EXTEND_S

        def _emit(value_m: float) -> None:
            if on_sample is None:
                return
            try:
                on_sample(float(value_m))
            except Exception:
                pass

        while time.monotonic() < deadline and len(buf) < _LRF_REFINE_SAMPLES:
            time.sleep(_LRF_LOCK_POLL_S)
            reading = self._query_slr_distance_m(log=False, fresh=True)
            if reading is None:
                continue
            buf.append(float(reading))
            _emit(float(reading))
            if (
                len(buf) >= 3
                and self._slr_still_climbing(buf[-3:])
                and deadline < max_deadline
            ):
                deadline = min(deadline + 0.45, max_deadline)
        if not buf:
            return None
        tail_n = min(_LRF_REFINE_SAMPLES, len(buf))
        tail = [float(v) for v in buf[-tail_n:]]
        refined = self._slr_median(tail)
        if len(buf) >= 5 and buf[-1] > buf[0] + 2.0:
            upper = sorted(tail)[max(0, len(tail) - max(3, len(tail) // 2)) :]
            if upper:
                refined = self._slr_median(upper)
        print(f"[VGCS:lrf] refine median={refined:.1f} m from {buf}")
        return float(refined)

    @staticmethod
    def _slr_moved_from_baseline(value_m: float, baseline_m: float | None) -> bool:
        if baseline_m is None:
            return False
        return abs(float(value_m) - float(baseline_m)) >= _LRF_MOVED_MIN_M

    def _read_gimbal_attitude_deg(self) -> tuple[float, float] | None:
        active_port = int(self._transport._port)
        try:
            reply = self._transport.send_and_receive(
                build_gac_query(),
                expect_reply=True,
                log=False,
                timeout_s=0.45,
            )
            if not reply:
                return None
            dec = parse_top_frame(reply)
            yaw, pitch = extract_attitude_deg(dec)
            if yaw is None and pitch is None:
                return None
            return float(yaw or 0.0), float(pitch or 0.0)
        except Exception:
            return None
        finally:
            self._transport._port = active_port

    def lock_lrf_target_at_norm(
        self,
        u: float,
        v: float,
        *,
        frame_w: int = _LRF_FRAME_W,
        frame_h: int = _LRF_FRAME_H,
        on_sample: Callable[[float], None] | None = None,
    ) -> float | None:
        """C13: GOT+SUM at click pixel, then fresh SLR reads (never slews gimbal)."""
        if not self.gimbal_telemetry_ok():
            return None
        # PROTOCAL §3.3.5: GOT coordinates are always on 1280×720 companion video.
        fw = _LRF_FRAME_W
        fh = _LRF_FRAME_H
        x_px = max(0, min(fw, int(round(max(0.0, min(1.0, float(u))) * fw))))
        y_px = max(0, min(fh, int(round(max(0.0, min(1.0, float(v))) * fh))))
        self._ensure_active_transport()
        active_port = int(self._active_port)
        dist_m: float | None = None
        samples: list[float] = []
        try:
            with self._status_lock:
                self._laser_range_m = None
                self._laser_range_mono = 0.0
                self._lrf_locked_distance_m = None

            att_before = self._read_gimbal_attitude_deg()
            dyaw, dpitch_img = self._pixel_boresight_offset_deg(x_px, y_px)
            print(
                f"[VGCS:lrf] lock start px=({x_px},{y_px}) "
                f"att={att_before} offset=({dyaw:.1f}°,{dpitch_img:.1f}°)"
            )

            if self._query_slr_distance_m(log=False, fresh=False) is None:
                if self._query_slr_distance_m(log=True, fresh=True) is None:
                    print(
                        "[VGCS:lrf] lock failed — cannot read SLR (check C13 TOP link)"
                    )
                    return None

            pre_slr = self._query_slr_distance_m(log=True, fresh=True)
            if pre_slr is not None:
                print(f"[VGCS:lrf] pre-lock SLR={float(pre_slr):.1f} m")

            att_latest = att_before
            print(
                f"[VGCS:lrf] GOT track px=({x_px},{y_px}) "
                f"— lock at current aim (no gimbal slew)"
            )
            if abs(dyaw) >= 5.0 or abs(dpitch_img) >= 5.0:
                print(
                    f"[VGCS:lrf] hint: click {abs(dyaw):.0f}°/{abs(dpitch_img):.0f}° "
                    f"off centre — aim LRF at target manually before locking"
                )
            self._send_got_track(x_px, y_px, frame_w=fw, frame_h=fh)
            time.sleep(_LRF_POST_GOT_WAIT_S)

            start_mono = time.monotonic()
            deadline = start_mono + _LRF_LOCK_MAX_WAIT_S

            def _emit_sample(value_m: float) -> None:
                if on_sample is None:
                    return
                try:
                    on_sample(float(value_m))
                except Exception:
                    pass

            while time.monotonic() < deadline:
                elapsed = time.monotonic() - start_mono
                time.sleep(_LRF_LOCK_POLL_S)
                reading = self._query_slr_distance_m(
                    log=len(samples) == 0,
                    fresh=True,
                )
                if reading is None:
                    continue
                samples.append(float(reading))
                att_now = self._read_gimbal_attitude_deg()
                if att_now is not None:
                    att_latest = att_now
                _emit_sample(float(reading))
                print(
                    f"[VGCS:lrf] SLR sample {reading:.1f} m "
                    f"(n={len(samples)}, t={elapsed:.1f}s)"
                )

                if elapsed < _LRF_LOCK_MIN_WAIT_S:
                    continue

                stable = self._try_accept_stable_slr(samples, elapsed=elapsed)
                if stable is not None:
                    print(
                        f"[VGCS:lrf] lock stable {stable:.1f} m "
                        f"(att={att_before}->{att_latest})"
                    )
                    dist_m = self._refine_slr_estimate(on_sample) or float(stable)
                    _emit_sample(float(dist_m))
                    break

            if dist_m is None and samples:
                stable = self._try_accept_stable_slr(
                    samples,
                    elapsed=time.monotonic() - start_mono,
                )
                if stable is not None:
                    dist_m = self._refine_slr_estimate(on_sample) or float(stable)
                else:
                    print(
                        f"[VGCS:lrf] lock rejected — SLR did not stabilize "
                        f"(samples={samples[-5:]})"
                    )

            with self._status_lock:
                if dist_m is not None:
                    self._lrf_locked = True
                    self._lrf_lock_x = x_px
                    self._lrf_lock_y = y_px
                    self._lrf_locked_distance_m = float(dist_m)
                    self._laser_range_m = float(dist_m)
                    self._laser_range_mono = time.monotonic()
                else:
                    self._lrf_locked = False
                    self._lrf_locked_distance_m = None
                    self._laser_range_m = None
                    self._laser_range_mono = 0.0
            if dist_m is not None:
                print(f"[VGCS:lrf] lock ok range={dist_m:.1f} m (samples={len(samples)})")
            else:
                print(
                    f"[VGCS:lrf] lock failed — no stable range "
                    f"(samples={len(samples)})"
                )
            return dist_m
        except Exception as exc:
            print(f"[VGCS:lrf] lock exception: {exc}")
            with self._status_lock:
                self._lrf_locked = False
            return None
        finally:
            self._transport._port = active_port

    def _query_slr_distance_m(self, *, log: bool = False, fresh: bool = False) -> float | None:
        """Single SLR read — optional fresh shot (PROTOCAL §4.22 read fires + re-read)."""
        active_port = int(self._transport._port)
        frames_read = (
            build_slr_query(dest="D"),
            build_slr_query(dest="E"),
        )
        try:
            if fresh:
                for frame in frames_read:
                    try:
                        self._transport.send_and_receive(
                            frame,
                            expect_reply=True,
                            log=False,
                            timeout_s=0.35,
                        )
                    except Exception:
                        continue
                time.sleep(_LRF_SLR_SHOT_SETTLE_S)
            last_raw: bytes | None = None
            for frame in frames_read:
                try:
                    reply = self._transport.send_and_receive(
                        frame,
                        expect_reply=True,
                        log=log,
                        timeout_s=0.5,
                    )
                except Exception:
                    continue
                if not reply:
                    continue
                last_raw = reply
                dist_m = parse_slr_distance_from_payload(reply)
                if dist_m is not None:
                    if log:
                        hx = slr_raw_hex(reply)
                        print(
                            f"[VGCS:lrf] SLR reply hex={hx} "
                            f"range={float(dist_m):.1f} m "
                            f"raw={reply.decode('ascii', errors='ignore')!r}"
                        )
                    if not self._lrf_locked:
                        with self._status_lock:
                            self._laser_range_m = float(dist_m)
                            self._laser_range_mono = time.monotonic()
                    return float(dist_m)
            if log and last_raw:
                print(
                    f"[VGCS:lrf] SLR parse failed raw="
                    f"{last_raw.decode('ascii', errors='ignore')!r}"
                )
            return None
        except Exception:
            return None
        finally:
            self._transport._port = active_port

    def unlock_lrf(self) -> None:
        """Stop visual track and clear locked LRF reading."""
        active_port = int(self._transport._port)
        try:
            stop = build_sum_track(confirm=False)
            self._transport.send_and_receive(
                stop, expect_reply=False, log=True, timeout_s=0.08
            )
        except Exception:
            pass
        finally:
            self._transport._port = active_port
        with self._status_lock:
            self._lrf_locked = False
            self._laser_range_m = None
            self._laser_range_mono = 0.0
            self._lrf_locked_distance_m = None
            self._lrf_lock_x = 0
            self._lrf_lock_y = 0

    def gimbal_telemetry_ok(self) -> bool:
        with self._status_lock:
            return bool(self._status.supported)

    def get_status_cached(self) -> GimbalStatus:
        with self._status_lock:
            return self._status

    def get_status(self) -> GimbalStatus:
        """Non-blocking: returns last background poll result (safe on UI thread)."""
        return self.get_status_cached()

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._probe_finished.clear()
        self._transport.start_listener()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._status_poller = threading.Thread(target=self._status_loop, daemon=True)
        self._status_poller.start()
        self._probe_thread = threading.Thread(target=self._probe_endpoints_bg, daemon=True)
        self._probe_thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._queue.put_nowait(("NOOP", {}, False))
        except Exception:
            pass
        self._transport.close()

    def ptz(self, action: str) -> None:
        commands = self._profile.ptz_commands.get(str(action or "").strip().lower(), [])
        if not commands:
            return
        self._enqueue(commands, {}, False)

    def set_speed(self, yaw: float, pitch: float) -> None:
        y = float(yaw)
        p = float(pitch)
        commands = self._speed_commands_for(y, p)
        if not commands:
            return
        self._enqueue(commands, {"yaw": y, "pitch": p}, False)

    def set_angle(self, yaw: float, pitch: float) -> None:
        self.set_angle_axes(yaw_deg=float(yaw), pitch_deg=float(pitch))

    def set_angle_axes(
        self,
        *,
        yaw_deg: float | None = None,
        pitch_deg: float | None = None,
        approach_speed_dps: float = 25.0,
    ) -> None:
        """Absolute angle on one or both axes (GAY/GAP/GAM)."""
        params: dict[str, object] = {"speed": float(approach_speed_dps)}
        if yaw_deg is not None and pitch_deg is not None:
            self._enqueue(
                ["GAM"],
                {**params, "yaw": float(yaw_deg), "pitch": float(pitch_deg)},
                False,
            )
        elif yaw_deg is not None:
            self._enqueue(["GAY"], {**params, "yaw": float(yaw_deg)}, False)
        elif pitch_deg is not None:
            self._enqueue(["GAP"], {**params, "pitch": float(pitch_deg)}, False)

    @staticmethod
    def _speed_commands_for(yaw: float, pitch: float) -> list[str]:
        """Pick GSY / GSP / GSM so a zero axis does not mask the other (TOP spec)."""
        ay = abs(float(yaw)) >= 1e-6
        ap = abs(float(pitch)) >= 1e-6
        if not ay and not ap:
            return ["GSM"]
        if ay and ap:
            return ["GSM"]
        if ay:
            return ["GSY"]
        if ap:
            return ["GSP"]
        return []

    def camera_record_toggle(self) -> None:
        self._enqueue(self._profile.camera_commands.get("record_toggle", []), {}, True)

    def camera_photo(self) -> None:
        self._enqueue(self._profile.camera_commands.get("photo", []), {}, True)

    def camera_zoom(self, level: float) -> None:
        self._enqueue(["ZOOM_BURST"], {"level": float(level)}, False)

    def camera_focus_step(self, direction: int) -> None:
        key = "focus_in" if int(direction) < 0 else "focus_out"
        cmds = self._profile.camera_commands.get(key, [])
        self._enqueue(cmds, {}, False)
        self._enqueue(["FCC"], {"action": "stop"}, False)

    def poll_attitude_now(self) -> GimbalStatus | None:
        """Blocking poll — background threads only (never call from UI thread)."""
        for host in self._hosts:
            self._transport._host = host
            if self._poll_host_once(host):
                with self._status_lock:
                    if self._status.supported:
                        self._active_host = host
                        return self._status
        return None

    def _poll_host_once(self, host: str) -> bool:
        self._transport.set_route_host(host)
        self._transport._host = host
        if not self._gaa_enabled:
            try:
                self._transport.send_and_receive(
                    build_top_frame("GAA", {"hz": 5}),
                    expect_reply=True,
                    log=False,
                    timeout_s=_PROBE_TIMEOUT_S,
                )
                self._gaa_enabled = True
            except Exception:
                pass
        for status_cmd in self._profile.status_commands:
            try:
                frame = build_top_frame(status_cmd, {})
                reply = self._transport.send_and_receive(
                    frame,
                    expect_reply=True,
                    log=False,
                    timeout_s=_PROBE_TIMEOUT_S,
                )
                self._maybe_update_status(reply)
                with self._status_lock:
                    if self._status.supported:
                        return True
            except Exception:
                continue
        with self._status_lock:
            return bool(self._status.supported)

    def _poll_active_endpoint_once(self) -> None:
        self._transport._host = self._active_host
        self._transport._port = self._active_port
        self._poll_host_once(self._active_host)

    def _probe_endpoints_bg(self) -> None:
        try:
            self._probe_endpoints()
        finally:
            self._probe_finished.set()

    def _probe_endpoints(self) -> None:
        """Find host/port/profile that returns GAA/GAC/GAY attitude (C13 + RC gateway paths)."""
        configured = int(self._transport._port)
        ports: list[int] = []
        for p in (configured, *_C13_PROBE_PORTS):
            if int(p) not in ports:
                ports.append(int(p))
        profiles: list[SkydroidCommandProfile] = [self._profile]
        if self._profile.profile_id != "c13_alt":
            profiles.append(get_profile("c13_alt"))
        tried: list[str] = []
        for profile in profiles:
            prev = self._profile
            self._profile = profile
            for host in self._hosts:
                self._transport.set_route_host(host)
                self._transport._host = host
                for port in ports:
                    self._transport._port = int(port)
                    tried.append(f"{host}:{port}")
                    if self._poll_host_once(host):
                        self._active_host = host
                        self._active_port = int(port)
                        self._profile_id = profile.profile_id
                        print(
                            f"[VGCS:skydroid] gimbal OK via {host}:{port} profile={profile.profile_id}"
                        )
                        return
            self._profile = prev
        self._profile_id = self._profile.profile_id
        self._transport._host = self._active_host
        self._transport._port = configured
        print(
            f"[VGCS:skydroid] gimbal probe failed ({len(tried)} tries). "
            f"See logs/skydroid_top_udp.log — RTSP can work without TOP UDP from this PC."
        )

    @staticmethod
    def _is_motion_command(commands: list[str]) -> bool:
        return bool(commands) and str(commands[0]).upper() in _MOTION_COMMANDS

    def _drop_pending_motion_commands(self) -> None:
        """Keep only the latest motion command — avoids queue backlog and UI lag."""
        pending: list[tuple[list[str], dict[str, object], bool]] = []
        while True:
            try:
                pending.append(self._queue.get_nowait())
            except queue.Empty:
                break
        for item in pending:
            cmds, params, expect_reply = item
            if self._is_motion_command(cmds):
                continue
            try:
                self._queue.put_nowait((cmds, params, expect_reply))
            except queue.Full:
                pass

    def _enqueue(self, commands: list[str], params: dict[str, object], expect_reply: bool) -> None:
        if not commands:
            return
        if not self._running:
            self.start()
        if not expect_reply and self._is_motion_command(commands):
            self._drop_pending_motion_commands()
        try:
            self._queue.put_nowait((list(commands), params, expect_reply))
        except queue.Full:
            try:
                _ = self._queue.get_nowait()
            except Exception:
                pass
            try:
                self._queue.put_nowait((list(commands), params, expect_reply))
            except Exception:
                pass

    def _worker_loop(self) -> None:
        while self._running:
            try:
                commands, params, expect_reply = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if not self._running:
                break
            wait_s = self._min_dt - (time.monotonic() - self._last_send_mono)
            if not expect_reply:
                wait_s = min(wait_s, 0.03)
            if wait_s > 0:
                time.sleep(wait_s)
            self._last_send_mono = time.monotonic()
            if commands and commands[0] == "NOOP":
                continue
            if commands and commands[0] == "ZOOM_BURST":
                try:
                    level = float(params.get("level", 1.0) or 1.0)
                    self._send_zoom_burst(level)
                except Exception:
                    pass
                continue
            for command in commands:
                frame = build_top_frame(command, params)
                try:
                    reply = self._transport.send_and_receive(
                        frame,
                        expect_reply=expect_reply,
                        log=True,
                        timeout_s=0.08 if not expect_reply else None,
                    )
                    if expect_reply and reply:
                        self._maybe_update_status(reply)
                    break
                except Exception:
                    if not expect_reply:
                        break
                    continue

    def _send_zoom_burst(self, level: float) -> None:
        """Fire every known zoom frame on active + alternate TOP UDP ports (no reply wait)."""
        frames = build_zoom_command_burst(level)
        if not frames:
            return
        active_port = int(self._transport._port)
        ports: list[int] = []
        for p in (active_port, *_ZOOM_EXTRA_PORTS):
            if int(p) not in ports:
                ports.append(int(p))
        for port in ports:
            self._transport._port = int(port)
            for frame in frames:
                try:
                    self._transport.send_and_receive(
                        frame,
                        expect_reply=False,
                        log=True,
                        timeout_s=0.05,
                    )
                except Exception:
                    continue
        self._transport._port = active_port

    def _status_loop(self) -> None:
        while self._running:
            if not self._probe_finished.wait(timeout=0.25):
                continue
            self._poll_active_endpoint_once()
            interval = 1.0 if not self.gimbal_telemetry_ok() else 0.5
            time.sleep(interval)

    def _poll_laser_range_once(self) -> None:
        """C13 single laser rangefinding (PROTOCAL §4.22 SLR read on D-class)."""
        if not self.gimbal_telemetry_ok():
            return
        self._query_slr_distance_m(log=False)

    def _maybe_update_slr(self, payload: bytes) -> None:
        dist_m = parse_slr_distance_from_payload(payload)
        if dist_m is None:
            return
        if self._lrf_locked:
            self._note_slr_sample(float(dist_m))
            return
        with self._status_lock:
            self._laser_range_m = float(dist_m)
            self._laser_range_mono = time.monotonic()

    def _maybe_update_status(self, payload: bytes) -> None:
        dec = parse_top_frame(payload)
        if dec is None:
            return
        if dec.command == "SLR":
            self._maybe_update_slr(payload)
            return
        yaw, pitch = extract_attitude_deg(dec)
        if yaw is None and pitch is None:
            if dec.command not in self._profile.status_response_commands:
                return
        with self._status_lock:
            self._status = GimbalStatus(
                yaw_deg=yaw,
                pitch_deg=pitch,
                supported=(yaw is not None or pitch is not None),
                updated_mono=time.monotonic(),
            )
