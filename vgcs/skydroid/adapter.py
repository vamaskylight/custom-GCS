from __future__ import annotations

import os
import queue
import re
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from vgcs.skydroid.command_map import SkydroidCommandProfile, get_profile
from vgcs.skydroid.protocol import (
    build_c13_zoom_step_frames,
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
_LRF_LOCK_MIN_WAIT_S = 0.1
_LRF_LOCK_MAX_WAIT_S = 8.0
_LRF_LOCK_POLL_S = 0.1
_LRF_BASELINE_EPS_M = 1.5
_LRF_STABLE_EPS_M = 0.35
_LRF_MOVED_MIN_M = 2.0
_LRF_GIMBAL_SLR_MIN_M = 0.5
_LRF_GIMBAL_ACCEPT_MIN_DEG = 1.5
_LRF_GIMBAL_SLR_SETTLE_S = 0.5
_LRF_GIMBAL_SLR_SETTLE_ALIGN_OK_S = 0.0
_LRF_CONVERGE_SAMPLES = 2
_LRF_CONVERGE_SAMPLES_ALIGNED = 1
_LRF_FAST_LOCK_OFFSET_DEG = 1.5
_LRF_REFINE_SAMPLES = 3
_LRF_REFINE_HOLD_S = 0.75
_LRF_REFINE_EXTEND_S = 1.0
_LRF_CLIMB_EPS_M = 0.35
_LRF_SETTLE_WINDOW = 8
_LRF_SETTLE_MIN_DRIFT_M = 2.0
_LRF_POST_MOVE_MIN_WAIT_S = 0.1
_LRF_POST_MOVE_SETTLE_S = 0.2
_LRF_POST_ALIGN_SLR_SETTLE_S = 0.45
_LRF_GOT_RESEND_INTERVAL_S = 2.0
_LRF_BASELINE_RETRIES = 8
_LRF_BASELINE_GAP_S = 0.2
_LRF_POST_GOT_WAIT_S = 0.1
_LRF_SLR_SHOT_SETTLE_S = 0.12
_LRF_GAM_APPROACH_DPS = 20.0
_LRF_GAM_SETTLE_S = 2.0
_LRF_GAM_AIM_TOL_DEG = 2.5
_LRF_GAM_WAIT_S = 4.5
_LRF_GIMBAL_MOVE_MIN_DEG = 0.35
_LRF_FOV_H_DEG = 83.4
_LRF_FOV_V_DEG = 46.9
_LRF_REPOINT_AFTER_S = 4.0
_LRF_GIMBAL_SLEW_SPEED_DPS = 2.0
_LRF_GIMBAL_HOLD_REFRESH_S = 0.08
_LRF_GIMBAL_AXIS_MAX_S = 1.15
_LRF_GIMBAL_SLEW_SCALE = 0.90
_LRF_C13_INVERT_GSY = True
_LRF_C13_INVERT_GSP = False
_LRF_C13_NEGATE_IMAGE_YAW = True
_LRF_ALIGN_MIN_OFFSET_DEG = 0.35
_LRF_ALIGN_SPEED_DPS = 3.5
_LRF_ALIGN_BULK_MIN_DEG = 4.0
_LRF_ALIGN_BULK_SPEED_DPS = 4.5
_LRF_ALIGN_MAX_ITERS = 16
_LRF_ALIGN_TOL_DEG = 2.5
_LRF_ALIGN_YAW_TOL_DEG = 0.75
_LRF_ALIGN_PITCH_TOL_DEG = 0.75
_LRF_ALIGN_SNAP_MIN_DEG = 0.35
_LRF_ALIGN_GAP_PITCH_MIN_DEG = 3.0
_LRF_ALIGN_GAP_YAW_MIN_DEG = 3.0
_LRF_ALIGN_MAX_TOTAL_DEG = 45.0
_LRF_ALIGN_MAX_TOTAL_CAP_DEG = 55.0
_LRF_HOLD_MAX_CLICK_OFFSET_DEG = 4.0
_LRF_HOLD_MIN_PIXEL_MOVE_PX = 48
_LRF_LOCK_GIMBAL_DRIFT_MAX_DEG = 6.0
_LRF_PRELOCK_SAMPLES = 1
_LRF_LIVE_POLL_SAMPLES = 3
_LRF_SLR_QUERY_RETRIES = 3
_LRF_SLR_DEST_LASER = "E"
_LRF_SLR_DEST_SYSTEM = "D"
_LRF_SLR_DIVERGE_WARN_M = 2.0
_LRF_SLR_JUMP_RATIO_MAX = 2.5
_LRF_SLR_JUMP_MIN_M = 12.0
_LRF_BORESIGHT_TOL_U = 0.032
_LRF_BORESIGHT_TOL_V = 0.040
_LRF_GIMBAL_OVERSHOOT_FACTOR = 1.45
_LRF_GIMBAL_SLEW_SETTLE_S = 0.32
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
        port_i = int(port)
        if port_i not in _C13_PROBE_PORTS:
            try:
                print(
                    f"[VGCS:skydroid] configured TOP UDP port {port_i} is not PROTOCAL "
                    f"default 5000 — background probe will use 192.168.144.108:5000"
                )
            except Exception:
                pass
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
        self._lrf_armed = False
        self._lrf_lock_x = 0
        self._lrf_lock_y = 0
        self._lrf_locked_distance_m: float | None = None
        self._active_port = int(port)
        self._transport.set_datagram_handler(self._maybe_update_status)

    def active_endpoint(self) -> tuple[str, int, str]:
        return (str(self._active_host), int(self._active_port), str(self._profile_id))

    def get_laser_range_m(self, *, max_age_s: float = 4.0) -> float | None:
        """C13 SLR metres — locked value, or latest live read while LRF is armed."""
        with self._status_lock:
            if self._lrf_locked:
                dist = self._lrf_locked_distance_m
            elif self._lrf_armed:
                dist = self._laser_range_m
            else:
                return None
            ts = float(self._laser_range_mono or 0.0)
        if dist is None or ts <= 0.0:
            return None
        if time.monotonic() - ts > max(0.5, float(max_age_s)):
            return None
        return float(dist)

    def set_lrf_armed(self, armed: bool) -> None:
        with self._status_lock:
            self._lrf_armed = bool(armed)
            if not self._lrf_armed and not self._lrf_locked:
                self._laser_range_m = None
                self._laser_range_mono = 0.0

    def is_lrf_armed(self) -> bool:
        with self._status_lock:
            return bool(self._lrf_armed)

    def poll_live_slr(self, *, log: bool = False) -> float | None:
        """Fresh multi-shot SLR while armed (no lock); updates cached live readout."""
        if not self.gimbal_telemetry_ok():
            return None
        with self._status_lock:
            if self._lrf_locked:
                return self._lrf_locked_distance_m
            if not self._lrf_armed:
                return None
        samples: list[float] = []
        for i in range(int(_LRF_LIVE_POLL_SAMPLES)):
            reading = self._query_slr_distance_m(log=log and i == 0, fresh=True)
            if reading is not None:
                samples.append(float(reading))
            if i + 1 < int(_LRF_LIVE_POLL_SAMPLES):
                time.sleep(0.1)
        if not samples:
            return None
        value = float(self._slr_median(samples))
        with self._status_lock:
            if not self._lrf_locked:
                self._laser_range_m = value
                self._laser_range_mono = time.monotonic()
        return value

    def _read_slr_prelock_median(self) -> float | None:
        """Several fresh SLR shots before lock — reduces single-shot error."""
        with self._status_lock:
            cached = self._laser_range_m
            age = (
                time.monotonic() - float(self._laser_range_mono)
                if float(self._laser_range_mono or 0.0) > 0.0
                else 999.0
            )
        if cached is not None and age < 0.45:
            return float(cached)

        samples: list[float] = []
        n = int(_LRF_PRELOCK_SAMPLES)
        for i in range(n):
            reading = self._query_slr_distance_m(log=i == 0, fresh=True)
            if reading is not None:
                samples.append(float(reading))
            if i + 1 < n:
                time.sleep(0.08)
        if not samples:
            return None
        if len(samples) >= 5:
            med = float(self._slr_trimmed_median(samples, trim=1))
        else:
            med = float(self._slr_median(samples))
        spread = max(samples) - min(samples)
        if len(samples) >= 2 and spread > 0.5:
            print(
                f"[VGCS:lrf] pre-lock spread {spread:.1f} m "
                f"samples={[round(s, 1) for s in samples]} median={med:.1f} m"
            )
        return med

    def is_lrf_locked(self) -> bool:
        with self._status_lock:
            return bool(self._lrf_locked)

    def _send_got_mark_only(self, x_px: int, y_px: int, *, frame_w: int, frame_h: int) -> None:
        """Mark click on video for overlay — stop track, no SUM confirm (avoids gimbal slew)."""
        stop = build_sum_track(confirm=False)
        self._transport.send_and_receive(
            stop, expect_reply=False, log=False, timeout_s=0.08
        )
        time.sleep(0.08)
        got = build_got_target(x_px, y_px, frame_w=frame_w, frame_h=frame_h)
        self._transport.send_and_receive(
            got, expect_reply=False, log=True, timeout_s=0.08
        )

    def _send_got_track(self, x_px: int, y_px: int, *, frame_w: int, frame_h: int) -> None:
        """Legacy C12 visual track (GOT + SUM confirm) — not used for C13 LRF lock."""
        if _LRF_GOT_MARK_ONLY:
            self._send_got_mark_only(x_px, y_px, frame_w=frame_w, frame_h=frame_h)
            return
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
        yaw_tol = self._align_yaw_tol_deg()
        pitch_tol = self._align_pitch_tol_deg(float(dpitch))
        pitch_trusted = self._gac_pitch_trusted(float(att_end[1]), float(dpitch))
        if pitch_trusted:
            pitch_err = abs(self._angle_err_deg(pitch_tgt, att_end[1]))
        else:
            pitch_err = 0.0
        if yaw_err > float(yaw_tol) and yaw_delta < abs(float(dyaw)) * 0.35:
            print(
                f"[VGCS:lrf] gimbal missed aim yaw_err={yaw_err:.1f}° "
                f"target={yaw_tgt:.1f} att={att_end}"
            )
            return False
        if pitch_trusted and pitch_err > float(pitch_tol):
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
    def _c13_negate_image_yaw() -> bool:
        """C13 GAC yaw axis is opposite companion-video horizontal (field firmware)."""
        if str(os.environ.get("VGCS_LRF_NEGATE_YAW_DELTA", "") or "").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return False
        if str(os.environ.get("VGCS_LRF_NEGATE_YAW_DELTA", "") or "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            return True
        return bool(_LRF_C13_NEGATE_IMAGE_YAW)

    @staticmethod
    def _image_yaw_to_gac_delta(dyaw_image: float) -> float:
        """Map video-frame yaw offset (right = +) to GAC yaw delta."""
        d = float(dyaw_image)
        if SkydroidTopUdpAdapter._c13_negate_image_yaw():
            d = -d
        return d

    @staticmethod
    def _gimbal_yaw_target_deg(att_yaw: float, dyaw_image: float) -> float:
        return float(att_yaw) + SkydroidTopUdpAdapter._image_yaw_to_gac_delta(dyaw_image)

    @staticmethod
    def _gac_to_image_yaw_delta(dyaw_gac: float) -> float:
        d = float(dyaw_gac)
        if SkydroidTopUdpAdapter._c13_negate_image_yaw():
            d = -d
        return d

    @staticmethod
    def _gac_to_image_pitch_delta(dpitch_gac: float) -> float:
        return -float(dpitch_gac)

    @staticmethod
    def lrf_track_uv_from_attitude(
        ref_uv: tuple[float, float],
        ref_att: tuple[float, float],
        cur_att: tuple[float, float],
        *,
        gac_h_scale: float = 1.0,
        gac_v_scale: float = 1.0,
        clamp: bool = True,
    ) -> tuple[float, float]:
        """Screen UV of a world-fixed point marked at ref_uv when gimbal was at ref_att."""
        u0, v0 = float(ref_uv[0]), float(ref_uv[1])
        yaw0, pitch0 = float(ref_att[0]), float(ref_att[1])
        yaw, pitch = float(cur_att[0]), float(cur_att[1])
        dyaw_img0 = (u0 - 0.5) * _LRF_FOV_H_DEG
        dpitch_img0 = (v0 - 0.5) * _LRF_FOV_V_DEG
        gac_dy = yaw - yaw0
        gac_dp = pitch - pitch0
        hs = max(0.75, min(1.25, float(gac_h_scale)))
        vs = max(0.75, min(1.25, float(gac_v_scale)))
        img_dy_delta = (
            SkydroidTopUdpAdapter._gac_to_image_yaw_delta(gac_dy) * hs
        )
        img_dp_delta = (
            SkydroidTopUdpAdapter._gac_to_image_pitch_delta(gac_dp) * vs
        )
        u = 0.5 + (dyaw_img0 - img_dy_delta) / _LRF_FOV_H_DEG
        v = 0.5 + (dpitch_img0 - img_dp_delta) / _LRF_FOV_V_DEG
        if not clamp:
            return (float(u), float(v))
        return (
            max(0.0, min(1.0, float(u))),
            max(0.0, min(1.0, float(v))),
        )

    @staticmethod
    def calibrate_track_gac_scales(
        ref_uv: tuple[float, float],
        ref_att: tuple[float, float],
        lock_att: tuple[float, float],
        *,
        h_scale: float = 1.0,
        v_scale: float = 1.0,
        live: bool = False,
    ) -> tuple[float, float]:
        """Learn GAC→image scale from click-to-aim slew (reduces pan tracking error)."""
        u0, v0 = float(ref_uv[0]), float(ref_uv[1])
        dyaw_img0 = (u0 - 0.5) * _LRF_FOV_H_DEG
        dpitch_img0 = (v0 - 0.5) * _LRF_FOV_V_DEG
        gac_dy = float(lock_att[0]) - float(ref_att[0])
        gac_dp = float(lock_att[1]) - float(ref_att[1])
        img_dy = SkydroidTopUdpAdapter._gac_to_image_yaw_delta(gac_dy)
        img_dp = SkydroidTopUdpAdapter._gac_to_image_pitch_delta(gac_dp)
        h_new = float(h_scale)
        v_new = float(v_scale)
        min_gac = 0.12 if live else 0.4
        min_img = 0.35 if live else 1.0
        w_meas = 0.65 if live else 0.4
        w_old = 1.0 - w_meas
        if abs(float(img_dy)) > min_gac and abs(float(dyaw_img0)) > min_img:
            measured = abs(float(dyaw_img0)) / abs(float(img_dy))
            h_new = measured * w_meas + float(h_scale) * w_old
        if abs(float(img_dp)) > min_gac and abs(float(dpitch_img0)) > min_img:
            measured = abs(float(dpitch_img0)) / abs(float(img_dp))
            v_new = measured * w_meas + float(v_scale) * w_old
        return (
            max(0.75, min(1.25, float(h_new))),
            max(0.75, min(1.25, float(v_new))),
        )

    @staticmethod
    def _c13_invert_gsy() -> bool:
        if str(os.environ.get("VGCS_LRF_INVERT_GSY", "") or "").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return False
        if str(os.environ.get("VGCS_LRF_INVERT_GSY", "") or "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            return True
        return bool(_LRF_C13_INVERT_GSY)

    @staticmethod
    def _gsy_yaw_rate_for_offset(dyaw: float, spd: float) -> float:
        """C13 field firmware: GSY sign is opposite PROTOCAL §3.2.1 (yaw right = positive)."""
        rate = float(spd) if float(dyaw) > 0.0 else -float(spd)
        if SkydroidTopUdpAdapter._c13_invert_gsy():
            rate = -rate
        return rate

    @staticmethod
    def _c13_invert_gsp() -> bool:
        if str(os.environ.get("VGCS_LRF_INVERT_GSP", "") or "").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return False
        if str(os.environ.get("VGCS_LRF_INVERT_GSP", "") or "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            return True
        return bool(_LRF_C13_INVERT_GSP)

    @staticmethod
    def _gsp_pitch_rate_for_image_offset(dpitch_image: float, spd: float) -> float:
        """Image offset (+ = below centre) → GSP rate to centre laser on click."""
        rate = -float(spd) if float(dpitch_image) > 0.0 else float(spd)
        if SkydroidTopUdpAdapter._c13_invert_gsp():
            rate = -rate
        return rate

    @staticmethod
    def _gsp_pitch_rate_for_error(pitch_err: float, spd: float) -> float:
        rate = float(spd) if float(pitch_err) > 0.0 else -float(spd)
        if SkydroidTopUdpAdapter._c13_invert_gsp():
            rate = -rate
        return rate

    @staticmethod
    def _gac_pitch_trusted(att_pitch: float, dpitch_image: float) -> bool:
        """C13 GAC pitch often reads 0.0° even while the gimbal tilts (yaw is reliable)."""
        if abs(float(dpitch_image)) < 2.5:
            return True
        return abs(float(att_pitch)) > 0.15

    @staticmethod
    def _gimbal_pitch_target_deg(att_pitch: float, dpitch_image: float) -> float:
        return max(-90.0, min(90.0, float(att_pitch) - float(dpitch_image)))

    @staticmethod
    def _align_yaw_tol_deg() -> float:
        try:
            return float(
                os.environ.get(
                    "VGCS_LRF_ALIGN_YAW_TOL",
                    str(_LRF_ALIGN_YAW_TOL_DEG),
                )
                or _LRF_ALIGN_YAW_TOL_DEG
            )
        except ValueError:
            return float(_LRF_ALIGN_YAW_TOL_DEG)

    @staticmethod
    def _align_pitch_tol_deg(dpitch_image: float) -> float:
        try:
            base = float(
                os.environ.get(
                    "VGCS_LRF_ALIGN_PITCH_TOL",
                    str(_LRF_ALIGN_PITCH_TOL_DEG),
                )
                or _LRF_ALIGN_PITCH_TOL_DEG
            )
        except ValueError:
            base = float(_LRF_ALIGN_PITCH_TOL_DEG)
        # Large vertical clicks need tighter aim than the global yaw tolerance.
        need = abs(float(dpitch_image))
        if need >= 12.0:
            return min(base, 0.55)
        if need >= 6.0:
            return min(base, 0.65)
        return base

    @staticmethod
    def _align_speed_for_need(need_deg: float) -> float:
        try:
            base = float(
                os.environ.get(
                    "VGCS_LRF_ALIGN_SPEED_DPS",
                    str(_LRF_ALIGN_SPEED_DPS),
                )
                or _LRF_ALIGN_SPEED_DPS
            )
        except ValueError:
            base = float(_LRF_ALIGN_SPEED_DPS)
        need = abs(float(need_deg))
        if need >= 15.0:
            return max(base, float(_LRF_ALIGN_BULK_SPEED_DPS))
        if need >= 8.0:
            return max(base, 6.5)
        if need >= float(_LRF_ALIGN_BULK_MIN_DEG):
            return max(base, 5.5)
        return base

    def _align_bulk_goto(
        self,
        yaw_tgt: float,
        pitch_tgt: float,
        att: tuple[float, float],
        *,
        spd: float,
    ) -> tuple[float, float]:
        """Large click: one GAP + one GAY at moderate speed, single stop at end."""
        att_now = att
        bulk_spd = max(float(spd), float(_LRF_ALIGN_BULK_SPEED_DPS))
        pitch_delta = abs(self._angle_err_deg(float(pitch_tgt), float(att_now[1])))
        yaw_delta = abs(self._angle_err_deg(float(yaw_tgt), float(att_now[0])))
        moved = False
        if pitch_delta >= 2.5:
            print(
                f"[VGCS:lrf] align bulk pitch GAP -> {float(pitch_tgt):.1f}° "
                f"(Δ={pitch_delta:.1f}° spd={bulk_spd:.0f})"
            )
            self._send_gap_pitch_direct(
                float(pitch_tgt),
                bulk_spd,
                att_pitch=float(att_now[1]),
                finalize=False,
            )
            att_now = self._read_gimbal_attitude_deg() or att_now
            moved = True
        if yaw_delta >= 2.5:
            print(
                f"[VGCS:lrf] align bulk yaw GAY -> {float(yaw_tgt):.1f}° "
                f"(Δ={yaw_delta:.1f}° spd={bulk_spd:.0f})"
            )
            self._send_gay_yaw_direct(
                float(yaw_tgt),
                bulk_spd,
                att_yaw=float(att_now[0]),
                finalize=False,
            )
            att_now = self._read_gimbal_attitude_deg() or att_now
            moved = True
        if moved:
            time.sleep(_LRF_GIMBAL_SLEW_SETTLE_S)
            self._gimbal_stop_hard()
        return att_now

    def _send_gay_yaw_direct(
        self,
        yaw_tgt: float,
        spd: float,
        *,
        att_yaw: float = 0.0,
        finalize: bool = True,
    ) -> None:
        """Absolute yaw (GAY) — large horizontal clicks."""
        self._ensure_active_transport()
        self._drop_pending_motion_commands()
        try:
            self._transport.send_and_receive(
                build_top_frame(
                    "GAY",
                    {"yaw": float(yaw_tgt), "speed": float(spd)},
                ),
                expect_reply=False,
                log=True,
                timeout_s=0.12,
            )
        except Exception:
            pass
        delta = abs(self._angle_err_deg(float(yaw_tgt), float(att_yaw)))
        settle = min(2.5, delta / max(3.0, float(spd)) + 0.30)
        time.sleep(float(settle))
        if finalize:
            self._gimbal_stop_hard()

    def _send_gap_pitch_direct(
        self,
        pitch_tgt: float,
        spd: float,
        *,
        att_pitch: float = 0.0,
        finalize: bool = True,
    ) -> None:
        """Absolute pitch (GAP) — precise vertical aim on C13."""
        self._ensure_active_transport()
        self._drop_pending_motion_commands()
        try:
            self._transport.send_and_receive(
                build_top_frame(
                    "GAP",
                    {"pitch": float(pitch_tgt), "speed": float(spd)},
                ),
                expect_reply=False,
                log=True,
                timeout_s=0.12,
            )
        except Exception:
            pass
        delta = abs(self._angle_err_deg(float(pitch_tgt), float(att_pitch)))
        settle = min(2.5, delta / max(3.0, float(spd)) + 0.30)
        time.sleep(float(settle))
        if finalize:
            self._gimbal_stop_hard()

    def _refine_align_axes(
        self,
        att: tuple[float, float],
        yaw_tgt: float,
        pitch_tgt: float,
        dyaw_image: float,
        dpitch_image: float,
        *,
        spd: float = _LRF_ALIGN_SPEED_DPS,
    ) -> tuple[float, float]:
        """Final GAY/GAP nudge when iterative bursts stop short of click."""
        att_now = self._read_gimbal_attitude_deg() or att
        snap = float(_LRF_ALIGN_SNAP_MIN_DEG)
        yaw_err = self._angle_err_deg(float(yaw_tgt), float(att_now[0]))
        pitch_err = self._angle_err_deg(float(pitch_tgt), float(att_now[1]))
        if abs(float(pitch_err)) > snap:
            print(
                f"[VGCS:lrf] refine pitch GAP {float(att_now[1]):.1f}° "
                f"-> {float(pitch_tgt):.1f}° (err={float(pitch_err):+.1f}°)"
            )
            self._send_gap_pitch_direct(
                float(pitch_tgt), float(spd), att_pitch=float(att_now[1])
            )
            att_now = self._read_gimbal_attitude_deg() or att_now
            pitch_err = self._angle_err_deg(float(pitch_tgt), float(att_now[1]))
        yaw_err = self._angle_err_deg(float(yaw_tgt), float(att_now[0]))
        if abs(float(yaw_err)) > snap:
            print(
                f"[VGCS:lrf] refine yaw GAY {float(att_now[0]):.1f}° "
                f"-> {float(yaw_tgt):.1f}° (err={float(yaw_err):+.1f}°)"
            )
            self._send_gay_yaw_direct(
                float(yaw_tgt), float(spd), att_yaw=float(att_now[0])
            )
            att_now = self._read_gimbal_attitude_deg() or att_now
        return att_now

    @staticmethod
    def _click_uv_from_px(
        x_px: int, y_px: int, *, fw: int = _LRF_FRAME_W, fh: int = _LRF_FRAME_H
    ) -> tuple[float, float]:
        u = max(0.0, min(1.0, float(x_px) / max(1.0, float(fw))))
        v = max(0.0, min(1.0, float(y_px) / max(1.0, float(fh))))
        return (float(u), float(v))

    def _verify_click_on_boresight(
        self,
        u: float,
        v: float,
        att_at_click: tuple[float, float],
        att_now: tuple[float, float],
        *,
        gac_h_scale: float = 1.0,
        gac_v_scale: float = 1.0,
    ) -> tuple[bool, float, float]:
        """True when the clicked world point reprojects to frame centre at att_now."""
        u_chk, v_chk = SkydroidTopUdpAdapter.lrf_track_uv_from_attitude(
            (float(u), float(v)),
            (float(att_at_click[0]), float(att_at_click[1])),
            (float(att_now[0]), float(att_now[1])),
            gac_h_scale=float(gac_h_scale),
            gac_v_scale=float(gac_v_scale),
            clamp=False,
        )
        ok = (
            abs(float(u_chk) - 0.5) <= float(_LRF_BORESIGHT_TOL_U)
            and abs(float(v_chk) - 0.5) <= float(_LRF_BORESIGHT_TOL_V)
        )
        return bool(ok), float(u_chk), float(v_chk)

    @staticmethod
    def _slr_plausible_after_slew(
        pre_slr: float | None,
        post_slr: float,
        click_offset_deg: float,
    ) -> bool:
        """Reject locks only when SLR jumps farther (background), not when it moves nearer."""
        if pre_slr is None or float(click_offset_deg) < 3.0:
            return True
        pre = max(0.5, float(pre_slr))
        post = float(post_slr)
        # Pre-lock range is often along the OLD boresight (background). After click-to-aim
        # the laser correctly reads a nearer surface — field log 92 m -> 8 m must accept.
        if post < pre - 2.0:
            if pre >= 25.0 and post <= 30.0 and float(click_offset_deg) >= 5.0:
                print(
                    f"[VGCS:lrf] SLR nearer after aim {pre:.1f} m -> {post:.1f} m "
                    f"(background pre-lock → click target)"
                )
            return True
        jump = post - pre
        if jump < float(_LRF_SLR_JUMP_MIN_M):
            return True
        ratio = post / pre
        # Foreground pre-lock jumping to distant background (12 m -> 94 m).
        if pre < 14.0 and post > max(35.0, pre * 2.8):
            print(
                f"[VGCS:lrf] lock rejected — SLR {pre:.1f} m -> {post:.1f} m "
                f"(Δ={jump:.1f} m) after {float(click_offset_deg):.1f}° click-to-aim — "
                f"near target was {pre:.0f} m but laser locked background; "
                f"pick a feature with clear range or move closer"
            )
            return False
        if jump >= 25.0 and ratio > 3.5:
            print(
                f"[VGCS:lrf] lock rejected — SLR {pre:.1f} m -> {post:.1f} m "
                f"(Δ={jump:.1f} m) after {float(click_offset_deg):.1f}° click-to-aim — "
                f"laser likely hit background; pick a nearer feature or re-aim"
            )
            return False
        return True

    def _align_axes_ok(
        self,
        att: tuple[float, float],
        yaw_tgt: float,
        pitch_tgt: float,
        dpitch_image: float,
        *,
        pitch_open_sent: float = 0.0,
        pitch_trusted: bool = True,
    ) -> bool:
        yaw_tol = self._align_yaw_tol_deg()
        pitch_tol = self._align_pitch_tol_deg(float(dpitch_image))
        yaw_err = abs(self._angle_err_deg(float(yaw_tgt), float(att[0])))
        if pitch_trusted:
            pitch_err = abs(self._angle_err_deg(float(pitch_tgt), float(att[1])))
            pitch_ok = pitch_err <= float(pitch_tol)
        else:
            # Open-loop pitch counters are not proof of aim — large clicks need GAC or verify.
            pitch_ok = (
                abs(float(dpitch_image)) < 3.0
                and float(pitch_open_sent) >= abs(float(dpitch_image)) * 0.88
            )
        return yaw_err <= float(yaw_tol) and pitch_ok

    def _align_aim_satisfied(
        self,
        att: tuple[float, float] | None,
        yaw_tgt: float,
        pitch_tgt: float,
        dpitch: float,
        *,
        pitch_open_sent_deg: float = 0.0,
        dyaw: float = 0.0,
    ) -> bool:
        """Strict post-align check — must be on target, not merely 'moved a bit'."""
        if att is None:
            return False
        yaw_tol = self._align_yaw_tol_deg()
        pitch_tol = self._align_pitch_tol_deg(float(dpitch))
        yaw_err = abs(self._angle_err_deg(float(yaw_tgt), float(att[0])))
        pitch_err = abs(self._angle_err_deg(float(pitch_tgt), float(att[1])))
        pitch_trusted = self._gac_pitch_trusted(float(att[1]), float(dpitch))
        if yaw_err > float(yaw_tol):
            print(
                f"[VGCS:lrf] align miss yaw_err={yaw_err:.1f}° "
                f"(target={float(yaw_tgt):.1f} att={float(att[0]):.1f})"
            )
            return False
        if pitch_trusted:
            if pitch_err > float(pitch_tol):
                print(
                    f"[VGCS:lrf] align miss pitch_err={pitch_err:.1f}° "
                    f"(target={float(pitch_tgt):.1f} att={float(att[1]):.1f})"
                )
                return False
        elif float(pitch_open_sent_deg) < abs(float(dpitch)) * 0.82:
            print(
                f"[VGCS:lrf] align miss pitch open-loop "
                f"{pitch_open_sent_deg:.1f}° of {abs(float(dpitch)):.1f}°"
            )
            return False
        return True

    @staticmethod
    def _align_move_cap_deg(need_deg: float) -> float:
        try:
            base = float(
                os.environ.get(
                    "VGCS_LRF_ALIGN_MAX_DEG",
                    str(_LRF_ALIGN_MAX_TOTAL_DEG),
                )
                or _LRF_ALIGN_MAX_TOTAL_DEG
            )
        except ValueError:
            base = float(_LRF_ALIGN_MAX_TOTAL_DEG)
        return min(
            float(_LRF_ALIGN_MAX_TOTAL_CAP_DEG),
            max(base, float(need_deg) * 1.15 + 4.0),
        )

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
        gac_dyaw = self._image_yaw_to_gac_delta(dyaw)
        if abs(float(gac_dyaw)) >= 0.3:
            yaw_r = self._gsy_yaw_rate_for_offset(gac_dyaw, spd)
            dur_y = self._axis_burst_duration_s(gac_dyaw, spd)
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
            pitch_r = self._gsp_pitch_rate_for_image_offset(float(dpitch), spd)
            dur_p = min(1.0, self._axis_burst_duration_s(dpitch, spd))
            print(f"[VGCS:lrf] pitch burst {pitch_r:+.1f}°/s dur={dur_p:.2f}s")
            self._hold_speed_direct_burst(0.0, pitch_r, dur_p)
        self._gimbal_stop_hard()
        time.sleep(_LRF_GIMBAL_SLEW_SETTLE_S)
        return self._read_gimbal_attitude_deg() or att

    def _align_laser_boresight_to_pixel(
        self,
        x_px: int,
        y_px: int,
        att_start: tuple[float, float],
        *,
        on_tick: Callable[[], None] | None = None,
    ) -> tuple[tuple[float, float] | None, bool, float, float]:
        """Iterative GSY/GSP bursts until laser boresight matches click (C13, no GAM)."""
        dyaw0, dpitch0 = self._pixel_boresight_offset_deg(x_px, y_px)
        need = self._expected_offset_deg(dyaw0, dpitch0)
        if need < _LRF_ALIGN_MIN_OFFSET_DEG:
            return att_start, True, float(dyaw0), float(dpitch0)

        yaw_tgt = self._gimbal_yaw_target_deg(float(att_start[0]), float(dyaw0))
        pitch_tgt = self._gimbal_pitch_target_deg(float(att_start[1]), float(dpitch0))
        click_u, click_v = self._click_uv_from_px(x_px, y_px)
        spd = self._align_speed_for_need(float(need))
        att: tuple[float, float] = att_start
        total_move = 0.0
        pitch_open_sent = 0.0
        pitch_need_deg = abs(float(dpitch0))
        gap_pitch_sent = False
        gap_yaw_sent = False
        if float(need) >= float(_LRF_ALIGN_BULK_MIN_DEG):
            att = self._align_bulk_goto(
                float(yaw_tgt),
                float(pitch_tgt),
                att_start,
                spd=float(spd),
            )
            gap_pitch_sent = True
            gap_yaw_sent = abs(float(dyaw0)) >= 2.5
            pitch_open_sent = pitch_need_deg * 0.85
        yaw_tol = self._align_yaw_tol_deg()
        pitch_tol = self._align_pitch_tol_deg(float(dpitch0))
        move_cap = self._align_move_cap_deg(float(need))
        print(
            f"[VGCS:lrf] align laser px=({x_px},{y_px}) "
            f"-> ({yaw_tgt:.1f},{pitch_tgt:.1f}) "
            f"offset=({dyaw0:.1f}°,{dpitch0:.1f}°) "
            f"gac_yaw=({self._image_yaw_to_gac_delta(dyaw0):+.1f}°) "
            f"tol=(yaw={yaw_tol:.2f}°,pitch={pitch_tol:.2f}°) "
            f"cap={move_cap:.0f}°"
        )
        for n in range(int(_LRF_ALIGN_MAX_ITERS)):
            att_read = self._read_gimbal_attitude_deg() or att
            att = att_read
            yaw_err = self._angle_err_deg(yaw_tgt, att[0])
            pitch_err = self._angle_err_deg(pitch_tgt, att[1])
            pitch_trusted = self._gac_pitch_trusted(float(att[1]), float(dpitch0))
            if self._align_axes_ok(
                att,
                float(yaw_tgt),
                float(pitch_tgt),
                float(dpitch0),
                pitch_open_sent=float(pitch_open_sent),
                pitch_trusted=bool(pitch_trusted),
            ):
                att = self._refine_align_axes(
                    att,
                    float(yaw_tgt),
                    float(pitch_tgt),
                    float(dyaw0),
                    float(dpitch0),
                    spd=float(spd),
                )
                att = self._read_gimbal_attitude_deg() or att
                pitch_trusted = self._gac_pitch_trusted(float(att[1]), float(dpitch0))
                if self._align_axes_ok(
                    att,
                    float(yaw_tgt),
                    float(pitch_tgt),
                    float(dpitch0),
                    pitch_open_sent=float(pitch_open_sent),
                    pitch_trusted=bool(pitch_trusted),
                ):
                    bore_ok, u_chk, v_chk = self._verify_click_on_boresight(
                        click_u, click_v, att_start, att
                    )
                    if bore_ok:
                        yaw_e = self._angle_err_deg(float(yaw_tgt), float(att[0]))
                        pitch_e = self._angle_err_deg(float(pitch_tgt), float(att[1]))
                        print(
                            f"[VGCS:lrf] align ok iter={n + 1} att={att} "
                            f"residual=({float(yaw_e):+.2f}°,{float(pitch_e):+.2f}°) "
                            f"verify=({u_chk:.3f},{v_chk:.3f})"
                        )
                        return att, True, float(dyaw0), float(dpitch0)
                    print(
                        f"[VGCS:lrf] align iter={n + 1} — click still at "
                        f"({u_chk:.3f},{v_chk:.3f}), need boresight (0.500,0.500)"
                    )
                    gap_pitch_sent = False
                    pitch_open_sent = min(
                        float(pitch_open_sent), pitch_need_deg * 0.55
                    )
            yaw_err = self._angle_err_deg(yaw_tgt, att[0])
            pitch_err = self._angle_err_deg(pitch_tgt, att[1])
            pitch_ok = (
                abs(float(pitch_err)) <= float(pitch_tol)
                if pitch_trusted
                else pitch_open_sent >= pitch_need_deg * 0.88
            )
            yaw_ok = abs(float(yaw_err)) <= float(yaw_tol)
            if total_move >= float(move_cap):
                print(
                    f"[VGCS:lrf] align stopped — total move {total_move:.1f}° "
                    f"(cap={move_cap:.0f}°)"
                )
                break

            if not pitch_ok:
                spur = float(spd)
                if pitch_trusted and abs(float(pitch_err)) < 6.0:
                    spur = min(spur, max(1.5, abs(float(pitch_err)) * 1.2))
                use_gap = (
                    pitch_need_deg >= float(_LRF_ALIGN_GAP_PITCH_MIN_DEG)
                    and not gap_pitch_sent
                )
                if use_gap or (
                    not pitch_trusted
                    and pitch_need_deg >= 1.0
                    and not gap_pitch_sent
                ):
                    print(
                        f"[VGCS:lrf] align pitch GAP -> {pitch_tgt:.1f}° "
                        f"(need={pitch_need_deg:.1f}°)"
                    )
                    self._send_gap_pitch_direct(
                        float(pitch_tgt),
                        spur,
                        att_pitch=float(att[1]),
                    )
                    gap_pitch_sent = True
                    pitch_open_sent = max(
                        pitch_open_sent, pitch_need_deg * 0.95
                    )
                else:
                    if pitch_trusted:
                        rate = self._gsp_pitch_rate_for_error(
                            float(pitch_err), spur
                        )
                        need_d = abs(float(pitch_err))
                    else:
                        rate = self._gsp_pitch_rate_for_image_offset(
                            float(dpitch0), spur
                        )
                        need_d = max(0.0, pitch_need_deg - pitch_open_sent)
                    dur = min(
                        float(_LRF_GIMBAL_AXIS_MAX_S),
                        need_d / max(1.5, spur) * 0.95 + 0.10,
                    )
                    print(
                        f"[VGCS:lrf] align pitch "
                        f"{'err' if pitch_trusted else 'open'}="
                        f"{pitch_err if pitch_trusted else pitch_need_deg - pitch_open_sent:+.1f}° "
                        f"burst {rate:+.1f}°/s dur={dur:.2f}s"
                    )
                    self._hold_speed_direct_burst(0.0, rate, dur)
                    pitch_open_sent += spur * dur
            elif not yaw_ok:
                spur = float(spd)
                if abs(float(yaw_err)) < 6.0:
                    spur = min(spur, max(1.5, abs(float(yaw_err)) * 1.15))
                if (
                    abs(float(dyaw0)) >= float(_LRF_ALIGN_GAP_YAW_MIN_DEG)
                    and not gap_yaw_sent
                ):
                    print(
                        f"[VGCS:lrf] align yaw GAY -> {yaw_tgt:.1f}° "
                        f"(need={abs(float(dyaw0)):.1f}°)"
                    )
                    self._send_gay_yaw_direct(
                        float(yaw_tgt), spur, att_yaw=float(att[0])
                    )
                    gap_yaw_sent = True
                else:
                    dur = min(
                        float(_LRF_GIMBAL_AXIS_MAX_S),
                        abs(float(yaw_err)) / max(1.5, spur) * 0.95 + 0.10,
                    )
                    rate = self._gsy_yaw_rate_for_offset(float(yaw_err), spur)
                    print(
                        f"[VGCS:lrf] align yaw err={yaw_err:+.1f}° "
                        f"burst {rate:+.1f}°/s dur={dur:.2f}s"
                    )
                    self._hold_speed_direct_burst(rate, 0.0, dur)
            else:
                break

            self._gimbal_stop_hard()
            time.sleep(_LRF_GIMBAL_SLEW_SETTLE_S)
            att_new = self._read_gimbal_attitude_deg()
            if att_new is not None:
                total_move += self._gimbal_total_move_deg(att, att_new)
                att = att_new
            if on_tick is not None:
                try:
                    on_tick()
                except Exception:
                    pass

        att_end = self._refine_align_axes(
            att,
            float(yaw_tgt),
            float(pitch_tgt),
            float(dyaw0),
            float(dpitch0),
            spd=float(spd),
        )
        aim_ok = self._align_aim_satisfied(
            att_end,
            float(yaw_tgt),
            float(pitch_tgt),
            float(dpitch0),
            pitch_open_sent_deg=float(pitch_open_sent),
            dyaw=float(dyaw0),
        )
        bore_ok, u_chk, v_chk = self._verify_click_on_boresight(
            click_u, click_v, att_start, att_end
        )
        aim_ok = bool(aim_ok and bore_ok)
        if not bore_ok:
            print(
                f"[VGCS:lrf] align verify fail — click at ({u_chk:.3f},{v_chk:.3f}) "
                f"not boresight after slew"
            )
        move = self._gimbal_total_move_deg(att_start, att_end)
        print(
            f"[VGCS:lrf] align done att={att_start}->{att_end} "
            f"move={move:.1f}° ok={aim_ok}"
        )
        return att_end, aim_ok, float(dyaw0), float(dpitch0)

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
        yaw_tgt = self._gimbal_yaw_target_deg(float(att_now[0]), dyaw)
        pitch_tgt = self._gimbal_pitch_target_deg(float(att_now[1]), dpitch_img)
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
        """Closed-loop point at click; returns (attitude, aim_ok)."""
        if not self._lrf_lock_move_gimbal():
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
        yaw_tgt = self._gimbal_yaw_target_deg(float(att_now[0]), dyaw)
        pitch_tgt = self._gimbal_pitch_target_deg(float(att_now[1]), dpitch_img)
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
    def _slr_trimmed_median(values: list[float], *, trim: int = 1) -> float:
        s = sorted(float(v) for v in values)
        n = len(s)
        if n <= 0:
            return 0.0
        if n <= max(1, 2 * int(trim)):
            return SkydroidTopUdpAdapter._slr_median(s)
        core = s[int(trim) : n - int(trim)]
        return SkydroidTopUdpAdapter._slr_median(core)

    @staticmethod
    def _calibrate_slr_m(raw_m: float) -> float:
        """Optional field calibration: VGCS_LRF_OFFSET_M and VGCS_LRF_SCALE env vars."""
        try:
            scale = float(os.environ.get("VGCS_LRF_SCALE", "1") or "1")
            offset = float(os.environ.get("VGCS_LRF_OFFSET_M", "0") or "0")
        except ValueError:
            scale, offset = 1.0, 0.0
        return float(raw_m) * scale + offset

    @staticmethod
    def _slr_use_trigger() -> bool:
        """C13 field units usually need SLR write-01 before read; opt out with VGCS_LRF_TRIGGER=0."""
        env = str(os.environ.get("VGCS_LRF_TRIGGER", "") or "").strip().lower()
        if env in ("0", "false", "no", "off"):
            return False
        return True

    def _slr_probe_ports(self) -> tuple[int, ...]:
        active = int(self._active_port or self._transport._port or 5000)
        ordered: list[int] = [active]
        for port in _C13_PROBE_PORTS:
            p = int(port)
            if p not in ordered:
                ordered.append(p)
        return tuple(ordered)

    def _fire_slr_trigger(self, *, dest: str = _LRF_SLR_DEST_LASER) -> None:
        try:
            self._transport.send_and_receive(
                build_slr_trigger(dest=str(dest)),
                expect_reply=False,
                log=False,
                timeout_s=0.25,
            )
        except Exception:
            pass
        time.sleep(0.15)

    @staticmethod
    def _pick_slr_readings(
        laser_m: float | None,
        system_m: float | None,
        *,
        log: bool = False,
    ) -> float | None:
        """Prefer E-class (laser module); D-class is system/image path (PROTOCAL §2.2.2)."""
        if laser_m is not None and system_m is not None:
            if abs(float(laser_m) - float(system_m)) > _LRF_SLR_DIVERGE_WARN_M and log:
                print(
                    f"[VGCS:lrf] SLR E(laser)={float(laser_m):.1f} m "
                    f"D(system)={float(system_m):.1f} m — using E"
                )
            return float(laser_m)
        if laser_m is not None:
            return float(laser_m)
        if system_m is not None:
            return float(system_m)
        return None

    @staticmethod
    def _payload_is_slr_reply(payload: bytes) -> bool:
        dec = parse_top_frame(payload)
        return dec is not None and str(dec.command or "").upper() == "SLR"

    def _transmit_slr_read(
        self,
        frame: bytes,
        *,
        log: bool,
        retries: int = _LRF_SLR_QUERY_RETRIES,
    ) -> bytes | None:
        """Send SLR read; drain stray GAC/GAA datagrams on the shared UDP socket."""
        for attempt in range(max(1, int(retries))):
            try:
                self._transport.send_and_receive(
                    frame,
                    expect_reply=False,
                    log=log and attempt == 0,
                    timeout_s=0.08,
                )
            except Exception:
                pass
            reply = self._transport.receive_matching(
                self._payload_is_slr_reply,
                timeout_s=1.15 if attempt == 0 else 0.75,
                log=False,
            )
            if reply is not None:
                return reply
            if log:
                print(
                    f"[VGCS:lrf] SLR no reply (retry {attempt + 1}/{max(1, int(retries))})"
                )
            time.sleep(0.06)
        return None

    def _attitude_from_cache(self) -> tuple[float, float] | None:
        st = self.get_status_cached()
        if not st.supported:
            return None
        if st.yaw_deg is None and st.pitch_deg is None:
            return None
        return float(st.yaw_deg or 0.0), float(st.pitch_deg or 0.0)

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
        if len(tail) >= 5:
            return SkydroidTopUdpAdapter._slr_trimmed_median(tail, trim=1)
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
        align_ok: bool = False,
    ) -> float | None:
        """Accept a stable post-GOT SLR median (no stale-baseline gate)."""
        need_n = int(
            _LRF_CONVERGE_SAMPLES_ALIGNED if align_ok else _LRF_CONVERGE_SAMPLES
        )
        min_wait = 0.0 if align_ok else float(_LRF_LOCK_MIN_WAIT_S)
        if len(samples) < need_n:
            return None
        if float(elapsed) < min_wait:
            return None
        tail = [float(v) for v in samples[-need_n:]]
        converged = self._slr_converged(tail, need_n)
        if converged is None:
            return None
        if len(tail) >= 3 and self._slr_still_climbing(tail[-3:]):
            return None
        return float(converged)

    def _accept_align_ok_prelock_slr(
        self,
        pre_slr: float,
        click_offset_deg: float,
    ) -> float | None:
        """Verified boresight slew but C13 gave no post-aim SLR — keep pre-lock at short range."""
        pre = float(pre_slr)
        if pre > 10.0:
            return None
        if not self._slr_plausible_after_slew(pre, pre, float(click_offset_deg)):
            return None
        print(
            f"[VGCS:lrf] lock ok from pre-lock SLR {float(pre_slr):.1f} m "
            f"(align ok, post-slew SLR had no reply)"
        )
        return float(pre_slr)

    def _accept_align_phase_slr(
        self,
        align_samples: list[float],
        pre_slr: float | None,
        click_offset_deg: float,
    ) -> float | None:
        """Use stable SLR readings collected while slewing (post-poll often fails on C13)."""
        if len(align_samples) < 2:
            return None
        vals = [float(v) for v in align_samples]
        if pre_slr is not None:
            pre = max(0.5, float(pre_slr))
            near = [v for v in vals if v < pre * 0.55]
            tail = near[-6:] if len(near) >= 2 else vals[-4:]
        else:
            tail = vals[-4:]
        if len(tail) < 2:
            return None
        spread = max(tail) - min(tail)
        if spread > max(4.0, float(_LRF_STABLE_EPS_M) * 8.0):
            return None
        med = float(self._slr_median(tail))
        if not self._slr_plausible_after_slew(pre_slr, med, float(click_offset_deg)):
            return None
        print(
            f"[VGCS:lrf] lock ok from align-phase SLR median={med:.1f} m "
            f"(samples={tail}, post-slew poll had no reply)"
        )
        return med

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
        align_ok: bool = False,
    ) -> float | None:
        """Accept stable SLR after click-to-aim slew (range may match baseline on same facade)."""
        need_n = int(
            _LRF_CONVERGE_SAMPLES_ALIGNED if align_ok else _LRF_CONVERGE_SAMPLES
        )
        if gimbal_slew_mono is None or len(samples) < need_n:
            return None
        settle_s = (
            float(_LRF_GIMBAL_SLR_SETTLE_ALIGN_OK_S)
            if align_ok
            else float(_LRF_GIMBAL_SLR_SETTLE_S)
        )
        if settle_s > 0.0 and time.monotonic() - float(gimbal_slew_mono) < settle_s:
            return None
        if att_before is None or att_now is None:
            return None
        if yaw_tgt is not None and pitch_tgt is not None and not align_ok:
            pitch_open = (
                abs(float(dpitch)) * 0.95
                if not self._gac_pitch_trusted(float(att_now[1]), float(dpitch))
                else 0.0
            )
            if not self._align_aim_satisfied(
                att_now,
                float(yaw_tgt),
                float(pitch_tgt),
                float(dpitch),
                pitch_open_sent_deg=float(pitch_open),
            ):
                return None
        tail = [float(v) for v in samples[-need_n:]]
        converged = self._slr_converged(tail, need_n)
        if converged is None:
            return None
        if len(tail) >= 3 and self._slr_still_climbing(tail[-3:]):
            return None
        if align_ok:
            click_off = self._expected_offset_deg(float(dyaw), float(dpitch))
            if not self._slr_plausible_after_slew(
                baseline_m, float(converged), float(click_off)
            ):
                return None
            return float(converged)
        gimbal_move = self._gimbal_total_move_deg(att_before, att_now)
        if (
            baseline_m is not None
            and not self._slr_moved_from_baseline(converged, baseline_m)
            and gimbal_move < _LRF_GIMBAL_MOVE_MIN_DEG
        ):
            return None
        return float(converged)

    def _try_accept_lrf_lock_slr(
        self,
        samples: list[float],
        *,
        elapsed: float,
        pre_slr: float | None,
        align_attempted: bool,
        align_ok: bool = True,
        click_offset_deg: float,
        hold_gimbal: bool = False,
    ) -> float | None:
        """Stable SLR after lock; reject unchanged range when click is off laser boresight."""
        stable = self._try_accept_stable_slr(samples, elapsed=elapsed, align_ok=align_ok)
        if stable is None:
            return None
        off_centre = float(click_offset_deg) >= 3.0
        unchanged = (
            pre_slr is not None
            and abs(float(stable) - float(pre_slr)) < _LRF_MOVED_MIN_M
            and not self._slr_still_climbing(samples[-3:])
        )
        if align_attempted and align_ok:
            if not self._slr_plausible_after_slew(
                pre_slr, float(stable), float(click_offset_deg)
            ):
                return None
            return float(stable)
        if align_attempted and off_centre and unchanged:
            print(
                f"[VGCS:lrf] lock rejected — range {float(stable):.1f} m "
                f"unchanged from pre-lock {float(pre_slr):.1f} m "
                f"(gimbal slew did not reach target?)"
            )
            return None
        if hold_gimbal and off_centre and unchanged:
            print(
                f"[VGCS:lrf] lock rejected — range {float(stable):.1f} m unchanged "
                f"with click {float(click_offset_deg):.1f}° off laser aim — "
                f"aim gimbal at the new target first"
            )
            return None
        return float(stable)

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
        *,
        coarse_m: float | None = None,
    ) -> float | None:
        """Extra SLR polls after coarse converge — median filters jitter / late slew."""
        if coarse_m is not None:
            buf = [float(coarse_m)]
            for _ in range(2):
                time.sleep(max(0.08, float(_LRF_LOCK_POLL_S)))
                reading = self._query_slr_distance_m(log=False, fresh=True)
                if reading is None:
                    continue
                buf.append(float(reading))
            if len(buf) >= 3 and max(buf) - min(buf) <= _LRF_STABLE_EPS_M:
                refined = float(self._slr_median(buf))
                print(f"[VGCS:lrf] refine quick median={refined:.1f} m from {buf}")
                if on_sample is not None:
                    try:
                        on_sample(refined)
                    except Exception:
                        pass
                return refined

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
        refined = self._slr_trimmed_median(tail, trim=1) if len(tail) >= 5 else self._slr_median(tail)
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

    def _read_gimbal_attitude_deg(self, *, use_cache: bool = True) -> tuple[float, float] | None:
        """Blocking GAC read on the active probe endpoint (retries + recent cache fallback)."""
        self._ensure_active_transport()
        active_port = int(self._transport._port)
        try:
            for attempt in range(3):
                try:
                    reply = self._transport.send_and_receive(
                        build_gac_query(),
                        expect_reply=True,
                        log=False,
                        timeout_s=0.55 if attempt else 0.45,
                    )
                except Exception:
                    time.sleep(0.05)
                    continue
                if not reply:
                    time.sleep(0.04)
                    continue
                dec = parse_top_frame(reply)
                yaw, pitch = extract_attitude_deg(dec)
                if yaw is None and pitch is None:
                    time.sleep(0.04)
                    continue
                return float(yaw or 0.0), float(pitch or 0.0)
            if use_cache:
                st = self.get_status_cached()
                age = time.monotonic() - float(st.updated_mono or 0.0)
                if st.supported and age < 2.5:
                    cached = self._attitude_from_cache()
                    if cached is not None:
                        return cached
            return None
        finally:
            self._transport._port = active_port

    @staticmethod
    def _lrf_lock_move_gimbal() -> bool:
        """Default: slew gimbal to click on LRF lock. Set VGCS_LRF_HOLD_GIMBAL=1 for manual aim."""
        if str(os.environ.get("VGCS_LRF_HOLD_GIMBAL", "") or "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            return False
        if str(os.environ.get("VGCS_LRF_MOVE_GIMBAL", "") or "").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return False
        return True

    @staticmethod
    def _lrf_flip_x() -> bool:
        """Companion RTSP X is mirrored vs PROTOCAL GOT 1280×720 (field C13)."""
        if str(os.environ.get("VGCS_LRF_FLIP_X", "") or "").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return False
        if str(os.environ.get("VGCS_LRF_FLIP_X", "") or "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            return True
        return False

    @staticmethod
    def normalize_lrf_click_uv(u: float, v: float) -> tuple[float, float]:
        """Optional extra X flip via VGCS_LRF_FLIP_X (screen mirror is in map click norm)."""
        uf = max(0.0, min(1.0, float(u)))
        vf = max(0.0, min(1.0, float(v)))
        if SkydroidTopUdpAdapter._lrf_flip_x():
            uf = 1.0 - uf
        return float(uf), float(vf)

    @staticmethod
    def _hold_max_click_offset_deg() -> float:
        try:
            return float(
                os.environ.get(
                    "VGCS_LRF_HOLD_MAX_OFFSET_DEG",
                    str(_LRF_HOLD_MAX_CLICK_OFFSET_DEG),
                )
                or _LRF_HOLD_MAX_CLICK_OFFSET_DEG
            )
        except ValueError:
            return float(_LRF_HOLD_MAX_CLICK_OFFSET_DEG)

    def lock_lrf_target_at_norm(
        self,
        u: float,
        v: float,
        *,
        frame_w: int = _LRF_FRAME_W,
        frame_h: int = _LRF_FRAME_H,
        on_sample: Callable[[float], None] | None = None,
    ) -> float | None:
        """C13 LRF lock: slew gimbal to click (default), then confirm stable E-class SLR."""
        if not self.gimbal_telemetry_ok():
            return None
        move_gimbal = self._lrf_lock_move_gimbal()
        fw = _LRF_FRAME_W
        fh = _LRF_FRAME_H
        u_in, v_in = float(u), float(v)
        if self._lrf_flip_x():
            u, v = self.normalize_lrf_click_uv(u_in, v_in)
        else:
            u, v = u_in, v_in
        x_px = max(0, min(fw, int(round(max(0.0, min(1.0, float(u))) * fw))))
        y_px = max(0, min(fh, int(round(max(0.0, min(1.0, float(v))) * fh))))
        self._ensure_active_transport()
        active_port = int(self._active_port)
        dist_m: float | None = None
        samples: list[float] = []
        lock_started_mono = time.monotonic()

        def _emit_sample(value_m: float) -> None:
            if on_sample is None:
                return
            try:
                on_sample(float(value_m))
            except Exception:
                pass

        def _emit_slr_live(*, fresh: bool = False, log: bool = False) -> float | None:
            reading = self._query_slr_distance_m(log=log, fresh=fresh)
            if reading is not None:
                _emit_sample(float(reading))
            return reading

        try:
            with self._status_lock:
                prev_locked_m = (
                    float(self._lrf_locked_distance_m)
                    if self._lrf_locked and self._lrf_locked_distance_m is not None
                    else None
                )
                prev_lock_xy = (
                    (int(self._lrf_lock_x), int(self._lrf_lock_y))
                    if self._lrf_locked
                    else None
                )
                instant_slr = self._laser_range_m
                # DOOAF / observation may start a new pick while adapter still locked.
                self._lrf_locked = False
                self._lrf_locked_distance_m = None

            if instant_slr is not None:
                _emit_sample(float(instant_slr))

            att_before = self._read_gimbal_attitude_deg()
            dyaw, dpitch_img = self._pixel_boresight_offset_deg(x_px, y_px)
            click_offset_deg = self._expected_offset_deg(dyaw, dpitch_img)
            print(
                f"[VGCS:lrf] lock start px=({x_px},{y_px}) "
                f"att={att_before} offset=({dyaw:.1f}°,{dpitch_img:.1f}°) "
                f"mode={'click-to-aim' if move_gimbal else 'hold-gimbal'} "
                f"click=({u_in:.3f},{v_in:.3f}) frame=({u:.3f},{v:.3f})"
            )

            if not move_gimbal:
                max_off = self._hold_max_click_offset_deg()
                if float(click_offset_deg) > float(max_off):
                    print(
                        f"[VGCS:lrf] lock rejected — click {click_offset_deg:.1f}° "
                        f"off laser aim (max {max_off:.1f}°). "
                        f"Aim gimbal so the target is under the centre crosshair, then click."
                    )
                    return None

            pre_slr_available = self._query_slr_distance_m(log=False, fresh=False)
            if pre_slr_available is None:
                pre_slr_available = self._query_slr_distance_m(log=True, fresh=True)
            if pre_slr_available is None:
                if move_gimbal:
                    print(
                        "[VGCS:lrf] pre-lock SLR unavailable — slewing gimbal anyway "
                        "(range will be read after aim; DEM fallback if SLR stays offline)"
                    )
                else:
                    print(
                        "[VGCS:lrf] lock failed — cannot read SLR (check C13 TOP link)"
                    )
                    return None

            pre_slr = self._read_slr_prelock_median()
            if pre_slr is not None:
                print(f"[VGCS:lrf] pre-lock SLR={float(pre_slr):.1f} m (median)")
                _emit_sample(float(pre_slr))

            if (
                not move_gimbal
                and prev_locked_m is not None
                and pre_slr is not None
                and prev_lock_xy is not None
            ):
                dx = float(x_px - prev_lock_xy[0])
                dy = float(y_px - prev_lock_xy[1])
                moved_px = (dx * dx + dy * dy) ** 0.5
                if (
                    moved_px >= float(_LRF_HOLD_MIN_PIXEL_MOVE_PX)
                    and abs(float(pre_slr) - float(prev_locked_m)) < _LRF_MOVED_MIN_M
                ):
                    print(
                        f"[VGCS:lrf] lock rejected — range still {float(pre_slr):.1f} m "
                        f"at new click ({moved_px:.0f}px away); "
                        f"re-aim gimbal until live SLR shows the new target range"
                    )
                    return None

            self._send_got_mark_only(x_px, y_px, frame_w=fw, frame_h=fh)

            if (
                move_gimbal
                and float(click_offset_deg) < float(_LRF_FAST_LOCK_OFFSET_DEG)
            ):
                reading = pre_slr or _emit_slr_live(fresh=True, log=True)
                if reading is not None:
                    elapsed = time.monotonic() - lock_started_mono
                    print(
                        f"[VGCS:lrf] fast lock ok range={float(reading):.1f} m "
                        f"(offset={float(click_offset_deg):.1f}°, t={elapsed:.1f}s)"
                    )
                    dist_m = float(reading)
                    with self._status_lock:
                        self._lrf_locked = True
                        self._lrf_lock_x = x_px
                        self._lrf_lock_y = y_px
                        self._lrf_locked_distance_m = float(dist_m)
                        self._laser_range_m = float(dist_m)
                        self._laser_range_mono = time.monotonic()
                    return dist_m

            align_ok = True
            align_dyaw, align_dpitch = float(dyaw), float(dpitch_img)
            yaw_tgt: float | None = None
            pitch_tgt: float | None = None
            gimbal_slew_mono: float | None = None
            att_lock_ref = att_before
            got_sent = True
            align_slr_samples: list[float] = []

            if move_gimbal:
                if att_before is None:
                    print("[VGCS:lrf] lock failed — no GAC attitude for gimbal slew")
                    return None
                if float(click_offset_deg) >= _LRF_ALIGN_MIN_OFFSET_DEG:
                    print(
                        f"[VGCS:lrf] click-to-aim — slewing gimbal to px=({x_px},{y_px}) "
                        f"offset=({dyaw:.1f}°,{dpitch_img:.1f}°)"
                    )
                    gimbal_slew_mono = time.monotonic()

                    align_tick_n = 0

                    def _align_tick() -> None:
                        nonlocal align_tick_n
                        align_tick_n += 1
                        use_fresh = align_tick_n % 3 == 0 or not align_slr_samples
                        reading = self._query_slr_distance_m(
                            log=False, fresh=bool(use_fresh)
                        )
                        if reading is not None:
                            align_slr_samples.append(float(reading))
                            _emit_sample(float(reading))

                    att_after, align_ok, align_dyaw, align_dpitch = (
                        self._align_laser_boresight_to_pixel(
                            x_px,
                            y_px,
                            att_before,
                            on_tick=_align_tick,
                        )
                    )
                    att_lock_ref = att_after or att_before
                    yaw_tgt = self._gimbal_yaw_target_deg(
                        float(att_before[0]), float(align_dyaw)
                    )
                    pitch_tgt = self._gimbal_pitch_target_deg(
                        float(att_before[1]), float(align_dpitch)
                    )
                    if not align_ok:
                        print(
                            "[VGCS:lrf] lock failed — gimbal could not aim at click "
                            "(try a closer target or use gimbal sticks)"
                        )
                        return None
                    print(
                        f"[VGCS:lrf] slew done att={att_before}->{att_lock_ref} "
                        f"target=({yaw_tgt:.1f},{pitch_tgt:.1f})"
                    )
                else:
                    print("[VGCS:lrf] click near boresight — skipping slew")
                    gimbal_slew_mono = time.monotonic()
            else:
                print(
                    f"[VGCS:lrf] hold gimbal — lock SLR at current aim "
                    f"(offset=({dyaw:.1f}°,{dpitch_img:.1f}°), no camera move)"
                )
                if click_offset_deg >= 5.0:
                    print(
                        "[VGCS:lrf] hint: SLR measures along the laser, not the video click — "
                        "aim the building at frame centre with the gimbal first, then click to lock"
                    )
                gimbal_slew_mono = time.monotonic()

            if move_gimbal and align_ok and gimbal_slew_mono is not None:
                pitch_ch = abs(float(align_dpitch))
                settle_s = float(_LRF_POST_ALIGN_SLR_SETTLE_S) + (
                    0.35 if pitch_ch > 1.5 else 0.0
                )
                time.sleep(settle_s)
                self._gimbal_stop_hard()
                time.sleep(0.15)
                if not align_slr_samples:
                    for burst_i in range(5):
                        time.sleep(0.14)
                        reading = self._query_slr_distance_m(
                            log=burst_i == 0, fresh=True
                        )
                        if reading is None:
                            continue
                        align_slr_samples.append(float(reading))
                        samples.append(float(reading))
                        _emit_sample(float(reading))
                elif align_slr_samples:
                    samples.extend(float(v) for v in align_slr_samples[-4:])

            post_got_wait = 0.0

            att_latest = att_lock_ref
            start_mono = time.monotonic()
            deadline = start_mono + (
                _LRF_LOCK_MAX_WAIT_S + (2.0 if move_gimbal and gimbal_slew_mono else 0.0)
            )

            min_wait = (
                float(_LRF_POST_ALIGN_SLR_SETTLE_S)
                if move_gimbal and gimbal_slew_mono is not None and align_ok
                else (
                    _LRF_POST_MOVE_MIN_WAIT_S
                    if move_gimbal and gimbal_slew_mono is not None
                    else _LRF_LOCK_MIN_WAIT_S
                )
            )

            post_polls = 0
            while time.monotonic() < deadline:
                elapsed = time.monotonic() - start_mono
                time.sleep(_LRF_LOCK_POLL_S)
                post_polls += 1
                use_fresh = post_polls > 4
                reading = self._query_slr_distance_m(
                    log=len(samples) == 0 and post_polls <= 2,
                    fresh=bool(use_fresh),
                )
                if reading is None:
                    continue
                samples.append(float(reading))
                att_now = self._attitude_from_cache() or self._read_gimbal_attitude_deg()
                if att_now is not None:
                    att_latest = att_now
                if (
                    att_lock_ref is not None
                    and att_now is not None
                    and self._gimbal_total_move_deg(att_lock_ref, att_now)
                    > _LRF_LOCK_GIMBAL_DRIFT_MAX_DEG
                ):
                    print(
                        f"[VGCS:lrf] lock aborted — gimbal drift during lock "
                        f"{att_lock_ref}->{att_now}"
                    )
                    return None
                _emit_sample(float(reading))
                print(
                    f"[VGCS:lrf] SLR sample {reading:.1f} m "
                    f"(n={len(samples)}, t={elapsed:.1f}s)"
                )

                if elapsed < min_wait:
                    continue

                if move_gimbal and gimbal_slew_mono is not None:
                    stable = self._try_accept_gimbal_slew_slr(
                        samples,
                        pre_slr,
                        att_before,
                        att_latest,
                        gimbal_slew_mono=gimbal_slew_mono,
                        yaw_tgt=yaw_tgt,
                        pitch_tgt=pitch_tgt,
                        dyaw=align_dyaw,
                        dpitch=align_dpitch,
                        align_ok=bool(align_ok),
                    )
                else:
                    stable = self._try_accept_lrf_lock_slr(
                        samples,
                        elapsed=elapsed,
                        pre_slr=pre_slr,
                        align_attempted=move_gimbal,
                        align_ok=align_ok,
                        click_offset_deg=click_offset_deg,
                        hold_gimbal=not move_gimbal,
                    )
                if stable is not None:
                    print(
                        f"[VGCS:lrf] lock stable {stable:.1f} m "
                        f"(att={att_before}->{att_latest})"
                    )
                    if align_ok:
                        dist_m = float(stable)
                    else:
                        dist_m = self._refine_slr_estimate(
                            on_sample, coarse_m=float(stable)
                        ) or float(stable)
                    _emit_sample(float(dist_m))
                    break

            if dist_m is None and samples:
                if move_gimbal and gimbal_slew_mono is not None:
                    stable = self._try_accept_gimbal_slew_slr(
                        samples,
                        pre_slr,
                        att_before,
                        att_latest,
                        gimbal_slew_mono=gimbal_slew_mono,
                        yaw_tgt=yaw_tgt,
                        pitch_tgt=pitch_tgt,
                        dyaw=align_dyaw,
                        dpitch=align_dpitch,
                        align_ok=bool(align_ok),
                    )
                else:
                    stable = self._try_accept_lrf_lock_slr(
                        samples,
                        elapsed=time.monotonic() - start_mono,
                        pre_slr=pre_slr,
                        align_attempted=move_gimbal,
                        align_ok=align_ok,
                        click_offset_deg=click_offset_deg,
                        hold_gimbal=not move_gimbal,
                    )
                if stable is not None:
                    if align_ok:
                        dist_m = float(stable)
                    else:
                        dist_m = self._refine_slr_estimate(
                            on_sample, coarse_m=float(stable)
                        ) or float(stable)
                else:
                    print(
                        f"[VGCS:lrf] lock rejected — SLR did not stabilize "
                        f"(samples={samples[-5:]})"
                    )

            if dist_m is None and align_ok and align_slr_samples:
                fallback = self._accept_align_phase_slr(
                    align_slr_samples,
                    pre_slr,
                    click_offset_deg,
                )
                if fallback is not None:
                    dist_m = float(fallback)
                    _emit_sample(dist_m)

            if (
                dist_m is None
                and align_ok
                and pre_slr is not None
                and not samples
            ):
                fallback = self._accept_align_ok_prelock_slr(
                    float(pre_slr),
                    click_offset_deg,
                )
                if fallback is not None:
                    dist_m = float(fallback)
                    _emit_sample(dist_m)

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
                elapsed = time.monotonic() - lock_started_mono
                print(
                    f"[VGCS:lrf] lock ok range={dist_m:.1f} m "
                    f"(samples={len(samples)}, t={elapsed:.1f}s)"
                )
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
        """SLR read: E-class laser module first, D-class fallback (PROTOCAL §2.2.2 / §4.22)."""
        self._ensure_active_transport()
        active_port = int(self._transport._port)
        laser_q = build_slr_query(dest=_LRF_SLR_DEST_LASER)
        system_q = build_slr_query(dest=_LRF_SLR_DEST_SYSTEM)
        trigger = bool(fresh and self._slr_use_trigger())
        dist_m: float | None = None
        laser_m: float | None = None
        system_m: float | None = None
        laser_reply: bytes | None = None
        system_reply: bytes | None = None
        try:
            for port in self._slr_probe_ports():
                self._transport._port = int(port)
                if trigger:
                    self._fire_slr_trigger(dest=_LRF_SLR_DEST_LASER)
                    self._transmit_slr_read(laser_q, log=False)
                    time.sleep(_LRF_SLR_SHOT_SETTLE_S)
                laser_reply = self._transmit_slr_read(laser_q, log=log)
                system_reply = self._transmit_slr_read(system_q, log=False)
                laser_m = (
                    parse_slr_distance_from_payload(laser_reply)
                    if laser_reply is not None
                    else None
                )
                system_m = (
                    parse_slr_distance_from_payload(system_reply)
                    if system_reply is not None
                    else None
                )
                dist_m = self._pick_slr_readings(laser_m, system_m, log=log)
                if dist_m is not None:
                    if log and int(port) != int(active_port):
                        print(
                            f"[VGCS:lrf] SLR ok on port {int(port)} "
                            f"(active gimbal port {int(active_port)})"
                        )
                    break
            if dist_m is None:
                if log:
                    print("[VGCS:lrf] SLR read failed — no valid E/D reply")
                return None
            dist_m = self._calibrate_slr_m(float(dist_m))
            if log:
                hx = slr_raw_hex(laser_reply or system_reply or b"")
                print(
                    f"[VGCS:lrf] SLR reply hex={hx} "
                    f"range={float(dist_m):.1f} m "
                    f"(E={laser_m}, D={system_m})"
                )
            if not self._lrf_locked:
                with self._status_lock:
                    self._laser_range_m = float(dist_m)
                    self._laser_range_mono = time.monotonic()
            return float(dist_m)
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
            self._lrf_armed = False
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
        action_l = str(action or "").strip().lower()
        if action_l == "stop":
            self.ptz_stop_burst(count=3)
            return
        commands = self._profile.ptz_commands.get(action_l, [])
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

    def camera_zoom_step(self, direction: int) -> None:
        self._enqueue(["C13_ZOOM_STEP"], {"direction": int(direction)}, False)

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

    @staticmethod
    def _is_motion_stop_command(commands: list[str], params: dict[str, object]) -> bool:
        """PT_STOP / GSM(0,0) must not be dropped when coalescing motion queue."""
        if not commands:
            return False
        c0 = str(commands[0]).upper()
        if c0 in ("PT_STOP", "PTZ_STOP"):
            return True
        if c0 == "PTZ" and str(params.get("action", "")).strip().lower() == "stop":
            return True
        if c0 == "GSM":
            try:
                y = float(params.get("yaw", 0) or 0)
                p = float(params.get("pitch", 0) or 0)
            except (TypeError, ValueError):
                return False
            return abs(y) < 1e-6 and abs(p) < 1e-6
        return False

    def _drop_pending_motion_commands(self) -> None:
        """Coalesce motion queue — keep the latest stop so release→press races still halt."""
        pending: list[tuple[list[str], dict[str, object], bool]] = []
        latest_stop: tuple[list[str], dict[str, object], bool] | None = None
        while True:
            try:
                pending.append(self._queue.get_nowait())
            except queue.Empty:
                break
        for item in pending:
            cmds, params, expect_reply = item
            if self._is_motion_stop_command(cmds, params):
                latest_stop = item
                continue
            if self._is_motion_command(cmds):
                continue
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass
        if latest_stop is not None:
            try:
                self._queue.put_nowait(latest_stop)
            except queue.Full:
                pass

    def ptz_stop_burst(self, *, count: int = 3) -> None:
        """C13 can coast after a single PT_STOP — send a short burst on release."""
        commands = self._profile.ptz_commands.get("stop", [])
        if not commands:
            return
        n = max(1, min(6, int(count)))
        for i in range(n):
            self._enqueue(
                list(commands),
                {},
                False,
                coalesce_motion=(i == 0),
            )

    def _enqueue(
        self,
        commands: list[str],
        params: dict[str, object],
        expect_reply: bool,
        *,
        coalesce_motion: bool = True,
    ) -> None:
        if not commands:
            return
        if not self._running:
            self.start()
        if coalesce_motion and not expect_reply and self._is_motion_command(commands):
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
            if commands and commands[0] == "C13_ZOOM_STEP":
                try:
                    self._send_zoom_step(int(params.get("direction", 0) or 0))
                except Exception:
                    pass
                continue
            self._ensure_active_transport()
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
        """C13 absolute zoom: DZM + MUL on active host and alternate UDP ports."""
        self._ensure_active_transport()
        frames = build_zoom_command_burst(level)
        if not frames:
            return
        try:
            print(
                f"[VGCS:skydroid] zoom TOP abs level={float(level):.1f}× "
                f"frames={len(frames)} host={self._transport._host}:{self._transport._port}"
            )
        except Exception:
            pass
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

    def _send_zoom_step(self, direction: int) -> None:
        """C13 rail +/- : ZMC lens step + DZM §4.7 Zoom+/Zoom-."""
        if int(direction) == 0:
            return
        self._ensure_active_transport()
        frames = build_c13_zoom_step_frames(int(direction))
        if not frames:
            return
        sign = "+" if int(direction) > 0 else "-"
        try:
            print(
                f"[VGCS:skydroid] zoom TOP step dir={sign}1 "
                f"frames={len(frames)} host={self._transport._host}:{self._transport._port}"
            )
        except Exception:
            pass
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
        dist_m = self._calibrate_slr_m(float(dist_m))
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
