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
    build_gimbal_angle_axis,
    build_gimbal_speed,
    build_got_target,
    build_slr_query,
    build_sum_track,
    build_top_frame,
    build_zoom_command_burst,
    decode_slr_distance_m,
    extract_attitude_deg,
    parse_slr_distance_from_payload,
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
_LRF_POST_GOT_WAIT_S = 2.0
_LRF_GIMBAL_MOVE_MIN_DEG = 0.35
_LRF_FOV_H_DEG = 83.4
_LRF_FOV_V_DEG = 46.9
_LRF_NUDGE_AFTER_S = 3.0
_LRF_MAX_NUDGES = 2
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

    def _send_gimbal_point_at_pixel(
        self,
        x_px: int,
        y_px: int,
        att_now: tuple[float, float] | None,
    ) -> None:
        """Fallback when GOT/SUM does not slew — point gimbal at click using FOV math."""
        if att_now is None:
            att_now = self._read_gimbal_attitude_deg()
        if att_now is None:
            print("[VGCS:lrf] gimbal nudge skipped — no GAC attitude")
            return
        dyaw, dpitch_img = self._pixel_boresight_offset_deg(x_px, y_px)
        yaw_tgt = float(att_now[0]) + dyaw
        pitch_tgt = float(att_now[1]) - dpitch_img
        pitch_tgt = max(-90.0, min(90.0, pitch_tgt))
        gay = build_gimbal_angle_axis("GAY", yaw_tgt, speed=25.0)
        gap = build_gimbal_angle_axis("GAP", pitch_tgt, speed=25.0)
        for frame in (gay, gap):
            self._transport.send_and_receive(
                frame, expect_reply=False, log=True, timeout_s=0.08
            )
        slew_s = min(3.5, max(1.0, (abs(dyaw) + abs(dpitch_img)) / 12.0))
        print(
            f"[VGCS:lrf] gimbal nudge px=({x_px},{y_px}) "
            f"-> yaw={yaw_tgt:.2f} pitch={pitch_tgt:.2f} wait={slew_s:.1f}s"
        )
        time.sleep(slew_s)
        stop = build_gimbal_speed(0.0, 0.0)
        self._transport.send_and_receive(
            stop, expect_reply=False, log=True, timeout_s=0.08
        )

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
            reading = self._query_slr_distance_m(log=False)
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

    def _read_slr_baseline_m(self) -> float | None:
        """Median of several pre-lock SLR reads — required before accepting any lock."""
        readings: list[float] = []
        for _ in range(_LRF_BASELINE_RETRIES):
            reading = self._query_slr_distance_m(log=False)
            if reading is not None:
                readings.append(float(reading))
            time.sleep(_LRF_BASELINE_GAP_S)
        if not readings:
            return None
        tail = readings[-min(5, len(readings)) :]
        return float(self._slr_median(tail))

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
        """GOT + SUM track confirm at video pixel, then read SLR after tracker settles."""
        if not self.gimbal_telemetry_ok():
            return None
        # PROTOCAL §3.3.5: GOT coordinates are always on 1280×720 companion video.
        fw = _LRF_FRAME_W
        fh = _LRF_FRAME_H
        x_px = max(0, min(fw, int(round(max(0.0, min(1.0, float(u))) * fw))))
        y_px = max(0, min(fh, int(round(max(0.0, min(1.0, float(v))) * fh))))
        active_port = int(self._transport._port)
        dist_m: float | None = None
        try:
            with self._status_lock:
                self._laser_range_m = None
                self._laser_range_mono = 0.0
                self._lrf_locked_distance_m = None

            baseline_m = self._read_slr_baseline_m()
            att_before = self._read_gimbal_attitude_deg()
            print(
                f"[VGCS:lrf] lock start px=({x_px},{y_px}) "
                f"baseline={baseline_m} att={att_before}"
            )
            if baseline_m is None:
                print(
                    "[VGCS:lrf] lock failed — cannot read SLR baseline (check C13 TOP link)"
                )
                return None

            # Reset any RC/previous track before selecting a new target.
            stop = build_sum_track(confirm=False)
            self._transport.send_and_receive(
                stop, expect_reply=False, log=True, timeout_s=0.08
            )
            time.sleep(0.15)
            self._send_got_track(x_px, y_px, frame_w=fw, frame_h=fh)
            time.sleep(_LRF_POST_GOT_WAIT_S)

            samples: list[float] = []
            start_mono = time.monotonic()
            deadline = start_mono + _LRF_LOCK_MAX_WAIT_S
            next_resend = _LRF_GOT_RESEND_INTERVAL_S
            move_mono: float | None = None
            nudge_count = 0

            def _emit_sample(value_m: float) -> None:
                if on_sample is None or baseline_m is None:
                    return
                if abs(float(value_m) - float(baseline_m)) < _LRF_MOVED_MIN_M:
                    return
                try:
                    on_sample(float(value_m))
                except Exception:
                    pass

            while time.monotonic() < deadline:
                elapsed = time.monotonic() - start_mono
                if elapsed >= next_resend and next_resend <= _LRF_LOCK_MAX_WAIT_S - 1.0:
                    print(f"[VGCS:lrf] re-send GOT+SUM @ {next_resend:.1f}s")
                    self._send_got_track(x_px, y_px, frame_w=fw, frame_h=fh)
                    time.sleep(0.75)
                    next_resend += _LRF_GOT_RESEND_INTERVAL_S

                time.sleep(_LRF_LOCK_POLL_S)
                reading = self._query_slr_distance_m(log=False)
                if reading is None:
                    continue
                samples.append(float(reading))
                _emit_sample(float(reading))
                print(f"[VGCS:lrf] SLR sample {reading:.1f} m (n={len(samples)}, t={elapsed:.1f}s)")

                if elapsed < _LRF_LOCK_MIN_WAIT_S:
                    continue

                att_now = self._read_gimbal_attitude_deg()
                gimbal_moved = self._gimbal_attitude_moved(att_before, att_now)
                slr_moved = self._slr_samples_moved_from_baseline(samples, baseline_m)

                if (
                    not slr_moved
                    and gimbal_moved is not True
                    and nudge_count < _LRF_MAX_NUDGES
                    and elapsed >= _LRF_NUDGE_AFTER_S * float(nudge_count + 1)
                ):
                    self._send_gimbal_point_at_pixel(x_px, y_px, att_now)
                    self._send_got_track(x_px, y_px, frame_w=fw, frame_h=fh)
                    nudge_count += 1
                    move_mono = None
                    continue

                if gimbal_moved is False and not slr_moved:
                    continue

                if not slr_moved:
                    if gimbal_moved is True:
                        print(
                            f"[VGCS:lrf] gimbal slewed but SLR still {reading:.1f} m "
                            f"(baseline={baseline_m:.1f} m) — waiting for laser…"
                        )
                    continue

                post = self._slr_post_move_samples(samples, baseline_m)
                if not post:
                    continue
                if move_mono is None:
                    move_mono = time.monotonic()
                elapsed_since_move = time.monotonic() - float(move_mono)

                converged = self._slr_converged(post, _LRF_CONVERGE_SAMPLES)
                if converged is None:
                    continue
                if self._slr_still_settling(post, elapsed_since_move):
                    continue
                if not self._slr_moved_from_baseline(converged, baseline_m):
                    continue
                if gimbal_moved is False:
                    continue

                print(
                    f"[VGCS:lrf] tracker slew ok {converged:.1f} m "
                    f"(baseline={baseline_m}, att={att_before}->{att_now}, "
                    f"t={elapsed:.1f}s, since_move={elapsed_since_move:.1f}s)"
                )
                dist_m = self._refine_slr_estimate(on_sample)
                if dist_m is None:
                    dist_m = float(converged)
                break

            if dist_m is None and samples:
                post = self._slr_post_move_samples(samples, baseline_m)
                att_now = self._read_gimbal_attitude_deg()
                gimbal_moved = self._gimbal_attitude_moved(att_before, att_now)
                if post and move_mono is not None and gimbal_moved is not False:
                    converged = self._slr_converged(
                        post, min(_LRF_CONVERGE_SAMPLES, len(post))
                    )
                    if (
                        converged is not None
                        and self._slr_moved_from_baseline(converged, baseline_m)
                        and not self._slr_still_climbing(post[-min(4, len(post)):])
                    ):
                        dist_m = self._refine_slr_estimate(on_sample) or float(converged)
                if dist_m is None:
                    if not self._slr_samples_moved_from_baseline(samples, baseline_m):
                        print(
                            f"[VGCS:lrf] lock rejected — SLR stayed at baseline "
                            f"{baseline_m:.1f} m (GOT+SUM did not slew laser to target)"
                        )
                    elif gimbal_moved is False:
                        print(
                            f"[VGCS:lrf] lock rejected — gimbal did not move "
                            f"(att={att_before}->{att_now}); check C13 tracking support"
                        )
                    else:
                        print(
                            f"[VGCS:lrf] lock rejected — range moved but did not settle "
                            f"(baseline={baseline_m}, last={samples[-1]:.1f} m)"
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
                print("[VGCS:lrf] lock failed — no SLR samples")
            return dist_m
        except Exception as exc:
            print(f"[VGCS:lrf] lock exception: {exc}")
            with self._status_lock:
                self._lrf_locked = False
            return None
        finally:
            self._transport._port = active_port

    def _query_slr_distance_m(self, *, log: bool = False) -> float | None:
        """Single SLR read — returns distance parsed from this reply only (no stale cache)."""
        active_port = int(self._transport._port)
        try:
            frame = build_slr_query()
            reply = self._transport.send_and_receive(
                frame,
                expect_reply=True,
                log=log,
                timeout_s=0.5,
            )
            if not reply:
                return None
            dist_m = parse_slr_distance_from_payload(reply)
            if dist_m is not None and not self._lrf_locked:
                with self._status_lock:
                    self._laser_range_m = float(dist_m)
                    self._laser_range_mono = time.monotonic()
            return dist_m
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
