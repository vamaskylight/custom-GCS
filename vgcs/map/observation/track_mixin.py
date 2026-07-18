"""M13 — Moving target tracking (C13 GOT + SUM, GCS-computed map coordinates)."""

from __future__ import annotations

import os

from PySide6.QtCore import QThreadPool, QTimer

from vgcs.map.native_video_overlay import VideoOverlayM13Track
from vgcs.map.observation.types import (
    M13RangeTask,
    M13TrackBridge,
    M13TrackStartTask,
    M14FollowTask,
)
from vgcs.map.video.frame_convert import qimage_to_bgr_array
from vgcs.observe.geo_reference import compute_lrf_slant_geo
from vgcs.observe.visual_object_tracker import VisualObjectTracker, bbox_around_point
from vgcs.video.camera_control import uses_skydroid_top_camera
from vgcs.video.pipeline import (
    notify_companion_preview_motion,
    notify_companion_visual_track,
)

# Cameras where GOT/SUM has no confirmed continuous-follow capability (or,
# for C12, is simply not yet wired through — see DOCS/SKYDROID-TOP-PROTOCOL.md),
# so M13 tracks the target itself in software instead of trusting firmware.
_M14_AI_FOLLOW_PROFILE_IDS = {"c12_default"}
_M14_TRACK_BOX_SIZE_PX = 60
# M14 ticks faster than the legacy GOT+SUM path (100ms vs 200ms): CSRT's own
# search window is what actually limits how far a target can move between
# updates before tracking is lost — field-observed losing track on a sudden,
# sharp reversal — so sampling twice as often halves that worst-case jump
# distance. Measured round trip to the isolated tracker process is ~3.6ms
# (see visual_object_tracker.py), far under either interval's budget.
_M13_TRACK_INTERVAL_MS = 200
_M14_TRACK_INTERVAL_MS = 100
_M14_LOST_STREAK_STOP = 30  # ~3s at the 100ms M14 tick rate


_M13_PATH_MAX_POINTS = 400
_M13_GEO_MIN_INTERVAL_S = 1.0
_M13_SLR_FRESH_INTERVAL_S = 3.0


def _m13_slr_fresh_interval_s() -> float:
    """Cadence for re-firing the SLR laser during a track — firing the laser is a
    real hardware event (unlike a data-only command), and is one candidate for a
    field-reported ~0.1-0.2s video stutter during tracking. Override for A/B
    testing without a rebuild: set 0 to disable the periodic re-fire entirely
    (range then freezes at the start-time value again, trading accuracy for a
    clean signal on whether the laser trigger is the stutter's cause)."""
    try:
        return float(
            os.environ.get("VGCS_M13_SLR_FRESH_INTERVAL_S", "")
            or _M13_SLR_FRESH_INTERVAL_S
        )
    except ValueError:
        return float(_M13_SLR_FRESH_INTERVAL_S)
_M13_FOLLOW_WARN_AFTER_S = 6.0
_M13_FOLLOW_MOVE_DEG = 0.8
_M13_SUM_KEEPALIVE_TICKS = 10  # 200ms timer → 2s
_M13_VIDEO_STALL_S = 2.5
_M13_VIDEO_NUDGE_MIN_S = 8.0
# The C13 firmware reports no track-box, so the geo assumes the target sits on
# boresight. When the gimbal has slewed more than this (deg) since the range was
# measured, the range and the current look direction disagree — the target is
# likely off-centre and the plotted position is only approximate. We cannot
# correct it (no tracked-pixel feedback), so we flag it instead.
_M13_SLEW_UNCERTAIN_DEG = 1.0


class M13MovingTargetTrackMixin:
    """Click-to-track on C13 day video; map coords from GPS + gimbal + SLR."""

    def _m13_track_supported(self) -> bool:
        return uses_skydroid_top_camera(getattr(self, "_camera_control", None))

    def _m14_active_profile(self):
        cc = getattr(self, "_camera_control", None)
        fn = getattr(cc, "active_camera_profile", None)
        if not callable(fn):
            return None
        try:
            return fn()
        except Exception:
            return None

    def _m14_should_use_ai_follow(self) -> bool:
        """True when the connected camera has no confirmed continuous-follow
        capability of its own (see DOCS/SKYDROID-TOP-PROTOCOL.md) — M13 then
        tracks the target in software (vgcs/observe/visual_object_tracker.py)
        and drives the gimbal itself instead of sending GOT+SUM and hoping."""
        profile = self._m14_active_profile()
        pid = str(getattr(profile, "profile_id", "") or "")
        return pid in _M14_AI_FOLLOW_PROFILE_IDS

    def _m14_ai_follow_active(self) -> bool:
        return bool(getattr(self, "_m14_tracker_active", False))

    def _m13_track_mode_active(self) -> bool:
        return bool(getattr(self, "_m13_track_armed", False))

    def _m13_track_is_active(self) -> bool:
        return bool(getattr(self, "_m13_track_active", False))

    def _set_m13_track_armed(self, armed: bool) -> None:
        on = bool(armed)
        if on and not self._m13_track_supported():
            self._set_status("M13 track needs Skydroid C13 camera control")
            btn = getattr(self, "_btn_native_m13_track", None)
            if btn is not None:
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
            return
        if on and bool(getattr(self, "_lrf_lock_armed", False)):
            self._cancel_c13_lrf_arm()
        if not on and self._m13_track_is_active():
            self._stop_m13_track()
        self._m13_track_armed = on
        btn = getattr(self, "_btn_native_m13_track", None)
        if btn is not None:
            btn.blockSignals(True)
            btn.setChecked(on or self._m13_track_is_active())
            btn.blockSignals(False)
        self._refresh_m13_track_overlay()
        if on and not self._m13_track_is_active():
            notify_companion_preview_motion(duration_s=120.0)
            self._set_status("M13 track — click target on video to lock and follow")
            QTimer.singleShot(0, self._raise_flight_hud_above_video)
        elif not on and not self._m13_track_is_active():
            self._set_status("M13 track off")

    def _sync_m13_track_button(self) -> None:
        btn = getattr(self, "_btn_native_m13_track", None)
        if btn is None:
            return
        want = self._m13_track_mode_active() or self._m13_track_is_active()
        if btn.isChecked() != want:
            btn.blockSignals(True)
            btn.setChecked(want)
            btn.blockSignals(False)

    def _begin_m13_video_track(self, u: float, v: float) -> None:
        if not self._m13_track_supported():
            self._set_status("M13 track unavailable — connect Skydroid C13")
            return
        if bool(getattr(self, "_lrf_lock_in_progress", False)):
            self._set_status("Wait for LRF lock to finish before starting track")
            return
        if bool(getattr(self, "_m13_track_starting", False)):
            self._set_status("M13 track — wait for current lock to finish")
            return
        cc = getattr(self, "_camera_control", None)
        busy_fn = getattr(cc, "is_m13_track_start_in_progress", None)
        if callable(busy_fn) and bool(busy_fn()):
            self._set_status("M13 track — wait for current lock to finish")
            return
        if self._m13_track_is_active():
            self._stop_m13_track()
        if self._c13_lrf_is_locked():
            try:
                self._unlock_c13_lrf()
            except Exception:
                pass
        self._m13_track_click_uv = (float(u), float(v))
        att = self._read_gimbal_attitude_pair()
        self._m13_track_ref_att = att
        self._m13_track_armed = True
        self._m13_track_starting = True
        self._m13_track_generation = int(getattr(self, "_m13_track_generation", 0) or 0) + 1
        gen = int(self._m13_track_generation)
        self._m13_track_inflight_gen = gen
        self._sync_m13_track_button()
        self._refresh_m13_track_overlay(pending=True)
        notify_companion_visual_track(active=True)
        notify_companion_preview_motion(duration_s=300.0)
        if self._m14_should_use_ai_follow():
            self._m14_start_ai_follow(float(u), float(v), gen)
            return
        self._set_status("M13 track — locking target on C13…")
        cc = getattr(self, "_camera_control", None)
        bridge = getattr(self, "_m13_track_bridge", None)
        if cc is None or bridge is None:
            self._m13_track_starting = False
            notify_companion_visual_track(active=False)
            self._set_status("M13 track failed — no camera control")
            return
        task = M13TrackStartTask(cc, float(u), float(v), bridge, generation=gen)
        QThreadPool.globalInstance().start(task)

    def _m14_start_ai_follow(self, u: float, v: float, generation: int) -> None:
        """M14 — start software tracking instead of firmware GOT+SUM.

        Runs synchronously on the GUI thread: grabbing the current preview
        frame and initializing CSRT on a small crop is fast (single-digit ms),
        unlike the GOT+SUM path's UDP round trip, so this doesn't need a
        worker thread the way M13TrackStartTask does.
        """
        self._m13_track_starting = False
        profile = self._m14_active_profile()
        frame_w = int(getattr(profile, "frame_w", 1280) or 1280)
        frame_h = int(getattr(profile, "frame_h", 720) or 720)
        img = self._preview_image_copy_for_snapshot()
        frame_bgr = qimage_to_bgr_array(img) if img is not None else None
        if frame_bgr is None:
            print("[VGCS:m14] track start failed: no preview frame available to track from")
            self._m13_track_active = False
            notify_companion_visual_track(active=False)
            self._refresh_m13_track_overlay(failed=True)
            self._set_status("M13 track failed — no video frame to track from")
            self._sync_m13_track_button()
            return
        # frame_bgr's actual shape may differ slightly from the profile's
        # nominal frame_w/h (decode scaling) — use the real array shape.
        fh, fw = frame_bgr.shape[0], frame_bgr.shape[1]
        cx, cy = float(u) * fw, float(v) * fh
        bbox = bbox_around_point(
            cx, cy, box_w=_M14_TRACK_BOX_SIZE_PX, box_h=_M14_TRACK_BOX_SIZE_PX,
            frame_w=fw, frame_h=fh,
        )
        print(
            f"[VGCS:m14] track start click=({u:.3f},{v:.3f}) frame={fw}x{fh} "
            f"bbox=({bbox.x:.0f},{bbox.y:.0f},{bbox.w:.0f},{bbox.h:.0f})"
        )
        tracker = VisualObjectTracker()
        ok = tracker.start(frame_bgr, bbox)
        if not ok:
            print("[VGCS:m14] track start failed: tracker.start() returned False (see init error above)")
            self._m13_track_active = False
            notify_companion_visual_track(active=False)
            self._refresh_m13_track_overlay(failed=True)
            self._set_status("M13 track failed — could not initialize tracker on that spot")
            self._sync_m13_track_button()
            return
        print("[VGCS:m14] track armed — CSRT tracker initialized, gimbal follow starting")
        import time

        self._m14_tracker = tracker
        self._m14_tracker_active = True
        self._m14_tracker_frame_wh = (fw, fh)
        self._m14_follow_task_inflight = False
        self._m14_follow_lost_streak = 0
        self._m14_follow_ticks_skipped = 0
        self._m13_track_active = True
        self._m13_track_armed = True
        self._m13_track_click_uv = (float(u), float(v))
        self._m13_track_path = []
        self._m13_track_lat = None
        self._m13_track_lon = None
        self._m13_track_alt_m = None
        self._m13_track_geo_label = ""
        self._m13_track_range_m = None
        self._m13_track_start_mono = time.monotonic()
        self._m13_track_slr_fresh_mono = 0.0
        self._sync_m13_track_button()
        self._refresh_m13_track_overlay()
        QTimer.singleShot(0, lambda: self._update_m13_track_geo(force=True))
        self._sync_m13_track_timer()
        self._set_status("M13 tracking (software AI follow) — gimbal follow active")

    def _on_m13_track_started(self, ok: bool, u: float, v: float, generation: int = 0) -> None:
        gen = int(generation or 0)
        inflight = int(getattr(self, "_m13_track_inflight_gen", 0) or 0)
        current = int(getattr(self, "_m13_track_generation", 0) or 0)
        if gen == inflight:
            self._m13_track_starting = False
        if gen != current:
            print(
                f"[VGCS:m13] ignoring stale track result gen={gen} "
                f"current={current} inflight={inflight}"
            )
            return
        if not ok:
            self._m13_track_active = False
            notify_companion_visual_track(active=False)
            self._refresh_m13_track_overlay(failed=True)
            self._set_status(
                "M13 track failed — could not lock target on C13 "
                "(check gimbal telemetry and C13 link)"
            )
            self._sync_m13_track_button()
            return
        import time

        self._m13_track_active = True
        self._m13_track_armed = True
        self._m13_track_click_uv = (float(u), float(v))
        self._m13_track_path = []
        self._m13_track_lat = None
        self._m13_track_lon = None
        self._m13_track_alt_m = None
        self._m13_track_geo_label = ""
        self._m13_track_range_m = None
        self._m13_track_lock_att = self._read_gimbal_attitude_pair()
        self._m13_track_start_mono = time.monotonic()
        self._m13_track_follow_seen = False
        self._m13_track_follow_warned = False
        self._m13_track_slr_fresh_mono = 0.0
        self._m13_track_keepalive_ticks = 0
        self._sync_m13_track_button()
        self._refresh_m13_track_overlay()
        QTimer.singleShot(0, lambda: self._update_m13_track_geo(force=True))
        self._sync_m13_track_timer()
        self._set_status("M13 tracking — gimbal follow active")

    def _m13_invalidate_inflight_start(self) -> None:
        """Bump the track generation so a still-running M13TrackStartTask cannot
        re-activate the track after it was stopped/reset.

        ``_on_m13_track_started`` discards any callback whose generation != the
        current generation. Only ``_begin_m13_video_track`` bumps the generation,
        so without this a late worker success (which re-arms the C13 firmware)
        would flip ``_m13_track_active`` back on after the operator turned it off.
        """
        self._m13_track_generation = int(getattr(self, "_m13_track_generation", 0) or 0) + 1
        self._m13_track_starting = False

    def _m14_stop_ai_follow(self) -> None:
        """Stop the software tracker and command a zero-speed gimbal stop —
        without this, the last non-zero follow speed keeps the gimbal
        slewing after the track ends."""
        was_active = self._m14_ai_follow_active()
        tracker = getattr(self, "_m14_tracker", None)
        if tracker is not None:
            tracker.stop()
        self._m14_tracker = None
        self._m14_tracker_active = False
        self._m14_follow_task_inflight = False
        self._m14_follow_lost_streak = 0
        self._m14_follow_ticks_skipped = 0
        if was_active:
            cc = getattr(self, "_camera_control", None)
            set_speed = getattr(cc, "set_gimbal_speed", None)
            if callable(set_speed):
                try:
                    set_speed(0.0, 0.0)
                except Exception:
                    pass

    def _stop_m13_track(self) -> None:
        was = self._m13_track_is_active() or self._m13_track_mode_active()
        self._m13_invalidate_inflight_start()
        self._m14_stop_ai_follow()
        self._m13_track_active = False
        self._m13_track_armed = False
        notify_companion_visual_track(active=False)
        cc = getattr(self, "_camera_control", None)
        stop_fn = getattr(cc, "stop_target_track", None)
        if callable(stop_fn):
            try:
                stop_fn()
            except Exception:
                pass
        self._sync_m13_track_timer()
        self._clear_m13_track_map()
        self._refresh_m13_track_overlay()
        self._sync_m13_track_button()
        if was:
            self._set_status("M13 track stopped")

    def _reset_m13_track_for_disconnect(self) -> None:
        self._m13_invalidate_inflight_start()
        self._m14_stop_ai_follow()
        self._m13_track_active = False
        self._m13_track_armed = False
        notify_companion_visual_track(active=False)
        self._m13_track_click_uv = None
        self._m13_track_ref_att = None
        self._m13_track_lat = None
        self._m13_track_lon = None
        self._m13_track_alt_m = None
        self._m13_track_geo_label = ""
        self._m13_track_range_m = None
        self._m13_track_path = []
        t = getattr(self, "_m13_track_timer", None)
        if t is not None:
            t.stop()
        self._clear_m13_track_map()
        self._refresh_m13_track_overlay()
        self._sync_m13_track_button()

    def _sync_m13_track_timer(self) -> None:
        if not self._m13_track_is_active():
            t = getattr(self, "_m13_track_timer", None)
            if t is not None:
                t.stop()
            return
        t = getattr(self, "_m13_track_timer", None)
        if t is None:
            t = QTimer(self)
            t.timeout.connect(self._m13_track_timer_tick)
            self._m13_track_timer = t
        interval = _M14_TRACK_INTERVAL_MS if self._m14_ai_follow_active() else _M13_TRACK_INTERVAL_MS
        if t.interval() != interval:
            t.setInterval(interval)
        if not t.isActive():
            t.start()

    def _m13_track_timer_tick(self) -> None:
        if not self._m13_track_is_active():
            self._sync_m13_track_timer()
            return
        if self._m14_ai_follow_active():
            self._m14_track_timer_tick()
            return
        notify_companion_preview_motion(duration_s=60.0)
        cc = getattr(self, "_camera_control", None)
        active_fn = getattr(cc, "is_target_track_active", None)
        if callable(active_fn) and not bool(active_fn()):
            self._m13_track_active = False
            self._sync_m13_track_timer()
            self._refresh_m13_track_overlay()
            self._sync_m13_track_button()
            return
        notify_companion_preview_motion(duration_s=120.0)
        ticks = int(getattr(self, "_m13_track_keepalive_ticks", 0) or 0) + 1
        self._m13_track_keepalive_ticks = ticks
        if ticks >= _M13_SUM_KEEPALIVE_TICKS:
            self._m13_track_keepalive_ticks = 0
            keep_fn = getattr(cc, "refresh_visual_track_keepalive", None)
            if callable(keep_fn):
                try:
                    keep_fn()
                except Exception:
                    pass
        self._update_m13_track_geo(force=False)
        self._m13_check_gimbal_follow()
        self._m13_nudge_video_during_track()
        self._refresh_m13_track_overlay()

    def _m14_track_timer_tick(self) -> None:
        """AI-follow tick: update the software tracker, drive the gimbal from
        its result, and still run the shared video-stall nudge + overlay."""
        notify_companion_preview_motion(duration_s=60.0)
        self._m14_dispatch_follow_update()
        self._m13_nudge_video_during_track()
        self._refresh_m13_track_overlay()

    def _m14_dispatch_follow_update(self) -> None:
        if bool(getattr(self, "_m14_follow_task_inflight", False)):
            # Normal now that updates self-chain off completion (see
            # _on_m14_follow_updated): the 100ms QTimer's own call to this
            # method will usually find a chained cycle already in flight —
            # that's expected, not a problem. Only warn on a genuine stall
            # (no progress for ~2s+, e.g. the worker subprocess died).
            skipped = int(getattr(self, "_m14_follow_ticks_skipped", 0) or 0) + 1
            self._m14_follow_ticks_skipped = skipped
            if skipped == 20 or (skipped > 20 and skipped % 20 == 0):
                print(
                    f"[VGCS:m14] follow appears stalled — no update in "
                    f"{skipped * 0.1:.1f}s (previous CSRT round trip never returned)"
                )
            return
        if int(getattr(self, "_m14_follow_ticks_skipped", 0) or 0):
            self._m14_follow_ticks_skipped = 0
        tracker = getattr(self, "_m14_tracker", None)
        bridge = getattr(self, "_m14_follow_bridge", None)
        if tracker is None or bridge is None or not tracker.is_active():
            return
        img = self._preview_image_copy_for_snapshot()
        frame_bgr = qimage_to_bgr_array(img) if img is not None else None
        if frame_bgr is None:
            return
        profile = self._m14_active_profile()
        fov_h = float(getattr(profile, "fov_h_deg", 83.4) or 83.4)
        fov_v = float(getattr(profile, "fov_v_deg", 46.9) or 46.9)
        fw, fh = frame_bgr.shape[1], frame_bgr.shape[0]
        gen = int(getattr(self, "_m13_track_generation", 0) or 0)
        self._m14_follow_task_inflight = True
        task = M14FollowTask(
            tracker,
            frame_bgr,
            bridge,
            frame_w=fw,
            frame_h=fh,
            fov_h_deg=fov_h,
            fov_v_deg=fov_v,
            gains=None,
            generation=gen,
        )
        QThreadPool.globalInstance().start(task)

    def _on_m14_follow_updated(
        self,
        ok: bool,
        u_norm: float,
        v_norm: float,
        yaw_speed_dps: float,
        pitch_speed_dps: float,
        lost_streak: int,
        generation: int,
        box_w: float = 0.0,
        box_h: float = 0.0,
    ) -> None:
        self._m14_follow_task_inflight = False
        if int(generation or 0) != int(getattr(self, "_m13_track_generation", 0) or 0):
            return  # stale — track was stopped/restarted while this update was in flight
        if not self._m14_ai_follow_active():
            return
        if not ok:
            self._m14_follow_lost_streak = int(lost_streak)
            if int(lost_streak) >= _M14_LOST_STREAK_STOP:
                # Intentional safety stop (CSRT lost the target for
                # ~_M14_LOST_STREAK_STOP ticks) — logged so this doesn't look
                # like an unexplained crash/hang from the console alone; the
                # GUI status bar message alone was easy to miss mid-flight.
                print(
                    f"[VGCS:m14] follow mode auto-stopped — target lost for "
                    f"{int(lost_streak)} consecutive ticks (re-click to reacquire)"
                )
                self._set_status("M13 track — target lost, tracker stopped")
                self._stop_m13_track()
                return
            # Field-measured: the real CSRT round trip (real camera frame +
            # isolated-subprocess IPC) reliably runs a bit longer than the
            # 100ms tick, so waiting for the next fixed-interval QTimer tick
            # wastes up to a full tick doing nothing every single cycle —
            # this is what field logs showed as "gimbal reacts late". Chain
            # the next update immediately instead of polling for it.
            self._m14_dispatch_follow_update()
            return
        self._m14_follow_lost_streak = 0
        self._m13_track_click_uv = (float(u_norm), float(v_norm))
        cc = getattr(self, "_camera_control", None)
        set_speed = getattr(cc, "set_gimbal_speed", None)
        # Field-reported: box tracks the person correctly, but the gimbal
        # never physically moves, with zero error output anywhere. Every
        # link in this chain (this call -> SkydroidCameraControl.set_gimbal_
        # speed -> adapter.set_speed -> command queue) silently swallows
        # exceptions, so a failure at any point is invisible. Throttled
        # (~1/s at the 100ms tick rate) so this is diagnostic, not log spam.
        tick = int(getattr(self, "_m14_follow_log_tick", 0) or 0) + 1
        self._m14_follow_log_tick = tick
        if tick % 10 == 1:
            # Field-reported: commanded yaw/pitch speed stays substantial and
            # nonzero for many consecutive ticks with the tracked box's
            # position never converging — consistent with either the gimbal
            # genuinely not moving, or it having hit a mechanical travel
            # limit. Logging the gimbal's OWN reported attitude (independent
            # ground truth from telemetry, not inferred from box position)
            # settles which it is directly in the next field log.
            att_str = "unsupported"
            try:
                get_status = getattr(cc, "get_gimbal_status", None)
                st = get_status() if callable(get_status) else None
                if st is not None and bool(getattr(st, "supported", False)):
                    att_str = (
                        f"yaw={float(st.yaw_deg):.1f} pitch={float(st.pitch_deg):.1f}"
                        if st.yaw_deg is not None and st.pitch_deg is not None
                        else "no-reading"
                    )
            except Exception:
                pass
            print(
                f"[VGCS:m14] follow uv=({u_norm:.3f},{v_norm:.3f}) "
                f"yaw_spd={float(yaw_speed_dps):.2f}dps pitch_spd={float(pitch_speed_dps):.2f}dps "
                f"gimbal_att=({att_str}) box=({float(box_w):.0f}x{float(box_h):.0f}) "
                f"cc={type(cc).__name__ if cc is not None else None} "
                f"has_set_speed={callable(set_speed)}"
            )
        if callable(set_speed):
            try:
                set_speed(float(yaw_speed_dps), float(pitch_speed_dps))
            except Exception as ex:
                print(f"[VGCS:m14] set_gimbal_speed raised: {ex!r}")
        else:
            print("[VGCS:m14] set_gimbal_speed not callable on camera control — gimbal command NOT sent")
        self._update_m13_track_geo(force=False)
        # Chain immediately (see comment above) rather than waiting for the
        # next 100ms QTimer tick — runs the loop back-to-back at whatever
        # rate the real hardware/subprocess round trip actually sustains.
        self._m14_dispatch_follow_update()

    def _m13_nudge_video_during_track(self) -> None:
        if not self._m13_track_is_active():
            return
        import time

        now = time.monotonic()
        last_nudge = float(getattr(self, "_m13_video_nudge_mono", 0.0) or 0.0)
        if now - last_nudge < _M13_VIDEO_NUDGE_MIN_S:
            return
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(
            self, "_video", None
        )
        if vp is None:
            return
        stale_age = 0.0
        stale_src = None
        try:
            for sid in ("day",):
                src = vp.sources().get(sid)
                if src is None:
                    continue
                last_frame = float(getattr(src, "_ffmpeg_last_frame_mono", 0.0) or 0.0)
                if last_frame <= 0.0:
                    continue
                age = now - last_frame
                if age >= _M13_VIDEO_STALL_S and age > stale_age:
                    stale_age = age
                    stale_src = src
        except Exception:
            return
        if stale_src is None:
            return
        self._m13_video_nudge_mono = now
        notify_companion_preview_motion(duration_s=180.0)
        if self._m14_ai_follow_active():
            # M14's video feed IS the tracking input (unlike GOT+SUM, which
            # tracks entirely in C13 firmware — a stale GCS preview there is
            # only cosmetic). Reconnecting costs several seconds of zero
            # frames (decode restart + GOP warmup) with NOTHING for the CSRT
            # tracker to update on — actively worse for continuous follow
            # than tolerating the gap, consistent with how a stall this
            # short is already treated as normal everywhere else in the
            # video pipeline (the general stall watchdog is disabled by
            # default with that exact reasoning). M14's own lost-streak
            # safety net (_M14_LOST_STREAK_STOP, ~3s) still stops tracking
            # if the target is genuinely gone, so this isn't unguarded.
            try:
                print(
                    f"[VGCS:m13] video stall during M14 track ({stale_age:.1f}s) "
                    "— tolerating, not reconnecting (would interrupt follow)"
                )
            except Exception:
                pass
            return
        try:
            print(
                f"[VGCS:m13] video stall during track ({stale_age:.1f}s) "
                "— RTSP decode refresh"
            )
        except Exception:
            pass
        try:
            if hasattr(stale_src, "restart_decode"):
                stale_src.restart_decode(delay_ms=600)
        except Exception:
            pass

    def _m13_check_gimbal_follow(self) -> None:
        if not self._m13_track_is_active():
            return
        import time

        lock_att = getattr(self, "_m13_track_lock_att", None)
        if not isinstance(lock_att, tuple) or len(lock_att) < 2:
            return
        att = self._read_gimbal_attitude_pair()
        if att is None:
            return
        try:
            dy = abs(float(att[0]) - float(lock_att[0]))
            dp = abs(float(att[1]) - float(lock_att[1]))
        except (TypeError, ValueError):
            return
        if dy > _M13_FOLLOW_MOVE_DEG or dp > _M13_FOLLOW_MOVE_DEG:
            self._m13_track_follow_seen = True
            notify_companion_preview_motion(duration_s=60.0)
        start_mono = float(getattr(self, "_m13_track_start_mono", 0.0) or 0.0)
        if start_mono <= 0.0:
            return
        if (
            not getattr(self, "_m13_track_follow_warned", False)
            and not getattr(self, "_m13_track_follow_seen", False)
            and (time.monotonic() - start_mono) >= _M13_FOLLOW_WARN_AFTER_S
        ):
            self._m13_track_follow_warned = True
            self._set_status(
                "M13 track — gimbal not moving; check C13 follow mode or re-click target"
            )

    def _query_m13_track_range_m(self, *, fresh: bool = False) -> float | None:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return None
        fn = getattr(cc, "query_slr_distance_m", None)
        if callable(fn):
            try:
                dist = fn(fresh=bool(fresh))
                if dist is not None:
                    return float(dist)
            except TypeError:
                try:
                    dist = fn()
                    if dist is not None:
                        return float(dist)
                except Exception:
                    pass
            except Exception:
                pass
        raw = getattr(self, "_companion_laser_range_m", None)
        if raw is not None:
            try:
                return float(raw)
            except (TypeError, ValueError):
                return None
        return None

    def _update_m13_track_geo(self, *, force: bool = False) -> None:
        if not self._m13_track_is_active():
            return
        import time

        now = time.monotonic()
        # Fire a fresh SLR shot on a cadence, off the GUI thread. The laser
        # register only advances when re-triggered, so a moving (especially
        # radially moving) target needs periodic fresh shots — otherwise the
        # plotted position freezes at the start-time range. The trigger+settle
        # sleeps would stall the video/map if done inline on this 200 ms timer,
        # so the read runs on a worker and posts back via _on_m13_range_ready.
        # Setting VGCS_M13_SLR_FRESH_INTERVAL_S=0 disables only the PERIODIC
        # re-fire (for isolating whether it causes the video stutter); the
        # one-time fetch on track start (force=True) still runs so a position
        # is always available.
        fresh_interval = _m13_slr_fresh_interval_s()
        last_fresh = float(getattr(self, "_m13_track_slr_fresh_mono", 0.0) or 0.0)
        want_periodic = fresh_interval > 0.0 and (now - last_fresh) >= fresh_interval
        if force or want_periodic:
            self._dispatch_m13_range_fetch(fresh=True)
        last = float(getattr(self, "_m13_track_geo_mono", 0.0) or 0.0)
        if not force and (now - last) < _M13_GEO_MIN_INTERVAL_S:
            return
        self._m13_track_geo_mono = now
        # Recompute from the last fetched range with LIVE gimbal attitude, so the
        # marker still updates between range shots as the gimbal follows.
        self._recompute_m13_track_geo()

    def _dispatch_m13_range_fetch(self, *, fresh: bool) -> None:
        """Queue an off-GUI-thread SLR read; one at a time (no overlapping shots)."""
        if bool(getattr(self, "_m13_range_task_inflight", False)):
            return
        cc = getattr(self, "_camera_control", None)
        bridge = getattr(self, "_m13_range_bridge", None)
        if cc is None or bridge is None:
            return
        import time

        self._m13_range_task_inflight = True
        self._m13_track_slr_fresh_mono = time.monotonic()
        # Remember where the gimbal was aimed when this shot fired, so the geo can
        # tell whether the range still matches the current look direction.
        self._m13_track_range_att = self._read_gimbal_attitude_pair()
        gen = int(getattr(self, "_m13_track_generation", 0) or 0)
        task = M13RangeTask(
            self._query_m13_track_range_m, bridge, fresh=bool(fresh), generation=gen
        )
        QThreadPool.globalInstance().start(task)

    def _on_m13_range_ready(self, range_m: object, generation: int = 0) -> None:
        self._m13_range_task_inflight = False
        if int(generation or 0) != int(getattr(self, "_m13_track_generation", 0) or 0):
            return  # stale — track was stopped/restarted while this shot was in flight
        if not self._m13_track_is_active():
            return
        if range_m is not None:
            try:
                self._m13_track_range_m = float(range_m)
            except (TypeError, ValueError):
                pass
        self._recompute_m13_track_geo()

    @staticmethod
    def _m13_geo_uncertain_from_att(range_att: object, cur_att: object) -> bool:
        """True when the gimbal moved past the slew threshold since the range shot.

        Diffs two same-source attitude samples, so any constant convention offset
        cancels. Signals that the boresight-range geo is only approximate because
        the target has likely drifted off-centre (no track-box feedback to correct).
        """
        if not (isinstance(range_att, tuple) and isinstance(cur_att, tuple)):
            return False
        if len(range_att) < 2 or len(cur_att) < 2:
            return False
        try:
            dyaw = abs(float(cur_att[0]) - float(range_att[0]))
            dpitch = abs(float(cur_att[1]) - float(range_att[1]))
        except (TypeError, ValueError):
            return False
        return max(dyaw, dpitch) > _M13_SLEW_UNCERTAIN_DEG

    def _recompute_m13_track_geo(self) -> None:
        if not self._m13_track_is_active():
            return
        dist = getattr(self, "_m13_track_range_m", None)
        ctx = self._observation_context()
        if ctx.get("gimbal_yaw_deg") is None or dist is None:
            self._refresh_m13_track_map_marker()
            return
        try:
            slant = float(dist)
        except (TypeError, ValueError):
            self._refresh_m13_track_map_marker()
            return
        if slant < 0.5:
            self._refresh_m13_track_map_marker()
            return
        ai_follow = self._m14_ai_follow_active()
        if ai_follow:
            # Real tracked pixel from our own tracker — not a boresight guess
            # — and the ACTIVE camera's real FOV, not C13's hardcoded fallback.
            profile = self._m14_active_profile()
            hfov = float(getattr(profile, "fov_h_deg", 83.4) or 83.4)
            vfov = float(getattr(profile, "fov_v_deg", 46.9) or 46.9)
            click = getattr(self, "_m13_track_click_uv", None)
            video_x, video_y = (click if isinstance(click, tuple) else (0.5, 0.5))
        else:
            hfov, vfov = self._c13_lrf_geo_fov()
            video_x, video_y = 0.5, 0.5
        geo = compute_lrf_slant_geo(
            vehicle_lat=ctx.get("vehicle_lat"),  # type: ignore[arg-type]
            vehicle_lon=ctx.get("vehicle_lon"),  # type: ignore[arg-type]
            vehicle_heading_deg=ctx.get("vehicle_heading_deg"),  # type: ignore[arg-type]
            vehicle_roll_deg=ctx.get("vehicle_roll_deg"),  # type: ignore[arg-type]
            vehicle_pitch_deg=ctx.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
            vehicle_alt_msl_m=ctx.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
            gimbal_yaw_deg=ctx.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
            gimbal_pitch_deg=ctx.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
            slant_range_m=slant,
            video_x_norm=float(video_x),
            video_y_norm=float(video_y),
            gps_fix_type=int(ctx.get("gps_fix_type") or 0),
            gps_hdop=ctx.get("gps_hdop"),  # type: ignore[arg-type]
            camera_hfov_deg=hfov,
            camera_vfov_deg=vfov,
        )
        if geo is None or not geo.ok or geo.target_lat is None or geo.target_lon is None:
            self._m13_track_geo_label = geo.warning if geo is not None else "Computing track position…"
            self._refresh_m13_track_map_marker()
            return
        self._m13_track_lat = float(geo.target_lat)
        self._m13_track_lon = float(geo.target_lon)
        self._m13_track_alt_m = (
            float(geo.target_alt_m) if geo.target_alt_m is not None else None
        )
        label = f"{self._m13_track_lat:.6f}, {self._m13_track_lon:.6f}"
        if geo.range_m is not None:
            label += f" · {float(geo.range_m):.0f} m"
        if ai_follow:
            # Real tracked pixel every frame — no boresight-drift guess needed.
            uncertain = False
        else:
            uncertain = self._m13_geo_uncertain_from_att(
                getattr(self, "_m13_track_range_att", None),
                self._read_gimbal_attitude_pair(),
            )
        self._m13_track_geo_uncertain = bool(uncertain)
        if uncertain:
            label += " · approx (following)"
        self._m13_track_geo_label = label
        path = list(getattr(self, "_m13_track_path", None) or [])
        pt = (self._m13_track_lat, self._m13_track_lon)
        if not path or path[-1] != pt:
            path.append(pt)
            if len(path) > _M13_PATH_MAX_POINTS:
                path = path[-_M13_PATH_MAX_POINTS:]
            self._m13_track_path = path
        self._refresh_m13_track_map_marker()

    def _refresh_m13_track_map_marker(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        lat = getattr(self, "_m13_track_lat", None)
        lon = getattr(self, "_m13_track_lon", None)
        path = list(getattr(self, "_m13_track_path", None) or [])
        try:
            if hasattr(nm, "set_m13_track_marker"):
                if lat is None or lon is None:
                    nm.set_m13_track_marker(None, None)
                else:
                    nm.set_m13_track_marker(float(lat), float(lon))
            if hasattr(nm, "set_m13_track_path"):
                nm.set_m13_track_path(path)
        except Exception:
            pass

    def _clear_m13_track_map(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            if hasattr(nm, "set_m13_track_marker"):
                nm.set_m13_track_marker(None, None)
            if hasattr(nm, "set_m13_track_path"):
                nm.set_m13_track_path([])
        except Exception:
            pass

    def get_m13_track_latlon(self) -> tuple[float, float] | None:
        lat = getattr(self, "_m13_track_lat", None)
        lon = getattr(self, "_m13_track_lon", None)
        if lat is None or lon is None:
            return None
        try:
            return float(lat), float(lon)
        except (TypeError, ValueError):
            return None

    def is_m13_track_active(self) -> bool:
        return self._m13_track_is_active()

    def _refresh_m13_track_overlay(
        self,
        *,
        pending: bool = False,
        failed: bool = False,
    ) -> None:
        ly = getattr(self, "_native_video_overlay", None)
        if ly is None:
            return
        armed = self._m13_track_mode_active() and not self._m13_track_is_active()
        active = self._m13_track_is_active()
        if not armed and not active and not pending and not failed:
            try:
                ly.set_m13_track(None)
                ly.set_m13_track_armed(False)
                ly.update()
            except Exception:
                pass
            return
        click = getattr(self, "_m13_track_click_uv", None)
        u, v = (0.5, 0.5)
        if isinstance(click, tuple):
            u, v = float(click[0]), float(click[1])
        dist = getattr(self, "_m13_track_range_m", None)
        label = str(getattr(self, "_m13_track_geo_label", "") or "")
        overlay = VideoOverlayM13Track(
            x=float(u),
            y=float(v),
            distance_m=float(dist) if dist is not None else None,
            pending=bool(pending),
            failed=bool(failed),
            active=bool(active),
            geo_label=label,
        )
        try:
            ly.set_m13_track_armed(bool(armed))
            ly.set_m13_track(overlay)
            ly.update()
        except Exception:
            pass

    def enable_m13_track_ui(self, on: bool) -> None:
        btn = getattr(self, "_btn_native_m13_track", None)
        body = getattr(self, "_native_m13_track_body", None)
        if btn is not None:
            btn.setVisible(bool(on))
            btn.setEnabled(bool(on))
        if body is not None:
            body.setVisible(bool(on))
        if not on:
            self._reset_m13_track_for_disconnect()
