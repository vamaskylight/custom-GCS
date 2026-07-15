"""M13 — Moving target tracking (C13 GOT + SUM, GCS-computed map coordinates)."""

from __future__ import annotations

from PySide6.QtCore import QThreadPool, QTimer

from vgcs.map.native_video_overlay import VideoOverlayM13Track
from vgcs.map.observation.types import M13TrackBridge, M13TrackStartTask
from vgcs.observe.geo_reference import compute_lrf_slant_geo
from vgcs.video.camera_control import uses_skydroid_top_camera
from vgcs.video.pipeline import (
    notify_companion_preview_motion,
    notify_companion_visual_track,
)


_M13_PATH_MAX_POINTS = 400
_M13_GEO_MIN_INTERVAL_S = 1.0
_M13_SLR_FRESH_INTERVAL_S = 3.0
_M13_FOLLOW_WARN_AFTER_S = 6.0
_M13_FOLLOW_MOVE_DEG = 0.8


class M13MovingTargetTrackMixin:
    """Click-to-track on C13 day video; map coords from GPS + gimbal + SLR."""

    def _m13_track_supported(self) -> bool:
        return uses_skydroid_top_camera(getattr(self, "_camera_control", None))

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
        self._sync_m13_track_button()
        self._refresh_m13_track_overlay(pending=True)
        notify_companion_visual_track(active=True)
        self._set_status("M13 track — slewing gimbal and locking target…")
        cc = getattr(self, "_camera_control", None)
        bridge = getattr(self, "_m13_track_bridge", None)
        if cc is None or bridge is None:
            self._m13_track_starting = False
            notify_companion_visual_track(active=False)
            self._set_status("M13 track failed — no camera control")
            return
        task = M13TrackStartTask(cc, float(u), float(v), bridge, generation=gen)
        QThreadPool.globalInstance().start(task)

    def _on_m13_track_started(self, ok: bool, u: float, v: float, generation: int = 0) -> None:
        if int(generation or 0) != int(getattr(self, "_m13_track_generation", 0) or 0):
            print(
                f"[VGCS:m13] ignoring stale track result gen={generation} "
                f"current={getattr(self, '_m13_track_generation', 0)}"
            )
            return
        self._m13_track_starting = False
        if not ok:
            self._m13_track_active = False
            notify_companion_visual_track(active=False)
            self._refresh_m13_track_overlay(failed=True)
            self._set_status(
                "M13 track failed — gimbal did not slew to target (check C13 motion)"
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
        self._sync_m13_track_button()
        self._refresh_m13_track_overlay()
        self._update_m13_track_geo(force=True)
        self._sync_m13_track_timer()
        self._set_status("M13 tracking — gimbal follow active")

    def _stop_m13_track(self) -> None:
        was = self._m13_track_is_active() or self._m13_track_mode_active()
        self._m13_track_active = False
        self._m13_track_armed = False
        self._m13_track_starting = False
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
        self._m13_track_active = False
        self._m13_track_armed = False
        self._m13_track_starting = False
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
            t.setInterval(200)
            t.timeout.connect(self._m13_track_timer_tick)
            self._m13_track_timer = t
        if not t.isActive():
            t.start()

    def _m13_track_timer_tick(self) -> None:
        if not self._m13_track_is_active():
            self._sync_m13_track_timer()
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
        self._update_m13_track_geo()
        self._m13_check_gimbal_follow()
        self._refresh_m13_track_overlay()

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
        last = float(getattr(self, "_m13_track_geo_mono", 0.0) or 0.0)
        if not force and (now - last) < _M13_GEO_MIN_INTERVAL_S:
            return
        self._m13_track_geo_mono = now
        last_fresh = float(getattr(self, "_m13_track_slr_fresh_mono", 0.0) or 0.0)
        want_fresh = bool(force) and (now - last_fresh) >= _M13_SLR_FRESH_INTERVAL_S
        if want_fresh:
            self._m13_track_slr_fresh_mono = now
        dist = self._query_m13_track_range_m(fresh=want_fresh)
        self._m13_track_range_m = dist
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
        hfov, vfov = self._c13_lrf_geo_fov()
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
            video_x_norm=0.5,
            video_y_norm=0.5,
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
        self._m13_track_geo_label = (
            f"{self._m13_track_lat:.6f}, {self._m13_track_lon:.6f}"
        )
        if geo.range_m is not None:
            self._m13_track_geo_label += f" · {float(geo.range_m):.0f} m"
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
        if active:
            u, v = 0.5, 0.5
        elif isinstance(click, tuple):
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
