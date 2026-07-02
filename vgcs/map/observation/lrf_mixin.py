"""MapWidget mixin — see vgcs.map.observation package."""

from __future__ import annotations

from PySide6.QtCore import QThreadPool, QTimer

from vgcs.map.native_video_overlay import VideoOverlayLrfLock
from vgcs.map.observation.types import LrfLockBridge, LrfLockTask, PendingLrfVideoPick
from vgcs.observe.dooaf import DOOAF_ROLE_GUN, DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED
from vgcs.observe.geo_reference import compute_lrf_slant_geo
from vgcs.skydroid.protocol import format_slr_display_m
from vgcs.video.pipeline import notify_companion_lrf_lock


class LrfVideoLockMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _reset_c13_lrf_for_observe_reset(self) -> None:
        """OBSERVE Reset also clears C13 LRF lock / armed / failed reticle."""
        try:
            if self._c13_lrf_is_locked():
                self._unlock_c13_lrf()
                return
            if bool(getattr(self, "_lrf_lock_armed", False)) or bool(
                getattr(self, "_lrf_lock_failed", False)
            ):
                self._cancel_c13_lrf_arm()
        except Exception:
            pass

    def _read_gimbal_attitude_pair(self) -> tuple[float, float] | None:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return None
        try:
            st = cc.get_gimbal_status()
            if st is None or not bool(getattr(st, "supported", False)):
                return None
            yaw = getattr(st, "yaw_deg", None)
            pitch = getattr(st, "pitch_deg", None)
            if yaw is None or pitch is None:
                return None
            return float(yaw), float(pitch)
        except Exception:
            return None

    def _lrf_reticle_tracking_active(self) -> bool:
        if getattr(self, "_lrf_track_ref_uv", None) is None:
            return False
        if getattr(self, "_lrf_track_ref_att", None) is None:
            return False
        return bool(getattr(self, "_lrf_lock_in_progress", False)) or self._c13_lrf_is_locked()

    def _capture_lrf_track_ref(self, u: float, v: float) -> None:
        att = self._read_gimbal_attitude_pair()
        if att is None:
            self._lrf_track_ref_uv = None
            self._lrf_track_ref_att = None
            self._lrf_click_att = None
            return
        self._lrf_track_ref_uv = (float(u), float(v))
        self._lrf_track_ref_att = att
        self._lrf_click_att = att
        self._lrf_track_gac_h_scale = 1.0
        self._lrf_track_gac_v_scale = 1.0

    def _calibrate_lrf_track_after_lock(self) -> None:
        """Refine GAC→screen mapping from the slew we just completed."""
        ref_uv = getattr(self, "_lrf_track_ref_uv", None)
        ref_att = getattr(self, "_lrf_track_ref_att", None)
        lock_att = self._read_gimbal_attitude_pair()
        if ref_uv is None or ref_att is None or lock_att is None:
            return
        try:
            from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

            h_scale = float(getattr(self, "_lrf_track_gac_h_scale", 1.0) or 1.0)
            v_scale = float(getattr(self, "_lrf_track_gac_v_scale", 1.0) or 1.0)
            click_off = max(abs(float(ref_uv[0]) - 0.5), abs(float(ref_uv[1]) - 0.5))
            if click_off > 0.02:
                h_new, v_new = SkydroidTopUdpAdapter.calibrate_track_gac_scales(
                    ref_uv,
                    ref_att,
                    lock_att,
                    h_scale=h_scale,
                    v_scale=v_scale,
                )
                self._lrf_track_gac_h_scale = float(h_new)
                self._lrf_track_gac_v_scale = float(v_new)
                print(
                    f"[VGCS:lrf] track calibrate gac_scale=({h_new:.3f},{v_new:.3f})"
                )
            u_chk, v_chk = SkydroidTopUdpAdapter.lrf_track_uv_from_attitude(
                ref_uv,
                ref_att,
                lock_att,
                gac_h_scale=float(self._lrf_track_gac_h_scale),
                gac_v_scale=float(self._lrf_track_gac_v_scale),
            )
            res_u = abs(float(u_chk) - 0.5)
            res_v = abs(float(v_chk) - 0.5)
            # Laser range is along boresight — track from frame centre at lock pose.
            self._lrf_track_ref_uv = (0.5, 0.5)
            self._lrf_track_ref_att = lock_att
            self._lrf_lock_uv = (0.5, 0.5)
            print(
                f"[VGCS:lrf] track lock boresight residual=({res_u:.3f},{res_v:.3f})"
            )
        except Exception:
            pass

    def _clear_lrf_track_ref(self) -> None:
        self._lrf_track_ref_uv = None
        self._lrf_track_ref_att = None
        self._lrf_track_gac_h_scale = 1.0
        self._lrf_track_gac_v_scale = 1.0

    def _update_lrf_reticle_track(self) -> None:
        in_progress = bool(getattr(self, "_lrf_lock_in_progress", False))
        # During slew: attitude track from click pose so mark stays on the building.
        ref_uv = getattr(self, "_lrf_track_ref_uv", None)
        ref_att = getattr(self, "_lrf_track_ref_att", None)
        if ref_uv is None or ref_att is None:
            click_uv = getattr(self, "_lrf_click_uv", None)
            if click_uv is not None:
                self._lrf_lock_uv = (float(click_uv[0]), float(click_uv[1]))
            return
        if not in_progress and not self._lrf_reticle_tracking_active():
            return
        cur = self._read_gimbal_attitude_pair()
        if cur is None:
            return
        try:
            from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

            h_scale = float(getattr(self, "_lrf_track_gac_h_scale", 1.0) or 1.0)
            v_scale = float(getattr(self, "_lrf_track_gac_v_scale", 1.0) or 1.0)
            if in_progress:
                click_uv = getattr(self, "_lrf_click_uv", None)
                click_att = getattr(self, "_lrf_click_att", None)
                if click_uv is not None and click_att is not None:
                    dy = abs(float(cur[0]) - float(click_att[0]))
                    dp = abs(float(cur[1]) - float(click_att[1]))
                    if dy > 0.06 or dp > 0.06:
                        h_new, v_new = SkydroidTopUdpAdapter.calibrate_track_gac_scales(
                            (float(click_uv[0]), float(click_uv[1])),
                            click_att,
                            cur,
                            h_scale=h_scale,
                            v_scale=v_scale,
                            live=True,
                        )
                        h_scale = float(h_new)
                        v_scale = float(v_new)
                        self._lrf_track_gac_h_scale = h_scale
                        self._lrf_track_gac_v_scale = v_scale
                ref_uv = (float(click_uv[0]), float(click_uv[1])) if click_uv else ref_uv
                ref_att = click_att if click_att is not None else ref_att
            u, v = SkydroidTopUdpAdapter.lrf_track_uv_from_attitude(
                ref_uv,
                ref_att,
                cur,
                gac_h_scale=h_scale,
                gac_v_scale=v_scale,
            )
            self._lrf_lock_uv = (float(u), float(v))
        except Exception:
            pass

    def _clear_lrf_lock_geo(self) -> None:
        self._lrf_lock_lat = None
        self._lrf_lock_lon = None
        self._lrf_lock_alt_m = None
        self._lrf_lock_geo_label = ""
        try:
            nm = self._native_map
            if nm is not None and hasattr(nm, "set_lrf_lock_marker"):
                nm.set_lrf_lock_marker(None, None)
        except Exception:
            pass

    def _format_lrf_geo_label(self, lat: float, lon: float) -> str:
        return f"{float(lat):.6f}, {float(lon):.6f}"

    def _c13_lrf_geo_fov(self) -> tuple[float, float]:
        try:
            from vgcs.skydroid.adapter import (
                _LRF_FOV_H_DEG as hfov,
                _LRF_FOV_V_DEG as vfov,
            )

            return float(hfov), float(vfov)
        except Exception:
            return 83.4, 46.9

    def _update_lrf_lock_geo(self, distance_m: float | None) -> None:
        """Estimate locked target lat/lon from vehicle pose, gimbal, and LRF slant range."""
        if distance_m is None:
            return
        try:
            slant = float(distance_m)
        except (TypeError, ValueError):
            return
        if slant < 0.5:
            return
        ctx = self._observation_context()
        if ctx.get("gimbal_yaw_deg") is None:
            return
        in_progress = bool(getattr(self, "_lrf_lock_in_progress", False))
        click_uv = getattr(self, "_lrf_click_uv", None)
        use_click = (
            in_progress
            and click_uv is not None
            and not self._c13_lrf_is_locked()
            and getattr(self, "_lrf_lock_distance_m", None) is None
        )
        if use_click:
            video_u, video_v = float(click_uv[0]), float(click_uv[1])
        else:
            # After slew: SLR measures along gimbal boresight (frame centre).
            video_u, video_v = 0.5, 0.5
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
            video_x_norm=video_u,
            video_y_norm=video_v,
            gps_fix_type=int(ctx.get("gps_fix_type") or 0),
            gps_hdop=ctx.get("gps_hdop"),  # type: ignore[arg-type]
            camera_hfov_deg=hfov,
            camera_vfov_deg=vfov,
        )
        if not geo.ok or geo.target_lat is None or geo.target_lon is None:
            print(
                f"[VGCS:lrf] geo unavailable — {geo.warning or geo.quality}"
            )
            return
        self._lrf_lock_lat = float(geo.target_lat)
        self._lrf_lock_lon = float(geo.target_lon)
        if geo.target_alt_m is not None:
            try:
                self._lrf_lock_alt_m = float(geo.target_alt_m)
            except (TypeError, ValueError):
                pass
        self._lrf_lock_geo_label = self._format_lrf_geo_label(
            float(geo.target_lat), float(geo.target_lon)
        )
        try:
            nm = self._native_map
            if nm is not None and hasattr(nm, "set_lrf_lock_marker"):
                nm.set_lrf_lock_marker(float(geo.target_lat), float(geo.target_lon))
        except Exception:
            pass
        print(
            f"[VGCS:lrf] target geo=({float(geo.target_lat):.6f},"
            f"{float(geo.target_lon):.6f}) q={geo.quality} "
            f"range={slant:.1f} m"
        )

    def _refresh_lrf_lock_overlay(self, *, sync_geometry: bool = True) -> None:
        """Draw cyan LRF reticle on video at the VGCS-locked target point."""
        try:
            ly = self._native_video_overlay
        except AttributeError:
            return
        try:
            self._update_lrf_reticle_track()
            uv = getattr(self, "_lrf_lock_uv", None)
            armed = bool(getattr(self, "_lrf_lock_armed", False))
            dist = getattr(self, "_lrf_lock_distance_m", None)
            in_progress = bool(getattr(self, "_lrf_lock_in_progress", False))
            failed = bool(getattr(self, "_lrf_lock_failed", False))
            pending = in_progress or failed or (
                uv is not None and not self._c13_lrf_is_locked() and not armed
            )
            if uv is not None:
                ly.set_lrf_lock(
                    VideoOverlayLrfLock(
                        x=float(uv[0]),
                        y=float(uv[1]),
                        distance_m=float(dist) if dist is not None else None,
                        pending=pending and not failed,
                        failed=failed,
                        geo_label=str(getattr(self, "_lrf_lock_geo_label", "") or ""),
                    )
                )
            else:
                ly.set_lrf_lock(None)
            ly.set_lrf_armed_hint(armed and uv is None)
            self._refresh_dooaf_facade_overlay_hint()
            if bool(getattr(self, "_video_preview_enabled", False)):
                ly.show()
                ly.raise_()
            if sync_geometry:
                self._sync_native_video_overlay(from_lrf=True)
            elif self._video_mark_tracking_active():
                self._sync_dooaf_video_marks_on_overlay(ly)
        except Exception:
            pass

    def _sync_dooaf_video_marks_on_overlay(self, ly: object | None = None) -> None:
        """Reproject DOOAF / observation marks after LRF reticle moved (same gimbal sample)."""
        try:
            overlay = ly if ly is not None else self._native_video_overlay
            marks = self._video_overlay_marks()
            self._video_obs_marks = [(m.x, m.y) for m in marks]
            overlay.set_video_marks(marks)
            overlay.set_offscreen_hints(self._video_overlay_offscreen_hints())
            overlay.update()
        except Exception:
            pass

    def set_companion_laser_range_m(self, distance_m: float | None) -> None:
        """C13 TOP laser rangefinder (Skydroid SLR); updates PROXIMITY Range when target is locked."""
        try:
            self._companion_laser_range_m = float(distance_m) if distance_m is not None else None
            if distance_m is not None and (
                getattr(self, "_lrf_lock_uv", None) is not None
                or getattr(self, "_lrf_lock_in_progress", False)
            ):
                self._lrf_lock_distance_m = float(distance_m)
                self._refresh_lrf_lock_overlay()
        except (TypeError, ValueError):
            self._companion_laser_range_m = None
        try:
            self._obstacle_radar.set_companion_lrf_range_m(distance_m)
            if bool(getattr(self, "_last_link_connected", False)) and bool(
                getattr(self, "_web_ready", False)
            ):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def enable_c13_lrf_ui(self, enabled: bool = True) -> None:
        """PROXIMITY Range shows C13 lock icon until user picks a target on video."""
        try:
            self._obstacle_radar.enable_c13_lrf_ui(bool(enabled))
            if enabled and bool(getattr(self, "_last_link_connected", False)):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def _c13_lrf_is_locked(self) -> bool:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return False
        fn = getattr(cc, "is_lrf_locked", None)
        return bool(fn()) if callable(fn) else False

    def _on_c13_lrf_icon_clicked(self) -> None:
        if self._c13_lrf_is_locked():
            self._unlock_c13_lrf()
            return
        if bool(getattr(self, "_lrf_lock_armed", False)):
            self._cancel_c13_lrf_arm()
            return
        self._arm_c13_lrf_lock()

    def _clear_lrf_failed_reticle(self) -> None:
        if not bool(getattr(self, "_lrf_lock_failed", False)):
            return
        self._lrf_lock_failed = False
        if not self._c13_lrf_is_locked():
            self._lrf_lock_uv = None
            self._lrf_click_uv = None
            self._lrf_click_att = None
            self._lrf_lock_armed = True
            try:
                self._obstacle_radar.set_c13_lrf_armed(True)
            except Exception:
                pass
        self._refresh_lrf_lock_overlay()

    def is_c13_lrf_armed(self) -> bool:
        return bool(getattr(self, "_lrf_lock_armed", False))

    def is_c13_lrf_locking(self) -> bool:
        return bool(getattr(self, "_lrf_lock_in_progress", False))

    def get_c13_lrf_lock_latlon(self) -> tuple[float, float] | None:
        lat = getattr(self, "_lrf_lock_lat", None)
        lon = getattr(self, "_lrf_lock_lon", None)
        if lat is None or lon is None:
            return None
        try:
            return float(lat), float(lon)
        except (TypeError, ValueError):
            return None

    def _sync_lrf_armed_backend(self, armed: bool) -> None:
        cc = getattr(self, "_camera_control", None)
        fn = getattr(cc, "set_lrf_armed", None)
        if callable(fn):
            try:
                fn(bool(armed))
            except Exception:
                pass

    def _arm_c13_lrf_lock(self) -> None:
        self._lrf_lock_armed = True
        self._lrf_lock_failed = False
        self._lrf_lock_uv = None
        self._lrf_click_uv = None
        self._lrf_click_att = None
        self._lrf_lock_distance_m = None
        self._clear_lrf_track_ref()
        self._sync_lrf_armed_backend(True)
        try:
            self._obstacle_radar.set_c13_lrf_armed(True)
        except Exception:
            pass
        self._refresh_lrf_lock_overlay()
        self._set_status("Click target on video to lock C13 laser range (camera will slew)")
        QTimer.singleShot(0, self._raise_flight_hud_above_video)

    def _cancel_c13_lrf_arm(self) -> None:
        self._lrf_lock_armed = False
        self._lrf_lock_failed = False
        self._lrf_lock_uv = None
        self._lrf_click_uv = None
        self._lrf_click_att = None
        self._lrf_lock_distance_m = None
        self._lrf_lock_in_progress = False
        notify_companion_lrf_lock(active=False)
        self._clear_lrf_track_ref()
        self._clear_lrf_lock_geo()
        self._sync_lrf_armed_backend(False)
        try:
            self._obstacle_radar.set_c13_lrf_armed(False)
        except Exception:
            pass
        self._refresh_lrf_lock_overlay()
        self._set_status("C13 LRF lock cancelled")

    def _unlock_c13_lrf(self) -> None:
        self._lrf_lock_armed = False
        self._lrf_lock_failed = False
        self._lrf_lock_uv = None
        self._lrf_click_uv = None
        self._lrf_click_att = None
        self._lrf_lock_distance_m = None
        self._lrf_lock_in_progress = False
        notify_companion_lrf_lock(active=False)
        self._clear_lrf_track_ref()
        self._clear_lrf_lock_geo()
        self._sync_lrf_armed_backend(False)
        cc = getattr(self, "_camera_control", None)
        try:
            unlock = getattr(cc, "unlock_lrf", None)
            if callable(unlock):
                unlock()
        except Exception:
            pass
        try:
            self._obstacle_radar.set_c13_lrf_locked(None)
        except Exception:
            pass
        try:
            self._companion_laser_range_m = None
        except Exception:
            pass
        self._refresh_lrf_lock_overlay()
        self._set_status("C13 LRF target unlocked")

    def _begin_c13_lrf_video_lock(self, u: float, v: float) -> None:
        cc = getattr(self, "_camera_control", None)
        unlock = getattr(cc, "unlock_lrf", None)
        if callable(unlock):
            try:
                unlock()
            except Exception:
                pass
        lock_fn = getattr(cc, "lock_lrf_at_video_norm", None)
        if not callable(lock_fn):
            self._set_status("C13 LRF not available — set Camera provider to Skydroid")
            self._cancel_c13_lrf_arm()
            return
        self._lrf_lock_armed = False
        self._lrf_lock_failed = False
        self._lrf_click_uv = (float(u), float(v))
        self._lrf_lock_uv = (float(u), float(v))
        self._lrf_lock_distance_m = None
        self._lrf_lock_in_progress = True
        self._capture_lrf_lock_start_vehicle_pose()
        self._clear_lrf_lock_geo()
        notify_companion_lrf_lock(active=True)
        self._notify_companion_gimbal_motion(duration_s=120.0)
        self._capture_lrf_track_ref(float(u), float(v))
        self._sync_video_mark_track_timer()
        self._sync_lrf_armed_backend(False)
        try:
            from vgcs.video.camera_control import poll_companion_laser_range_m

            live = poll_companion_laser_range_m(cc)
            if live is not None:
                self._lrf_lock_distance_m = float(live)
                self._update_lrf_lock_geo(float(live))
        except Exception:
            pass
        self._refresh_lrf_lock_overlay()
        if getattr(self, "_lrf_lock_distance_m", None) is not None:
            status = f"Locking LRF… {format_slr_display_m(self._lrf_lock_distance_m)}"
            if getattr(self, "_lrf_lock_geo_label", ""):
                status += f" · {self._lrf_lock_geo_label}"
            self._set_status(status)
        else:
            self._set_status("Locking LRF — slewing camera to target…")
        try:
            self._obstacle_radar.set_c13_lrf_locking(
                getattr(self, "_lrf_lock_distance_m", None),
                geo_label=str(getattr(self, "_lrf_lock_geo_label", "") or ""),
            )
        except Exception:
            pass
        fw, fh = 1280, 720
        task = LrfLockTask(cc, u, v, self._lrf_lock_bridge, frame_w=fw, frame_h=fh)
        pool = getattr(self, "_video_pool", None) or QThreadPool.globalInstance()
        pool.start(task)

    def _on_c13_lrf_lock_progress(self, distance_m: float) -> None:
        try:
            dm = float(distance_m)
            self._lrf_lock_distance_m = dm
            self._notify_companion_gimbal_motion(duration_s=45.0)
            self._update_lrf_lock_geo(dm)
            self._refresh_tracked_video_marks_light()
            pending = getattr(self, "_pending_lrf_video_pick", None)
            try:
                self._obstacle_radar.set_c13_lrf_locking(
                    dm, geo_label=str(getattr(self, "_lrf_lock_geo_label", "") or "")
                )
            except Exception:
                pass
            if pending is not None:
                status = (
                    f"{pending.label} — locking LRF… {format_slr_display_m(dm)}"
                )
            else:
                status = f"Locking LRF… {format_slr_display_m(dm)}"
            if getattr(self, "_lrf_lock_geo_label", ""):
                status += f" · {self._lrf_lock_geo_label}"
            self._set_status(status)
        except (TypeError, ValueError):
            pass

    def _on_c13_lrf_lock_finished(self, dist: object, u: float, v: float) -> None:
        self._lrf_lock_in_progress = False
        notify_companion_lrf_lock(active=False)
        self._notify_companion_gimbal_motion(duration_s=15.0)
        self._sync_video_mark_track_timer()
        pending = getattr(self, "_pending_lrf_video_pick", None)
        if pending is not None:
            pick_role = str(
                getattr(pending, "pick_role", None) or DOOAF_ROLE_INTENDED
            )
            self._pending_lrf_video_pick = None
            slant_m: float | None = None
            if dist is not None:
                try:
                    slant_m = float(dist)
                except (TypeError, ValueError):
                    slant_m = None
            if slant_m is not None:
                self._lrf_lock_distance_m = slant_m
                self._lrf_lock_failed = False
                self._companion_laser_range_m = slant_m
                self._calibrate_lrf_track_after_lock()
                self._update_lrf_lock_geo(slant_m)
                if pending.purpose == "dooaf_setup" and pick_role in (
                    DOOAF_ROLE_GUN,
                    "gun_origin",
                ):
                    self._try_record_dooaf_facade_session(float(slant_m))
                elif pending.purpose == "observation":
                    obs_row = getattr(pending, "observation_row", None)
                    if (
                        obs_row is not None
                        and str(obs_row.get("dooaf_role") or "") == DOOAF_ROLE_IMPACT
                        and not self._dooaf_facade_uv_pick_ready()
                    ):
                        self._try_record_dooaf_facade_session(float(slant_m))
            try:
                if pending.purpose == "dooaf_setup":
                    self._complete_pending_dooaf_setup_lrf_pick(slant_m, pending)
                elif pending.purpose == "observation":
                    self._complete_pending_observation_lrf_pick(slant_m, pending)
            except Exception as exc:
                print(f"[VGCS:observe] DOOAF/LRF pick failed: {exc}")
                if pending.purpose == "dooaf_setup":
                    self._dooaf_video_pick_failed(str(exc))
                elif pending.purpose == "observation":
                    self._set_status(
                        f"Impact LRF pick failed — {exc}. Retry the click."
                    )
            if slant_m is not None:
                if pending.purpose == "dooaf_setup":
                    click_uv = getattr(self, "_lrf_click_uv", None)
                    self._sync_dooaf_setup_track_from_lrf_lock(
                        pick_role,
                        final_uv=click_uv
                        if isinstance(click_uv, tuple)
                        else final_uv,
                    )
                try:
                    self._obstacle_radar.set_c13_lrf_locked(
                        slant_m,
                        geo_label=str(getattr(self, "_lrf_lock_geo_label", "") or ""),
                    )
                except Exception:
                    pass
                self._hide_lrf_video_reticle_keep_range()
                self._schedule_video_marks_overlay_refresh()
            else:
                self._lrf_lock_uv = (float(u), float(v))
                self._lrf_lock_failed = True
                try:
                    self._obstacle_radar.set_c13_lrf_lock_failed()
                except Exception:
                    pass
            self._refresh_lrf_lock_overlay()
            return
        try:
            if dist is None:
                self._lrf_lock_uv = (float(u), float(v))
                self._lrf_lock_distance_m = None
                self._lrf_lock_failed = True
                self._lrf_lock_armed = False
                self._refresh_lrf_lock_overlay()
                try:
                    self._obstacle_radar.set_c13_lrf_lock_failed()
                except Exception:
                    pass
                self._set_status(
                    "LRF lock failed — retry click or tap ◎ to re-arm"
                )
                QTimer.singleShot(2500, self._clear_lrf_failed_reticle)
                return
            dm = float(dist)
            self._lrf_lock_failed = False
            self._calibrate_lrf_track_after_lock()
            self._update_lrf_reticle_track()
            uv = getattr(self, "_lrf_lock_uv", None) or (float(u), float(v))
            self._lrf_lock_distance_m = dm
            self._update_lrf_lock_geo(dm)
            self._companion_laser_range_m = dm
            self._try_record_dooaf_facade_session(dm)
            self._obstacle_radar.set_c13_lrf_locked(
                dm, geo_label=str(getattr(self, "_lrf_lock_geo_label", "") or "")
            )
            self._refresh_lrf_lock_overlay()
            status = f"C13 LRF locked · {format_slr_display_m(dm)}"
            if getattr(self, "_lrf_lock_geo_label", ""):
                status += f" · {self._lrf_lock_geo_label}"
            self._set_status(f"{status} — cyan box on video")
            print(
                f"[VGCS:lrf] locked u={uv[0]:.3f} v={uv[1]:.3f} range={dm:.1f} m"
            )
        except Exception as exc:
            print(f"[VGCS:lrf] lock result error: {exc}")
            self._lrf_lock_uv = None
            self._lrf_lock_distance_m = None
            self._refresh_lrf_lock_overlay()
            self._set_status("LRF lock failed")
            try:
                self._obstacle_radar.set_c13_lrf_armed(False)
            except Exception:
                pass
