"""MapWidget mixin — see vgcs.map.observation package."""

from __future__ import annotations

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtWidgets import QDialog, QMessageBox

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.dooaf_setup_dialog import (
    DOOAF_PICK_GUN,
    DOOAF_PICK_TARGET,
    DOOAF_VIDEO_PICK_FACADE_LRF,
    DOOAF_VIDEO_PICK_GROUND,
    DooafSetupDialog,
)
from vgcs.map.native_video_overlay import VideoOverlayFacadeHint
from vgcs.map.observation.types import PendingLrfVideoPick
from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_SURVEY,
    DooafSettings,
    _apply_geo_reference_to_mark_row,
    _forced_ray_geo_for_row,
    apply_dooaf_impact_geo_fallback,
    apply_map_pick_to_settings,
    build_dooaf_session,
    clear_dooaf_setup_video_mark,
    dooaf_role_display,
    dooaf_settings_kwargs,
    enrich_dooaf_settings_elevation_from_dem,
    format_dooaf_status,
    latest_mark,
    merge_dooaf_settings,
    merge_setup_video_marks,
    read_dooaf_facade_slant_range_m,
    read_dooaf_settings,
    refine_impact_geo_from_video_rays,
    resolved_dooaf_settings,
    write_dooaf_facade_slant_range_m,
    write_dooaf_settings,
    write_dooaf_setup_video_mark,
)
from vgcs.observe.dooaf_flight_session import build_facade_overlay_hint
from vgcs.observe.geo_reference import (
    apply_geo_reference_result_to_video_row,
    compute_geo_reference,
    compute_lrf_slant_geo,
)
from vgcs.observe.target_measure import (
    haversine_m,
    low_hover_ray_agl_m,
    ray_agl_suspect_dem_mismatch,
)
from vgcs.video.camera_control import NoopCameraControl


class DooafOperationsMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _dooaf_settings_store(self) -> QSettings:
        return QSettings(QS_ORG, QS_APP)

    def _resolve_facade_slant_for_session(self) -> float | None:
        session = getattr(self, "_dooaf_facade_session", None)
        slant = getattr(session, "slant_range_m", None) if session is not None else None
        if slant is not None:
            try:
                return float(slant)
            except (TypeError, ValueError):
                pass
        st = self._dooaf_settings_store()
        slant = read_dooaf_facade_slant_range_m(st)
        if slant is not None:
            return slant
        tracks = getattr(self, "_dooaf_setup_mark_track", None) or {}
        for track in tracks.values():
            if not isinstance(track, dict):
                continue
            raw = track.get("lrf_slant_range_m")
            if raw is None:
                continue
            try:
                s = float(raw)
            except (TypeError, ValueError):
                continue
            if s >= 8.0:
                return s
        for row in getattr(self, "_observations", None) or []:
            raw = row.get("lrf_slant_range_m")
            if raw is None:
                continue
            try:
                s = float(raw)
            except (TypeError, ValueError):
                continue
            if s >= 8.0:
                return s
        return None

    def _dooaf_session_kwargs(self) -> dict[str, object]:
        s = resolved_dooaf_settings(
            self._dooaf_settings_store(), self._observations
        )
        kw = dooaf_settings_kwargs(s)
        kw["dem_path"] = self._observe_dem_path()
        marks = getattr(self, "_dooaf_setup_video_marks", None) or {}
        merged = merge_setup_video_marks(marks, st=self._dooaf_settings_store())
        if merged:
            kw["setup_video_marks"] = merged
        slant = self._resolve_facade_slant_for_session()
        if slant is not None:
            kw["facade_slant_range_m"] = float(slant)
        return kw

    def _resolved_dooaf_settings(self) -> DooafSettings:
        return resolved_dooaf_settings(
            self._dooaf_settings_store(), self._observations
        )

    def _ensure_dooaf_impact_visible_on_map(self, row: dict[str, object]) -> None:
        """Pan the map toward a new fall-of-shot when geo placed it off-screen."""
        lat = row.get("target_lat")
        lon = row.get("target_lon")
        if lat is None or lon is None:
            return
        try:
            nm = getattr(self, "_native_map", None)
            if nm is None or not hasattr(nm, "set_center"):
                return
            la = float(lat)
            lo = float(lon)
            cla = float(getattr(nm, "_center_lat", la))
            clo = float(getattr(nm, "_center_lon", lo))
            dist = haversine_m(cla, clo, la, lo)
            if dist > 120.0:
                nm.set_center(la, lo)
        except Exception:
            pass

    def _refresh_dooaf_map_overlay(self) -> None:
        s = self._resolved_dooaf_settings()
        gun = (
            (float(s.gun_lat), float(s.gun_lon))
            if s.gun_lat is not None and s.gun_lon is not None
            else None
        )
        intended = (
            (float(s.target_lat), float(s.target_lon))
            if s.target_lat is not None and s.target_lon is not None
            else None
        )
        impact_mark = latest_mark(self._observations, DOOAF_ROLE_IMPACT)
        impact = (
            (impact_mark.lat, impact_mark.lon) if impact_mark is not None else None
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "set_dooaf_overlay"):
                nm.set_dooaf_overlay(gun=gun, intended=intended, impact=impact)
                try:
                    nm.update()
                except Exception:
                    pass
        except Exception:
            pass
        self._sync_3d_map_overlays()
        self._refresh_3d_marker_overlay()
        try:
            if bool(getattr(self, "_video_swapped", False)):
                QTimer.singleShot(0, self._update_native_minimap)
        except Exception:
            pass

    def _on_native_observation_map_click(self, lat: float, lon: float) -> None:
        if self._dooaf_pick_complete is not None:
            cb = self._dooaf_pick_complete
            self._end_dooaf_map_pick(restore_target_mode=True)
            cb(float(lat), float(lon))
            return
        self._log_observation("map_mark", map_lat=float(lat), map_lon=float(lon))

    def _end_dooaf_map_pick(self, *, restore_target_mode: bool = True) -> None:
        self._dooaf_pick_complete = None
        self._dooaf_pick_dialog = None
        self._dooaf_pick_from_video = False
        self._dooaf_video_pick_mode = ""
        try:
            if bool(getattr(self, "_dooaf_restore_target_after_pick", False)):
                self._dooaf_restore_target_after_pick = False
                restore_target_mode = True
                self._set_observation_mark_mode(True)
            elif restore_target_mode:
                on = bool(self._btn_native_target.isChecked())
                try:
                    self._native_video_preview.setCursor(
                        Qt.CursorShape.CrossCursor
                        if on
                        else Qt.CursorShape.ArrowCursor
                    )
                except Exception:
                    pass
        except Exception:
            pass
        if not restore_target_mode:
            return
        try:
            nm = getattr(self, "_native_map", None)
            on = bool(self._btn_native_target.isChecked())
            if nm is not None and hasattr(nm, "set_observation_mark_mode"):
                nm.set_observation_mark_mode(on)
        except Exception:
            pass

    def _preview_dooaf_overlay(self, settings: DooafSettings) -> None:
        gun = (
            (float(settings.gun_lat), float(settings.gun_lon))
            if settings.gun_lat is not None and settings.gun_lon is not None
            else None
        )
        intended = (
            (float(settings.target_lat), float(settings.target_lon))
            if settings.target_lat is not None and settings.target_lon is not None
            else None
        )
        impact_mark = latest_mark(self._observations, DOOAF_ROLE_IMPACT)
        impact = (
            (impact_mark.lat, impact_mark.lon) if impact_mark is not None else None
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "set_dooaf_overlay"):
                nm.set_dooaf_overlay(gun=gun, intended=intended, impact=impact)
        except Exception:
            pass

    def _compute_video_pick_geo(
        self, video_x: float, video_y: float
    ) -> tuple[float, float, float | None] | None:
        """Geo-reference a video click without logging an observation row."""
        row: dict[str, object] = {
            "kind": "video_mark",
            "video_x_norm": float(video_x),
            "video_y_norm": float(video_y),
        }
        row.update(self._observation_context())
        self._enrich_observation_geo_reference(row)
        lat = row.get("target_lat")
        lon = row.get("target_lon")
        if lat is None or lon is None:
            return None
        alt_raw = row.get("target_alt_m")
        alt_m: float | None = None
        if alt_raw is not None:
            try:
                alt_m = float(alt_raw)
            except (TypeError, ValueError):
                alt_m = None
        return float(lat), float(lon), alt_m

    def _complete_dooaf_setup_ground_video_pick(
        self,
        pick_role: str,
        video_x: float,
        video_y: float,
        *,
        label: str,
    ) -> bool:
        """GPS + DEM ray at click UV — mark stays where the operator clicked."""
        geo = self._compute_video_pick_geo(video_x, video_y)
        if geo is None:
            row: dict[str, object] = {
                "kind": "video_mark",
                "video_x_norm": float(video_x),
                "video_y_norm": float(video_y),
            }
            row.update(self._observation_context())
            self._enrich_observation_geo_reference(row)
            reason = str(row.get("geo_warning") or row.get("geo_quality") or "")
            self._dooaf_video_pick_failed(reason)
            return True
        lat, lon, alt_m = geo
        mark_u, mark_v = float(video_x), float(video_y)
        self._dooaf_setup_video_marks[pick_role] = (mark_u, mark_v)
        ref_att = self._read_gimbal_attitude_pair()
        self._register_dooaf_setup_mark_track(
            pick_role,
            ref_uv=(mark_u, mark_v),
            ref_att=ref_att,
            lock_att=ref_att,
            used_lrf_slew=False,
            ground_video_pick=True,
            geo_lat=float(lat),
            geo_lon=float(lon),
            geo_alt_m=alt_m,
        )
        try:
            write_dooaf_setup_video_mark(
                self._dooaf_settings_store(),
                pick_role,
                mark_u,
                mark_v,
            )
        except Exception:
            pass
        self._schedule_video_marks_overlay_refresh()
        cb = self._dooaf_pick_complete
        self._end_dooaf_map_pick(restore_target_mode=True)
        print(
            f"[VGCS:observe] dooaf ground video pick ok role={pick_role} "
            f"lat={lat:.7f} lon={lon:.7f} alt={alt_m} video=({mark_u:.3f},{mark_v:.3f})"
        )
        if callable(cb):
            try:
                cb(float(lat), float(lon), alt_m)
            except TypeError:
                cb(float(lat), float(lon))
        self._set_status(
            f"DOOAF {label} saved (ground video pick) — mark at click; OK to confirm"
        )
        return True

    def _dooaf_lrf_geo_enabled(self) -> bool:
        """True when C13 TOP LRF lock API is available (Skydroid companion)."""
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return False
        from vgcs.video.camera_control import NoopCameraControl

        if isinstance(cc, NoopCameraControl):
            return False
        return callable(getattr(cc, "lock_lrf_at_video_norm", None))

    def _apply_lrf_slant_geo_to_row(
        self,
        row: dict[str, object],
        slant_range_m: float,
        video_x: float,
        video_y: float,
        *,
        boresight_after_slew: bool = False,
    ) -> bool:
        """Place mark geo from measured C13 SLR along the post-slew look vector."""
        try:
            slant = float(slant_range_m)
        except (TypeError, ValueError):
            return False
        if slant < 0.5:
            return False
        ctx = self._observation_context()
        if ctx.get("gimbal_yaw_deg") is None:
            return False
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
            video_x_norm=0.5 if boresight_after_slew else float(video_x),
            video_y_norm=0.5 if boresight_after_slew else float(video_y),
            gps_fix_type=int(ctx.get("gps_fix_type") or 0),
            gps_hdop=ctx.get("gps_hdop"),  # type: ignore[arg-type]
            camera_hfov_deg=hfov,
            camera_vfov_deg=vfov,
        )
        if not geo.ok or geo.target_lat is None or geo.target_lon is None:
            print(
                f"[VGCS:observe] LRF geo unavailable — {geo.warning or geo.quality}"
            )
            return False
        row.update(
            {
                k: ctx[k]
                for k in (
                    "vehicle_lat",
                    "vehicle_lon",
                    "vehicle_heading_deg",
                    "vehicle_roll_deg",
                    "vehicle_pitch_deg",
                    "ekf_rel_alt_m",
                    "vehicle_rel_alt_m",
                    "vehicle_alt_msl_m",
                    "gimbal_yaw_deg",
                    "gimbal_pitch_deg",
                    "gps_fix_type",
                    "gps_satellites",
                    "gps_hdop",
                    "rangefinder_down_m",
                    "measure_agl_m",
                    "agl_source",
                    "dem_ground_agl_m",
                )
                if k in ctx
            }
        )
        apply_geo_reference_result_to_video_row(row, geo, slant_range_m=slant)
        print(
            f"[VGCS:observe] LRF geo=({float(geo.target_lat):.7f},"
            f"{float(geo.target_lon):.7f}) slant={slant:.1f} m "
            f"q={geo.quality} method={geo.method}"
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "add_geo_referenced_marker"):
                nm.add_geo_referenced_marker(
                    float(geo.target_lat), float(geo.target_lon)
                )
        except Exception:
            pass
        return True

    def _append_lrf_fallback_warning(self, row: dict[str, object], note: str) -> None:
        prev = str(row.get("geo_warning") or "").strip()
        row["geo_warning"] = f"{prev}; {note}" if prev else note

    def _begin_c13_lrf_video_lock_for_pick(self, u: float, v: float, *, label: str) -> None:
        """Start async LRF lock for a DOOAF / observation video pick."""
        self._set_status(f"{label} — slewing camera and locking LRF…")
        self._begin_c13_lrf_video_lock(float(u), float(v))

    def _complete_pending_dooaf_setup_lrf_pick(
        self,
        slant_m: float | None,
        pending: PendingLrfVideoPick,
    ) -> None:
        """Finish DOOAF Setup video pick after LRF lock (or DEM fallback)."""
        video_x, video_y = float(pending.u), float(pending.v)
        pick_role = str(pending.pick_role or DOOAF_ROLE_INTENDED)
        if slant_m is None:
            row: dict[str, object] = {
                "kind": "video_mark",
                "video_x_norm": video_x,
                "video_y_norm": video_y,
            }
            row.update(self._observation_context())
            self._enrich_observation_geo_reference(row)
            lat_fb = row.get("target_lat")
            lon_fb = row.get("target_lon")
            if lat_fb is not None and lon_fb is not None:
                print(
                    "[VGCS:observe] LRF lock failed — using DEM/ray geo estimate "
                    f"lat={float(lat_fb):.7f} lon={float(lon_fb):.7f}"
                )
                alt_raw = row.get("target_alt_m")
                alt_fb: float | None = None
                if alt_raw is not None:
                    try:
                        alt_fb = float(alt_raw)
                    except (TypeError, ValueError):
                        alt_fb = None
                self._append_lrf_fallback_warning(
                    row,
                    "LRF lock failed — position from DEM ray estimate",
                )
                mark_u, mark_v = float(video_x), float(video_y)
                self._dooaf_setup_video_marks[pick_role] = (mark_u, mark_v)
                self._register_dooaf_setup_mark_track(
                    pick_role,
                    ref_uv=(video_x, video_y),
                    ref_att=getattr(self, "_lrf_click_att", None),
                    lock_att=self._read_gimbal_attitude_pair(),
                    used_lrf_slew=False,
                    geo_lat=float(lat_fb),
                    geo_lon=float(lon_fb),
                    geo_alt_m=alt_fb,
                )
                try:
                    write_dooaf_setup_video_mark(
                        self._dooaf_settings_store(),
                        pick_role,
                        mark_u,
                        mark_v,
                    )
                except Exception:
                    pass
                self._schedule_video_marks_overlay_refresh()
                self._end_dooaf_map_pick(restore_target_mode=True)
                print(
                    f"[VGCS:observe] dooaf video pick ok (DEM fallback) "
                    f"lat={float(lat_fb):.7f} lon={float(lon_fb):.7f} "
                    f"video=({mark_u:.3f},{mark_v:.3f})"
                )
                cb = self._dooaf_pick_complete
                if callable(cb):
                    try:
                        cb(float(lat_fb), float(lon_fb), alt_fb)
                    except TypeError:
                        cb(float(lat_fb), float(lon_fb))
                self._set_status(
                    f"DOOAF {pending.label} saved (DEM estimate) — "
                    "confirm or re-pick with LRF for better accuracy"
                )
                return
            self._dooaf_video_pick_failed(
                "LRF lock failed — gimbal aimed at click but rangefinder did not "
                "confirm; retry the pick or choose a surface with a clear laser return"
            )
            return
        row: dict[str, object] = {
            "kind": "video_mark",
            "video_x_norm": video_x,
            "video_y_norm": video_y,
        }
        row.update(self._observation_context())
        used_lrf = False
        if slant_m is not None:
            used_lrf = self._apply_lrf_slant_geo_to_row(
                row,
                float(slant_m),
                video_x,
                video_y,
                boresight_after_slew=True,
            )
        if not used_lrf:
            self._enrich_observation_geo_reference(row)
            self._append_lrf_fallback_warning(
                row,
                "LRF geo failed — position from DEM ray estimate",
            )
        lat = row.get("target_lat")
        lon = row.get("target_lon")
        if lat is None or lon is None:
            reason = str(row.get("geo_warning") or row.get("geo_quality") or "")
            self._dooaf_video_pick_failed(reason)
            return
        alt_raw = row.get("target_alt_m")
        alt_m: float | None = None
        if alt_raw is not None:
            try:
                alt_m = float(alt_raw)
            except (TypeError, ValueError):
                alt_m = None
        cb = self._dooaf_pick_complete
        click_att = getattr(self, "_lrf_click_att", None)
        slew_locked = slant_m is not None and click_att is not None
        used_lrf_slew = bool(used_lrf or slew_locked)
        # Persist operator click for audit; on-screen mark uses post-slew boresight.
        mark_u, mark_v = float(video_x), float(video_y)
        display_uv: tuple[float, float] | None = None
        if used_lrf and lat is not None and lon is not None:
            proj = self._project_geo_to_video_norm(
                float(lat), float(lon), alt_m=alt_m
            )
            if proj is not None:
                display_uv = (float(proj[0]), float(proj[1]))
            else:
                display_uv = (0.5, 0.5)
            du, dv = display_uv
            if abs(float(du) - 0.5) < 0.06 and abs(float(dv) - 0.5) < 0.06:
                print(
                    f"[VGCS:observe] gun locked at frame centre ({du:.3f},{dv:.3f}) "
                    f"after slew — you clicked ({video_x:.3f},{video_y:.3f}); "
                    f"LRF range is along the laser at centre"
                )
            else:
                print(
                    f"[VGCS:observe] gun mark at ({du:.3f},{dv:.3f}) from LRF geo "
                    f"(operator click was {video_x:.3f},{video_y:.3f})"
                )
        overlay_uv = display_uv if display_uv is not None else (mark_u, mark_v)
        self._dooaf_setup_video_marks[pick_role] = overlay_uv
        self._register_dooaf_setup_mark_track(
            pick_role,
            ref_uv=(video_x, video_y),
            ref_att=click_att,
            lock_att=self._read_gimbal_attitude_pair(),
            used_lrf_slew=used_lrf_slew,
            geo_lat=float(lat),
            geo_lon=float(lon),
            geo_alt_m=alt_m,
            lrf_slant_range_m=float(slant_m) if slant_m is not None else None,
            display_uv=display_uv,
        )
        try:
            write_dooaf_setup_video_mark(
                self._dooaf_settings_store(),
                pick_role,
                float(overlay_uv[0]),
                float(overlay_uv[1]),
            )
        except Exception:
            pass
        self._schedule_video_marks_overlay_refresh()
        self._end_dooaf_map_pick(restore_target_mode=True)
        lrf_note = (
            f" LRF {float(slant_m):.1f} m"
            if used_lrf and slant_m is not None
            else " (DEM estimate)"
        )
        print(
            f"[VGCS:observe] dooaf video pick ok lat={float(lat):.7f} lon={float(lon):.7f} "
            f"alt={alt_m} video=({overlay_uv[0]:.3f},{overlay_uv[1]:.3f}) "
            f"click=({video_x:.3f},{video_y:.3f}){lrf_note}"
        )
        if callable(cb):
            try:
                cb(float(lat), float(lon), alt_m)
            except TypeError:
                cb(float(lat), float(lon))
        method = str(row.get("geo_method") or "")
        slew_note = ""
        if used_lrf and display_uv is not None and pick_role == DOOAF_ROLE_GUN:
            slew_note = " — gimbal aimed so locked point is at frame centre"
        self._set_status(
            f"DOOAF {pending.label} saved{lrf_note}{slew_note} — OK to confirm or pick again"
            + (" · lrf_slant" if method == "lrf_slant" else "")
        )

    def _dooaf_video_pick_failed(self, reason: str) -> None:
        dlg = self._dooaf_pick_dialog
        self._end_dooaf_map_pick(restore_target_mode=True)
        detail = (reason or "").strip() or "Could not compute a ground position."
        print(f"[VGCS:observe] dooaf video pick failed: {detail}")
        self._set_status(f"Video pick failed — {detail}")
        if dlg is not None:
            try:
                dlg.show()
                dlg.raise_()
                dlg.activateWindow()
            except Exception:
                pass
            try:
                QMessageBox.warning(
                    dlg,
                    "Pick on video",
                    (
                        f"{detail}\n\n"
                        "Tips:\n"
                        "• Actual target: click roof / aim point on the building.\n"
                        "• Impact Target: click ground in the lower part of the video (not sky).\n"
                        "• Wait for mavlink GPS fix (3D GPS helps).\n"
                        "• Pitch gimbal down if the scene is oblique.\n"
                        "• Pick on map if video geo is unreliable."
                    ),
                )
            except Exception:
                pass

    def _dooaf_facade_pending_pick_labels(self) -> list[str]:
        """Short labels for marks still needed in the current DOOAF session."""
        missing: list[str] = []
        s = self._resolved_dooaf_settings()
        if s.gun_lat is None or s.gun_lon is None:
            missing.append("Gun")
        if s.target_lat is None or s.target_lon is None:
            missing.append("Target")
        if latest_mark(self._observations, DOOAF_ROLE_IMPACT) is None:
            missing.append("Impact")
        return missing

    def _refresh_dooaf_facade_overlay_hint(self) -> None:
        """Show in-video banner when facade LRF lock enables rapid UV picks."""
        try:
            ly = self._native_video_overlay
        except AttributeError:
            return
        session = getattr(self, "_dooaf_facade_session", None)
        if (
            session is None
            or not session.has_lock
            or bool(getattr(self, "_lrf_lock_in_progress", False))
            or not self._dooaf_lrf_geo_enabled()
        ):
            ly.set_facade_hint(None)
            return
        ctx = self._observation_context()
        ready = bool(session.uv_pick_valid(ctx))
        text = build_facade_overlay_hint(
            slant_range_m=session.slant_range_m,
            uv_pick_ready=ready,
            pending_roles=self._dooaf_facade_pending_pick_labels(),
        )
        if text is None:
            ly.set_facade_hint(None)
            return
        title, subtitle = text
        ly.set_facade_hint(
            VideoOverlayFacadeHint(
                title=title,
                subtitle=subtitle,
                ready=ready,
            )
        )

    def _dooaf_facade_uv_pick_ready(self) -> bool:
        """True when a fast facade UV pick can reuse the last LRF lock."""
        session = getattr(self, "_dooaf_facade_session", None)
        if session is None or not session.has_lock:
            return False
        return bool(session.uv_pick_valid(self._observation_context()))

    def _geo_from_facade_uv_pick(
        self, video_x: float, video_y: float
    ) -> tuple[float, float, float | None] | None:
        session = getattr(self, "_dooaf_facade_session", None)
        if session is None or not session.has_lock:
            return None
        hfov, vfov = self._c13_lrf_geo_fov()
        geo = session.geo_from_uv(
            float(video_x),
            float(video_y),
            hfov_deg=hfov,
            vfov_deg=vfov,
            ctx=self._observation_context(),
        )
        if not geo.ok or geo.target_lat is None or geo.target_lon is None:
            return None
        alt: float | None = None
        if geo.target_alt_m is not None:
            try:
                alt = float(geo.target_alt_m)
            except (TypeError, ValueError):
                alt = None
        return float(geo.target_lat), float(geo.target_lon), alt

    def _facade_lock_gimbal_att(self) -> tuple[float, float] | None:
        session = getattr(self, "_dooaf_facade_session", None)
        lock = getattr(session, "_lock", None) if session is not None else None
        if lock is None:
            return self._read_gimbal_attitude_pair()
        return (float(lock.gimbal_yaw_deg), float(lock.gimbal_pitch_deg))

    def _complete_dooaf_setup_facade_uv_pick(
        self,
        pick_role: str,
        video_x: float,
        video_y: float,
        *,
        label: str,
    ) -> bool:
        """Place a DOOAF setup mark from facade session (no LRF slew)."""
        geo = self._geo_from_facade_uv_pick(video_x, video_y)
        if geo is None:
            return False
        lat, lon, alt_m = geo
        slant = getattr(self._dooaf_facade_session, "slant_range_m", None)
        lock_att = self._facade_lock_gimbal_att()
        mark_u, mark_v = float(video_x), float(video_y)
        self._dooaf_setup_video_marks[pick_role] = (mark_u, mark_v)
        self._register_dooaf_setup_mark_track(
            pick_role,
            ref_uv=(mark_u, mark_v),
            ref_att=lock_att,
            lock_att=lock_att,
            used_lrf_slew=False,
            facade_uv_pick=True,
            geo_lat=float(lat),
            geo_lon=float(lon),
            geo_alt_m=alt_m,
            lrf_slant_range_m=float(slant) if slant is not None else None,
        )
        try:
            write_dooaf_setup_video_mark(
                self._dooaf_settings_store(),
                pick_role,
                mark_u,
                mark_v,
            )
        except Exception:
            pass
        self._schedule_video_marks_overlay_refresh()
        self._refresh_dooaf_facade_overlay_hint()
        cb = self._dooaf_pick_complete
        slant_note = f" facade LRF {float(slant):.1f} m" if slant is not None else ""
        print(
            f"[VGCS:observe] dooaf facade uv pick ok role={pick_role} "
            f"lat={lat:.7f} lon={lon:.7f} video=({mark_u:.3f},{mark_v:.3f}){slant_note}"
        )
        self._end_dooaf_map_pick(restore_target_mode=True)
        if cb is not None:
            try:
                cb(float(lat), float(lon), alt_m)
            except TypeError:
                cb(float(lat), float(lon))
        self._set_status(
            f"DOOAF {label} saved (facade pick){slant_note} — OK to confirm"
        )
        return True

    def _capture_lrf_lock_start_vehicle_pose(self) -> None:
        ctx = self._observation_context()
        vlat = ctx.get("vehicle_lat")
        vlon = ctx.get("vehicle_lon")
        if vlat is None or vlon is None:
            self._lrf_lock_start_vehicle_lat = None
            self._lrf_lock_start_vehicle_lon = None
            return
        try:
            self._lrf_lock_start_vehicle_lat = float(vlat)
            self._lrf_lock_start_vehicle_lon = float(vlon)
        except (TypeError, ValueError):
            self._lrf_lock_start_vehicle_lat = None
            self._lrf_lock_start_vehicle_lon = None

    def _vehicle_shift_during_lrf_lock_m(self) -> float | None:
        slat = getattr(self, "_lrf_lock_start_vehicle_lat", None)
        slon = getattr(self, "_lrf_lock_start_vehicle_lon", None)
        if slat is None or slon is None:
            return None
        ctx = self._observation_context()
        clat = ctx.get("vehicle_lat")
        clon = ctx.get("vehicle_lon")
        if clat is None or clon is None:
            return None
        try:
            return float(
                self._haversine_m(float(slat), float(slon), float(clat), float(clon))
            )
        except (TypeError, ValueError):
            return None

    def _try_record_dooaf_facade_session(
        self,
        slant_m: float,
        *,
        max_shift_m: float = 4.0,
    ) -> bool:
        """Store shared facade lock for rapid UV picks if the aircraft stayed still."""
        shift = self._vehicle_shift_during_lrf_lock_m()
        if shift is not None and shift > float(max_shift_m):
            print(
                f"[VGCS:observe] facade session skipped — vehicle moved "
                f"{shift:.1f} m during LRF lock (max {float(max_shift_m):.1f} m)"
            )
            self._set_status(
                f"LRF locked — vehicle moved {shift:.1f} m during lock; "
                "TARGET/IMPACT may need map pick or re-lock GUN for fast video picks"
            )
            self._refresh_dooaf_facade_overlay_hint()
            return False
        try:
            self._dooaf_facade_session.record_from_context(
                float(slant_m), self._observation_context()
            )
            write_dooaf_facade_slant_range_m(
                self._dooaf_settings_store(), float(slant_m)
            )
            print(
                f"[VGCS:observe] facade session lock slant={float(slant_m):.1f} m "
                f"(rapid UV picks enabled for nearby marks)"
            )
            self._refresh_dooaf_facade_overlay_after_change()
            return True
        except Exception:
            return False

    def _refresh_dooaf_facade_overlay_after_change(self) -> None:
        """Update facade banner and status after session record or UV pick."""
        self._refresh_dooaf_facade_overlay_hint()
        self._schedule_video_marks_overlay_refresh()

    def _handle_dooaf_video_pick(self, video_x: float, video_y: float) -> bool:
        if self._dooaf_pick_complete is None or not bool(
            getattr(self, "_dooaf_pick_from_video", False)
        ):
            return False
        if self._lrf_lock_in_progress or self._pending_lrf_video_pick is not None:
            self._set_status("LRF lock in progress — wait for range to confirm…")
            return True
        pick_role = str(getattr(self, "_dooaf_pick_role", "") or DOOAF_ROLE_INTENDED)
        label = dooaf_role_display(pick_role)
        mode = str(
            getattr(self, "_dooaf_video_pick_mode", "") or DOOAF_VIDEO_PICK_GROUND
        )
        if (
            self._dooaf_lrf_geo_enabled()
            and pick_role != DOOAF_ROLE_GUN
            and self._dooaf_facade_uv_pick_ready()
        ):
            if self._complete_dooaf_setup_facade_uv_pick(
                pick_role, float(video_x), float(video_y), label=label
            ):
                return True
            print(
                "[VGCS:observe] facade uv pick geo failed — "
                "falling back to ground video pick"
            )
        use_facade_lrf = (
            mode == DOOAF_VIDEO_PICK_FACADE_LRF
            and self._dooaf_lrf_geo_enabled()
            and pick_role == DOOAF_ROLE_GUN
        )
        if not use_facade_lrf:
            return self._complete_dooaf_setup_ground_video_pick(
                pick_role,
                float(video_x),
                float(video_y),
                label=label,
            )
        self._dooaf_setup_video_marks[pick_role] = (
            float(video_x),
            float(video_y),
        )
        ref_att = self._read_gimbal_attitude_pair()
        if ref_att is not None:
            self._dooaf_setup_mark_track[pick_role] = {
                "ref_uv": (float(video_x), float(video_y)),
                "ref_att": (float(ref_att[0]), float(ref_att[1])),
                "h_scale": 1.0,
                "v_scale": 1.0,
                "lrf_slew": True,
            }
        else:
            self._dooaf_setup_mark_track.pop(pick_role, None)
        self._sync_video_mark_track_timer()
        self._schedule_video_marks_overlay_refresh()
        self._pending_lrf_video_pick = PendingLrfVideoPick(
            purpose="dooaf_setup",
            u=float(video_x),
            v=float(video_y),
            label=label,
            pick_role=pick_role,
        )
        self._begin_c13_lrf_video_lock_for_pick(
            float(video_x), float(video_y), label=label
        )
        return True

    def _prepare_dooaf_video_pick(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "set_observation_mark_mode"):
                nm.set_observation_mark_mode(False)
        except Exception:
            pass
        if not bool(getattr(self, "_web_ready", False)):
            return
        if bool(getattr(self, "_video_preview_enabled", False)):
            if not bool(getattr(self, "_video_swapped", False)):
                self._video_swapped = True
                self._video_swap_user_map_main = False
                self._refresh_native_overlay_insets()
            try:
                self._native_video_preview.setCursor(Qt.CursorShape.CrossCursor)
            except Exception:
                pass
            # Keep fullscreen / swapped preview — do not restart PiP (that shrinks video).
            try:
                self._layout_native_video_preview()
                self._schedule_native_minimap_refresh()
                self._stack_native_overlays_above_tile_map()
            except Exception:
                pass
            return
        try:
            self._auto_start_mini_video_pip(preserve_layout=True)
        except Exception:
            pass

    def _begin_dooaf_pick(
        self,
        role: str,
        dlg: DooafSetupDialog,
        *,
        from_video: bool,
        video_mode: str = DOOAF_VIDEO_PICK_GROUND,
    ) -> None:
        if from_video and not self._gps_available_for_geo_pick():
            QMessageBox.warning(
                dlg,
                "GPS unavailable",
                "GPS is not available (need 3D fix and vehicle position).\n\n"
                "You cannot pick coordinates on video until GPS is ready.",
            )
            return
        self._end_dooaf_map_pick(restore_target_mode=True)
        self._dooaf_pick_dialog = dlg
        pick_role = DOOAF_ROLE_GUN if role == DOOAF_PICK_GUN else DOOAF_ROLE_INTENDED
        self._dooaf_pick_role = pick_role
        label = dooaf_role_display(pick_role)
        source = "video" if from_video else "map"

        def on_picked(
            lat: float,
            lon: float,
            alt_m: float | None = None,
        ) -> None:
            pick_alt = alt_m
            if pick_alt is None:
                pick_alt = self._dem_elevation_at(float(lat), float(lon))
            st = self._dooaf_settings_store()
            merged = apply_map_pick_to_settings(
                self._resolved_dooaf_settings(),
                pick_role,
                float(lat),
                float(lon),
                alt_m=pick_alt,
            )
            merged = enrich_dooaf_settings_elevation_from_dem(
                merged, self._observe_dem_path()
            )
            write_dooaf_settings(st, merged)
            if merged.gun_lat is not None and merged.gun_lon is not None:
                dlg.set_point_coords(
                    DOOAF_PICK_GUN,
                    float(merged.gun_lat),
                    float(merged.gun_lon),
                    alt_m=merged.gun_alt_m,
                )
            if merged.target_lat is not None and merged.target_lon is not None:
                dlg.set_point_coords(
                    DOOAF_PICK_TARGET,
                    float(merged.target_lat),
                    float(merged.target_lon),
                    alt_m=merged.target_alt_m,
                )
            self._refresh_dooaf_map_overlay()
            dlg.show()
            dlg.raise_()
            dlg.activateWindow()
            show_alt = (
                merged.target_alt_m
                if pick_role == DOOAF_ROLE_INTENDED
                else merged.gun_alt_m
            )
            if show_alt is None:
                show_alt = pick_alt
            alt_note = f", alt {show_alt:.1f} m (DEM)" if show_alt is not None else ""
            self._set_status(
                f"DOOAF {label} saved from {source}{alt_note} — OK to confirm or pick again"
            )

        self._dooaf_pick_complete = on_picked
        self._dooaf_pick_from_video = from_video
        self._dooaf_video_pick_mode = (
            str(video_mode) if from_video else ""
        )
        if from_video:
            self._dooaf_restore_target_after_pick = False
            try:
                btn = getattr(self, "_btn_native_target", None)
                if btn is not None and btn.isChecked():
                    self._dooaf_restore_target_after_pick = True
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    self._obs_mark_mode = False
            except Exception:
                pass
        dlg.hide()
        if from_video:
            self._prepare_dooaf_video_pick()
            print(
                f"[VGCS:observe] dooaf video pick started role={role} "
                f"mode={self._dooaf_video_pick_mode} "
                f"(Target paused={self._dooaf_restore_target_after_pick})"
            )
            if self._dooaf_video_pick_mode == DOOAF_VIDEO_PICK_FACADE_LRF:
                if self._dooaf_lrf_geo_enabled():
                    self._set_status(
                        f"Click a point on the building face for {label} — "
                        "camera will slew to centre and one LRF lock starts "
                        "facade session for fast TARGET picks"
                    )
                else:
                    self._set_status(
                        f"LRF unavailable — use Pick on video (ground) or map for {label}"
                    )
            elif self._dooaf_lrf_geo_enabled() and self._dooaf_facade_uv_pick_ready():
                slant = getattr(
                    getattr(self, "_dooaf_facade_session", None),
                    "slant_range_m",
                    None,
                )
                slant_note = (
                    f" (LRF {float(slant):.1f} m)" if slant is not None else ""
                )
                self._set_status(
                    f"Click {label} on the same building face{slant_note} — "
                    "fast pick reuses facade lock (no gimbal slew)"
                )
            else:
                self._set_status(
                    f"Click {label} on video — mark stays at your click "
                    "(GPS + DEM ray; open ground / hills)"
                )
        else:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "set_observation_mark_mode"):
                    nm.set_observation_mark_mode(True)
            except Exception:
                pass
            self._set_status(f"Click the map to set {label}")

    def _begin_dooaf_map_pick(self, role: str, dlg: DooafSetupDialog) -> None:
        self._begin_dooaf_pick(role, dlg, from_video=False)

    def _begin_dooaf_video_pick(self, role: str, dlg: DooafSetupDialog) -> None:
        self._begin_dooaf_pick(
            role, dlg, from_video=True, video_mode=DOOAF_VIDEO_PICK_GROUND
        )

    def _begin_dooaf_facade_lrf_video_pick(
        self, role: str, dlg: DooafSetupDialog
    ) -> None:
        self._begin_dooaf_pick(
            role, dlg, from_video=True, video_mode=DOOAF_VIDEO_PICK_FACADE_LRF
        )

    def _sync_dooaf_settings_from_dialog(
        self,
        dlg: DooafSetupDialog,
        *,
        gun: bool = True,
        target: bool = True,
    ) -> None:
        partial = dlg.result_settings()
        cur = read_dooaf_settings(self._dooaf_settings_store())
        write_dooaf_settings(
            self._dooaf_settings_store(),
            DooafSettings(
                gun_lat=partial.gun_lat if gun else cur.gun_lat,
                gun_lon=partial.gun_lon if gun else cur.gun_lon,
                gun_alt_m=partial.gun_alt_m if gun else cur.gun_alt_m,
                target_lat=partial.target_lat if target else cur.target_lat,
                target_lon=partial.target_lon if target else cur.target_lon,
                target_alt_m=partial.target_alt_m if target else cur.target_alt_m,
            ),
        )

    def _on_dooaf_setup_coordinates_changed(
        self, scope: str, dlg: DooafSetupDialog
    ) -> None:
        clear_gun = scope in ("gun", "all")
        clear_target = scope in ("target", "all")
        roles_remove: set[str] = set()
        if clear_gun:
            roles_remove.add(DOOAF_ROLE_GUN)
            self._dooaf_setup_video_marks.pop(DOOAF_ROLE_GUN, None)
            self._dooaf_setup_mark_track.pop(DOOAF_ROLE_GUN, None)
            try:
                clear_dooaf_setup_video_mark(
                    self._dooaf_settings_store(), DOOAF_ROLE_GUN
                )
            except Exception:
                pass
        if clear_target:
            roles_remove.add(DOOAF_ROLE_INTENDED)
            self._dooaf_setup_video_marks.pop(DOOAF_ROLE_INTENDED, None)
            self._dooaf_setup_mark_track.pop(DOOAF_ROLE_INTENDED, None)
            try:
                clear_dooaf_setup_video_mark(
                    self._dooaf_settings_store(), DOOAF_ROLE_INTENDED
                )
            except Exception:
                pass
        if roles_remove:
            self._observations = [
                r
                for r in self._observations
                if str(r.get("dooaf_role") or DOOAF_ROLE_SURVEY) not in roles_remove
            ]
            self._rebuild_observation_map_markers()
        self._sync_dooaf_settings_from_dialog(
            dlg, gun=clear_gun, target=clear_target
        )
        self._refresh_dooaf_map_overlay()
        if clear_gun or clear_target:
            self._flush_video_marks_overlay()
        self._sync_video_mark_track_timer()

    def _show_dooaf_setup_dialog(self) -> None:
        pending = self._dooaf_pick_dialog
        if pending is not None:
            self._end_dooaf_map_pick(restore_target_mode=True)
            try:
                pending.show()
                pending.raise_()
                pending.activateWindow()
            except Exception:
                pass
            return
        st = self._dooaf_settings_store()
        dlg = DooafSetupDialog(self, settings=self._resolved_dooaf_settings())
        dlg.pick_point_requested.connect(
            lambda role: self._begin_dooaf_map_pick(role, dlg)
        )
        dlg.pick_video_requested.connect(
            lambda role: self._begin_dooaf_video_pick(role, dlg)
        )
        dlg.pick_video_facade_lrf_requested.connect(
            lambda role: self._begin_dooaf_facade_lrf_video_pick(role, dlg)
        )
        dlg.coordinates_changed.connect(
            lambda scope: self._on_dooaf_setup_coordinates_changed(scope, dlg)
        )
        dlg.finished.connect(lambda _code: self._end_dooaf_map_pick())
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        saved_settings = dlg.result_settings()
        QTimer.singleShot(0, lambda: self._commit_dooaf_setup_dialog(saved_settings))

    def _commit_dooaf_setup_dialog(self, partial: DooafSettings) -> None:
        """Apply DOOAF Setup after the modal closes (keeps OK responsive)."""
        st = self._dooaf_settings_store()
        new_settings = merge_dooaf_settings(
            self._resolved_dooaf_settings(), partial
        )
        new_settings = enrich_dooaf_settings_elevation_from_dem(
            new_settings, self._observe_dem_path()
        )
        write_dooaf_settings(st, new_settings)
        self._hide_lrf_video_reticle_keep_range()
        self._refresh_dooaf_map_overlay()
        self._schedule_video_marks_overlay_refresh()
        session = build_dooaf_session(self._observations, **self._dooaf_session_kwargs())
        self._set_status(f"DOOAF setup saved — {format_dooaf_status(session)}")
        try:
            pts: list[tuple[float, float]] = []
            if session.gun is not None:
                pts.append((session.gun.lat, session.gun.lon))
            if session.intended is not None:
                pts.append((session.intended.lat, session.intended.lon))
            if pts:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "set_center"):
                    la = sum(p[0] for p in pts) / len(pts)
                    lo = sum(p[1] for p in pts) / len(pts)
                    nm.set_center(la, lo)
        except Exception:
            pass

    def _observe_dem_path(self) -> str | None:
        _, dem_path, _ = self._m8_geo_settings()
        return dem_path

    def _dem_elevation_at(self, lat: float, lon: float) -> float | None:
        path = self._observe_dem_path()
        if not path:
            return None
        try:
            from vgcs.observe.dem import elevation_at_wgs84

            return elevation_at_wgs84(float(lat), float(lon), path)
        except Exception:
            return None

    def _m8_geo_settings(self) -> tuple[float, str | None, bool]:
        st = QSettings(QS_ORG, QS_APP)
        try:
            hfov = float(st.value("observe/camera_hfov_deg", 62.0) or 62.0)
        except Exception:
            hfov = 62.0
        dem = (
            str(st.value("observe/dem_path", "") or st.value("observe/dem_csv", "") or "")
            .strip()
            or None
        )
        raw_t = st.value("observe/dem_terrain_enabled", True)
        if isinstance(raw_t, bool):
            dem_terrain = raw_t
        else:
            dem_terrain = str(raw_t or "true").strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
        return hfov, dem, dem_terrain

    def _enrich_observation_geo_reference(self, row: dict[str, object]) -> None:
        """M8 — compute ground lat/lon for video marks; copy map coords for map marks."""
        kind = str(row.get("kind") or "")
        if kind == "map_mark":
            row["target_lat"] = row.get("map_lat")
            row["target_lon"] = row.get("map_lon")
            row["geo_quality"] = "map_direct"
            row["geo_method"] = "map_click"
            return
        if kind != "video_mark":
            return
        vx = row.get("video_x_norm")
        vy = row.get("video_y_norm")
        if vx is None or vy is None:
            row["geo_quality"] = "insufficient"
            row["geo_warning"] = "video click missing"
            return
        hfov, dem_path, dem_terrain = self._m8_geo_settings()
        from vgcs.observe.target_measure import resolve_ray_agl_for_geo

        ray_agl, ray_src = resolve_ray_agl_for_geo(
            relative_alt_m=row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
            rangefinder_down_m=row.get("rangefinder_down_m"),  # type: ignore[arg-type]
            video_y_norm=row.get("video_y_norm"),  # type: ignore[arg-type]
            vehicle_alt_msl_m=row.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
            vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
            vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
            dem_path=dem_path,
        )
        row["measure_agl_m"] = ray_agl
        row["geo_agl_source"] = ray_src
        row["agl_source"] = ray_src
        geo = compute_geo_reference(
            vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
            vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
            vehicle_heading_deg=row.get("vehicle_heading_deg"),  # type: ignore[arg-type]
            vehicle_roll_deg=row.get("vehicle_roll_deg"),  # type: ignore[arg-type]
            vehicle_pitch_deg=row.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
            vehicle_rel_alt_m=row.get("ekf_rel_alt_m"),  # type: ignore[arg-type]
            vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            rangefinder_down_m=row.get("rangefinder_down_m"),  # type: ignore[arg-type]
            gimbal_yaw_deg=row.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
            gimbal_pitch_deg=row.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
            video_x_norm=float(vx),
            video_y_norm=float(vy),
            gps_fix_type=int(row.get("gps_fix_type") or 0),
            gps_hdop=row.get("gps_hdop"),  # type: ignore[arg-type]
            camera_hfov_deg=hfov,
            dem_path=dem_path,
            dem_terrain=dem_terrain,
        )
        row["target_lat"] = geo.target_lat
        row["target_lon"] = geo.target_lon
        row["target_alt_m"] = geo.target_alt_m
        row["geo_quality"] = geo.quality
        row["geo_warning"] = geo.warning
        row["geo_method"] = geo.method
        from vgcs.observe.target_measure import is_plausible_ground_range

        q = str(geo.quality or "")
        ray_for_plaus = ray_agl
        long_range_ray = ray_src == "rangefinder_clamped_long" or str(
            geo.method or ""
        ).startswith("ray_slant_long")
        if geo.depression_deg is not None:
            row["geo_depression_deg"] = geo.depression_deg
        else:
            row["geo_depression_deg"] = None
        if (
            geo.horizontal_range_m is not None
            and geo.bearing_deg is not None
            and geo.depression_deg is not None
            and ray_for_plaus is not None
            and (
                long_range_ray
                or is_plausible_ground_range(
                    float(ray_for_plaus),
                    float(geo.horizontal_range_m),
                    float(geo.depression_deg),
                )
            )
        ):
            row["geo_range_m"] = geo.horizontal_range_m
            row["geo_bearing_deg"] = geo.bearing_deg
        else:
            row["geo_range_m"] = None
            row["geo_bearing_deg"] = None
        ekf = row.get("ekf_rel_alt_m")
        needs_ray_retry = (
            row.get("target_lat") is None
            or row.get("target_lon") is None
            or str(geo.quality or "") == "insufficient"
            or not geo.ok
            or ray_agl_suspect_dem_mismatch(ray_agl, ray_src, ekf)
        )
        if needs_ray_retry:
            retry_agl = low_hover_ray_agl_m(ekf)  # type: ignore[arg-type]
            retry_geo = _forced_ray_geo_for_row(
                row,
                dem_path=dem_path,
                camera_hfov_deg=hfov,
                vehicle_alt_msl_m=self._vehicle_alt_msl_m,
                force_agl_m=retry_agl,
            )
            if (
                retry_geo is not None
                and retry_geo.ok
                and retry_geo.target_lat is not None
                and retry_geo.target_lon is not None
            ):
                _apply_geo_reference_to_mark_row(
                    row,
                    retry_geo,
                    ray_agl=retry_agl,
                    ray_src="ray_facade_retry",
                )
                row["geo_quality"] = "fair"
                row["geo_method"] = "ray_facade_retry"
                warn = str(retry_geo.warning or "")
                if ekf is not None:
                    try:
                        if float(ekf) < 2.5:
                            warn = (
                                f"low-hover ray retry (EKF {float(ekf):.1f} m — "
                                "hover ≥3 m for better accuracy)"
                            )
                    except (TypeError, ValueError):
                        pass
                row["geo_warning"] = warn or "geo from low-hover ray retry"
        from vgcs.observe.geo_reference import enrich_video_mark_target_altitude

        enrich_video_mark_target_altitude(row)
        if row.get("target_lat") is None or row.get("target_lon") is None:
            row["target_lat"] = None
            row["target_lon"] = None
        elif row.get("target_lat") is not None and row.get("target_lon") is not None:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "add_geo_referenced_marker"):
                    nm.add_geo_referenced_marker(
                        float(row["target_lat"]), float(row["target_lon"])
                    )
            except Exception:
                pass
        if (
            str(row.get("dooaf_role") or "") == DOOAF_ROLE_IMPACT
            and observation_target_latlon(row) is not None
        ):
            rs = self._resolved_dooaf_settings()
            marks = getattr(self, "_dooaf_setup_video_marks", None) or {}
            merged = merge_setup_video_marks(marks, st=self._dooaf_settings_store())
            if refine_impact_geo_from_video_rays(
                row,
                target_lat=rs.target_lat,
                target_lon=rs.target_lon,
                setup_video_marks=merged,
                dem_path=dem_path,
                camera_hfov_deg=hfov,
                vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            ):
                try:
                    nm = getattr(self, "_native_map", None)
                    if nm is not None and hasattr(nm, "add_geo_referenced_marker"):
                        la = row.get("target_lat")
                        lo = row.get("target_lon")
                        if la is not None and lo is not None:
                            nm.add_geo_referenced_marker(float(la), float(lo))
                except Exception:
                    pass
        if (
            str(row.get("dooaf_role") or "") == DOOAF_ROLE_IMPACT
            and observation_target_latlon(row) is None
        ):
            rs = self._resolved_dooaf_settings()
            marks = getattr(self, "_dooaf_setup_video_marks", None) or {}
            merged = merge_setup_video_marks(marks, st=self._dooaf_settings_store())
            if apply_dooaf_impact_geo_fallback(
                row,
                target_lat=rs.target_lat,
                target_lon=rs.target_lon,
                setup_video_marks=merged,
                dem_path=self._observe_dem_path(),
                camera_hfov_deg=hfov,
                vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            ):
                try:
                    nm = getattr(self, "_native_map", None)
                    if nm is not None and hasattr(nm, "add_geo_referenced_marker"):
                        la = row.get("target_lat")
                        lo = row.get("target_lon")
                        if la is not None and lo is not None:
                            nm.add_geo_referenced_marker(float(la), float(lo))
                except Exception:
                    pass
