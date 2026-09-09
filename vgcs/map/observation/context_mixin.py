"""MapWidget mixin — see vgcs.map.observation package."""

from __future__ import annotations

from PySide6.QtCore import Qt

from vgcs.observe.dooaf import DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED
from vgcs.observe.target_measure import dem_ground_agl_m, resolve_ray_agl_for_geo, sanitize_dem_ground_agl_m


class ObservationContextMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _observation_mark_active(self) -> bool:
        """Target ON — video/minimap clicks mark fall of shot; never toggle map/video swap."""
        if self._dooaf_pick_complete is not None and bool(
            getattr(self, "_dooaf_pick_from_video", False)
        ):
            return False
        if bool(getattr(self, "_obs_mark_mode", False)):
            return True
        try:
            btn = getattr(self, "_btn_native_target", None)
            if btn is not None and btn.isChecked():
                return True
        except Exception:
            pass
        return False

    def _set_observation_mark_mode(self, enabled: bool) -> None:
        enabled = bool(enabled)
        self._obs_mark_mode = enabled
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_observation_mark_mode(bool(enabled))
        except Exception:
            pass
        if not self._map_uses_legacy_web_bridge():
            if bool(getattr(self, "_is_3d_mode", False)):
                self._run_js(
                    f"if (window.setObservationMarkMode) setObservationMarkMode({1 if enabled else 0});"
                )
        else:
            self._run_js(
                f"if (window.setObservationMarkMode) setObservationMarkMode({1 if enabled else 0});"
            )
        try:
            self._btn_native_target.blockSignals(True)
            self._btn_native_target.setChecked(bool(enabled))
        finally:
            try:
                self._btn_native_target.blockSignals(False)
            except Exception:
                pass
        try:
            if enabled:
                self._native_minimap.setToolTip(
                    "Target ON: click here to mark on map · drag to pan"
                )
                self._native_minimap.setCursor(Qt.CursorShape.CrossCursor)
                self._native_video_preview.setCursor(Qt.CursorShape.CrossCursor)
            else:
                self._native_minimap.setToolTip(
                    "Drag to pan map · click (no drag) to swap back to map"
                )
                self._native_minimap.setCursor(Qt.CursorShape.OpenHandCursor)
                self._native_video_preview.setCursor(Qt.CursorShape.ArrowCursor)
        except Exception:
            pass
        if enabled:
            # Says which point the next click sets, now that the rail can
            # choose. Telling them it is always the fall of shot was the old
            # behaviour and would now be wrong half the time.
            what = (
                "actual target"
                if self._current_observe_dooaf_role() == DOOAF_ROLE_INTENDED
                else "fall of shot"
            )
            self._set_status(
                f"Target ON: click the video or map to set the {what} — "
                "Set TGT / Set HIT on the camera rail chooses which"
            )
            try:
                if bool(getattr(self, "_video_swapped", False)):
                    self._ensure_video_pro_hud_visible()
                else:
                    self._raise_flight_hud_above_video()
            except Exception:
                pass
        else:
            self._set_status("Observation mark mode OFF")

    def _observation_context(self) -> dict[str, object]:
        gimbal_yaw = None
        gimbal_pitch = None
        st = None
        try:
            st = self._camera_control.get_gimbal_status()
            if st is not None and bool(getattr(st, "supported", False)):
                yaw = getattr(st, "yaw_deg", None)
                pitch = getattr(st, "pitch_deg", None)
                # C13/Skydroid often reports 0,0 when level — still pass through; geo
                # layer applies downward pitch when rangefinder AGL is available.
                if yaw is not None:
                    gimbal_yaw = float(yaw)
                if pitch is not None:
                    gimbal_pitch = float(pitch)
                elif yaw is not None:
                    gimbal_pitch = 0.0
        except Exception:
            pass
        v_lat = self._lat
        v_lon = self._lon
        if v_lat is None or v_lon is None:
            try:
                pos = self.get_vehicle_display_position()
                if pos is not None:
                    v_lat, v_lon = float(pos[0]), float(pos[1])
            except Exception:
                pass
        from vgcs.observe.target_measure import dem_ground_agl_m, resolve_ray_agl_for_geo

        dem_path = self._observe_dem_path()
        dem_ground, dem_ground_src = dem_ground_agl_m(
            vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            vehicle_lat=v_lat,
            vehicle_lon=v_lon,
            dem_path=dem_path,
        )
        ekf_raw = self._vehicle_rel_alt_m
        try:
            ekf_stored = float(ekf_raw) if ekf_raw is not None else None
        except (TypeError, ValueError):
            ekf_stored = None
        dem_ground = sanitize_dem_ground_agl_m(dem_ground, ekf_stored)
        agl_m, agl_src = resolve_ray_agl_for_geo(
            relative_alt_m=ekf_stored,
            rangefinder_down_m=self._rangefinder_down_m,
            vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            vehicle_lat=v_lat,
            vehicle_lon=v_lon,
            dem_path=dem_path,
        )
        return {
            "vehicle_lat": v_lat,
            "vehicle_lon": v_lon,
            "vehicle_heading_deg": self._heading,
            "vehicle_roll_deg": self._vehicle_roll_deg,
            "vehicle_pitch_deg": self._vehicle_pitch_deg,
            "ekf_rel_alt_m": ekf_stored,
            "vehicle_rel_alt_m": ekf_stored,
            "vehicle_alt_msl_m": self._vehicle_alt_msl_m,
            "dem_ground_agl_m": dem_ground,
            "dem_ground_agl_source": dem_ground_src,
            "measure_agl_m": agl_m,
            "agl_source": agl_src,
            "gimbal_yaw_deg": gimbal_yaw,
            "gimbal_pitch_deg": gimbal_pitch,
            "gps_fix_type": int(getattr(self, "_gps_fix_type", 0) or 0),
            "gps_satellites": int(getattr(self, "_gps_satellites", 0) or 0),
            "gps_hdop": self._gps_hdop,
            "rangefinder_down_m": self._rangefinder_down_m,
            "target_lat": None,
            "target_lon": None,
            "target_alt_m": None,
            "geo_quality": "",
            "geo_warning": "",
            "geo_method": "",
            "geo_range_m": None,
            "geo_bearing_deg": None,
        }

    def _current_observe_dooaf_role(self) -> str:
        """What the next video or map click marks.

        This used to be the fall of shot and nothing else, so the actual target
        could only be placed through the DOOAF Setup dialog: two different ways
        to mark two points the operator thinks of as one action. The camera
        rail now carries a Set TGT / Set HIT choice, and this reads it.

        Still defaults to the fall of shot, which is the common case and what
        every existing habit expects.
        """
        block = getattr(self, "_native_observe_body", None)
        getter = getattr(block, "current_dooaf_role", None)
        if callable(getter):
            try:
                role = str(getter() or "")
            except Exception:
                role = ""
            if role in (DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED):
                return role
        return DOOAF_ROLE_IMPACT
