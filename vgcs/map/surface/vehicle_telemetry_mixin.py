"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import json
import math
import time

from PySide6.QtCore import QTimer

from vgcs.map.surface.constants import (
    _MAP_MOVE_ARM_SAMPLES,
    _MAP_MOVE_ARM_SPEED_MPS,
    _MAP_MOVE_DISARM_SAMPLES,
    _MAP_MOVE_DISARM_SPEED_MPS,
    _MAP_POSITION_MIN_MOVE_M,
)


class VehicleTelemetryMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _gps_available_for_geo_pick(self) -> bool:
        fix = int(getattr(self, "_gps_fix_type", 0) or 0)
        if fix < 3:
            return False
        lat = self._lat
        lon = self._lon
        if lat is None or lon is None:
            try:
                pos = self.get_vehicle_display_position()
                if pos is not None:
                    lat, lon = float(pos[0]), float(pos[1])
            except Exception:
                lat, lon = None, None
        return lat is not None and lon is not None

    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6_371_000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(min(1.0, math.sqrt(max(0.0, a))))

    def _update_map_motion_state(self, groundspeed_mps: float) -> None:
        gs = max(0.0, float(groundspeed_mps))
        self._last_groundspeed_mps = gs
        if gs >= _MAP_MOVE_ARM_SPEED_MPS:
            self._map_speed_hi_streak = int(getattr(self, "_map_speed_hi_streak", 0)) + 1
            self._map_speed_lo_streak = 0
        elif gs < _MAP_MOVE_DISARM_SPEED_MPS:
            self._map_speed_lo_streak = int(getattr(self, "_map_speed_lo_streak", 0)) + 1
            self._map_speed_hi_streak = 0
        if not bool(getattr(self, "_map_motion_armed", False)):
            if self._map_speed_hi_streak >= _MAP_MOVE_ARM_SAMPLES:
                self._map_motion_armed = True
        elif self._map_speed_lo_streak >= _MAP_MOVE_DISARM_SAMPLES:
            was_armed = bool(getattr(self, "_map_motion_armed", False))
            self._map_motion_armed = False
            self._map_speed_hi_streak = 0
            if was_armed:
                self._apply_map_vehicle_heading()

    def _apply_map_vehicle_heading(self) -> None:
        """Push stored heading to native map / legacy 3D JS (independent of GPS motion lock)."""
        if self._heading is None:
            return
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.set_heading(self._heading)
            except Exception:
                pass
        self._schedule_vehicle_pose_js(immediate=False)

    def set_vehicle_position(
        self,
        lat: float,
        lon: float,
        *,
        relative_alt_m: float | None = None,
        groundspeed_mps: float | None = None,
    ) -> None:
        first_fix = self._lat is None or self._lon is None
        self._lat = lat
        self._lon = lon
        if groundspeed_mps is not None:
            self._update_map_motion_state(float(groundspeed_mps))
        gs = float(self._last_groundspeed_mps)
        if relative_alt_m is not None:
            try:
                self._vehicle_rel_alt_m = float(relative_alt_m)
            except Exception:
                pass
        if relative_alt_m is None:
            self._coords.setText(f"Lat/Lon: {lat:.7f}, {lon:.7f}")
        else:
            self._coords.setText(
                f"Lat/Lon: {lat:.7f}, {lon:.7f}  |  Rel Alt: {relative_alt_m:.1f} m"
            )
        raw_lat, raw_lon = float(lat), float(lon)
        map_moved = False
        append_track = False
        if first_fix:
            self._map_display_lat = raw_lat
            self._map_display_lon = raw_lon
            self._map_motion_armed = False
            self._map_speed_hi_streak = 0
            self._map_speed_lo_streak = 0
            map_moved = True
        elif not bool(getattr(self, "_map_motion_armed", False)):
            # Hard lock while parked: ignore all GPS drift until sustained real movement.
            if self._map_display_lat is None or self._map_display_lon is None:
                self._map_display_lat = raw_lat
                self._map_display_lon = raw_lon
                map_moved = True
        elif self._map_display_lat is not None and self._map_display_lon is not None:
            shift_m = self._haversine_m(
                self._map_display_lat, self._map_display_lon, raw_lat, raw_lon
            )
            if shift_m >= _MAP_POSITION_MIN_MOVE_M:
                self._map_display_lat = raw_lat
                self._map_display_lon = raw_lon
                map_moved = True
                append_track = True
        else:
            self._map_display_lat = raw_lat
            self._map_display_lon = raw_lon
            map_moved = True
        display_lat = float(self._map_display_lat if self._map_display_lat is not None else raw_lat)
        display_lon = float(self._map_display_lon if self._map_display_lon is not None else raw_lon)
        nm = getattr(self, "_native_map", None)
        if nm is not None and (map_moved or first_fix):
            try:
                nm.set_vehicle_filtered(
                    display_lat,
                    display_lon,
                    append_track=bool(append_track and not first_fix),
                )
            except Exception:
                pass
            if bool(getattr(self, "_video_follow_enabled", False)) and map_moved:
                try:
                    now = time.monotonic()
                    last = float(getattr(self, "_video_follow_last_center_mono", 0.0))
                    if first_fix or now - last >= 0.25:
                        self._video_follow_last_center_mono = now
                        self.center_on_vehicle()
                except Exception:
                    pass
            elif first_fix:
                try:
                    nm.set_center(float(lat), float(lon))
                except Exception:
                    pass
        try:
            if self._native_minimap_wrap.isVisible():
                self._schedule_native_minimap_refresh()
        except Exception:
            pass
        if map_moved or first_fix:
            self._schedule_vehicle_pose_js(immediate=first_fix)
            if bool(getattr(self, "_is_3d_mode", False)):
                self._refresh_3d_marker_overlay()
        if self._video_mark_tracking_active():
            self._refresh_tracked_video_marks_light()

    def set_vehicle_attitude(
        self,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        *,
        yaw_deg: float | None = None,
    ) -> None:
        """M8 — body roll/pitch from MAVLink ATTITUDE (optional yaw override)."""
        if roll_deg is not None:
            try:
                self._vehicle_roll_deg = float(roll_deg)
            except Exception:
                pass
        if pitch_deg is not None:
            try:
                self._vehicle_pitch_deg = float(pitch_deg)
            except Exception:
                pass
        if yaw_deg is not None:
            self.set_vehicle_heading(float(yaw_deg), source="att")
        elif roll_deg is not None or pitch_deg is not None:
            if self._video_mark_tracking_active():
                self._refresh_tracked_video_marks_light()

    def set_vehicle_alt_msl(self, alt_msl_m: float | None) -> None:
        if alt_msl_m is None:
            return
        try:
            self._vehicle_alt_msl_m = float(alt_msl_m)
        except Exception:
            pass

    def set_gps_hdop(self, hdop: float | None) -> None:
        if hdop is None:
            self._gps_hdop = None
            return
        try:
            self._gps_hdop = float(hdop)
        except Exception:
            self._gps_hdop = None

    def set_vehicle_heading(self, heading_deg: float, *, source: str = "mixed") -> None:
        """Store heading for HUD/observations and rotate the map vehicle icon."""
        src = str(source or "mixed")
        self._heading = float(heading_deg) % 360.0
        self._heading_js_source = src
        self._heading_label.setText(f"Heading: {self._heading:.1f}°")
        try:
            self._native_compass.set_heading_deg(self._heading)
        except Exception:
            pass
        try:
            self._obstacle_radar.set_vehicle_heading_deg(self._heading)
        except Exception:
            pass
        self._apply_map_vehicle_heading()
        if self._video_mark_tracking_active():
            self._refresh_tracked_video_marks_light()

    def clear_flight_track(self) -> None:
        """Clear the orange breadcrumb trail (e.g. on reconnect / disconnect)."""
        self._map_display_lat = None
        self._map_display_lon = None
        self._last_groundspeed_mps = 0.0
        self._map_motion_armed = False
        self._map_speed_hi_streak = 0
        self._map_speed_lo_streak = 0
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.clear_track()
            except Exception:
                pass
        self._run_js("clearFlightTrack();")

    def set_obstacle_distance(self, payload: dict) -> None:
        """M9 — forward OBSTACLE_DISTANCE (LiDAR / proximity radar bins) to the radar panel."""
        try:
            self._obstacle_radar.set_obstacle_distance(payload)
            if bool(getattr(self, "_last_link_connected", False)) and bool(
                getattr(self, "_web_ready", False)
            ):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def set_distance_sensor(self, payload: dict) -> None:
        """M9 — forward DISTANCE_SENSOR (rangefinder) to the radar panel."""
        try:
            ori = int(payload.get("orientation", 0) or 0)
            cur = payload.get("current_distance_m")
            if cur is not None and is_downward_sensor_orientation(ori):
                self._rangefinder_down_m = float(cur)
        except Exception:
            pass
        try:
            self._obstacle_radar.set_distance_sensor(payload)
            if bool(getattr(self, "_last_link_connected", False)) and bool(
                getattr(self, "_web_ready", False)
            ):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def get_obstacle_sensor_summary(self) -> tuple[str, str]:
        """Nearest obstacle + rangefinder text for dashboard telemetry panel."""
        try:
            return self._obstacle_radar.summary_text()
        except Exception:
            return "N/A", "N/A"

    def is_map_motion_armed(self) -> bool:
        return bool(getattr(self, "_map_motion_armed", False))

    def get_vehicle_display_position(self) -> tuple[float, float] | None:
        la = getattr(self, "_map_display_lat", None)
        lo = getattr(self, "_map_display_lon", None)
        if la is not None and lo is not None:
            return float(la), float(lo)
        if self._lat is not None and self._lon is not None:
            return float(self._lat), float(self._lon)
        return None

    def set_flight_telemetry(
        self,
        *,
        relative_alt_m: float,
        ground_speed_mps: float,
        vertical_speed_mps: float = 0.0,
        flight_time_text: str,
        msl_alt_m: float = 0.0,
        distance_from_home_m: float = 0.0,
    ) -> None:
        del msl_alt_m
        rel_alt_m = f"{float(relative_alt_m):.1f}"
        gs_mps = f"{float(ground_speed_mps):.1f}"
        vs_mps = f"{float(vertical_speed_mps):.1f}"
        dist_m = f"{float(distance_from_home_m):.1f}"
        ttime = str(flight_time_text)
        sig = f"{rel_alt_m}|{gs_mps}|{vs_mps}|{dist_m}|{ttime}"
        if sig == self._last_flight_telemetry_sig:
            return
        self._last_flight_telemetry_sig = sig
        # HUD display only — keep _vehicle_rel_alt_m from set_vehicle_position (raw MAVLink).
        try:
            self._native_telemetry.set_values(
                f"{rel_alt_m} m",
                f"{vs_mps} m/s",
                ttime,
                f"{dist_m} m",
                f"{gs_mps} m/s",
                f"{rel_alt_m} m",
            )
            QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass
        self._run_js(
            "setTelemetryOverlay("
            f"{float(relative_alt_m):.3f}, "
            f"{float(ground_speed_mps):.3f}, "
            f"{json.dumps(ttime)}, "
            f"{float(distance_from_home_m):.3f}, "
            f"{float(vertical_speed_mps):.3f}"
            ");"
        )
