"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import QSettings, QTimer
from PySide6.QtWidgets import QFileDialog, QMessageBox

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.surface.settings_keys import (
    _KEY_PLAN_CURRENT_MISSION_JSON,
    _KEY_PLAN_LAST_MISSION_JSON_LEGACY,
    _KEY_TOOLBAR_EXPORT_MISSION_JSON,
)
from vgcs.mission import Waypoint, load_waypoints_json, save_waypoints_json, save_waypoints_kml


class PlanMissionMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _on_plan_panel_exit(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.disable_plan_edit_modes()
        except Exception:
            pass
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.hide()
        try:
            mar = getattr(self, "_map_action_rail", None)
            if mar is not None:
                mar.show()
        except Exception:
            pass
        self._set_map_footer_hud_visible(True)
        try:
            self._layout_native_hud()
            self._stack_native_overlays_above_tile_map()
        except Exception:
            pass
        self.plan_flight_exited.emit()

    def _sync_native_plan_edit_mode_for_rail_tool(self, tool: str) -> None:
        """Keep native map placement modes aligned with the active plan rail tool."""
        tl = (tool or "").strip().lower()
        if tl == "waypoint":
            self._enable_add_waypoint_mode()
            return
        if tl == "roi":
            self._enable_fence_polygon_mode()
            return
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.disable_plan_edit_modes()
        except Exception:
            pass

    def _on_plan_panel_tool(self, tool: str) -> None:
        t = (tool or "").strip()
        if not t:
            return
        self._plan_rail_tool_state = t
        self._sync_native_plan_edit_mode_for_rail_tool(t)
        self.plan_tool_requested.emit(t)

    def _on_plan_panel_mission_changed(self, payload: object) -> None:
        data = dict(payload) if isinstance(payload, dict) else {}
        self.plan_mission_panel_changed.emit(data)

    def _on_plan_panel_waypoints_changed(self, waypoints: object) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        try:
            n = len(waypoints) if hasattr(waypoints, "__len__") else 0
        except Exception:
            n = 0
        panel.set_waypoint_count(int(n))

    def _on_plan_panel_set_launch_to_map_center(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            lat = float(getattr(nm, "_center_lat", 0.0) or 0.0)
            lon = float(getattr(nm, "_center_lon", 0.0) or 0.0)
        except Exception:
            return
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_launch_position(lat, lon)

    def set_plan_flight_visible(self, visible: bool) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        if visible:
            self._layout_plan_flight_panel()
            panel.set_rail_tool(self._plan_rail_tool_state or "File")
            panel.set_waypoint_count(len(self._waypoints_model))
            panel.show()
            panel.raise_()
            try:
                mar = getattr(self, "_map_action_rail", None)
                if mar is not None:
                    mar.hide()
            except Exception:
                pass
            self._set_map_footer_hud_visible(False)
        else:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None:
                    nm.disable_plan_edit_modes()
            except Exception:
                pass
            panel.hide()
            try:
                mar = getattr(self, "_map_action_rail", None)
                if mar is not None:
                    mar.show()
            except Exception:
                pass
            self._set_map_footer_hud_visible(True)
        try:
            self._layout_native_hud()
            self._stack_native_overlays_above_tile_map()
        except Exception:
            pass

    def set_plan_flight_metrics(
        self,
        *,
        alt_diff_m: str,
        gradient: str,
        azimuth: str,
        heading: str,
        dist_prev_wp_m: str,
        mission_distance_m: str,
        mission_time: str,
        max_telem_dist_m: str,
    ) -> None:
        payload = {
            "altDiffM": alt_diff_m,
            "gradient": gradient,
            "azimuth": azimuth,
            "heading": heading,
            "distPrevWpM": dist_prev_wp_m,
            "missionDistanceM": mission_distance_m,
            "missionTime": mission_time,
            "maxTelemDistM": max_telem_dist_m,
        }
        if payload == self._last_plan_flight_metrics_payload:
            return
        self._last_plan_flight_metrics_payload = payload
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_metrics(payload)

    def refresh_plan_flight_chrome(self, *, link_ok: bool, waypoint_count: int) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_chrome_state(bool(link_ok), max(0, int(waypoint_count)))

    def set_plan_rail_tool(self, tool: str) -> None:
        t = (tool or "").strip()
        if not t:
            return
        self._plan_rail_tool_state = t
        self._sync_native_plan_edit_mode_for_rail_tool(t)
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_rail_tool(t)

    def apply_plan_mission_panel_state(self, state: dict[str, object]) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.apply_panel_state(dict(state) if isinstance(state, dict) else {})

    def set_plan_sequence_template(self, template_id: str | None) -> None:
        """Show/hide Mission tab pattern row (Survey / Corridor / Structure) to match template picks."""
        tid = (template_id or "").strip().lower()
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_sequence_template(tid)

    def set_plan_mission_start_stack(self, enabled: bool, survey_label: str = "Survey") -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_mission_start_stack(bool(enabled), str(survey_label or "Survey"))

    def set_plan_vehicle_info(self, firmware: str, vehicle: str) -> None:
        key = (str(firmware), str(vehicle))
        if key == self._last_plan_vehicle_info_key:
            return
        self._last_plan_vehicle_info_key = key
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_vehicle_info(str(firmware or ""), str(vehicle or ""))

    def get_default_waypoint_alt_m(self) -> float:
        return float(self._default_alt.value())

    def set_default_waypoint_alt_m(self, alt_m: float) -> None:
        self._default_alt.setValue(max(1.0, float(alt_m)))

    def request_mission_upload_from_map(self) -> None:
        self._request_upload()

    def request_mission_download_from_map(self) -> None:
        self._request_download()

    def clear_map_waypoints(self) -> None:
        self._clear_waypoints()

    def clear_plan_current_mission_path(self) -> None:
        """Forget the Plan Flight JSON path (e.g. after download or generated pattern)."""
        s = QSettings(QS_ORG, QS_APP)
        s.remove(_KEY_PLAN_CURRENT_MISSION_JSON)
        if s.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            s.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)

    def start_waypoint_planning(self) -> None:
        self._enable_add_waypoint_mode()

    def start_roi_planning(self) -> None:
        self._enable_fence_polygon_mode()

    def open_mission_file(self) -> None:
        self._import_mission()

    def get_vehicle_position(self) -> tuple[float, float] | None:
        if self._lat is None or self._lon is None:
            return None
        return float(self._lat), float(self._lon)

    def set_mission_nav_seq(self, seq: int) -> None:
        """MAVLink MISSION_CURRENT.seq: trim planned route / sync with vehicle progress."""
        self._run_js(
            f"window.__missionNavSeq = {max(0, int(seq))}; updateMissionRoutePolyline();"
        )

    def set_mission_waypoint_count(self, count: int) -> None:
        self._waypoint_count = max(0, int(count))
        self._mission.setText(f"Mission WPs: {self._waypoint_count}")

    def _enable_add_waypoint_mode(self) -> None:
        self._run_js("enableAddWaypoint();")
        self._set_status(
            "Click map to add waypoint · right-click or double-click a waypoint to remove"
        )

    def _clear_waypoints(self) -> None:
        self._run_js(
            "clearWaypoints();",
            callback=lambda _: self._after_clear_waypoints(),
        )
        self._set_status("waypoints cleared")

    def _after_clear_waypoints(self) -> None:
        self._after_waypoints_mutated()
        self.clear_plan_current_mission_path()

    def _sync_waypoint_count_from_map(self) -> None:
        # Poll lightweight count only; fetching full waypoint JSON every second can stall WebEngine on slow clients.
        self._run_js("getWaypointCount();", callback=self._on_waypoint_count)

    def _on_waypoint_count(self, count) -> None:
        try:
            c = int(count)
        except Exception:
            return
        c = max(0, c)
        if c == int(getattr(self, "_waypoint_count", 0) or 0):
            return
        # Count changed; now fetch full list once to sync model/UI.
        self._run_js("JSON.stringify(getWaypoints());", callback=self._on_waypoints_json)

    def _on_waypoints_json(self, payload: str | None) -> None:
        if not payload:
            self.set_mission_waypoint_count(0)
            self._waypoints_model = []
            self._rebuild_wp_selector()
            self.waypoints_changed.emit([])
            return
        waypoints = self._waypoints_from_map_json(payload)
        self._waypoints_model = waypoints
        self.set_mission_waypoint_count(len(waypoints))
        self._rebuild_wp_selector()
        self.waypoints_changed.emit(waypoints)

    def _after_waypoints_mutated(self) -> None:
        self._sync_waypoint_count_from_map()

    def _plan_waypoints_snapshot(self) -> list[Waypoint]:
        """Current plan waypoints for upload/save/export (native map + in-memory model).

        Avoids relying on the legacy JS ``getWaypoints()`` callback path, which can
        never fire if the bridge is not ready or the script is not recognized.
        """
        if self._waypoints_model:
            return list(self._waypoints_model)
        nm = getattr(self, "_native_map", None)
        if nm is not None and nm.waypoint_count() > 0:
            return self._waypoints_from_map_json(nm.waypoints_json())
        return []

    def _request_upload(self) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to upload")
            return
        self.mission_upload_requested.emit(wps)
        self._set_status(f"Mission upload requested ({len(wps)} WPs)")

    def _waypoints_from_map_json(self, payload: str | None) -> list[Waypoint]:
        if not payload:
            return []
        try:
            rows = json.loads(payload)
        except Exception:
            return []
        waypoints: list[Waypoint] = []
        for idx, row in enumerate(rows):
            if not (isinstance(row, list) and len(row) >= 2):
                continue
            lat = float(row[0])
            lon = float(row[1])
            alt = (
                self._waypoints_model[idx].alt_m
                if idx < len(self._waypoints_model)
                else float(self._default_alt.value())
            )
            spd = (
                float(getattr(self._waypoints_model[idx], "speed_mps", 5.0))
                if idx < len(self._waypoints_model)
                else float(self._default_speed.value())
            )
            waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt, speed_mps=spd))
        return waypoints

    def _request_download(self) -> None:
        self.mission_download_requested.emit()
        self._set_status("Mission download requested")

    @staticmethod
    def _plan_current_mission_path(settings: QSettings) -> str:
        cur = str(settings.value(_KEY_PLAN_CURRENT_MISSION_JSON, "") or "")
        if cur:
            return cur
        return str(settings.value(_KEY_PLAN_LAST_MISSION_JSON_LEGACY, "") or "")

    def save_plan_mission_json(self, *, save_as: bool) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to save")
            QMessageBox.information(
                self,
                "Plan Flight",
                "There are no waypoints to save. Add waypoints on the map first.",
            )
            return
        settings = QSettings(QS_ORG, QS_APP)
        path = ""
        if not save_as:
            last = self._plan_current_mission_path(settings)
            if last:
                parent = Path(last).expanduser().resolve().parent
                if parent.is_dir():
                    path = last
        if not path or save_as:
            path, _ = QFileDialog.getSaveFileName(
                self,
                "Save current mission as…" if save_as else "Set current mission file…",
                self._plan_current_mission_path(settings) or "mission-waypoints.json",
                "JSON files (*.json)",
            )
            if not path:
                return
        try:
            save_waypoints_json(path, wps)
        except Exception:
            self._set_status("Save failed")
            QMessageBox.warning(self, "Plan Flight", "Could not save the mission file.")
            return
        settings.setValue(_KEY_PLAN_CURRENT_MISSION_JSON, path)
        if settings.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            settings.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)
        self._set_status(f"Current mission saved ({len(wps)} WPs)")
        QMessageBox.information(
            self,
            "Plan saved",
            f"Saved {len(wps)} waypoint(s) to:\n{path}",
        )

    def save_plan_mission_kml(self) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to export")
            QMessageBox.information(
                self,
                "Plan Flight",
                "There are no waypoints to export as KML.",
            )
            return
        settings = QSettings(QS_ORG, QS_APP)
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save mission as KML",
            str(settings.value("plan_last_mission_kml", "") or "mission-waypoints.kml"),
            "KML files (*.kml)",
        )
        if not path:
            return
        try:
            save_waypoints_kml(path, wps)
        except Exception:
            self._set_status("KML export failed")
            QMessageBox.warning(self, "Plan Flight", "Could not save the KML file.")
            return
        settings.setValue("plan_last_mission_kml", path)
        self._set_status(f"KML saved ({len(wps)} WPs)")
        QMessageBox.information(
            self,
            "Export complete",
            f"Saved {len(wps)} waypoint(s) as KML:\n{path}",
        )

    def _export_mission(self) -> None:
        settings = QSettings(QS_ORG, QS_APP)
        last_export = str(settings.value(_KEY_TOOLBAR_EXPORT_MISSION_JSON, "") or "")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export mission to file…",
            last_export or "mission-waypoints.json",
            "JSON files (*.json)",
        )
        if not path:
            return

        waypoints = self._plan_waypoints_snapshot()
        if not waypoints:
            self._set_status("No waypoints to export")
            return
        try:
            save_waypoints_json(path, waypoints)
        except Exception:
            self._set_status("Export failed")
            return
        settings.setValue(_KEY_TOOLBAR_EXPORT_MISSION_JSON, path)
        self._set_status(f"Exported copy to file ({len(waypoints)} WPs)")

    def _import_mission(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Import mission waypoints",
            "",
            "JSON files (*.json)",
        )
        if not path:
            return
        try:
            waypoints = load_waypoints_json(path)
        except Exception:
            self._set_status("Import failed")
            return
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        js = f"setWaypoints({json.dumps(rows)});"
        self._run_js(js, callback=lambda _: self._after_waypoints_mutated())
        s = QSettings(QS_ORG, QS_APP)
        s.setValue(_KEY_PLAN_CURRENT_MISSION_JSON, path)
        if s.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            s.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)
        self._set_status(f"Mission opened as current file ({len(waypoints)} WPs)")

    def set_waypoints(
        self, waypoints: list[Waypoint], *, clear_plan_current_file: bool = False
    ) -> None:
        if clear_plan_current_file:
            self.clear_plan_current_mission_path()
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        nm = getattr(self, "_native_map", None)
        if nm is not None and not bool(getattr(self, "_is_3d_mode", False)):
            nm.set_waypoint_rows(rows)
            self.set_mission_waypoint_count(len(waypoints))
            self._rebuild_wp_selector()
            self.waypoints_changed.emit(list(waypoints))
            panel = getattr(self, "_plan_flight_panel", None)
            if panel is not None:
                panel.set_waypoint_count(len(waypoints))
        else:
            self._run_js(
                f"setWaypoints({json.dumps(rows)});",
                callback=lambda _: self._after_waypoints_mutated(),
            )
        self._set_status(f"Mission loaded ({len(waypoints)} WPs)")

    def get_waypoint_meta(self) -> list[dict[str, float]]:
        """Per-waypoint meta for the Plan Flight right panel."""
        out: list[dict[str, float]] = []
        for wp in self._waypoints_model:
            out.append(
                {
                    "alt_m": float(getattr(wp, "alt_m", 20.0)),
                    "speed_mps": float(getattr(wp, "speed_mps", 5.0)),
                }
            )
        return out

    def apply_waypoint_meta(self, meta: list[object]) -> None:
        """Apply per-waypoint alt/speed edits from Plan Flight panel."""
        if not self._waypoints_model:
            return
        changed = False
        for i, row in enumerate(meta):
            if i >= len(self._waypoints_model):
                break
            if not isinstance(row, dict):
                continue
            try:
                alt_m = float(row.get("alt_m", self._waypoints_model[i].alt_m))
                spd = float(row.get("speed_mps", getattr(self._waypoints_model[i], "speed_mps", 5.0)))
            except Exception:
                continue
            alt_m = max(1.0, alt_m)
            spd = max(0.1, spd)
            if float(self._waypoints_model[i].alt_m) != alt_m:
                self._waypoints_model[i].alt_m = alt_m
                changed = True
            if float(getattr(self._waypoints_model[i], "speed_mps", 5.0)) != spd:
                setattr(self._waypoints_model[i], "speed_mps", spd)
                changed = True
        if changed:
            self.waypoints_changed.emit(list(self._waypoints_model))

    def _rebuild_wp_selector(self) -> None:
        current = self._wp_selector.currentIndex()
        self._wp_selector.blockSignals(True)
        self._wp_selector.clear()
        for idx in range(len(self._waypoints_model)):
            self._wp_selector.addItem(f"WP {idx + 1}", idx)
        self._wp_selector.blockSignals(False)
        if self._waypoints_model:
            self._wp_selector.setCurrentIndex(max(0, min(current, len(self._waypoints_model) - 1)))
            self._on_wp_selected(self._wp_selector.currentIndex())
        else:
            self._wp_alt.setValue(float(self._default_alt.value()))
            self._wp_speed.setValue(float(self._default_speed.value()))

    def _on_wp_selected(self, index: int) -> None:
        if 0 <= index < len(self._waypoints_model):
            self._wp_alt.setValue(float(self._waypoints_model[index].alt_m))
            self._wp_speed.setValue(float(getattr(self._waypoints_model[index], "speed_mps", 5.0)))

    def _apply_altitude_to_selected(self) -> None:
        idx = self._wp_selector.currentIndex()
        if idx < 0 or idx >= len(self._waypoints_model):
            self._set_status("No waypoint selected")
            return
        self._waypoints_model[idx].alt_m = float(self._wp_alt.value())
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated WP {idx + 1} altitude to {self._wp_alt.value():.1f} m")

    def _apply_altitude_to_all(self) -> None:
        if not self._waypoints_model:
            self._set_status("No waypoints available")
            return
        alt = float(self._wp_alt.value())
        for wp in self._waypoints_model:
            wp.alt_m = alt
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated all waypoint altitudes to {alt:.1f} m")

    def _apply_speed_to_selected(self) -> None:
        idx = self._wp_selector.currentIndex()
        if idx < 0 or idx >= len(self._waypoints_model):
            self._set_status("No waypoint selected")
            return
        spd = max(0.1, float(self._wp_speed.value()))
        setattr(self._waypoints_model[idx], "speed_mps", spd)
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated WP {idx + 1} speed to {spd:.1f} m/s")

    def _apply_speed_to_all(self) -> None:
        if not self._waypoints_model:
            self._set_status("No waypoints available")
            return
        spd = max(0.1, float(self._wp_speed.value()))
        for wp in self._waypoints_model:
            setattr(wp, "speed_mps", spd)
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated all waypoint speeds to {spd:.1f} m/s")
