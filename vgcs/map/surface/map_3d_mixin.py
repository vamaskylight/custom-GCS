"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import json

from PySide6.QtCore import QTimer

from vgcs.map.legacy_leaflet_build import build_leaflet_html
from vgcs.map.map_3d_marker_overlay import Map3dLayer, Map3dMarkerOverlay
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D, assets_base_url, create_map_3d_web_view
from vgcs.map.surface.helpers import _web_2d_fallback_allowed


class Map3dMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    _HIDE_LEGACY_HTML_HUD_JS = (
        "(function(){"
        "var s=document.getElementById('vgcs_3d_hide_overlays_style');"
        "if(!s){s=document.createElement('style');s.id='vgcs_3d_hide_overlays_style';"
        "document.head.appendChild(s);}"
        "s.textContent='#linkBanner,#actionRail,#planFlightLayer,#cameraRail,"
        "#mapFooterHud,#telemetryStrip,#compass,#hdrMapModeBtn,#videoPreview"
        "{display:none !important;}"
        "#mapWrap>.overlay,#map3dMarkerCanvas{pointer-events:none !important;}';"
        "})();"
    )

    def _prime_3d_vehicle_coords_js(self) -> None:
        """Push known vehicle coords into legacy JS before the first 3D camera snap."""
        la = getattr(self, "_lat", None)
        lo = getattr(self, "_lon", None)
        if la is None or lo is None:
            return
        disp_la = getattr(self, "_map_display_lat", None)
        disp_lo = getattr(self, "_map_display_lon", None)
        lat = float(disp_la if disp_la is not None else la)
        lon = float(disp_lo if disp_lo is not None else lo)
        try:
            self._run_js(
                f"window.__lastVehLat={lat:.8f};window.__lastVehLon={lon:.8f};"
            )
        except Exception:
            pass

    def _recenter_3d_after_enable(self) -> None:
        """Snap the Cesium camera to the drone immediately after entering 3D."""
        if not bool(getattr(self, "_is_3d_mode", False)):
            return
        try:
            self._relayout_web_map_view()
        except Exception:
            pass
        self._prime_3d_vehicle_coords_js()
        try:
            self._run_js(
                "window.__lastFollowVehLat=null;window.__lastFollowVehLon=null;"
                "window.__vgcs3dUserInputMs=0;"
                "if(typeof __relayoutMap3d==='function')__relayoutMap3d();"
                "if(typeof __snap3dCameraToVehicle==='function')__snap3dCameraToVehicle(true);"
                "else centerOnVehicle();"
            )
        except Exception:
            pass
        try:
            self.center_on_vehicle()
        except Exception:
            pass

    def _set_3d_marker_overlay_active(self, active: bool) -> None:
        overlay = getattr(self, "_map_3d_marker_overlay", None)
        timer = getattr(self, "_map_3d_overlay_timer", None)
        if overlay is None:
            return
        if active:
            self._sync_3d_map_overlays()
            self._refresh_3d_marker_overlay()
            if timer is not None:
                timer.start()
        else:
            if timer is not None:
                timer.stop()
            overlay = getattr(self, "_map_3d_marker_overlay", None)
            if overlay is not None:
                overlay.hide()
                overlay.set_items([])

    def _layout_map_3d_marker_overlay(self) -> None:
        """Markers render in the HTML canvas; keep the Qt overlay hidden (must not cover WebEngine)."""
        overlay = getattr(self, "_map_3d_marker_overlay", None)
        if overlay is not None:
            overlay.hide()
            overlay.set_items([])

    def _refresh_3d_marker_overlay(self) -> None:
        if not bool(getattr(self, "_is_3d_mode", False)):
            return
        if not bool(getattr(self, "_web_3d_ready", False)):
            return
        try:
            payload = self._build_3d_overlay_payload()
            blob = json.dumps(payload)
            self._run_js(
                f"sync3dMapOverlays({blob});"
                "if(typeof __schedule3dMarkerPaint==='function')__schedule3dMarkerPaint();"
            )
        except Exception:
            pass

    def _on_3d_marker_overlay_json(self, payload: object) -> None:
        overlay = getattr(self, "_map_3d_marker_overlay", None)
        if overlay is None:
            return
        items: list[dict] = []
        if isinstance(payload, list):
            items = [x for x in payload if isinstance(x, dict)]
        elif payload is not None:
            try:
                parsed = json.loads(str(payload))
                if isinstance(parsed, list):
                    items = [x for x in parsed if isinstance(x, dict)]
            except Exception:
                pass
        overlay.set_items(items)

    def _build_3d_overlay_payload(self) -> dict[str, object]:
        """Collect observation / DOOAF / waypoint overlays for Cesium (mirrors native 2D map)."""
        obs_marks: list[list[float]] = []
        geo_marks: list[list[float]] = []
        for row in self._observations:
            kind = str(row.get("kind") or "")
            if kind == "map_mark":
                la = row.get("map_lat")
                lo = row.get("map_lon")
                if la is not None and lo is not None:
                    obs_marks.append([float(la), float(lo)])
            elif kind == "video_mark":
                la = row.get("target_lat")
                lo = row.get("target_lon")
                if la is not None and lo is not None:
                    geo_marks.append([float(la), float(lo)])
        s = self._resolved_dooaf_settings()
        gun = (
            [float(s.gun_lat), float(s.gun_lon)]
            if s.gun_lat is not None and s.gun_lon is not None
            else None
        )
        intended = (
            [float(s.target_lat), float(s.target_lon)]
            if s.target_lat is not None and s.target_lon is not None
            else None
        )
        impact_mark = latest_mark(self._observations, DOOAF_ROLE_IMPACT)
        impact = (
            [float(impact_mark.lat), float(impact_mark.lon)]
            if impact_mark is not None
            else None
        )
        labels, _ = self._observation_measure_labels_and_segments()
        track = target_track_from_observations(self._observations)
        track_rows = [[float(a), float(b)] for a, b in track]
        waypoints: list[list[float]] = []
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                wps = json.loads(nm.waypoints_json())
                if isinstance(wps, list):
                    waypoints = [
                        [float(r[0]), float(r[1])]
                        for r in wps
                        if isinstance(r, (list, tuple)) and len(r) >= 2
                    ]
            except Exception:
                pass
        if not waypoints and self._waypoints_model:
            waypoints = [[float(wp.lat), float(wp.lon)] for wp in self._waypoints_model]
        vehicle: list[float] | None = None
        if self._lat is not None and self._lon is not None:
            vehicle = [
                float(self._map_display_lat if self._map_display_lat is not None else self._lat),
                float(self._map_display_lon if self._map_display_lon is not None else self._lon),
            ]
        heading = float(self._heading) if self._heading is not None else 0.0
        return {
            "obsMarks": obs_marks,
            "geoMarks": geo_marks,
            "dooaf": {"gun": gun, "intended": intended, "impact": impact},
            "targetTrack": track_rows,
            "targetLabels": [str(x) for x in labels],
            "waypoints": waypoints,
            "vehicle": vehicle,
            "heading": heading,
        }

    def _sync_3d_map_overlays(self) -> None:
        if not bool(getattr(self, "_is_3d_mode", False)):
            return
        try:
            payload = self._build_3d_overlay_payload()
            self._run_js(f"sync3dMapOverlays({json.dumps(payload)});")
        except Exception:
            pass

    def _toggle_3d_mode(self, enabled: bool) -> None:
        active = self.set_3d_enabled(enabled)
        if active != enabled:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(active)
            self._btn_3d.blockSignals(False)

    def _emit_map_3d_mode_changed(self) -> None:
        try:
            self.map_3d_mode_changed.emit()
        except Exception:
            pass

    def _ensure_web_3d_view(self) -> bool:
        """Lazily create the legacy WebEngine map used only for 3D (Cesium)."""
        if not HAS_WEBENGINE_3D:
            return False
        if getattr(self, "_web_3d_view", None) is not None:
            return True
        w = create_map_3d_web_view(None)
        if w is None:
            return False
        if getattr(self, "_map_3d_marker_overlay", None) is None:
            overlay = Map3dMarkerOverlay(self._map_canvas)
            overlay.hide()
            self._map_3d_marker_overlay = overlay
        layer = Map3dLayer(w)
        try:
            import vgcs.map.legacy_leaflet_build as _llb

            _llb._leaflet_template = None  # noqa: SLF001 — pick up HTML edits (vehicle overlay export)
            html = build_leaflet_html()
            w.loadFinished.connect(self._on_web_3d_load_finished)
            w.titleChanged.connect(self._on_web_title_changed)
            w.setHtml(html, assets_base_url())
        except Exception as e:
            self._set_status(f"3D HTML build failed: {e}")
            try:
                layer.deleteLater()
            except Exception:
                pass
            return False
        self._web_3d_view = w
        self._map_3d_layer = layer
        self._web_3d_ready = False
        self._map_stack.addWidget(layer)
        return True

    def _inject_legacy_html_hud_hide(self) -> None:
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is None:
            return
        try:
            w3.page().runJavaScript(self._HIDE_LEGACY_HTML_HUD_JS, lambda *_: None)
        except Exception:
            pass

    def _on_web_3d_load_finished(self, ok: bool) -> None:
        self._web_3d_ready = bool(ok)
        if not ok:
            self._pending_3d_activate = False
            self._set_status("3D map page failed to load (check network / WebEngine)")
            return
        self._inject_legacy_html_hud_hide()
        if getattr(self, "_pending_web_2d_fallback", False):
            self._pending_web_2d_fallback = False
            if _web_2d_fallback_allowed():
                try:
                    self._activate_web_2d_fallback()
                except Exception:
                    pass
            else:
                self._ensure_native_map_visible()
            return
        if not getattr(self, "_pending_3d_activate", False):
            self._ensure_native_map_visible()
            return
        self._pending_3d_activate = False
        try:
            self._map_stack.setCurrentIndex(1)
            self._is_3d_mode = True
            self._emit_map_3d_mode_changed()
            self._prime_3d_vehicle_coords_js()
            self._web_3d_view.page().runJavaScript(
                "set3DEnabled(true);",
                lambda res: self._on_3d_toggle_result(True, res),
            )
            try:
                QTimer.singleShot(0, self._web_3d_view.setFocus)
            except Exception:
                pass
        except Exception:
            self._is_3d_mode = False
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            self._on_3d_toggle_result(True, False)
        try:
            self._activate_startup_tile_source()
            QTimer.singleShot(1200, lambda: self._probe_current_tiles(reason="3d_startup"))
        except Exception:
            pass
        self._schedule_vehicle_pose_js(immediate=True)
        QTimer.singleShot(0, self._recenter_3d_after_enable)
        QTimer.singleShot(100, self._recenter_3d_after_enable)
        QTimer.singleShot(300, self._recenter_3d_after_enable)
        QTimer.singleShot(150, self._sync_3d_map_overlays)
        self._set_3d_marker_overlay_active(True)

    def set_3d_enabled(self, enabled: bool) -> bool:
        if not enabled:
            self._set_3d_marker_overlay_active(False)
            self._is_3d_mode = False
            self._pending_3d_activate = False
            if getattr(self, "_web_2d_fallback_active", False):
                self._web_2d_fallback_active = False
            w3 = getattr(self, "_web_3d_view", None)
            if w3 is not None and self._web_3d_ready:
                try:
                    w3.page().runJavaScript("set3DEnabled(false);", lambda *_: None)
                except Exception:
                    pass
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._set_status("2D mode active")
            self._schedule_vehicle_pose_js(immediate=True)
            try:
                self._native_compass.set_map_bearing_deg(0.0)
            except Exception:
                pass
            self._emit_map_3d_mode_changed()
            return True

        if not HAS_WEBENGINE_3D:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._set_status("3D requires Qt WebEngine (install PySide6 WebEngine)")
            return False
        if not self._web_ready:
            self._set_status("3D view unavailable: map backend not ready")
            return False
        if not self._ensure_web_3d_view():
            self._set_status("3D view could not start WebEngine")
            return False
        w3 = self._web_3d_view
        assert w3 is not None

        def _apply_3d_js() -> None:
            import vgcs.map.legacy_leaflet_build as _llb

            def _enable_3d(export_ok: object = True) -> None:
                if not bool(export_ok):
                    try:
                        _llb._leaflet_template = None  # noqa: SLF001
                        self._web_3d_ready = False
                        self._pending_3d_activate = True
                        w3.setHtml(build_leaflet_html(), assets_base_url())
                    except Exception:
                        pass
                    return
            try:
                self._map_stack.setCurrentIndex(1)
                self._is_3d_mode = True
                self._inject_legacy_html_hud_hide()
                self._emit_map_3d_mode_changed()
                self._prime_3d_vehicle_coords_js()
                w3.page().runJavaScript(
                    "set3DEnabled(true);",
                    lambda ok: self._on_3d_toggle_result(True, ok),
                )
                try:
                    QTimer.singleShot(0, w3.setFocus)
                except Exception:
                    pass
                self._schedule_vehicle_pose_js(immediate=True)
                QTimer.singleShot(0, self._recenter_3d_after_enable)
                QTimer.singleShot(100, self._recenter_3d_after_enable)
                QTimer.singleShot(300, self._recenter_3d_after_enable)
                QTimer.singleShot(150, self._sync_3d_map_overlays)
                self._set_3d_marker_overlay_active(True)
            except Exception:
                self._is_3d_mode = False
                try:
                    self._map_stack.setCurrentIndex(0)
                except Exception:
                    pass
                self._on_3d_toggle_result(True, False)

            if self._web_3d_ready:
                try:
                    w3.page().runJavaScript(
                        "(typeof paint3dMarkerCanvas==='function'"
                        "&&typeof __wire3dMapPanHandler==='function')",
                        _enable_3d,
                    )
                except Exception:
                    _enable_3d(True)
            else:
                self._pending_3d_activate = True

        if self._web_3d_ready:
            self._pending_3d_activate = False
            _apply_3d_js()
            try:
                self._activate_startup_tile_source()
            except Exception:
                pass
            return True
        self._pending_3d_activate = True
        return True

    def _on_3d_toggle_result(self, requested: bool, result: object) -> None:
        active = bool(result)
        self._is_3d_mode = active
        if not active:
            self._set_3d_marker_overlay_active(False)
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            try:
                self._native_compass.set_map_bearing_deg(0.0)
            except Exception:
                pass
        self._btn_3d.blockSignals(True)
        self._btn_3d.setChecked(active)
        self._btn_3d.blockSignals(False)
        if requested and active:
            self._set_status("3D mode active")
            self._set_3d_marker_overlay_active(True)
            self._schedule_vehicle_pose_js(immediate=True)
            QTimer.singleShot(0, self._recenter_3d_after_enable)
            QTimer.singleShot(100, self._recenter_3d_after_enable)
            QTimer.singleShot(150, self._sync_3d_map_overlays)
        elif requested:
            self._set_status("3D mode unavailable; using 2D")
        else:
            self._set_status("2D mode active")
        self._emit_map_3d_mode_changed()
