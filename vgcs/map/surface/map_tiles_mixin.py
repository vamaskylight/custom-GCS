"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from PySide6.QtCore import QSettings, QThreadPool, QTimer, Qt, QUrl
from PySide6.QtWidgets import QFileDialog, QSizePolicy, QStackedWidget

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D
from vgcs.map.native_tile_map import NativeTileMapView, bundled_seed_root
from vgcs.map.surface.constants import _WEB_MAP_RELAYOUT_JS, MAP_BACKEND_BUILD
from vgcs.map.surface.helpers import _web_2d_fallback_allowed
from vgcs.map.surface.settings_keys import (
    _KEY_MAP_LOW_SPEC_MODE,
    _KEY_MAP_OFFLINE_TILE_ROOT,
    _KEY_MAP_TILE_MODE,
    _KEY_MAP_WEBCAM_ENABLED,
)
from vgcs.map.surface.tile_probe import _TileProbeBridge, _TileProbeTask


class MapTilesMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _native_map_tile_count(self) -> int:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return 0
        try:
            return int(nm.loaded_tile_count())
        except Exception:
            return 0

    def _promote_native_map_if_ready(self) -> bool:
        """Use Qt tiles for the main map when they are available (swap PiP can still grab native)."""
        if getattr(self, "_is_3d_mode", False):
            return False
        if self._native_map_tile_count() < 4:
            return False
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        if getattr(self, "_web_2d_fallback_active", False):
            self._web_2d_fallback_active = False
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.show()
                nm.update()
                nm.repaint()
            except Exception:
                pass
        return True

    def _show_map_main_surface(self) -> None:
        """After swap-to-map: native Qt tiles only (never WebEngine for 2D)."""
        self._ensure_native_map_visible()
        self._promote_native_map_if_ready()
        try:
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def _activate_startup_tile_source(self) -> None:
        """Default to Esri satellite; use offline only when explicitly configured and tiles exist."""
        try:
            s = QSettings(QS_ORG, QS_APP)
            mode = str(s.value(_KEY_MAP_TILE_MODE, "sat") or "sat").strip().lower()
            root = str(s.value(_KEY_MAP_OFFLINE_TILE_ROOT, "") or "").strip()
        except Exception:
            mode, root = "sat", ""
        use_offline = mode == "offline" and bool(root) and Path(root).is_dir()
        if use_offline:
            self.activate_offline_tiles(root)
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "local_viewport_has_tiles"):
                try:
                    if not nm.local_viewport_has_tiles():
                        use_offline = False
                        print(
                            "[VGCS:map] offline folder has no tiles for this location — "
                            "using Esri World Imagery"
                        )
                except Exception:
                    use_offline = False
        if not use_offline:
            self.activate_satellite_tiles()

    def _native_tile_startup_check(self) -> None:
        """If the native 2D map still has no tiles, switch to Esri or nudge fetch (no cache wipe loop)."""
        if getattr(self, "_native_tile_fallback_done", False):
            return
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            n = int(nm.loaded_tile_count())
        except Exception:
            n = 0
        if n > 2:
            self._native_tile_fallback_done = True
            return
        tmpl = str(getattr(nm, "_tile_template", "") or "").lower()
        retries = int(getattr(self, "_native_tile_startup_retries", 0) or 0)
        if retries >= 2:
            try:
                QTimer.singleShot(1500, self._native_tile_startup_check_final)
            except Exception:
                pass
            return
        self._native_tile_startup_retries = retries + 1
        try:
            print(
                f"[VGCS:map] native tiles still loading (loaded={n}) — "
                f"{'switching to Esri' if '{local}' in tmpl else 'nudge fetch'}"
            )
        except Exception:
            pass
        if "{local}" in tmpl:
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
        elif "world_imagery" in tmpl:
            try:
                nm.prefetch_viewport_tiles()
                nm._warm_disk_tiles_for_viewport()
            except Exception:
                pass
        else:
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
        try:
            QTimer.singleShot(2500, self._native_tile_startup_check)
        except Exception:
            pass

    def _native_tile_startup_check_final(self) -> None:
        if getattr(self, "_native_tile_fallback_done", False):
            return
        if getattr(self, "_web_2d_fallback_active", False):
            return
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            n = int(nm.loaded_tile_count())
        except Exception:
            n = 0
        if n > 2:
            self._native_tile_fallback_done = True
            return
        self._native_tile_fallback_done = True
        tmpl = str(getattr(nm, "_tile_template", "") or "").lower()
        if "{local}" in tmpl or n <= 0:
            try:
                print("[VGCS:map] tiles still missing — activating Esri World Imagery")
                self.activate_satellite_tiles()
            except Exception:
                pass
        try:
            nm.prefetch_viewport_tiles()
            nm._warm_disk_tiles_for_viewport()
        except Exception:
            pass
        self._set_status(
            "Satellite tiles loading slowly — check internet/firewall or pick Offline Tiles in the toolbar"
        )

    def _sync_web_map_center_from_native(self) -> None:
        """Push native map center/zoom into Leaflet before showing the WebEngine view."""
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            lat = float(getattr(nm, "_center_lat", 37.7749))
            lon = float(getattr(nm, "_center_lon", -122.4194))
            zoom = float(getattr(nm, "_zoom", 16.0))
        except Exception:
            return
        try:
            self._run_js(
                f"window.__vgcsPendingCenter=[{lat},{lon}];"
                f"window.__vgcsPendingZoom={zoom};"
            )
        except Exception:
            pass

    def _relayout_web_map_view(self) -> None:
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is None:
            return
        try:
            host = getattr(self, "_map_stack", None) or self._map_canvas
            w3.resize(max(1, host.width()), max(1, host.height()))
        except Exception:
            pass
        try:
            w3.page().runJavaScript(_WEB_MAP_RELAYOUT_JS, lambda *_: None)
        except Exception:
            pass

    def _ensure_native_map_visible(self) -> None:
        """Keep 2D map on NativeTileMapView (no WebEngine Leaflet layer)."""
        if getattr(self, "_is_3d_mode", False):
            return
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        if getattr(self, "_web_2d_fallback_active", False):
            self._web_2d_fallback_active = False
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            if int(nm.loaded_tile_count()) >= 4:
                nm.update()
                nm.repaint()
                return
        except Exception:
            pass
        if not getattr(self, "_native_tile_fallback_done", False):
            try:
                QTimer.singleShot(0, self._native_tile_startup_check)
            except Exception:
                pass

    def _ensure_map_tiles_visible(self) -> None:
        """Alias: 2D map is always native Qt (see ``_ensure_native_map_visible``)."""
        self._ensure_native_map_visible()

    def _activate_web_2d_fallback(self) -> bool:
        """Optional Leaflet 2D via WebEngine (off by default; set VGCS_ALLOW_WEB_2D_FALLBACK=1)."""
        if not _web_2d_fallback_allowed():
            self._ensure_native_map_visible()
            return False
        if getattr(self, "_web_2d_fallback_active", False):
            self._relayout_web_map_view()
            return True
        if not HAS_WEBENGINE_3D:
            try:
                print("[VGCS:map] web 2D fallback unavailable (Qt WebEngine not installed)")
            except Exception:
                pass
            return False
        if not self._ensure_web_3d_view():
            return False

        def _apply() -> None:
            self._web_2d_fallback_active = True
            self._native_tile_fallback_done = True
            self._pending_web_2d_fallback = False
            self._is_3d_mode = False
            self._sync_web_map_center_from_native()
            try:
                self._map_stack.setCurrentWidget(self._web_3d_view)
            except Exception:
                try:
                    self._map_stack.setCurrentIndex(1)
                except Exception:
                    pass
            self._inject_legacy_html_hud_hide()
            w3 = getattr(self, "_web_3d_view", None)
            if w3 is not None:
                try:
                    w3.show()
                    w3.raise_()
                except Exception:
                    pass
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._schedule_vehicle_pose_js(immediate=True)
            self._relayout_web_map_view()
            for ms in (0, 80, 250, 600):
                QTimer.singleShot(ms, self._relayout_web_map_view)
            try:
                print("[VGCS:map] activated WebEngine 2D fallback (native tiles did not load)")
            except Exception:
                pass
            self._set_status("Map: web view active (native tiles blocked on this PC)")

        w3 = getattr(self, "_web_3d_view", None)
        if w3 is not None and bool(getattr(self, "_web_3d_ready", False)):
            _apply()
            return True
        self._pending_web_2d_fallback = True
        return True

    def _probe_current_tiles(self, *, reason: str) -> None:
        # Probe the *current* view tile (not just z=0), because placeholders often occur only at higher zooms.
        nm = getattr(self, "_native_map", None)
        if nm is not None and hasattr(nm, "get_map_view_dict"):
            try:
                payload = json.dumps(nm.get_map_view_dict())
                self._probe_current_tiles_from_payload(payload, reason=reason)
                return
            except Exception:
                pass

        def _kick(payload: str | None) -> None:
            self._probe_current_tiles_from_payload(payload, reason=reason)

        try:
            self._run_js("window.__vgcsGetMapView ? window.__vgcsGetMapView() : '';", callback=_kick)
        except Exception:
            _kick(None)

    def _probe_current_tiles_from_payload(self, payload: str | None, *, reason: str) -> None:
        try:
            print(f"[VGCS:map] map_view_payload ({reason}) {str(payload or '')[:220]}")
        except Exception:
            pass
        try:
            data = json.loads(payload or "{}")
        except Exception:
            data = {}
        try:
            z = int(data.get("z", 0) or 0)
            lat = float(data.get("lat", 0.0) or 0.0)
            lng = float(data.get("lng", 0.0) or 0.0)
            tmpl = str(data.get("template", "") or "")
        except Exception:
            z, lat, lng, tmpl = 0, 0.0, 0.0, ""

        def slippy_xy(lat_deg: float, lon_deg: float, zoom: int) -> tuple[int, int]:
            import math

            lat_rad = math.radians(max(-85.0511, min(85.0511, lat_deg)))
            n = 2.0**zoom
            xt = int((lon_deg + 180.0) / 360.0 * n)
            yt = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
            return max(0, xt), max(0, yt)

        candidates: list[tuple[str, str]] = []
        if tmpl:
            x, y = slippy_xy(lat, lng, max(0, min(19, z)))
            if tmpl == "{local}":
                nm = getattr(self, "_native_map", None)
                root = getattr(nm, "_offline_root", None) if nm is not None else None
                if root:
                    p = Path(str(root)) / str(z) / str(x) / f"{y}.png"
                    candidates.append(("active_view", p.as_uri()))
            else:
                url = (
                    tmpl.replace("{z}", str(z))
                    .replace("{x}", str(x))
                    .replace("{y}", str(y))
                    .replace("{s}", "a")
                )
                candidates.append(("active_view", url))
        else:
            candidates.extend(
                [
                    (
                        "esri_imagery_view",
                        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/10/0/0",
                    ),
                    (
                        "esri_streets_view",
                        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/10/0/0",
                    ),
                ]
            )

        if tmpl and tmpl != "{local}":
            z0 = (
                tmpl.replace("{z}", "0")
                .replace("{x}", "0")
                .replace("{y}", "0")
                .replace("{s}", "a")
            )
            candidates.append(("active_world", z0))

        for label, url in candidates:
            try:
                QThreadPool.globalInstance().start(
                    _TileProbeTask(
                        url=url,
                        provider_label=f"{label}:{reason}",
                        bridge=self._tile_probe_bridge,
                    )
                )
            except Exception:
                pass

    def _on_tile_probe_result(self, provider_label: str, outcome: str, detail: str) -> None:
        try:
            print(f"[VGCS:map] tile_probe {provider_label} -> {outcome} ({detail})")
        except Exception:
            pass
        label = str(provider_label)
        detail_s = str(detail)
        network_fail = outcome.startswith("error:") and (
            "URLError" in detail_s or "urlopen error" in detail_s.lower()
        )
        if "active_view" in label and network_fail:
            if not getattr(self, "_tile_network_fallback_done", False):
                self._tile_network_fallback_done = True
                self._set_status(
                    "Satellite tiles unreachable — check internet or use Offline Tiles in the toolbar"
                )
                try:
                    QTimer.singleShot(800, self.activate_satellite_tiles)
                except Exception:
                    pass
            return
        # Do not auto-switch map type (product default is Esri World Imagery satellite).
        if "active_view" in label and outcome == "placeholder_suspected":
            self._set_status(
                "Satellite tile looks blocked/placeholder — use Offline Tiles or Esri Streets in the toolbar"
            )
        if "esri_imagery" in label and outcome == "placeholder_suspected":
            self._set_status(
                "Satellite imagery blocked on this network — use Offline Tiles or Esri Streets in the toolbar"
            )

    def _set_webcam_enabled(self, enabled: bool) -> None:
        on = bool(enabled)
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_WEBCAM_ENABLED, on)
        except Exception:
            pass
        if not on:
            self._stop_video_preview(clear_overlay=True)
            self._set_status("Webcam disabled")
            return
        # Start if the page is ready; do not require telemetry link.
        if bool(getattr(self, "_web_ready", False)):
            self._start_video_preview()
            self._set_status("Webcam enabled")
        else:
            self._set_status("Webcam enabled (will start when map is ready)")

    def _on_perf_mode_changed(self, idx: int) -> None:
        # 0=Auto, 1=Low, 2=High
        mode = "auto"
        if int(idx) == 1:
            mode = "on"
        elif int(idx) == 2:
            mode = "off"
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_LOW_SPEC_MODE, mode)
        except Exception:
            pass
        if mode == "on":
            self._apply_low_spec_mode(True, reason="manual")
        elif mode == "off":
            self._apply_low_spec_mode(False, reason="manual")
        else:
            # Auto: apply any already-detected effective mode; otherwise default to high.
            self._apply_low_spec_mode(bool(getattr(self, "_low_spec_autodetected", False)), reason="auto")

    def _apply_low_spec_mode(self, enabled: bool, *, reason: str) -> None:
        on = bool(enabled)
        self._low_spec_effective = on
        # Python-side throttles (JS bridge pressure is a major cause of lag).
        try:
            self._vehicle_pose_timer.setInterval(240 if on else 120)
        except Exception:
            pass
        try:
            if hasattr(self, "_video_push_timer"):
                self._video_push_timer.setInterval(66)
        except Exception:
            pass
        # Keep video quality stable even in low-spec mode.
        # Low-spec still reduces map/tile workload, but stream fidelity is preserved.
        try:
            self._video_encode_max_w = 1920
            self._video_encode_max_h = 1080
            self._video_encode_format = "PNG"
            self._video_encode_quality = 1
        except Exception:
            pass
        # JS-side tile/label adjustments.
        try:
            self._run_js("setLowSpecMode(true);" if on else "setLowSpecMode(false);")
        except Exception:
            pass
        if reason == "manual":
            self._set_status("Performance: Low" if on else "Performance: High")

    def _maybe_autodetect_low_spec(self) -> None:
        # Heuristic: measure WebEngine JS callback latency; if it's consistently high, enable low-spec.
        try:
            s = QSettings(QS_ORG, QS_APP)
            mode = str(s.value(_KEY_MAP_LOW_SPEC_MODE, "auto") or "auto").strip().lower()
        except Exception:
            mode = "auto"
        if mode != "auto":
            return
        if getattr(self, "_low_spec_autodetected", False):
            return
        if not bool(getattr(self, "_web_ready", False)):
            return

        try:
            from time import perf_counter
        except Exception:
            return

        samples: list[float] = []

        def one() -> None:
            t0 = perf_counter()

            def cb(_val) -> None:
                dt = (perf_counter() - t0) * 1000.0
                samples.append(float(dt))
                if len(samples) >= 3:
                    avg = sum(samples) / len(samples)
                    if avg >= 120.0:
                        self._low_spec_autodetected = True
                        self._apply_low_spec_mode(True, reason="auto")
                        self._set_status("Performance: Auto (low-spec detected)")
                    return
                one()

            self._run_js("Date.now();", callback=cb)

        one()

    def center_on_vehicle(self) -> None:
        """Recenter the map on the vehicle (native: `set_center` from widget coords, else native vehicle)."""
        if bool(getattr(self, "_is_3d_mode", False)):
            self._run_js("centerOnVehicle();")
            return
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                la = getattr(self, "_lat", None)
                lo = getattr(self, "_lon", None)
                if la is not None and lo is not None:
                    nm.set_center(float(la), float(lo))
                else:
                    nm.center_on_vehicle()
                try:
                    if self._native_minimap_wrap.isVisible():
                        self._schedule_native_minimap_refresh()
                except Exception:
                    pass
                return
        except Exception:
            pass
        self._run_js("centerOnVehicle();")

    def _init_map_backend(self) -> None:
        self._map_canvas.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self._map_canvas.setAutoFillBackground(True)
        self._map_stack = QStackedWidget(self._map_canvas)
        self._map_stack.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._native_map = NativeTileMapView()
        self._native_map.setMinimumHeight(260)
        self._native_map.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._map_stack.addWidget(self._native_map)
        self._map_canvas_layout.addWidget(self._map_stack, 1)
        self._map_3d_marker_overlay = None
        self._map_3d_overlay_timer = QTimer(self)
        self._map_3d_overlay_timer.setInterval(50)
        self._map_3d_overlay_timer.timeout.connect(self._refresh_3d_marker_overlay)
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        # PiP / compass / rail layer stay on `_map_canvas`; keep the map stack below those siblings.
        try:
            self._map_stack.lower()
        except Exception:
            pass
        self._native_map.user_waypoints_changed.connect(self._on_native_user_waypoints_changed)
        self._native_map.observation_map_click.connect(self._on_native_observation_map_click)
        self._native_map.zoom_changed.connect(self._sync_native_map_zoom_label)
        try:
            self._sync_native_map_zoom_label(float(getattr(self._native_map, "_zoom", 16.0)))
        except Exception:
            pass
        try:
            seed_ok = bundled_seed_root().is_dir()
            print(
                f"[VGCS:map] backend build {MAP_BACKEND_BUILD} | "
                f"[VGCS:map-native] ready bundled_seed={'yes' if seed_ok else 'no'}"
            )
        except Exception:
            pass
        self._web_ready = True
        self._set_status("Map backend: Native Qt tiles")
        self._on_map_loaded(True)
        QTimer.singleShot(0, self._refresh_dooaf_map_overlay)

        try:
            s = QSettings(QS_ORG, QS_APP)
            mode = str(s.value(_KEY_MAP_LOW_SPEC_MODE, "auto") or "auto").strip().lower()
        except Exception:
            mode = "auto"
        if mode == "on":
            self._apply_low_spec_mode(True, reason="manual")
            try:
                self._perf_mode.setCurrentIndex(1)
            except Exception:
                pass
        elif mode == "off":
            self._apply_low_spec_mode(False, reason="manual")
            try:
                self._perf_mode.setCurrentIndex(2)
            except Exception:
                pass
        else:
            try:
                self._perf_mode.setCurrentIndex(0)
            except Exception:
                pass

        # 2D map stays on NativeTileMapView. WebEngine loads only when the operator enables 3D.
        try:
            QTimer.singleShot(0, self._ensure_native_map_visible)
        except Exception:
            pass

    def _on_native_user_waypoints_changed(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is None:
                return
            self._on_waypoints_json(nm.waypoints_json())
        except Exception:
            pass

    def _on_map_loaded(self, ok: bool) -> None:
        self._web_ready = bool(ok)
        if self._web_ready:
            self._last_plan_flight_metrics_payload = None
            self._last_flight_telemetry_sig = None
            self._last_link_connected = None
            self._last_flight_status_key = None
            self._last_header_gps_key = None
            self._last_header_battery = None
            self._last_header_mode = None
            self._last_plan_vehicle_info_key = None
            self._vehicle_pose_timer.stop()
            self._set_status("Map ready")
            t = self._plan_rail_tool_state
            self._run_js(
                f"window.__planRailTool = {json.dumps(t)}; setPlanRailTool({json.dumps(t)});"
            )
            # Tile selection: Esri satellite by default; offline only if mode=offline and tiles exist.
            try:
                self._activate_startup_tile_source()
                try:
                    QTimer.singleShot(1200, lambda: self._probe_current_tiles(reason="startup"))
                except Exception:
                    pass
                try:
                    QTimer.singleShot(2000, self._native_tile_startup_check)
                except Exception:
                    pass
                for delay_ms in (900, 1800, 3500):
                    try:
                        QTimer.singleShot(delay_ms, self._ensure_native_map_visible)
                    except Exception:
                        pass
            except Exception:
                pass
            self.map_page_ready.emit()
            try:
                self._sync_native_camera_rail_toggles()
            except Exception:
                pass
            try:
                self.apply_video_settings()
            except Exception:
                pass
            try:
                self._sync_native_thermal_feed_button()
            except Exception:
                pass
            try:
                self._native_compass.show()
                self._native_telemetry.show()
                mz = getattr(self, "_native_map_zoom_ctrl", None)
                if mz is not None:
                    mz.show()
                if bool(getattr(self, "_last_link_connected", False)):
                    try:
                        self._obstacle_radar.show()
                    except Exception:
                        pass
                # Keep camera rail (ZOOM/FOCUS/GIMBAL/OBSERVE) hidden until MAVLink is connected.
                # This ensures the UI follows the desired flow: "Disconnected" -> no camera controls.
                if bool(getattr(self, "_last_link_connected", False)):
                    if self._plan_flight_layer_obscures_native_camera_ui():
                        try:
                            self._native_hud_right.hide()
                            self._native_rail_layer.hide()
                            self._btn_camera_rail_show.hide()
                        except Exception:
                            pass
                    else:
                        self._sync_camera_rail_panel_visibility()
                    # Layout + Z-order immediately: `resizeEvent` can run while the layer is still hidden,
                    # which skipped `ly.raise_()` when gated on `isVisible()` — map then stayed above the rail.
                    try:
                        self._layout_native_hud()
                        self._stack_native_overlays_above_tile_map()
                    except Exception:
                        pass
                    QTimer.singleShot(0, self._layout_native_hud)
                else:
                    try:
                        self._native_hud_right.hide()
                    except Exception:
                        pass
                    try:
                        self._native_rail_layer.hide()
                        self._btn_camera_rail_show.hide()
                    except Exception:
                        pass
                    # Disconnected: still position compass, telemetry, zoom, and action rail.
                    try:
                        self._layout_native_hud()
                        self._stack_native_overlays_above_tile_map()
                    except Exception:
                        pass
                    QTimer.singleShot(0, self._layout_native_hud)
            except Exception:
                pass
            # Start preview only after MAVLink connect (see `_on_mavlink_link_show_mini_video`).
            try:
                if bool(getattr(self, "_last_link_connected", False)):
                    QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
            except Exception:
                pass
            # Auto-detect low-spec devices and reduce map workload if needed.
            try:
                QTimer.singleShot(250, self._maybe_autodetect_low_spec)
            except Exception:
                pass
        else:
            self._set_status("Map failed to load")

    def _enable_fence_polygon_mode(self) -> None:
        self._run_js("enableFencePolygon();")
        self._set_status("Fence polygon mode: click map to add points")

    def _set_esri_street_tiles(self) -> None:
        self.activate_esri_street_tiles()

    def _set_osm_tiles(self) -> None:
        self.activate_osm_tiles()

    def _set_satellite_tiles(self) -> None:
        self.activate_satellite_tiles()

    def _pick_offline_tiles(self) -> None:
        root = QFileDialog.getExistingDirectory(
            self,
            "Select offline tile root (contains z/x/y.png)",
            "",
        )
        if not root:
            return
        self.activate_offline_tiles(root)

    def activate_esri_street_tiles(self) -> None:
        """Default online tiles: most compatible in locked-down client networks."""
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_TILE_MODE, "esri_streets")
        except Exception:
            pass
        tmpl = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}"
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Tiles © Esri', 19);")
        self._set_status("Online tiles active (Esri Streets)")

    def activate_osm_tiles(self) -> None:
        """OSM tiles are often blocked for desktop apps (referrer policy). Keep optional."""
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_TILE_MODE, "osm")
        except Exception:
            pass
        tmpl = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, '&copy; OpenStreetMap contributors', 19);")
        self._set_status("Online tiles active (OSM)")

    def activate_satellite_tiles(self) -> None:
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_TILE_MODE, "sat")
        except Exception:
            pass
        tmpl = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
                try:
                    nm._warm_disk_tiles_for_viewport()
                except Exception:
                    pass
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Tiles © Esri', 19);")
        self._set_status("Satellite imagery active (Esri World Imagery)")

    def activate_offline_tiles(self, root: str) -> None:
        root = str(root or "").strip()
        if not root or not Path(root).is_dir():
            self._set_status("Offline tiles: invalid folder")
            return
        # Remember for next launch.
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_OFFLINE_TILE_ROOT, root)
            QSettings(QS_ORG, QS_APP).setValue(_KEY_MAP_TILE_MODE, "offline")
        except Exception:
            pass
        url = QUrl.fromLocalFile(root).toString().rstrip("/")
        tmpl = f"{url}/{{z}}/{{x}}/{{y}}.png"
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Offline tile cache', 19);")
        self._set_status("Offline tiles active")

    def _apply_geofence(self) -> None:
        def _circle_polygon_points(
            center_lat: float,
            center_lon: float,
            radius_m: float,
            *,
            segments: int = 24,
        ) -> list[list[float]]:
            import math

            r = max(10.0, float(radius_m))
            n = max(8, min(96, int(segments)))
            lat0 = math.radians(center_lat)
            lon0 = math.radians(center_lon)
            earth_r = 6_371_000.0
            ang = r / earth_r
            out: list[list[float]] = []
            for i in range(n):
                theta = 2.0 * math.pi * (i / n)
                # Destination point from (lat0,lon0) given distance/heading on a sphere.
                lat = math.asin(
                    math.sin(lat0) * math.cos(ang)
                    + math.cos(lat0) * math.sin(ang) * math.cos(theta)
                )
                lon = lon0 + math.atan2(
                    math.sin(theta) * math.sin(ang) * math.cos(lat0),
                    math.cos(ang) - math.sin(lat0) * math.sin(lat),
                )
                out.append([math.degrees(lat), math.degrees(lon)])
            return out

        def _after_fence_points(payload: str | None) -> None:
            points: list[list[float]] = []
            if payload:
                try:
                    rows = json.loads(payload)
                    for row in rows:
                        if isinstance(row, list) and len(row) >= 2:
                            points.append([float(row[0]), float(row[1])])
                except Exception:
                    points = []
            if len(points) >= 3:
                self.geofence_upload_requested.emit(
                    {
                        "points": points,
                        "alt_max_m": float(self._fence_alt_max.value()),
                        "action": float(self._fence_action.currentData() or 1.0),
                    }
                )
                self._set_status(f"Fence polygon requested ({len(points)} pts)")
                return
            if self._lat is None or self._lon is None:
                self._set_status("Fence: waiting for vehicle position")
                return
            radius = float(self._fence_radius.value())
            self._fence_radius_m = radius
            self._run_js(f"setFence({self._lat:.8f}, {self._lon:.8f}, {radius:.1f});")
            # ArduPilot "circular" fence is centered on HOME, not an arbitrary lat/lon.
            # To match what the operator sees on the map, upload a polygon approximation
            # centered on the current vehicle position.
            poly = _circle_polygon_points(self._lat, self._lon, radius, segments=28)
            self.geofence_upload_requested.emit(
                {
                    "points": poly,
                    "alt_max_m": float(self._fence_alt_max.value()),
                    "action": float(self._fence_action.currentData() or 1.0),
                }
            )
            self._set_status(f"Fence requested (circle→polygon, r={radius:.0f}m)")

        self._run_js("JSON.stringify(getFencePoints());", callback=_after_fence_points)

    def _clear_geofence(self) -> None:
        self._run_js("clearFence();")
        self.geofence_upload_requested.emit({"disable": True})
        self._set_status("Fence cleared")
