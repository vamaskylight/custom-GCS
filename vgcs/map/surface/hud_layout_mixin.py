"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import json

from PySide6.QtCore import QPoint, QSettings, QTimer, Qt
from PySide6.QtWidgets import QApplication

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.cam_rail_widgets import CamRailShowHandle
from vgcs.map.surface.constants import (
    _CAM_RAIL_GIMBAL_GRID_GAP,
    _CAM_RAIL_LAYER_INSET,
    _CAM_RAIL_PAD_BTN_W,
    _MAP_ACTION_RAIL_HEIGHT_PX,
    _MAP_ACTION_RAIL_LEFT_PX,
    _MAP_ACTION_RAIL_TOP_PX,
    _MAP_HUD_MARGIN_PX,
    _MAP_HUD_TOP_PX,
    _NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX,
    _NATIVE_CAM_RAIL_TOP_PX,
    _OBSTACLE_PANEL_MAX_H_PX,
    _OBSTACLE_PANEL_TOP_PX,
)
from vgcs.map.surface.settings_keys import _KEY_CAMERA_RAIL_VISIBLE


class NativeHudLayoutMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def set_dashboard_mode(self, enabled: bool) -> None:
        """Hide map-edit controls for clean dashboard layout."""
        self._toolbar.setVisible(not enabled)
        self._status_box.setVisible(not enabled)
        if enabled:
            self._panel.setTitle("")
            self._panel.setFlat(True)
            self._panel.setStyleSheet("QGroupBox { border: 0; margin-top: 0; padding: 0; }")
            self._map_canvas.setObjectName("")
            self._map_canvas.setStyleSheet("QFrame { border: 0; margin: 0; padding: 0; }")
            self._panel_layout.setContentsMargins(0, 0, 0, 0)
            self._panel_layout.setSpacing(0)
        else:
            self._panel.setTitle("3D Map")
            self._panel.setFlat(False)
            self._panel.setStyleSheet("")
            self._map_canvas.setObjectName("statusChip")
            self._map_canvas.setStyleSheet("")
            self._panel_layout.setContentsMargins(0, 0, 0, 0)
            self._panel_layout.setSpacing(8)

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(event)
        if bool(getattr(self, "_video_swapped", False)):
            self._refresh_native_overlay_insets()
        # Always relayout native HUD (camera rail, compass, telemetry) on resize — independent of video PiP.
        self._layout_native_video_preview()
        try:
            self._layout_native_hud()
        except Exception:
            pass
        try:
            self._layout_map_3d_marker_overlay()
        except Exception:
            pass
        try:
            self._layout_plan_flight_panel()
        except Exception:
            pass
        if not getattr(self, "_is_3d_mode", False):
            try:
                QTimer.singleShot(0, self._ensure_native_map_visible)
            except Exception:
                pass

    def showEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().showEvent(event)
        # First frame / platform quirks: layout can restore map above HUD siblings; fix Z-order after show.
        try:
            if bool(getattr(self, "_last_link_connected", False)):
                QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
            QTimer.singleShot(0, self._layout_native_hud)
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def _refresh_native_overlay_insets(self) -> None:
        if not bool(getattr(self, "_web_ready", False)):
            return
        self._native_overlay_insets = {
            "left": 170,
            "top": _NATIVE_CAM_RAIL_TOP_PX,
            "right": 192,
            "bottom": 130,
        }
        self._layout_native_video_preview()

    def _map_canvas_rect_on_panel(self, x: int, y: int, w: int, h: int) -> tuple[int, int, int, int]:
        """Map a `_map_canvas` child rect to `_panel` coordinates."""
        host = getattr(self, "_map_canvas", None)
        panel = getattr(self, "_panel", None)
        if host is None or panel is None:
            return x, y, w, h
        try:
            pt = host.mapTo(panel, QPoint(int(x), int(y)))
            return int(pt.x()), int(pt.y()), int(w), int(h)
        except Exception:
            return x, y, w, h

    def _raise_panel_flight_overlays(self) -> None:
        """Panel-level HUD above fullscreen video (legacy alias)."""
        self._raise_flight_hud_above_video()

    def _stack_native_overlays_above_tile_map(self) -> None:
        """
        Lower the tile map, raise video, then raise all flight HUD above the video layer.
        """
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.lower()
            except Exception:
                pass
        try:
            if bool(getattr(self, "_is_3d_mode", False)):
                self._layout_map_3d_marker_overlay()
        except Exception:
            pass
        try:
            pv = getattr(self, "_native_video_preview", None)
            if pv is not None and pv.isVisible():
                pv.raise_()
                try:
                    self._sync_native_video_overlay()
                except Exception:
                    pass
        except Exception:
            pass
        self._raise_flight_hud_above_video()

    def _sync_native_map_vehicle_arrow_scale(self) -> None:
        """Larger vehicle chevron while map is shown in the swap PiP (grab scales the view down)."""
        nm = getattr(self, "_native_map", None)
        if nm is None or not hasattr(nm, "set_vehicle_arrow_scale"):
            return
        swapped = bool(getattr(self, "_video_swapped", False))
        preview_on = bool(getattr(self, "_video_preview_enabled", False))
        plan_on = self._plan_flight_layer_obscures_native_camera_ui()
        boost = 1.85 if (preview_on and swapped and not plan_on) else 1.0
        try:
            nm.set_vehicle_arrow_scale(boost)
        except Exception:
            pass

    def _on_main_map_zoom_step(self, step: int) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            cur = float(getattr(nm, "_zoom", 16.0))
        except Exception:
            cur = 16.0
        try:
            nm.set_zoom(cur + float(step))
        except Exception:
            pass

    def _sync_native_map_zoom_label(self, z: float | None = None) -> None:
        ctrl = getattr(self, "_native_map_zoom_ctrl", None)
        if ctrl is None:
            return
        if z is None:
            nm = getattr(self, "_native_map", None)
            try:
                z = float(getattr(nm, "_zoom", 16.0)) if nm is not None else 16.0
            except Exception:
                z = 16.0
        try:
            ctrl.set_zoom_level(float(z))
        except Exception:
            pass

    def _camera_rail_visible_pref(self) -> bool:
        try:
            raw = QSettings(QS_ORG, QS_APP).value(_KEY_CAMERA_RAIL_VISIBLE, True)
            if raw in (False, "false", "False", "0", 0):
                return False
            return True
        except Exception:
            return True

    def _camera_rail_may_appear(self) -> bool:
        if not bool(getattr(self, "_last_link_connected", False)):
            return False
        if not bool(getattr(self, "_web_ready", False)):
            return False
        if self._plan_flight_layer_obscures_native_camera_ui():
            return False
        return True

    def _set_camera_rail_panel_visible(self, visible: bool) -> None:
        try:
            QSettings(QS_ORG, QS_APP).setValue(_KEY_CAMERA_RAIL_VISIBLE, bool(visible))
        except Exception:
            pass
        self._sync_camera_rail_panel_visibility()
        try:
            QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def _sync_camera_rail_panel_visibility(self) -> None:
        """Operator hide/show for the native `#cameraRail` (record, gimbal, OBSERVE)."""
        show_rail = self._camera_rail_may_appear() and self._camera_rail_visible_pref()
        tab = getattr(self, "_btn_camera_rail_show", None)
        try:
            if show_rail:
                self._native_hud_right.show()
                self._native_rail_layer.show()
                if tab is not None:
                    tab.hide()
            else:
                self._native_hud_right.hide()
                self._native_rail_layer.hide()
                if (
                    tab is not None
                    and self._camera_rail_may_appear()
                    and not self._camera_rail_visible_pref()
                ):
                    tab.show()
                    tab.raise_()
                elif tab is not None:
                    tab.hide()
        except Exception:
            pass

    def _position_camera_rail_show_tab(self) -> None:
        tab = getattr(self, "_btn_camera_rail_show", None)
        if tab is None or not tab.isVisible():
            return
        try:
            w = max(1, self._map_canvas.width())
            tw = int(getattr(CamRailShowHandle, "WIDTH_PX", 36))
            th = int(getattr(CamRailShowHandle, "HEIGHT_PX", 92))
            pt = self._map_canvas.mapTo(
                self._panel, QPoint(max(0, w - tw), int(_NATIVE_CAM_RAIL_TOP_PX))
            )
            tab.setGeometry(pt.x(), pt.y(), tw, th)
            tab.raise_()
        except Exception:
            pass

    def suppress_floating_overlays(self) -> dict[str, bool]:
        """Hide map chrome that can paint over modal dialogs (e.g. camera-rail tab)."""
        restore: dict[str, bool] = {}
        tab = getattr(self, "_btn_camera_rail_show", None)
        if tab is not None:
            restore["show_tab"] = bool(tab.isVisible())
            if restore["show_tab"]:
                tab.hide()
        return restore

    def restore_floating_overlays(self, restore: dict[str, bool] | None) -> None:
        if not restore:
            return
        if restore.get("show_tab"):
            tab = getattr(self, "_btn_camera_rail_show", None)
            if tab is not None:
                tab.show()
                self._position_camera_rail_show_tab()

    def _layout_native_hud(self) -> None:
        try:
            plan_on = self._plan_flight_layer_obscures_native_camera_ui()
            if plan_on:
                try:
                    self._native_rail_layer.hide()
                    self._native_hud_right.hide()
                    self._btn_camera_rail_show.hide()
                    self._native_video_preview.hide()
                    self._native_minimap_wrap.hide()
                    self._btn_native_minimap_plus.hide()
                    self._btn_native_minimap_minus.hide()
                    self._native_map_zoom_ctrl.hide()
                    self._obstacle_radar.hide()
                except Exception:
                    pass
            else:
                self._sync_camera_rail_panel_visibility()
            w = max(1, self._map_canvas.width())
            h = max(1, self._map_canvas.height())
            rail = self._native_hud_right
            ly = getattr(self, "_native_rail_layer", None)
            panel_y = int(_NATIVE_CAM_RAIL_TOP_PX)
            bottom_margin = 12
            available_h = max(120, h - panel_y - bottom_margin)
            rail.setMinimumWidth(_NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX)
            rl = rail.layout()
            if rl is not None:
                rl.activate()
            rail.updateGeometry()
            gimbal_pad_w = (
                58
                + 8
                + 3 * _CAM_RAIL_PAD_BTN_W
                + 2 * _CAM_RAIL_GIMBAL_GRID_GAP
                + 8  # pad border / grid padding
            )
            rail_margins_h = 8 + 16
            content_w = max(
                _NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX,
                int(rail.sizeHint().width()),
                int(rail.minimumSizeHint().width()),
                gimbal_pad_w + rail_margins_h,
            )
            inl, intop, inr, inb = _CAM_RAIL_LAYER_INSET
            panel_w = content_w + inl + inr
            panel_x = max(0, w - panel_w - 18)  # git `#cameraRail { right: 18px }`
            rail.setFixedWidth(content_w)
            need_h = max(120, int(rail.sizeHint().height()))
            # Never shrink below content height (that clips LENS / gimbal buttons). Shift up instead.
            content_h = need_h
            panel_h = content_h + intop + inb
            if panel_y + panel_h > h - bottom_margin:
                panel_y = max(0, h - bottom_margin - panel_h)
            if ly is not None:
                pt = self._map_canvas.mapTo(self._panel, QPoint(panel_x, panel_y))
                ly.setGeometry(pt.x(), pt.y(), panel_w, panel_h)
            rail.setGeometry(inl, intop, content_w, content_h)
            self._position_camera_rail_show_tab()
            # Git `#mapFooterHud { right: 10px; bottom: 2px }` — compass stays bottom-right of the
            # map even when `#cameraRail` is shown after MAVLink connect (rail is top-right; z-order
            # keeps HUD above the map tiles, not shoved left).
            comp_w, comp_h = 176, 176
            margin_r, margin_b = 10, 2
            cx = max(0, w - margin_r - comp_w)
            cy = max(0, h - margin_b - comp_h)
            po = self._map_canvas.mapTo(self._panel, QPoint(0, 0))
            self._native_compass.setGeometry(po.x() + cx, po.y() + cy, comp_w, comp_h)
            self._native_telemetry.updateGeometry()
            mw = self._native_telemetry.minimumSizeHint().width()
            mh = self._native_telemetry.minimumSizeHint().height()
            sw = self._native_telemetry.sizeHint().width()
            sh = self._native_telemetry.sizeHint().height()
            # Avoid clipping m/s / m / time: never cap width to a small constant; pad for border + font metrics.
            tel_w = max(mw, sw) + 20
            tel_h = max(40, max(mh, sh) + 4)
            tel_x = cx - 12 - tel_w
            if tel_x < 8:
                tel_x = 8
            tel_y = cy + (comp_h - tel_h) // 2
            self._native_telemetry.setGeometry(
                po.x() + tel_x, po.y() + tel_y, tel_w, tel_h
            )
            mar = getattr(self, "_map_action_rail", None)
            if mar is not None:
                mar_w = max(54, mar.width())
                mar_h = max(116, mar.height())
                mar.setGeometry(
                    po.x() + _MAP_ACTION_RAIL_LEFT_PX,
                    po.y() + _MAP_ACTION_RAIL_TOP_PX,
                    mar_w,
                    mar_h,
                )
            swapped = bool(getattr(self, "_video_swapped", False))
            preview_on = bool(getattr(self, "_video_preview_enabled", False))
            preview_maps = preview_on and not plan_on
            mz_ctrl = getattr(self, "_native_map_zoom_ctrl", None)
            if mz_ctrl is not None:
                try:
                    self._sync_native_map_zoom_label()
                except Exception:
                    pass
                ctrl_w = int(mz_ctrl.width()) if mz_ctrl.width() > 0 else 32
                ctrl_h = int(mz_ctrl.height()) if mz_ctrl.height() > 0 else 104
                ctrl_x = 10
                ctrl_y = max(0, h - 2 - ctrl_h)
                if preview_maps and not swapped:
                    _px, pip_y, _pw, pip_h = self._mini_video_pip_rect(w, h)
                    ctrl_y = min(ctrl_y, max(8, int(pip_y) - ctrl_h - 8))
                mz_ctrl.setGeometry(po.x() + ctrl_x, po.y() + ctrl_y, ctrl_w, ctrl_h)
                if plan_on or not bool(getattr(self, "_web_ready", False)):
                    mz_ctrl.hide()
                else:
                    mz_ctrl.show()
            _pip_x, _pip_y, _pip_w, pip_h = self._mini_video_pip_rect(w, h)
            margin = _MAP_HUD_MARGIN_PX
            # M9 obstacle radar — compact card top-left; mini-video is a small fixed PiP bottom-left.
            obr = getattr(self, "_obstacle_radar", None)
            if obr is not None and bool(getattr(self, "_last_link_connected", False)):
                obr_w = int(obr.sizeHint().width())
                obr_h = min(_OBSTACLE_PANEL_MAX_H_PX, int(obr.sizeHint().height()))
                obr_x = margin
                obr_y = _OBSTACLE_PANEL_TOP_PX
                # Keep the card above the bottom-left video PiP when both are visible.
                if preview_maps and not swapped:
                    _px, pip_y, _pw, pip_h = self._mini_video_pip_rect(w, h)
                    pip_top = int(po.y() + pip_y)
                    canvas_bottom = int(po.y() + h)
                    max_h = max(150, pip_top - int(po.y() + obr_y) - 10)
                    obr_h = min(obr_h, max_h)
                gx, gy, gw, gh = self._map_canvas_rect_on_panel(obr_x, obr_y, obr_w, obr_h)
                obr.setGeometry(gx, gy, gw, obr_h)
                if not plan_on:
                    obr.show()
            elif obr is not None:
                obr.hide()
            # PiP mode: show video only (no second minimap card — main map is the overview).
            # Fullscreen camera swap: minimap takes the **same PiP slot as the video** (bottom-left)
            # so the swap is symmetric — clicking the corner card swaps back.
            # When preview is off, keep minimap hidden (do not resurrect on resize after _stop_video_preview).
            if not preview_maps or (preview_maps and not swapped):
                self._native_minimap_wrap.hide()
            else:
                mini_x, mini_y, mini_w, mini_h = self._mini_video_pip_rect(w, h)
                mini_x = int(po.x() + mini_x)
                mini_y = int(po.y() + mini_y)
                _side = int(getattr(self, "_native_minimap_btn_side", 32))
                _pad = int(getattr(self, "_native_minimap_btn_pad", 8))
                self._native_minimap_wrap.setGeometry(mini_x, mini_y, mini_w, mini_h)
                try:
                    nmz = getattr(self, "_native_map", None)
                    if nmz is not None:
                        zmx = int(getattr(nmz, "_max_zoom", 19))
                        self._native_minimap_zoom = int(
                            max(3, min(zmx, round(float(getattr(nmz, "_zoom", 16.0))))))
                except Exception:
                    self._native_minimap_zoom = 16
                # Map fills the entire card (video-style); +/- float on top — no left gutter / strip.
                self._native_minimap.setGeometry(0, 0, mini_w, mini_h)
                self._btn_native_minimap_plus.move(_pad, _pad)
                self._btn_native_minimap_minus.move(_pad, _pad + _side + 4)
                try:
                    self._native_minimap.lower()
                except Exception:
                    pass
                self._btn_native_minimap_plus.raise_()
                self._btn_native_minimap_minus.raise_()
                self._native_minimap_wrap.show()
                self._native_minimap.show()
                self._btn_native_minimap_plus.show()
                self._btn_native_minimap_minus.show()
                try:
                    QTimer.singleShot(0, lambda: self._schedule_native_minimap_refresh(force=True))
                except Exception:
                    pass
            self._sync_native_map_vehicle_arrow_scale()
            self._stack_native_overlays_above_tile_map()
            if bool(getattr(self, "_video_swapped", False)):
                self._ensure_video_pro_hud_visible()
        except Exception:
            return

    def _sync_map_action_rail_enabled(self) -> None:
        """Match legacy `setActionButtonsEnabled`: Takeoff/Return on map when MAVLink link is up."""
        ok = bool(getattr(self, "_last_link_connected", False))
        for b in (
            getattr(self, "_map_action_takeoff_btn", None),
            getattr(self, "_map_action_return_btn", None),
        ):
            if b is not None:
                b.setEnabled(ok)

    def set_link_connected(self, connected: bool) -> None:
        c = bool(connected)
        if self._last_link_connected == c:
            return
        self._last_link_connected = c
        try:
            print(f"[VGCS:map] link_connected={c}")
        except Exception:
            pass
        self._sync_map_action_rail_enabled()
        self._run_js("setLinkConnected(true);" if c else "setLinkConnected(false);")
        if c:
            if self._video_preview_should_run() and not bool(
                getattr(self, "_video_swap_user_map_main", False)
            ):
                self._video_swapped = False
            try:
                self.clear_flight_track()
                self.set_video_follow_enabled(True)
                if getattr(self, "_lat", None) is not None and getattr(self, "_lon", None) is not None:
                    self.center_on_vehicle()
            except Exception:
                pass
        # Run a one-time tile probe after connect to log "blocked vs placeholder" clearly.
        if c and not getattr(self, "_tile_probe_ran", False):
            self._tile_probe_ran = True
            try:
                QTimer.singleShot(1500, lambda: self._probe_current_tiles(reason="connect"))
            except Exception:
                pass
        if not c:
            # Keep camera controls hidden until MAVLink link-up (heartbeat).
            try:
                self._native_hud_right.hide()
            except Exception:
                pass
            try:
                self._native_rail_layer.hide()
                self._btn_camera_rail_show.hide()
            except Exception:
                pass
            try:
                self._obstacle_radar.hide()
                self._obstacle_radar.notify_link_connected(False)
                self._lrf_lock_armed = False
                self._lrf_lock_uv = None
                self._lrf_lock_distance_m = None
                cc = getattr(self, "_camera_control", None)
                unlock = getattr(cc, "unlock_lrf", None)
                if callable(unlock):
                    unlock()
                self._refresh_lrf_lock_overlay()
            except Exception:
                pass
            # Companion RTSP (192.168.144.x) is independent of MAVLink — keep FFmpeg running;
            # only hide the PiP shell until the vehicle reconnects (CR: no split before connect).
            if self._uses_companion_rtsp() and self._video_preview_should_run():
                try:
                    self._native_video_preview.hide()
                    self._native_video_overlay.hide()
                except Exception:
                    pass
                return
            self._stop_video_preview(clear_overlay=True)
            return
        # MAVLink connected: refresh HUD geometry (rail stays visible whenever map is ready; see `_on_map_loaded`).
        try:
            if self._plan_flight_layer_obscures_native_camera_ui():
                self._native_hud_right.hide()
                self._native_rail_layer.hide()
                try:
                    self._btn_camera_rail_show.hide()
                except Exception:
                    pass
            else:
                self._sync_camera_rail_panel_visibility()
            if bool(getattr(self, "_web_ready", False)):
                self._native_compass.show()
                self._native_telemetry.show()
                mz = getattr(self, "_native_map_zoom_ctrl", None)
                if mz is not None:
                    mz.show()
                try:
                    self._obstacle_radar.show()
                    self._obstacle_radar.notify_link_connected(True)
                except Exception:
                    pass
            QTimer.singleShot(0, self._layout_native_hud)
            QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def set_flight_status(self, status: str, detail: str = "") -> None:
        st = (status or "").strip().lower()
        if st not in {"green", "yellow", "red", "idle"}:
            st = "red"
        d = str(detail)
        key = (st, d)
        if self._last_flight_status_key == key:
            return
        self._last_flight_status_key = key
        # Idle/neutral is owned by Qt `#linkBanner` styling; skip legacy JS tint when offline maps only.
        if st == "idle":
            return
        self._run_js(f"setFlightStatus({json.dumps(st)}, {json.dumps(detail)});")

    def set_header_mode(self, mode_text: str) -> None:
        t = str(mode_text)
        if t == self._last_header_mode:
            return
        self._last_header_mode = t
        self._run_js(f"setHeaderMode({json.dumps(mode_text)});")

    def set_header_vehicle_msg(self, msg_text: str) -> None:
        self._run_js(f"setHeaderVehicleMsg({json.dumps(msg_text)});")

    def set_header_gps(
        self,
        satellites: int | str,
        hdop_text: str,
        *,
        fix_type: int | None = None,
    ) -> None:
        try:
            self._gps_satellites = int(satellites)
        except Exception:
            pass
        if fix_type is not None:
            try:
                self._gps_fix_type = int(fix_type)
            except Exception:
                pass
        key = (str(satellites), str(hdop_text), str(fix_type))
        if self._last_header_gps_key == key:
            return
        self._last_header_gps_key = key
        self._run_js(f"setHeaderGps({json.dumps(str(satellites))}, {json.dumps(hdop_text)});")

    def set_header_battery(self, battery_text: str) -> None:
        bt = str(battery_text)
        if self._last_header_battery == bt:
            return
        self._last_header_battery = bt
        self._run_js(f"setHeaderBattery({json.dumps(battery_text)});")

    def set_header_remote_id(self, rid_text: str) -> None:
        self._run_js(f"setHeaderRemoteId({json.dumps(rid_text)});")

    def _layout_plan_flight_panel(self) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        try:
            w = max(1, self._map_canvas.width())
            h = max(1, self._map_canvas.height())
            origin = self._map_canvas.mapTo(self._panel, QPoint(0, 0))
            panel.setGeometry(origin.x(), origin.y(), w, h)
        except Exception:
            pass

    def _plan_flight_layer_obscures_native_camera_ui(self) -> bool:
        """True while Plan Flight covers the map — hide PiP / camera rail so planning stays uncluttered."""
        panel = getattr(self, "_plan_flight_panel", None)
        try:
            return panel is not None and panel.isVisible()
        except Exception:
            return False

    def _set_map_footer_hud_visible(self, visible: bool) -> None:
        """Mirror legacy `setPlanFlightVisible(...)` which hid the bottom compass/telemetry while planning."""
        show = bool(visible) and bool(getattr(self, "_web_ready", False))
        try:
            if hasattr(self, "_native_telemetry"):
                self._native_telemetry.setVisible(show)
            if hasattr(self, "_native_compass"):
                self._native_compass.setVisible(show)
            if hasattr(self, "_native_map_zoom_ctrl"):
                self._native_map_zoom_ctrl.setVisible(show)
            if hasattr(self, "_obstacle_radar") and bool(getattr(self, "_last_link_connected", False)):
                self._obstacle_radar.setVisible(show)
        except Exception:
            pass
