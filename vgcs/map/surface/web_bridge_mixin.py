"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import base64
import json
import time

from PySide6.QtCore import QPoint, QTimer

from vgcs.video.pipeline import notify_companion_app_background, notify_companion_app_foreground


class WebBridgeMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def on_application_background(self) -> None:
        """Windows minimize / alt-tab — avoid RTSP reconnect death spiral while hidden."""
        try:
            self._video_preview_background_mono = time.monotonic()
        except Exception:
            pass
        try:
            notify_companion_app_background()
        except Exception:
            pass

    def on_application_foreground(self) -> None:
        """Return from background — reopen companion RTSP with a clean GOP."""
        try:
            notify_companion_app_foreground()
        except Exception:
            pass
        bg = float(getattr(self, "_video_preview_background_mono", 0.0) or 0.0)
        if bg <= 0.0:
            return
        elapsed = time.monotonic() - bg
        self._video_preview_background_mono = 0.0
        if elapsed < 2.5:
            return
        if not bool(getattr(self, "_video_preview_enabled", False)):
            return
        if not self._uses_companion_rtsp():
            return
        self._refresh_companion_video_after_foreground(elapsed_bg_s=elapsed)

    def _on_web_title_changed(self, title: str) -> None:
        if title.startswith("VGCS_MAP_TILES_READY:"):
            try:
                QTimer.singleShot(0, self._ensure_native_map_visible)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_3D_OVERLAY:"):
            try:
                rest = title[len("VGCS_3D_OVERLAY:") :]
                b64 = rest.rsplit(":", 1)[0]
                raw = base64.b64decode(b64).decode("utf-8")
                items = json.loads(raw)
                self._on_3d_marker_overlay_json(items)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_3D_MAP_BEARING:"):
            try:
                parts = title.split(":")
                b = float(parts[1]) if len(parts) >= 2 else 0.0
                if bool(getattr(self, "_is_3d_mode", False)):
                    self._native_compass.set_map_bearing_deg(b)
                    self._refresh_3d_marker_overlay()
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_OBS_MAP_MARK:"):
            try:
                parts = title.split(":")
                lat = float(parts[1]) if len(parts) >= 2 else None
                lon = float(parts[2]) if len(parts) >= 3 else None
                self._log_observation("map_mark", map_lat=lat, map_lon=lon)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_OBS_VIDEO_MARK:"):
            try:
                parts = title.split(":")
                x = float(parts[1]) if len(parts) >= 2 else None
                y = float(parts[2]) if len(parts) >= 3 else None
                if x is not None and y is not None:
                    if self._handle_dooaf_video_pick(float(x), float(y)):
                        pass
                    else:
                        self._log_observation("video_mark", video_x=x, video_y=y)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_OBS_CLIP_REQUEST:"):
            try:
                self._capture_observation_clip()
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_OBS_EXPORT_REQUEST:"):
            try:
                self._export_observations()
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_OBS_CLEAR_REQUEST:"):
            try:
                self._clear_observations()
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_VIDEO_MODE_REQUEST:") or title.startswith("VGCS_CAM_VIDEO_TOGGLE:"):
            try:
                # Keep fullscreen/swap state; refresh decode without resetting layout to map PiP.
                self._start_video_preview(reset_swapped=False, force_decode=True)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_VISION_TOGGLE:"):
            try:
                if self._companion_has_dual_feed():
                    vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
                    cur = "day"
                    if vp is not None:
                        try:
                            cur = str(vp.active_source_id() or "day").strip().lower()
                        except Exception:
                            cur = "day"
                    nxt = "thermal" if cur == "day" else "day"
                    self._companion_switch_active_feed(nxt, reason="vision_toggle")
                else:
                    cur = str(getattr(self, "_video_vision_mode", "day") or "day").lower()
                    self._video_vision_mode = "night" if cur != "night" else "day"
            except Exception:
                self._video_vision_mode = "day"
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_PHOTO_REQUEST:"):
            try:
                print("[VGCS:cam_rail] PHOTO capture (shutter / legacy request)")
            except Exception:
                pass
            self._trigger_hardware_photo()
            try:
                sdir = None
                try:
                    raw = str(QSettings(_QS_NS, _QS_APP).value(_KEY_MEDIA_LAST_PHOTO_DIR, "") or "").strip()
                    if raw:
                        p = Path(raw)
                        if p.is_dir():
                            sdir = p
                except Exception:
                    sdir = None
                chosen, _ = QFileDialog.getSaveFileName(
                    self,
                    "Save photo",
                    suggested_photo_save_path(directory=sdir),
                    "Images (*.jpg *.png)",
                )
                if not chosen:
                    self._run_js("document.title = 'VGCS Map';")
                    return
                path = self._capture_photo_quick(chosen)
                if path:
                    try:
                        QSettings(_QS_NS, _QS_APP).setValue(
                            _KEY_MEDIA_LAST_PHOTO_DIR, str(Path(path).parent)
                        )
                    except Exception:
                        pass
                    name = Path(path).name
                    print(f"[VGCS:cam_rail] PHOTO saved -> {path}")
                    self._set_status(f"Photo saved: {name}")
                    self._flash_photo_feedback(ok=True, name=name)
                else:
                    print("[VGCS:cam_rail] PHOTO capture failed: no active frame")
                    self._set_status("Photo capture failed (no active frame)")
                    self._flash_photo_feedback(ok=False)
            except Exception as exc:
                try:
                    print(f"[VGCS:cam_rail] PHOTO exception: {exc!r}")
                except Exception:
                    pass
                self._flash_photo_feedback(ok=False)
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_RECORD_TOGGLE:"):
            # Format: VGCS_CAM_RECORD_TOGGLE:<0|1>:<ts>
            try:
                parts = title.split(":")
                want_on = bool(int(parts[1])) if len(parts) >= 2 else False
            except Exception:
                want_on = False
            try:
                self._ensure_video_preview_backend()
                rec_sid = self._operator_preview_source_id()
                src = self._operator_preview_video_source()
                rec = src.recorder() if src is not None and hasattr(src, "recorder") else None
                # RTSP sources use ffmpeg recording.
                if src is not None and hasattr(src, "start_recording") and hasattr(src, "stop_recording"):
                    if want_on and not bool(getattr(self, "_video_recording", False)):
                        tag = f"_{rec_sid}" if rec_sid else ""
                        tmp = Path(tempfile.gettempdir()) / (
                            f"vgcs_recording{tag}_{int(time.time())}.{self._video_record_suffix()}"
                        )
                        self._sync_payload_hardware_recording(True)
                        ok = bool(src.start_recording(str(tmp)))
                        self._video_recording = bool(ok)
                        self._video_recording_tmp_path = str(tmp) if ok else ""
                        self._video_recording_source_id = rec_sid if ok else ""
                        if ok and rec_sid:
                            try:
                                print(f"[VGCS:cam_rail] RECORD start source={rec_sid!r}")
                            except Exception:
                                pass
                        if ok:
                            self._sync_video_recording_preview_transform(rec_sid)
                            self._start_native_cam_recording_tick_timer()
                        else:
                            self._sync_payload_hardware_recording(False)
                            self._stop_native_cam_recording_tick_timer()
                    if (not want_on) and bool(getattr(self, "_video_recording", False)):
                        stop_sid = str(getattr(self, "_video_recording_source_id", "") or "").strip()
                        stop_src = self._video_source_by_id(stop_sid) if stop_sid else src
                        if stop_src is None:
                            stop_src = src
                        self._sync_payload_hardware_recording(False)
                        try:
                            if stop_src is not None:
                                stop_src.stop_recording()
                        except Exception:
                            pass
                        self._video_recording = False
                        self._video_recording_source_id = ""
                        self._stop_native_cam_recording_tick_timer()
                        tmp_path = str(getattr(self, "_video_recording_tmp_path", "") or "")
                        self._video_recording_tmp_path = ""
                        if tmp_path:
                            save_to, _ = QFileDialog.getSaveFileName(
                                self,
                                "Save recording",
                                suggested_recording_save_path(),
                                "Video (*.mp4 *.mov *.mkv)",
                            )
                            if save_to:
                                try:
                                    shutil.move(tmp_path, str(save_to))
                                except Exception:
                                    pass
                    self._run_js("document.title = 'VGCS Map';")
                    return
                if rec is None:
                    self._video_recording = False
                    self._stop_native_cam_recording_tick_timer()
                    self._run_js("document.title = 'VGCS Map';")
                    return
                if want_on and not bool(getattr(self, "_video_recording", False)):
                    tmp = Path(tempfile.gettempdir()) / (
                        f"vgcs_recording_{int(time.time())}.{self._video_record_suffix()}"
                    )
                    self._sync_payload_hardware_recording(True)
                    try:
                        rec.setOutputLocation(QUrl.fromLocalFile(str(tmp)))
                    except Exception:
                        pass
                    try:
                        rec.record()
                        self._video_recording = True
                        self._video_recording_tmp_path = str(tmp)
                        self._start_native_cam_recording_tick_timer()
                    except Exception:
                        self._video_recording = False
                        self._video_recording_tmp_path = ""
                        self._sync_payload_hardware_recording(False)
                        self._stop_native_cam_recording_tick_timer()
                if (not want_on) and bool(getattr(self, "_video_recording", False)):
                    self._sync_payload_hardware_recording(False)
                    try:
                        rec.stop()
                    except Exception:
                        pass
                    try:
                        wait_qmedia_recorder_stopped(rec, timeout_s=25.0)
                    except Exception:
                        pass
                    self._video_recording = False
                    self._stop_native_cam_recording_tick_timer()
                    tmp_path = str(getattr(self, "_video_recording_tmp_path", "") or "")
                    self._video_recording_tmp_path = ""
                    if tmp_path:
                        save_to, _ = QFileDialog.getSaveFileName(
                            self,
                            "Save recording",
                            suggested_recording_save_path(),
                            "Video (*.mp4 *.mov *.mkv)",
                        )
                        if save_to:
                            try:
                                shutil.move(tmp_path, str(save_to))
                            except Exception:
                                pass
            except Exception:
                self._video_recording = False
                self._video_recording_tmp_path = ""
                self._video_recording_source_id = ""
                self._stop_native_cam_recording_tick_timer()
            if getattr(self, "_camera_rail_ui_mode", "video") == "video":
                try:
                    self._btn_native_record.blockSignals(True)
                    self._btn_native_record.setChecked(bool(getattr(self, "_video_recording", False)))
                finally:
                    try:
                        self._btn_native_record.blockSignals(False)
                    except Exception:
                        pass
            else:
                self._sync_native_record_button_for_rail_mode()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_SPLIT_TOGGLE:"):
            # Format: VGCS_CAM_SPLIT_TOGGLE:<0|1>:<ts>
            print(f"[VGCS:cam_rail] handler SPLIT {title!r}")
            try:
                parts = title.split(":")
                self._video_split_enabled = bool(int(parts[1])) if len(parts) >= 2 else False
            except Exception:
                self._video_split_enabled = False
            try:
                if bool(getattr(self, "_video_split_enabled", False)):
                    self._ensure_video_preview_backend()
                    self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(false);")
                    self._run_js("if (window.setNativeHudMode) setNativeHudMode(false);")
                    # Do not clear fullscreen swap: split in PiP stays PiP; split in fullscreen stays fullscreen.
                    self._start_video_preview(reset_swapped=False)
                    QTimer.singleShot(0, lambda: self._apply_native_split_mode_changed(True))
                    self._push_video_preview_any_to_overlay()
                else:
                    self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(true);")
                    self._run_js("if (window.setNativeHudMode) setNativeHudMode(true);")
                    # Force UI to re-render in single mode even if the underlying frame hasn't changed.
                    self._last_video_pushed = ""
                    self._run_js("clearVideoPreviewGrid();")
                    self._run_js("setVideoPreviewMode('single');")
                    QTimer.singleShot(0, lambda: self._apply_native_split_mode_changed(False))
                    try:
                        self._push_video_preview_any_to_overlay()
                    except Exception:
                        pass
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_ZOOM_STEP:"):
            # Format: VGCS_CAM_ZOOM_STEP:<-1|1>:<ts>
            try:
                parts = title.split(":")
                step = int(parts[1]) if len(parts) >= 2 else 0
            except Exception:
                step = 0
            try:
                prev = float(getattr(self, "_video_zoom", 1.0))
            except Exception:
                prev = 1.0
            zmin, zmax, zstep = self._video_zoom_limits()
            cur = round(max(zmin, min(zmax, prev + zstep * float(step))), 1)
            if abs(cur - prev) < 1e-6:
                self._run_js("document.title = 'VGCS Map';")
                return
            self._video_zoom = cur
            try:
                self._sync_native_video_zoom_label()
            except Exception:
                pass
            try:
                self._sync_video_recording_preview_transform()
            except Exception:
                pass
            try:
                self._reapply_preview_zoom_now()
            except Exception:
                pass
            try:
                # Skydroid day: MUL optical (lens); MAVLink: ZOOM_TYPE_STEP; thermal: software crop.
                self._camera_control.handle_zoom_step(int(step), float(cur))
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_FOLLOW_TOGGLE:"):
            # Format: VGCS_CAM_FOLLOW_TOGGLE:<0|1>:<ts>
            print(f"[VGCS:cam_rail] handler FOLLOW {title!r}")
            prev = bool(getattr(self, "_video_follow_enabled", False))
            try:
                parts = title.split(":")
                self._video_follow_enabled = bool(int(parts[1])) if len(parts) >= 2 else False
                self._video_follow_last_center_mono = 0.0
            except Exception:
                self._video_follow_enabled = False
                self._video_follow_last_center_mono = 0.0
            now_en = bool(getattr(self, "_video_follow_enabled", False))
            if now_en != prev:
                self.video_follow_enabled_changed.emit(now_en)
            # Match webview: recenter as soon as follow is enabled (not only on the next throttled pose tick).
            if bool(getattr(self, "_video_follow_enabled", False)):
                try:
                    self._schedule_vehicle_pose_js(immediate=True)
                    if bool(getattr(self, "_is_3d_mode", False)):
                        self._run_js(
                            "window.__lastFollowVehLat = null; window.__lastFollowVehLon = null;"
                            "window.__vgcs3dUserInputMs = 0;"
                        )
                    self.center_on_vehicle()
                    if getattr(self, "_lat", None) is None or getattr(self, "_lon", None) is None:
                        self._set_status("Follow on (waiting for vehicle position)")
                except Exception:
                    pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_SWAP_TOGGLE:"):
            # Format: VGCS_CAM_SWAP_TOGGLE:<0|1>:<ts>
            try:
                parts = title.split(":")
                self._video_swapped = bool(int(parts[1])) if len(parts) >= 2 else False
            except Exception:
                self._video_swapped = False
            if not bool(getattr(self, "_video_swapped", False)):
                self._prepare_video_swap_layout(entering_fullscreen=False)
                self._video_swap_user_map_main = True
            else:
                self._video_swap_user_map_main = False
                self._prepare_video_swap_layout(entering_fullscreen=True)
            # Ignore Web swap state for rendering; native layer controls camera fullscreen.
            try:
                self._run_js("setVideoSwapMode(false);")
            except Exception:
                pass
            if self._video_swapped:
                self._refresh_native_overlay_insets()
            else:
                self._show_map_main_surface()
            self._layout_native_video_preview()
            if self._video_swapped:
                self._ensure_video_pro_hud_visible()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_FOCUS_STEP:"):
            # Format: VGCS_CAM_FOCUS_STEP:<-1|1>:<ts>
            try:
                parts = title.split(":")
                step = int(parts[1]) if len(parts) >= 2 else 0
            except Exception:
                step = 0
            try:
                cur = float(getattr(self, "_video_focus", 0.0))
            except Exception:
                cur = 0.0
            cur += 0.25 * float(step)
            cur = max(-5.0, min(5.0, cur))
            self._video_focus = cur
            cc = getattr(self, "_camera_control", None)
            if cc is None or isinstance(cc, NoopCameraControl):
                self._set_status("Focus disabled: camera control not connected")
            else:
                self._set_status("Focus: near" if int(step) < 0 else "Focus: far")
                try:
                    self._camera_control.handle_focus_step(int(step))
                except Exception:
                    self._set_status("Focus command failed (check camera control backend)")
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CAM_GIMBAL_CENTER:"):
            self._native_gimbal_center()
            return
        if title.startswith("VGCS_CAM_GIMBAL_NADIR:"):
            self._native_gimbal_point_down()
            return
        if title.startswith("VGCS_CAM_GIMBAL_NUDGE:"):
            # Format: VGCS_CAM_GIMBAL_NUDGE:<dx>:<dy>:<ts> — short pulse for legacy web bridge.
            try:
                parts = title.split(":")
                dx = int(parts[1]) if len(parts) >= 2 else 0
                dy = int(parts[2]) if len(parts) >= 3 else 0
            except Exception:
                dx = 0
                dy = 0
            if dx == 0 and dy == 0:
                self._native_gimbal_speed_stop()
            else:
                self._native_gimbal_speed_start(dx, dy)
                QTimer.singleShot(180, self._native_gimbal_speed_stop)
            self._run_js("document.title = 'VGCS Map';")
            return

        if title.startswith("VGCS_ASSET_ERROR:"):
            reason = title.split(":", 2)[1] if ":" in title else "asset"
            if "cesium" in reason:
                try:
                    self._btn_3d.setChecked(False)
                    self._btn_3d.setEnabled(False)
                except Exception:
                    pass
                self._set_status("3D unavailable (Cesium blocked/unreachable)")
            else:
                self._set_status("Map assets failed to load (check internet/proxy/firewall)")
            try:
                self._run_js("document.title = 'VGCS Map';")
            except Exception:
                pass
            return
        if title.startswith("VGCS_TILE_ERROR:"):
            if not getattr(self, "_tile_error_notified", False):
                self._tile_error_notified = True
                self._set_status("Tile load errors detected — use Offline Tiles… or check network/proxy")
            try:
                self._run_js("document.title = 'VGCS Map';")
            except Exception:
                pass
            return
        if title.startswith("VGCS_TILE_FALLBACK:"):
            # Esri blocked/unreachable: we auto-fell back to OSM to keep the map usable.
            self._set_status("Tiles: Esri blocked — using OpenStreetMap")
            try:
                self._run_js("document.title = 'VGCS Map';")
            except Exception:
                pass
            return
        if title.startswith("VGCS_TILE_PLACEHOLDER:"):
            # Esri returned placeholder tiles ("Map data not yet available"); switch to OSM.
            self._set_status("Tiles: Esri returned placeholders — using OpenStreetMap")
            try:
                self._run_js("document.title = 'VGCS Map';")
            except Exception:
                pass
            return
        if title.startswith("VGCS_PLAN_EXIT:"):
            self.plan_flight_exited.emit()
            self._run_js("disablePlanEditModes(); document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_PLAN_ACTION:"):
            parts = title.split(":")
            action = parts[1] if len(parts) >= 2 else ""
            if action:
                self.plan_action_requested.emit(action)
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_PLAN_TOOL_REQUEST:"):
            parts = title.split(":")
            tool = parts[1] if len(parts) >= 2 else ""
            if tool:
                self._plan_rail_tool_state = tool
            self.plan_tool_requested.emit(tool)
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_PLAN_MISSION_PANEL:"):
            try:
                raw_b64 = title.split(":", 1)[1].strip()
                payload = base64.b64decode(raw_b64).decode("utf-8")
                data = json.loads(payload)
                if isinstance(data, dict):
                    self.plan_mission_panel_changed.emit(data)
            except Exception:
                pass
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_MENU_REQUEST:"):
            parts = title.split(":")
            gx = -1
            gy = -1
            if len(parts) >= 4:
                try:
                    vx = int(parts[1])
                    vy = int(parts[2])
                    if hasattr(self, "_native_map"):
                        gp = self._native_map.mapToGlobal(QPoint(vx, vy))
                        gx, gy = int(gp.x()), int(gp.y())
                except Exception:
                    gx, gy = -1, -1
            self.menu_requested.emit(gx, gy)
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_CONNECT_REQUEST:"):
            self.connect_requested.emit()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_TAKEOFF_REQUEST:"):
            self.takeoff_requested.emit()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_RETURN_REQUEST:"):
            self.return_requested.emit()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_TOGGLE_3D_REQUEST:"):
            self.toggle_3d_requested.emit()
            self._run_js("document.title = 'VGCS Map';")
            return
        if title.startswith("VGCS_MISSION_START_REQUEST:"):
            self.mission_start_requested.emit()
            self._run_js("document.title = 'VGCS Map';")

    def _map_uses_legacy_web_bridge(self) -> bool:
        """Native Qt map is default; legacy Leaflet/WebEngine JS is optional (3D / old path)."""
        if bool(getattr(self, "_is_3d_mode", False)):
            return bool(getattr(self, "_web_ready", False))
        return getattr(self, "_native_map", None) is None and bool(getattr(self, "_web_ready", False))

    def _run_js(self, script: str, callback=None) -> None:
        # Native Qt 2D map: dispatch JS-compat commands to NativeTileMapView (default path).
        nm = getattr(self, "_native_map", None)
        if nm is not None and not bool(getattr(self, "_is_3d_mode", False)):
            try:
                self._last_tile_template = str(getattr(nm, "_tile_template", "") or "")
            except Exception:
                pass
            if callback is None:
                nm.eval_script(script)
                return
            if nm.eval_script_with_callback(script, callback):
                return
            nm.eval_script(script)
            try:
                callback(None)
            except Exception:
                pass
            return

        if not self._map_uses_legacy_web_bridge():
            if callback is not None:
                try:
                    callback(None)
                except Exception:
                    pass
            return
        if not getattr(self, "_web_ready", False):
            return
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is not None and bool(getattr(self, "_is_3d_mode", False)):
            if not bool(getattr(self, "_web_3d_ready", False)):
                return
            try:
                if callback is None:
                    # Single-arg `runJavaScript(script)` blocks the Qt GUI thread until the
                    # render process returns — with 3D + video overlay toggles this freezes the
                    # whole app ("Not Responding") on Application Settings → Apply.
                    w3.page().runJavaScript(script, lambda *_: None)
                else:
                    w3.page().runJavaScript(script, callback)
                try:
                    w3.page().runJavaScript(
                        "window.__lastTileTemplate || '';",
                        lambda v: setattr(self, "_last_tile_template", str(v or "")),
                    )
                except Exception:
                    pass
            except Exception:
                pass
            return
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            self._last_tile_template = str(getattr(nm, "_tile_template", "") or "")
        except Exception:
            pass
        if callback is None:
            nm.eval_script(script)
            return
        if nm.eval_script_with_callback(script, callback):
            return
        nm.eval_script(script)
        try:
            callback(None)
        except Exception:
            pass

    def _set_status(self, text: str) -> None:
        self._status.setText(f"Map status: {text}")

    def _schedule_vehicle_pose_js(self, *, immediate: bool) -> None:
        if immediate:
            self._vehicle_pose_timer.stop()
            self._flush_vehicle_pose_js()
            return
        if not self._vehicle_pose_timer.isActive():
            self._vehicle_pose_timer.start()

    def _flush_vehicle_pose_js(self) -> None:
        self._vehicle_pose_timer.stop()
        if self._lat is None or self._lon is None:
            return
        # Native map position is set directly in set_vehicle_position (filtered). JS is for
        # legacy WebEngine only — never push raw GPS back into NativeTileMapView via setVehicle().
        hd = float(self._heading) if self._heading is not None else 0.0
        src = self._heading_js_source or "mixed"
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is not None and bool(getattr(self, "_is_3d_mode", False)):
            lat = float(self._map_display_lat if self._map_display_lat is not None else self._lat)
            lon = float(self._map_display_lon if self._map_display_lon is not None else self._lon)
            self._run_js(
                f"setVehicle({lat:.8f}, {lon:.8f}); "
                f"updateHeading({hd:.2f}, undefined, undefined, {json.dumps(src)});"
            )
