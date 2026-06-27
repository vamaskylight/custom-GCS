"""MapWidget video mixin — see vgcs.map.video package."""

from __future__ import annotations

import base64
import json
import os
import time

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QLabel

from vgcs.map.native_video_overlay import NativeVideoOverlayLayer
from vgcs.map.surface.constants import (
    _MAP_HUD_MARGIN_PX,
    _MINI_VIDEO_PIP_H_PX,
    _MINI_VIDEO_PIP_W_PX,
)
from vgcs.map.video.helpers import _format_video_zoom_label
from vgcs.video.camera_control import (
    camera_preview_applies_digital_zoom,
    camera_recording_applies_digital_zoom,
    camera_zoom_limits,
)
from vgcs.video.pipeline import (
    VideoFrame,
    notify_companion_feed_switch,
    notify_companion_preview_motion,
    release_companion_rtsp_host,
    set_companion_decode_gate,
)


class VideoPreviewUiMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _on_native_video_click(self, event) -> None:
        try:
            if event is not None and hasattr(event, "button"):
                btn = event.button()
                if btn != Qt.MouseButton.LeftButton:
                    return
        except Exception:
            pass
        if self._dooaf_pick_complete is not None and bool(
            getattr(self, "_dooaf_pick_from_video", False)
        ):
            try:
                if event is not None:
                    event.accept()
                pos = event.position()
                xn, yn = self._native_video_click_norm(pos)
                if self._handle_dooaf_video_pick(float(xn), float(yn)):
                    return
            except Exception as e:
                print(f"[VGCS:observe] dooaf video pick failed: {e}")
                self._set_status(f"Video pick failed: {e}")
            return
        if bool(getattr(self, "_lrf_lock_armed", False)):
            try:
                if event is not None:
                    event.accept()
                pos = event.position()
                xn, yn = self._native_video_click_norm(pos)
                self._begin_c13_lrf_video_lock(float(xn), float(yn))
            except Exception as e:
                print(f"[VGCS:lrf] video pick failed: {e}")
                self._set_status(f"LRF lock failed: {e}")
            return
        if self._observation_mark_active():
            try:
                if event is not None:
                    event.accept()
                if self._lrf_lock_in_progress or self._pending_lrf_video_pick is not None:
                    self._set_status("LRF lock in progress — wait for range to confirm…")
                    return
                pos = event.position()
                xn, yn = self._native_video_click_norm(pos)
                self._log_observation("video_mark", video_x=xn, video_y=yn)
            except Exception as e:
                print(f"[VGCS:observe] video mark failed: {e}")
                self._set_status(f"Video mark failed: {e}")
            return
        was_swapped = bool(getattr(self, "_video_swapped", False))
        split_on = bool(getattr(self, "_video_split_enabled", False))
        preview_on = bool(getattr(self, "_video_preview_enabled", False))
        # Video is already fullscreen with a 2×2 layout: left-click switches which quadrant is
        # stretched (does not toggle map/video — that was wrongly clearing the pick and exiting).
        # Use the small map PiP (same corner) to swap back to map-main, unchanged.
        if was_swapped and split_on and preview_on:
            self._pick_split_fullscreen_source_from_click(event)
            QTimer.singleShot(0, self._apply_native_video_click_layout)
            try:
                self._run_js("setVideoSwapMode(false);")
            except Exception:
                pass
            return

        if was_swapped:
            self._split_fullscreen_source_id = None
        elif split_on and preview_on:
            self._pick_split_fullscreen_source_from_click(event)
        else:
            self._split_fullscreen_source_id = None

        self._video_swapped = not was_swapped
        if self._video_swapped:
            self._video_swap_user_map_main = False
            self._prepare_video_swap_layout(entering_fullscreen=True)
            self._refresh_native_overlay_insets()
        else:
            self._video_swap_user_map_main = True
            self._prepare_video_swap_layout(entering_fullscreen=False)
        QTimer.singleShot(0, self._apply_native_video_click_layout)
        # Native fullscreen toggle is fully handled in Qt. Keep Web map in map mode
        # to avoid duplicating/fragmenting video content in Web overlays/minimap grabs.
        try:
            self._run_js("setVideoSwapMode(false);")
        except Exception:
            pass

    def _apply_native_video_click_layout(self) -> None:
        """Deferred layout after video click so the GUI thread stays responsive under dual RTSP."""
        try:
            self._layout_native_video_preview()
            if bool(getattr(self, "_video_swapped", False)):
                self._ensure_video_pro_hud_visible()
            else:
                self._show_map_main_surface()
        except Exception:
            pass

    @staticmethod
    def _split_hit_slot_in_composite(u: float, v: float, snap: dict[str, object]) -> int:
        """Return slot index 0..3 for a hit inside a cell, or -1 (gap / divider)."""
        try:
            gap = int(snap.get("gap") or 6)
            cw = int(snap.get("cw") or 1)
            ch = int(snap.get("ch") or 1)
        except Exception:
            return -1
        rects = (
            (0, 0, cw, ch),
            (cw + gap, 0, cw, ch),
            (0, ch + gap, cw, ch),
            (cw + gap, ch + gap, cw, ch),
        )
        for i, (x0, y0, ww, hh) in enumerate(rects):
            if x0 <= u < x0 + ww and y0 <= v < y0 + hh:
                return i
        return -1

    def _split_slot_from_video_click(self, event) -> int:
        """Map a click on the video label to grid slot 0..3 (tl,tr,bl,br), or -1."""
        snap = getattr(self, "_split_layout_snapshot", None)
        if not isinstance(snap, dict):
            return -1
        pip = getattr(self, "_split_pip_hit", None)
        if not isinstance(pip, dict):
            return -1
        try:
            pw = float(pip.get("pw") or 0)
            ph = float(pip.get("ph") or 0)
            if pw <= 1.0 or ph <= 1.0:
                return -1
            src_w = float(pip.get("src_w") or 0)
            comp_w = float(snap.get("out_w") or 0)
            comp_h = float(snap.get("out_h") or 0)
            lx0 = float(event.position().x())
            ly0 = float(event.position().y())
            lx = lx0 - float(pip.get("cr_left", 0.0))
            ly = ly0 - float(pip.get("cr_top", 0.0))
            ox = float(pip["ox"])
            oy = float(pip["oy"])
        except Exception:
            return -1
        cx = lx - ox
        cy = ly - oy
        if cx < 0.0 or cy < 0.0 or cx >= pw or cy >= ph:
            return -1
        # Fullscreen single-channel stretch: pixmap is one stream — use screen quadrants, not
        # composite pixel coords (src_w != out_w would mis-assign every click to cell 0).
        if comp_w > 1.0 and comp_h > 1.0 and abs(src_w - comp_w) > 8.0:
            col = 0 if (cx / pw) < 0.5 else 1
            row = 0 if (cy / ph) < 0.5 else 1
            return row * 2 + col
        u = cx / pw * comp_w
        v = cy / ph * comp_h
        return self._split_hit_slot_in_composite(u, v, snap)

    def _pick_split_fullscreen_source_from_click(self, event) -> None:
        """Choose which channel fills fullscreen when leaving split PiP (click on a quadrant)."""
        slot = self._split_slot_from_video_click(event)
        snap = getattr(self, "_split_layout_snapshot", None)
        if slot < 0 or not isinstance(snap, dict):
            self._split_fullscreen_source_id = None
            return
        try:
            ids = snap.get("slot_source_ids")
            if not isinstance(ids, list) or slot >= len(ids):
                self._split_fullscreen_source_id = None
                return
            sid = ids[slot]
        except Exception:
            self._split_fullscreen_source_id = None
            return
        if not sid:
            self._split_fullscreen_source_id = None
            return
        # Remember operator intent even before the first thermal frame (avoid fullscreen 2×2 composite).
        self._split_fullscreen_source_id = str(sid)
        if self._companion_has_dual_feed():
            self._companion_switch_active_feed(str(sid), reason="split_click")

    def _pick_primary_split_source_id(self) -> str | None:
        """Best live stream for fullscreen when the operator did not pick a quadrant."""
        cache = getattr(self, "_split_last_images", None) or {}
        try:
            vp = getattr(self, "_video", None)
            src_keys = list(vp.sources().keys()) if vp is not None else []
        except Exception:
            src_keys = []
        ordered: list[str] = []
        for sid in ("day", "thermal"):
            if sid in src_keys or sid in cache:
                ordered.append(sid)
        for sid in src_keys:
            if sid not in ordered:
                ordered.append(str(sid))
        for sid in cache.keys():
            if sid not in ordered:
                ordered.append(str(sid))
        for sid in ordered:
            im = cache.get(sid)
            if isinstance(im, QImage) and not im.isNull() and im.width() > 0:
                return str(sid)
        if ordered:
            return str(ordered[0])
        lf = getattr(self, "_native_pip_last_source_frame", None)
        if isinstance(lf, QImage) and not lf.isNull() and lf.width() > 0:
            return "day"
        return None

    def _ensure_split_fullscreen_focus(self) -> str | None:
        """Never stretch the 2×2 composite to fullscreen — pick one channel instead."""
        focus = getattr(self, "_split_fullscreen_source_id", None)
        if focus:
            return str(focus)
        picked = self._pick_primary_split_source_id()
        if picked:
            self._split_fullscreen_source_id = picked
        return picked

    def _prepare_video_swap_layout(self, *, entering_fullscreen: bool) -> None:
        """Enter/leave map↔video swap: pick a single stream and repaint at display size."""
        if entering_fullscreen and bool(getattr(self, "_video_split_enabled", False)):
            self._ensure_split_fullscreen_focus()
        try:
            if entering_fullscreen:
                if bool(getattr(self, "_video_split_enabled", False)):
                    self._flush_split_preview_render()
                else:
                    im = getattr(self, "_native_pip_last_source_frame", None)
                    if not isinstance(im, QImage) or im.isNull():
                        im = getattr(self, "_native_video_last", None)
                    if isinstance(im, QImage) and not im.isNull():
                        self._render_native_video_preview(im)
            else:
                self._split_fullscreen_source_id = None
        except Exception:
            pass

    def _mini_video_pip_rect(self, w: int, h: int) -> tuple[int, int, int, int]:
        """Bottom-left PiP size in `_map_canvas` coordinates."""
        pw = min(_MINI_VIDEO_PIP_W_PX, max(200, int(w * 0.22)))
        ph = min(_MINI_VIDEO_PIP_H_PX, max(112, int(h * 0.17)))
        x = _MAP_HUD_MARGIN_PX
        y = max(0, h - ph - _MAP_HUD_MARGIN_PX)
        return x, y, pw, ph

    def _video_stream_configured(self) -> bool:
        src = str(getattr(self, "_video_settings_source", "rtsp") or "rtsp").strip().lower()
        if src == "disabled":
            return False
        day = str(getattr(self, "_video_settings_day", "") or "").strip()
        thermal = str(getattr(self, "_video_settings_thermal", "") or "").strip()
        return bool(day or thermal) or src in ("udp_h264", "udp_h265")

    def _show_mini_video_pip_shell(self) -> None:
        """Always paint the bottom-left PiP frame (even before FFmpeg / pipeline sources exist)."""
        if not self._mini_video_pip_allowed():
            return
        if not bool(getattr(self, "_web_ready", False)):
            return
        if self._plan_flight_layer_obscures_native_camera_ui():
            return
        try:
            self._read_video_settings()
        except Exception:
            pass
        self._video_preview_enabled = True
        host = self._map_canvas
        if host is None:
            return
        cw = max(1, host.width())
        ch = max(1, host.height())
        if bool(getattr(self, "_video_swapped", False)):
            px, py, pw, ph = 0, 0, cw, ch
        else:
            px, py, pw, ph = self._mini_video_pip_rect(cw, ch)
        gx, gy, gw, gh = self._map_canvas_rect_on_panel(px, py, pw, ph)
        self._native_video_preview.setGeometry(gx, gy, gw, gh)
        if self._video_stream_configured():
            hint = "Live video\n(connecting…)"
        else:
            hint = "Live video\n(Settings → Video)"
        self._set_native_video_pip_placeholder(True, message=hint)
        self._native_video_preview.show()
        self._native_video_preview.raise_()
        try:
            self._sync_native_video_overlay()
        except Exception:
            pass
        self._stack_native_overlays_above_tile_map()

    def _set_native_video_pip_placeholder(self, on: bool, *, message: str = "") -> None:
        """Hint in the PiP when preview is on but no decoded frame yet."""
        lab = getattr(self, "_native_video_preview", None)
        if lab is None:
            return
        if on and not bool(getattr(self, "_video_swapped", False)):
            lab.clear()
            lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
            txt = str(message or "").strip() or "Live video\n(connecting…)"
            lab.setText(txt)
            lab.setStyleSheet(
                "QLabel#nativeVideoPreview {"
                "background: #0c1018;"
                "color: #9fb0cc;"
                "font: 600 11px \"Segoe UI\", Arial, sans-serif;"
                "border: 1px solid rgba(206, 220, 242, 0.45);"
                "border-radius: 8px;"
                "}"
            )
        elif not on:
            lab.setText("")

    def _layout_native_video_preview(self) -> None:
        try:
            if not self._mini_video_pip_allowed():
                try:
                    self._native_video_preview.hide()
                except Exception:
                    pass
                return
            if not bool(getattr(self, "_video_preview_enabled", False)):
                return
            if self._plan_flight_layer_obscures_native_camera_ui():
                try:
                    self._native_video_preview.hide()
                except Exception:
                    pass
                return
            host = self._map_canvas
            if host is None:
                return
            cw = max(1, host.width())
            ch = max(1, host.height())
            if bool(getattr(self, "_video_swapped", False)):
                px, py, pw, ph = 0, 0, cw, ch
                self._native_video_preview.setStyleSheet(
                    "QLabel#nativeVideoPreview {"
                    "background: #000;"
                    "border: none;"
                    "border-radius: 0px;"
                    "}"
                )
                # PiP placeholder text must not persist when stretched to fullscreen.
                try:
                    self._native_video_preview.setText("")
                except Exception:
                    pass
            else:
                px, py, pw, ph = self._mini_video_pip_rect(cw, ch)
                if self._native_video_last.isNull():
                    self._set_native_video_pip_placeholder(True)
                else:
                    self._set_native_video_pip_placeholder(False)
                    self._native_video_preview.setStyleSheet(
                        "QLabel#nativeVideoPreview {"
                        "background: #000;"
                        "border: 1px solid rgba(206, 220, 242, 0.55);"
                        "border-radius: 8px;"
                        "}"
                    )
            gx, gy, gw, gh = self._map_canvas_rect_on_panel(px, py, pw, ph)
            self._native_video_preview.setGeometry(gx, gy, gw, gh)
            self._native_video_preview.show()
            self._native_video_preview.raise_()
            if bool(getattr(self, "_obs_clip_active", False)):
                try:
                    self._position_obs_clip_banner()
                except Exception:
                    pass
            # Split mode must repaint from `_split_last_images`; `_native_video_last` may still be the
            # last single-view frame and would undo the 2×2 composite after every resize.
            if bool(getattr(self, "_video_split_enabled", False)):
                # Do not use QLabel `scaledContents` for split: it can hide grid/labels; we scale in
                # `_render_native_video_preview` so the full 2×2 composite is always visible.
                try:
                    self._native_video_preview.setScaledContents(False)
                except Exception:
                    pass
                self._render_native_split_preview()
            else:
                try:
                    self._native_video_preview.setScaledContents(False)
                except Exception:
                    pass
                if not self._native_video_last.isNull():
                    self._render_native_video_preview(self._native_video_last)
            # Reposition HUD (minimap vs PiP) after video geometry is known.
            self._layout_native_hud()
            if bool(getattr(self, "_video_swapped", False)):
                # Fullscreen: camera rail must stay above video (never hide under obs marks).
                self._ensure_video_pro_hud_visible()
            else:
                self._raise_flight_hud_above_video()
        except Exception:
            return
        finally:
            try:
                self._sync_native_camera_rail_toggles()
            except Exception:
                pass
            try:
                self._sync_native_video_overlay()
            except Exception:
                pass

    def _ensure_video_pro_hud_visible(self) -> None:
        """Video Pro (fullscreen video): keep camera rail, compass, telemetry, and action buttons visible."""
        if not bool(getattr(self, "_web_ready", False)):
            return
        if self._plan_flight_layer_obscures_native_camera_ui():
            return
        try:
            self._native_compass.show()
            self._native_telemetry.show()
            mz = getattr(self, "_native_map_zoom_ctrl", None)
            if mz is not None:
                mz.show()
        except Exception:
            pass
        if bool(getattr(self, "_last_link_connected", False)):
            try:
                self._sync_camera_rail_panel_visibility()
                self._obstacle_radar.show()
            except Exception:
                pass
        mar = getattr(self, "_map_action_rail", None)
        if mar is not None:
            try:
                mar.show()
            except Exception:
                pass
        self._raise_flight_hud_above_video()

    def _raise_flight_hud_above_video(self) -> None:
        """Stack Takeoff/Return, camera rail, compass, telemetry, obstacle, minimap above fullscreen video."""
        try:
            for w in (
                getattr(self, "_map_action_rail", None),
                getattr(self, "_map_action_takeoff_btn", None),
                getattr(self, "_map_action_return_btn", None),
            ):
                if w is not None and w.isVisible():
                    w.raise_()
            ly = getattr(self, "_native_rail_layer", None)
            if ly is not None and ly.isVisible():
                ly.raise_()
                try:
                    self._native_hud_right.raise_()
                except Exception:
                    pass
            tab = getattr(self, "_btn_camera_rail_show", None)
            if tab is not None and tab.isVisible():
                tab.raise_()
            obr = getattr(self, "_obstacle_radar", None)
            if obr is not None and obr.isVisible():
                obr.raise_()
            for hud in (self._native_compass, self._native_telemetry, getattr(self, "_native_map_zoom_ctrl", None)):
                if hud is not None and hud.isVisible():
                    hud.raise_()
            preview_on = bool(getattr(self, "_video_preview_enabled", False))
            swapped = bool(getattr(self, "_video_swapped", False))
            if preview_on and swapped:
                wrap = getattr(self, "_native_minimap_wrap", None)
                if wrap is not None and wrap.isVisible():
                    wrap.raise_()
                    self._btn_native_minimap_plus.raise_()
                    self._btn_native_minimap_minus.raise_()
            ov = getattr(self, "_native_video_overlay", None)
            if ov is not None and ov.isVisible():
                ov.raise_()
            # Target ON (map-main PiP only): lift video above the map so PiP clicks reach the preview.
            # Fullscreen swap keeps the camera rail above video — do not raise video over the rail.
            if (self._observation_mark_active() or bool(getattr(self, "_lrf_lock_armed", False))) and preview_on and not swapped:
                pv = getattr(self, "_native_video_preview", None)
                if pv is not None and pv.isVisible():
                    pv.raise_()
            if ov is not None and ov.isVisible():
                ov.raise_()
        except Exception:
            pass

    def _video_zoom_limits(self) -> tuple[float, float, float]:
        return camera_zoom_limits(getattr(self, "_camera_control", None))

    def _effective_preview_digital_zoom(self, source_id: str | None = None) -> float:
        sid = str(source_id or "").strip()
        if not sid:
            try:
                sid = self._operator_preview_source_id()
            except Exception:
                sid = ""
        if not camera_preview_applies_digital_zoom(
            getattr(self, "_camera_control", None),
            sid,
        ):
            return 1.0
        try:
            return float(getattr(self, "_video_zoom", 1.0))
        except Exception:
            return 1.0

    def _apply_video_recording_preview_transform(self, source_id: str) -> None:
        sid = str(source_id or "").strip()
        if not sid:
            return
        src = self._video_source_by_id(sid)
        if src is None or not hasattr(src, "set_recording_preview_transform"):
            return
        try:
            src.set_recording_preview_transform(
                digital_zoom=self._effective_preview_digital_zoom(sid),
                apply_digital_zoom=camera_recording_applies_digital_zoom(
                    sid,
                    getattr(self, "_camera_control", None),
                ),
            )
        except Exception:
            pass

    def _sync_video_recording_preview_transform(self, source_id: str | None = None) -> None:
        """Keep RTSP recording frames aligned with on-screen preview (thermal software zoom)."""
        if not bool(getattr(self, "_video_recording", False)):
            return
        sid = str(source_id or getattr(self, "_video_recording_source_id", "") or "").strip()
        if not sid:
            sid = self._operator_preview_source_id()
        self._apply_video_recording_preview_transform(sid)

    def _sync_native_video_zoom_label(self) -> None:
        lbl = getattr(self, "_lbl_camera_top_zoom", None)
        if lbl is None:
            return
        try:
            z = float(getattr(self, "_video_zoom", 1.0))
        except Exception:
            z = 1.0
        try:
            lbl.setText(_format_video_zoom_label(z))
        except Exception:
            pass

    def _retry_native_video_pixmap(self) -> None:
        """Re-run paint after layout; avoids stale single-view pixmap when split is on."""
        try:
            if bool(getattr(self, "_video_split_enabled", False)):
                self._render_native_split_preview()
                return
            im = getattr(self, "_native_video_last", None)
            if isinstance(im, QImage) and not im.isNull():
                self._render_native_video_preview(im)
        except Exception:
            pass

    def _native_video_click_mirror_x(self) -> bool:
        """Optional horizontal flip for click coords (off by default — reticle and gimbal share same u,v)."""
        env = str(os.environ.get("VGCS_VIDEO_MIRROR_X", "") or "").strip().lower()
        return env in ("1", "true", "yes", "on")

    def _native_video_click_norm(self, pos: QPointF) -> tuple[float, float]:
        """Normalized click (0..1) on the visible video pixmap; falls back to full label size."""
        try:
            cr = self._native_video_content_rect()
            if cr:
                left = float(cr.get("cr_left", 0.0)) + float(cr.get("ox", 0.0))
                top = float(cr.get("cr_top", 0.0)) + float(cr.get("oy", 0.0))
                pw = max(1.0, float(cr.get("pw", 1.0)))
                ph = max(1.0, float(cr.get("ph", 1.0)))
                xn = (float(pos.x()) - left) / pw
                yn = (float(pos.y()) - top) / ph
                if self._native_video_click_mirror_x():
                    xn = 1.0 - xn
                return (
                    max(0.0, min(1.0, xn)),
                    max(0.0, min(1.0, yn)),
                )
        except Exception:
            pass
        w = max(1, int(self._native_video_preview.width()))
        h = max(1, int(self._native_video_preview.height()))
        xn = float(pos.x()) / float(w)
        yn = float(pos.y()) / float(h)
        if self._native_video_click_mirror_x():
            xn = 1.0 - xn
        return (
            max(0.0, min(1.0, xn)),
            max(0.0, min(1.0, yn)),
        )

    def _native_video_content_rect(self) -> dict[str, float] | None:
        """Logical rect of the scaled video pixmap inside ``_native_video_preview`` (for overlays)."""
        hit = getattr(self, "_split_pip_hit", None)
        if isinstance(hit, dict) and hit:
            return dict(hit)
        try:
            pv = self._native_video_preview
            pm = pv.pixmap()
            if pm is None or pm.isNull():
                return None
            cr = pv.contentsRect()
            Wc = float(max(1, cr.width()))
            Hc = float(max(1, cr.height()))
            spw = float(max(1, pm.width()))
            sph = float(max(1, pm.height()))
            try:
                dpr = max(1.0, float(pm.devicePixelRatio()))
            except Exception:
                dpr = 1.0
            spw /= dpr
            sph /= dpr
            return {
                "cr_left": float(cr.left()),
                "cr_top": float(cr.top()),
                "ox": (Wc - spw) / 2.0,
                "oy": (Hc - sph) / 2.0,
                "pw": spw,
                "ph": sph,
            }
        except Exception:
            return None

    def _sync_native_video_overlay(self, *, from_lrf: bool = False) -> None:
        """Match overlay layer to video preview geometry and refresh content rect."""
        try:
            pv = self._native_video_preview
            ly = self._native_video_overlay
        except AttributeError:
            return
        try:
            ly.setGeometry(0, 0, max(1, pv.width()), max(1, pv.height()))
            ly.set_content_rect(self._native_video_content_rect())
            if pv.isVisible() and bool(getattr(self, "_video_preview_enabled", False)):
                ly.show()
                ly.raise_()
            else:
                ly.hide()
            if not from_lrf and self._lrf_reticle_tracking_active():
                self._refresh_lrf_lock_overlay(sync_geometry=False)
            elif getattr(self, "_lrf_lock_uv", None) is not None or bool(
                getattr(self, "_lrf_lock_armed", False)
            ):
                ly.update()
        except Exception:
            pass

    def _render_native_video_preview(self, img: QImage) -> None:
        if img is None or img.isNull():
            return
        self._set_native_video_pip_placeholder(False)
        self._native_video_last = img
        split_on = bool(getattr(self, "_video_split_enabled", False))
        try:
            if not split_on:
                self._native_video_preview.setScaledContents(False)
        except Exception:
            pass
        try:
            pm = QPixmap.fromImage(img)
            if pm.isNull():
                return
            size = self._native_video_preview.size()
            if size.width() <= 0 or size.height() <= 0:
                QTimer.singleShot(0, self._retry_native_video_pixmap)
                return
            swap_on = bool(getattr(self, "_video_swapped", False))
            ar_mode = (
                Qt.AspectRatioMode.IgnoreAspectRatio
                if swap_on
                else Qt.AspectRatioMode.KeepAspectRatio
            )
            scaled_pm = pm.scaled(size, ar_mode, Qt.TransformationMode.FastTransformation)
            self._native_video_preview.setPixmap(scaled_pm)
            # Record PiP layout for split-grid hit-testing (must match paint; pixmap().size() alone
            # mis-maps clicks on HiDPI — device pixels vs logical coords skew u,v into cell 0 / day).
            if split_on and not swap_on:
                try:
                    cr = self._native_video_preview.contentsRect()
                    Wc = float(max(1, cr.width()))
                    Hc = float(max(1, cr.height()))
                    spw = float(max(1, scaled_pm.width()))
                    sph = float(max(1, scaled_pm.height()))
                    try:
                        dpr = max(1.0, float(scaled_pm.devicePixelRatio()))
                    except Exception:
                        dpr = 1.0
                    spw /= dpr
                    sph /= dpr
                    self._split_pip_hit = {
                        "cr_left": float(cr.left()),
                        "cr_top": float(cr.top()),
                        "ox": (Wc - spw) / 2.0,
                        "oy": (Hc - sph) / 2.0,
                        "pw": spw,
                        "ph": sph,
                        "src_w": float(img.width()),
                        "src_h": float(img.height()),
                    }
                except Exception:
                    self._split_pip_hit = None
            elif split_on and swap_on:
                # Fullscreen 2×2 or single-channel stretch: record layout for quadrant hit-testing.
                try:
                    cr = self._native_video_preview.contentsRect()
                    Wc = float(max(1, cr.width()))
                    Hc = float(max(1, cr.height()))
                    spw = float(max(1, scaled_pm.width()))
                    sph = float(max(1, scaled_pm.height()))
                    try:
                        dpr = max(1.0, float(scaled_pm.devicePixelRatio()))
                    except Exception:
                        dpr = 1.0
                    spw /= dpr
                    sph /= dpr
                    self._split_pip_hit = {
                        "cr_left": float(cr.left()),
                        "cr_top": float(cr.top()),
                        "ox": (Wc - spw) / 2.0,
                        "oy": (Hc - sph) / 2.0,
                        "pw": spw,
                        "ph": sph,
                        "src_w": float(img.width()),
                        "src_h": float(img.height()),
                    }
                except Exception:
                    self._split_pip_hit = None
            else:
                self._split_pip_hit = None
        except Exception:
            return
        finally:
            try:
                self._sync_native_video_overlay()
            except Exception:
                pass

    def _schedule_split_preview_render(self) -> None:
        """Coalesce 2×2 composite paints (~25 Hz) so dual RTSP does not freeze the GUI."""
        if not bool(getattr(self, "_video_split_enabled", False)):
            return
        if not bool(getattr(self, "_video_preview_enabled", False)):
            return
        now = time.monotonic()
        gap = 0.04
        last = float(getattr(self, "_split_ui_render_mono", 0.0) or 0.0)
        t = getattr(self, "_split_render_timer", None)
        if t is None:
            self._split_ui_render_mono = now
            self._render_native_split_preview()
            return
        if now - last >= gap:
            t.stop()
            self._split_ui_render_mono = now
            self._render_native_split_preview()
            return
        if not t.isActive():
            t.start(max(1, int((gap - (now - last)) * 1000)))

    def _flush_split_preview_render(self) -> None:
        self._split_ui_render_mono = time.monotonic()
        try:
            self._render_native_split_preview()
        except Exception:
            pass

    def _render_split_fullscreen_waiting(self, source_id: str) -> None:
        """Fullscreen single channel before the first decoded frame (avoid painting a 2×2 grid)."""
        label = self._split_cell_label(source_id) or str(source_id)
        try:
            w = max(320, int(self._native_video_preview.width() or 640))
            h = max(180, int(self._native_video_preview.height() or 360))
        except Exception:
            w, h = 640, 360
        try:
            out = QImage(w, h, QImage.Format.Format_RGB32)
            out.fill(QColor(10, 13, 20))
            p = QPainter(out)
            p.setPen(QColor(180, 200, 230))
            p.setFont(QFont("Segoe UI", 14, QFont.Weight.DemiBold))
            p.drawText(
                out.rect(),
                Qt.AlignmentFlag.AlignCenter,
                f"{label}\nWaiting for stream…",
            )
            p.end()
            self._render_native_video_preview(out)
        except Exception:
            pass

    def _render_native_split_preview(self) -> None:
        """
        Git `setVideoPreviewGrid` parity: 4 cells in id order (day, thermal, …); empty cells stay dark.

        Single-source (typical `thermal=''`) shows the live feed in **cell 1 only**; cells 2–4 are
        dark placeholders labeled `Empty`. The earlier code duplicated the same frame four times,
        which looked identical to single mode on the SMPTE test pattern.
        """
        if not bool(getattr(self, "_video_split_enabled", False)):
            return
        if not self._mini_video_pip_allowed():
            return
        # Fullscreen: one live channel only — never upscale the PiP 2×2 composite.
        if bool(getattr(self, "_video_swapped", False)):
            focus = self._ensure_split_fullscreen_focus()
            if focus:
                cache0 = getattr(self, "_split_last_images", None) or {}
                im0 = cache0.get(str(focus))
                if isinstance(im0, QImage) and not im0.isNull() and im0.width() > 0:
                    try:
                        self._render_native_video_preview(im0)
                    except Exception:
                        pass
                    return
                lf = getattr(self, "_native_pip_last_source_frame", None)
                if isinstance(lf, QImage) and not lf.isNull() and lf.width() > 0:
                    try:
                        self._render_native_video_preview(lf)
                    except Exception:
                        pass
                    return
                try:
                    self._render_split_fullscreen_waiting(str(focus))
                except Exception:
                    pass
                return
            lf = getattr(self, "_native_pip_last_source_frame", None)
            if isinstance(lf, QImage) and not lf.isNull() and lf.width() > 0:
                try:
                    self._render_native_video_preview(lf)
                except Exception:
                    pass
                return
            return
        cache = getattr(self, "_split_last_images", None) or {}
        keys: list[str] = []
        try:
            vp = getattr(self, "_video", None)
            if vp is not None:
                src_keys = list(vp.sources().keys())
            else:
                src_keys = list(cache.keys())
        except Exception:
            src_keys = list(cache.keys())
        for k in ("day", "thermal"):
            if k in src_keys and k not in keys:
                keys.append(k)
        for k in src_keys:
            if k not in keys:
                keys.append(k)
        if not keys:
            keys = list(cache.keys())
        keys = keys[:4]

        ordered: list[tuple[str | None, QImage | None]] = []
        for sid in keys:
            im = cache.get(sid)
            if isinstance(im, QImage) and not im.isNull() and im.width() > 0:
                ordered.append((sid, im))
            else:
                ordered.append((sid, None))
        # Fallback: if there is no cached frame yet for the active source, fill cell 1 with the latest single-view frame.
        if all(im is None for _, im in ordered):
            lf = getattr(self, "_native_pip_last_source_frame", None)
            if isinstance(lf, QImage) and not lf.isNull() and lf.width() > 0:
                if ordered:
                    ordered[0] = (ordered[0][0] or "day", lf)
                else:
                    ordered.append(("day", lf))
        while len(ordered) < 4:
            ordered.append((None, None))
        self._render_native_split_grid_4(ordered)

    @staticmethod
    def _split_cell_label(source_id: str | None) -> str:
        if not source_id:
            return ""
        sid = str(source_id).strip().lower()
        if sid == "day":
            return "Day"
        if sid == "thermal":
            return "Thermal"
        return str(source_id)

    def _render_native_split_grid_4(self, cells: list[tuple[str | None, QImage | None]]) -> None:
        """Draw a clear 2×2 grid; filled cells show the feed + label, empty cells show 'Empty'."""
        try:
            gap = 6
            heights = [im.height() for _, im in cells if isinstance(im, QImage) and not im.isNull() and im.height() > 0]
            ref_h = max(heights) if heights else 360
            ch_target = max(120, min(540, ref_h)) // 2
            widths = [im.width() for _, im in cells if isinstance(im, QImage) and not im.isNull() and im.width() > 0]
            ref_w = max(widths) if widths else 640
            cw_target = max(160, min(960, ref_w)) // 2

            cw = int(cw_target)
            ch = int(ch_target)
            out_w = cw * 2 + gap
            out_h = ch * 2 + gap
            out = QImage(out_w, out_h, QImage.Format.Format_RGB32)
            out.fill(QColor(10, 13, 20))
            p = QPainter(out)
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
            positions = ((0, 0), (cw + gap, 0), (0, ch + gap), (cw + gap, ch + gap))
            slot_numbers = ("1", "2", "3", "4")

            for slot, (dx, dy), (sid, im) in zip(slot_numbers, positions, cells):
                p.fillRect(int(dx), int(dy), int(cw), int(ch), QColor(8, 10, 16))
                if isinstance(im, QImage) and not im.isNull() and im.width() > 0 and im.height() > 0:
                    scaled = im.scaled(
                        cw,
                        ch,
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.FastTransformation,
                    )
                    x0 = dx + max(0, (cw - scaled.width()) // 2)
                    y0 = dy + max(0, (ch - scaled.height()) // 2)
                    p.drawImage(x0, y0, scaled)
                    label = self._split_cell_label(sid)
                    tag = f"{slot} · {label}" if label else slot
                    self._draw_split_cell_label(p, int(dx), int(dy), int(cw), tag, filled=True)
                else:
                    try:
                        p.setPen(QPen(QColor(80, 96, 124, 160), 1, Qt.PenStyle.DashLine))
                        p.drawRect(int(dx) + 2, int(dy) + 2, int(cw) - 4, int(ch) - 4)
                    except Exception:
                        pass
                    empty_label = f"Cell {slot}\nEmpty"
                    if (
                        sid == "thermal"
                        and self._companion_has_dual_feed()
                        and self._uses_companion_rtsp()
                    ):
                        empty_label = "Cell 2\nThermal (IR)"
                    try:
                        font_e = QFont("Segoe UI", 14, QFont.Weight.DemiBold)
                        p.setFont(font_e)
                        p.setPen(QColor(140, 156, 188))
                        p.drawText(
                            int(dx),
                            int(dy),
                            int(cw),
                            int(ch),
                            Qt.AlignmentFlag.AlignCenter,
                            empty_label,
                        )
                    except Exception:
                        pass
                    self._draw_split_cell_label(p, int(dx), int(dy), int(cw), slot, filled=False)

            try:
                p.setPen(QPen(QColor(232, 240, 255), 3))
                gx = cw + gap // 2
                gy = ch + gap // 2
                p.drawLine(gx, 0, gx, out_h)
                p.drawLine(0, gy, out_w, gy)
            except Exception:
                pass
            p.end()
            slot_ids: list[str | None] = []
            for sid, _im in cells[:4]:
                slot_ids.append(str(sid) if sid else None)
            while len(slot_ids) < 4:
                slot_ids.append(None)
            self._split_layout_snapshot = {
                "gap": gap,
                "cw": cw,
                "ch": ch,
                "out_w": out_w,
                "out_h": out_h,
                "slot_source_ids": slot_ids[:4],
            }
            self._render_native_video_preview(out)
        except Exception:
            return

    @staticmethod
    def _draw_split_cell_label(p: QPainter, x: int, y: int, cw: int, text: str, *, filled: bool) -> None:
        try:
            font = QFont("Segoe UI", 11, QFont.Weight.Bold)
            p.setFont(font)
            pad_x = 8
            pad_y = 4
            metrics = p.fontMetrics()
            tw = min(cw - 12, metrics.horizontalAdvance(text) + pad_x * 2)
            th = metrics.height() + pad_y
            bx = x + 6
            by = y + 6
            p.fillRect(bx, by, tw, th, QColor(0, 0, 0, 200))
            p.setPen(QColor(140, 230, 175) if filled else QColor(180, 200, 230))
            p.drawText(bx + pad_x, by + metrics.ascent() + pad_y // 2, text)
        except Exception:
            pass

    def _best_native_preview_frame(self) -> QImage | None:
        """Latest frame suitable for single-view repaint or split cache seed."""
        for candidate in (
            getattr(self, "_native_pip_last_source_frame", None),
            getattr(self, "_native_video_last", None),
        ):
            if isinstance(candidate, QImage) and not candidate.isNull() and candidate.width() > 0:
                return candidate
        cache = getattr(self, "_split_last_images", None) or {}
        for sid in ("day", "thermal"):
            im = cache.get(sid)
            if isinstance(im, QImage) and not im.isNull() and im.width() > 0:
                return im
        for im in cache.values():
            if isinstance(im, QImage) and not im.isNull() and im.width() > 0:
                return im
        return None

    def _apply_native_split_mode_changed(self, enabled: bool) -> None:
        """Repaint native PiP/fullscreen after SPLIT rail toggle (single ↔ 2×2)."""
        if not isinstance(getattr(self, "_split_last_images", None), dict):
            self._split_last_images = {}
        if enabled:
            vp = getattr(self, "_video", None)
            if vp is not None and bool(getattr(self, "_video_preview_enabled", False)):
                try:
                    self._connect_video_pipeline_frame_slots(vp)
                    self._start_video_decode_sources(vp)
                except Exception:
                    pass
            self._seed_split_cache_from_last_frame()
            try:
                self._layout_native_video_preview()
                self._render_native_split_preview()
                QTimer.singleShot(0, self._retry_native_video_pixmap)
            except Exception:
                pass
            return
        self._split_fullscreen_source_id = None
        self._split_layout_snapshot = None
        self._split_pip_hit = None
        try:
            vp0 = getattr(self, "_video", None)
            if vp0 is not None:
                self._stop_idle_video_decode_sources(vp0)
        except Exception:
            pass
        im = self._best_native_preview_frame()
        try:
            self._layout_native_video_preview()
            if isinstance(im, QImage) and not im.isNull():
                self._native_video_last = im
                self._set_native_video_pip_placeholder(False)
                self._render_native_video_preview(im)
            elif self._native_video_last.isNull():
                self._set_native_video_pip_placeholder(True)
        except Exception:
            pass

    def _seed_split_cache_from_last_frame(self) -> None:
        """After enabling split, paint immediately from the last single-view frame (avoids blank until next tick)."""
        if not bool(getattr(self, "_video_split_enabled", False)):
            return
        try:
            im = self._best_native_preview_frame()
            if not isinstance(im, QImage) or im.isNull():
                return
            c = getattr(self, "_split_last_images", None)
            if c is None:
                self._split_last_images = {}
                c = self._split_last_images
            vp = getattr(self, "_video", None)
            if vp is not None:
                keys = list(vp.sources().keys())
                if "day" in keys:
                    c["day"] = im.copy()
                elif "thermal" in keys:
                    c["thermal"] = im.copy()
                elif keys:
                    c[str(keys[0])] = im.copy()
                else:
                    c["day"] = im.copy()
            else:
                c["day"] = im.copy()
        except Exception:
            pass

    def _on_mavlink_link_show_mini_video(self) -> None:
        """MAVLink connected: show PiP shell immediately, then start decode when possible."""
        if not bool(getattr(self, "_last_link_connected", False)):
            return
        self._show_mini_video_pip_shell()
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is not None:
            try:
                self._stop_idle_video_decode_sources(vp)
            except Exception:
                pass
        if self._video_preview_should_run():
            self._auto_start_mini_video_pip(force_decode=False, preserve_layout=True)
            QTimer.singleShot(
                800,
                lambda: self._companion_start_decode_if_needed(reason="mavlink_link"),
            )

    def _companion_has_dual_feed(self) -> bool:
        if not self._uses_companion_rtsp():
            return False
        try:
            self._read_video_settings()
        except Exception:
            pass
        day = str(getattr(self, "_video_settings_day", "") or "").strip()
        thermal = str(getattr(self, "_video_settings_thermal", "") or "").strip()
        if bool(day) and bool(thermal):
            return True
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is not None:
            try:
                if "thermal" in vp.sources():
                    th = vp.sources().get("thermal")
                    url = str(getattr(th, "_url", "") or "").strip()
                    if url:
                        return True
            except Exception:
                pass
        return False

    def _companion_show_ir_button(self) -> bool:
        """Show IR toggle on Skydroid / 192.168.144.x companion links."""
        try:
            self._read_video_settings()
        except Exception:
            pass
        return self._uses_companion_rtsp()

    def _companion_switch_active_feed(self, source_id: str, *, reason: str = "") -> None:
        """C13: swap day ↔ thermal RTSP (one client). Gimbal UDP is unaffected."""
        sid = str(source_id or "").strip().lower()
        if sid not in ("day", "thermal"):
            return
        if not self._companion_has_dual_feed():
            return
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            return
        try:
            sources = vp.sources()
        except Exception:
            return
        if sid not in sources:
            return
        try:
            cur = str(vp.active_source_id() or "").strip().lower()
        except Exception:
            cur = ""
        if cur == sid:
            self._sync_native_thermal_feed_button()
            return
        try:
            print(f"[VGCS:video] companion feed switch {cur!r} -> {sid!r} ({reason})")
        except Exception:
            pass
        try:
            notify_companion_feed_switch(duration_s=12.0)
            notify_companion_preview_motion(duration_s=8.0)
        except Exception:
            pass
        for _osid, src in sources.items():
            try:
                if hasattr(src, "companion_hard_stop_decode"):
                    src.companion_hard_stop_decode(join_s=2.5)
                else:
                    src.stop()
                    release_companion_rtsp_host(
                        str(getattr(src, "source_id", "") or ""),
                        str(getattr(src, "_url", "") or ""),
                    )
            except Exception:
                pass
        try:
            release_all_companion_rtsp_hosts()
        except Exception:
            pass
        self._split_fullscreen_source_id = sid
        self._native_pip_last_source_frame = QImage()
        self._last_video_pushed = ""
        try:
            self._video_gui_logged_frame = False
        except Exception:
            pass

        def _finish_switch() -> None:
            vp2 = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
            if vp2 is None:
                return
            try:
                vp2.set_active_source(sid)
                self._video_active_source = vp2.active_source()
            except Exception:
                pass
            try:
                self._connect_video_pipeline_frame_slots(vp2)
            except Exception:
                pass
            try:
                active_now = str(vp2.active_source_id() or "").strip()
                print(
                    f"[VGCS:video] companion feed switch done: active={active_now!r} "
                    f"(want={sid!r})"
                )
            except Exception:
                pass
            try:
                self._start_video_decode_sources(vp2)
            except Exception:
                pass
            self._sync_native_thermal_feed_button()
            label = "Thermal" if sid == "thermal" else "Day"
            self._set_status(
                f"Camera feed: {label} "
                "(C13 — one RTSP stream; gimbal unchanged)"
            )
            try:
                if bool(getattr(self, "_video_swapped", False)):
                    self._render_split_fullscreen_waiting(sid)
                elif not bool(getattr(self, "_video_split_enabled", False)):
                    self._native_video_preview.setPixmap(QPixmap())
                    self._native_video_last = QImage()
                else:
                    self._flush_split_preview_render()
            except Exception:
                pass

        QTimer.singleShot(2800, _finish_switch)

    def _sync_native_thermal_feed_button(self) -> None:
        btn = getattr(self, "_btn_native_thermal", None)
        if btn is None:
            return
        show = self._companion_show_ir_button()
        dual = self._companion_has_dual_feed()
        try:
            btn.setVisible(bool(show))
            btn.setEnabled(bool(dual))
            if dual:
                btn.setToolTip(
                    "Thermal IR feed (C13: switches day ↔ thermal — one RTSP stream at a time; "
                    "gimbal unchanged)"
                )
            else:
                btn.setToolTip(
                    "Thermal IR — set both day and thermal RTSP URLs in "
                    "Application Settings → Video → Apply"
                )
        except Exception:
            pass
        if not show:
            return
        if not dual:
            try:
                print(
                    "[VGCS:video] IR button visible but disabled — configure thermal RTSP URL "
                    f"(day={getattr(self, '_video_settings_day', '')!r} "
                    f"thermal={getattr(self, '_video_settings_thermal', '')!r})"
                )
            except Exception:
                pass
            try:
                btn.blockSignals(True)
                btn.setChecked(False)
            finally:
                try:
                    btn.blockSignals(False)
                except Exception:
                    pass
            return
        active = "day"
        vp = getattr(self, "_video", None)
        if vp is not None:
            try:
                active = str(vp.active_source_id() or "day").strip().lower()
            except Exception:
                active = "day"
        try:
            btn.blockSignals(True)
            btn.setChecked(active == "thermal")
        finally:
            try:
                btn.blockSignals(False)
            except Exception:
                pass

    def _on_native_thermal_feed_toggled(self, on: bool) -> None:
        if not self._companion_has_dual_feed():
            self._sync_native_thermal_feed_button()
            self._set_status(
                "Thermal IR — set day + thermal RTSP in Application Settings → Video → Apply"
            )
            return
        target = "thermal" if bool(on) else "day"
        self._companion_switch_active_feed(target, reason="ir_button")

    def _companion_video_decode_gate(self, source_id: str) -> bool:
        """C13 allows one RTSP client — decode only sources needed for current preview mode."""
        if not self._mini_video_pip_allowed():
            return False
        if not self._video_preview_should_run():
            return False
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            return str(source_id or "").strip() == "day"
        try:
            want = set(self._video_preview_source_ids_to_run(vp))
        except Exception:
            return str(source_id or "").strip() == "day"
        return str(source_id or "").strip() in want

    def _sync_video_split_from_settings(self) -> None:
        try:
            self._read_video_settings()
            self._video_split_enabled = (
                str(getattr(self, "_video_settings_default_view", "Single") or "Single")
                .strip()
                .lower()
                == "split"
            )
        except Exception:
            pass

    def _uses_companion_rtsp(self) -> bool:
        day = str(getattr(self, "_video_settings_day", "") or "").strip().lower()
        return "192.168.144." in day

    def _should_defer_companion_rtsp_decode(self) -> bool:
        """Wait for MAVLink link before opening RTSP on 192.168.144.x (often unreachable at boot)."""
        if not self._uses_companion_rtsp():
            return False
        return not bool(getattr(self, "_last_link_connected", False))

    def _companion_decode_running(self, vp) -> bool:
        try:
            for sid in self._video_preview_source_ids_to_run(vp):
                src = vp.sources().get(sid)
                if src is None:
                    continue
                if not getattr(src, "_running", False):
                    continue
                th = getattr(src, "_ffmpeg_thread", None)
                if th is not None and th.is_alive():
                    return True
        except Exception:
            pass
        return False

    def _companion_wire_preview_ui(self) -> bool:
        """Connect frame slots and show the native video overlay (no FFmpeg stop/start)."""
        if not self._mini_video_pip_allowed():
            return False
        if not self._video_preview_should_run():
            return False
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            return False
        try:
            if not vp.sources():
                return False
        except Exception:
            return False
        setattr(self, "_video_skip_preview_flag_reset_in_ensure", True)
        if not self._ensure_video_preview_backend(from_start=True):
            return False
        try:
            self._connect_video_pipeline_frame_slots(vp)
        except Exception:
            pass
        try:
            self._video_preview_enabled = True
            self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(true);")
            self._run_js("if (window.setNativeHudMode) setNativeHudMode(true);")
            if not self._plan_flight_layer_obscures_native_camera_ui():
                if not bool(getattr(self, "_video_swap_user_map_main", False)):
                    self._video_swapped = False
                self._native_video_preview.show()
                self._layout_native_video_preview()
                self._stack_native_overlays_above_tile_map()
        except Exception:
            pass
        try:
            self._sync_native_thermal_feed_button()
        except Exception:
            pass
        return True

    def _companion_start_decode_if_needed(self, *, reason: str = "") -> None:
        """Start RTSP decode once; never tear down a session that is already connecting."""
        if not self._video_preview_should_run():
            return
        if self._should_defer_companion_rtsp_decode():
            return
        if not self._companion_wire_preview_ui():
            return
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            return
        try:
            self._stop_idle_video_decode_sources(vp)
        except Exception:
            pass
        if self._companion_decode_running(vp):
            if bool(getattr(self, "_video_preview_enabled", False)):
                try:
                    print(
                        f"[VGCS:video] decode already active ({reason}), "
                        "skipping restart (companion allows one RTSP client)"
                    )
                except Exception:
                    pass
                return
        now = time.monotonic()
        last = float(getattr(self, "_companion_video_restart_mono", 0.0) or 0.0)
        if now - last < 8.0:
            return
        self._companion_video_restart_mono = now
        try:
            print(f"[VGCS:video] companion decode start ({reason})")
        except Exception:
            pass
        self._restart_video_preview_after_settings(force_decode=False)

    def _request_companion_video_restart(self, *, reason: str = "") -> None:
        """Alias: start decode if needed (no forced restart_decode)."""
        self._companion_start_decode_if_needed(reason=reason)

    def _reapply_preview_zoom_now(self) -> None:
        """Re-crop the cached raw frame when zoom +/- changes (thermal software zoom only)."""
        try:
            sid = self._operator_preview_source_id()
        except Exception:
            sid = ""
        if not camera_preview_applies_digital_zoom(
            getattr(self, "_camera_control", None),
            sid,
        ):
            return
        raw = None
        cache = getattr(self, "_split_last_raw_images", None) or {}
        if sid and sid in cache:
            raw = cache.get(sid)
        if raw is None or not isinstance(raw, QImage) or raw.isNull():
            raw = getattr(self, "_native_pip_last_raw_frame", None)
        if raw is None or not isinstance(raw, QImage) or raw.isNull():
            return
        try:
            zimg = self._apply_digital_zoom(raw.copy(), self._effective_preview_digital_zoom(sid))
        except Exception:
            return
        try:
            self._native_pip_last_source_frame = zimg
            if sid:
                if not isinstance(getattr(self, "_split_last_images", None), dict):
                    self._split_last_images = {}
                self._split_last_images[sid] = zimg
            self._video_cache_mono = 0.0
            self._render_native_video_preview(zimg)
        except Exception:
            pass
