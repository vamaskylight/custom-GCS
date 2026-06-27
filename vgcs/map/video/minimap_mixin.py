"""MapWidget video mixin — see vgcs.map.video package."""

from __future__ import annotations

import time
from urllib.request import Request

from PySide6.QtCore import QPoint, QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QApplication

from vgcs.map.native_tile_map import fetch_tile_http_bytes


class NativeMinimapMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _schedule_minimap_grab_refresh(self) -> None:
        try:
            t = getattr(self, "_minimap_grab_refresh_timer", None)
            if t is None:
                return
            t.stop()
            t.start()
        except Exception:
            pass

    def _on_native_minimap_image_press(self, event) -> None:
        try:
            if event.button() != Qt.MouseButton.LeftButton:
                return
        except Exception:
            return
        if not bool(getattr(self, "_video_swapped", False)):
            return
        self._minimap_img_dragging = False
        self._minimap_img_press = QPointF(event.position())
        self._minimap_img_drag_last = QPointF(event.position())
        try:
            self._native_minimap.setCursor(Qt.CursorShape.ClosedHandCursor)
        except Exception:
            pass
        try:
            event.accept()
        except Exception:
            pass

    def _on_native_minimap_image_wheel(self, event) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            delta = float(event.angleDelta().y())
        except Exception:
            delta = 0.0
        if delta == 0.0:
            return
        try:
            cur_z = float(getattr(nm, "_zoom", 16.0))
        except Exception:
            cur_z = 16.0
        step = (delta / 120.0) * 1.0
        try:
            zmax = float(getattr(nm, "_max_zoom", 19))
        except Exception:
            zmax = 19.0
        new_z = max(3.0, min(zmax, cur_z + step))
        try:
            nm.set_zoom(new_z)
        except Exception:
            pass
        self._schedule_minimap_grab_refresh()
        try:
            event.accept()
        except Exception:
            pass

    def _on_native_minimap_image_move(self, event) -> None:
        try:
            held = bool(event.buttons() & Qt.MouseButton.LeftButton)
        except Exception:
            held = False
        if not held:
            return
        if not bool(getattr(self, "_video_swapped", False)):
            return
        last = getattr(self, "_minimap_img_drag_last", None)
        press = getattr(self, "_minimap_img_press", None)
        if last is None or press is None:
            return
        cur = QPointF(event.position())
        if (cur - press).manhattanLength() > 5.0:
            self._minimap_img_dragging = True
        dx = float(cur.x() - last.x())
        dy = float(cur.y() - last.y())
        self._minimap_img_drag_last = cur
        if dx == 0.0 and dy == 0.0:
            return
        nm = getattr(self, "_native_map", None)
        # Drag in the card pans the underlying map. Scale pixel deltas because the card image is
        # a scaled-down view of the real map; otherwise a small drag in the card causes a tiny pan.
        if nm is not None and hasattr(nm, "nudge_center_by_pixels"):
            try:
                lbl_w = max(1.0, float(self._native_minimap.width()))
                lbl_h = max(1.0, float(self._native_minimap.height()))
                nm_w = max(1.0, float(nm.width()))
                nm_h = max(1.0, float(nm.height()))
                sx = nm_w / lbl_w
                sy = nm_h / lbl_h
                nm.nudge_center_by_pixels(dx * sx, dy * sy)
            except Exception:
                pass
        self._schedule_minimap_grab_refresh()
        try:
            event.accept()
        except Exception:
            pass

    def _on_native_minimap_image_release(self, event) -> None:
        try:
            if event.button() != Qt.MouseButton.LeftButton:
                return
        except Exception:
            return
        try:
            self._native_minimap.setCursor(Qt.CursorShape.OpenHandCursor)
        except Exception:
            pass
        was_drag = bool(getattr(self, "_minimap_img_dragging", False))
        press = getattr(self, "_minimap_img_press", None)
        self._minimap_img_drag_last = None
        self._minimap_img_press = None
        self._minimap_img_dragging = False
        try:
            self._minimap_grab_refresh_timer.stop()
        except Exception:
            pass
        try:
            event.accept()
        except Exception:
            pass
        if was_drag:
            self._update_native_minimap()
            return
        # Short click (no meaningful drag).
        if press is not None:
            try:
                cur = QPointF(event.position())
                if (cur - press).manhattanLength() > 8.0:
                    self._update_native_minimap()
                    return
            except Exception:
                pass
        if not bool(getattr(self, "_video_swapped", False)):
            return
        # Target mode: click minimap to mark on map (do not swap layout).
        if self._observation_mark_active():
            try:
                latlon = self._minimap_click_to_lat_lon(QPointF(event.position()))
                if latlon is not None:
                    self._log_observation("map_mark", map_lat=float(latlon[0]), map_lon=float(latlon[1]))
                    return
            except Exception:
                pass
        self._video_swap_user_map_main = True
        self._video_swapped = False
        self._split_fullscreen_source_id = None
        self._layout_native_video_preview()
        self._show_map_main_surface()
        try:
            self._run_js("setVideoSwapMode(false);")
        except Exception:
            pass

    def _on_native_minimap_plus_clicked(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        cur = int(round(float(getattr(nm, "_zoom", 16.0))))
        zmax = int(getattr(nm, "_max_zoom", 19))
        self._native_minimap_set_zoom(min(zmax, cur + 1))

    def _on_native_minimap_minus_clicked(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        cur = int(round(float(getattr(nm, "_zoom", 16.0))))
        self._native_minimap_set_zoom(max(3, cur - 1))

    def _native_minimap_set_zoom(self, z: int) -> None:
        nm = getattr(self, "_native_map", None)
        zmax = int(getattr(nm, "_max_zoom", 19)) if nm is not None else 19
        self._native_minimap_zoom = max(3, min(zmax, int(z)))
        self._native_minimap_tile_key = None
        if nm is not None:
            try:
                nm.set_zoom(float(self._native_minimap_zoom))
            except Exception:
                pass
        self._update_native_minimap()
        try:
            QTimer.singleShot(220, self._update_native_minimap)
        except Exception:
            pass

    def _raise_native_minimap_zoom_buttons(self) -> None:
        """Pixmap refresh can restack the image label above the buttons — keep controls on top."""
        try:
            if not bool(getattr(self, "_video_swapped", False)):
                return
            if not self._native_minimap_wrap.isVisible():
                return
            self._btn_native_minimap_plus.raise_()
            self._btn_native_minimap_minus.raise_()
        except Exception:
            pass

    def _update_native_minimap_from_web_grab(self) -> bool:
        """
        Render the **full** map view into the swap mini-card so it doubles as a pan thumbnail.

        Earlier this only mirrored the lower-left 24%×20% of the map (legacy Web overlay shape),
        which made the card look like a blank tile if the user had panned away from that corner.
        """
        try:
            nm = getattr(self, "_native_map", None)
            if nm is None:
                return False
            tw = max(1, int(self._native_minimap.width()))
            th = max(1, int(self._native_minimap.height()))
            if tw <= 4 or th <= 4:
                return False
            try:
                nm.update()
                nm.repaint()
            except Exception:
                pass
            try:
                QApplication.processEvents()
            except Exception:
                pass
            shot = nm.grab()
            if shot.isNull():
                return False
            sw = shot.width()
            sh = shot.height()
            if sw <= 8 or sh <= 8:
                return False
            # HiDPI: QLabel is sized in logical px; grab pixmap may be device pixels — match DPR so the
            # preview fills the card instead of a small corner slice.
            dpr = max(1.0, float(self._native_minimap.devicePixelRatioF()))
            tw_px = max(1, int(round(float(tw) * dpr)))
            th_px = max(1, int(round(float(th) * dpr)))
            scaled = shot.scaled(
                tw_px,
                th_px,
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            if scaled.isNull():
                return False
            try:
                scaled.setDevicePixelRatio(dpr)
            except Exception:
                pass
            self._native_minimap.setPixmap(scaled)
            return True
        except Exception:
            return False

    def _render_native_minimap_fallback(self) -> None:
        """Render a clean neutral minimap card without external tile artifacts."""
        try:
            w = max(8, int(self._native_minimap.width()))
            h = max(8, int(self._native_minimap.height()))
            dpr = max(1.0, float(self._native_minimap.devicePixelRatioF()))
            wi = max(8, int(round(float(w) * dpr)))
            hi = max(8, int(round(float(h) * dpr)))
            img = QImage(wi, hi, QImage.Format.Format_RGB32)
            img.fill(QColor(24, 34, 54))
            p = QPainter(img)
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            # Simple grid look
            p.setPen(QPen(QColor(52, 72, 104), 1))
            step = int(round(18 * dpr))
            step = max(8, step)
            for x in range(0, wi, step):
                p.drawLine(x, 0, x, hi)
            for y in range(0, hi, step):
                p.drawLine(0, y, wi, y)
            # Center marker
            cx = wi // 2
            cy = hi // 2
            pr = max(4, int(round(5 * dpr)))
            p.setPen(QPen(QColor(35, 0, 0), 2))
            p.setBrush(QColor(240, 40, 44))
            p.drawEllipse(QPoint(cx, cy), pr, pr)
            p.end()
            pm = QPixmap.fromImage(img)
            try:
                pm.setDevicePixelRatio(dpr)
            except Exception:
                pass
            self._native_minimap.setPixmap(pm)
        except Exception:
            return

    def _native_minimap_tile_bad(self, img: QImage) -> bool:
        try:
            if img.isNull() or img.width() <= 0 or img.height() <= 0:
                return True
            step_x = max(1, img.width() // 24)
            step_y = max(1, img.height() // 24)
            colors: set[tuple[int, int, int]] = set()
            for y in range(0, img.height(), step_y):
                for x in range(0, img.width(), step_x):
                    c = img.pixelColor(x, y)
                    colors.add((c.red() // 16, c.green() // 16, c.blue() // 16))
                    if len(colors) > 64:
                        return False
            # Color bars/placeholders usually have very low color diversity.
            return len(colors) <= 24
        except Exception:
            return False

    def _fetch_tile_image(self, url: str) -> QImage:
        try:
            req = Request(
                str(url),
                headers={
                    "User-Agent": (
                        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                        "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"
                    ),
                    "Referer": "https://www.arcgis.com/",
                },
                method="GET",
            )
            with urlopen(req, timeout=2.2) as resp:
                raw = resp.read()
            return QImage.fromData(raw)
        except Exception:
            return QImage()

    def _schedule_native_minimap_refresh(self, *, force: bool = False) -> None:
        """Throttle swap minimap grabs — full map `grab()` on every telemetry tick freezes video."""
        try:
            if not self._native_minimap_wrap.isVisible():
                return
        except Exception:
            return
        now = time.monotonic()
        min_gap = 0.35
        last = float(getattr(self, "_minimap_refresh_mono", 0.0) or 0.0)
        if not force and now - last < min_gap:
            try:
                t = getattr(self, "_minimap_grab_refresh_timer", None)
                if t is not None and not t.isActive():
                    t.start(max(40, int((min_gap - (now - last)) * 1000)))
            except Exception:
                pass
            return
        self._minimap_refresh_mono = now
        self._update_native_minimap()

    def _update_native_minimap(self) -> None:
        try:
            if not self._native_minimap_wrap.isVisible():
                return
        except Exception:
            return
        # Best path: mirror current in-app map rendering.
        if not self._update_native_minimap_from_web_grab():
            # Never display external placeholder/test-pattern tiles again.
            self._render_native_minimap_fallback()
        self._raise_native_minimap_zoom_buttons()

    def _minimap_click_to_lat_lon(self, pos: QPointF) -> tuple[float, float] | None:
        """Map a click on the swap minimap thumbnail to lat/lon on the native map."""
        nm = getattr(self, "_native_map", None)
        if nm is None or not hasattr(nm, "_screen_to_lat_lon"):
            return None
        try:
            lbl = self._native_minimap
            pm = lbl.pixmap()
            if pm is None or pm.isNull():
                return None
            cr = lbl.contentsRect()
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
            ox = (Wc - spw) / 2.0
            oy = (Hc - sph) / 2.0
            lx = float(pos.x()) - float(cr.left())
            ly = float(pos.y()) - float(cr.top())
            if lx < ox or ly < oy or lx > ox + spw or ly > oy + sph:
                return None
            u = (lx - ox) / spw
            v = (ly - oy) / sph
            nm_w = float(max(1, nm.width()))
            nm_h = float(max(1, nm.height()))
            return nm._screen_to_lat_lon(QPointF(u * nm_w, v * nm_h))
        except Exception:
            return None
