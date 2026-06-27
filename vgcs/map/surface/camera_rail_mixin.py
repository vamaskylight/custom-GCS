"""MapWidget surface mixin — see vgcs.map.surface package."""

from __future__ import annotations

import math
import time

from PySide6.QtCore import QSettings, QTimer, Qt
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QLabel, QMessageBox, QPushButton

from vgcs.video.camera_control import NoopCameraControl, camera_zoom_limits


class CameraRailMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    # Skydroid C13: hold arrows use PTZ (PT_UP/…) — GSY/GSP speed hold is unreliable on field firmware.
    _GIMBAL_HOLD_SPEED_YAW_DPS = 5.0
    _GIMBAL_HOLD_SPEED_PITCH_DPS = 5.0

    def set_camera_control(self, control) -> None:
        """Inject a camera control backend (MAVLink/SDK)."""
        try:
            self._camera_control = control
        except Exception:
            pass
        try:
            zmin, zmax, _ = camera_zoom_limits(control)
            cur = float(getattr(self, "_video_zoom", 1.0))
            if cur < zmin or cur > zmax:
                self._video_zoom = max(zmin, min(zmax, cur))
                self._sync_native_video_zoom_label()
        except Exception:
            pass
        self._payload_hardware_recording = False
        self._video_inited = False
        self._shared_vp_hooks_connected = False

        # Do not reset `_video_split_enabled` here: this runs on every connect/disconnect and on
        # camera-backend hot-swap; the operator's SPLIT choice must persist (apply_video_settings
        # still seeds from Application Settings → Video when the user saves there).
        try:
            if getattr(self, "_web_ready", False):
                if bool(getattr(self, "_video_split_enabled", False)):
                    self._run_js("setVideoPreviewMode('grid');")
                else:
                    self._run_js("setVideoPreviewMode('single');")
        except Exception:
            pass
        self._sync_native_camera_rail_toggles()
        try:
            if bool(getattr(self, "_web_ready", False)) and self._video_preview_should_run():
                QTimer.singleShot(
                    400,
                    lambda: self._companion_start_decode_if_needed(reason="camera_control"),
                )
        except Exception:
            pass

    def _apply_digital_zoom(self, img: QImage, zoom: float) -> QImage:
        try:
            z = float(zoom)
        except Exception:
            z = 1.0
        if z <= 1.001:
            return img
        w = img.width()
        h = img.height()
        if w <= 0 or h <= 0:
            return img
        cw = max(1, int(w / z))
        ch = max(1, int(h / z))
        x = max(0, (w - cw) // 2)
        y = max(0, (h - ch) // 2)
        try:
            cropped = img.copy(x, y, cw, ch)
            return cropped.scaled(w, h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation)
        except Exception:
            return img

    def _ensure_obs_clip_banner(self) -> QLabel:
        lbl = getattr(self, "_obs_clip_banner", None)
        if lbl is not None:
            return lbl
        parent = getattr(self, "_native_video_preview", None)
        lbl = QLabel("", parent)
        lbl.setObjectName("obsClipBanner")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(
            "QLabel#obsClipBanner {"
            "  background: rgba(185, 28, 28, 230);"
            "  color: #ffffff;"
            "  padding: 10px 18px;"
            "  border-radius: 8px;"
            "  font-size: 15px;"
            "  font-weight: 700;"
            "}"
        )
        lbl.hide()
        self._obs_clip_banner = lbl
        return lbl

    def _position_obs_clip_banner(self) -> None:
        lbl = getattr(self, "_obs_clip_banner", None)
        parent = getattr(self, "_native_video_preview", None)
        if lbl is None or parent is None:
            return
        try:
            lbl.adjustSize()
            pw = max(1, int(parent.width()))
            lw = max(80, int(lbl.width()))
            lbl.move(max(8, (pw - lw) // 2), 14)
            lbl.raise_()
        except Exception:
            pass

    def _obs_clip_ui_preparing(self) -> None:
        """Immediate feedback when Clip is pressed (before RTSP/ffmpeg work)."""
        self._set_status("Observation clip: starting…")
        try:
            self._ensure_obs_clip_banner()
            self._show_obs_clip_banner("Observation clip — starting…")
        except Exception:
            pass
        try:
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is not None:
                lbl.show()
                lbl.setText("CLIP…")
                lbl.setStyleSheet("color: #fca5a5; font-weight: 700;")
        except Exception:
            pass

    def _obs_clip_update_countdown_labels(self) -> None:
        left = max(0, int(getattr(self, "_obs_clip_secs_left", 0) or 0))
        text = f"● REC {left}s" if left > 0 else "● REC"
        try:
            self._show_obs_clip_banner(f"Clip recording… {left}s remaining")
        except Exception:
            pass
        try:
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is not None:
                lbl.show()
                lbl.setText(text)
                lbl.setStyleSheet("color: #f87171; font-weight: 700;")
        except Exception:
            pass

    def _show_obs_clip_banner(self, text: str) -> None:
        lbl = self._ensure_obs_clip_banner()
        lbl.setText(str(text or "").strip())
        self._position_obs_clip_banner()
        lbl.show()
        lbl.raise_()

    def _hide_obs_clip_banner(self) -> None:
        lbl = getattr(self, "_obs_clip_banner", None)
        if lbl is not None:
            try:
                lbl.hide()
            except Exception:
                pass

    def _obs_clip_countdown_tick(self) -> None:
        if not bool(getattr(self, "_obs_clip_active", False)):
            return
        self._obs_clip_secs_left = max(0, int(self._obs_clip_secs_left or 0) - 1)
        if self._obs_clip_secs_left > 0:
            self._obs_clip_update_countdown_labels()
            return
        t = getattr(self, "_obs_clip_countdown_timer", None)
        if t is not None:
            try:
                t.stop()
            except Exception:
                pass

    def _obs_clip_ui_finished(self, *, ok: bool, detail: str = "") -> None:
        self._obs_clip_active = False
        t = getattr(self, "_obs_clip_countdown_timer", None)
        if t is not None:
            try:
                t.stop()
            except Exception:
                pass
        self._hide_obs_clip_banner()
        try:
            btn = getattr(self, "_btn_native_clip", None)
            if btn is not None:
                btn.setText("Clip")
                btn.setProperty("recording", False)
                btn.style().unpolish(btn)
                btn.style().polish(btn)
                btn.setEnabled(True)
        except Exception:
            pass
        try:
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is not None:
                if ok:
                    short = str(detail or "Clip saved")[:22]
                    lbl.setText(f"✓ {short}")
                    lbl.setStyleSheet("color: #86efac; font-weight: 700;")
                else:
                    lbl.setText("Clip failed")
                    lbl.setStyleSheet("color: #fca5a5; font-weight: 700;")
                if not hasattr(self, "_photo_flash_timer"):
                    self._photo_flash_timer = QTimer(self)
                    self._photo_flash_timer.setSingleShot(True)
                    self._photo_flash_timer.timeout.connect(self._clear_photo_flash)
                self._photo_flash_prev_text = "00:00:00"
                self._photo_flash_timer.start(2200)
        except Exception:
            pass
        self._sync_native_cam_timer_visibility()

    def _obs_clip_ui_failed(self, message: str, *, popup: bool = True) -> None:
        msg = str(message or "Observation clip failed").strip()
        self._obs_clip_ui_finished(ok=False, detail="")
        self._set_status(msg)
        print(f"[VGCS:observe] clip failed: {msg}")
        if popup:
            try:
                QMessageBox.warning(self, "Observation Clip", msg)
            except Exception:
                pass

    def _obs_cell(self, val: object) -> str:
        if val is None:
            return "N/A"
        if isinstance(val, float) and (math.isnan(val) or math.isinf(val)):
            return "N/A"
        s = str(val).strip()
        return s if s else "N/A"

    def _on_native_split_rail_toggled(self, _checked: bool) -> None:
        try:
            self._split_rail_debounce.start()
        except Exception:
            pass

    def _commit_native_split_rail_toggle(self) -> None:
        try:
            on = bool(self._btn_native_split.isChecked())
        except Exception:
            on = False
        print(f"[VGCS:cam_rail] SPLIT commit checked={on}")
        self._on_web_title_changed(f"VGCS_CAM_SPLIT_TOGGLE:{1 if on else 0}:0")
        self._sync_native_camera_rail_toggles()

    def _on_camera_rail_mode_id_clicked(self, bid: int) -> None:
        """Exclusive Video vs Photo row — photo is mode only; shutter is the center record button."""
        self._camera_rail_ui_mode = "photo" if int(bid) == 1 else "video"
        try:
            print(f"[VGCS:cam_rail] rail UI mode -> {self._camera_rail_ui_mode}")
        except Exception:
            pass
        self._sync_native_record_button_for_rail_mode()
        if self._camera_rail_ui_mode == "video":
            self._on_web_title_changed("VGCS_CAM_VIDEO_MODE_REQUEST:0")

    def _sync_native_cam_timer_visibility(self) -> None:
        """Recording timer is video-only; hide in photo mode (shutter feedback briefly shows the label)."""
        lbl = getattr(self, "_lbl_native_cam_timer", None)
        if lbl is None:
            return
        mode = getattr(self, "_camera_rail_ui_mode", "video")
        if mode == "video":
            if bool(getattr(self, "_obs_clip_active", False)):
                try:
                    lbl.show()
                except Exception:
                    pass
                return
            t = getattr(self, "_photo_flash_timer", None)
            if t is not None and t.isActive():
                try:
                    t.stop()
                except Exception:
                    pass
                try:
                    self._clear_photo_flash()
                except Exception:
                    pass
            try:
                lbl.show()
            except Exception:
                pass
            return
        flash_on = bool(
            getattr(self, "_photo_flash_timer", None) is not None
            and self._photo_flash_timer.isActive()
        )
        try:
            if flash_on:
                lbl.show()
            else:
                lbl.hide()
        except Exception:
            pass

    def _on_native_follow_rail_toggled(self, _checked: bool) -> None:
        try:
            self._follow_rail_debounce.start()
        except Exception:
            pass

    def _commit_native_follow_rail_toggle(self) -> None:
        try:
            on = bool(self._btn_native_follow.isChecked())
        except Exception:
            on = False
        print(f"[VGCS:cam_rail] FOLLOW commit checked={on}")
        self._on_web_title_changed(f"VGCS_CAM_FOLLOW_TOGGLE:{1 if on else 0}:0")
        self._sync_native_camera_rail_toggles()

    def _sync_native_camera_rail_toggles(self) -> None:
        """Keep Split / Follow aligned with pipeline flags; Split green when 4-up is meaningful (not single-channel fullscreen)."""
        try:
            if hasattr(self, "_btn_native_split"):
                en = bool(getattr(self, "_video_split_enabled", False))
                self._btn_native_split.blockSignals(True)
                self._btn_native_split.setChecked(en)
                # Green split highlight: on whenever split mode is on, except when the main canvas
                # is full-bleed video showing a single zoomed channel (PiP 4-up and full 2×2 composite
                # both keep the highlight; map-main + corner split does too).
                hide_split_chrome = en and bool(getattr(self, "_video_swapped", False)) and bool(
                    getattr(self, "_split_fullscreen_source_id", None)
                )
                try:
                    self._btn_native_split.setProperty("splitHidden", hide_split_chrome)
                except Exception:
                    pass
                self._btn_native_split.blockSignals(False)
                try:
                    st = self._btn_native_split.style()
                    if st is not None:
                        st.unpolish(self._btn_native_split)
                        st.polish(self._btn_native_split)
                except Exception:
                    pass
            if hasattr(self, "_btn_native_follow"):
                fen = bool(getattr(self, "_video_follow_enabled", False))
                self._btn_native_follow.blockSignals(True)
                self._btn_native_follow.setChecked(fen)
                self._btn_native_follow.blockSignals(False)
            self._sync_native_thermal_feed_button()
        except Exception:
            pass

    def _native_gimbal_uses_ptz_hold(self) -> bool:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return False
        try:
            from vgcs.video.camera_control import SkydroidCameraControl

            return isinstance(cc, SkydroidCameraControl)
        except Exception:
            return False

    @staticmethod
    @staticmethod
    def _native_gimbal_ptz_action(dx: int, dy: int) -> str | None:
        if int(dy) > 0:
            return "up"
        if int(dy) < 0:
            return "down"
        if int(dx) < 0:
            return "left"
        if int(dx) > 0:
            return "right"
        return None

    def _gimbal_hold_speeds(self, dx: int, dy: int) -> tuple[float, float]:
        s = QSettings("VGCS", "VGCS")
        try:
            sy = float(s.value("camera/skydroid_gimbal_speed_yaw", self._GIMBAL_HOLD_SPEED_YAW_DPS) or self._GIMBAL_HOLD_SPEED_YAW_DPS)
        except Exception:
            sy = float(self._GIMBAL_HOLD_SPEED_YAW_DPS)
        try:
            sp = float(
                s.value("camera/skydroid_gimbal_speed_pitch", self._GIMBAL_HOLD_SPEED_PITCH_DPS)
                or self._GIMBAL_HOLD_SPEED_PITCH_DPS
            )
        except Exception:
            sp = float(self._GIMBAL_HOLD_SPEED_PITCH_DPS)
        # Older builds defaulted to 1.8 deg/s — too slow on C13; treat as outdated setting.
        if sy < 2.5:
            sy = float(self._GIMBAL_HOLD_SPEED_YAW_DPS)
        if sp < 2.5:
            sp = float(self._GIMBAL_HOLD_SPEED_PITCH_DPS)
        return (float(dx) * sy, float(dy) * sp)

    def _wire_native_gimbal_hold_button(self, btn: QPushButton, dx: int, dy: int) -> None:
        """Press/hold = PTZ once on C13 (PT_RIGHT…); GSY/GSP refresh on other backends."""

        def _start() -> None:
            self._native_gimbal_speed_stop()
            self._gimbal_hold_axis = (int(dx), int(dy))
            self._native_gimbal_speed_start(dx, dy)
            if self._native_gimbal_uses_ptz_hold():
                return
            if not self._gimbal_hold_timer.isActive():
                self._gimbal_hold_timer.start()

        def _stop() -> None:
            self._gimbal_hold_axis = None
            self._gimbal_hold_timer.stop()
            self._native_gimbal_speed_stop()

        btn.pressed.connect(_start)
        btn.released.connect(_stop)

    def _on_gimbal_hold_tick(self) -> None:
        axis = self._gimbal_hold_axis
        if axis is None:
            return
        if self._native_gimbal_uses_ptz_hold():
            return
        self._notify_companion_gimbal_motion(duration_s=1.2)
        self._native_gimbal_speed_start(axis[0], axis[1])

    def _native_gimbal_speed_start(self, dx: int, dy: int) -> None:
        self._notify_companion_gimbal_motion(duration_s=2.5)
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return
        if self._native_gimbal_uses_ptz_hold():
            act = self._native_gimbal_ptz_action(dx, dy)
            if act:
                try:
                    cc.ptz(act)
                except Exception:
                    pass
            return
        yaw_s, pitch_s = self._gimbal_hold_speeds(dx, dy)
        try:
            cc.set_gimbal_speed(yaw_s, pitch_s)
        except Exception:
            pass

    def _native_gimbal_speed_stop(self) -> None:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
            return
        if self._native_gimbal_uses_ptz_hold():
            try:
                cc.ptz("stop")
            except Exception:
                pass
        else:
            try:
                cc.set_gimbal_speed(0.0, 0.0)
            except Exception:
                pass
        self._notify_companion_gimbal_motion(duration_s=1.5)
        # Trigger auto-focus shortly after gimbal stops so the image sharpens immediately
        # instead of waiting for the camera's internal AF timer (can take 3-5s on ZR10).
        QTimer.singleShot(400, self._trigger_gimbal_stop_autofocus)

    def _native_gimbal_center(self) -> None:
        self._notify_companion_gimbal_motion(duration_s=4.0)
        self._gimbal_hold_axis = None
        if self._gimbal_hold_timer.isActive():
            self._gimbal_hold_timer.stop()
        self._native_gimbal_speed_stop()
        cc = getattr(self, "_camera_control", None)
        if cc is None or isinstance(cc, NoopCameraControl):
            self._set_status("Gimbal center — connect camera control first")
            return
        try:
            cc.gimbal_center()
            self._set_status("Gimbal recentered")
            QTimer.singleShot(400, self._trigger_gimbal_stop_autofocus)
        except Exception:
            self._set_status("Gimbal center failed")

    def _native_gimbal_point_down(self) -> None:
        self._notify_companion_gimbal_motion(duration_s=6.0)
        self._gimbal_hold_axis = None
        if self._gimbal_hold_timer.isActive():
            self._gimbal_hold_timer.stop()
        self._native_gimbal_speed_stop()
        cc = getattr(self, "_camera_control", None)
        if cc is None or isinstance(cc, NoopCameraControl):
            self._set_status("Gimbal 90° — connect camera control first")
            return
        try:
            from vgcs.video.camera_control import gimbal_nadir_pitch_deg  # noqa: PLC0415

            pitch = gimbal_nadir_pitch_deg()
            cc.gimbal_point_down()
            self._set_status(f"Gimbal pitch → {pitch:.0f}°")
            QTimer.singleShot(400, self._trigger_gimbal_stop_autofocus)
        except Exception:
            self._set_status("Gimbal pitch-down failed")

    def _siyi_autofocus_adapter(self) -> object | None:
        cc = getattr(self, "_camera_control", None)
        if cc is None or isinstance(cc, NoopCameraControl):
            return None
        adapter = getattr(cc, "_adapter", None)
        if adapter is None:
            primary = getattr(cc, "_primary", None)
            adapter = getattr(primary, "_adapter", None) if primary is not None else None
        if adapter is not None and hasattr(adapter, "camera_auto_focus"):
            return adapter
        return None

    def _trigger_gimbal_stop_autofocus(self) -> None:
        adapter = self._siyi_autofocus_adapter()
        if adapter is None:
            return
        try:
            # First pulse after settle; second pulse catches slow ZR10 AF hunts.
            adapter.camera_auto_focus()
            QTimer.singleShot(550, lambda: adapter.camera_auto_focus())
        except Exception:
            pass
