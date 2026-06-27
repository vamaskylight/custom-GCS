"""MapWidget video mixin — see vgcs.map.video package."""

from __future__ import annotations

import time

from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QMessageBox

from vgcs.map.image_io import save_qimage_to_path
from vgcs.video.camera_control import NoopCameraControl
from vgcs.video.pipeline import (
    suggested_photo_save_path,
    suggested_recording_save_path,
    wait_qmedia_recorder_stopped,
)


class VideoRecordingMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _trigger_hardware_photo(self) -> None:
        cc = getattr(self, "_camera_control", None)
        if cc is None or isinstance(cc, NoopCameraControl):
            return
        try:
            cc.camera_trigger_photo()
        except Exception:
            pass

    def _sync_payload_hardware_recording(self, want_on: bool) -> None:
        want = bool(want_on)
        if bool(getattr(self, "_payload_hardware_recording", False)) == want:
            return
        cc = getattr(self, "_camera_control", None)
        if cc is None or isinstance(cc, NoopCameraControl):
            return
        try:
            cc.camera_toggle_record()
            self._payload_hardware_recording = want
        except Exception:
            pass

    def _capture_photo_quick(self, output_path: str | None = None) -> str | None:
        """
        Save a still image from the best available live preview source.

        If ``output_path`` is set, the file is written there (after creating parent
        folders). If ``None``, writes ``captures/photo_YYYYMMDD_HHMMSS.*`` for
        silent snapshots (e.g. observation logging).
        """
        stamp = time.strftime("%Y%m%d_%H%M%S")
        explicit = str(output_path or "").strip()
        photos_dir: Path | None = None
        if explicit:
            dest = Path(explicit).expanduser()
            suf = dest.suffix.lower()
            if suf not in (".jpg", ".jpeg", ".png"):
                dest = dest.with_suffix(".jpg")
            try:
                dest.parent.mkdir(parents=True, exist_ok=True)
            except Exception:
                return None
        else:
            photos_dir = Path.cwd() / "captures"
            try:
                photos_dir.mkdir(parents=True, exist_ok=True)
            except Exception:
                return None
            dest = photos_dir / f"photo_{stamp}.jpg"

        img = self._preview_image_copy_for_snapshot()
        if img is not None and _save_qimage_to_path(img, dest):
            return str(dest)

        # Last resort: QtMultimedia capture (can block; avoid on observation hot path).
        try:
            self._ensure_video_preview_backend()
            src = self._operator_preview_video_source()
            if src is not None and hasattr(src, "take_photo"):
                if bool(src.take_photo(str(dest))):
                    return str(dest)
        except Exception:
            pass
        return None

    def _flash_photo_feedback(self, *, ok: bool, name: str = "") -> None:
        """Briefly replace the cam timer text with `Saved` / `No frame` so the operator sees feedback."""
        try:
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is None:
                return
            if not hasattr(self, "_photo_flash_timer"):
                self._photo_flash_timer = QTimer(self)
                self._photo_flash_timer.setSingleShot(True)
                self._photo_flash_timer.timeout.connect(self._clear_photo_flash)
            prev = getattr(self, "_photo_flash_prev_text", None)
            if prev is None:
                self._photo_flash_prev_text = str(lbl.text() or "")
            lbl.show()
            if ok:
                short = name[:14] if name else "Photo saved"
                lbl.setText(f"✓ {short}")
            else:
                lbl.setText("No frame")
            self._photo_flash_timer.start(1400)
        except Exception:
            pass

    def _clear_photo_flash(self) -> None:
        try:
            if bool(getattr(self, "_obs_clip_active", False)):
                return
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is None:
                return
            prev = getattr(self, "_photo_flash_prev_text", None)
            if prev is not None:
                lbl.setText(str(prev))
            self._photo_flash_prev_text = None
            self._sync_native_cam_timer_visibility()
        except Exception:
            pass

    def _obs_clip_ui_recording_started(self, *, seconds: int = 8) -> None:
        self._obs_clip_active = True
        self._obs_clip_secs_left = max(1, int(seconds))
        try:
            btn = getattr(self, "_btn_native_clip", None)
            if btn is not None:
                btn.setText("REC")
                btn.setProperty("recording", True)
                btn.style().unpolish(btn)
                btn.style().polish(btn)
                btn.setEnabled(False)
        except Exception:
            pass
        self._obs_clip_update_countdown_labels()
        t = getattr(self, "_obs_clip_countdown_timer", None)
        if t is None:
            t = QTimer(self)
            t.timeout.connect(self._obs_clip_countdown_tick)
            self._obs_clip_countdown_timer = t
        try:
            t.start(1000)
        except Exception:
            pass
        self._set_status(
            f"Observation clip recording — {self._obs_clip_secs_left}s (do not press Clip again)"
        )

    def _sync_native_record_button_for_rail_mode(self) -> None:
        """Photo mode: center button is a non-checkable shutter. Video mode: checkable record."""
        btn = getattr(self, "_btn_native_record", None)
        if btn is None:
            return
        btn.blockSignals(True)
        try:
            if getattr(self, "_camera_rail_ui_mode", "video") == "photo":
                btn.setCheckable(False)
                btn.setChecked(False)
                btn.setToolTip("Take photo (shutter)")
            else:
                btn.setCheckable(True)
                btn.setChecked(bool(getattr(self, "_video_recording", False)))
                btn.setToolTip("Record video")
        finally:
            btn.blockSignals(False)
        self._sync_native_cam_timer_visibility()

    @staticmethod
    def _format_native_cam_recording_duration(total_secs: int) -> str:
        total_secs = max(0, int(total_secs))
        h = total_secs // 3600
        m = (total_secs % 3600) // 60
        s = total_secs % 60
        return f"{h:02d}:{m:02d}:{s:02d}"

    def _ensure_native_cam_recording_tick_timer(self) -> QTimer:
        t = getattr(self, "_native_cam_recording_tick_timer", None)
        if t is None:
            t = QTimer(self)
            t.setInterval(250)
            t.timeout.connect(self._on_native_cam_recording_tick)
            self._native_cam_recording_tick_timer = t
        return t

    def _on_native_cam_recording_tick(self) -> None:
        if not bool(getattr(self, "_video_recording", False)):
            self._stop_native_cam_recording_tick_timer(reset_label=True)
            return
        t0 = float(getattr(self, "_native_cam_recording_started_mono", 0.0) or 0.0)
        elapsed = int(time.monotonic() - t0)
        lbl = getattr(self, "_lbl_native_cam_timer", None)
        if lbl is None:
            return
        try:
            lbl.setText(self._format_native_cam_recording_duration(elapsed))
        except Exception:
            pass

    def _start_native_cam_recording_tick_timer(self) -> None:
        self._native_cam_recording_started_mono = time.monotonic()
        lbl = getattr(self, "_lbl_native_cam_timer", None)
        if lbl is not None:
            try:
                lbl.setText("00:00:00")
            except Exception:
                pass
        try:
            self._ensure_native_cam_recording_tick_timer().start()
        except Exception:
            pass

    def _stop_native_cam_recording_tick_timer(self, *, reset_label: bool = True) -> None:
        t = getattr(self, "_native_cam_recording_tick_timer", None)
        if t is not None:
            try:
                t.stop()
            except Exception:
                pass
        if reset_label:
            lbl = getattr(self, "_lbl_native_cam_timer", None)
            if lbl is not None:
                try:
                    lbl.setText("00:00:00")
                except Exception:
                    pass
        self._native_cam_recording_started_mono = 0.0

    def _on_native_record_center_clicked(self) -> None:
        if getattr(self, "_camera_rail_ui_mode", "video") != "photo":
            return
        try:
            print("[VGCS:cam_rail] SHUTTER click (photo mode)")
        except Exception:
            pass
        self._on_web_title_changed("VGCS_CAM_PHOTO_REQUEST:0")

    def _on_native_record_toggled(self, on: bool) -> None:
        if getattr(self, "_camera_rail_ui_mode", "video") != "video":
            return
        self._on_web_title_changed(f"VGCS_CAM_RECORD_TOGGLE:{1 if on else 0}:0")
