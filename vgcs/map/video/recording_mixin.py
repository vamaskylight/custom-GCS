"""MapWidget video mixin — see vgcs.map.video package."""

from __future__ import annotations

import os
import shutil
import time

from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QFileDialog, QMessageBox

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

    # --- finishing a recording, whatever state the video backend is in -------
    #
    # Field report 2026-09-18 (C14 Pro session): "after stop the recording timer
    # disappears" and no save window. Two ways that happened:
    #  1. The stop press looked up the *current* preview source first; while the
    #     stream was being reopened there was none, so the press fell through to
    #     "nothing is recording" - no stop_recording(), no save dialog, temp
    #     file orphaned.
    #  2. Any video-backend restart (camera control re-set, settings applied,
    #     link reconnect) cleared the recording flags in _ensure_video_preview_backend
    #     without stopping the recorder, so the next stop press had nothing to
    #     finish and the file was never finalised.
    # Both now go through _finish_video_recording, which finds the recorder by
    # the source id captured at start, finalises the file, and always offers
    # the save dialog when there is a file worth saving.

    def _recording_source_for_stop(self):
        """The source that is recording: by the id captured at start, else the active one."""
        sid = str(getattr(self, "_video_recording_source_id", "") or "").strip()
        src = None
        if sid:
            try:
                src = self._video_source_by_id(sid)
            except Exception:
                src = None
        if src is None:
            try:
                src = self._operator_preview_video_source()
            except Exception:
                src = None
        if src is None:
            src = getattr(self, "_video_active_source", None)
        return src

    def _finish_video_recording(self, *, reason: str = "", prompt: bool = True) -> bool:
        """Stop a running recording and offer to save it. True if one was running.

        Safe to call when nothing is recording (returns False, touches nothing).
        ``reason`` is shown when the stop was not the operator's own press.
        """
        if not bool(getattr(self, "_video_recording", False)):
            return False
        src = self._recording_source_for_stop()
        try:
            self._sync_payload_hardware_recording(False)
        except Exception:
            pass
        stop_fn = getattr(src, "stop_recording", None)
        if callable(stop_fn):
            try:
                stop_fn()
            except Exception:
                pass
        else:
            # QMediaRecorder-backed source (non-RTSP): stop it the old way.
            rec = None
            try:
                rec = src.recorder() if src is not None and hasattr(src, "recorder") else None
            except Exception:
                rec = None
            if rec is not None:
                try:
                    rec.stop()
                except Exception:
                    pass
                try:
                    wait_qmedia_recorder_stopped(rec, timeout_s=25.0)
                except Exception:
                    pass
        self._video_recording = False
        self._video_recording_source_id = ""
        tmp_path = str(getattr(self, "_video_recording_tmp_path", "") or "")
        self._video_recording_tmp_path = ""
        try:
            from vgcs.video.pipeline import notify_companion_recording

            notify_companion_recording(active=False)
        except Exception:
            pass
        self._stop_native_cam_recording_tick_timer()
        if src is None:
            try:
                print("[VGCS:cam_rail] RECORD stop: video source already gone; keeping the file")
            except Exception:
                pass
        if reason:
            try:
                print(f"[VGCS:cam_rail] RECORD stopped — {reason}")
            except Exception:
                pass
        if prompt:
            self._prompt_save_recording(tmp_path, reason=reason)
        else:
            self._video_recording_unsaved_path = tmp_path
        return True

    def _offer_unsaved_recording(self) -> None:
        """Save dialog for a recording that a backend restart had to stop."""
        path = str(getattr(self, "_video_recording_unsaved_path", "") or "")
        self._video_recording_unsaved_path = ""
        if path:
            self._prompt_save_recording(path, reason="video restarted")

    def _prompt_save_recording(self, tmp_path: str, *, reason: str = "") -> None:
        """Save dialog for a finished recording; says why when there is nothing to save."""
        path = str(tmp_path or "").strip()
        size = 0
        try:
            size = os.path.getsize(path) if path and os.path.exists(path) else 0
        except Exception:
            size = 0
        if size <= 0:
            msg = "Recording stopped but the file is empty — no frames were captured"
            if reason:
                msg += f" ({reason})"
            try:
                print(f"[VGCS:cam_rail] RECORD save skipped — {msg}")
            except Exception:
                pass
            self._set_status(msg)
            return
        if reason:
            self._set_status(f"Recording stopped — {reason}. Choose where to save it.")
        save_to, _ = QFileDialog.getSaveFileName(
            self,
            "Save recording",
            suggested_recording_save_path(),
            "Video (*.mp4 *.mov *.mkv)",
        )
        if not save_to:
            self._set_status("Recording discarded (no file name chosen)")
            return
        try:
            shutil.move(path, str(save_to))
            self._set_status(f"Recording saved: {save_to}")
        except Exception as e:
            self._set_status(f"Could not save recording: {e}")

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
        # Always say the press arrived. Every branch below used to be able to
        # swallow it, so "recording button is not working" (field report
        # 2026-08-20) produced a log with no RECORD line of any kind — leaving
        # no way to tell a press that never reached Python from one that was
        # discarded here.
        mode = str(getattr(self, "_camera_rail_ui_mode", "video"))
        try:
            print(f"[VGCS:cam_rail] RECORD button toggled={bool(on)} rail_mode={mode!r}")
        except Exception:
            pass
        if mode != "video":
            # Photo mode repurposes this button as the shutter, so a record
            # press here is meaningless — but silently doing nothing looks
            # identical to a broken button.
            self._set_status(
                "Recording is video mode only — switch the camera rail from Photo to Video"
            )
            return
        self._on_web_title_changed(f"VGCS_CAM_RECORD_TOGGLE:{1 if on else 0}:0")
