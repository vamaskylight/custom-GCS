from __future__ import annotations

import shutil
import tempfile
import time
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QSettings, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
    QFileDialog,
)

from vgcs.video.pipeline import (
    HAS_MULTIMEDIA,
    VideoFrame,
    VideoPipeline,
    QS_KEY_LAST_PHOTO_SAVE_DIR,
    suggested_photo_save_path,
    suggested_recording_save_path,
    wait_qmedia_recorder_stopped,
)


def _apply_day_night(img: QImage, mode: str) -> QImage:
    """
    Simple, fast day/night preview transform for M3.
    - day: passthrough
    - night: grayscale + boost
    """
    if img is None or img.isNull():
        return img
    m = (mode or "day").strip().lower()
    if m == "day":
        return img
    # Night: convert to grayscale and brighten slightly.
    try:
        g = img.convertToFormat(QImage.Format.Format_Grayscale8)
        # Brightness boost by scaling to RGB32 and using a painter would be slower;
        # for now, keep grayscale (good enough for requirement).
        return g
    except Exception:
        return img


def _apply_digital_zoom(img: QImage, zoom: float) -> QImage:
    if img is None or img.isNull():
        return img
    z = float(zoom)
    if z <= 1.001:
        return img
    w = img.width()
    h = img.height()
    if w <= 0 or h <= 0:
        return img
    # Crop center region then scale back.
    cw = max(1, int(w / z))
    ch = max(1, int(h / z))
    x = max(0, (w - cw) // 2)
    y = max(0, (h - ch) // 2)
    try:
        cropped = img.copy(x, y, cw, ch)
        return cropped.scaled(w, h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation)
    except Exception:
        return img


class VideoPreviewLabel(QLabel):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(220, 140)
        self.setStyleSheet(
            "QLabel { background: rgba(10, 12, 16, 0.9); border: 1px solid rgba(140, 160, 190, 0.25); }"
        )
        self._last_img: Optional[QImage] = None

    def set_frame(self, img: QImage) -> None:
        self._last_img = img
        if img is None or img.isNull():
            self.setText("No video")
            self.setPixmap(QPixmap())
            return
        pm = QPixmap.fromImage(img)
        if not pm.isNull():
            self.setPixmap(pm.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation))

    def resizeEvent(self, e) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(e)
        if self._last_img is not None and not self._last_img.isNull():
            self.set_frame(self._last_img)


class CameraControlPanel(QGroupBox):
    follow_triggered = Signal(bool)

    def __init__(
        self,
        pipeline: VideoPipeline,
        parent: QWidget | None = None,
        camera_control=None,
    ) -> None:
        super().__init__("Camera Control", parent)
        self._pipeline = pipeline
        self._camera_control = camera_control
        self._zoom = 1.0
        self._mode = "day"
        self._record_tmp_path: Optional[str] = None

        root = QVBoxLayout()

        self._status = QLabel("Multimedia: OK" if HAS_MULTIMEDIA else "Multimedia: unavailable")
        self._status.setObjectName("telemetryValue")
        root.addWidget(self._status)

        row = QHBoxLayout()
        self._camera_combo = QComboBox()
        self._btn_refresh = QPushButton("Refresh")
        self._btn_start = QPushButton("Start")
        self._btn_stop = QPushButton("Stop")
        row.addWidget(self._camera_combo, 1)
        row.addWidget(self._btn_refresh)
        row.addWidget(self._btn_start)
        row.addWidget(self._btn_stop)
        root.addLayout(row)

        mode_row = QHBoxLayout()
        self._btn_day = QPushButton("Day")
        self._btn_night = QPushButton("Night")
        self._btn_day.setCheckable(True)
        self._btn_night.setCheckable(True)
        self._btn_day.setChecked(True)
        mode_row.addWidget(QLabel("Vision"))
        mode_row.addWidget(self._btn_day)
        mode_row.addWidget(self._btn_night)
        mode_row.addStretch(1)
        root.addLayout(mode_row)

        zoom_row = QHBoxLayout()
        self._zoom_slider = QSlider(Qt.Orientation.Horizontal)
        self._zoom_slider.setRange(10, 40)  # 1.0x .. 4.0x
        self._zoom_slider.setValue(10)
        self._zoom_lab = QLabel("1.0x")
        zoom_row.addWidget(QLabel("Zoom"))
        zoom_row.addWidget(self._zoom_slider, 1)
        zoom_row.addWidget(self._zoom_lab)
        root.addLayout(zoom_row)

        act_row = QHBoxLayout()
        self._btn_photo = QPushButton("Photo…")
        self._btn_record = QPushButton("Record…")
        self._btn_record.setCheckable(True)
        self._btn_follow = QPushButton("Follow map")
        self._btn_follow.setCheckable(True)
        self._btn_follow.setToolTip("Center the map on the vehicle while enabled (same as map camera-rail Follow).")
        act_row.addWidget(self._btn_photo)
        act_row.addWidget(self._btn_record)
        act_row.addWidget(self._btn_follow)
        root.addLayout(act_row)

        self.setLayout(root)
        self.setMinimumWidth(260)

        self._pipeline.sources_changed.connect(
            self._rebuild_sources, Qt.ConnectionType.QueuedConnection
        )
        self._pipeline.active_source_changed.connect(
            self._sync_active_source, Qt.ConnectionType.QueuedConnection
        )
        self._btn_refresh.clicked.connect(lambda: QTimer.singleShot(0, self._pipeline.refresh_sources))
        self._camera_combo.currentIndexChanged.connect(self._on_camera_selected)
        self._btn_start.clicked.connect(self._start_active)
        self._btn_stop.clicked.connect(self._stop_active)
        self._btn_day.clicked.connect(lambda: self._set_mode("day"))
        self._btn_night.clicked.connect(lambda: self._set_mode("night"))
        self._zoom_slider.valueChanged.connect(self._on_zoom_changed)
        self._btn_photo.clicked.connect(self._take_photo)
        self._btn_record.toggled.connect(self._toggle_recording)
        self._btn_follow.toggled.connect(self.follow_triggered.emit)

        self._rebuild_sources()

    def set_camera_control(self, control) -> None:
        self._camera_control = control

    def sync_video_follow_toggle(self, enabled: bool) -> None:
        """Keep the Follow control aligned when the map rail (or code) changes follow mode."""
        self._btn_follow.blockSignals(True)
        self._btn_follow.setChecked(bool(enabled))
        self._btn_follow.blockSignals(False)

    def vision_mode(self) -> str:
        return str(self._mode)

    def zoom(self) -> float:
        return float(self._zoom)

    def transform_frame(self, img: QImage) -> QImage:
        out = _apply_day_night(img, self._mode)
        out = _apply_digital_zoom(out, self._zoom)
        return out

    def _rebuild_sources(self) -> None:
        self._camera_combo.blockSignals(True)
        self._camera_combo.clear()
        sources = self._pipeline.sources()
        for sid, src in sources.items():
            self._camera_combo.addItem(src.device_name, sid)
        self._camera_combo.blockSignals(False)
        self._sync_active_source(self._pipeline.active_source_id())

    def _sync_active_source(self, source_id: str) -> None:
        for i in range(self._camera_combo.count()):
            if str(self._camera_combo.itemData(i)) == str(source_id):
                self._camera_combo.setCurrentIndex(i)
                return

    def _on_camera_selected(self) -> None:
        sid = str(self._camera_combo.currentData() or "")
        if sid:
            self._pipeline.set_active_source(sid)

    def _set_mode(self, mode: str) -> None:
        m = (mode or "day").strip().lower()
        self._mode = "night" if m == "night" else "day"
        self._btn_day.setChecked(self._mode == "day")
        self._btn_night.setChecked(self._mode == "night")

    def _on_zoom_changed(self, v: int) -> None:
        self._zoom = max(1.0, float(v) / 10.0)
        self._zoom_lab.setText(f"{self._zoom:.1f}x")

    def _start_active(self) -> None:
        src = self._pipeline.active_source()
        if src is None:
            return
        src.start()

    def _stop_active(self) -> None:
        src = self._pipeline.active_source()
        if src is None:
            return
        src.stop()

    def _take_photo(self) -> None:
        cc = getattr(self, "_camera_control", None)
        if cc is not None:
            try:
                cc.camera_trigger_photo()
            except Exception:
                pass
        src = self._pipeline.active_source()
        if src is None:
            return
        st = QSettings("VGCS", "VGCS")
        last = str(st.value(QS_KEY_LAST_PHOTO_SAVE_DIR, "") or "").strip()
        start_dir = Path(last) if last and Path(last).is_dir() else None
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save photo",
            suggested_photo_save_path(directory=start_dir),
            "Images (*.jpg *.png)",
        )
        if not filename:
            return
        ok = src.take_photo(filename)
        if ok:
            try:
                st.setValue(QS_KEY_LAST_PHOTO_SAVE_DIR, str(Path(filename).parent))
            except Exception:
                pass
        if not ok:
            self._status.setText("Photo capture unsupported on this backend")

    def _toggle_recording(self, enabled: bool) -> None:
        src = self._pipeline.active_source()
        if src is None:
            return
        rec = src.recorder()
        if rec is None:
            self._status.setText("Recording unsupported on this backend")
            self._btn_record.blockSignals(True)
            self._btn_record.setChecked(False)
            self._btn_record.blockSignals(False)
            return

        if enabled:
            try:
                # setOutputLocation expects QUrl; allow string fallback.
                try:
                    from PySide6.QtCore import QUrl

                    tmp = Path(tempfile.gettempdir()) / f"vgcs_recording_{int(time.time())}.mp4"
                    rec.setOutputLocation(QUrl.fromLocalFile(str(tmp)))
                    self._record_tmp_path = str(tmp)
                except Exception:
                    pass
                rec.record()
                self._status.setText("Recording…")
            except Exception:
                self._status.setText("Failed to start recording")
                self._btn_record.blockSignals(True)
                self._btn_record.setChecked(False)
                self._btn_record.blockSignals(False)
                self._record_tmp_path = None
        else:
            try:
                rec.stop()
                try:
                    wait_qmedia_recorder_stopped(rec, timeout_s=25.0)
                except Exception:
                    pass
                tmp_path = str(self._record_tmp_path or "")
                self._record_tmp_path = None
                if tmp_path:
                    filename, _ = QFileDialog.getSaveFileName(
                        self,
                        "Save recording",
                        suggested_recording_save_path(),
                        "Video (*.mp4 *.mov *.mkv)",
                    )
                    if filename:
                        try:
                            shutil.move(tmp_path, str(filename))
                        except Exception:
                            pass
                self._status.setText("Recording stopped")
            except Exception:
                self._status.setText("Failed to stop recording")
                self._record_tmp_path = None

