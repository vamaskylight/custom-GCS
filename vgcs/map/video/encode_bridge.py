"""Background JPEG/PNG encode for video preview data URLs."""

from __future__ import annotations

import base64

from PySide6.QtCore import QByteArray, QBuffer, QObject, QRunnable, Qt, Signal


class VideoEncodeBridge(QObject):
    encoded = Signal(str)


class VideoEncodeTask(QRunnable):
    def __init__(
        self,
        img,
        bridge: VideoEncodeBridge,
        *,
        max_w: int = 1280,
        max_h: int = 720,
        encode_format: str = "PNG",
        encode_quality: int = 3,
    ) -> None:
        super().__init__()
        self._img = img
        self._bridge = bridge
        self._max_w = max(160, int(max_w))
        self._max_h = max(90, int(max_h))
        fmt = str(encode_format or "PNG").strip().upper()
        self._encode_format = (
            "PNG" if fmt not in ("PNG", "JPG", "JPEG") else ("JPG" if fmt == "JPEG" else fmt)
        )
        if self._encode_format == "PNG":
            self._encode_quality = max(0, min(9, int(encode_quality)))
        else:
            self._encode_quality = max(1, min(100, int(encode_quality)))

    def run(self) -> None:
        try:
            img = self._img
            if img is None or img.isNull():
                return
            try:
                iw = int(img.width())
                ih = int(img.height())
                if iw > 0 and ih > 0 and (iw > self._max_w or ih > self._max_h):
                    img = img.scaled(
                        self._max_w,
                        self._max_h,
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.FastTransformation,
                    )
            except Exception:
                pass
            ba = QByteArray()
            buf = QBuffer(ba)
            if not buf.open(QBuffer.OpenModeFlag.WriteOnly):
                return
            try:
                img.save(buf, self._encode_format, self._encode_quality)
            finally:
                buf.close()
            raw = bytes(ba)
            if not raw:
                return
            mime = "image/png" if self._encode_format == "PNG" else "image/jpeg"
            data_url = f"data:{mime};base64," + base64.b64encode(raw).decode("ascii")
            self._bridge.encoded.emit(data_url)
        except Exception:
            return

# Legacy names used by pipeline_mixin / map_widget extraction.
_VideoEncodeBridge = VideoEncodeBridge
_VideoEncodeTask = VideoEncodeTask
