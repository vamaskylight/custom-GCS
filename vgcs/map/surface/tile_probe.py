"""HTTP tile probe used by map startup health checks."""

from __future__ import annotations

from PySide6.QtCore import QObject, QRunnable, Signal
from PySide6.QtGui import QImage

from vgcs.map.native_tile_map import fetch_tile_http_bytes



class _TileProbeBridge(QObject):
    result = Signal(str, str, str)  # provider_label, outcome, detail


class _TileProbeTask(QRunnable):
    def __init__(self, *, url: str, provider_label: str, bridge: _TileProbeBridge) -> None:
        super().__init__()
        self._url = url
        self._provider_label = provider_label
        self._bridge = bridge

    @staticmethod
    def _classify_image(raw: bytes) -> str:
        img = QImage.fromData(raw)
        if img.isNull():
            return "decode_failed"
        w = img.width()
        h = img.height()
        if w <= 0 or h <= 0:
            return "decode_failed"
        # Sample a grid of pixels for luminance variance.
        sx = max(1, w // 16)
        sy = max(1, h // 16)
        n = 0
        sum_y = 0.0
        sum2_y = 0.0
        for y in range(0, h, sy):
            for x in range(0, w, sx):
                c = img.pixelColor(x, y)
                yy = 0.2126 * c.red() + 0.7152 * c.green() + 0.0722 * c.blue()
                sum_y += yy
                sum2_y += yy * yy
                n += 1
        if n <= 0:
            return "decode_failed"
        mean = sum_y / n
        var_y = (sum2_y / n) - (mean * mean)
        # Placeholder tiles tend to be flat/gray with very low variance and often tiny payloads.
        # Keep thresholds strict to avoid false positives on pale/low-contrast basemaps.
        raw_len = len(raw)
        if 150.0 < mean < 235.0 and var_y < 50.0 and raw_len < 6000:
            return "placeholder_suspected"
        return "ok"

    def run(self) -> None:  # pragma: no cover - network dependent
        url = self._url
        try:
            raw = fetch_tile_http_bytes(url, timeout_s=5.0)
            code = 200
            ctype = "image"
            if int(code) >= 400:
                self._bridge.result.emit(
                    self._provider_label,
                    f"http_{int(code)}",
                    f"url={url} content_type={ctype}".strip(),
                )
                return
            if not raw:
                self._bridge.result.emit(
                    self._provider_label,
                    "empty_body",
                    f"url={url} content_type={ctype}".strip(),
                )
                return
            outcome = self._classify_image(raw)
            self._bridge.result.emit(
                self._provider_label,
                outcome,
                f"url={url} bytes={len(raw)} content_type={ctype}".strip(),
            )
        except Exception as e:
            self._bridge.result.emit(
                self._provider_label,
                f"error:{type(e).__name__}",
                f"url={url}",
            )
