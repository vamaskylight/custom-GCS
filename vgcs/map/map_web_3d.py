"""Optional Qt WebEngine view for legacy Leaflet + Cesium 3D (lazy-loaded on top of native 2D tiles)."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QUrl, Qt
from PySide6.QtWidgets import QWidget

try:
    from PySide6.QtWebEngineCore import (
        QWebEnginePage,
        QWebEngineProfile,
        QWebEngineSettings,
        QWebEngineUrlRequestInterceptor,
    )
    from PySide6.QtWebEngineWidgets import QWebEngineView

    class _TileHeaderInterceptor(QWebEngineUrlRequestInterceptor):  # type: ignore[misc]
        """Browser-like headers for tile CDNs (ported from e48c1a7 map_widget)."""

        def interceptRequest(self, info) -> None:  # pragma: no cover
            try:
                url = info.requestUrl().toString()
            except Exception:
                return
            if not url:
                return
            try:
                u = url.lower()
            except Exception:
                u = url
            try:
                if "openstreetmap.org" in u:
                    info.setHttpHeader(b"Referer", b"https://www.openstreetmap.org/")
                    info.setHttpHeader(
                        b"User-Agent",
                        b"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
                        b"(KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36",
                    )
                    return
                if "arcgisonline.com" in u or "arcgis.com" in u:
                    info.setHttpHeader(b"Referer", b"https://www.arcgis.com/")
                    info.setHttpHeader(
                        b"User-Agent",
                        b"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
                        b"(KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36",
                    )
                    return
            except Exception:
                return

    class _LoggingWebPage(QWebEnginePage):  # type: ignore[misc]
        def javaScriptConsoleMessage(
            self, level, message, lineNumber, sourceID
        ) -> None:  # pragma: no cover
            try:
                print(f"[VGCS:map-3d] {sourceID}:{int(lineNumber)} {message}")
            except Exception:
                pass

    HAS_WEBENGINE = True
except Exception:  # pragma: no cover - optional Qt module
    HAS_WEBENGINE = False
    QWebEngineView = None  # type: ignore[misc, assignment]
    QWebEnginePage = None  # type: ignore[misc, assignment]
    QWebEngineProfile = None  # type: ignore[misc, assignment]
    QWebEngineSettings = None  # type: ignore[misc, assignment]
    QWebEngineUrlRequestInterceptor = None  # type: ignore[misc, assignment]


def assets_base_url() -> QUrl:
    assets_root = (Path(__file__).resolve().parents[1] / "assets").resolve()
    return QUrl.fromLocalFile(str(assets_root) + "/")


def create_map_3d_web_view(parent: QWidget):
    """Return a configured QWebEngineView, or None if WebEngine is not installed."""
    if not HAS_WEBENGINE or QWebEngineView is None:
        return None
    w = QWebEngineView(parent)
    w.setMinimumHeight(260)
    w.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
    # Cesium uses right-drag / wheel heavily; Qt's default web context menu steals those gestures.
    try:
        w.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
    except Exception:
        pass
    try:
        w.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
    except Exception:
        pass
    try:
        if QWebEngineProfile is not None:
            prof = QWebEngineProfile.defaultProfile()
            try:
                prof.setUrlRequestInterceptor(_TileHeaderInterceptor())
            except Exception:
                pass
            cache_root = (Path.home() / ".vgcs-webengine-cache").resolve()
            cache_root.mkdir(parents=True, exist_ok=True)
            prof.setCachePath(str(cache_root))
            prof.setPersistentStoragePath(str(cache_root))
            try:
                prof.setHttpCacheType(QWebEngineProfile.HttpCacheType.DiskHttpCache)
                prof.setHttpCacheMaximumSize(512 * 1024 * 1024)
            except Exception:
                pass
    except Exception:
        pass
    try:
        w.setPage(_LoggingWebPage(w))
    except Exception:
        pass
    if QWebEngineSettings is not None:
        settings = w.settings()
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True
        )
        try:
            settings.setAttribute(
                QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True
            )
        except Exception:
            pass
        settings.setAttribute(QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled, False)
        settings.setAttribute(QWebEngineSettings.WebAttribute.ScrollAnimatorEnabled, False)
    return w
