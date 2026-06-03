"""M2 map scaffold with live position API and native Qt map (slippy tiles)."""

from __future__ import annotations

import base64
import csv
import json
import math
import os
import shutil
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import Request, urlopen

from PySide6.QtCore import (
    QByteArray,
    QBuffer,
    QObject,
    QPoint,
    QPointF,
    QSize,
    QRunnable,
    QSettings,
    QThreadPool,
    QTimer,
    Qt,
    QUrl,
)
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QFileDialog,
    QMessageBox,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
    QSizePolicy,
)
from PySide6.QtGui import (
    QBrush,
    QDesktopServices,
    QFont,
    QIcon,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QColor,
    QRadialGradient,
)
from vgcs.map.native_video_overlay import NativeVideoOverlayLayer, VideoOverlayDetection
from vgcs.observe.geo_reference import compute_geo_reference
from vgcs.observe.target_measure import (
    format_target_segment_label,
    haversine_m,
    is_downward_sensor_orientation,
    observation_target_latlon,
    resolve_vehicle_agl_m,
    segment_distance_between_rows,
    target_track_from_observations,
    video_mark_span_norm,
)
from vgcs.mission import (
    Waypoint,
    load_waypoints_json,
    save_waypoints_json,
    save_waypoints_kml,
)

# Optional: live camera preview for map overlay (M3 video pipeline).
from vgcs.video.pipeline import (
    HAS_MULTIMEDIA,
    VideoFrame,
    VideoPipeline,
    QS_KEY_LAST_PHOTO_SAVE_DIR,
    suggested_photo_save_path,
    suggested_recording_save_path,
    wait_qmedia_recorder_stopped,
)
from vgcs.video.camera_control import NoopCameraControl
from vgcs.map.native_tile_map import NativeTileMapView, bundled_seed_root, fetch_tile_http_bytes
from vgcs.map.legacy_leaflet_build import build_leaflet_html
from vgcs.map.map_footer_hud import (
    MapFooterCompass,
    MapFooterTelemetryStrip,
    TelemetryStripIcon,
)
from vgcs.map.sensor_obstacle_widget import ObstacleRadarPanel
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D, assets_base_url, create_map_3d_web_view
from vgcs.map.cam_rail_widgets import (
    CamObserveBlock,
    CamRailGimbalPad,
    CamRecordTimerRow,
)
from vgcs.map.plan_flight_panel import PlanFlightPanel

try:
    from PySide6.QtSvg import QSvgRenderer
except Exception:  # optional component on some Qt builds
    QSvgRenderer = None  # type: ignore[misc, assignment]

# Git `e48c1a7` `#cameraTopRow` SVG icons (`camIcon` classes).
_GIT_CAM_VIDEO_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">'
    '<rect x="3" y="7" width="12" height="10" rx="2" fill="#e8edf8"/>'
    '<polygon points="16,10 21,8 21,16 16,14" fill="#e8edf8"/>'
    "</svg>"
)
_GIT_CAM_PHOTO_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">'
    '<path d="M8 6h8l1.2 2H20a2 2 0 0 1 2 2v7a3 3 0 0 1-3 3H5a3 3 0 0 1-3-3v-7a2 2 0 0 1 2-2h2.8L8 6z" fill="#e8edf8"/>'
    '<circle cx="12" cy="13.5" r="3.2" fill="rgba(39,47,61,242)"/>'
    "</svg>"
)
# git `25970f0` `#camSplitBtn` (4-up) / `#camFollowBtn` — match `camIcon` stroke/fill.
_GIT_CAM_SPLIT_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">'
    '<rect x="3" y="3" width="8" height="8" rx="1.5" fill="#e8edf8"/>'
    '<rect x="13" y="3" width="8" height="8" rx="1.5" fill="#e8edf8"/>'
    '<rect x="3" y="13" width="8" height="8" rx="1.5" fill="#e8edf8"/>'
    '<rect x="13" y="13" width="8" height="8" rx="1.5" fill="#e8edf8"/>'
    "</svg>"
)
_GIT_CAM_FOLLOW_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">'
    '<circle cx="12" cy="12" r="6.5" fill="none" stroke="#e8edf8" stroke-width="1.75"/>'
    '<path d="M12 4.5v3.2M12 16.3v3.2M4.5 12h3.2M16.3 12h3.2" stroke="#e8edf8" stroke-width="2" stroke-linecap="round"/>'
    '<circle cx="12" cy="12" r="2.2" fill="#e8edf8"/>'
    "</svg>"
)


def _git_cam_icon_from_svg(svg_xml: str, logical_px: int = 22) -> QIcon:
    """Rasterize embedded SVG for toolbar-sized icons (falls back to empty QIcon if Svg unavailable)."""
    if QSvgRenderer is None:
        return QIcon()
    r = QSvgRenderer(QByteArray(svg_xml.encode("utf-8")))
    if not r.isValid():
        return QIcon()
    d = max(16, int(logical_px))
    pm = QPixmap(d, d)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    r.render(p)
    p.end()
    return QIcon(pm)


# Map action rail: icon size (logical px) — same line-art style as ``TelemetryStripIcon`` (footer HUD).
_MAP_ACTION_ICON_LOGICAL_PX = 26

# QSettings: Plan Flight Save vs map toolbar export use different "last file" keys.
_QS_NS = "VGCS"
_QS_APP = "VGCS"
_KEY_PLAN_CURRENT_MISSION_JSON = "plan_current_mission_json"
_KEY_TOOLBAR_EXPORT_MISSION_JSON = "toolbar_export_mission_json"
_KEY_PLAN_LAST_MISSION_JSON_LEGACY = "plan_last_mission_json"  # legacy; read fallback only
_KEY_MAP_OFFLINE_TILE_ROOT = "map_offline_tile_root"
_KEY_MAP_WEBCAM_ENABLED = "map_webcam_enabled"
_KEY_MAP_LOW_SPEC_MODE = "map_low_spec_mode"  # 'auto' | 'on' | 'off'
_KEY_MAP_TILE_MODE = "map_tile_mode"  # 'esri_streets' | 'osm' | 'sat' | 'offline'

# Last directory for camera-rail photo Save dialog (same key as ``QS_KEY_LAST_PHOTO_SAVE_DIR``).
_KEY_MEDIA_LAST_PHOTO_DIR = QS_KEY_LAST_PHOTO_SAVE_DIR

# M3 video settings (QSettings keys, written by Application Settings → Video).
_KEY_VIDEO_ENABLED = "video/enabled"
_KEY_VIDEO_SOURCE = "video/source"  # 'disabled' | 'rtsp' | 'udp_h264' | 'udp_h265'
_KEY_VIDEO_RTSP_DAY = "video/rtsp_day"
_KEY_VIDEO_RTSP_THERMAL = "video/rtsp_thermal"
_KEY_VIDEO_RTSP_TRANSPORT = "video/rtsp_transport"  # 'auto' | 'udp' | 'tcp'
_KEY_VIDEO_LOW_LATENCY = "video/low_latency"
_KEY_VIDEO_RECORD_FORMAT = "video/record_format"  # 'mp4' | 'mkv'
_KEY_VIDEO_DEFAULT_VIEW = "video/default_view"  # 'Single' | 'Split'

# Native HUD top inset from the map canvas (px) — Takeoff/Return and camera rail share the same gap below the app header.
# Legacy Web used ~78px to clear an in-map `#linkBanner`; native 2D has no banner, so keep this tight.
_MAP_HUD_TOP_PX = 10
_MAP_ACTION_RAIL_LEFT_PX = 10
_MAP_ACTION_RAIL_TOP_PX = _MAP_HUD_TOP_PX
_NATIVE_CAM_RAIL_TOP_PX = _MAP_HUD_TOP_PX
# LENS row: 42 + 6 + (16+34+34)*2 + 6 + ~11 layout margins.
_NATIVE_CAM_RAIL_MIN_WIDTH_PX = 258
# Camera rail vertical rhythm (touch targets / gaps — do not change font-size in QSS below).
_CAM_RAIL_PAD_BTN_H = 34
_CAM_RAIL_PAD_BTN_W = 40
_CAM_RAIL_LENS_ROW_H = 40
_CAM_RAIL_LENS_BTN_H = 34
_CAM_RAIL_LAYOUT_SPACING = 5
_CAM_RAIL_GIMBAL_GRID_GAP = 5
# Native HUD margins (mini-video bottom-left; obstacle top-left under Takeoff/Return).
_MAP_HUD_MARGIN_PX = 12
_MAP_ACTION_RAIL_HEIGHT_PX = 54 + 8 + 54
_OBSTACLE_PANEL_TOP_PX = _MAP_ACTION_RAIL_TOP_PX + _MAP_ACTION_RAIL_HEIGHT_PX + 8
# Must fit ObstacleRadarPanel.sizeHint() — do not clamp below panel minimum or widgets overlap.
_OBSTACLE_PANEL_MAX_H_PX = 360
# Fixed mini-video PiP (bottom-left) — do not scale to a large % of the map.
_MINI_VIDEO_PIP_W_PX = 236
_MINI_VIDEO_PIP_H_PX = 132

# Primary 2D map: NativeTileMapView only. Optional 3D globe: lazy Qt WebEngine + Cesium (see map_web_3d).
HAS_WEBENGINE = HAS_WEBENGINE_3D
# Bumped when map loading / fallback behaviour changes (visible in client console).
MAP_BACKEND_BUILD = "2026-05-18-native2d-telemetry"
# Map icon/track only move after sustained GPS speed (avoids ground jitter + false vx/vy).
_MAP_MOVE_ARM_SPEED_MPS = 1.0
_MAP_MOVE_DISARM_SPEED_MPS = 0.35
_MAP_MOVE_ARM_SAMPLES = 5
_MAP_MOVE_DISARM_SAMPLES = 15
_MAP_POSITION_MIN_MOVE_M = 2.0

def _web_2d_fallback_allowed() -> bool:
    """2D map is NativeTileMapView only; WebEngine Leaflet is opt-in for debugging."""
    return str(os.environ.get("VGCS_ALLOW_WEB_2D_FALLBACK", "0") or "0").strip() == "1"

_WEB_MAP_RELAYOUT_JS = """
(function(){
  try {
    if (typeof window.__vgcsRelayoutMap2d === 'function') {
      window.__vgcsRelayoutMap2d();
      return;
    }
    var m2 = document.getElementById('map2d');
    var m3 = document.getElementById('map3d');
    if (m2) m2.style.display = 'block';
    if (m3) m3.style.display = 'none';
    if (typeof map !== 'undefined' && map) {
      map.invalidateSize(true);
      if (window.__vgcsPendingCenter && window.__vgcsPendingCenter.length === 2) {
        var zz = window.__vgcsPendingZoom;
        if (typeof zz !== 'number' || !isFinite(zz)) zz = map.getZoom();
        map.setView(window.__vgcsPendingCenter, zz, {animate: false});
      }
    }
  } catch (e) {
    try { console.log('[diag] relayout err=' + String(e)); } catch (e2) {}
  }
})();
"""

# Legacy placeholder (HTML lives in legacy_leaflet_map.html + legacy_leaflet_build).
LEAFLET_HTML = ""


def _save_qimage_to_path(img: QImage, path: Path) -> bool:
    """Write ``QImage`` using extension: ``.png`` → PNG, else JPEG."""
    try:
        if path.suffix.lower() == ".png":
            return bool(img.save(str(path), "PNG"))
        return bool(img.save(str(path), "JPG", 92))
    except Exception:
        return False


def _native_cam_record_dot_pixmap(size: int = 14) -> QPixmap:
    """Inner red dot raster for `#camRecordBtn` (`size` px, matches button scale)."""
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    rg = QRadialGradient(float(size) * 0.5, float(size) * 0.35, float(size) * 0.48)
    rg.setColorAt(0.0, QColor("#ff4b4b"))
    rg.setColorAt(1.0, QColor("#f62d2d"))
    p.setBrush(QBrush(rg))
    p.setPen(Qt.PenStyle.NoPen)
    p.drawEllipse(0, 0, size, size)
    p.end()
    return pm


# Native HUD glass — same opacity as ``MapFooterTelemetryStrip`` (`map_footer_hud.py`).
_MAP_HUD_GLASS_BG = "rgba(26, 33, 45, 215)"
_MAP_HUD_GLASS_BORDER = "rgba(80, 92, 118, 107)"

# Native `#cameraRail` — label/button text matches `TELEMETRY_STRIP_VALUE_STYLE` in map_footer_hud.py
# (`color: #dce5f5; font-size: 15px; font-weight: 600`).
# Inner rail: transparent (glass is on ``#nativeCameraRailLayer``); control chrome below.
_NATIVE_CAMERA_RAIL_QSS = (
    "QFrame#nativeCameraRail {\n"
    "  background: transparent;\n"
    "  border: none;\n"
    f"  min-width: {_NATIVE_CAM_RAIL_MIN_WIDTH_PX}px;\n"
    '  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;\n'
    "  color: #dce5f5;\n"
    "  font-size: 15px;\n"
    "  font-weight: 600;\n"
    "}\n"
    "#cameraTopRow {\n"
    "  background: transparent;\n"
    "  border: none;\n"
    "}\n"
) + """
QPushButton#camVideoBtn {
  width: 28px;
  height: 28px;
  max-width: 28px;
  max-height: 28px;
  min-width: 28px;
  min-height: 28px;
  border-radius: 14px;
  padding: 0px;
  margin: 0px;
  margin-right: 2px;
  border: 1px solid transparent;
  background: transparent;
  color: #dce5f5;
  font-size: 15px;
  font-weight: 600;
}
QPushButton#camVideoBtn:hover {
  background: rgba(110, 123, 148, 51);
}
QPushButton#camVideoBtn:checked {
  border: 1px solid rgba(214, 224, 241, 230);
  background: rgba(27, 33, 45, 245);
}
QPushButton#camPhotoBtn {
  width: 28px;
  height: 28px;
  min-width: 28px;
  max-width: 28px;
  min-height: 28px;
  max-height: 28px;
  border-radius: 14px;
  padding: 0px;
  margin: 0px;
  margin-right: 2px;
  border: 1px solid transparent;
  background: transparent;
  color: #dce5f5;
  font-size: 15px;
  font-weight: 600;
}
QPushButton#camPhotoBtn:hover {
  background: rgba(110, 123, 148, 51);
}
QPushButton#camPhotoBtn:checked {
  border: 1px solid rgba(214, 224, 241, 230);
  background: rgba(27, 33, 45, 245);
}
/* Split / Follow: icon-only — same 28px footprint as `#camPhotoBtn` (git `camSplitBtn` / `camFollowBtn`). */
QPushButton#camSplitBtn, QPushButton#camFollowBtn {
  width: 28px;
  height: 28px;
  min-width: 28px;
  max-width: 28px;
  min-height: 28px;
  max-height: 28px;
  padding: 0px;
  margin: 0px;
  border-radius: 7px;
  border: 1px solid rgba(196, 209, 230, 55);
  background-color: rgba(22, 27, 38, 235);
  color: #e8edf8;
}
QPushButton#camSplitBtn:hover, QPushButton#camFollowBtn:hover {
  background-color: rgba(40, 48, 62, 245);
  border-color: rgba(229, 237, 251, 85);
}
QPushButton#camSplitBtn:checked, QPushButton#camFollowBtn:checked {
  border: 1px solid rgba(105, 232, 111, 220);
  background-color: rgba(24, 52, 34, 250);
  color: #c8ffc8;
}
/* Split is logically on but main canvas is a single zoomed channel (not the 2×2 composite): neutral chrome. */
QPushButton#camSplitBtn:checked[splitHidden="true"] {
  border: 1px solid rgba(196, 209, 230, 55);
  background-color: rgba(22, 27, 38, 235);
  color: #e8edf8;
}
QPushButton#camRecordBtn {
  width: 34px;
  height: 34px;
  min-width: 34px;
  max-width: 34px;
  min-height: 34px;
  max-height: 34px;
  padding: 0px;
  border-radius: 17px;
  border: 1px solid rgba(231, 239, 255, 180);
  background: qradialgradient(cx:0.5, cy:0.35, radius:0.7, fx:0.5, fy:0.35,
    stop:0 rgba(35, 43, 57, 250), stop:1 rgba(20, 26, 36, 250));
}
QPushButton#camRecordBtn:checked {
  border: 1px solid rgba(255, 130, 130, 220);
}
QLabel#camTimer {
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  color: #dce5f5;
  background: rgba(255, 65, 65, 220);
  border-radius: 6px;
  padding: 3px 10px;
  min-width: 82px;
  border: none;
}
QLabel#camSectionHeader {
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  letter-spacing: 0.06em;
  padding: 0px 1px 2px 1px;
  margin-top: 0px;
  border: none;
  background: transparent;
  min-height: 18px;
}
QLabel#camSectionHeaderInline {
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  letter-spacing: 0.04em;
  padding: 0px 4px 0px 0px;
  border: none;
  background: transparent;
  min-width: 58px;
  max-width: 58px;
}
QLabel#camAxisLabel {
  color: #c8d3ea;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  padding: 0px;
  margin: 0px;
  border: none;
  background: transparent;
  min-width: 16px;
  max-width: 16px;
}
QLabel#camSectionHeaderLens {
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  letter-spacing: 0.04em;
  padding: 0px 2px 0px 0px;
  border: none;
  background: transparent;
  min-width: 42px;
  max-width: 42px;
}
QPushButton[camLensPadBtn=true] {
  min-width: 34px;
  max-width: 34px;
  min-height: 34px;
  max-height: 34px;
  border-radius: 6px;
  border: 1px solid rgba(196, 209, 230, 38);
  background: rgba(18, 22, 32, 75);
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  padding: 0px;
}
QPushButton[camLensPadBtn=true]:hover {
  background: rgba(110, 123, 148, 45);
  border-color: rgba(229, 237, 251, 70);
}
QWidget#camLensRow {
  background: transparent;
  min-height: 40px;
  max-height: 40px;
}
QWidget#camLensControls, QWidget#camInlineRowBody {
  background: transparent;
}
QFrame#camRecordArch {
  margin-top: 0px;
  border: none;
  background: transparent;
}
QFrame#camRailSep {
  background: rgba(188, 202, 224, 45);
  max-height: 1px;
  min-height: 1px;
  border: none;
  margin-top: 3px;
  margin-bottom: 3px;
}
QPushButton#observeTarget, QPushButton#observeClip,
QPushButton#observeReport, QPushButton#observeReset {
  min-height: 34px;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  border-radius: 6px;
  border: 1px solid rgba(196, 209, 230, 38);
  background: rgba(18, 22, 32, 75);
  color: #dce5f5;
  padding: 2px 6px;
}
QPushButton#observeTarget:hover, QPushButton#observeClip:hover,
QPushButton#observeReport:hover, QPushButton#observeReset:hover {
  background: rgba(110, 123, 148, 45);
  border-color: rgba(229, 237, 251, 70);
}
QPushButton#observeTarget:checked, QPushButton#observeClip:checked {
  border-color: rgba(214, 224, 241, 230);
  background: rgba(27, 33, 45, 245);
}
QPushButton#observeClip[recording="true"] {
  background: rgba(200, 45, 45, 240);
  border-color: rgba(255, 130, 130, 220);
  color: #ffffff;
  font-weight: 700;
}
QPushButton[camPadBtn=true] {
  min-width: 40px;
  min-height: 34px;
  border-radius: 6px;
  border: 1px solid rgba(196, 209, 230, 38);
  background: rgba(18, 22, 32, 75);
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 15px;
  font-weight: 600;
  padding: 1px 4px;
}
QPushButton[camPadBtn=true]:hover {
  background: rgba(110, 123, 148, 45);
  border-color: rgba(229, 237, 251, 70);
}
QPushButton[camPadBtn=true]:checked {
  border-color: rgba(214, 224, 241, 230);
  background: rgba(27, 33, 45, 245);
}
"""


def _cam_rail_sep() -> QFrame:
    """Divider between git `#cameraRail` core (video…settings) and MAVLink pad extras."""
    f = QFrame()
    f.setObjectName("camRailSep")
    f.setFixedHeight(1)
    return f


def _cam_rail_inline_row(label: str, body: QWidget) -> QWidget:
    """Compact row: section label + controls (one line, no extra header row)."""
    w = QWidget()
    w.setObjectName("camInlineRow")
    w.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 2, 0, 2)
    h.setSpacing(8)
    lab = QLabel(label)
    lab.setObjectName("camSectionHeaderInline")
    lab.setAlignment(
        Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
    )
    body.setObjectName("camInlineRowBody")
    body.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    h.addWidget(lab, 0, Qt.AlignmentFlag.AlignVCenter)
    h.addWidget(body, 1, Qt.AlignmentFlag.AlignVCenter)
    return w


def _cam_lens_pad_btn(btn: QPushButton) -> QPushButton:
    """Lens −/+ only — QSS min/max must match fixed size or Qt overlaps widgets."""
    btn.setProperty("camLensPadBtn", True)
    btn.setFixedSize(34, _CAM_RAIL_LENS_BTN_H)
    btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    return btn


def _cam_rail_lens_row(
    zoom_minus: QPushButton,
    zoom_plus: QPushButton,
    focus_minus: QPushButton,
    focus_plus: QPushButton,
) -> QWidget:
    """Single flat row: LENS · Z − + · F − + (fixed height so buttons are not vertically clipped)."""
    row = QWidget()
    row.setObjectName("camLensRow")
    row.setFixedHeight(_CAM_RAIL_LENS_ROW_H)
    row.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
    lay = QHBoxLayout(row)
    lay.setContentsMargins(0, 4, 0, 4)
    lay.setSpacing(8)
    lab = QLabel("LENS")
    lab.setObjectName("camSectionHeaderLens")
    lab.setFixedWidth(42)
    lab.setAlignment(
        Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
    )
    lay.addWidget(lab, 0, Qt.AlignmentFlag.AlignVCenter)

    def _add_axis(cap: str, minus_btn: QPushButton, plus_btn: QPushButton) -> None:
        cap_lbl = QLabel(cap)
        cap_lbl.setObjectName("camAxisLabel")
        cap_lbl.setFixedWidth(16)
        cap_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        _cam_lens_pad_btn(minus_btn)
        _cam_lens_pad_btn(plus_btn)
        lay.addWidget(cap_lbl, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(minus_btn, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(plus_btn, 0, Qt.AlignmentFlag.AlignVCenter)

    _add_axis("Z", zoom_minus, zoom_plus)
    lay.addSpacing(6)
    _add_axis("F", focus_minus, focus_plus)
    lay.addStretch(1)
    return row


def _cam_pad_btn(
    btn: QPushButton,
    *,
    width: int = _CAM_RAIL_PAD_BTN_W,
    height: int = _CAM_RAIL_PAD_BTN_H,
) -> QPushButton:
    btn.setProperty("camPadBtn", True)
    btn.setFixedSize(int(width), int(height))
    return btn


class _VideoEncodeBridge(QObject):
    encoded = Signal(str)


class _VideoEncodeTask(QRunnable):
    def __init__(
        self,
        img,
        bridge: _VideoEncodeBridge,
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
        self._encode_format = "PNG" if fmt not in ("PNG", "JPG", "JPEG") else ("JPG" if fmt == "JPEG" else fmt)
        # PNG: 0..9 compression level (0 fastest). JPG: 1..100 quality.
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
                # Avoid upscaling small frames; only downscale when above cap.
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


class _ObservationSnapshotBridge(QObject):
    finished = Signal(int, str)  # observation index, snapshot path (may be empty)


class _ObservationSnapshotTask(QRunnable):
    """Save a preview still off the GUI thread (Target / Report must not freeze the app)."""

    def __init__(self, img: QImage, dest: Path, idx: int, bridge: _ObservationSnapshotBridge) -> None:
        super().__init__()
        self._img = img
        self._dest = dest
        self._idx = int(idx)
        self._bridge = bridge

    def run(self) -> None:
        path = ""
        try:
            if _save_qimage_to_path(self._img, self._dest):
                path = str(self._dest)
        except Exception:
            path = ""
        try:
            self._bridge.finished.emit(self._idx, path)
        except Exception:
            pass


class _ObservationExportBridge(QObject):
    finished = Signal(bool, str)  # ok, summary message


class _ObservationExportTask(QRunnable):
    def __init__(
        self,
        *,
        rows: list[dict[str, object]],
        csv_path: str,
        html_path: str,
        obs_cell_fn,
        bridge: _ObservationExportBridge,
    ) -> None:
        super().__init__()
        self._rows = list(rows)
        self._csv_path = str(csv_path)
        self._html_path = str(html_path)
        self._obs_cell_fn = obs_cell_fn
        self._bridge = bridge

    def run(self) -> None:
        fields = [
            "timestamp_utc",
            "kind",
            "map_lat",
            "map_lon",
            "video_x_norm",
            "video_y_norm",
            "vehicle_lat",
            "vehicle_lon",
            "vehicle_heading_deg",
            "vehicle_roll_deg",
            "vehicle_pitch_deg",
            "vehicle_rel_alt_m",
            "gimbal_yaw_deg",
            "gimbal_pitch_deg",
            "gps_fix_type",
            "gps_satellites",
            "gps_hdop",
            "target_lat",
            "target_lon",
            "target_alt_m",
            "geo_quality",
            "geo_warning",
            "geo_method",
            "geo_range_m",
            "geo_bearing_deg",
            "segment_distance_m",
            "agl_source",
            "snapshot_path",
            "clip_path",
        ]
        ok = False
        summary = ""
        try:
            with open(self._csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                for row in self._rows:
                    w.writerow({k: row.get(k) for k in fields})
            html_rows = []
            for idx, row in enumerate(self._rows, start=1):
                html_rows.append(
                    "<tr>"
                    f"<td>{idx}</td>"
                    f"<td>{row.get('timestamp_utc','')}</td>"
                    f"<td>{row.get('kind','')}</td>"
                    f"<td>{row.get('map_lat','')}</td>"
                    f"<td>{row.get('map_lon','')}</td>"
                    f"<td>{self._obs_cell_fn(row.get('target_lat'))}</td>"
                    f"<td>{self._obs_cell_fn(row.get('target_lon'))}</td>"
                    f"<td>{row.get('geo_quality','')}</td>"
                    f"<td>{row.get('vehicle_lat','')}</td>"
                    f"<td>{row.get('vehicle_lon','')}</td>"
                    f"<td>{self._obs_cell_fn(row.get('gimbal_yaw_deg'))}</td>"
                    f"<td>{self._obs_cell_fn(row.get('gimbal_pitch_deg'))}</td>"
                    f"<td>{row.get('video_x_norm','')}</td>"
                    f"<td>{row.get('video_y_norm','')}</td>"
                    f"<td>{row.get('vehicle_rel_alt_m','')}</td>"
                    f"<td>{row.get('geo_range_m','')}</td>"
                    f"<td>{row.get('segment_distance_m','')}</td>"
                    f"<td>{row.get('gps_fix_type','')}</td>"
                    f"<td>{row.get('gps_satellites','')}</td>"
                    f"<td>{row.get('gps_hdop','')}</td>"
                    f"<td>{row.get('geo_warning','')}</td>"
                    f"<td>{row.get('snapshot_path','')}</td>"
                    f"<td>{row.get('clip_path','')}</td>"
                    "</tr>"
                )
            html = (
                "<!doctype html><html><head><meta charset='utf-8'/>"
                "<title>Observation Summary</title>"
                "<style>body{font-family:Segoe UI,Arial,sans-serif;padding:20px;} table{border-collapse:collapse;width:100%;}"
                "th,td{border:1px solid #ccc;padding:6px;font-size:12px;} th{background:#f3f6fb;text-align:left;}</style>"
                "</head><body>"
                f"<h2>Observation Report ({len(self._rows)} entries)</h2>"
                "<table><thead><tr>"
                "<th>#</th><th>UTC Time</th><th>Kind</th><th>Map Lat</th><th>Map Lon</th>"
                "<th>Target Lat</th><th>Target Lon</th><th>Geo Quality</th>"
                "<th>Vehicle Lat</th><th>Vehicle Lon</th><th>Gimbal Yaw</th><th>Gimbal Pitch</th>"
                "<th>Video X</th><th>Video Y</th><th>Rel Alt (m)</th><th>Geo Range (m)</th>"
                "<th>Target Sep (m)</th>"
                "<th>GPS Fix</th><th>GPS Sats</th><th>HDOP</th><th>Geo Warning</th>"
                "<th>Snapshot</th><th>Clip</th>"
                "</tr></thead><tbody>"
                + "".join(html_rows)
                + "</tbody></table></body></html>"
            )
            Path(self._html_path).write_text(html, encoding="utf-8")
            csv_abs = str(Path(self._csv_path).resolve())
            html_abs = str(Path(self._html_path).resolve())
            summary = f"Exported {len(self._rows)} observation(s):\n{csv_abs}\n{html_abs}"
            ok = True
        except Exception as e:
            summary = f"Observation export failed: {e}"
            ok = False
        try:
            self._bridge.finished.emit(bool(ok), summary)
        except Exception:
            pass


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


class MapWidget(QWidget):
    """Map panel with Leaflet backend and waypoint click workflow."""
    waypoints_changed = Signal(list)  # list[Waypoint]
    mission_upload_requested = Signal(list)  # list[Waypoint]
    mission_download_requested = Signal()
    geofence_upload_requested = Signal(object)  # dict fence settings
    connect_requested = Signal()
    menu_requested = Signal(int, int)
    takeoff_requested = Signal()
    return_requested = Signal()
    plan_tool_requested = Signal(str)
    plan_action_requested = Signal(str)
    plan_flight_exited = Signal()
    map_page_ready = Signal()
    toggle_3d_requested = Signal()
    map_3d_mode_changed = Signal()  # _is_3d_mode updated (async load / JS / back to 2D)
    mission_start_requested = Signal()
    plan_mission_panel_changed = Signal(object)
    video_follow_enabled_changed = Signal(bool)

    def __init__(self, parent=None, *, video_pipeline: VideoPipeline | None = None) -> None:
        super().__init__(parent)
        # When embedded in MainWindow, share its VideoPipeline so RTSP is decoded once.
        # Without this, map PiP + footer "Split camera video" each ran a separate pipeline (duplicate UI).
        self._video_pipeline_shared: VideoPipeline | None = video_pipeline
        # When `VideoPipeline.refresh_sources()` finishes, `RtspSource` instances are replaced;
        # reconnect preview + frame slots (see `_on_video_pipeline_sources_changed`).
        self._video_sources_changed_conn_id: int | None = None
        if self._video_pipeline_shared is not None:
            self._hook_video_pipeline_sources_changed(self._video_pipeline_shared)
        self._lat: float | None = None
        self._lon: float | None = None
        self._map_display_lat: float | None = None
        self._map_display_lon: float | None = None
        self._last_groundspeed_mps = 0.0
        self._map_motion_armed = False
        self._map_speed_hi_streak = 0
        self._map_speed_lo_streak = 0
        self._heading: float | None = None
        self._waypoint_count = 0
        self._waypoints_model: list[Waypoint] = []
        self._web_ready = False
        self._is_3d_mode = False
        self._web_3d_view = None
        self._web_3d_ready = False
        self._pending_3d_activate = False
        self._fence_radius_m = 80.0
        self._last_plan_flight_metrics_payload: dict[str, object] | None = None
        self._vehicle_pose_timer = QTimer(self)
        self._vehicle_pose_timer.setSingleShot(True)
        # Throttle JS bridge updates; very high rates can make WebEngine feel laggy on low-end devices.
        self._vehicle_pose_timer.setInterval(120)
        self._vehicle_pose_timer.timeout.connect(self._flush_vehicle_pose_js)
        self._heading_js_source = "mixed"
        self._last_flight_telemetry_sig: str | None = None
        self._last_link_connected: bool | None = None
        self._last_flight_status_key: tuple[str, str] | None = None
        self._last_header_gps_key: tuple[str, str] | None = None
        self._last_header_battery: str | None = None
        self._last_header_mode: str | None = None
        self._last_plan_vehicle_info_key: tuple[str, str] | None = None
        self._plan_rail_tool_state = "File"
        self._tile_error_notified = False
        self._low_spec_effective = False
        self._low_spec_autodetected = False
        self._tile_probe_bridge = _TileProbeBridge(self)
        self._tile_probe_bridge.result.connect(self._on_tile_probe_result)
        self._tile_probe_ran = False
        self._native_tile_fallback_done = False
        self._native_tile_startup_retries = 0
        self._video_follow_enabled = True
        self._web_2d_fallback_active = False
        self._pending_web_2d_fallback = False

        # M3 video settings snapshot (applied lazily when video backend initializes).
        self._video_settings_enabled = False
        self._video_settings_source = "rtsp"
        self._video_settings_day = ""
        self._video_settings_thermal = ""
        self._video_settings_rtsp_transport = "auto"
        self._video_settings_default_view = "Single"
        self._native_video_last_frame_mono = 0.0
        self._video_preview_got_frame = False
        self._video_preview_started_mono = 0.0
        self._video_preview_stall_recovery_active = False
        self._camera_control = NoopCameraControl()
        self._obs_mark_mode = False
        self._observations: list[dict[str, object]] = []
        self._video_obs_marks: list[tuple[float, float]] = []
        self._obs_snapshot_bridge = _ObservationSnapshotBridge(self)
        self._obs_snapshot_bridge.finished.connect(self._on_observation_snapshot_saved)
        self._obs_export_bridge = _ObservationExportBridge(self)
        self._obs_export_bridge.finished.connect(self._on_observation_export_finished)
        self._obs_export_busy = False
        self._obs_export_quick = False
        self._obs_marks_overlay_timer: QTimer | None = None
        self._video_ui_render_mono = 0.0
        self._split_ui_render_mono = 0.0
        self._split_cache_mono: dict[str, float] = {}
        self._split_render_timer: QTimer | None = None
        self._video_cache_mono = 0.0
        self._obs_clip_active = False
        self._obs_clip_secs_left = 0
        self._obs_clip_countdown_timer: QTimer | None = None
        self._obs_clip_banner: QLabel | None = None
        self._ai_phase = 0.0
        self._payload_hardware_recording = False
        self._vehicle_rel_alt_m: float | None = None
        self._rangefinder_down_m: float | None = None
        self._vehicle_alt_msl_m: float | None = None
        self._vehicle_roll_deg: float | None = None
        self._vehicle_pitch_deg: float | None = None
        self._gps_fix_type: int = 0
        self._gps_satellites: int = 0
        self._gps_hdop: float | None = None
        # Camera rail: "video" = live/shooting (record toggles); "photo" = still mode (center = shutter).
        self._camera_rail_ui_mode: str = "video"

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        panel = QGroupBox("3D Map")
        self._panel = panel
        panel_layout = QVBoxLayout()
        self._panel_layout = panel_layout
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(0)

        self._map_canvas = QFrame()
        self._map_canvas.setObjectName("statusChip")
        self._map_canvas_layout = QVBoxLayout()
        self._map_canvas_layout.setContentsMargins(0, 0, 0, 0)
        self._map_canvas_layout.setSpacing(0)
        self._map_canvas.setLayout(self._map_canvas_layout)
        # Parent `_panel` (not `_map_canvas`) so PiP stacks above the tile map like compass/rail.
        self._native_video_preview = QLabel(self._panel)
        self._native_video_preview.setObjectName("nativeVideoPreview")
        self._native_video_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._native_video_preview.setAutoFillBackground(True)
        self._native_video_preview.setStyleSheet(
            "QLabel#nativeVideoPreview {"
            "background: #000;"
            "border: 1px solid rgba(206, 220, 242, 0.35);"
            "border-radius: 8px;"
            "}"
        )
        self._native_video_preview.hide()
        self._native_video_preview.raise_()
        self._native_video_overlay = NativeVideoOverlayLayer(self._native_video_preview)
        self._native_video_overlay.hide()
        self._native_video_last = QImage()
        self._video_swapped = False
        # Operator chose map-main (PiP video); do not auto-force fullscreen video on gaps/first frame.
        self._video_swap_user_map_main = False
        # Split PiP → fullscreen: which source to fill the canvas (None = entire 2×2 grid).
        self._split_fullscreen_source_id: str | None = None
        # Last 2×2 composite geometry + per-slot source ids (for hit-testing PiP clicks).
        self._split_layout_snapshot: dict[str, object] | None = None
        # Last PiP paint: pixmap rect in label logical coords + source image size (fixes HiDPI hit-test).
        self._split_pip_hit: dict[str, float] | None = None
        self._native_overlay_insets = {
            "left": 170,
            "top": _NATIVE_CAM_RAIL_TOP_PX,
            "right": 192,
            "bottom": 130,
        }
        self._native_video_preview.mousePressEvent = self._on_native_video_click  # type: ignore[assignment]
        # Rail is a **sibling** of `_map_canvas` under the map panel (not a child of `_map_canvas`).
        # PiP `QLabel` + tile map live inside `_map_canvas`; keeping the rail on the same surface made
        # `setPixmap`/stacking reorder the PiP above the rail on some platforms (photo / rail unclickable).
        # Opaque `QFrame` host: on Windows a bare `QWidget` can be hit-transparent; clicks fall through to the map.
        self._native_rail_layer = QFrame(panel)
        self._native_rail_layer.setObjectName("nativeCameraRailLayer")
        self._native_rail_layer.setFrameShape(QFrame.Shape.NoFrame)
        self._native_rail_layer.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        # One continuous glass panel (same as ``MapFooterTelemetryStrip``) — rgba in QSS, not WA_TranslucentBackground.
        self._native_rail_layer.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_rail_layer.setAutoFillBackground(False)
        self._native_rail_layer.setStyleSheet(
            "QFrame#nativeCameraRailLayer {"
            " background: " + _MAP_HUD_GLASS_BG + ";"
            " border: 1px solid " + _MAP_HUD_GLASS_BORDER + ";"
            " border-radius: 14px;"
            "}"
        )
        self._native_rail_layer.hide()
        # Native `#cameraRail`: single `QFrame` — height follows layout (no `QScrollArea` scrollbar).
        self._native_hud_right = QFrame(self._native_rail_layer)
        self._native_hud_right.setObjectName("nativeCameraRail")
        self._native_hud_right.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_hud_right.setAutoFillBackground(False)
        self._native_hud_right.setStyleSheet(_NATIVE_CAMERA_RAIL_QSS)
        self._native_hud_right_layout = QVBoxLayout(self._native_hud_right)
        self._native_hud_right_layout.setContentsMargins(8, 7, 8, 8)
        self._native_hud_right_layout.setSpacing(_CAM_RAIL_LAYOUT_SPACING)

        self._camera_top_row = QFrame(self._native_hud_right)
        self._camera_top_row.setObjectName("cameraTopRow")
        ctr_layout = QHBoxLayout(self._camera_top_row)
        ctr_layout.setContentsMargins(3, 3, 3, 3)
        ctr_layout.setSpacing(4)

        self._btn_native_video = QPushButton()
        self._btn_native_video.setObjectName("camVideoBtn")
        self._btn_native_video.setCheckable(True)
        self._btn_native_video.setChecked(True)
        self._btn_native_video.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        _ico_main = 20
        _iv = _git_cam_icon_from_svg(_GIT_CAM_VIDEO_SVG, _ico_main)
        if _iv.isNull():
            self._btn_native_video.setText("🎥")
        else:
            self._btn_native_video.setIcon(_iv)
            self._btn_native_video.setIconSize(QSize(_ico_main, _ico_main))

        self._btn_native_photo = QPushButton()
        self._btn_native_photo.setObjectName("camPhotoBtn")
        self._btn_native_photo.setCheckable(True)
        self._btn_native_photo.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_photo.setToolTip("Photo mode — use the red shutter below to capture")
        self._btn_native_video.setToolTip("Video / live mode — red button records")
        _ip = _git_cam_icon_from_svg(_GIT_CAM_PHOTO_SVG, _ico_main)
        if _ip.isNull():
            self._btn_native_photo.setText("📷")
        else:
            self._btn_native_photo.setIcon(_ip)
            self._btn_native_photo.setIconSize(QSize(_ico_main, _ico_main))

        ctr_layout.addWidget(self._btn_native_video)
        ctr_layout.addWidget(self._btn_native_photo)

        # git `25970f0` `#cameraTopRow`: Split (4-up) + Follow — same embedded SVGs as legacy `camIcon` row.
        self._btn_native_split = QPushButton()
        self._btn_native_split.setObjectName("camSplitBtn")
        self._btn_native_split.setCheckable(True)
        self._btn_native_split.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_split.setToolTip("Split view (4-up)")
        self._btn_native_split.setFixedSize(28, 28)
        self._btn_native_split.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        _is = _git_cam_icon_from_svg(_GIT_CAM_SPLIT_SVG, _ico_main)
        if _is.isNull():
            self._btn_native_split.setText("▦")
        else:
            self._btn_native_split.setIcon(_is)
            self._btn_native_split.setIconSize(QSize(_ico_main, _ico_main))

        self._btn_native_follow = QPushButton()
        self._btn_native_follow.setObjectName("camFollowBtn")
        self._btn_native_follow.setCheckable(True)
        self._btn_native_follow.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_follow.setToolTip("Follow vehicle (center map)")
        self._btn_native_follow.setFixedSize(28, 28)
        self._btn_native_follow.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        _ifw = _git_cam_icon_from_svg(_GIT_CAM_FOLLOW_SVG, _ico_main)
        if _ifw.isNull():
            self._btn_native_follow.setText("◎")
        else:
            self._btn_native_follow.setIcon(_ifw)
            self._btn_native_follow.setIconSize(QSize(_ico_main, _ico_main))

        ctr_layout.addWidget(self._btn_native_split, 0)
        ctr_layout.addWidget(self._btn_native_follow, 0)
        ctr_layout.addStretch(1)

        self._btn_native_record = QPushButton()
        self._btn_native_record.setObjectName("camRecordBtn")
        self._btn_native_record.setCheckable(True)
        self._btn_native_record.setFixedSize(34, 34)
        self._btn_native_record.setText("")
        self._btn_native_record.setIcon(QIcon(_native_cam_record_dot_pixmap(14)))
        self._btn_native_record.setIconSize(QSize(14, 14))
        self._btn_native_record.setToolTip("Record video")

        self._lbl_native_cam_timer = QLabel("00:00:00")
        self._lbl_native_cam_timer.setObjectName("camTimer")
        self._lbl_native_cam_timer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_native_cam_timer.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        self._native_hud_right_layout.addWidget(self._camera_top_row)
        self._native_hud_right_layout.addWidget(
            CamRecordTimerRow(
                self._btn_native_record,
                self._lbl_native_cam_timer,
                self._native_hud_right,
            )
        )

        self._btn_native_zoom_minus = QPushButton("−")
        self._btn_native_zoom_plus = QPushButton("+")
        self._btn_native_zoom_minus.setToolTip("Zoom out")
        self._btn_native_zoom_plus.setToolTip("Zoom in")
        self._btn_native_focus_minus = QPushButton("−")
        self._btn_native_focus_plus = QPushButton("+")
        self._btn_native_focus_minus.setToolTip("Focus nearer")
        self._btn_native_focus_plus.setToolTip("Focus farther")
        self._btn_native_gimbal_up = QPushButton("↑")
        self._btn_native_gimbal_down = QPushButton("↓")
        self._btn_native_gimbal_left = QPushButton("←")
        self._btn_native_gimbal_right = QPushButton("→")
        self._btn_native_gimbal_center = QPushButton("⌂")
        self._btn_native_gimbal_nadir = QPushButton("90°")
        self._btn_native_target = QPushButton("Target")
        self._btn_native_target.setCheckable(True)
        self._btn_native_clip = QPushButton("Clip")
        self._btn_native_report = QPushButton("Report")
        self._btn_native_report.setObjectName("observeReport")
        self._btn_native_reset = QPushButton("Reset")
        self._btn_native_reset.setObjectName("observeReset")
        for b in (
            self._btn_native_gimbal_up,
            self._btn_native_gimbal_down,
            self._btn_native_gimbal_left,
            self._btn_native_gimbal_right,
            self._btn_native_gimbal_center,
            self._btn_native_gimbal_nadir,
        ):
            _cam_pad_btn(b)

        self._btn_native_gimbal_left.setToolTip("Gimbal yaw left — press and hold")
        self._btn_native_gimbal_up.setToolTip("Gimbal pitch up — press and hold")
        self._btn_native_gimbal_right.setToolTip("Gimbal yaw right — press and hold")
        self._btn_native_gimbal_down.setToolTip("Gimbal pitch down — press and hold")
        self._btn_native_gimbal_center.setToolTip("Recenter gimbal (yaw and pitch to 0°)")
        self._btn_native_gimbal_nadir.setToolTip(
            "Point gimbal straight down (default −90° pitch; set camera/gimbal_nadir_pitch_deg to 90 if needed)"
        )

        gimbal_pad = CamRailGimbalPad(
            [
                [
                    self._btn_native_gimbal_left,
                    self._btn_native_gimbal_up,
                    self._btn_native_gimbal_right,
                ],
                [
                    self._btn_native_gimbal_center,
                    self._btn_native_gimbal_down,
                    self._btn_native_gimbal_nadir,
                ],
            ],
            btn_height=_CAM_RAIL_PAD_BTN_H,
            grid_gap=_CAM_RAIL_GIMBAL_GRID_GAP,
        )
        observe_body = CamObserveBlock(
            self._btn_native_target,
            self._btn_native_clip,
            self._btn_native_report,
            self._btn_native_reset,
        )

        self._native_hud_right_layout.addWidget(_cam_rail_sep())
        self._native_hud_right_layout.addWidget(
            _cam_rail_lens_row(
                self._btn_native_zoom_minus,
                self._btn_native_zoom_plus,
                self._btn_native_focus_minus,
                self._btn_native_focus_plus,
            )
        )
        self._native_hud_right_layout.addWidget(
            _cam_rail_inline_row("GIMBAL", gimbal_pad)
        )
        self._native_hud_right_layout.addWidget(
            _cam_rail_inline_row("OBSERVE", observe_body)
        )

        self._native_hud_right.setMinimumWidth(_NATIVE_CAM_RAIL_MIN_WIDTH_PX)
        self._native_hud_right.hide()
        self._native_rail_layer.hide()

        # Native tile map ignores `setTelemetryOverlay` / compass DOM (see native_tile_map._eval_one skips).
        # Mirror git e48c1a7 bottom HUD: painted compass + 3-cell telemetry strip (Web CSS parity).
        # Parent = `_panel` (not `_map_canvas`) so the HUD stacks above `#nativeCameraRailLayer`,
        # which is also a floating child of `_panel` and otherwise covered the bottom-right compass.
        self._native_compass = MapFooterCompass(self._panel)
        self._native_compass.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._native_compass.hide()
        self._native_compass.raise_()

        self._native_telemetry = MapFooterTelemetryStrip(self._panel)
        self._native_telemetry.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._native_telemetry.hide()
        self._native_telemetry.raise_()

        # M9 — obstacle radar on `_panel` (same stacking layer as PiP / compass).
        self._obstacle_radar = ObstacleRadarPanel(self._panel)
        self._obstacle_radar.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._obstacle_radar.hide()

        # Legacy Web `#actionRail` / `.actionBtn` — parent `_panel` (not `_map_canvas`) so Takeoff/Return
        # stay above fullscreen RTSP video; raising the video label every frame no longer covers them.
        self._map_action_rail = QFrame(self._panel)
        self._map_action_rail.setObjectName("mapActionRail")
        self._map_action_rail.setStyleSheet(
            "QFrame#mapActionRail { background: transparent; border: none; }"
        )
        ar_l = QVBoxLayout(self._map_action_rail)
        ar_l.setContentsMargins(0, 0, 0, 0)
        ar_l.setSpacing(8)
        _map_action_btn_base_ss = (
            "QPushButton#mapActionTakeoffBtn, QPushButton#mapActionReturnBtn {"
            "min-width:54px; max-width:54px; min-height:54px; max-height:54px;"
            "border:1px solid rgba(255,255,255,0.35);"
            "background:rgba(34,42,56,0.92);"
            "color:#c8d3ea;"
            "font:600 11px \"Segoe UI\", Arial, sans-serif;"
            "padding:0px;"
            "outline:none;"
            "}"
            "QPushButton#mapActionTakeoffBtn:hover:enabled, QPushButton#mapActionReturnBtn:hover:enabled {"
            "background:rgba(46,58,78,0.95);"
            "border-color:rgba(136,164,205,0.7);"
            "}"
            "QPushButton#mapActionTakeoffBtn:disabled, QPushButton#mapActionReturnBtn:disabled { opacity:0.45; }"
        )
        _takeoff_ss = (
            _map_action_btn_base_ss
            + "QPushButton#mapActionTakeoffBtn {"
            "border-top-left-radius:8px; border-bottom-left-radius:0px;"
            "border-top-right-radius:0px; border-bottom-right-radius:0px;"
            "}"
        )
        _return_ss = (
            _map_action_btn_base_ss
            + "QPushButton#mapActionReturnBtn {"
            "border-top-left-radius:0px; border-bottom-left-radius:8px;"
            "border-top-right-radius:0px; border-bottom-right-radius:0px;"
            "}"
        )
        self._map_action_takeoff_btn = QPushButton(self._map_action_rail)
        self._map_action_takeoff_btn.setObjectName("mapActionTakeoffBtn")
        self._map_action_takeoff_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._map_action_takeoff_btn.setFlat(True)
        self._map_action_takeoff_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._map_action_takeoff_btn.setToolTip(
            "NAV_TAKEOFF at main Takeoff alt (m). Connect vehicle first (same as dashboard Takeoff)."
        )
        _to_lay = QVBoxLayout()
        _to_lay.setContentsMargins(3, 5, 3, 5)
        _to_lay.setSpacing(1)
        _to_ic = TelemetryStripIcon("up", self._map_action_takeoff_btn, icon_size=_MAP_ACTION_ICON_LOGICAL_PX)
        _to_lbl = QLabel("Takeoff", self._map_action_takeoff_btn)
        _to_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop)
        _to_lbl.setStyleSheet(
            "color:#c8d3ea; font-weight:600; font-size:11px; background:transparent; border:none;"
        )
        _to_lbl.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        _to_lay.addWidget(_to_ic, 0, Qt.AlignmentFlag.AlignHCenter)
        _to_lay.addWidget(_to_lbl, 0, Qt.AlignmentFlag.AlignHCenter)
        self._map_action_takeoff_btn.setLayout(_to_lay)
        self._map_action_takeoff_btn.setStyleSheet(_takeoff_ss)

        self._map_action_return_btn = QPushButton(self._map_action_rail)
        self._map_action_return_btn.setObjectName("mapActionReturnBtn")
        self._map_action_return_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._map_action_return_btn.setFlat(True)
        self._map_action_return_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._map_action_return_btn.setToolTip("Return to launch / RTL (same as dashboard Return).")
        _re_lay = QVBoxLayout()
        _re_lay.setContentsMargins(3, 5, 3, 5)
        _re_lay.setSpacing(1)
        _re_ic = TelemetryStripIcon("return_home", self._map_action_return_btn, icon_size=_MAP_ACTION_ICON_LOGICAL_PX)
        _re_lbl = QLabel("Return", self._map_action_return_btn)
        _re_lbl.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop)
        _re_lbl.setStyleSheet(
            "color:#c8d3ea; font-weight:600; font-size:11px; background:transparent; border:none;"
        )
        _re_lbl.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        _re_lay.addWidget(_re_ic, 0, Qt.AlignmentFlag.AlignHCenter)
        _re_lay.addWidget(_re_lbl, 0, Qt.AlignmentFlag.AlignHCenter)
        self._map_action_return_btn.setLayout(_re_lay)
        self._map_action_return_btn.setStyleSheet(_return_ss)
        ar_l.addWidget(self._map_action_takeoff_btn)
        ar_l.addWidget(self._map_action_return_btn)
        self._map_action_takeoff_btn.setEnabled(False)
        self._map_action_return_btn.setEnabled(False)
        self._map_action_takeoff_btn.clicked.connect(lambda: self.takeoff_requested.emit())
        self._map_action_return_btn.clicked.connect(lambda: self.return_requested.emit())
        self._map_action_rail.setFixedSize(54, 54 + 8 + 54)
        self._map_action_rail.show()
        self._map_action_rail.raise_()

        self._plan_flight_panel = PlanFlightPanel(panel)
        self._plan_flight_panel.hide()
        self._plan_flight_panel.exit_requested.connect(self._on_plan_panel_exit)
        self._plan_flight_panel.action_requested.connect(self.plan_action_requested.emit)
        self._plan_flight_panel.tool_requested.connect(self._on_plan_panel_tool)
        self._plan_flight_panel.mission_panel_changed.connect(self._on_plan_panel_mission_changed)
        self._plan_flight_panel.mission_start_requested.connect(self.mission_start_requested.emit)
        self._plan_flight_panel.return_requested.connect(self.return_requested.emit)
        self._plan_flight_panel.set_launch_to_map_center_requested.connect(
            self._on_plan_panel_set_launch_to_map_center
        )
        self.waypoints_changed.connect(self._on_plan_panel_waypoints_changed)

        # Wrapper + inner image: pixmap must not live on the same QLabel as +/- children — Qt paints
        # the pixmap over child widgets, so clicks never reached the zoom buttons.
        self._native_minimap_wrap = QFrame(self._panel)
        self._native_minimap_wrap.setObjectName("nativeMinimapWrap")
        self._native_minimap_wrap.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._native_minimap_wrap.setAutoFillBackground(False)
        self._native_minimap_wrap.setStyleSheet(
            "QFrame#nativeMinimapWrap { background: transparent; border: 1px solid rgba(132,152,190,0.45); border-radius: 8px; }"
        )
        self._native_minimap_wrap.hide()

        self._native_minimap = QLabel(self._native_minimap_wrap)
        self._native_minimap.setObjectName("nativeMinimapImage")
        self._native_minimap.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._native_minimap.setStyleSheet("QLabel#nativeMinimapImage { background: transparent; border: none; }")
        self._native_minimap.setCursor(Qt.CursorShape.OpenHandCursor)
        self._native_minimap.setToolTip("Drag to pan map · click (no drag) to swap back to map")
        # QLabel does not get move events without mouse tracking unless a button is held; tracking is
        # cheap here and lets us update the cursor + still works fine when the user drags.
        self._native_minimap.setMouseTracking(True)
        self._native_minimap.mousePressEvent = self._on_native_minimap_image_press  # type: ignore[assignment]
        self._native_minimap.mouseMoveEvent = self._on_native_minimap_image_move  # type: ignore[assignment]
        self._native_minimap.mouseReleaseEvent = self._on_native_minimap_image_release  # type: ignore[assignment]
        # Wheel pans zoom on the live map — feels natural since the card *is* the map.
        self._native_minimap.wheelEvent = self._on_native_minimap_image_wheel  # type: ignore[assignment]
        self._native_minimap.hide()
        self._minimap_img_dragging = False
        self._minimap_img_drag_last: QPointF | None = None
        self._minimap_img_press: QPointF | None = None
        self._minimap_grab_refresh_timer = QTimer(self)
        self._minimap_grab_refresh_timer.setSingleShot(True)
        self._minimap_grab_refresh_timer.setInterval(75)
        self._minimap_grab_refresh_timer.timeout.connect(self._update_native_minimap)

        self._native_minimap_zoom = 16
        self._native_minimap_tile_key: tuple[int, int, int] | None = None
        self._native_minimap_tile_img = QImage()
        # Larger, high-contrast controls so +/− stay readable over satellite tiles when swap shows the map PiP.
        self._native_minimap_btn_side = 32
        self._native_minimap_btn_pad = 8
        self._btn_native_minimap_plus = QPushButton("+", self._native_minimap_wrap)
        self._btn_native_minimap_minus = QPushButton("-", self._native_minimap_wrap)
        _mini_f = QFont()
        _mini_f.setPointSize(16)
        _mini_f.setWeight(QFont.Weight.Black)
        _mini_ss = (
            "QPushButton {"
            "background-color: rgba(18, 26, 40, 0.88);"
            "color: #f5f8ff;"
            "border: 2px solid rgba(200, 218, 255, 0.95);"
            "border-radius: 6px;"
            "padding: 0px;"
            "}"
            "QPushButton:hover { background-color: rgba(32, 46, 68, 0.92); border-color: #ffffff; color: #ffffff; }"
            "QPushButton:pressed { background-color: rgba(12, 18, 28, 0.95); border-color: #9eb6e8; }"
        )
        for b in (self._btn_native_minimap_plus, self._btn_native_minimap_minus):
            b.setFont(_mini_f)
            b.setFixedSize(self._native_minimap_btn_side, self._native_minimap_btn_side)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.setAutoDefault(False)
            b.setDefault(False)
            b.setStyleSheet(_mini_ss)
            b.hide()
            b.raise_()
        self._btn_native_minimap_plus.clicked.connect(self._on_native_minimap_plus_clicked)
        self._btn_native_minimap_minus.clicked.connect(self._on_native_minimap_minus_clicked)

        self._cam_rail_mode_group = QButtonGroup(self)
        self._cam_rail_mode_group.setExclusive(True)
        self._cam_rail_mode_group.addButton(self._btn_native_video, 0)
        self._cam_rail_mode_group.addButton(self._btn_native_photo, 1)
        self._btn_native_video.setChecked(True)
        self._btn_native_photo.setChecked(False)
        self._cam_rail_mode_group.idClicked.connect(self._on_camera_rail_mode_id_clicked)

        # Debounced commit: some platforms deliver several `toggled` edges per physical click while
        # the rail relayouts / video moves; reading `isChecked()` once after a short quiet window fixes 1/0/1/0 churn.
        self._split_rail_debounce = QTimer(self)
        self._split_rail_debounce.setSingleShot(True)
        self._split_rail_debounce.setInterval(50)
        self._split_rail_debounce.timeout.connect(self._commit_native_split_rail_toggle)
        self._follow_rail_debounce = QTimer(self)
        self._gimbal_hold_timer = QTimer(self)
        self._gimbal_hold_timer.setInterval(80)
        self._gimbal_hold_timer.timeout.connect(self._on_gimbal_hold_tick)
        self._gimbal_hold_axis: tuple[int, int] | None = None
        self._follow_rail_debounce.setSingleShot(True)
        self._follow_rail_debounce.setInterval(50)
        self._follow_rail_debounce.timeout.connect(self._commit_native_follow_rail_toggle)
        self._btn_native_split.toggled.connect(self._on_native_split_rail_toggled)
        self._btn_native_follow.toggled.connect(self._on_native_follow_rail_toggled)
        self._btn_native_record.clicked.connect(self._on_native_record_center_clicked)
        self._btn_native_record.toggled.connect(self._on_native_record_toggled)
        self._sync_native_record_button_for_rail_mode()
        self._btn_native_zoom_minus.clicked.connect(lambda: self._on_web_title_changed("VGCS_CAM_ZOOM_STEP:-1:0"))
        self._btn_native_zoom_plus.clicked.connect(lambda: self._on_web_title_changed("VGCS_CAM_ZOOM_STEP:1:0"))
        self._btn_native_focus_minus.clicked.connect(lambda: self._on_web_title_changed("VGCS_CAM_FOCUS_STEP:-1:0"))
        self._btn_native_focus_plus.clicked.connect(lambda: self._on_web_title_changed("VGCS_CAM_FOCUS_STEP:1:0"))
        # Hold = continuous GSY/GSP speed (smooth); release = GSM stop. No PTZ steps on pitch.
        self._wire_native_gimbal_hold_button(self._btn_native_gimbal_up, 0, 1)
        self._wire_native_gimbal_hold_button(self._btn_native_gimbal_down, 0, -1)
        self._wire_native_gimbal_hold_button(self._btn_native_gimbal_left, -1, 0)
        self._wire_native_gimbal_hold_button(self._btn_native_gimbal_right, 1, 0)
        self._btn_native_gimbal_center.clicked.connect(self._native_gimbal_center)
        self._btn_native_gimbal_nadir.clicked.connect(self._native_gimbal_point_down)
        def _obs_target(on: bool) -> None:
            print(f"[VGCS:cam_rail] OBSERVE Target toggled={bool(on)}")
            self._set_observation_mark_mode(on)

        def _obs_clip() -> None:
            print("[VGCS:cam_rail] OBSERVE Clip clicked")
            self._obs_clip_ui_preparing()
            QTimer.singleShot(0, self._capture_observation_clip)

        def _obs_report() -> None:
            print("[VGCS:cam_rail] OBSERVE Report clicked")
            QTimer.singleShot(0, lambda: self._export_observations(quick=True))

        def _obs_reset() -> None:
            print("[VGCS:cam_rail] OBSERVE Reset clicked")
            self._clear_observations()

        self._btn_native_target.toggled.connect(_obs_target)
        self._btn_native_clip.clicked.connect(_obs_clip)
        self._btn_native_report.clicked.connect(_obs_report)
        self._btn_native_reset.clicked.connect(_obs_reset)

        self._status = QLabel("Map status: waiting for telemetry")
        self._status.setObjectName("telemetryValue")
        self._coords = QLabel("Lat/Lon: —")
        self._coords.setObjectName("telemetryValue")
        self._heading_label = QLabel("Heading: —")
        self._heading_label.setObjectName("telemetryValue")
        self._mission = QLabel("Mission WPs: 0")
        self._mission.setObjectName("telemetryValue")

        toolbar = QFrame()
        toolbar.setObjectName("statusChip")
        tools = QGridLayout()
        tools.setHorizontalSpacing(6)
        tools.setVerticalSpacing(6)
        self._btn_add_wp = QPushButton("Add WP")
        self._btn_clear_wp = QPushButton("Clear WPs")
        self._btn_upload = QPushButton("Upload Mission")
        self._btn_download = QPushButton("Download Mission")
        self._btn_export = QPushButton("Export to file…")
        self._btn_export.setToolTip(
            "Write a copy of waypoints to a JSON file you choose. "
            "Does not change the Plan Flight “current mission” path."
        )
        self._btn_import = QPushButton("Import Mission")
        self._btn_3d = QPushButton("3D Toggle")
        self._btn_3d.setCheckable(True)
        self._btn_fence_poly = QPushButton("Fence Polygon")
        self._btn_tiles_esri = QPushButton("Online Tiles (Esri Streets)")
        self._btn_tiles_osm = QPushButton("Online Tiles (OSM)")
        self._btn_tiles_sat = QPushButton("Satellite (Esri)")
        self._btn_tiles_pick = QPushButton("Offline Tiles…")
        self._btn_webcam = QPushButton("Webcam")
        self._btn_webcam.setCheckable(True)
        self._btn_obs_mode = QPushButton("Mark Target")
        self._btn_obs_mode.setCheckable(True)
        self._btn_obs_clip = QPushButton("Short Clip")
        self._btn_obs_export = QPushButton("Export Obs")
        self._btn_obs_clear = QPushButton("Clear Obs")
        self._perf_mode = QComboBox()
        self._perf_mode.setMinimumWidth(92)
        self._perf_mode.addItems(["Perf: Auto", "Perf: Low", "Perf: High"])
        try:
            s = QSettings(_QS_NS, _QS_APP)
            mode = str(s.value(_KEY_MAP_LOW_SPEC_MODE, "auto") or "auto").strip().lower()
        except Exception:
            mode = "auto"
        if mode == "on":
            self._perf_mode.setCurrentIndex(1)
        elif mode == "off":
            self._perf_mode.setCurrentIndex(2)
        else:
            self._perf_mode.setCurrentIndex(0)
        try:
            s = QSettings(_QS_NS, _QS_APP)
            self._btn_webcam.setChecked(bool(s.value(_KEY_MAP_WEBCAM_ENABLED, False)))
        except Exception:
            self._btn_webcam.setChecked(False)
        self._fence_radius = QDoubleSpinBox()
        self._fence_radius.setRange(10.0, 5000.0)
        self._fence_radius.setDecimals(0)
        self._fence_radius.setSingleStep(10.0)
        self._fence_radius.setValue(80.0)
        self._fence_alt_max = QDoubleSpinBox()
        self._fence_alt_max.setRange(5.0, 2000.0)
        self._fence_alt_max.setDecimals(0)
        self._fence_alt_max.setSingleStep(5.0)
        self._fence_alt_max.setValue(120.0)
        self._fence_action = QComboBox()
        self._fence_action.setMinimumWidth(110)
        self._fence_action.addItem("RTL (default)", 1.0)
        self._fence_action.addItem("Land", 2.0)
        self._fence_action.addItem("None", 0.0)
        self._btn_fence_apply = QPushButton("Apply Fence")
        self._btn_fence_clear = QPushButton("Clear Fence")
        self._default_alt = QDoubleSpinBox()
        self._default_alt.setRange(1.0, 500.0)
        self._default_alt.setDecimals(1)
        self._default_alt.setSingleStep(1.0)
        self._default_alt.setValue(20.0)
        self._default_speed = QDoubleSpinBox()
        self._default_speed.setRange(0.1, 50.0)
        self._default_speed.setDecimals(1)
        self._default_speed.setSingleStep(0.5)
        self._default_speed.setValue(5.0)
        self._wp_selector = QComboBox()
        self._wp_selector.setMinimumWidth(90)
        self._wp_alt = QDoubleSpinBox()
        self._wp_alt.setRange(1.0, 500.0)
        self._wp_alt.setDecimals(1)
        self._wp_alt.setSingleStep(1.0)
        self._wp_alt.setValue(20.0)
        self._wp_speed = QDoubleSpinBox()
        self._wp_speed.setRange(0.1, 50.0)
        self._wp_speed.setDecimals(1)
        self._wp_speed.setSingleStep(0.5)
        self._wp_speed.setValue(5.0)
        self._btn_apply_wp_alt = QPushButton("Set WP Alt")
        self._btn_apply_all_alt = QPushButton("Set All Alt")
        self._btn_apply_wp_speed = QPushButton("Set WP Speed")
        self._btn_apply_all_speed = QPushButton("Set All Speed")
        tools.addWidget(self._btn_add_wp, 0, 0)
        tools.addWidget(self._btn_clear_wp, 0, 1)
        tools.addWidget(self._btn_upload, 0, 2)
        tools.addWidget(self._btn_download, 0, 3)
        tools.addWidget(self._btn_export, 0, 4)
        tools.addWidget(self._btn_import, 0, 5)
        tools.addWidget(self._btn_3d, 0, 6)
        tools.addWidget(self._btn_fence_poly, 0, 7)
        tools.addWidget(QLabel("Fence R (m)"), 0, 8)
        tools.addWidget(self._fence_radius, 0, 9)
        tools.addWidget(QLabel("Fence Alt"), 0, 10)
        tools.addWidget(self._fence_alt_max, 0, 11)
        tools.addWidget(QLabel("Fence action"), 0, 12)
        tools.addWidget(self._fence_action, 0, 13)
        tools.addWidget(self._btn_fence_apply, 0, 14)
        tools.addWidget(self._btn_fence_clear, 0, 15)
        tools.addWidget(QLabel("Default Alt (m)"), 1, 0)
        tools.addWidget(self._default_alt, 1, 1)
        tools.addWidget(QLabel("Default Spd (m/s)"), 1, 2)
        tools.addWidget(self._default_speed, 1, 3)
        tools.addWidget(QLabel("WP"), 1, 4)
        tools.addWidget(self._wp_selector, 1, 5)
        tools.addWidget(QLabel("Alt (m)"), 1, 6)
        tools.addWidget(self._wp_alt, 1, 7)
        tools.addWidget(self._btn_apply_wp_alt, 1, 8)
        tools.addWidget(self._btn_apply_all_alt, 1, 9)

        tools.addWidget(QLabel("Spd (m/s)"), 2, 0)
        tools.addWidget(self._wp_speed, 2, 1)
        tools.addWidget(self._btn_apply_wp_speed, 2, 2)
        tools.addWidget(self._btn_apply_all_speed, 2, 3)
        tools.addWidget(self._btn_tiles_esri, 2, 8)
        tools.addWidget(self._btn_tiles_osm, 2, 9)
        tools.addWidget(self._btn_tiles_sat, 2, 10)
        tools.addWidget(self._btn_tiles_pick, 2, 11)
        tools.addWidget(self._btn_webcam, 2, 12)
        tools.addWidget(self._perf_mode, 2, 13)
        tools.addWidget(self._btn_obs_mode, 2, 14)
        tools.addWidget(self._btn_obs_clip, 2, 15)
        tools.addWidget(self._btn_obs_export, 2, 16)
        tools.addWidget(self._btn_obs_clear, 2, 17)
        toolbar.setLayout(tools)
        self._toolbar = toolbar

        status_box = QFrame()
        status_box.setObjectName("statusChip")
        status_layout = QGridLayout()
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(6)
        status_layout.addWidget(self._status, 0, 0, 1, 2)
        status_layout.addWidget(self._coords, 1, 0)
        status_layout.addWidget(self._heading_label, 1, 1)
        status_layout.addWidget(self._mission, 2, 0, 1, 2)
        status_box.setLayout(status_layout)
        self._status_box = status_box

        panel_layout.addWidget(self._map_canvas, 1)
        panel_layout.addWidget(toolbar)
        panel_layout.addWidget(status_box)
        panel.setLayout(panel_layout)
        root.addWidget(panel, 1)
        self.setLayout(root)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._map_canvas.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )

        self._btn_add_wp.clicked.connect(self._enable_add_waypoint_mode)
        self._btn_clear_wp.clicked.connect(self._clear_waypoints)
        self._btn_upload.clicked.connect(self._request_upload)
        self._btn_download.clicked.connect(self._request_download)
        self._btn_export.clicked.connect(self._export_mission)
        self._btn_import.clicked.connect(self._import_mission)
        self._btn_3d.clicked.connect(self._toggle_3d_mode)
        self._btn_fence_poly.clicked.connect(self._enable_fence_polygon_mode)
        self._btn_fence_apply.clicked.connect(self._apply_geofence)
        self._btn_fence_clear.clicked.connect(self._clear_geofence)
        self._btn_tiles_esri.clicked.connect(self._set_esri_street_tiles)
        self._btn_tiles_osm.clicked.connect(self._set_osm_tiles)
        self._btn_tiles_sat.clicked.connect(self._set_satellite_tiles)
        self._btn_tiles_pick.clicked.connect(self._pick_offline_tiles)
        self._btn_webcam.toggled.connect(self._set_webcam_enabled)
        self._btn_obs_mode.toggled.connect(self._set_observation_mark_mode)
        self._btn_obs_clip.clicked.connect(self._capture_observation_clip)
        self._btn_obs_export.clicked.connect(self._export_observations)
        self._btn_obs_clear.clicked.connect(self._clear_observations)
        self._perf_mode.currentIndexChanged.connect(self._on_perf_mode_changed)
        self._wp_selector.currentIndexChanged.connect(self._on_wp_selected)
        self._btn_apply_wp_alt.clicked.connect(self._apply_altitude_to_selected)
        self._btn_apply_all_alt.clicked.connect(self._apply_altitude_to_all)
        self._btn_apply_wp_speed.clicked.connect(self._apply_speed_to_selected)
        self._btn_apply_all_speed.clicked.connect(self._apply_speed_to_all)

        self._wp_poll = QTimer(self)
        self._wp_poll.setInterval(1000)
        self._wp_poll.timeout.connect(self._sync_waypoint_count_from_map)
        self._wp_poll.start()

        self._init_map_backend()

    def set_dashboard_mode(self, enabled: bool) -> None:
        """Hide map-edit controls for clean dashboard layout."""
        self._toolbar.setVisible(not enabled)
        self._status_box.setVisible(not enabled)
        if enabled:
            self._panel.setTitle("")
            self._panel.setFlat(True)
            self._panel.setStyleSheet("QGroupBox { border: 0; margin-top: 0; padding: 0; }")
            self._map_canvas.setObjectName("")
            self._map_canvas.setStyleSheet("QFrame { border: 0; margin: 0; padding: 0; }")
            self._panel_layout.setContentsMargins(0, 0, 0, 0)
            self._panel_layout.setSpacing(0)
        else:
            self._panel.setTitle("3D Map")
            self._panel.setFlat(False)
            self._panel.setStyleSheet("")
            self._map_canvas.setObjectName("statusChip")
            self._map_canvas.setStyleSheet("")
            self._panel_layout.setContentsMargins(0, 0, 0, 0)
            self._panel_layout.setSpacing(8)

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(event)
        if bool(getattr(self, "_video_swapped", False)):
            self._refresh_native_overlay_insets()
        # Always relayout native HUD (camera rail, compass, telemetry) on resize — independent of video PiP.
        self._layout_native_video_preview()
        try:
            self._layout_native_hud()
        except Exception:
            pass
        try:
            self._layout_plan_flight_panel()
        except Exception:
            pass
        if not getattr(self, "_is_3d_mode", False):
            try:
                QTimer.singleShot(0, self._ensure_native_map_visible)
            except Exception:
                pass

    def showEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().showEvent(event)
        # First frame / platform quirks: layout can restore map above HUD siblings; fix Z-order after show.
        try:
            if bool(getattr(self, "_last_link_connected", False)):
                QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def _on_native_video_click(self, event) -> None:
        try:
            if event is not None and hasattr(event, "button"):
                btn = event.button()
                if btn != Qt.MouseButton.LeftButton:
                    return
        except Exception:
            pass
        if bool(getattr(self, "_obs_mark_mode", False)):
            try:
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
            self._refresh_native_overlay_insets()
        else:
            self._video_swap_user_map_main = True
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
        if bool(getattr(self, "_obs_mark_mode", False)):
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

    def _native_map_tile_count(self) -> int:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return 0
        try:
            return int(nm.loaded_tile_count())
        except Exception:
            return 0

    def _promote_native_map_if_ready(self) -> bool:
        """Use Qt tiles for the main map when they are available (swap PiP can still grab native)."""
        if getattr(self, "_is_3d_mode", False):
            return False
        if self._native_map_tile_count() < 4:
            return False
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        if getattr(self, "_web_2d_fallback_active", False):
            self._web_2d_fallback_active = False
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.show()
                nm.update()
                nm.repaint()
            except Exception:
                pass
        return True

    def _show_map_main_surface(self) -> None:
        """After swap-to-map: native Qt tiles only (never WebEngine for 2D)."""
        self._ensure_native_map_visible()
        self._promote_native_map_if_ready()
        try:
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def _refresh_native_overlay_insets(self) -> None:
        if not bool(getattr(self, "_web_ready", False)):
            return
        self._native_overlay_insets = {
            "left": 170,
            "top": _NATIVE_CAM_RAIL_TOP_PX,
            "right": 192,
            "bottom": 130,
        }
        self._layout_native_video_preview()

    def _mini_video_pip_rect(self, w: int, h: int) -> tuple[int, int, int, int]:
        """Bottom-left PiP size in `_map_canvas` coordinates."""
        pw = min(_MINI_VIDEO_PIP_W_PX, max(200, int(w * 0.22)))
        ph = min(_MINI_VIDEO_PIP_H_PX, max(112, int(h * 0.17)))
        x = _MAP_HUD_MARGIN_PX
        y = max(0, h - ph - _MAP_HUD_MARGIN_PX)
        return x, y, pw, ph

    def _map_canvas_rect_on_panel(self, x: int, y: int, w: int, h: int) -> tuple[int, int, int, int]:
        """Map a `_map_canvas` child rect to `_panel` coordinates."""
        host = getattr(self, "_map_canvas", None)
        panel = getattr(self, "_panel", None)
        if host is None or panel is None:
            return x, y, w, h
        try:
            pt = host.mapTo(panel, QPoint(int(x), int(y)))
            return int(pt.x()), int(pt.y()), int(w), int(h)
        except Exception:
            return x, y, w, h

    def _video_stream_configured(self) -> bool:
        src = str(getattr(self, "_video_settings_source", "rtsp") or "rtsp").strip().lower()
        if src == "disabled":
            return False
        day = str(getattr(self, "_video_settings_day", "") or "").strip()
        thermal = str(getattr(self, "_video_settings_thermal", "") or "").strip()
        return bool(day or thermal) or src in ("udp_h264", "udp_h265")

    def _show_mini_video_pip_shell(self) -> None:
        """Always paint the bottom-left PiP frame (even before FFmpeg / pipeline sources exist)."""
        if not bool(getattr(self, "_web_ready", False)):
            return
        if self._plan_flight_layer_obscures_native_camera_ui():
            return
        try:
            self._read_video_settings()
        except Exception:
            pass
        self._video_preview_enabled = True
        if not bool(getattr(self, "_video_swap_user_map_main", False)):
            self._video_swapped = False
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
        except Exception:
            pass
        if bool(getattr(self, "_last_link_connected", False)):
            try:
                self._native_hud_right.show()
                self._native_rail_layer.show()
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
            obr = getattr(self, "_obstacle_radar", None)
            if obr is not None and obr.isVisible():
                obr.raise_()
            for hud in (self._native_compass, self._native_telemetry):
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
        except Exception:
            pass

    def _raise_panel_flight_overlays(self) -> None:
        """Panel-level HUD above fullscreen video (legacy alias)."""
        self._raise_flight_hud_above_video()

    def _stack_native_overlays_above_tile_map(self) -> None:
        """
        Lower the tile map, raise video, then raise all flight HUD above the video layer.
        """
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.lower()
            except Exception:
                pass
        try:
            pv = getattr(self, "_native_video_preview", None)
            if pv is not None and pv.isVisible():
                pv.raise_()
                try:
                    self._sync_native_video_overlay()
                except Exception:
                    pass
        except Exception:
            pass
        self._raise_flight_hud_above_video()

    def _sync_native_map_vehicle_arrow_scale(self) -> None:
        """Larger vehicle chevron while map is shown in the swap PiP (grab scales the view down)."""
        nm = getattr(self, "_native_map", None)
        if nm is None or not hasattr(nm, "set_vehicle_arrow_scale"):
            return
        swapped = bool(getattr(self, "_video_swapped", False))
        preview_on = bool(getattr(self, "_video_preview_enabled", False))
        plan_on = self._plan_flight_layer_obscures_native_camera_ui()
        boost = 1.85 if (preview_on and swapped and not plan_on) else 1.0
        try:
            nm.set_vehicle_arrow_scale(boost)
        except Exception:
            pass

    def _layout_native_hud(self) -> None:
        try:
            plan_on = self._plan_flight_layer_obscures_native_camera_ui()
            if plan_on:
                try:
                    self._native_rail_layer.hide()
                    self._native_hud_right.hide()
                    self._native_video_preview.hide()
                    self._native_minimap_wrap.hide()
                    self._btn_native_minimap_plus.hide()
                    self._btn_native_minimap_minus.hide()
                    self._obstacle_radar.hide()
                except Exception:
                    pass
            else:
                try:
                    if bool(getattr(self, "_last_link_connected", False)) and bool(
                        getattr(self, "_web_ready", False)
                    ):
                        self._native_hud_right.show()
                        self._native_rail_layer.show()
                    else:
                        self._native_hud_right.hide()
                        self._native_rail_layer.hide()
                except Exception:
                    pass
            w = max(1, self._map_canvas.width())
            h = max(1, self._map_canvas.height())
            rail = self._native_hud_right
            ly = getattr(self, "_native_rail_layer", None)
            panel_y = int(_NATIVE_CAM_RAIL_TOP_PX)
            bottom_margin = 12
            available_h = max(120, h - panel_y - bottom_margin)
            rail.setMinimumWidth(_NATIVE_CAM_RAIL_MIN_WIDTH_PX)
            rl = rail.layout()
            if rl is not None:
                rl.activate()
            rail.updateGeometry()
            panel_w = max(
                _NATIVE_CAM_RAIL_MIN_WIDTH_PX,
                int(rail.sizeHint().width()),
                int(rail.minimumSizeHint().width()),
            )
            panel_x = max(0, w - panel_w - 18)  # git `#cameraRail { right: 18px }`
            rail.setFixedWidth(panel_w)
            need_h = max(120, int(rail.sizeHint().height()))
            # Never shrink below content height (that clips LENS / gimbal buttons). Shift up instead.
            panel_h = need_h
            if panel_y + panel_h > h - bottom_margin:
                panel_y = max(0, h - bottom_margin - panel_h)
            if ly is not None:
                pt = self._map_canvas.mapTo(self._panel, QPoint(panel_x, panel_y))
                ly.setGeometry(pt.x(), pt.y(), panel_w, panel_h)
            rail.setGeometry(0, 0, panel_w, panel_h)
            # Git `#mapFooterHud { right: 10px; bottom: 2px }` — compass stays bottom-right of the
            # map even when `#cameraRail` is shown after MAVLink connect (rail is top-right; z-order
            # keeps HUD above the map tiles, not shoved left).
            comp_w, comp_h = 176, 176
            margin_r, margin_b = 10, 2
            cx = max(0, w - margin_r - comp_w)
            cy = max(0, h - margin_b - comp_h)
            po = self._map_canvas.mapTo(self._panel, QPoint(0, 0))
            self._native_compass.setGeometry(po.x() + cx, po.y() + cy, comp_w, comp_h)
            self._native_telemetry.updateGeometry()
            mw = self._native_telemetry.minimumSizeHint().width()
            mh = self._native_telemetry.minimumSizeHint().height()
            sw = self._native_telemetry.sizeHint().width()
            sh = self._native_telemetry.sizeHint().height()
            # Avoid clipping m/s / m / time: never cap width to a small constant; pad for border + font metrics.
            tel_w = max(mw, sw) + 20
            tel_h = max(40, max(mh, sh) + 4)
            tel_x = cx - 12 - tel_w
            if tel_x < 8:
                tel_x = 8
            tel_y = cy + (comp_h - tel_h) // 2
            self._native_telemetry.setGeometry(
                po.x() + tel_x, po.y() + tel_y, tel_w, tel_h
            )
            mar = getattr(self, "_map_action_rail", None)
            if mar is not None:
                mar_w = max(54, mar.width())
                mar_h = max(116, mar.height())
                mar.setGeometry(
                    po.x() + _MAP_ACTION_RAIL_LEFT_PX,
                    po.y() + _MAP_ACTION_RAIL_TOP_PX,
                    mar_w,
                    mar_h,
                )
            swapped = bool(getattr(self, "_video_swapped", False))
            preview_on = bool(getattr(self, "_video_preview_enabled", False))
            preview_maps = preview_on and not plan_on
            _pip_x, _pip_y, _pip_w, pip_h = self._mini_video_pip_rect(w, h)
            margin = _MAP_HUD_MARGIN_PX
            # M9 obstacle radar — compact card top-left; mini-video is a small fixed PiP bottom-left.
            obr = getattr(self, "_obstacle_radar", None)
            if obr is not None and bool(getattr(self, "_last_link_connected", False)):
                obr_w = int(obr.sizeHint().width())
                obr_h = min(_OBSTACLE_PANEL_MAX_H_PX, int(obr.sizeHint().height()))
                obr_x = margin
                obr_y = _OBSTACLE_PANEL_TOP_PX
                # Keep the card above the bottom-left video PiP when both are visible.
                if preview_maps and not swapped:
                    _px, pip_y, _pw, pip_h = self._mini_video_pip_rect(w, h)
                    pip_top = int(po.y() + pip_y)
                    canvas_bottom = int(po.y() + h)
                    max_h = max(150, pip_top - int(po.y() + obr_y) - 10)
                    obr_h = min(obr_h, max_h)
                gx, gy, gw, gh = self._map_canvas_rect_on_panel(obr_x, obr_y, obr_w, obr_h)
                obr.setGeometry(gx, gy, gw, obr_h)
                if not plan_on:
                    obr.show()
            elif obr is not None:
                obr.hide()
            # PiP mode: show video only (no second minimap card — main map is the overview).
            # Fullscreen camera swap: minimap takes the **same PiP slot as the video** (bottom-left)
            # so the swap is symmetric — clicking the corner card swaps back.
            # When preview is off, keep minimap hidden (do not resurrect on resize after _stop_video_preview).
            if not preview_maps or (preview_maps and not swapped):
                self._native_minimap_wrap.hide()
            else:
                mini_x, mini_y, mini_w, mini_h = self._mini_video_pip_rect(w, h)
                mini_x = int(po.x() + mini_x)
                mini_y = int(po.y() + mini_y)
                _side = int(getattr(self, "_native_minimap_btn_side", 32))
                _pad = int(getattr(self, "_native_minimap_btn_pad", 8))
                self._native_minimap_wrap.setGeometry(mini_x, mini_y, mini_w, mini_h)
                try:
                    nmz = getattr(self, "_native_map", None)
                    if nmz is not None:
                        zmx = int(getattr(nmz, "_max_zoom", 19))
                        self._native_minimap_zoom = int(
                            max(3, min(zmx, round(float(getattr(nmz, "_zoom", 16.0))))))
                except Exception:
                    self._native_minimap_zoom = 16
                # Map fills the entire card (video-style); +/- float on top — no left gutter / strip.
                self._native_minimap.setGeometry(0, 0, mini_w, mini_h)
                self._btn_native_minimap_plus.move(_pad, _pad)
                self._btn_native_minimap_minus.move(_pad, _pad + _side + 4)
                try:
                    self._native_minimap.lower()
                except Exception:
                    pass
                self._btn_native_minimap_plus.raise_()
                self._btn_native_minimap_minus.raise_()
                self._native_minimap_wrap.show()
                self._native_minimap.show()
                self._btn_native_minimap_plus.show()
                self._btn_native_minimap_minus.show()
                try:
                    QTimer.singleShot(0, self._update_native_minimap)
                except Exception:
                    pass
            self._sync_native_map_vehicle_arrow_scale()
            self._stack_native_overlays_above_tile_map()
            if bool(getattr(self, "_video_swapped", False)):
                self._ensure_video_pro_hud_visible()
        except Exception:
            return

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
                return (
                    max(0.0, min(1.0, xn)),
                    max(0.0, min(1.0, yn)),
                )
        except Exception:
            pass
        w = max(1, int(self._native_video_preview.width()))
        h = max(1, int(self._native_video_preview.height()))
        return (
            max(0.0, min(1.0, float(pos.x()) / float(w))),
            max(0.0, min(1.0, float(pos.y()) / float(h))),
        )

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

    def _sync_native_video_overlay(self) -> None:
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
            # Swapped fullscreen: stretch like the map (no letterboxing). PiP keeps aspect ratio.
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
        # Fullscreen from split: operator clicked a quadrant — stretch that channel only.
        if bool(getattr(self, "_video_swapped", False)):
            focus = getattr(self, "_split_fullscreen_source_id", None)
            if focus:
                cache0 = getattr(self, "_split_last_images", None) or {}
                im0 = cache0.get(str(focus))
                if isinstance(im0, QImage) and not im0.isNull() and im0.width() > 0:
                    try:
                        self._render_native_video_preview(im0)
                    except Exception:
                        pass
                    return
                try:
                    self._render_split_fullscreen_waiting(str(focus))
                except Exception:
                    pass
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
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, True)
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
                            f"Cell {slot}\nEmpty",
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

    def _seed_split_cache_from_last_frame(self) -> None:
        """After enabling split, paint immediately from the last single-view frame (avoids blank until next tick)."""
        if not bool(getattr(self, "_video_split_enabled", False)):
            return
        try:
            im = getattr(self, "_native_video_last", None)
            if not isinstance(im, QImage) or im.isNull():
                return
            c = getattr(self, "_split_last_images", None)
            if c is None:
                return
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

    def _sync_map_action_rail_enabled(self) -> None:
        """Match legacy `setActionButtonsEnabled`: Takeoff/Return on map when MAVLink link is up."""
        ok = bool(getattr(self, "_last_link_connected", False))
        for b in (
            getattr(self, "_map_action_takeoff_btn", None),
            getattr(self, "_map_action_return_btn", None),
        ):
            if b is not None:
                b.setEnabled(ok)

    def set_link_connected(self, connected: bool) -> None:
        c = bool(connected)
        if self._last_link_connected == c:
            return
        self._last_link_connected = c
        try:
            print(f"[VGCS:map] link_connected={c}")
        except Exception:
            pass
        self._sync_map_action_rail_enabled()
        self._run_js("setLinkConnected(true);" if c else "setLinkConnected(false);")
        if c:
            if self._video_preview_should_run() and not bool(
                getattr(self, "_video_swap_user_map_main", False)
            ):
                self._video_swapped = False
            try:
                self.clear_flight_track()
                self.set_video_follow_enabled(True)
                if getattr(self, "_lat", None) is not None and getattr(self, "_lon", None) is not None:
                    self.center_on_vehicle()
            except Exception:
                pass
        # Run a one-time tile probe after connect to log "blocked vs placeholder" clearly.
        if c and not getattr(self, "_tile_probe_ran", False):
            self._tile_probe_ran = True
            try:
                QTimer.singleShot(1500, lambda: self._probe_current_tiles(reason="connect"))
            except Exception:
                pass
        webcam_enabled = bool(getattr(self, "_btn_webcam", None)) and bool(self._btn_webcam.isChecked())
        # Do not stop video preview purely because telemetry link is down.
        # This matches QGC behavior and avoids blank video when the vehicle is in
        # pre-arm/GPS-failure states.
        if not c:
            # Keep camera controls hidden until MAVLink link-up (heartbeat).
            try:
                self._native_hud_right.hide()
            except Exception:
                pass
            try:
                self._native_rail_layer.hide()
            except Exception:
                pass
            try:
                self._obstacle_radar.hide()
                self._obstacle_radar.notify_link_connected(False)
            except Exception:
                pass
            # Companion RTSP (192.168.144.x) is independent of the serial MAVLink link — do not
            # tear down FFmpeg or clear the preview pixmap on COM disconnect/reconnect.
            if self._uses_companion_rtsp() and self._video_preview_should_run():
                return
            if webcam_enabled and bool(getattr(self, "_web_ready", False)):
                self._start_video_preview()
            else:
                self._stop_video_preview(clear_overlay=True)
            return
        # MAVLink connected: refresh HUD geometry (rail stays visible whenever map is ready; see `_on_map_loaded`).
        try:
            if self._plan_flight_layer_obscures_native_camera_ui():
                self._native_hud_right.hide()
                self._native_rail_layer.hide()
            else:
                self._native_hud_right.show()
                try:
                    self._native_rail_layer.show()
                except Exception:
                    pass
            if bool(getattr(self, "_web_ready", False)):
                self._native_compass.show()
                self._native_telemetry.show()
                try:
                    self._obstacle_radar.show()
                    self._obstacle_radar.notify_link_connected(True)
                except Exception:
                    pass
            QTimer.singleShot(0, self._layout_native_hud)
            QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
            QTimer.singleShot(0, self._stack_native_overlays_above_tile_map)
        except Exception:
            pass

    def _on_mavlink_link_show_mini_video(self) -> None:
        """MAVLink connected: show PiP shell immediately, then start decode when possible."""
        if not bool(getattr(self, "_last_link_connected", False)):
            return
        self._show_mini_video_pip_shell()
        if self._video_preview_should_run():
            self._auto_start_mini_video_pip(force_decode=True, preserve_layout=True)
            QTimer.singleShot(
                800,
                lambda: self._companion_start_decode_if_needed(reason="mavlink_link"),
            )

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
        if self._companion_decode_running(vp):
            try:
                print(
                    f"[VGCS:video] decode already active ({reason}), "
                    "skipping restart (ZR10 allows one RTSP client)"
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

    def _activate_startup_tile_source(self) -> None:
        """Default to Esri satellite; use offline only when explicitly configured and tiles exist."""
        try:
            s = QSettings(_QS_NS, _QS_APP)
            mode = str(s.value(_KEY_MAP_TILE_MODE, "sat") or "sat").strip().lower()
            root = str(s.value(_KEY_MAP_OFFLINE_TILE_ROOT, "") or "").strip()
        except Exception:
            mode, root = "sat", ""
        use_offline = mode == "offline" and bool(root) and Path(root).is_dir()
        if use_offline:
            self.activate_offline_tiles(root)
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "local_viewport_has_tiles"):
                try:
                    if not nm.local_viewport_has_tiles():
                        use_offline = False
                        print(
                            "[VGCS:map] offline folder has no tiles for this location — "
                            "using Esri World Imagery"
                        )
                except Exception:
                    use_offline = False
        if not use_offline:
            self.activate_satellite_tiles()

    def _native_tile_startup_check(self) -> None:
        """If the native 2D map still has no tiles, switch to Esri or nudge fetch (no cache wipe loop)."""
        if getattr(self, "_native_tile_fallback_done", False):
            return
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            n = int(nm.loaded_tile_count())
        except Exception:
            n = 0
        if n > 2:
            self._native_tile_fallback_done = True
            return
        tmpl = str(getattr(nm, "_tile_template", "") or "").lower()
        retries = int(getattr(self, "_native_tile_startup_retries", 0) or 0)
        if retries >= 2:
            try:
                QTimer.singleShot(1500, self._native_tile_startup_check_final)
            except Exception:
                pass
            return
        self._native_tile_startup_retries = retries + 1
        try:
            print(
                f"[VGCS:map] native tiles still loading (loaded={n}) — "
                f"{'switching to Esri' if '{local}' in tmpl else 'nudge fetch'}"
            )
        except Exception:
            pass
        if "{local}" in tmpl:
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
        elif "world_imagery" in tmpl:
            try:
                nm.prefetch_viewport_tiles()
                nm._warm_disk_tiles_for_viewport()
            except Exception:
                pass
        else:
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
        try:
            QTimer.singleShot(2500, self._native_tile_startup_check)
        except Exception:
            pass

    def _native_tile_startup_check_final(self) -> None:
        if getattr(self, "_native_tile_fallback_done", False):
            return
        if getattr(self, "_web_2d_fallback_active", False):
            return
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            n = int(nm.loaded_tile_count())
        except Exception:
            n = 0
        if n > 2:
            self._native_tile_fallback_done = True
            return
        self._native_tile_fallback_done = True
        tmpl = str(getattr(nm, "_tile_template", "") or "").lower()
        if "{local}" in tmpl or n <= 0:
            try:
                print("[VGCS:map] tiles still missing — activating Esri World Imagery")
                self.activate_satellite_tiles()
            except Exception:
                pass
        try:
            nm.prefetch_viewport_tiles()
            nm._warm_disk_tiles_for_viewport()
        except Exception:
            pass
        self._set_status(
            "Satellite tiles loading slowly — check internet/firewall or pick Offline Tiles in the toolbar"
        )

    def _sync_web_map_center_from_native(self) -> None:
        """Push native map center/zoom into Leaflet before showing the WebEngine view."""
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            lat = float(getattr(nm, "_center_lat", 37.7749))
            lon = float(getattr(nm, "_center_lon", -122.4194))
            zoom = float(getattr(nm, "_zoom", 16.0))
        except Exception:
            return
        try:
            self._run_js(
                f"window.__vgcsPendingCenter=[{lat},{lon}];"
                f"window.__vgcsPendingZoom={zoom};"
            )
        except Exception:
            pass

    def _relayout_web_map_view(self) -> None:
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is None:
            return
        try:
            host = getattr(self, "_map_stack", None) or self._map_canvas
            w3.resize(max(1, host.width()), max(1, host.height()))
        except Exception:
            pass
        try:
            w3.page().runJavaScript(_WEB_MAP_RELAYOUT_JS, lambda *_: None)
        except Exception:
            pass

    def _ensure_native_map_visible(self) -> None:
        """Keep 2D map on NativeTileMapView (no WebEngine Leaflet layer)."""
        if getattr(self, "_is_3d_mode", False):
            return
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        if getattr(self, "_web_2d_fallback_active", False):
            self._web_2d_fallback_active = False
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            if int(nm.loaded_tile_count()) >= 4:
                nm.update()
                nm.repaint()
                return
        except Exception:
            pass
        if not getattr(self, "_native_tile_fallback_done", False):
            try:
                QTimer.singleShot(0, self._native_tile_startup_check)
            except Exception:
                pass

    def _ensure_map_tiles_visible(self) -> None:
        """Alias: 2D map is always native Qt (see ``_ensure_native_map_visible``)."""
        self._ensure_native_map_visible()

    def _activate_web_2d_fallback(self) -> bool:
        """Optional Leaflet 2D via WebEngine (off by default; set VGCS_ALLOW_WEB_2D_FALLBACK=1)."""
        if not _web_2d_fallback_allowed():
            self._ensure_native_map_visible()
            return False
        if getattr(self, "_web_2d_fallback_active", False):
            self._relayout_web_map_view()
            return True
        if not HAS_WEBENGINE_3D:
            try:
                print("[VGCS:map] web 2D fallback unavailable (Qt WebEngine not installed)")
            except Exception:
                pass
            return False
        if not self._ensure_web_3d_view():
            return False

        def _apply() -> None:
            self._web_2d_fallback_active = True
            self._native_tile_fallback_done = True
            self._pending_web_2d_fallback = False
            self._is_3d_mode = False
            self._sync_web_map_center_from_native()
            try:
                self._map_stack.setCurrentWidget(self._web_3d_view)
            except Exception:
                try:
                    self._map_stack.setCurrentIndex(1)
                except Exception:
                    pass
            self._inject_legacy_html_hud_hide()
            w3 = getattr(self, "_web_3d_view", None)
            if w3 is not None:
                try:
                    w3.show()
                    w3.raise_()
                except Exception:
                    pass
            try:
                self.activate_satellite_tiles()
            except Exception:
                pass
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._schedule_vehicle_pose_js(immediate=True)
            self._relayout_web_map_view()
            for ms in (0, 80, 250, 600):
                QTimer.singleShot(ms, self._relayout_web_map_view)
            try:
                print("[VGCS:map] activated WebEngine 2D fallback (native tiles did not load)")
            except Exception:
                pass
            self._set_status("Map: web view active (native tiles blocked on this PC)")

        w3 = getattr(self, "_web_3d_view", None)
        if w3 is not None and bool(getattr(self, "_web_3d_ready", False)):
            _apply()
            return True
        self._pending_web_2d_fallback = True
        return True

    def _probe_current_tiles(self, *, reason: str) -> None:
        # Probe the *current* view tile (not just z=0), because placeholders often occur only at higher zooms.
        nm = getattr(self, "_native_map", None)
        if nm is not None and hasattr(nm, "get_map_view_dict"):
            try:
                payload = json.dumps(nm.get_map_view_dict())
                self._probe_current_tiles_from_payload(payload, reason=reason)
                return
            except Exception:
                pass

        def _kick(payload: str | None) -> None:
            self._probe_current_tiles_from_payload(payload, reason=reason)

        try:
            self._run_js("window.__vgcsGetMapView ? window.__vgcsGetMapView() : '';", callback=_kick)
        except Exception:
            _kick(None)

    def _probe_current_tiles_from_payload(self, payload: str | None, *, reason: str) -> None:
        try:
            print(f"[VGCS:map] map_view_payload ({reason}) {str(payload or '')[:220]}")
        except Exception:
            pass
        try:
            data = json.loads(payload or "{}")
        except Exception:
            data = {}
        try:
            z = int(data.get("z", 0) or 0)
            lat = float(data.get("lat", 0.0) or 0.0)
            lng = float(data.get("lng", 0.0) or 0.0)
            tmpl = str(data.get("template", "") or "")
        except Exception:
            z, lat, lng, tmpl = 0, 0.0, 0.0, ""

        def slippy_xy(lat_deg: float, lon_deg: float, zoom: int) -> tuple[int, int]:
            import math

            lat_rad = math.radians(max(-85.0511, min(85.0511, lat_deg)))
            n = 2.0**zoom
            xt = int((lon_deg + 180.0) / 360.0 * n)
            yt = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
            return max(0, xt), max(0, yt)

        candidates: list[tuple[str, str]] = []
        if tmpl:
            x, y = slippy_xy(lat, lng, max(0, min(19, z)))
            if tmpl == "{local}":
                nm = getattr(self, "_native_map", None)
                root = getattr(nm, "_offline_root", None) if nm is not None else None
                if root:
                    p = Path(str(root)) / str(z) / str(x) / f"{y}.png"
                    candidates.append(("active_view", p.as_uri()))
            else:
                url = (
                    tmpl.replace("{z}", str(z))
                    .replace("{x}", str(x))
                    .replace("{y}", str(y))
                    .replace("{s}", "a")
                )
                candidates.append(("active_view", url))
        else:
            candidates.extend(
                [
                    (
                        "esri_imagery_view",
                        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/10/0/0",
                    ),
                    (
                        "esri_streets_view",
                        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/10/0/0",
                    ),
                ]
            )

        if tmpl and tmpl != "{local}":
            z0 = (
                tmpl.replace("{z}", "0")
                .replace("{x}", "0")
                .replace("{y}", "0")
                .replace("{s}", "a")
            )
            candidates.append(("active_world", z0))

        for label, url in candidates:
            try:
                QThreadPool.globalInstance().start(
                    _TileProbeTask(
                        url=url,
                        provider_label=f"{label}:{reason}",
                        bridge=self._tile_probe_bridge,
                    )
                )
            except Exception:
                pass

    def _on_tile_probe_result(self, provider_label: str, outcome: str, detail: str) -> None:
        try:
            print(f"[VGCS:map] tile_probe {provider_label} -> {outcome} ({detail})")
        except Exception:
            pass
        label = str(provider_label)
        detail_s = str(detail)
        network_fail = outcome.startswith("error:") and (
            "URLError" in detail_s or "urlopen error" in detail_s.lower()
        )
        if "active_view" in label and network_fail:
            if not getattr(self, "_tile_network_fallback_done", False):
                self._tile_network_fallback_done = True
                self._set_status(
                    "Satellite tiles unreachable — check internet or use Offline Tiles in the toolbar"
                )
                try:
                    QTimer.singleShot(800, self.activate_satellite_tiles)
                except Exception:
                    pass
            return
        # Do not auto-switch map type (product default is Esri World Imagery satellite).
        if "active_view" in label and outcome == "placeholder_suspected":
            self._set_status(
                "Satellite tile looks blocked/placeholder — use Offline Tiles or Esri Streets in the toolbar"
            )
        if "esri_imagery" in label and outcome == "placeholder_suspected":
            self._set_status(
                "Satellite imagery blocked on this network — use Offline Tiles or Esri Streets in the toolbar"
            )

    def _set_webcam_enabled(self, enabled: bool) -> None:
        on = bool(enabled)
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_WEBCAM_ENABLED, on)
        except Exception:
            pass
        if not on:
            self._stop_video_preview(clear_overlay=True)
            self._set_status("Webcam disabled")
            return
        # Start if the page is ready; do not require telemetry link.
        if bool(getattr(self, "_web_ready", False)):
            self._start_video_preview()
            self._set_status("Webcam enabled")
        else:
            self._set_status("Webcam enabled (will start when map is ready)")

    def _on_perf_mode_changed(self, idx: int) -> None:
        # 0=Auto, 1=Low, 2=High
        mode = "auto"
        if int(idx) == 1:
            mode = "on"
        elif int(idx) == 2:
            mode = "off"
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_LOW_SPEC_MODE, mode)
        except Exception:
            pass
        if mode == "on":
            self._apply_low_spec_mode(True, reason="manual")
        elif mode == "off":
            self._apply_low_spec_mode(False, reason="manual")
        else:
            # Auto: apply any already-detected effective mode; otherwise default to high.
            self._apply_low_spec_mode(bool(getattr(self, "_low_spec_autodetected", False)), reason="auto")

    def _apply_low_spec_mode(self, enabled: bool, *, reason: str) -> None:
        on = bool(enabled)
        self._low_spec_effective = on
        # Python-side throttles (JS bridge pressure is a major cause of lag).
        try:
            self._vehicle_pose_timer.setInterval(240 if on else 120)
        except Exception:
            pass
        try:
            if hasattr(self, "_video_push_timer"):
                self._video_push_timer.setInterval(66)
        except Exception:
            pass
        # Keep video quality stable even in low-spec mode.
        # Low-spec still reduces map/tile workload, but stream fidelity is preserved.
        try:
            self._video_encode_max_w = 1920
            self._video_encode_max_h = 1080
            self._video_encode_format = "PNG"
            self._video_encode_quality = 1
        except Exception:
            pass
        # JS-side tile/label adjustments.
        try:
            self._run_js("setLowSpecMode(true);" if on else "setLowSpecMode(false);")
        except Exception:
            pass
        if reason == "manual":
            self._set_status("Performance: Low" if on else "Performance: High")

    def _maybe_autodetect_low_spec(self) -> None:
        # Heuristic: measure WebEngine JS callback latency; if it's consistently high, enable low-spec.
        try:
            s = QSettings(_QS_NS, _QS_APP)
            mode = str(s.value(_KEY_MAP_LOW_SPEC_MODE, "auto") or "auto").strip().lower()
        except Exception:
            mode = "auto"
        if mode != "auto":
            return
        if getattr(self, "_low_spec_autodetected", False):
            return
        if not bool(getattr(self, "_web_ready", False)):
            return

        try:
            from time import perf_counter
        except Exception:
            return

        samples: list[float] = []

        def one() -> None:
            t0 = perf_counter()

            def cb(_val) -> None:
                dt = (perf_counter() - t0) * 1000.0
                samples.append(float(dt))
                if len(samples) >= 3:
                    avg = sum(samples) / len(samples)
                    if avg >= 120.0:
                        self._low_spec_autodetected = True
                        self._apply_low_spec_mode(True, reason="auto")
                        self._set_status("Performance: Auto (low-spec detected)")
                    return
                one()

            self._run_js("Date.now();", callback=cb)

        one()

    def set_flight_status(self, status: str, detail: str = "") -> None:
        st = (status or "").strip().lower()
        if st not in {"green", "yellow", "red", "idle"}:
            st = "red"
        d = str(detail)
        key = (st, d)
        if self._last_flight_status_key == key:
            return
        self._last_flight_status_key = key
        # Idle/neutral is owned by Qt `#linkBanner` styling; skip legacy JS tint when offline maps only.
        if st == "idle":
            return
        self._run_js(f"setFlightStatus({json.dumps(st)}, {json.dumps(detail)});")

    def set_header_mode(self, mode_text: str) -> None:
        t = str(mode_text)
        if t == self._last_header_mode:
            return
        self._last_header_mode = t
        self._run_js(f"setHeaderMode({json.dumps(mode_text)});")

    def set_header_vehicle_msg(self, msg_text: str) -> None:
        self._run_js(f"setHeaderVehicleMsg({json.dumps(msg_text)});")

    def set_header_gps(
        self,
        satellites: int | str,
        hdop_text: str,
        *,
        fix_type: int | None = None,
    ) -> None:
        try:
            self._gps_satellites = int(satellites)
        except Exception:
            pass
        if fix_type is not None:
            try:
                self._gps_fix_type = int(fix_type)
            except Exception:
                pass
        key = (str(satellites), str(hdop_text), str(fix_type))
        if self._last_header_gps_key == key:
            return
        self._last_header_gps_key = key
        self._run_js(f"setHeaderGps({json.dumps(str(satellites))}, {json.dumps(hdop_text)});")

    def set_header_battery(self, battery_text: str) -> None:
        bt = str(battery_text)
        if self._last_header_battery == bt:
            return
        self._last_header_battery = bt
        self._run_js(f"setHeaderBattery({json.dumps(battery_text)});")

    def set_header_remote_id(self, rid_text: str) -> None:
        self._run_js(f"setHeaderRemoteId({json.dumps(rid_text)});")

    def _layout_plan_flight_panel(self) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        try:
            w = max(1, self._map_canvas.width())
            h = max(1, self._map_canvas.height())
            origin = self._map_canvas.mapTo(self._panel, QPoint(0, 0))
            panel.setGeometry(origin.x(), origin.y(), w, h)
        except Exception:
            pass

    def _plan_flight_layer_obscures_native_camera_ui(self) -> bool:
        """True while Plan Flight covers the map — hide PiP / camera rail so planning stays uncluttered."""
        panel = getattr(self, "_plan_flight_panel", None)
        try:
            return panel is not None and panel.isVisible()
        except Exception:
            return False

    def _set_map_footer_hud_visible(self, visible: bool) -> None:
        """Mirror legacy `setPlanFlightVisible(...)` which hid the bottom compass/telemetry while planning."""
        show = bool(visible) and bool(getattr(self, "_web_ready", False))
        try:
            if hasattr(self, "_native_telemetry"):
                self._native_telemetry.setVisible(show)
            if hasattr(self, "_native_compass"):
                self._native_compass.setVisible(show)
            if hasattr(self, "_obstacle_radar") and bool(getattr(self, "_last_link_connected", False)):
                self._obstacle_radar.setVisible(show)
        except Exception:
            pass

    def _on_plan_panel_exit(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.disable_plan_edit_modes()
        except Exception:
            pass
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.hide()
        try:
            mar = getattr(self, "_map_action_rail", None)
            if mar is not None:
                mar.show()
        except Exception:
            pass
        self._set_map_footer_hud_visible(True)
        try:
            self._layout_native_hud()
            self._stack_native_overlays_above_tile_map()
        except Exception:
            pass
        self.plan_flight_exited.emit()

    def _sync_native_plan_edit_mode_for_rail_tool(self, tool: str) -> None:
        """Keep native map placement modes aligned with the active plan rail tool."""
        tl = (tool or "").strip().lower()
        if tl == "waypoint":
            self._enable_add_waypoint_mode()
            return
        if tl == "roi":
            self._enable_fence_polygon_mode()
            return
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.disable_plan_edit_modes()
        except Exception:
            pass

    def _on_plan_panel_tool(self, tool: str) -> None:
        t = (tool or "").strip()
        if not t:
            return
        self._plan_rail_tool_state = t
        self._sync_native_plan_edit_mode_for_rail_tool(t)
        self.plan_tool_requested.emit(t)

    def _on_plan_panel_mission_changed(self, payload: object) -> None:
        data = dict(payload) if isinstance(payload, dict) else {}
        self.plan_mission_panel_changed.emit(data)

    def _on_plan_panel_waypoints_changed(self, waypoints: object) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        try:
            n = len(waypoints) if hasattr(waypoints, "__len__") else 0
        except Exception:
            n = 0
        panel.set_waypoint_count(int(n))

    def _on_plan_panel_set_launch_to_map_center(self) -> None:
        nm = getattr(self, "_native_map", None)
        if nm is None:
            return
        try:
            lat = float(getattr(nm, "_center_lat", 0.0) or 0.0)
            lon = float(getattr(nm, "_center_lon", 0.0) or 0.0)
        except Exception:
            return
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_launch_position(lat, lon)

    def set_plan_flight_visible(self, visible: bool) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is None:
            return
        if visible:
            self._layout_plan_flight_panel()
            panel.set_rail_tool(self._plan_rail_tool_state or "File")
            panel.set_waypoint_count(len(self._waypoints_model))
            panel.show()
            panel.raise_()
            try:
                mar = getattr(self, "_map_action_rail", None)
                if mar is not None:
                    mar.hide()
            except Exception:
                pass
            self._set_map_footer_hud_visible(False)
        else:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None:
                    nm.disable_plan_edit_modes()
            except Exception:
                pass
            panel.hide()
            try:
                mar = getattr(self, "_map_action_rail", None)
                if mar is not None:
                    mar.show()
            except Exception:
                pass
            self._set_map_footer_hud_visible(True)
        try:
            self._layout_native_hud()
            self._stack_native_overlays_above_tile_map()
        except Exception:
            pass

    def set_plan_flight_metrics(
        self,
        *,
        alt_diff_m: str,
        gradient: str,
        azimuth: str,
        heading: str,
        dist_prev_wp_m: str,
        mission_distance_m: str,
        mission_time: str,
        max_telem_dist_m: str,
    ) -> None:
        payload = {
            "altDiffM": alt_diff_m,
            "gradient": gradient,
            "azimuth": azimuth,
            "heading": heading,
            "distPrevWpM": dist_prev_wp_m,
            "missionDistanceM": mission_distance_m,
            "missionTime": mission_time,
            "maxTelemDistM": max_telem_dist_m,
        }
        if payload == self._last_plan_flight_metrics_payload:
            return
        self._last_plan_flight_metrics_payload = payload
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_metrics(payload)

    def refresh_plan_flight_chrome(self, *, link_ok: bool, waypoint_count: int) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_chrome_state(bool(link_ok), max(0, int(waypoint_count)))

    def center_on_vehicle(self) -> None:
        """Recenter the map on the vehicle (native: `set_center` from widget coords, else native vehicle)."""
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                la = getattr(self, "_lat", None)
                lo = getattr(self, "_lon", None)
                if la is not None and lo is not None:
                    nm.set_center(float(la), float(lo))
                else:
                    nm.center_on_vehicle()
                try:
                    if self._native_minimap_wrap.isVisible():
                        self._update_native_minimap()
                except Exception:
                    pass
                return
        except Exception:
            pass
        self._run_js("centerOnVehicle();")

    def set_plan_rail_tool(self, tool: str) -> None:
        t = (tool or "").strip()
        if not t:
            return
        self._plan_rail_tool_state = t
        self._sync_native_plan_edit_mode_for_rail_tool(t)
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_rail_tool(t)

    def apply_plan_mission_panel_state(self, state: dict[str, object]) -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.apply_panel_state(dict(state) if isinstance(state, dict) else {})

    def set_plan_sequence_template(self, template_id: str | None) -> None:
        """Show/hide Mission tab pattern row (Survey / Corridor / Structure) to match template picks."""
        tid = (template_id or "").strip().lower()
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_sequence_template(tid)

    def set_plan_mission_start_stack(self, enabled: bool, survey_label: str = "Survey") -> None:
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_mission_start_stack(bool(enabled), str(survey_label or "Survey"))

    def set_plan_vehicle_info(self, firmware: str, vehicle: str) -> None:
        key = (str(firmware), str(vehicle))
        if key == self._last_plan_vehicle_info_key:
            return
        self._last_plan_vehicle_info_key = key
        panel = getattr(self, "_plan_flight_panel", None)
        if panel is not None:
            panel.set_vehicle_info(str(firmware or ""), str(vehicle or ""))

    def get_default_waypoint_alt_m(self) -> float:
        return float(self._default_alt.value())

    def set_default_waypoint_alt_m(self, alt_m: float) -> None:
        self._default_alt.setValue(max(1.0, float(alt_m)))

    def request_mission_upload_from_map(self) -> None:
        self._request_upload()

    def request_mission_download_from_map(self) -> None:
        self._request_download()

    def clear_map_waypoints(self) -> None:
        self._clear_waypoints()

    def clear_plan_current_mission_path(self) -> None:
        """Forget the Plan Flight JSON path (e.g. after download or generated pattern)."""
        s = QSettings(_QS_NS, _QS_APP)
        s.remove(_KEY_PLAN_CURRENT_MISSION_JSON)
        if s.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            s.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)

    def start_waypoint_planning(self) -> None:
        self._enable_add_waypoint_mode()

    def start_roi_planning(self) -> None:
        self._enable_fence_polygon_mode()

    def open_mission_file(self) -> None:
        self._import_mission()

    def get_vehicle_position(self) -> tuple[float, float] | None:
        if self._lat is None or self._lon is None:
            return None
        return float(self._lat), float(self._lon)

    def _init_map_backend(self) -> None:
        self._map_canvas.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self._map_canvas.setAutoFillBackground(True)
        self._map_stack = QStackedWidget(self._map_canvas)
        self._map_stack.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._native_map = NativeTileMapView()
        self._native_map.setMinimumHeight(260)
        self._native_map.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._map_stack.addWidget(self._native_map)
        self._map_canvas_layout.addWidget(self._map_stack, 1)
        try:
            self._map_stack.setCurrentIndex(0)
        except Exception:
            pass
        # PiP / compass / rail layer stay on `_map_canvas`; keep the map stack below those siblings.
        try:
            self._map_stack.lower()
        except Exception:
            pass
        self._native_map.user_waypoints_changed.connect(self._on_native_user_waypoints_changed)
        self._native_map.observation_map_click.connect(
            lambda la, lo: self._log_observation("map_mark", map_lat=float(la), map_lon=float(lo))
        )
        try:
            seed_ok = bundled_seed_root().is_dir()
            print(
                f"[VGCS:map] backend build {MAP_BACKEND_BUILD} | "
                f"[VGCS:map-native] ready bundled_seed={'yes' if seed_ok else 'no'}"
            )
        except Exception:
            pass
        self._web_ready = True
        self._set_status("Map backend: Native Qt tiles")
        self._on_map_loaded(True)

        try:
            s = QSettings(_QS_NS, _QS_APP)
            mode = str(s.value(_KEY_MAP_LOW_SPEC_MODE, "auto") or "auto").strip().lower()
        except Exception:
            mode = "auto"
        if mode == "on":
            self._apply_low_spec_mode(True, reason="manual")
            try:
                self._perf_mode.setCurrentIndex(1)
            except Exception:
                pass
        elif mode == "off":
            self._apply_low_spec_mode(False, reason="manual")
            try:
                self._perf_mode.setCurrentIndex(2)
            except Exception:
                pass
        else:
            try:
                self._perf_mode.setCurrentIndex(0)
            except Exception:
                pass

        # 2D map stays on NativeTileMapView. WebEngine loads only when the operator enables 3D.
        try:
            QTimer.singleShot(0, self._ensure_native_map_visible)
        except Exception:
            pass

    def _on_native_user_waypoints_changed(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is None:
                return
            self._on_waypoints_json(nm.waypoints_json())
        except Exception:
            pass

    def _on_map_loaded(self, ok: bool) -> None:
        self._web_ready = bool(ok)
        if self._web_ready:
            self._last_plan_flight_metrics_payload = None
            self._last_flight_telemetry_sig = None
            self._last_link_connected = None
            self._last_flight_status_key = None
            self._last_header_gps_key = None
            self._last_header_battery = None
            self._last_header_mode = None
            self._last_plan_vehicle_info_key = None
            self._vehicle_pose_timer.stop()
            self._set_status("Map ready")
            t = self._plan_rail_tool_state
            self._run_js(
                f"window.__planRailTool = {json.dumps(t)}; setPlanRailTool({json.dumps(t)});"
            )
            # Tile selection: Esri satellite by default; offline only if mode=offline and tiles exist.
            try:
                self._activate_startup_tile_source()
                try:
                    QTimer.singleShot(1200, lambda: self._probe_current_tiles(reason="startup"))
                except Exception:
                    pass
                try:
                    QTimer.singleShot(2000, self._native_tile_startup_check)
                except Exception:
                    pass
                for delay_ms in (900, 1800, 3500):
                    try:
                        QTimer.singleShot(delay_ms, self._ensure_native_map_visible)
                    except Exception:
                        pass
            except Exception:
                pass
            self.map_page_ready.emit()
            try:
                self._sync_native_camera_rail_toggles()
            except Exception:
                pass
            try:
                self.apply_video_settings()
            except Exception:
                pass
            try:
                self._native_compass.show()
                self._native_telemetry.show()
                if bool(getattr(self, "_last_link_connected", False)):
                    try:
                        self._obstacle_radar.show()
                    except Exception:
                        pass
                # Keep camera rail (ZOOM/FOCUS/GIMBAL/OBSERVE) hidden until MAVLink is connected.
                # This ensures the UI follows the desired flow: "Disconnected" -> no camera controls.
                if bool(getattr(self, "_last_link_connected", False)):
                    if self._plan_flight_layer_obscures_native_camera_ui():
                        try:
                            self._native_hud_right.hide()
                            self._native_rail_layer.hide()
                        except Exception:
                            pass
                    else:
                        self._native_hud_right.show()
                        try:
                            self._native_rail_layer.show()
                        except Exception:
                            pass
                    # Layout + Z-order immediately: `resizeEvent` can run while the layer is still hidden,
                    # which skipped `ly.raise_()` when gated on `isVisible()` — map then stayed above the rail.
                    try:
                        self._layout_native_hud()
                        self._stack_native_overlays_above_tile_map()
                    except Exception:
                        pass
                    QTimer.singleShot(0, self._layout_native_hud)
                else:
                    try:
                        self._native_hud_right.hide()
                    except Exception:
                        pass
                    try:
                        self._native_rail_layer.hide()
                    except Exception:
                        pass
            except Exception:
                pass
            # Start preview if user enabled it (do not require telemetry link).
            try:
                if bool(getattr(self, "_last_link_connected", False)):
                    QTimer.singleShot(0, self._on_mavlink_link_show_mini_video)
                elif self._video_preview_should_run():
                    self._show_mini_video_pip_shell()
                    self._auto_start_mini_video_pip(force_decode=False)
            except Exception:
                pass
            # Auto-detect low-spec devices and reduce map workload if needed.
            try:
                QTimer.singleShot(250, self._maybe_autodetect_low_spec)
            except Exception:
                pass
        else:
            self._set_status("Map failed to load")

    def _hook_video_pipeline_sources_changed(self, vp: VideoPipeline | None) -> None:
        if vp is None:
            return
        vid = id(vp)
        if getattr(self, "_video_sources_changed_conn_id", None) == vid:
            return
        try:
            vp.sources_changed.connect(
                self._on_video_pipeline_sources_changed,
                Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            return
        self._video_sources_changed_conn_id = vid

    def _detach_video_pipeline_frame_slots(self, vp: VideoPipeline | None) -> None:
        """Remove MapWidget as a receiver on every source `frame` signal (safe before re-bind)."""
        if vp is None:
            return
        try:
            for _sid, src in list(vp.sources().items())[:12]:
                try:
                    if hasattr(src, "frame") and hasattr(src.frame, "disconnect"):
                        src.frame.disconnect(self)
                except Exception:
                    pass
        except Exception:
            pass

    def _connect_video_pipeline_frame_slots(self, vp: VideoPipeline | None) -> bool:
        """Bind RtspSource.frame → native preview handlers (required after refresh_sources)."""
        if vp is None:
            return False
        try:
            sources = vp.sources()
        except Exception:
            return False
        if not sources:
            return False
        self._detach_video_pipeline_frame_slots(vp)
        try:
            self._video = vp
            self._video_active_source = vp.active_source()
            src_ids = set(sources.keys())
            preferred_id = ""
            if "day" in src_ids:
                preferred_id = "day"
            elif "thermal" in src_ids:
                preferred_id = "thermal"
            elif src_ids:
                preferred_id = str(next(iter(src_ids)))
            if preferred_id:
                try:
                    vp.set_active_source(preferred_id)
                    self._video_active_source = vp.active_source()
                except Exception:
                    pass
            active = self._video_active_source
            if active is not None:
                try:
                    active.frame.disconnect(self)
                except Exception:
                    pass
                active.frame.connect(
                    self._on_pipeline_frame,
                    Qt.ConnectionType.QueuedConnection,
                )
            for sid, src in list(sources.items())[:4]:
                if not isinstance(getattr(self, "_video_encode_bridge_by_id", None), dict):
                    self._video_encode_bridge_by_id = {}
                if sid not in self._video_encode_bridge_by_id:
                    bridge = _VideoEncodeBridge(self)
                    bridge.encoded.connect(
                        lambda data_url, sid=sid: self._on_video_frame_encoded_for(sid, data_url)
                    )
                    self._video_encode_bridge_by_id[sid] = bridge
                try:
                    if hasattr(src, "error") and hasattr(src.error, "disconnect"):
                        src.error.disconnect(self)
                except Exception:
                    pass
                try:
                    if hasattr(src, "error") and hasattr(src.error, "connect"):
                        src.error.connect(
                            lambda msg, sid=sid: (
                                print(f"[VGCS:video] Video({sid}) error: {str(msg)}"),
                                self._set_status(f"Video({sid}) error: {str(msg)}"),
                            )[1],
                            Qt.ConnectionType.QueuedConnection,
                        )
                except Exception:
                    pass
                try:
                    if hasattr(src, "frame") and hasattr(src.frame, "disconnect"):
                        src.frame.disconnect(self)
                except Exception:
                    pass
                src.frame.connect(
                    lambda vf, sid=sid: self._on_pipeline_frame_for(sid, vf),
                    Qt.ConnectionType.QueuedConnection,
                )
            shared = getattr(self, "_video_pipeline_shared", None)
            if shared is not None and vp is shared:
                self._shared_vp_hooks_connected = True
            return active is not None
        except Exception:
            return False

    def _on_video_pipeline_sources_changed(self) -> None:
        """`refresh_sources()` swapped in new `RtspSource` objects — re-run preview hook-up."""
        if not HAS_MULTIMEDIA:
            return
        if not bool(getattr(self, "_web_ready", False)):
            return
        if not self._video_preview_should_run():
            return
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            return
        try:
            if not vp.sources():
                return
        except Exception:
            return
        self._video_inited = False
        self._shared_vp_hooks_connected = False
        setattr(self, "_video_skip_preview_flag_reset_in_ensure", True)
        if not self._ensure_video_preview_backend(from_start=True):
            try:
                print("[VGCS:video] sources_changed: preview hook-up failed (no sources?)")
            except Exception:
                pass
            return
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
        if self._should_defer_companion_rtsp_decode():
            return
        self._companion_start_decode_if_needed(reason="sources_changed")

    def _ensure_video_preview_backend(self, *, from_start: bool = False) -> bool:
        if not HAS_MULTIMEDIA:
            return False
        if getattr(self, "_video_inited", False):
            return bool(getattr(self, "_video", None)) and bool(getattr(self, "_video_active_source", None))

        shared = getattr(self, "_video_pipeline_shared", None)
        # After set_camera_control(), _video_inited is cleared but the shared pipeline + signal hooks stay valid.
        if shared is not None and getattr(self, "_shared_vp_hooks_connected", False):
            self._video_inited = True
            self._video = shared
            if not isinstance(getattr(self, "_split_last_images", None), dict):
                self._split_last_images = {}
            if not isinstance(getattr(self, "_native_pip_last_source_frame", None), QImage):
                self._native_pip_last_source_frame = QImage()
            if not self._connect_video_pipeline_frame_slots(shared):
                self._shared_vp_hooks_connected = False
                self._video_inited = False
            else:
                return bool(self._video_active_source)

        _skip_pv_reset = bool(getattr(self, "_video_skip_preview_flag_reset_in_ensure", False))
        try:
            delattr(self, "_video_skip_preview_flag_reset_in_ensure")
        except Exception:
            pass
        if not _skip_pv_reset and not from_start:
            self._video_preview_enabled = False

        self._video: VideoPipeline | None = None
        self._video_active_source = None
        # Do not clear `_video_split_enabled` / `_video_follow_enabled` here: the native rail
        # may set them just before this call (`VGCS_CAM_*_TOGGLE`); resetting would undo the click.
        self._video_recording = False
        self._video_recording_tmp_path = ""
        self._video_recording_source_id = ""
        self._stop_native_cam_recording_tick_timer(reset_label=True)
        self._video_vision_mode = "day"  # 'day' | 'night'
        self._video_zoom = 1.0  # 1.0x .. 4.0x
        self._video_follow_last_center_mono = 0.0
        self._video_last_data_url = ""
        self._video_last_data_urls = {}
        self._video_encode_inflight_by_id = {}
        self._video_encode_pending_by_id = {}
        self._video_encode_bridge_by_id = {}
        self._video_encode_inflight = False
        self._video_encode_pending = None
        self._video_encode_max_w = 1920
        self._video_encode_max_h = 1080
        self._video_encode_format = "PNG"
        self._video_encode_quality = 1
        self._video_pool = QThreadPool.globalInstance()
        self._split_last_images = {}
        self._native_pip_last_source_frame = QImage()
        self._ai_phase = 0.0

        try:
            if shared is not None:
                self._video = shared
            else:
                self._video = VideoPipeline(self)
            self._hook_video_pipeline_sources_changed(self._video)
            # Re-fetch sources after optional configure below.
            # When MainWindow's shared `VideoPipeline` already has RTSP sources (e.g. right after
            # `apply_video_settings_for_settings_dialog` scheduled a refresh), calling
            # `_configure_video_pipeline` again only re-schedules coalesced work — avoid doing it
            # when `sources()` is already populated so preview hook-up stays cheap.
            try:
                have_sources = bool(self._video.sources()) if self._video is not None else False
            except Exception:
                have_sources = False
            if not have_sources:
                try:
                    self._configure_video_pipeline(self._video)
                except Exception:
                    pass
            sources = self._video.sources()
        except Exception:
            self._video = None
            sources = {}
        if not sources:
            try:
                print(
                    "[VGCS:video] preview backend: no pipeline sources yet "
                    "(waiting for refresh_sources)"
                )
            except Exception:
                pass
            return False

        if not hasattr(self, "_video_push_timer") or self._video_push_timer is None:
            self._video_push_timer = QTimer(self)
            self._video_push_timer.setInterval(66)  # ~15 fps push to WebEngine
            self._video_push_timer.timeout.connect(self._push_video_preview_any_to_overlay)
        if not hasattr(self, "_video_preview_stall_timer") or self._video_preview_stall_timer is None:
            self._video_preview_stall_timer = QTimer(self)
            self._video_preview_stall_timer.setInterval(1000)
            self._video_preview_stall_timer.timeout.connect(self._on_video_preview_stall_check)
        if not hasattr(self, "_split_render_timer") or self._split_render_timer is None:
            self._split_render_timer = QTimer(self)
            self._split_render_timer.setSingleShot(True)
            self._split_render_timer.timeout.connect(self._flush_split_preview_render)
        if not hasattr(self, "_ai_timer") or self._ai_timer is None:
            self._ai_timer = QTimer(self)
            self._ai_timer.setInterval(250)  # 4 Hz — lower paint load during Target / RTSP
            self._ai_timer.timeout.connect(self._push_dummy_ai_overlay)

        try:
            old_br = getattr(self, "_video_encode_bridge", None)
            if old_br is not None:
                try:
                    old_br.encoded.disconnect(self)
                except Exception:
                    pass
        except Exception:
            pass
        self._video_encode_bridge = _VideoEncodeBridge(self)
        self._video_encode_bridge.encoded.connect(self._on_video_frame_encoded)

        try:
            if not isinstance(getattr(self, "_video_encode_bridge_by_id", None), dict):
                self._video_encode_bridge_by_id = {}
            self._video_encode_inflight_by_id = {}
            self._video_encode_pending_by_id = {}
            self._video_last_data_urls = {}
            if not self._connect_video_pipeline_frame_slots(self._video):
                self._video_active_source = None
                self._video_inited = False
                return False
            self._video_inited = True
        except Exception:
            self._video_active_source = None
            self._video_inited = False
            return False
        return True

    def _configure_video_pipeline(self, vp: VideoPipeline | None) -> None:
        """Apply Application Settings → Video URLs and mode to the shared (or local) pipeline."""
        if vp is None:
            return
        self._read_video_settings()
        if not bool(getattr(self, "_video_settings_enabled", False)):
            vp.set_rtsp_sources(
                day_url="",
                thermal_url="",
                transport="auto",
                stream_kind="rtsp",
            )
            return
        # Always register a non-empty thermal URL with the pipeline. Gating on default_view
        # == "split" broke the common case: operator sets day + thermal URLs but leaves Default
        # view on Single, then uses the camera rail ▦ split — the UI shows 4-up while only the
        # "day" RtspSource existed, so cells 2–4 stayed black.
        thermal_for_pipeline = str(self._video_settings_thermal or "").strip()
        kind = str(getattr(self, "_video_settings_source", "rtsp") or "rtsp").strip().lower()
        if kind == "disabled":
            kind = "rtsp"
        vp.set_rtsp_sources(
            day_url=str(self._video_settings_day),
            thermal_url=thermal_for_pipeline,
            transport=str(getattr(self, "_video_settings_rtsp_transport", "auto") or "auto"),
            stream_kind=kind,
            low_latency=bool(getattr(self, "_video_settings_low_latency", False)),
        )

    def _read_video_settings(self) -> None:
        s = QSettings(_QS_NS, _QS_APP)
        source = str(s.value(_KEY_VIDEO_SOURCE, "rtsp") or "rtsp").strip().lower()
        self._video_settings_source = source
        self._video_settings_day = str(s.value(_KEY_VIDEO_RTSP_DAY, "") or "").strip()
        self._video_settings_thermal = str(s.value(_KEY_VIDEO_RTSP_THERMAL, "") or "").strip()
        has_stream = bool(self._video_settings_day or self._video_settings_thermal) or source in (
            "udp_h264",
            "udp_h265",
        )
        explicit_on = bool(s.value(_KEY_VIDEO_ENABLED, False))
        self._video_settings_enabled = (explicit_on or has_stream) and source != "disabled"
        self._video_settings_rtsp_transport = str(s.value(_KEY_VIDEO_RTSP_TRANSPORT, "auto") or "auto").strip().lower()
        self._video_settings_low_latency = bool(s.value(_KEY_VIDEO_LOW_LATENCY, False))
        rec_fmt = str(s.value(_KEY_VIDEO_RECORD_FORMAT, "mp4") or "mp4").strip().lower()
        self._video_settings_record_format = rec_fmt if rec_fmt in ("mp4", "mkv") else "mp4"
        self._video_settings_default_view = str(s.value(_KEY_VIDEO_DEFAULT_VIEW, "Single") or "Single")

    def _video_record_suffix(self) -> str:
        return str(getattr(self, "_video_settings_record_format", "mp4") or "mp4")

    def _video_preview_should_run(self) -> bool:
        """True when a stream is configured and the map is ready (no toolbar toggle required)."""
        return bool(getattr(self, "_video_settings_enabled", False)) and bool(
            getattr(self, "_web_ready", False)
        )

    def _auto_start_mini_video_pip(
        self,
        *,
        force_decode: bool = False,
        preserve_layout: bool = False,
    ) -> None:
        """Show bottom-left mini-video automatically when RTSP/UDP is configured."""
        if not bool(getattr(self, "_last_link_connected", False)) and not self._video_preview_should_run():
            return
        self._show_mini_video_pip_shell()
        if not self._video_preview_should_run():
            return
        if preserve_layout:
            reset_swapped = False
        else:
            reset_swapped = not bool(getattr(self, "_video_swap_user_map_main", False))
        self._start_video_preview(reset_swapped=reset_swapped, force_decode=force_decode)

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

    def _apply_video_settings_read_toolbar(self) -> bool:
        """Reload QSettings-backed video fields, log, and mirror the webcam toolbar toggle."""
        was_preview_on = bool(getattr(self, "_video_preview_enabled", False))
        self._read_video_settings()
        try:
            print(
                f"[VGCS:video] settings enabled={self._video_settings_enabled} "
                f"source={self._video_settings_source} "
                f"day={self._video_settings_day!r} thermal={self._video_settings_thermal!r} "
                f"transport={self._video_settings_rtsp_transport!r}"
            )
        except Exception:
            pass
        try:
            self._btn_webcam.blockSignals(True)
            self._btn_webcam.setChecked(bool(self._video_settings_enabled))
        finally:
            try:
                self._btn_webcam.blockSignals(False)
            except Exception:
                pass
        return was_preview_on

    def apply_video_settings_for_settings_dialog(self) -> None:
        """Apply path used only after Application Settings → Video → Apply (see MainWindow).

        Stages preview teardown across timers; `VideoPipeline.set_rtsp_sources` schedules
        `refresh_sources()` asynchronously so the GUI thread never blocks on FFmpeg RTSP stop.
        """
        self._apply_video_settings_read_toolbar()

        def phase_configure_and_tail() -> None:
            vp = getattr(self, "_video_pipeline_shared", None)
            if vp is None:
                vp = getattr(self, "_video", None)
            try:
                self._configure_video_pipeline(vp)
            except Exception:
                pass
            try:
                self._video_split_enabled = (
                    str(getattr(self, "_video_settings_default_view", "Single") or "Single").strip().lower()
                    == "split"
                )
            except Exception:
                pass
            self._sync_native_camera_rail_toggles()
            QTimer.singleShot(
                200,
                lambda: self._companion_start_decode_if_needed(reason="settings_apply"),
            )

        def phase_stop_preview() -> None:
            # `_stop_video_preview` used to do WebEngine `runJavaScript` + FFmpeg `stop()` in one
            # slot; both can block the GUI for many seconds (wrong Wi‑Fi RTSP) → "(Not Responding)".
            if not bool(getattr(self, "_video_preview_enabled", False)):
                QTimer.singleShot(48, phase_configure_and_tail)
                return
            try:
                self._stop_video_preview_begin()
            except Exception:
                pass
            try:
                self._silence_pipeline_video_sources()
            except Exception:
                pass

            def sources_end_and_configure() -> None:
                # Do not call `_stop_video_preview_stop_sources()` here: it runs FFmpeg/Qt `stop()`
                # on the GUI thread, and `VideoPipeline.refresh_sources()` will stop the same
                # objects again — double teardown was a major source of "(Not Responding)" on Apply.
                # `_stop_video_preview_begin` already blocked `frame` emissions from those sources.
                try:
                    self._stop_video_preview_end(clear_overlay=True)
                except Exception:
                    pass
                QTimer.singleShot(48, phase_configure_and_tail)

            QTimer.singleShot(45, sources_end_and_configure)

        QTimer.singleShot(0, phase_stop_preview)

    def apply_video_settings(self) -> None:
        """Reconfigure the video pipeline from QSettings (map load, reconnect, etc.).

        For Application Settings → Video → Apply, MainWindow uses
        :meth:`apply_video_settings_for_settings_dialog` instead (staged RTSP teardown).
        """
        self._apply_video_settings_read_toolbar()

        try:
            if bool(getattr(self, "_video_preview_enabled", False)):
                self._stop_video_preview(clear_overlay=True)
        except Exception:
            pass

        vp = getattr(self, "_video_pipeline_shared", None)
        if vp is None:
            vp = getattr(self, "_video", None)
        try:
            self._configure_video_pipeline(vp)
        except Exception:
            pass

        self._video_inited = False
        self._shared_vp_hooks_connected = False
        try:
            self._video_split_enabled = (
                str(getattr(self, "_video_settings_default_view", "Single") or "Single").strip().lower()
                == "split"
            )
        except Exception:
            pass
        self._sync_native_camera_rail_toggles()

        if self._video_preview_should_run():
            self._schedule_video_preview_after_settings()

    def _schedule_video_preview_after_settings(self, *, _retry: int = 0) -> None:
        """Start preview when RTSP sources exist (now or after async refresh_sources)."""
        if not self._video_preview_should_run():
            return
        vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
        if vp is None:
            try:
                print("[VGCS:video] preview schedule: no VideoPipeline")
            except Exception:
                pass
            return
        try:
            if vp.sources():
                self._auto_start_mini_video_pip(
                    force_decode=bool(getattr(self, "_last_link_connected", False)),
                )
                if bool(getattr(self, "_last_link_connected", False)):
                    self._companion_start_decode_if_needed(reason="schedule")
                return
        except Exception:
            pass
        if int(_retry) >= 3:
            try:
                print("[VGCS:video] preview schedule: gave up (no pipeline sources after retries)")
            except Exception:
                pass
            return
        if int(_retry) == 0:
            try:
                print(
                    "[VGCS:video] preview schedule: waiting for pipeline sources "
                    "(sources_changed will start decode)"
                )
            except Exception:
                pass
        nxt = int(_retry) + 1
        QTimer.singleShot(
            1200, lambda r=nxt: self._schedule_video_preview_after_settings(_retry=r)
        )

    def _restart_video_preview_after_settings(self, *, force_decode: bool = False) -> None:
        try:
            if not self._video_preview_should_run():
                return
            if self._should_defer_companion_rtsp_decode() and not force_decode:
                try:
                    print(
                        "[VGCS:video] companion RTSP deferred until MAVLink link "
                        "(connect radio, then open video or Apply settings)"
                    )
                except Exception:
                    pass
                return
            # `refresh_sources()` replaces source objects; force full hook-up (not the stale
            # `_video_inited` fast-path) so `src.start()` targets the new `RtspSource`.
            self._video_inited = False
            self._shared_vp_hooks_connected = False
            self._auto_start_mini_video_pip(force_decode=force_decode)
        except Exception:
            pass

    def set_camera_control(self, control) -> None:
        """Inject a camera control backend (MAVLink/SDK)."""
        try:
            self._camera_control = control
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

    def _operator_preview_source_id(self) -> str:
        """
        Pipeline source id for the feed the operator is viewing (record / photo / clip).

        Split fullscreen after clicking a quadrant uses ``_split_fullscreen_source_id``; the
        pipeline ``active_source`` often stays ``day``.
        """
        focus = getattr(self, "_split_fullscreen_source_id", None)
        if focus and bool(getattr(self, "_video_swapped", False)):
            sid = str(focus).strip()
            if sid:
                return sid
        vp = getattr(self, "_video", None)
        if vp is not None:
            try:
                sid = str(vp.active_source_id() or "").strip()
                if sid:
                    return sid
            except Exception:
                pass
        src = getattr(self, "_video_active_source", None)
        if src is not None:
            return str(getattr(src, "source_id", "") or "").strip()
        return ""

    def _video_source_by_id(self, source_id: str):
        sid = str(source_id or "").strip()
        if not sid:
            return None
        vp = getattr(self, "_video", None)
        if vp is None:
            return None
        try:
            return vp.sources().get(sid)
        except Exception:
            return None

    def _operator_preview_video_source(self):
        """RtspSource (or backend) matching the on-screen preview for capture/record."""
        sid = self._operator_preview_source_id()
        if sid:
            src = self._video_source_by_id(sid)
            if src is not None:
                return src
        return getattr(self, "_video_active_source", None)

    def _video_preview_source_ids_to_run(self, vp) -> list[str]:
        """Source ids that should decode while preview is on (split = all, single = active only)."""
        try:
            sources = vp.sources()
        except Exception:
            return []
        keys = list(sources.keys())
        if not keys:
            return []
        if bool(getattr(self, "_video_split_enabled", False)):
            ordered: list[str] = []
            for k in ("day", "thermal"):
                if k in keys:
                    ordered.append(k)
            for k in keys:
                if k not in ordered:
                    ordered.append(k)
            return ordered[:4]
        active = ""
        try:
            active = str(vp.active_source_id() or "").strip()
        except Exception:
            active = ""
        if active in keys:
            return [active]
        if "day" in keys:
            return ["day"]
        return [keys[0]]

    def _start_video_decode_sources(self, vp, *, force: bool = False) -> None:
        """Start only the decoders needed for the current preview mode."""
        if self._should_defer_companion_rtsp_decode() and not force:
            try:
                print(
                    "[VGCS:video] decode start skipped: companion RTSP deferred until MAVLink link"
                )
            except Exception:
                pass
            return
        want = set(self._video_preview_source_ids_to_run(vp))
        for sid, src in vp.sources().items():
            if sid in want:
                try:
                    if force and hasattr(src, "restart_decode"):
                        src.restart_decode()
                    else:
                        src.start()
                except Exception:
                    pass

    def _stop_idle_video_decode_sources(self, vp) -> None:
        """Stop decoders that are not needed (e.g. thermal when leaving split view)."""
        want = set(self._video_preview_source_ids_to_run(vp))
        for sid, src in vp.sources().items():
            if sid not in want:
                try:
                    src.stop()
                except Exception:
                    pass

    def _video_gui_stall_recovery_enabled(self) -> bool:
        raw = str(os.environ.get("VGCS_VIDEO_GUI_STALL", "") or "").strip()
        if raw == "0":
            return False
        if raw == "1":
            return True
        # SIYI companion: off by default — killing a live FFmpeg session causes RTSP -138 freezes.
        return False

    def _on_video_preview_stall_check(self) -> None:
        """Restart decode when preview paint stalls (opt-in via VGCS_VIDEO_GUI_STALL=1)."""
        if not self._video_gui_stall_recovery_enabled():
            return
        if not bool(getattr(self, "_video_preview_enabled", False)):
            return
        if bool(getattr(self, "_video_preview_stall_recovery_active", False)):
            return
        started = float(getattr(self, "_video_preview_started_mono", 0.0) or 0.0)
        got = bool(getattr(self, "_video_preview_got_frame", False))
        # Default: long startup grace (USB / slow RTSP). Companion SIYI: once we have painted
        # frames, do NOT block stall recovery for 25s — HEVC often drops at ~10–20s and FFmpeg
        # reconnect can hang without GUI recovery.
        if self._uses_companion_rtsp():
            if not got and started > 0.0 and time.monotonic() - started < 12.0:
                return
        else:
            if started > 0.0 and time.monotonic() - started < 25.0:
                return
        if not got:
            return
        last = float(getattr(self, "_native_video_last_frame_mono", 0.0) or 0.0)
        if last <= 0.0:
            return
        default_stall = "20.0" if self._uses_companion_rtsp() else "12.0"
        try:
            stall_s = float(
                str(os.environ.get("VGCS_VIDEO_GUI_STALL_S", default_stall) or default_stall).strip()
            )
        except ValueError:
            stall_s = 20.0 if self._uses_companion_rtsp() else 12.0
        stall_s = max(6.0, min(30.0, stall_s))
        if time.monotonic() - last < stall_s:
            return
        vp = getattr(self, "_video", None)
        if vp is None:
            return
        want_ids = self._video_preview_source_ids_to_run(vp)
        if not want_ids:
            return
        # Do not tear down RTSP when FFmpeg is still decoding (GUI paint can lag HEVC gaps).
        decode_grace = stall_s + 6.0
        for sid in want_ids:
            try:
                src = vp.sources().get(sid)
                if src is not None and hasattr(src, "decode_recently_active"):
                    if src.decode_recently_active(decode_grace):
                        if not bool(getattr(self, "_video_gui_stall_skip_logged", False)):
                            self._video_gui_stall_skip_logged = True
                            try:
                                print(
                                    "[VGCS:video] GUI stall recovery skipped: "
                                    f"FFmpeg decode active (within {decode_grace:.0f}s)"
                                )
                            except Exception:
                                pass
                        return
            except Exception:
                pass
        self._video_gui_stall_skip_logged = False
        self._video_preview_stall_recovery_active = True
        try:
            print(
                f"[VGCS:video] GUI preview stall (no paint for {stall_s:.1f}s), "
                f"restarting decode for {want_ids!r}"
            )
        except Exception:
            pass
        for sid in want_ids:
            try:
                src = vp.sources().get(sid)
                if src is not None and hasattr(src, "restart_decode"):
                    src.restart_decode(delay_ms=1500)
            except Exception:
                pass

        def _restart() -> None:
            try:
                self._video_preview_started_mono = time.monotonic()
                self._native_video_last_frame_mono = 0.0
            finally:
                self._video_preview_stall_recovery_active = False

        QTimer.singleShot(500, _restart)

    def _start_video_preview(self, *, reset_swapped: bool = True, force_decode: bool = False) -> None:
        if not getattr(self, "_web_ready", False):
            try:
                print("[VGCS:video] preview start skipped: map not ready")
            except Exception:
                pass
            return
        self._show_mini_video_pip_shell()
        try:
            self._video_preview_enabled = True
        except Exception:
            pass
        if not self._ensure_video_preview_backend(from_start=True):
            try:
                vp = getattr(self, "_video_pipeline_shared", None) or getattr(self, "_video", None)
                nsrc = 0
                try:
                    nsrc = len(vp.sources()) if vp is not None else 0
                except Exception:
                    nsrc = -1
                print(
                    f"[VGCS:video] preview shell visible; decode waiting for pipeline "
                    f"(HAS_MULTIMEDIA={HAS_MULTIMEDIA} pipeline_sources={nsrc})"
                )
            except Exception:
                pass
            self._schedule_video_preview_after_settings()
            self._run_js("setVideoPreviewImage('');")
            return
        try:
            vp_hook = getattr(self, "_video", None)
            if vp_hook is not None:
                self._connect_video_pipeline_frame_slots(vp_hook)
        except Exception:
            pass
        try:
            # Always start in day preview unless explicitly toggled by operator.
            self._video_vision_mode = "day"
            if reset_swapped:
                self._video_swapped = False
                self._video_swap_user_map_main = False
                self._split_fullscreen_source_id = None
            # Keep Web layer in map mode; native side handles fullscreen camera.
            self._run_js("setVideoSwapMode(false);")
            if self._plan_flight_layer_obscures_native_camera_ui():
                self._native_video_preview.hide()
                self._layout_native_hud()
            else:
                self._native_video_preview.show()
                self._layout_native_video_preview()
                if self._native_video_last.isNull():
                    self._set_native_video_pip_placeholder(True)
            # Native minimap position follows PiP; camera rail is shown from `_on_map_loaded`.
            self._update_native_minimap()
            # Native mode: hide Web preview layer to avoid double-render fragmentation.
            self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(true);")
            self._run_js("if (window.setNativeHudMode) setNativeHudMode(true);")
            # Split: decode every registered source (day + thermal + …). Single view: only the
            # active source — avoids a second RTSP session on the same URL (SIYI ZR10 stall risk).
            vp = getattr(self, "_video", None)
            if vp is not None:
                self._start_video_decode_sources(vp, force=force_decode)
                try:
                    self._video_preview_got_frame = False
                    self._video_preview_started_mono = time.monotonic()
                    t_stall = getattr(self, "_video_preview_stall_timer", None)
                    if t_stall is not None and self._video_gui_stall_recovery_enabled():
                        t_stall.start()
                except Exception:
                    pass
                try:
                    src0 = vp.active_source()
                    if src0 is not None:
                        sid = str(getattr(src0, "source_id", "") or "")
                        dname = str(getattr(src0, "device_name", "") or sid or "video")
                        self._set_status(f"Video preview: {dname} [{sid}]")
                except Exception:
                    pass
            t_ai = getattr(self, "_ai_timer", None)
            if t_ai is not None and not t_ai.isActive():
                t_ai.start()
            self._tick_native_ai_overlay()
            if self._should_defer_companion_rtsp_decode() and not force_decode:
                self._set_status("Mini-video ready — stream starts when vehicle connects")
                return
        except Exception:
            self._run_js("setVideoPreviewImage('');")

    def _stop_video_preview_begin(self) -> None:
        """First teardown slice: drop preview flag, JS overlay hints, timers, native pixmap state."""
        self._video_preview_enabled = False
        self._video_gui_logged_frame = False
        self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(false);")
        self._run_js("if (window.setNativeHudMode) setNativeHudMode(false);")
        if hasattr(self, "_video_push_timer") and self._video_push_timer.isActive():
            self._video_push_timer.stop()
        if hasattr(self, "_ai_timer") and self._ai_timer.isActive():
            self._ai_timer.stop()
        try:
            self._native_video_overlay.clear_all()
            self._native_video_overlay.hide()
        except Exception:
            pass
        try:
            t_stall = getattr(self, "_video_preview_stall_timer", None)
            if t_stall is not None and t_stall.isActive():
                t_stall.stop()
        except Exception:
            pass
        try:
            self._native_video_preview.hide()
            self._native_minimap_wrap.hide()
            self._native_video_preview.setPixmap(QPixmap())
            self._native_video_last = QImage()
            self._native_pip_last_source_frame = QImage()
            self._video_swapped = False
            self._video_swap_user_map_main = False
            try:
                if hasattr(self, "_split_last_images"):
                    self._split_last_images.clear()
            except Exception:
                pass
            self._split_fullscreen_source_id = None
            self._split_layout_snapshot = None
            self._split_pip_hit = None
        except Exception:
            pass

    def _silence_pipeline_video_sources(self) -> None:
        """Block `frame` signals from shared pipeline sources (Apply path only — see apply_video_settings_for_settings_dialog)."""
        try:
            if getattr(self, "_video", None) is not None:
                for _, s in list(self._video.sources().items())[:4]:
                    try:
                        if hasattr(s, "blockSignals"):
                            s.blockSignals(True)
                    except Exception:
                        pass
        except Exception:
            pass

    def _stop_video_preview_stop_sources(self) -> None:
        """Stop FFmpeg/Qt video sources (can block on Windows — call from a delayed timer slot)."""
        try:
            src = getattr(self, "_video_active_source", None)
            if src is not None:
                src.stop()
            if getattr(self, "_video", None) is not None:
                for _, s in list(self._video.sources().items())[:4]:
                    try:
                        s.stop()
                    except Exception:
                        pass
        except Exception:
            pass

    def _stop_video_preview_end(self, *, clear_overlay: bool) -> None:
        """Final slice: map arrow scale + optional Web overlay reset."""
        try:
            self._sync_native_map_vehicle_arrow_scale()
        except Exception:
            pass
        if clear_overlay and getattr(self, "_web_ready", False):
            self._run_js("setVideoPreviewImage('');")
            self._run_js("setVideoPreviewMode('single');")
            self._run_js("clearAiOverlays();")

    def _stop_video_preview(self, *, clear_overlay: bool) -> None:
        self._stop_video_preview_begin()
        self._stop_video_preview_stop_sources()
        self._stop_video_preview_end(clear_overlay=clear_overlay)

    def _on_pipeline_frame(self, vf: VideoFrame) -> None:
        # Called on the GUI thread.
        if bool(getattr(self, "_video_split_enabled", False)):
            # Per-source `_on_pipeline_frame_for` owns split cache + paint (active is also in sources()).
            return
        if not bool(getattr(self, "_video_preview_enabled", False)):
            if self._uses_companion_rtsp():
                self._video_preview_enabled = True
                if not bool(getattr(self, "_video_swap_user_map_main", False)):
                    self._video_swapped = False
                try:
                    self._native_video_preview.show()
                    if bool(getattr(self, "_web_ready", False)):
                        self._native_compass.show()
                        self._native_telemetry.show()
                        if bool(getattr(self, "_last_link_connected", False)):
                            try:
                                self._obstacle_radar.show()
                            except Exception:
                                pass
                    self._layout_native_video_preview()
                    self._layout_native_hud()
                    self._stack_native_overlays_above_tile_map()
                except Exception:
                    pass
                if not bool(getattr(self, "_video_preview_recover_logged", False)):
                    self._video_preview_recover_logged = True
                    try:
                        print(
                            "[VGCS:video] preview auto-enabled on first decoded frame "
                            "(mini PiP, frame slots were late)"
                        )
                    except Exception:
                        pass
            else:
                return
        now_frame = time.monotonic()
        self._native_video_last_frame_mono = now_frame
        last_render = float(getattr(self, "_video_ui_render_mono", 0.0) or 0.0)
        render_ui = (now_frame - last_render) >= 0.04  # ~25 Hz paint cap (RTSP may be 30 Hz)
        self._video_preview_got_frame = True
        if not bool(getattr(self, "_video_gui_logged_frame", False)):
            self._video_gui_logged_frame = True
            try:
                print(
                    "[VGCS:video] GUI preview receiving frames "
                    f"(swapped={bool(getattr(self, '_video_swapped', False))})"
                )
            except Exception:
                pass
        try:
            img = vf.image
        except RuntimeError:
            return
        except Exception:
            return
        if img is None or img.isNull():
            return
        # Day/Night preview: night = grayscale.
        try:
            if str(getattr(self, "_video_vision_mode", "day") or "day").lower() == "night":
                img = img.convertToFormat(QImage.Format.Format_Grayscale8)
        except Exception:
            pass
        last_cache = float(getattr(self, "_video_cache_mono", 0.0) or 0.0)
        refresh_cache = render_ui or (now_frame - last_cache) >= 0.15
        if not refresh_cache:
            return

        img = self._apply_digital_zoom(img, float(getattr(self, "_video_zoom", 1.0)))
        try:
            img2 = img.copy()
        except Exception:
            img2 = img
        try:
            self._native_pip_last_source_frame = img2
            self._video_cache_mono = now_frame
        except Exception:
            pass

        if not render_ui:
            return

        self._video_ui_render_mono = now_frame

        try:
            self._render_native_video_preview(img2)
        except RuntimeError:
            pass

    def _on_pipeline_frame_for(self, source_id: str, vf: VideoFrame) -> None:
        if not bool(getattr(self, "_video_preview_enabled", False)):
            return
        if not bool(getattr(self, "_video_split_enabled", False)):
            return
        now_frame = time.monotonic()
        sid = str(source_id or "").strip()
        cache_times = getattr(self, "_split_cache_mono", None)
        if not isinstance(cache_times, dict):
            cache_times = {}
            self._split_cache_mono = cache_times
        last_sid = float(cache_times.get(sid, 0.0) or 0.0)
        if now_frame - last_sid < 0.04:
            return
        cache_times[sid] = now_frame
        self._native_video_last_frame_mono = now_frame
        self._video_preview_got_frame = True
        try:
            img = vf.image
        except RuntimeError:
            return
        except Exception:
            return
        if img is None or img.isNull():
            return
        try:
            if str(getattr(self, "_video_vision_mode", "day") or "day").lower() == "night":
                img = img.convertToFormat(QImage.Format.Format_Grayscale8)
        except Exception:
            pass
        img = self._apply_digital_zoom(img, float(getattr(self, "_video_zoom", 1.0)))
        try:
            img2 = img.copy()
        except Exception:
            img2 = img

        try:
            self._split_last_images[str(source_id)] = img2
        except Exception:
            pass
        try:
            self._schedule_split_preview_render()
        except RuntimeError:
            pass

    def _on_video_frame_encoded_for(self, source_id: str, data_url: str) -> None:
        self._video_last_data_urls[source_id] = str(data_url or "")
        self._video_encode_inflight_by_id[source_id] = False
        pending = self._video_encode_pending_by_id.get(source_id)
        if pending is None:
            return
        self._video_encode_pending_by_id[source_id] = None
        self._video_encode_inflight_by_id[source_id] = True
        bridge = self._video_encode_bridge_by_id.get(source_id)
        if bridge is None:
            self._video_encode_inflight_by_id[source_id] = False
            return
        task = _VideoEncodeTask(
            pending,
            bridge,
            max_w=int(getattr(self, "_video_encode_max_w", 1920)),
            max_h=int(getattr(self, "_video_encode_max_h", 1080)),
            encode_format=str(getattr(self, "_video_encode_format", "PNG")),
            encode_quality=int(getattr(self, "_video_encode_quality", 1)),
        )
        try:
            self._video_pool.start(task)
        except Exception:
            self._video_encode_inflight_by_id[source_id] = False

    def _on_video_frame_encoded(self, data_url: str) -> None:
        self._video_last_data_url = str(data_url or "")
        self._video_encode_inflight = False
        pending = getattr(self, "_video_encode_pending", None)
        if pending is None:
            return
        self._video_encode_pending = None
        self._video_encode_inflight = True
        task = _VideoEncodeTask(
            pending,
            self._video_encode_bridge,
            max_w=int(getattr(self, "_video_encode_max_w", 1920)),
            max_h=int(getattr(self, "_video_encode_max_h", 1080)),
            encode_format=str(getattr(self, "_video_encode_format", "PNG")),
            encode_quality=int(getattr(self, "_video_encode_quality", 1)),
        )
        try:
            self._video_pool.start(task)
        except Exception:
            self._video_encode_inflight = False
            return

    def _push_video_preview_any_to_overlay(self) -> None:
        if not getattr(self, "_web_ready", False):
            return
        if bool(getattr(self, "_video_split_enabled", False)) and getattr(self, "_video", None) is not None:
            keys = list(self._video.sources().keys())
            # Prefer RTSP Day/Thermal ordering when available.
            ids: list[str] = []
            for k in ("day", "thermal"):
                if k in keys:
                    ids.append(k)
            for k in keys:
                if k not in ids:
                    ids.append(k)
            ids = ids[:4]
            imgs = [str(self._video_last_data_urls.get(i, "") or "") for i in ids]
            # Ensure we always have a valid single-view fallback image when toggling split off.
            # Use the first available cell (Day, then Thermal, then others).
            try:
                first = next((s for s in imgs if str(s).strip()), "")
            except Exception:
                first = ""
            if first:
                self._video_last_data_url = str(first)
            payload = json.dumps(imgs)
            self._run_js(f"setVideoPreviewGrid({payload});")
            labels = []
            for sid in ids:
                if sid == "day":
                    labels.append("Day")
                elif sid == "thermal":
                    labels.append("Thermal")
                else:
                    labels.append(str(sid))
            self._run_js(f"setVideoPreviewLabels({json.dumps(labels)});")
            return
        src = str(getattr(self, "_video_last_data_url", "") or "")
        if not src:
            return
        last = str(getattr(self, "_last_video_pushed", "") or "")
        if src == last:
            return
        self._last_video_pushed = src
        self._run_js("setVideoPreviewMode('single');")
        self._run_js(f"setVideoPreviewImage({json.dumps(src)});")

    def _tick_native_ai_overlay(self) -> None:
        """M7 demo detection box on native video preview (replaces inactive WebEngine-only path)."""
        if not bool(getattr(self, "_video_preview_enabled", False)):
            return
        try:
            self._ai_phase = float(getattr(self, "_ai_phase", 0.0)) + 0.06
        except Exception:
            self._ai_phase = 0.0
        p = float(getattr(self, "_ai_phase", 0.0))
        x = 0.1 + (0.6 * (0.5 + 0.5 * math.sin(p)))  # 0.1..0.7
        det = [
            VideoOverlayDetection(x=x, y=0.18, w=0.22, h=0.22, label="demo", score=0.86),
        ]
        try:
            self._native_video_overlay.set_detections(det)
        except Exception:
            pass

    def _push_dummy_ai_overlay(self) -> None:
        self._tick_native_ai_overlay()

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

    def _preview_image_copy_for_snapshot(self) -> QImage | None:
        """Return a deep copy of the latest preview frame (GUI thread only, no RTSP init)."""
        sid = self._operator_preview_source_id()
        if sid:
            try:
                cache = getattr(self, "_split_last_images", None) or {}
                im = cache.get(sid)
                if isinstance(im, QImage) and not im.isNull() and im.width() > 0 and im.height() > 0:
                    return im.copy()
            except Exception:
                pass
        try:
            img = getattr(self, "_native_pip_last_source_frame", None)
            if isinstance(img, QImage) and not img.isNull() and img.width() > 0 and img.height() > 0:
                return img.copy()
        except Exception:
            pass
        try:
            cache = getattr(self, "_split_last_images", None) or {}
            ordered = [
                cache.get("day"),
                cache.get("thermal"),
                *[v for k, v in cache.items() if k not in ("day", "thermal")],
            ]
            for im in ordered:
                if isinstance(im, QImage) and not im.isNull() and im.width() > 0 and im.height() > 0:
                    return im.copy()
        except Exception:
            pass
        try:
            data_url = str(getattr(self, "_video_last_data_url", "") or "").strip()
            if data_url.startswith("data:image/") and "," in data_url:
                head, b64 = data_url.split(",", 1)
                raw = base64.b64decode(b64)
                im = QImage.fromData(raw)
                if not im.isNull():
                    return im.copy()
        except Exception:
            pass
        return None

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

    def _set_observation_mark_mode(self, enabled: bool) -> None:
        self._obs_mark_mode = bool(enabled)
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_observation_mark_mode(bool(enabled))
        except Exception:
            pass
        if not self._map_uses_legacy_web_bridge():
            pass
        else:
            self._run_js(
                f"if (window.setObservationMarkMode) setObservationMarkMode({1 if enabled else 0});"
            )
        try:
            self._btn_native_target.blockSignals(True)
            self._btn_native_target.setChecked(bool(enabled))
        finally:
            try:
                self._btn_native_target.blockSignals(False)
            except Exception:
                pass
        try:
            if enabled:
                self._native_minimap.setToolTip(
                    "Target ON: click here to mark on map · drag to pan"
                )
                self._native_minimap.setCursor(Qt.CursorShape.CrossCursor)
                self._native_video_preview.setCursor(Qt.CursorShape.CrossCursor)
            else:
                self._native_minimap.setToolTip(
                    "Drag to pan map · click (no drag) to swap back to map"
                )
                self._native_minimap.setCursor(Qt.CursorShape.OpenHandCursor)
                self._native_video_preview.setCursor(Qt.CursorShape.ArrowCursor)
        except Exception:
            pass
        if enabled:
            self._set_status(
                "Target ON: click video feed (or map) to add marks, then Report to export"
            )
        else:
            self._set_status("Observation mark mode OFF")

    def _observation_context(self) -> dict[str, object]:
        gimbal_yaw = None
        gimbal_pitch = None
        st = None
        try:
            st = self._camera_control.get_gimbal_status()
            if st is not None and bool(getattr(st, "supported", False)):
                yaw = getattr(st, "yaw_deg", None)
                pitch = getattr(st, "pitch_deg", None)
                if yaw is not None or pitch is not None:
                    if not (
                        abs(float(yaw or 0.0)) < 0.05 and abs(float(pitch or 0.0)) < 0.05
                    ):
                        gimbal_yaw = yaw
                        gimbal_pitch = pitch
        except Exception:
            pass
        v_lat = self._lat
        v_lon = self._lon
        if v_lat is None or v_lon is None:
            try:
                pos = self.get_vehicle_display_position()
                if pos is not None:
                    v_lat, v_lon = float(pos[0]), float(pos[1])
            except Exception:
                pass
        agl_m, agl_src = resolve_vehicle_agl_m(
            relative_alt_m=self._vehicle_rel_alt_m,
            rangefinder_down_m=self._rangefinder_down_m,
        )
        return {
            "vehicle_lat": v_lat,
            "vehicle_lon": v_lon,
            "vehicle_heading_deg": self._heading,
            "vehicle_roll_deg": self._vehicle_roll_deg,
            "vehicle_pitch_deg": self._vehicle_pitch_deg,
            "vehicle_rel_alt_m": agl_m,
            "agl_source": agl_src,
            "gimbal_yaw_deg": gimbal_yaw,
            "gimbal_pitch_deg": gimbal_pitch,
            "gps_fix_type": int(getattr(self, "_gps_fix_type", 0) or 0),
            "gps_satellites": int(getattr(self, "_gps_satellites", 0) or 0),
            "gps_hdop": self._gps_hdop,
            "target_lat": None,
            "target_lon": None,
            "target_alt_m": None,
            "geo_quality": "",
            "geo_warning": "",
            "geo_method": "",
            "geo_range_m": None,
            "geo_bearing_deg": None,
        }

    def _m8_geo_settings(self) -> tuple[float, str | None]:
        st = QSettings(_QS_NS, _QS_APP)
        try:
            hfov = float(st.value("observe/camera_hfov_deg", 62.0) or 62.0)
        except Exception:
            hfov = 62.0
        dem = str(st.value("observe/dem_csv", "") or "").strip() or None
        return hfov, dem

    def _enrich_observation_geo_reference(self, row: dict[str, object]) -> None:
        """M8 — compute ground lat/lon for video marks; copy map coords for map marks."""
        kind = str(row.get("kind") or "")
        if kind == "map_mark":
            row["target_lat"] = row.get("map_lat")
            row["target_lon"] = row.get("map_lon")
            row["geo_quality"] = "map_direct"
            row["geo_method"] = "map_click"
            return
        if kind != "video_mark":
            return
        vx = row.get("video_x_norm")
        vy = row.get("video_y_norm")
        if vx is None or vy is None:
            row["geo_quality"] = "insufficient"
            row["geo_warning"] = "video click missing"
            return
        hfov, dem_path = self._m8_geo_settings()
        geo = compute_geo_reference(
            vehicle_lat=row.get("vehicle_lat"),  # type: ignore[arg-type]
            vehicle_lon=row.get("vehicle_lon"),  # type: ignore[arg-type]
            vehicle_heading_deg=row.get("vehicle_heading_deg"),  # type: ignore[arg-type]
            vehicle_roll_deg=row.get("vehicle_roll_deg"),  # type: ignore[arg-type]
            vehicle_pitch_deg=row.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
            vehicle_rel_alt_m=row.get("vehicle_rel_alt_m"),  # type: ignore[arg-type]
            vehicle_alt_msl_m=self._vehicle_alt_msl_m,
            rangefinder_down_m=self._rangefinder_down_m,
            gimbal_yaw_deg=row.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
            gimbal_pitch_deg=row.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
            video_x_norm=float(vx),
            video_y_norm=float(vy),
            gps_fix_type=int(row.get("gps_fix_type") or 0),
            gps_hdop=row.get("gps_hdop"),  # type: ignore[arg-type]
            camera_hfov_deg=hfov,
            dem_path=dem_path,
        )
        row["target_lat"] = geo.target_lat
        row["target_lon"] = geo.target_lon
        row["target_alt_m"] = geo.target_alt_m
        row["geo_quality"] = geo.quality
        row["geo_warning"] = geo.warning
        row["geo_method"] = geo.method
        row["geo_range_m"] = geo.horizontal_range_m
        row["geo_bearing_deg"] = geo.bearing_deg
        row["geo_depression_deg"] = geo.depression_deg
        if geo.ok and geo.target_lat is not None and geo.target_lon is not None:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "add_geo_referenced_marker"):
                    nm.add_geo_referenced_marker(float(geo.target_lat), float(geo.target_lon))
            except Exception:
                pass

    def _log_observation(
        self,
        kind: str,
        *,
        map_lat: float | None = None,
        map_lon: float | None = None,
        video_x: float | None = None,
        video_y: float | None = None,
        clip_path: str | None = None,
        capture_snapshot: bool = True,
    ) -> None:
        """Return quickly from UI handlers; heavy snapshot I/O runs on a worker thread."""
        QTimer.singleShot(
            0,
            lambda: self._log_observation_impl(
                kind,
                map_lat=map_lat,
                map_lon=map_lon,
                video_x=video_x,
                video_y=video_y,
                clip_path=clip_path,
                capture_snapshot=capture_snapshot,
            ),
        )

    def _log_observation_impl(
        self,
        kind: str,
        *,
        map_lat: float | None = None,
        map_lon: float | None = None,
        video_x: float | None = None,
        video_y: float | None = None,
        clip_path: str | None = None,
        capture_snapshot: bool = True,
    ) -> None:
        ts = datetime.now(timezone.utc).isoformat()
        row: dict[str, object] = {
            "timestamp_utc": ts,
            "kind": str(kind),
            "map_lat": map_lat,
            "map_lon": map_lon,
            "video_x_norm": video_x,
            "video_y_norm": video_y,
            "snapshot_path": "",
            "clip_path": str(clip_path or "").strip(),
        }
        row.update(self._observation_context())
        self._enrich_observation_geo_reference(row)
        track_before = target_track_from_observations(self._observations)
        seg_m = None
        pt = observation_target_latlon(row)
        if pt is not None and track_before:
            prev_row = self._observations[-1]
            hfov, _ = self._m8_geo_settings()
            seg_m = segment_distance_between_rows(prev_row, row, hfov_deg=hfov)
            if seg_m is None:
                seg_m = haversine_m(
                    track_before[-1][0], track_before[-1][1], pt[0], pt[1]
                )
            row["segment_distance_m"] = seg_m
        else:
            row["segment_distance_m"] = None
        self._observations.append(row)
        idx = len(self._observations) - 1
        if capture_snapshot:
            self._schedule_observation_snapshot(idx)
        try:
            print(
                f"[VGCS:observe] logged {kind} count={len(self._observations)} "
                f"video=({video_x},{video_y}) map=({map_lat},{map_lon}) "
                f"geo=({row.get('target_lat')},{row.get('target_lon')}) q={row.get('geo_quality')}"
            )
        except Exception:
            pass
        msg = f"Observation logged ({len(self._observations)}): {kind}"
        if row.get("vehicle_lat") is None or row.get("vehicle_lon") is None:
            fix = int(row.get("gps_fix_type") or 0)
            sat = int(row.get("gps_satellites") or 0)
            if fix < 2:
                msg += f" — no GPS fix yet (fix={fix} sats={sat}; wait for 3D GPS / clear PreArm)"
            else:
                msg += " — GPS fix ok but position not in map state (retry mark)"
        elif row.get("gimbal_yaw_deg") is None and row.get("gimbal_pitch_deg") is None:
            msg += (
                " — gimbal N/A (Skydroid C13: TOP UDP port 5000; on RC hotspot try Host=RC gateway "
                "e.g. 192.168.43.1; ZR10: SIYI SDK UDP 37260)"
            )
        elif kind in ("video_mark", "map_mark"):
            gq = str(row.get("geo_quality") or "")
            if gq in ("good", "fair", "map_direct"):
                rng = row.get("geo_range_m")
                if rng is not None:
                    msg += f" — drone→target {float(rng):.0f} m"
                if row.get("target_lat") is not None:
                    msg += f" @ {float(row['target_lat']):.6f},{float(row['target_lon']):.6f}"
                if seg_m is not None:
                    msg += f" — targets {float(seg_m):.0f} m apart"
            elif kind == "video_mark":
                warn = str(row.get("geo_warning") or "geo insufficient")
                msg += f" — {warn}"
        self._set_status(msg)
        self._refresh_observation_measure_overlays()

        # Native OBSERVE -> Target needs a visible marker on the Qt map.
        if kind == "map_mark" and map_lat is not None and map_lon is not None:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "add_observation_map_marker"):
                    nm.add_observation_map_marker(float(map_lat), float(map_lon))
            except Exception:
                pass
        if kind == "video_mark" and video_x is not None and video_y is not None:
            try:
                self._video_obs_marks.append((float(video_x), float(video_y)))
                self._schedule_video_marks_overlay_refresh()
            except Exception:
                pass

    def _schedule_video_marks_overlay_refresh(self) -> None:
        """Coalesce overlay repaints when the operator places several Target marks quickly."""
        t = getattr(self, "_obs_marks_overlay_timer", None)
        if t is None:
            t = QTimer(self)
            t.setSingleShot(True)
            t.timeout.connect(self._flush_video_marks_overlay)
            self._obs_marks_overlay_timer = t
        t.start(32)

    def _flush_video_marks_overlay(self) -> None:
        try:
            self._native_video_overlay.set_video_marks(list(self._video_obs_marks))
            self._native_video_overlay.set_target_measure_segments(
                self._observation_video_measure_segments()
            )
        except Exception:
            pass

    def _observation_video_measure_segments(self) -> list[tuple[float, float, float, float, str]]:
        """Dashed lines on video between consecutive marks that have ground coords."""
        segs: list[tuple[float, float, float, float, str]] = []
        hfov, _ = self._m8_geo_settings()
        prev_row: dict[str, object] | None = None
        prev_xy: tuple[float, float] | None = None
        for row in self._observations:
            vx = row.get("video_x_norm")
            vy = row.get("video_y_norm")
            if vx is None or vy is None:
                continue
            if observation_target_latlon(row) is None:
                continue
            xy = (float(vx), float(vy))
            if prev_row is not None and prev_xy is not None:
                d = segment_distance_between_rows(prev_row, row, hfov_deg=hfov)
                if d is None:
                    pa = observation_target_latlon(prev_row)
                    pb = observation_target_latlon(row)
                    if pa and pb:
                        d = haversine_m(pa[0], pa[1], pb[0], pb[1])
                if d is not None:
                    pix = video_mark_span_norm(prev_xy[0], prev_xy[1], xy[0], xy[1])
                    label = format_target_segment_label(d, video_span_norm=pix)
                    segs.append((prev_xy[0], prev_xy[1], xy[0], xy[1], label))
            prev_row = row
            prev_xy = xy
        return segs

    def _refresh_observation_measure_overlays(self) -> None:
        """Sync map measure lines + video segment labels from logged observations."""
        labels: list[str] = []
        hfov, _ = self._m8_geo_settings()
        prev_row: dict[str, object] | None = None
        prev_xy: tuple[float, float] | None = None
        for row in self._observations:
            if observation_target_latlon(row) is None:
                continue
            vx = row.get("video_x_norm")
            vy = row.get("video_y_norm")
            if prev_row is not None and vx is not None and vy is not None:
                d = segment_distance_between_rows(prev_row, row, hfov_deg=hfov)
                if d is not None:
                    pix = None
                    if prev_xy is not None:
                        pix = video_mark_span_norm(
                            prev_xy[0], prev_xy[1], float(vx), float(vy)
                        )
                    labels.append(
                        format_target_segment_label(d, video_span_norm=pix)
                    )
            prev_row = row
            if vx is not None and vy is not None:
                prev_xy = (float(vx), float(vy))
        track = target_track_from_observations(self._observations)
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "set_observation_target_track"):
                nm.set_observation_target_track(track, segment_labels=labels)
        except Exception:
            pass
        self._schedule_video_marks_overlay_refresh()

    def _schedule_observation_snapshot(self, idx: int) -> None:
        """Queue JPEG write on a worker thread so Target / map clicks stay responsive."""
        if idx < 0 or idx >= len(self._observations):
            return
        img = self._preview_image_copy_for_snapshot()
        if img is None or img.isNull():
            return
        photos_dir = Path.cwd() / "captures" / "observations"
        try:
            photos_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        dest = photos_dir / f"obs_snap_{stamp}_{idx}.jpg"
        pool = getattr(self, "_video_pool", None) or QThreadPool.globalInstance()
        pool.start(_ObservationSnapshotTask(img, dest, idx, self._obs_snapshot_bridge))

    def _on_observation_snapshot_saved(self, idx: int, path: str) -> None:
        if idx < 0 or idx >= len(self._observations):
            return
        p = str(path or "").strip()
        if p:
            self._observations[idx]["snapshot_path"] = p

    def _fill_observation_snapshot(self, idx: int) -> None:
        """Backward-compatible entry: async snapshot only."""
        self._schedule_observation_snapshot(idx)

    def _observation_export_dir(self) -> Path:
        base = Path.cwd()
        try:
            if not base.exists():
                base = Path.home()
        except Exception:
            base = Path.home()
        return (base / "captures" / "observations").resolve()

    def _capture_observation_clip(self) -> None:
        if bool(getattr(self, "_obs_clip_active", False)):
            self._set_status("Observation clip already recording — please wait")
            return
        ok = self._ensure_video_preview_backend()
        if not ok:
            self._obs_clip_ui_failed(
                "Observation clip failed: video is not ready.\n\n"
                "Enable video streaming in Application Settings and confirm RTSP "
                "rtsp://192.168.144.108:554/stream=1 is playing on screen."
            )
            return
        src = self._operator_preview_video_source()
        if src is None:
            self._obs_clip_ui_failed(
                "Observation clip failed: no active video source.\n\n"
                "Wait until the live camera preview is visible, then press Clip again."
            )
            return
        clip_sid = self._operator_preview_source_id()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        clip_tag = f"_{clip_sid}" if clip_sid else ""
        ext = self._video_record_suffix()
        try:
            tmp_fd, tmp_str = tempfile.mkstemp(
                suffix=f".{ext}",
                prefix=f"vgcs_clip{clip_tag}_{stamp}_",
            )
            os.close(tmp_fd)
            out_path = Path(tmp_str)
        except Exception:
            self._obs_clip_ui_failed("Observation clip failed: cannot create temporary file.")
            return
        self._obs_clip_suggested_name = f"obs_clip{clip_tag}_{stamp}.{ext}"
        started = False
        try:
            if hasattr(src, "start_recording") and hasattr(src, "stop_recording"):
                # Surface backend errors (RTSP decode / ffmpeg missing / empty URL).
                # Best-effort: connect only once per source instance.
                try:
                    src_id = id(src)
                    if not bool(getattr(self, "_obs_clip_error_hooked_for", None)) or getattr(
                        self, "_obs_clip_error_hooked_for", None
                    ) != src_id:
                        setattr(self, "_obs_clip_error_hooked_for", src_id)
                        if hasattr(src, "error") and hasattr(src.error, "connect"):
                            src.error.connect(
                                lambda msg: self._set_status(f"Observation clip error: {msg}"),
                                Qt.ConnectionType.QueuedConnection,
                            )
                except Exception:
                    pass
                # Some backends require a running player; start() is harmless for recording backends.
                try:
                    if hasattr(src, "start"):
                        src.start()
                except Exception:
                    pass
                started = bool(src.start_recording(str(out_path)))
                if started:
                    QTimer.singleShot(8000, lambda: self._stop_observation_clip_rtsp(src, str(out_path)))
            else:
                rec = src.recorder() if hasattr(src, "recorder") else None
                if rec is not None:
                    rec.setOutputLocation(QUrl.fromLocalFile(str(out_path)))
                    rec.record()
                    started = True
                    QTimer.singleShot(8000, lambda: self._stop_observation_clip_rec(rec, str(out_path)))
        except Exception:
            started = False
        if started:
            self._obs_clip_ui_recording_started(seconds=8)
        else:
            try:
                if shutil.which("ffmpeg") is None:
                    self._obs_clip_ui_failed(
                        "Observation clip could not start: ffmpeg was not found.\n\n"
                        "Install ffmpeg, add it to PATH, restart VGCS, then press Clip again."
                    )
                    return
            except Exception:
                pass
            try:
                url = str(getattr(src, "_url", "") or "").strip()
                if not url:
                    self._obs_clip_ui_failed("Observation clip failed: RTSP URL is empty in settings.")
                    return
            except Exception:
                pass
            self._obs_clip_ui_failed(
                "Observation clip failed to start recording.\n\n"
                "Check that video is playing and see the log for RTSP/ffmpeg errors."
            )

    def _stop_observation_clip_rtsp(self, src: object, out_path: str) -> None:
        try:
            src.stop_recording()
        except Exception:
            pass
        self._finish_observation_clip(out_path)

    def _stop_observation_clip_rec(self, rec: object, out_path: str) -> None:
        try:
            rec.stop()
        except Exception:
            pass
        try:
            wait_qmedia_recorder_stopped(rec, timeout_s=25.0)
        except Exception:
            pass
        self._finish_observation_clip(out_path)

    def _finish_observation_clip(self, out_path: str) -> None:
        p = Path(str(out_path or "").strip())
        try:
            if not (p.is_file() and p.stat().st_size > 0):
                self._obs_clip_ui_failed(
                    f"Observation clip failed or file is empty: {p.name}",
                    popup=True,
                )
                return
        except Exception:
            self._obs_clip_ui_failed("Observation clip failed: could not read temp file.")
            return

        # Ask user where to save the clip (mirrors the Record button behaviour).
        ext = p.suffix.lstrip(".") or "mp4"
        suggested_name = str(getattr(self, "_obs_clip_suggested_name", None) or p.name)
        s = QSettings(_QS_NS, _QS_APP)
        last_dir = str(s.value("media/last_clip_save_dir", "") or "").strip()
        start_dir = Path(last_dir) if last_dir and Path(last_dir).is_dir() else Path.home() / "Downloads"
        if not start_dir.is_dir():
            start_dir = Path.home()
        suggested_path = str(start_dir / suggested_name)
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save observation clip",
            suggested_path,
            f"Video (*.{ext} *.mp4 *.mov *.mkv)",
        )
        if not filename:
            # User cancelled — clean up temp file silently.
            try:
                p.unlink(missing_ok=True)
            except Exception:
                pass
            self._obs_clip_ui_finished(ok=False, detail="")
            self._set_status("Observation clip save cancelled")
            return

        try:
            shutil.move(str(p), filename)
        except Exception as exc:
            self._obs_clip_ui_failed(f"Observation clip: could not move file — {exc}")
            return

        try:
            s.setValue("media/last_clip_save_dir", str(Path(filename).parent))
        except Exception:
            pass

        name = Path(filename).name
        self._log_observation("clip", clip_path=filename, capture_snapshot=True)
        self._obs_clip_ui_finished(ok=True, detail=name)
        self._set_status(f"Observation clip saved: {name} — press Report to export CSV/HTML")
        try:
            self._show_obs_clip_banner(f"Clip saved: {name}")
            QTimer.singleShot(2500, self._hide_obs_clip_banner)
        except Exception:
            pass

    def _clear_observations(self) -> None:
        if bool(getattr(self, "_obs_clip_active", False)):
            self._set_status("Cannot reset while observation clip is recording")
            return
        n = len(self._observations)
        self._observations.clear()
        self._video_obs_marks.clear()
        # Clear native markers (Qt) + web markers (if any).
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "clear_observation_marks"):
                nm.clear_observation_marks()
        except Exception:
            pass
        try:
            self._native_video_overlay.clear_all()
        except Exception:
            pass
        self._run_js("if (window.clearObservationMarks) clearObservationMarks();")
        self._refresh_observation_measure_overlays()
        self._set_status(f"Cleared observations: {n}")

    def _export_observations(self, *, quick: bool = False) -> None:
        n = len(self._observations)
        if n == 0:
            msg = (
                "No observation marks yet. Turn Target ON, then click the video (or map) "
                "to place at least one mark before Report."
            )
            self._set_status(msg)
            print(f"[VGCS:observe] export skipped: {msg}")
            if quick:
                QMessageBox.warning(self, "Observation Report", msg)
            return
        if bool(getattr(self, "_obs_export_busy", False)):
            self._set_status("Observation export already in progress…")
            return
        suggested = f"observations_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        csv_path = ""
        if quick:
            out_dir = self._observation_export_dir()
            try:
                out_dir.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                self._set_status(f"Observation export failed: {e}")
                print(f"[VGCS:observe] export failed: {e}")
                QMessageBox.warning(self, "Observation Report", f"Could not create folder:\n{e}")
                return
            csv_path = str(out_dir / suggested)
        else:
            try:
                csv_path, _ = QFileDialog.getSaveFileName(
                    self,
                    "Export observations CSV",
                    str(Path.cwd() / suggested),
                    "CSV files (*.csv)",
                )
            except Exception as e:
                self._set_status(f"Observation export dialog failed: {e}")
                return
            if not csv_path:
                return
        html_path = str(Path(csv_path).with_suffix(".html"))
        self._obs_export_busy = True
        self._obs_export_quick = bool(quick)
        self._set_status("Exporting observation report…")
        rows = [dict(r) for r in self._observations]
        pool = getattr(self, "_video_pool", None) or QThreadPool.globalInstance()
        pool.start(
            _ObservationExportTask(
                rows=rows,
                csv_path=csv_path,
                html_path=html_path,
                obs_cell_fn=self._obs_cell,
                bridge=self._obs_export_bridge,
            )
        )

    def _on_observation_export_finished(self, ok: bool, summary: str) -> None:
        self._obs_export_busy = False
        quick = bool(getattr(self, "_obs_export_quick", False))
        self._obs_export_quick = False
        if not ok:
            self._set_status(str(summary or "Observation export failed"))
            if quick:
                QMessageBox.warning(self, "Observation Report", str(summary or "Export failed"))
            return
        short = str(summary).replace("\n", " | ")
        self._set_status(short)
        print(f"[VGCS:observe] export ok {summary}")
        if quick:
            # Modal QMessageBox + shell-open while RTSP/ffmpeg run on the GUI thread freezes Windows.
            def _open_folder() -> None:
                try:
                    lines = [ln.strip() for ln in str(summary).splitlines() if ln.strip()]
                    folder = ""
                    for ln in lines[1:]:
                        p = Path(ln)
                        folder = str(p.parent if p.suffix else p)
                        break
                    if folder:
                        QDesktopServices.openUrl(QUrl.fromLocalFile(folder))
                except Exception:
                    pass

            QTimer.singleShot(400, _open_folder)

    def _obs_cell(self, val: object) -> str:
        if val is None:
            return "N/A"
        if isinstance(val, float) and (math.isnan(val) or math.isinf(val)):
            return "N/A"
        s = str(val).strip()
        return s if s else "N/A"

    def _write_observation_html_summary(self, path: str) -> None:
        rows = []
        for idx, row in enumerate(self._observations, start=1):
            rows.append(
                "<tr>"
                f"<td>{idx}</td>"
                f"<td>{row.get('timestamp_utc','')}</td>"
                f"<td>{row.get('kind','')}</td>"
                f"<td>{row.get('map_lat','')}</td>"
                f"<td>{row.get('map_lon','')}</td>"
                f"<td>{self._obs_cell(row.get('target_lat'))}</td>"
                f"<td>{self._obs_cell(row.get('target_lon'))}</td>"
                f"<td>{row.get('geo_quality','')}</td>"
                f"<td>{row.get('vehicle_lat','')}</td>"
                f"<td>{row.get('vehicle_lon','')}</td>"
                f"<td>{self._obs_cell(row.get('gimbal_yaw_deg'))}</td>"
                f"<td>{self._obs_cell(row.get('gimbal_pitch_deg'))}</td>"
                f"<td>{row.get('video_x_norm','')}</td>"
                f"<td>{row.get('video_y_norm','')}</td>"
                f"<td>{row.get('vehicle_rel_alt_m','')}</td>"
                f"<td>{row.get('geo_range_m','')}</td>"
                f"<td>{row.get('segment_distance_m','')}</td>"
                f"<td>{row.get('gps_fix_type','')}</td>"
                f"<td>{row.get('gps_satellites','')}</td>"
                f"<td>{row.get('gps_hdop','')}</td>"
                f"<td>{row.get('geo_warning','')}</td>"
                f"<td>{row.get('snapshot_path','')}</td>"
                f"<td>{row.get('clip_path','')}</td>"
                "</tr>"
            )
        html = (
            "<!doctype html><html><head><meta charset='utf-8'/>"
            "<title>Observation Summary</title>"
            "<style>body{font-family:Segoe UI,Arial,sans-serif;padding:20px;} table{border-collapse:collapse;width:100%;}"
            "th,td{border:1px solid #ccc;padding:6px;font-size:12px;} th{background:#f3f6fb;text-align:left;}</style>"
            "</head><body>"
            f"<h2>Observation Report ({len(self._observations)} entries)</h2>"
            "<table><thead><tr>"
            "<th>#</th><th>UTC Time</th><th>Kind</th><th>Map Lat</th><th>Map Lon</th>"
            "<th>Target Lat</th><th>Target Lon</th><th>Geo Quality</th>"
            "<th>Vehicle Lat</th><th>Vehicle Lon</th><th>Gimbal Yaw</th><th>Gimbal Pitch</th>"
            "<th>Video X</th><th>Video Y</th><th>Rel Alt (m)</th><th>Geo Range (m)</th>"
            "<th>Target Sep (m)</th>"
            "<th>GPS Fix</th><th>GPS Sats</th><th>HDOP</th><th>Geo Warning</th>"
            "<th>Snapshot</th><th>Clip</th>"
            "</tr></thead><tbody>"
            + "".join(rows)
            + "</tbody></table></body></html>"
        )
        Path(path).write_text(html, encoding="utf-8")

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

    def set_video_follow_enabled(self, enabled: bool) -> None:
        """Same Follow behavior as the map camera rail (center map on vehicle while on)."""
        self._on_web_title_changed(f"VGCS_CAM_FOLLOW_TOGGLE:{1 if bool(enabled) else 0}:0")
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
        except Exception:
            pass

    # Skydroid TOP: hold = continuous GSY/GSP (deg/s). Fire-and-forget UDP — no reply wait.
    _GIMBAL_HOLD_SPEED_YAW_DPS = 5.0
    _GIMBAL_HOLD_SPEED_PITCH_DPS = 5.0

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
        """Press/hold = immediate GSY/GSP; release = GSM stop (responsive, no UDP reply wait)."""

        def _start() -> None:
            self._gimbal_hold_axis = (int(dx), int(dy))
            self._native_gimbal_speed_start(dx, dy)
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
        self._native_gimbal_speed_start(axis[0], axis[1])

    def _native_gimbal_speed_start(self, dx: int, dy: int) -> None:
        cc = getattr(self, "_camera_control", None)
        if cc is None:
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
        try:
            cc.set_gimbal_speed(0.0, 0.0)
        except Exception:
            pass
        # Trigger auto-focus shortly after gimbal stops so the image sharpens immediately
        # instead of waiting for the camera's internal AF timer (can take 3-5s on ZR10).
        QTimer.singleShot(400, self._trigger_gimbal_stop_autofocus)

    def _native_gimbal_center(self) -> None:
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

    def _on_web_title_changed(self, title: str) -> None:
        if title.startswith("VGCS_MAP_TILES_READY:"):
            try:
                QTimer.singleShot(0, self._ensure_native_map_visible)
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
                    try:
                        self._seed_split_cache_from_last_frame()
                        self._layout_native_video_preview()
                        self._push_video_preview_any_to_overlay()
                        self._render_native_split_preview()
                        QTimer.singleShot(0, self._retry_native_video_pixmap)
                    except Exception:
                        pass
                else:
                    self._split_fullscreen_source_id = None
                    self._split_layout_snapshot = None
                    self._split_pip_hit = None
                    try:
                        vp0 = getattr(self, "_video", None)
                        if vp0 is not None:
                            self._stop_idle_video_decode_sources(vp0)
                    except Exception:
                        pass
                    self._run_js("if (window.setNativeVideoOverlayMode) setNativeVideoOverlayMode(true);")
                    self._run_js("if (window.setNativeHudMode) setNativeHudMode(true);")
                    # Force UI to re-render in single mode even if the underlying frame hasn't changed.
                    self._last_video_pushed = ""
                    self._run_js("clearVideoPreviewGrid();")
                    self._run_js("setVideoPreviewMode('single');")
                    # Immediately repaint a single frame (don't wait for next timer tick).
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
                cur = float(getattr(self, "_video_zoom", 1.0))
            except Exception:
                cur = 1.0
            cur += 0.25 * float(step)
            cur = max(1.0, min(4.0, cur))
            self._video_zoom = cur
            try:
                # Rail +/- : MAVLink uses ZOOM_TYPE_STEP (real payloads); Skydroid uses absolute TOP level.
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
                self._split_fullscreen_source_id = None
                self._video_swap_user_map_main = True
            else:
                self._video_swap_user_map_main = False
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

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6_371_000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(min(1.0, math.sqrt(max(0.0, a))))

    def _update_map_motion_state(self, groundspeed_mps: float) -> None:
        gs = max(0.0, float(groundspeed_mps))
        self._last_groundspeed_mps = gs
        if gs >= _MAP_MOVE_ARM_SPEED_MPS:
            self._map_speed_hi_streak = int(getattr(self, "_map_speed_hi_streak", 0)) + 1
            self._map_speed_lo_streak = 0
        elif gs < _MAP_MOVE_DISARM_SPEED_MPS:
            self._map_speed_lo_streak = int(getattr(self, "_map_speed_lo_streak", 0)) + 1
            self._map_speed_hi_streak = 0
        if not bool(getattr(self, "_map_motion_armed", False)):
            if self._map_speed_hi_streak >= _MAP_MOVE_ARM_SAMPLES:
                self._map_motion_armed = True
        elif self._map_speed_lo_streak >= _MAP_MOVE_DISARM_SAMPLES:
            was_armed = bool(getattr(self, "_map_motion_armed", False))
            self._map_motion_armed = False
            self._map_speed_hi_streak = 0
            if was_armed:
                self._apply_map_vehicle_heading()

    def _apply_map_vehicle_heading(self) -> None:
        """Push stored heading to native map / legacy 3D JS (independent of GPS motion lock)."""
        if self._heading is None:
            return
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.set_heading(self._heading)
            except Exception:
                pass
        self._schedule_vehicle_pose_js(immediate=False)

    def set_vehicle_position(
        self,
        lat: float,
        lon: float,
        *,
        relative_alt_m: float | None = None,
        groundspeed_mps: float | None = None,
    ) -> None:
        first_fix = self._lat is None or self._lon is None
        self._lat = lat
        self._lon = lon
        if groundspeed_mps is not None:
            self._update_map_motion_state(float(groundspeed_mps))
        gs = float(self._last_groundspeed_mps)
        if relative_alt_m is not None:
            try:
                self._vehicle_rel_alt_m = float(relative_alt_m)
            except Exception:
                pass
        if relative_alt_m is None:
            self._coords.setText(f"Lat/Lon: {lat:.7f}, {lon:.7f}")
        else:
            self._coords.setText(
                f"Lat/Lon: {lat:.7f}, {lon:.7f}  |  Rel Alt: {relative_alt_m:.1f} m"
            )
        raw_lat, raw_lon = float(lat), float(lon)
        map_moved = False
        append_track = False
        if first_fix:
            self._map_display_lat = raw_lat
            self._map_display_lon = raw_lon
            self._map_motion_armed = False
            self._map_speed_hi_streak = 0
            self._map_speed_lo_streak = 0
            map_moved = True
        elif not bool(getattr(self, "_map_motion_armed", False)):
            # Hard lock while parked: ignore all GPS drift until sustained real movement.
            if self._map_display_lat is None or self._map_display_lon is None:
                self._map_display_lat = raw_lat
                self._map_display_lon = raw_lon
                map_moved = True
        elif self._map_display_lat is not None and self._map_display_lon is not None:
            shift_m = self._haversine_m(
                self._map_display_lat, self._map_display_lon, raw_lat, raw_lon
            )
            if shift_m >= _MAP_POSITION_MIN_MOVE_M:
                self._map_display_lat = raw_lat
                self._map_display_lon = raw_lon
                map_moved = True
                append_track = True
        else:
            self._map_display_lat = raw_lat
            self._map_display_lon = raw_lon
            map_moved = True
        display_lat = float(self._map_display_lat if self._map_display_lat is not None else raw_lat)
        display_lon = float(self._map_display_lon if self._map_display_lon is not None else raw_lon)
        nm = getattr(self, "_native_map", None)
        if nm is not None and (map_moved or first_fix):
            try:
                nm.set_vehicle_filtered(
                    display_lat,
                    display_lon,
                    append_track=bool(append_track and not first_fix),
                )
            except Exception:
                pass
            if bool(getattr(self, "_video_follow_enabled", False)) and map_moved:
                try:
                    now = time.monotonic()
                    last = float(getattr(self, "_video_follow_last_center_mono", 0.0))
                    if first_fix or now - last >= 0.25:
                        self._video_follow_last_center_mono = now
                        nm.center_on_vehicle()
                except Exception:
                    pass
            elif first_fix:
                try:
                    nm.set_center(float(lat), float(lon))
                except Exception:
                    pass
        try:
            if self._native_minimap_wrap.isVisible():
                self._update_native_minimap()
        except Exception:
            pass
        if map_moved or first_fix:
            self._schedule_vehicle_pose_js(immediate=first_fix)

    def set_vehicle_attitude(
        self,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        *,
        yaw_deg: float | None = None,
    ) -> None:
        """M8 — body roll/pitch from MAVLink ATTITUDE (optional yaw override)."""
        if roll_deg is not None:
            try:
                self._vehicle_roll_deg = float(roll_deg)
            except Exception:
                pass
        if pitch_deg is not None:
            try:
                self._vehicle_pitch_deg = float(pitch_deg)
            except Exception:
                pass
        if yaw_deg is not None:
            self.set_vehicle_heading(float(yaw_deg), source="att")

    def set_vehicle_alt_msl(self, alt_msl_m: float | None) -> None:
        if alt_msl_m is None:
            return
        try:
            self._vehicle_alt_msl_m = float(alt_msl_m)
        except Exception:
            pass

    def set_gps_hdop(self, hdop: float | None) -> None:
        if hdop is None:
            self._gps_hdop = None
            return
        try:
            self._gps_hdop = float(hdop)
        except Exception:
            self._gps_hdop = None

    def set_vehicle_heading(self, heading_deg: float, *, source: str = "mixed") -> None:
        """Store heading for HUD/observations and rotate the map vehicle icon."""
        src = str(source or "mixed")
        self._heading = float(heading_deg) % 360.0
        self._heading_js_source = src
        self._heading_label.setText(f"Heading: {self._heading:.1f}°")
        try:
            self._native_compass.set_heading_deg(self._heading)
        except Exception:
            pass
        try:
            self._obstacle_radar.set_vehicle_heading_deg(self._heading)
        except Exception:
            pass
        self._apply_map_vehicle_heading()

    def clear_flight_track(self) -> None:
        """Clear the orange breadcrumb trail (e.g. on reconnect / disconnect)."""
        self._map_display_lat = None
        self._map_display_lon = None
        self._last_groundspeed_mps = 0.0
        self._map_motion_armed = False
        self._map_speed_hi_streak = 0
        self._map_speed_lo_streak = 0
        nm = getattr(self, "_native_map", None)
        if nm is not None:
            try:
                nm.clear_track()
            except Exception:
                pass
        self._run_js("clearFlightTrack();")

    def set_obstacle_distance(self, payload: dict) -> None:
        """M9 — forward OBSTACLE_DISTANCE (LiDAR / proximity radar bins) to the radar panel."""
        try:
            self._obstacle_radar.set_obstacle_distance(payload)
            if bool(getattr(self, "_last_link_connected", False)) and bool(
                getattr(self, "_web_ready", False)
            ):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def set_distance_sensor(self, payload: dict) -> None:
        """M9 — forward DISTANCE_SENSOR (rangefinder) to the radar panel."""
        try:
            ori = int(payload.get("orientation", 0) or 0)
            cur = payload.get("current_distance_m")
            if cur is not None and is_downward_sensor_orientation(ori):
                self._rangefinder_down_m = float(cur)
        except Exception:
            pass
        try:
            self._obstacle_radar.set_distance_sensor(payload)
            if bool(getattr(self, "_last_link_connected", False)) and bool(
                getattr(self, "_web_ready", False)
            ):
                self._obstacle_radar.show()
                QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass

    def get_obstacle_sensor_summary(self) -> tuple[str, str]:
        """Nearest obstacle + rangefinder text for dashboard telemetry panel."""
        try:
            return self._obstacle_radar.summary_text()
        except Exception:
            return "N/A", "N/A"

    def set_mission_nav_seq(self, seq: int) -> None:
        """MAVLink MISSION_CURRENT.seq: trim planned route / sync with vehicle progress."""
        self._run_js(
            f"window.__missionNavSeq = {max(0, int(seq))}; updateMissionRoutePolyline();"
        )

    def is_map_motion_armed(self) -> bool:
        return bool(getattr(self, "_map_motion_armed", False))

    def get_vehicle_display_position(self) -> tuple[float, float] | None:
        la = getattr(self, "_map_display_lat", None)
        lo = getattr(self, "_map_display_lon", None)
        if la is not None and lo is not None:
            return float(la), float(lo)
        if self._lat is not None and self._lon is not None:
            return float(self._lat), float(self._lon)
        return None

    def set_flight_telemetry(
        self,
        *,
        relative_alt_m: float,
        ground_speed_mps: float,
        vertical_speed_mps: float = 0.0,
        flight_time_text: str,
        msl_alt_m: float = 0.0,
        distance_from_home_m: float = 0.0,
    ) -> None:
        del msl_alt_m
        rel_alt_m = f"{float(relative_alt_m):.1f}"
        gs_mps = f"{float(ground_speed_mps):.1f}"
        vs_mps = f"{float(vertical_speed_mps):.1f}"
        dist_m = f"{float(distance_from_home_m):.1f}"
        ttime = str(flight_time_text)
        sig = f"{rel_alt_m}|{gs_mps}|{vs_mps}|{dist_m}|{ttime}"
        if sig == self._last_flight_telemetry_sig:
            return
        self._last_flight_telemetry_sig = sig
        try:
            self._vehicle_rel_alt_m = float(relative_alt_m)
        except Exception:
            pass
        try:
            self._native_telemetry.set_values(
                f"{rel_alt_m} m",
                f"{vs_mps} m/s",
                ttime,
                f"{dist_m} m",
                f"{gs_mps} m/s",
                f"{rel_alt_m} m",
            )
            QTimer.singleShot(0, self._layout_native_hud)
        except Exception:
            pass
        self._run_js(
            "setTelemetryOverlay("
            f"{float(relative_alt_m):.3f}, "
            f"{float(ground_speed_mps):.3f}, "
            f"{json.dumps(ttime)}, "
            f"{float(distance_from_home_m):.3f}, "
            f"{float(vertical_speed_mps):.3f}"
            ");"
        )

    def set_mission_waypoint_count(self, count: int) -> None:
        self._waypoint_count = max(0, int(count))
        self._mission.setText(f"Mission WPs: {self._waypoint_count}")

    def _enable_add_waypoint_mode(self) -> None:
        self._run_js("enableAddWaypoint();")
        self._set_status(
            "Click map to add waypoint · right-click or double-click a waypoint to remove"
        )

    def _clear_waypoints(self) -> None:
        self._run_js(
            "clearWaypoints();",
            callback=lambda _: self._after_clear_waypoints(),
        )
        self._set_status("waypoints cleared")

    def _after_clear_waypoints(self) -> None:
        self._after_waypoints_mutated()
        self.clear_plan_current_mission_path()

    def _sync_waypoint_count_from_map(self) -> None:
        # Poll lightweight count only; fetching full waypoint JSON every second can stall WebEngine on slow clients.
        self._run_js("getWaypointCount();", callback=self._on_waypoint_count)

    def _on_waypoint_count(self, count) -> None:
        try:
            c = int(count)
        except Exception:
            return
        c = max(0, c)
        if c == int(getattr(self, "_waypoint_count", 0) or 0):
            return
        # Count changed; now fetch full list once to sync model/UI.
        self._run_js("JSON.stringify(getWaypoints());", callback=self._on_waypoints_json)

    def _on_waypoints_json(self, payload: str | None) -> None:
        if not payload:
            self.set_mission_waypoint_count(0)
            self._waypoints_model = []
            self._rebuild_wp_selector()
            self.waypoints_changed.emit([])
            return
        waypoints = self._waypoints_from_map_json(payload)
        self._waypoints_model = waypoints
        self.set_mission_waypoint_count(len(waypoints))
        self._rebuild_wp_selector()
        self.waypoints_changed.emit(waypoints)

    def _after_waypoints_mutated(self) -> None:
        self._sync_waypoint_count_from_map()

    def _plan_waypoints_snapshot(self) -> list[Waypoint]:
        """Current plan waypoints for upload/save/export (native map + in-memory model).

        Avoids relying on the legacy JS ``getWaypoints()`` callback path, which can
        never fire if the bridge is not ready or the script is not recognized.
        """
        if self._waypoints_model:
            return list(self._waypoints_model)
        nm = getattr(self, "_native_map", None)
        if nm is not None and nm.waypoint_count() > 0:
            return self._waypoints_from_map_json(nm.waypoints_json())
        return []

    def _request_upload(self) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to upload")
            return
        self.mission_upload_requested.emit(wps)
        self._set_status(f"Mission upload requested ({len(wps)} WPs)")

    def _waypoints_from_map_json(self, payload: str | None) -> list[Waypoint]:
        if not payload:
            return []
        try:
            rows = json.loads(payload)
        except Exception:
            return []
        waypoints: list[Waypoint] = []
        for idx, row in enumerate(rows):
            if not (isinstance(row, list) and len(row) >= 2):
                continue
            lat = float(row[0])
            lon = float(row[1])
            alt = (
                self._waypoints_model[idx].alt_m
                if idx < len(self._waypoints_model)
                else float(self._default_alt.value())
            )
            spd = (
                float(getattr(self._waypoints_model[idx], "speed_mps", 5.0))
                if idx < len(self._waypoints_model)
                else float(self._default_speed.value())
            )
            waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt, speed_mps=spd))
        return waypoints

    def _request_download(self) -> None:
        self.mission_download_requested.emit()
        self._set_status("Mission download requested")

    @staticmethod
    def _plan_current_mission_path(settings: QSettings) -> str:
        cur = str(settings.value(_KEY_PLAN_CURRENT_MISSION_JSON, "") or "")
        if cur:
            return cur
        return str(settings.value(_KEY_PLAN_LAST_MISSION_JSON_LEGACY, "") or "")

    def save_plan_mission_json(self, *, save_as: bool) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to save")
            QMessageBox.information(
                self,
                "Plan Flight",
                "There are no waypoints to save. Add waypoints on the map first.",
            )
            return
        settings = QSettings(_QS_NS, _QS_APP)
        path = ""
        if not save_as:
            last = self._plan_current_mission_path(settings)
            if last:
                parent = Path(last).expanduser().resolve().parent
                if parent.is_dir():
                    path = last
        if not path or save_as:
            path, _ = QFileDialog.getSaveFileName(
                self,
                "Save current mission as…" if save_as else "Set current mission file…",
                self._plan_current_mission_path(settings) or "mission-waypoints.json",
                "JSON files (*.json)",
            )
            if not path:
                return
        try:
            save_waypoints_json(path, wps)
        except Exception:
            self._set_status("Save failed")
            QMessageBox.warning(self, "Plan Flight", "Could not save the mission file.")
            return
        settings.setValue(_KEY_PLAN_CURRENT_MISSION_JSON, path)
        if settings.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            settings.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)
        self._set_status(f"Current mission saved ({len(wps)} WPs)")
        QMessageBox.information(
            self,
            "Plan saved",
            f"Saved {len(wps)} waypoint(s) to:\n{path}",
        )

    def save_plan_mission_kml(self) -> None:
        wps = self._plan_waypoints_snapshot()
        if not wps:
            self._set_status("No waypoints to export")
            QMessageBox.information(
                self,
                "Plan Flight",
                "There are no waypoints to export as KML.",
            )
            return
        settings = QSettings(_QS_NS, _QS_APP)
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save mission as KML",
            str(settings.value("plan_last_mission_kml", "") or "mission-waypoints.kml"),
            "KML files (*.kml)",
        )
        if not path:
            return
        try:
            save_waypoints_kml(path, wps)
        except Exception:
            self._set_status("KML export failed")
            QMessageBox.warning(self, "Plan Flight", "Could not save the KML file.")
            return
        settings.setValue("plan_last_mission_kml", path)
        self._set_status(f"KML saved ({len(wps)} WPs)")
        QMessageBox.information(
            self,
            "Export complete",
            f"Saved {len(wps)} waypoint(s) as KML:\n{path}",
        )


    def _export_mission(self) -> None:
        settings = QSettings(_QS_NS, _QS_APP)
        last_export = str(settings.value(_KEY_TOOLBAR_EXPORT_MISSION_JSON, "") or "")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export mission to file…",
            last_export or "mission-waypoints.json",
            "JSON files (*.json)",
        )
        if not path:
            return

        waypoints = self._plan_waypoints_snapshot()
        if not waypoints:
            self._set_status("No waypoints to export")
            return
        try:
            save_waypoints_json(path, waypoints)
        except Exception:
            self._set_status("Export failed")
            return
        settings.setValue(_KEY_TOOLBAR_EXPORT_MISSION_JSON, path)
        self._set_status(f"Exported copy to file ({len(waypoints)} WPs)")

    def _import_mission(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Import mission waypoints",
            "",
            "JSON files (*.json)",
        )
        if not path:
            return
        try:
            waypoints = load_waypoints_json(path)
        except Exception:
            self._set_status("Import failed")
            return
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        js = f"setWaypoints({json.dumps(rows)});"
        self._run_js(js, callback=lambda _: self._after_waypoints_mutated())
        s = QSettings(_QS_NS, _QS_APP)
        s.setValue(_KEY_PLAN_CURRENT_MISSION_JSON, path)
        if s.contains(_KEY_PLAN_LAST_MISSION_JSON_LEGACY):
            s.remove(_KEY_PLAN_LAST_MISSION_JSON_LEGACY)
        self._set_status(f"Mission opened as current file ({len(waypoints)} WPs)")

    def set_waypoints(
        self, waypoints: list[Waypoint], *, clear_plan_current_file: bool = False
    ) -> None:
        if clear_plan_current_file:
            self.clear_plan_current_mission_path()
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        nm = getattr(self, "_native_map", None)
        if nm is not None and not bool(getattr(self, "_is_3d_mode", False)):
            nm.set_waypoint_rows(rows)
            self.set_mission_waypoint_count(len(waypoints))
            self._rebuild_wp_selector()
            self.waypoints_changed.emit(list(waypoints))
            panel = getattr(self, "_plan_flight_panel", None)
            if panel is not None:
                panel.set_waypoint_count(len(waypoints))
        else:
            self._run_js(
                f"setWaypoints({json.dumps(rows)});",
                callback=lambda _: self._after_waypoints_mutated(),
            )
        self._set_status(f"Mission loaded ({len(waypoints)} WPs)")

    def get_waypoint_meta(self) -> list[dict[str, float]]:
        """Per-waypoint meta for the Plan Flight right panel."""
        out: list[dict[str, float]] = []
        for wp in self._waypoints_model:
            out.append(
                {
                    "alt_m": float(getattr(wp, "alt_m", 20.0)),
                    "speed_mps": float(getattr(wp, "speed_mps", 5.0)),
                }
            )
        return out

    def apply_waypoint_meta(self, meta: list[object]) -> None:
        """Apply per-waypoint alt/speed edits from Plan Flight panel."""
        if not self._waypoints_model:
            return
        changed = False
        for i, row in enumerate(meta):
            if i >= len(self._waypoints_model):
                break
            if not isinstance(row, dict):
                continue
            try:
                alt_m = float(row.get("alt_m", self._waypoints_model[i].alt_m))
                spd = float(row.get("speed_mps", getattr(self._waypoints_model[i], "speed_mps", 5.0)))
            except Exception:
                continue
            alt_m = max(1.0, alt_m)
            spd = max(0.1, spd)
            if float(self._waypoints_model[i].alt_m) != alt_m:
                self._waypoints_model[i].alt_m = alt_m
                changed = True
            if float(getattr(self._waypoints_model[i], "speed_mps", 5.0)) != spd:
                setattr(self._waypoints_model[i], "speed_mps", spd)
                changed = True
        if changed:
            self.waypoints_changed.emit(list(self._waypoints_model))

    def _rebuild_wp_selector(self) -> None:
        current = self._wp_selector.currentIndex()
        self._wp_selector.blockSignals(True)
        self._wp_selector.clear()
        for idx in range(len(self._waypoints_model)):
            self._wp_selector.addItem(f"WP {idx + 1}", idx)
        self._wp_selector.blockSignals(False)
        if self._waypoints_model:
            self._wp_selector.setCurrentIndex(max(0, min(current, len(self._waypoints_model) - 1)))
            self._on_wp_selected(self._wp_selector.currentIndex())
        else:
            self._wp_alt.setValue(float(self._default_alt.value()))
            self._wp_speed.setValue(float(self._default_speed.value()))

    def _on_wp_selected(self, index: int) -> None:
        if 0 <= index < len(self._waypoints_model):
            self._wp_alt.setValue(float(self._waypoints_model[index].alt_m))
            self._wp_speed.setValue(float(getattr(self._waypoints_model[index], "speed_mps", 5.0)))

    def _apply_altitude_to_selected(self) -> None:
        idx = self._wp_selector.currentIndex()
        if idx < 0 or idx >= len(self._waypoints_model):
            self._set_status("No waypoint selected")
            return
        self._waypoints_model[idx].alt_m = float(self._wp_alt.value())
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated WP {idx + 1} altitude to {self._wp_alt.value():.1f} m")

    def _apply_altitude_to_all(self) -> None:
        if not self._waypoints_model:
            self._set_status("No waypoints available")
            return
        alt = float(self._wp_alt.value())
        for wp in self._waypoints_model:
            wp.alt_m = alt
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated all waypoint altitudes to {alt:.1f} m")

    def _apply_speed_to_selected(self) -> None:
        idx = self._wp_selector.currentIndex()
        if idx < 0 or idx >= len(self._waypoints_model):
            self._set_status("No waypoint selected")
            return
        spd = max(0.1, float(self._wp_speed.value()))
        setattr(self._waypoints_model[idx], "speed_mps", spd)
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated WP {idx + 1} speed to {spd:.1f} m/s")

    def _apply_speed_to_all(self) -> None:
        if not self._waypoints_model:
            self._set_status("No waypoints available")
            return
        spd = max(0.1, float(self._wp_speed.value()))
        for wp in self._waypoints_model:
            setattr(wp, "speed_mps", spd)
        self.waypoints_changed.emit(list(self._waypoints_model))
        self._set_status(f"Updated all waypoint speeds to {spd:.1f} m/s")

    def _toggle_3d_mode(self, enabled: bool) -> None:
        active = self.set_3d_enabled(enabled)
        if active != enabled:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(active)
            self._btn_3d.blockSignals(False)

    def _enable_fence_polygon_mode(self) -> None:
        self._run_js("enableFencePolygon();")
        self._set_status("Fence polygon mode: click map to add points")

    def _set_esri_street_tiles(self) -> None:
        self.activate_esri_street_tiles()

    def _set_osm_tiles(self) -> None:
        self.activate_osm_tiles()

    def _set_satellite_tiles(self) -> None:
        self.activate_satellite_tiles()

    def _pick_offline_tiles(self) -> None:
        root = QFileDialog.getExistingDirectory(
            self,
            "Select offline tile root (contains z/x/y.png)",
            "",
        )
        if not root:
            return
        self.activate_offline_tiles(root)

    def activate_esri_street_tiles(self) -> None:
        """Default online tiles: most compatible in locked-down client networks."""
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "esri_streets")
        except Exception:
            pass
        tmpl = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}"
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Tiles © Esri', 19);")
        self._set_status("Online tiles active (Esri Streets)")

    def activate_osm_tiles(self) -> None:
        """OSM tiles are often blocked for desktop apps (referrer policy). Keep optional."""
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "osm")
        except Exception:
            pass
        tmpl = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, '&copy; OpenStreetMap contributors', 19);")
        self._set_status("Online tiles active (OSM)")

    def activate_satellite_tiles(self) -> None:
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "sat")
        except Exception:
            pass
        tmpl = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        )
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
                try:
                    nm._warm_disk_tiles_for_viewport()
                except Exception:
                    pass
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Tiles © Esri', 19);")
        self._set_status("Satellite imagery active (Esri World Imagery)")

    def activate_offline_tiles(self, root: str) -> None:
        root = str(root or "").strip()
        if not root or not Path(root).is_dir():
            self._set_status("Offline tiles: invalid folder")
            return
        # Remember for next launch.
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_OFFLINE_TILE_ROOT, root)
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "offline")
        except Exception:
            pass
        url = QUrl.fromLocalFile(root).toString().rstrip("/")
        tmpl = f"{url}/{{z}}/{{x}}/{{y}}.png"
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None:
                nm.set_tile_source(tmpl, "", 19)
        except Exception:
            pass
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Offline tile cache', 19);")
        self._set_status("Offline tiles active")

    def _apply_geofence(self) -> None:
        def _circle_polygon_points(
            center_lat: float,
            center_lon: float,
            radius_m: float,
            *,
            segments: int = 24,
        ) -> list[list[float]]:
            import math

            r = max(10.0, float(radius_m))
            n = max(8, min(96, int(segments)))
            lat0 = math.radians(center_lat)
            lon0 = math.radians(center_lon)
            earth_r = 6_371_000.0
            ang = r / earth_r
            out: list[list[float]] = []
            for i in range(n):
                theta = 2.0 * math.pi * (i / n)
                # Destination point from (lat0,lon0) given distance/heading on a sphere.
                lat = math.asin(
                    math.sin(lat0) * math.cos(ang)
                    + math.cos(lat0) * math.sin(ang) * math.cos(theta)
                )
                lon = lon0 + math.atan2(
                    math.sin(theta) * math.sin(ang) * math.cos(lat0),
                    math.cos(ang) - math.sin(lat0) * math.sin(lat),
                )
                out.append([math.degrees(lat), math.degrees(lon)])
            return out

        def _after_fence_points(payload: str | None) -> None:
            points: list[list[float]] = []
            if payload:
                try:
                    rows = json.loads(payload)
                    for row in rows:
                        if isinstance(row, list) and len(row) >= 2:
                            points.append([float(row[0]), float(row[1])])
                except Exception:
                    points = []
            if len(points) >= 3:
                self.geofence_upload_requested.emit(
                    {
                        "points": points,
                        "alt_max_m": float(self._fence_alt_max.value()),
                        "action": float(self._fence_action.currentData() or 1.0),
                    }
                )
                self._set_status(f"Fence polygon requested ({len(points)} pts)")
                return
            if self._lat is None or self._lon is None:
                self._set_status("Fence: waiting for vehicle position")
                return
            radius = float(self._fence_radius.value())
            self._fence_radius_m = radius
            self._run_js(f"setFence({self._lat:.8f}, {self._lon:.8f}, {radius:.1f});")
            # ArduPilot "circular" fence is centered on HOME, not an arbitrary lat/lon.
            # To match what the operator sees on the map, upload a polygon approximation
            # centered on the current vehicle position.
            poly = _circle_polygon_points(self._lat, self._lon, radius, segments=28)
            self.geofence_upload_requested.emit(
                {
                    "points": poly,
                    "alt_max_m": float(self._fence_alt_max.value()),
                    "action": float(self._fence_action.currentData() or 1.0),
                }
            )
            self._set_status(f"Fence requested (circle→polygon, r={radius:.0f}m)")

        self._run_js("JSON.stringify(getFencePoints());", callback=_after_fence_points)

    def _clear_geofence(self) -> None:
        self._run_js("clearFence();")
        self.geofence_upload_requested.emit({"disable": True})
        self._set_status("Fence cleared")

    def _emit_map_3d_mode_changed(self) -> None:
        try:
            self.map_3d_mode_changed.emit()
        except Exception:
            pass

    def _ensure_web_3d_view(self) -> bool:
        """Lazily create the legacy WebEngine map used only for 3D (Cesium)."""
        if not HAS_WEBENGINE_3D:
            return False
        if getattr(self, "_web_3d_view", None) is not None:
            return True
        w = create_map_3d_web_view(self._map_stack)
        if w is None:
            return False
        try:
            html = build_leaflet_html()
            w.loadFinished.connect(self._on_web_3d_load_finished)
            w.titleChanged.connect(self._on_web_title_changed)
            w.setHtml(html, assets_base_url())
        except Exception as e:
            self._set_status(f"3D HTML build failed: {e}")
            try:
                w.deleteLater()
            except Exception:
                pass
            return False
        self._web_3d_view = w
        self._web_3d_ready = False
        self._map_stack.addWidget(w)
        return True

    # Native Qt overlays (#linkBanner, telemetry, compass, camera rail, etc.) live on top of `_map_canvas`.
    # The legacy Leaflet/Cesium HTML embeds duplicates of these inside the page — hide them when 3D
    # is active so the native overlays are the only HUD, and we just see the Cesium globe underneath.
    _HIDE_LEGACY_HTML_HUD_JS = (
        "(function(){"
        "var s=document.getElementById('vgcs_3d_hide_overlays_style');"
        "if(!s){s=document.createElement('style');s.id='vgcs_3d_hide_overlays_style';"
        "document.head.appendChild(s);}"
        "s.textContent='#linkBanner,#actionRail,#planFlightLayer,#cameraRail,"
        "#mapFooterHud,#telemetryStrip,#compass,#hdrMapModeBtn{display:none !important;}';"
        "})();"
    )

    def _inject_legacy_html_hud_hide(self) -> None:
        w3 = getattr(self, "_web_3d_view", None)
        if w3 is None:
            return
        try:
            w3.page().runJavaScript(self._HIDE_LEGACY_HTML_HUD_JS, lambda *_: None)
        except Exception:
            pass

    def _on_web_3d_load_finished(self, ok: bool) -> None:
        self._web_3d_ready = bool(ok)
        if not ok:
            self._pending_3d_activate = False
            self._set_status("3D map page failed to load (check network / WebEngine)")
            return
        self._inject_legacy_html_hud_hide()
        if getattr(self, "_pending_web_2d_fallback", False):
            self._pending_web_2d_fallback = False
            if _web_2d_fallback_allowed():
                try:
                    self._activate_web_2d_fallback()
                except Exception:
                    pass
            else:
                self._ensure_native_map_visible()
            return
        if not getattr(self, "_pending_3d_activate", False):
            self._ensure_native_map_visible()
            return
        self._pending_3d_activate = False
        try:
            self._map_stack.setCurrentIndex(1)
            self._is_3d_mode = True
            self._emit_map_3d_mode_changed()
            self._web_3d_view.page().runJavaScript(
                "set3DEnabled(true);",
                lambda res: self._on_3d_toggle_result(True, res),
            )
            try:
                QTimer.singleShot(0, self._web_3d_view.setFocus)
            except Exception:
                pass
        except Exception:
            self._is_3d_mode = False
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            self._on_3d_toggle_result(True, False)
        try:
            self._activate_startup_tile_source()
            QTimer.singleShot(1200, lambda: self._probe_current_tiles(reason="3d_startup"))
        except Exception:
            pass
        self._schedule_vehicle_pose_js(immediate=True)

    def set_3d_enabled(self, enabled: bool) -> bool:
        if not enabled:
            self._is_3d_mode = False
            self._pending_3d_activate = False
            if getattr(self, "_web_2d_fallback_active", False):
                self._web_2d_fallback_active = False
            w3 = getattr(self, "_web_3d_view", None)
            if w3 is not None and self._web_3d_ready:
                try:
                    w3.page().runJavaScript("set3DEnabled(false);", lambda *_: None)
                except Exception:
                    pass
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._set_status("2D mode active")
            self._schedule_vehicle_pose_js(immediate=True)
            try:
                self._native_compass.set_map_bearing_deg(0.0)
            except Exception:
                pass
            self._emit_map_3d_mode_changed()
            return True

        if not HAS_WEBENGINE_3D:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(False)
            self._btn_3d.blockSignals(False)
            self._set_status("3D requires Qt WebEngine (install PySide6 WebEngine)")
            return False
        if not self._web_ready:
            self._set_status("3D view unavailable: map backend not ready")
            return False
        if not self._ensure_web_3d_view():
            self._set_status("3D view could not start WebEngine")
            return False
        w3 = self._web_3d_view
        assert w3 is not None

        def _apply_3d_js() -> None:
            try:
                self._map_stack.setCurrentIndex(1)
                self._is_3d_mode = True
                self._inject_legacy_html_hud_hide()
                self._emit_map_3d_mode_changed()
                w3.page().runJavaScript(
                    "set3DEnabled(true);",
                    lambda ok: self._on_3d_toggle_result(True, ok),
                )
                try:
                    QTimer.singleShot(0, w3.setFocus)
                except Exception:
                    pass
            except Exception:
                self._is_3d_mode = False
                try:
                    self._map_stack.setCurrentIndex(0)
                except Exception:
                    pass
                self._on_3d_toggle_result(True, False)

        if self._web_3d_ready:
            self._pending_3d_activate = False
            _apply_3d_js()
            try:
                self._activate_startup_tile_source()
            except Exception:
                pass
            return True
        self._pending_3d_activate = True
        return True

    def _on_3d_toggle_result(self, requested: bool, result: object) -> None:
        active = bool(result)
        self._is_3d_mode = active
        if not active:
            try:
                self._map_stack.setCurrentIndex(0)
            except Exception:
                pass
            try:
                self._native_compass.set_map_bearing_deg(0.0)
            except Exception:
                pass
        self._btn_3d.blockSignals(True)
        self._btn_3d.setChecked(active)
        self._btn_3d.blockSignals(False)
        if requested and active:
            self._set_status("3D mode active")
        elif requested:
            self._set_status("3D mode unavailable; using 2D")
        else:
            self._set_status("2D mode active")
        self._emit_map_3d_mode_changed()

