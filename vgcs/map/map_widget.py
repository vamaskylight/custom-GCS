"""M2 map scaffold with live position API and native Qt map (slippy tiles)."""

from __future__ import annotations

import base64
from dataclasses import dataclass
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
    QDialog,
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
from vgcs.map.native_video_overlay import (
    NativeVideoOverlayLayer,
    VideoOverlayFacadeHint,
    VideoOverlayLrfLock,
    VideoOverlayMark,
    VideoOverlayOffscreenHint,
    offscreen_hint_edge_uv,
)
from vgcs.map.dooaf_setup_dialog import (
    DOOAF_PICK_GUN,
    DOOAF_PICK_TARGET,
    DooafSetupDialog,
)
from vgcs.observe.dooaf import (
    DOOAF_ROLE_GUN,
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_SURVEY,
    build_dooaf_session,
    dooaf_intended_impact_video_segment,
    assemble_observation_report_html,
    dooaf_role_display,
    format_dooaf_html_summary,
    format_dooaf_status,
    format_gimbal_pitch_direction,
    format_gimbal_yaw_direction,
    format_observation_detailed_log_html,
    latest_mark_row,
    latest_mark,
    DooafSettings,
    apply_map_pick_to_settings,
    enrich_dooaf_settings_elevation_from_dem,
    dooaf_settings_kwargs,
    merge_dooaf_settings,
    merge_setup_video_marks,
    read_dooaf_settings,
    resolved_dooaf_settings,
    write_dooaf_settings,
    write_dooaf_setup_video_mark,
    clear_dooaf_setup_video_mark,
    apply_dooaf_impact_geo_fallback,
    dooaf_export_blockers,
    refine_impact_geo_from_video_rays,
    _apply_geo_reference_to_mark_row,
    _forced_ray_geo_for_row,
)
from vgcs.observe.geo_reference import (
    apply_geo_reference_result_to_video_row,
    compute_geo_reference,
    compute_lrf_slant_geo,
    project_wgs84_to_video_norm,
)
from vgcs.observe.dooaf_flight_session import (
    DooafFacadeSession,
    build_facade_overlay_hint,
    mark_track_use_geo_in_flight,
)
from vgcs.observe.grid_reference import format_grid_reference
from vgcs.observe.target_measure import (
    band_width_partner_row,
    clear_tape_pair_override,
    measure_agl_ok,
    format_target_segment_label,
    haversine_m,
    is_downward_sensor_orientation,
    low_hover_ray_agl_m,
    marks_need_level_warning,
    ray_agl_suspect_dem_mismatch,
    sanitize_dem_ground_agl_m,
    marks_same_height_band,
    MARKS_NOT_LEVEL_HINT,
    observation_facade_video_segments,
    observation_building_height_segments,
    observation_target_latlon,
    resolve_vehicle_agl_m,
    segment_distance_between_rows,
    segment_distance_video_fallback,
    session_facade_reference_range_m,
    session_peak_geo_range_m,
    session_rangefinder_reference_m,
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
    notify_companion_preview_motion,
    notify_companion_app_background,
    notify_companion_app_foreground,
    notify_companion_lrf_lock,
    notify_companion_feed_switch,
    release_all_companion_rtsp_hosts,
    release_companion_rtsp_host,
    set_companion_decode_gate,
    suggested_photo_save_path,
    suggested_recording_save_path,
    wait_qmedia_recorder_stopped,
)
from vgcs.video.camera_control import (
    NoopCameraControl,
    camera_preview_applies_digital_zoom,
    camera_recording_applies_digital_zoom,
    camera_zoom_limits,
)
from vgcs.skydroid.protocol import format_slr_display_m
from vgcs.map.native_tile_map import NativeTileMapView, bundled_seed_root, fetch_tile_http_bytes
from vgcs.map.legacy_leaflet_build import build_leaflet_html
from vgcs.map.map_footer_hud import (
    MapFooterCompass,
    MapFooterTelemetryStrip,
    MapZoomControlPanel,
    TelemetryStripIcon,
)
from vgcs.map.sensor_obstacle_widget import ObstacleRadarPanel
from vgcs.map.map_3d_marker_overlay import Map3dLayer, Map3dMarkerOverlay
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D, assets_base_url, create_map_3d_web_view
from vgcs.map.cam_rail_widgets import (
    CamObserveBlock,
    CamRailGimbalPad,
    CamRailShowHandle,
    CamRecordTimerRow,
)
from vgcs.map.plan_flight_panel import PlanFlightPanel
from vgcs.map.surface.constants import (
    _CAM_RAIL_GIMBAL_GRID_GAP,
    _CAM_RAIL_LAYER_INSET,
    _CAM_RAIL_LAYOUT_SPACING,
    _CAM_RAIL_LENS_BTN_H,
    _CAM_RAIL_LENS_ROW_H,
    _CAM_RAIL_PAD_BTN_H,
    _CAM_RAIL_PAD_BTN_W,
    _MAP_HUD_GLASS_BG,
    _MAP_HUD_GLASS_BORDER,
    _MINI_VIDEO_PIP_H_PX,
    _MINI_VIDEO_PIP_W_PX,
    _NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX,
    _NATIVE_CAM_RAIL_MIN_WIDTH_PX,
    _NATIVE_CAM_RAIL_TOP_PX,
    MAP_BACKEND_BUILD,
)

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

# Primary 2D map: NativeTileMapView only. Optional 3D globe: lazy Qt WebEngine + Cesium (see map_web_3d).
HAS_WEBENGINE = HAS_WEBENGINE_3D
# Map icon/track only move after sustained GPS speed (avoids ground jitter + false vx/vy).



# Legacy placeholder (HTML lives in legacy_leaflet_map.html + legacy_leaflet_build).
LEAFLET_HTML = ""


def _save_qimage_to_path(img: QImage, path: Path) -> bool:
    """Compatibility wrapper — see :func:`vgcs.map.image_io.save_qimage_to_path`."""
    from vgcs.map.image_io import save_qimage_to_path

    return save_qimage_to_path(img, path)


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
QPushButton#camSplitBtn, QPushButton#camFollowBtn, QPushButton#camThermalBtn {
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
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 11px;
  font-weight: 700;
}
QPushButton#camSplitBtn:hover, QPushButton#camFollowBtn:hover, QPushButton#camThermalBtn:hover {
  background-color: rgba(40, 48, 62, 245);
  border-color: rgba(229, 237, 251, 85);
}
QPushButton#camSplitBtn:checked, QPushButton#camFollowBtn:checked, QPushButton#camThermalBtn:checked {
  border: 1px solid rgba(105, 232, 111, 220);
  background-color: rgba(24, 52, 34, 250);
  color: #c8ffc8;
}
QPushButton#camThermalBtn:disabled {
  opacity: 0.45;
}
/* Split is logically on but main canvas is a single zoomed channel (not the 2×2 composite): neutral chrome. */
QPushButton#camSplitBtn:checked[splitHidden="true"] {
  border: 1px solid rgba(196, 209, 230, 55);
  background-color: rgba(22, 27, 38, 235);
  color: #e8edf8;
}
QLabel#camMagnificationLabel {
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 14px;
  font-weight: 600;
  padding: 4px 8px;
  margin-left: 2px;
  border-radius: 6px;
  border: 1px solid rgba(80, 92, 118, 107);
  background: rgba(26, 33, 45, 215);
  min-width: 44px;
}
QPushButton#camRailHideBtn {
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
  font-size: 18px;
  font-weight: 600;
}
QPushButton#camRailHideBtn:hover {
  background-color: rgba(40, 48, 62, 245);
  border-color: rgba(229, 237, 251, 85);
  color: #ffffff;
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
QPushButton#observeDooafSetup {
  min-height: 34px;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 14px;
  font-weight: 600;
  border-radius: 6px;
  border: 1px solid rgba(120, 168, 230, 90);
  background: rgba(36, 58, 92, 210);
  color: #e8f0ff;
  padding: 2px 6px;
}
QPushButton#observeDooafSetup:hover {
  background: rgba(52, 82, 128, 235);
  border-color: rgba(180, 210, 255, 120);
}
QLabel#observeDooafHint {
  color: #8fa4c4;
  font-size: 11px;
  font-weight: 500;
  line-height: 1.25;
}
QPushButton#observeClip[recording="true"] {
  background: rgba(200, 45, 45, 240);
  border-color: rgba(255, 130, 130, 220);
  color: #ffffff;
  font-weight: 700;
}
QPushButton[camPadBtn=true] {
  min-width: 42px;
  max-width: 42px;
  min-height: 34px;
  max-height: 34px;
  border-radius: 6px;
  border: 1px solid rgba(196, 209, 230, 38);
  background: rgba(18, 22, 32, 75);
  color: #dce5f5;
  font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
  font-size: 14px;
  font-weight: 600;
  padding: 0px;
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


def _cam_rail_gimbal_row(label: str, pad: QWidget) -> QWidget:
    """Gimbal pad needs explicit width; keep inset from the glass panel's right edge."""
    w = QWidget()
    w.setObjectName("camInlineRow")
    w.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 2, 2, 2)
    h.setSpacing(8)
    lab = QLabel(label)
    lab.setObjectName("camSectionHeaderInline")
    lab.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
    pad.setObjectName("camInlineRowBody")
    pad.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    h.addWidget(lab, 0, Qt.AlignmentFlag.AlignVCenter)
    h.addWidget(pad, 0, Qt.AlignmentFlag.AlignVCenter)
    h.addStretch(1)
    return w


def _cam_rail_inline_row(
    label: str,
    body: QWidget,
    *,
    align_body_right: bool = False,
) -> QWidget:
    """Compact row: section label + controls (one line, no extra header row)."""
    w = QWidget()
    w.setObjectName("camInlineRow")
    w.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 2, 2 if align_body_right else 0, 2)
    h.setSpacing(8)
    lab = QLabel(label)
    lab.setObjectName("camSectionHeaderInline")
    lab.setAlignment(
        Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
    )
    body.setObjectName("camInlineRowBody")
    if align_body_right:
        body.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        h.addWidget(lab, 0, Qt.AlignmentFlag.AlignVCenter)
        h.addStretch(1)
        h.addWidget(body, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    else:
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
    btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
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






from vgcs.map.observation import MapObservationMixins
from vgcs.map.video import MapVideoMixins
from vgcs.map.surface import MapSurfaceMixins
from vgcs.map.surface.settings_keys import _KEY_MAP_LOW_SPEC_MODE, _KEY_MAP_WEBCAM_ENABLED
from vgcs.map.video.helpers import _format_video_zoom_label
from vgcs.map.surface.tile_probe import _TileProbeBridge, _TileProbeTask
from vgcs.map.observation.types import (
    LrfLockBridge,
    LrfLockTask,
    ObservationExportBridge,
    ObservationSnapshotBridge,
    PendingLrfVideoPick,
    PendingLrfVideoPick as _PendingLrfVideoPick,
)

class MapWidget(MapObservationMixins, MapVideoMixins, MapSurfaceMixins, QWidget):
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
        set_companion_decode_gate(self._companion_video_decode_gate)
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
        self._map_3d_layer = None
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
        self._lrf_lock_armed = False
        self._lrf_lock_uv: tuple[float, float] | None = None
        self._lrf_click_uv: tuple[float, float] | None = None
        self._lrf_click_att: tuple[float, float] | None = None
        self._lrf_lock_distance_m: float | None = None
        self._lrf_lock_in_progress = False
        self._lrf_lock_failed = False
        self._lrf_lock_start_vehicle_lat: float | None = None
        self._lrf_lock_start_vehicle_lon: float | None = None
        self._lrf_track_ref_uv: tuple[float, float] | None = None
        self._lrf_track_ref_att: tuple[float, float] | None = None
        self._lrf_track_gac_h_scale = 1.0
        self._lrf_track_gac_v_scale = 1.0
        self._lrf_lock_lat: float | None = None
        self._lrf_lock_lon: float | None = None
        self._lrf_lock_alt_m: float | None = None
        self._lrf_lock_geo_label: str = ""
        self._lrf_lock_bridge = LrfLockBridge(self)
        self._lrf_lock_bridge.finished.connect(self._on_c13_lrf_lock_finished)
        self._lrf_lock_bridge.progress.connect(self._on_c13_lrf_lock_progress)
        self._pending_lrf_video_pick: _PendingLrfVideoPick | None = None
        self._obs_mark_mode = False
        self._observations: list[dict[str, object]] = []
        self._dooaf_pick_complete = None
        self._dooaf_pick_dialog: DooafSetupDialog | None = None
        self._dooaf_pick_from_video = False
        self._dooaf_pick_role: str = ""
        self._dooaf_setup_video_marks: dict[str, tuple[float, float]] = {}
        self._dooaf_setup_mark_track: dict[str, dict[str, object]] = {}
        self._dooaf_facade_session = DooafFacadeSession()
        self._dooaf_restore_target_after_pick = False
        self._video_obs_marks: list[tuple[float, float]] = []
        self._obs_snapshot_bridge = ObservationSnapshotBridge(self)
        self._obs_snapshot_bridge.finished.connect(self._on_observation_snapshot_saved)
        self._obs_export_bridge = ObservationExportBridge(self)
        self._obs_export_bridge.finished.connect(self._on_observation_export_finished)
        self._obs_export_busy = False
        self._obs_export_quick = False
        self._obs_marks_overlay_timer: QTimer | None = None
        self._video_mark_track_timer: QTimer | None = None
        self._video_ui_render_mono = 0.0
        self._split_ui_render_mono = 0.0
        self._split_cache_mono: dict[str, float] = {}
        self._split_render_timer: QTimer | None = None
        self._video_cache_mono = 0.0
        self._obs_clip_active = False
        self._obs_clip_secs_left = 0
        self._obs_clip_countdown_timer: QTimer | None = None
        self._obs_clip_banner: QLabel | None = None
        self._payload_hardware_recording = False
        self._vehicle_rel_alt_m: float | None = None
        self._rangefinder_down_m: float | None = None
        self._companion_laser_range_m: float | None = None
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
        # Split PiP → fullscreen: which source fills the canvas (None = auto primary stream).
        self._split_fullscreen_source_id: str | None = None
        self._minimap_refresh_mono: float = 0.0
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
        self._native_hud_right_layout.setContentsMargins(8, 7, 16, 8)
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

        self._btn_native_thermal = QPushButton("IR")
        self._btn_native_thermal.setObjectName("camThermalBtn")
        self._btn_native_thermal.setCheckable(True)
        self._btn_native_thermal.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_thermal.setFixedSize(28, 28)
        self._btn_native_thermal.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self._btn_native_thermal.setToolTip(
            "Thermal IR camera (C13: one RTSP stream — switches between day and thermal; "
            "gimbal control unchanged)"
        )
        self._btn_native_thermal.setText("IR")
        self._btn_native_thermal.hide()

        ctr_layout.addWidget(self._btn_native_split, 0)
        ctr_layout.addWidget(self._btn_native_thermal, 0)
        ctr_layout.addWidget(self._btn_native_follow, 0)
        self._lbl_camera_top_zoom = QLabel(_format_video_zoom_label(1.0))
        self._lbl_camera_top_zoom.setObjectName("camMagnificationLabel")
        self._lbl_camera_top_zoom.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_camera_top_zoom.setToolTip("Camera magnification")
        ctr_layout.addWidget(self._lbl_camera_top_zoom, 0, Qt.AlignmentFlag.AlignVCenter)
        ctr_layout.addStretch(1)
        self._btn_native_rail_hide = QPushButton("›")
        self._btn_native_rail_hide.setObjectName("camRailHideBtn")
        self._btn_native_rail_hide.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_rail_hide.setFixedSize(28, 28)
        self._btn_native_rail_hide.setToolTip("Hide camera panel")
        self._btn_native_rail_hide.clicked.connect(
            lambda: self._set_camera_rail_panel_visible(False)
        )
        ctr_layout.addWidget(self._btn_native_rail_hide, 0, Qt.AlignmentFlag.AlignVCenter)

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
            btn_width=_CAM_RAIL_PAD_BTN_W,
            grid_gap=_CAM_RAIL_GIMBAL_GRID_GAP,
        )
        observe_body = CamObserveBlock(
            self._btn_native_target,
            self._btn_native_clip,
            self._btn_native_report,
            self._btn_native_reset,
        )
        self._native_observe_body = observe_body

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
            _cam_rail_gimbal_row("GIMBAL", gimbal_pad)
        )
        self._native_hud_right_layout.addWidget(
            _cam_rail_inline_row("OBSERVE", observe_body)
        )

        self._native_hud_right.setMinimumWidth(_NATIVE_CAM_RAIL_CONTENT_MIN_WIDTH_PX)
        self._native_hud_right.hide()
        self._native_rail_layer.hide()

        self._btn_camera_rail_show = CamRailShowHandle(
            self._panel,
            icon=_git_cam_icon_from_svg(_GIT_CAM_VIDEO_SVG, 22),
        )
        self._btn_camera_rail_show.clicked.connect(
            lambda: self._set_camera_rail_panel_visible(True)
        )
        self._btn_camera_rail_show.hide()
        self._btn_camera_rail_show.raise_()

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

        self._native_map_zoom_ctrl = MapZoomControlPanel(self._panel)
        self._native_map_zoom_ctrl.zoom_step_requested.connect(self._on_main_map_zoom_step)
        self._native_map_zoom_ctrl.hide()
        self._native_map_zoom_ctrl.raise_()

        # M9 — obstacle radar on `_panel` (same stacking layer as PiP / compass).
        self._obstacle_radar = ObstacleRadarPanel(self._panel)
        self._obstacle_radar.c13_lrf_lock_clicked.connect(self._on_c13_lrf_icon_clicked)
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
        self._btn_native_thermal.toggled.connect(self._on_native_thermal_feed_toggled)
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
            self._reset_c13_lrf_for_observe_reset()
            self._clear_observations()

        def _obs_dooaf_setup() -> None:
            print("[VGCS:cam_rail] OBSERVE DOOAF Setup clicked")
            QTimer.singleShot(0, self._show_dooaf_setup_dialog)

        self._btn_native_target.toggled.connect(_obs_target)
        self._btn_native_clip.clicked.connect(_obs_clip)
        self._btn_native_report.clicked.connect(_obs_report)
        self._btn_native_reset.clicked.connect(_obs_reset)
        observe_body.setup_clicked.connect(_obs_dooaf_setup)

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
