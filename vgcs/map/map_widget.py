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
    QCheckBox,
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
    QSpinBox,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
    QSizePolicy,
)
from PySide6.QtGui import (
    QDesktopServices,
    QImage,
    QKeySequence,
    QPainterPath,
    QPen,
    QPolygonF,
    QShortcut,
)
from vgcs.app import ui_theme
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
    DEFAULT_MISSION_END_ACTION,
    MAX_WP_HOVER_S,
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
)
from vgcs.map.sensor_obstacle_widget import ObstacleRadarPanel
from vgcs.map.map_3d_marker_overlay import Map3dLayer, Map3dMarkerOverlay
from vgcs.map.map_web_3d import HAS_WEBENGINE as HAS_WEBENGINE_3D, assets_base_url, create_map_3d_web_view
from vgcs.map.cam_rail_widgets import (
    CamM13TrackBlock,
    CamObserveBlock,
    CamRailShowHandle,
)
from vgcs.map.plan_flight_panel import PlanFlightPanel
from vgcs.map.surface.constants import (
    _MINI_VIDEO_PIP_H_PX,
    _MINI_VIDEO_PIP_W_PX,
    _NATIVE_CAM_RAIL_TOP_PX,
    MAP_BACKEND_BUILD,
)

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


# The camera panels' sheet (vgcs/app/ui_theme.py, the 2026-11-09 restyle). One sheet on the panels'
# layer styles the bar, the gimbal pad, the Observe panel and the text buttons in them.
_NATIVE_CAMERA_RAIL_QSS = ui_theme.camera_panels_qss()


from vgcs.map.observation import MapObservationMixins
from vgcs.map.video import MapVideoMixins
from vgcs.map.surface import MapSurfaceMixins
from vgcs.map.surface.settings_keys import _KEY_MAP_LOW_SPEC_MODE, _KEY_MAP_WEBCAM_ENABLED
from vgcs.map.video.helpers import _format_video_zoom_label
from vgcs.map.surface.tile_probe import _TileProbeBridge, _TileProbeTask
from vgcs.map.observation.types import (
    LrfLockBridge,
    LrfLockTask,
    M13RangeBridge,
    M13TrackBridge,
    M14DetectBridge,
    M14FollowBridge,
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
    land_requested = Signal()
    disarm_requested = Signal()
    follow_object_requested = Signal()  # Follow Me: the TRACK block's Follow button (the main window asks first)
    # The range of the PROXIMITY card's four sides, chosen on the card (B19, the window saves it).
    obstacle_bar_range_chosen = Signal(float)
    plan_tool_requested = Signal(str)
    plan_action_requested = Signal(str)
    plan_flight_exited = Signal()
    map_page_ready = Signal()
    toggle_3d_requested = Signal()
    map_3d_mode_changed = Signal()  # _is_3d_mode updated (async load / JS / back to 2D)
    mission_start_requested = Signal()
    mission_pause_requested = Signal()
    mission_resume_requested = Signal()
    mission_jump_requested = Signal(int)  # 0-based waypoint to fly to now
    plan_mission_panel_changed = Signal(object)
    plan_waypoint_selection_changed = Signal(int)  # selected WP index, -1 for none
    video_follow_enabled_changed = Signal(bool)
    # Offline map tile packs: progress, and the result (also of an import).
    tile_pack_progress = Signal(str)
    tile_pack_finished = Signal(str)
    # A one-off result for the operator (a recording, a plan file). The map's own
    # status line is hidden in the map-first window: the main window shows these.
    operator_notice = Signal(str)

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
        # What the vehicle does after the last waypoint (hold / rtl / land).
        self._mission_end_action = DEFAULT_MISSION_END_ACTION
        # Live AUTO progress, as reported by the vehicle (see mission_progress).
        self._mission_progress_wp_index: int | None = None
        self._web_ready = False
        self._is_3d_mode = False
        self._web_3d_view = None
        self._map_3d_layer = None
        self._web_3d_ready = False
        self._pending_3d_activate = False
        self._fence_radius_m = 80.0
        # What the link has reported about the fence of the drone on screen
        # (vgcs/map/surface/fence_mixin.py). Empty: nothing is known.
        self._fence_drone_state: dict = {}
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
        self._dooaf_video_pick_mode: str = ""
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
        self._m13_track_armed = False
        self._m13_track_active = False
        self._m13_track_click_uv: tuple[float, float] | None = None
        self._m13_track_ref_att: tuple[float, float] | None = None
        self._m13_track_lat: float | None = None
        self._m13_track_lon: float | None = None
        self._m13_track_alt_m: float | None = None
        self._m13_track_geo_label = ""
        self._m13_track_range_m: float | None = None
        self._m13_track_range_att: tuple[float, float] | None = None
        self._m13_track_geo_uncertain = False
        self._m13_track_path: list[tuple[float, float]] = []
        self._m13_track_geo_mono = 0.0
        self._m13_track_timer: QTimer | None = None
        self._m13_track_bridge = M13TrackBridge(self)
        self._m13_track_bridge.started.connect(self._on_m13_track_started)
        self._m13_range_bridge = M13RangeBridge(self)
        self._m13_range_bridge.ready.connect(self._on_m13_range_ready)
        self._m13_range_task_inflight = False
        self._m14_follow_bridge = M14FollowBridge(self)
        self._m14_follow_bridge.updated.connect(self._on_m14_follow_updated)
        self._m14_tracker = None
        self._m14_tracker_active = False
        self._m14_follow_task_inflight = False
        self._m14_follow_lost_streak = 0
        self._m14_detect_bridge = M14DetectBridge(self)
        self._m14_detect_bridge.detected.connect(self._on_m14_detect_result)
        self._m14_detect_task_inflight = False
        self._m14_detect_generation = 0
        # Dedicated bridge for sizing the M13/M14 click-to-track box from a
        # real object detection — separate from _m14_detect_bridge (the
        # on-demand "Detect" button) so the two async flows never cross-talk.
        self._m14_track_init_detect_bridge = M14DetectBridge(self)
        self._m14_track_init_detect_bridge.detected.connect(self._on_m14_track_init_detect_result)
        self._m14_track_init_pending_gen = None
        self._m14_track_init_frame_bgr = None
        self._m14_track_init_click_uv = None
        self._m14_threat_zone_streak = 0
        self._m14_threat_zone_reported = False
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
            "border: 1px solid rgba(255, 255, 255, 110);"
            "border-radius: 10px;"
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
        # The camera panels, like the app's (2026-11-09 restyle): a slim camera bar on the right edge
        # (video or photo, the shutter, the camera's own switches, Track and Follow, Observe), a pad for
        # the gimbal and the lens beside it, and the Observe panel (Target, Clip, Report, Reset, Set TGT
        # and Set HIT, the laser switch, DOOAF Setup), which opens from the bar and stays open until it
        # is closed. Until then all of it was one wide box of small text buttons. Every button keeps its
        # name, its signals and its hold logic; only where it sits and how it looks changed.
        #
        # The panels are children of one transparent layer, a **sibling** of `_map_canvas` under the
        # map panel (not a child of `_map_canvas`): PiP `QLabel` + tile map live inside `_map_canvas`;
        # keeping the panels on the same surface made `setPixmap`/stacking reorder the PiP above them on
        # some platforms (photo / rail unclickable). Hiding the layer hides every panel. Its mask (set in
        # `_layout_native_hud`) is the panels themselves, so a click between them reaches the map.
        self._native_rail_layer = QFrame(panel)
        self._native_rail_layer.setObjectName("nativeCameraRailLayer")
        self._native_rail_layer.setFrameShape(QFrame.Shape.NoFrame)
        self._native_rail_layer.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._native_rail_layer.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_rail_layer.setAutoFillBackground(False)
        self._native_rail_layer.setStyleSheet(_NATIVE_CAMERA_RAIL_QSS)
        self._native_rail_layer.hide()
        _cam_small = ui_theme.CAMERA_SMALL_BUTTON
        _cam_btn = ui_theme.CAMERA_BUTTON
        _cam_m = ui_theme.RAIL_MARGIN

        # --- The camera bar -------------------------------------------------------------------------
        self._native_hud_right = QFrame(self._native_rail_layer)
        self._native_hud_right.setObjectName("nativeCameraRail")
        self._native_hud_right.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_hud_right.setAutoFillBackground(False)
        self._native_hud_right_layout = QVBoxLayout(self._native_hud_right)
        self._native_hud_right_layout.setContentsMargins(_cam_m, _cam_m + 2, _cam_m, _cam_m + 2)
        self._native_hud_right_layout.setSpacing(ui_theme.RAIL_SPACING)

        mode_row = QWidget(self._native_hud_right)
        mode_row.setObjectName("cameraModeRow")
        mode_lay = QHBoxLayout(mode_row)
        mode_lay.setContentsMargins(0, 0, 0, 0)
        mode_lay.setSpacing(6)
        self._btn_native_video = QPushButton(mode_row)
        self._btn_native_video.setObjectName("camVideoBtn")
        self._btn_native_video.setCheckable(True)
        self._btn_native_video.setChecked(True)
        self._btn_native_video.setToolTip("Video / live mode — red button records")
        ui_theme.style_round_button(self._btn_native_video, icon_name="vama_record", size=_cam_small, tone="info")
        self._btn_native_photo = QPushButton(mode_row)
        self._btn_native_photo.setObjectName("camPhotoBtn")
        self._btn_native_photo.setCheckable(True)
        self._btn_native_photo.setToolTip("Photo mode — use the red shutter below to capture")
        ui_theme.style_round_button(self._btn_native_photo, icon_name="vama_photo", size=_cam_small, tone="info")
        mode_lay.addWidget(self._btn_native_video)
        mode_lay.addWidget(self._btn_native_photo)

        # The shutter: records in video mode (red while it records), takes a photo in photo mode.
        self._btn_native_record = QPushButton(self._native_hud_right)
        self._btn_native_record.setObjectName("camRecordBtn")
        self._btn_native_record.setCheckable(True)
        ui_theme.style_round_button(self._btn_native_record, size=ui_theme.SHUTTER_SIZE, tone="record")
        self._btn_native_record.setText("")
        self._btn_native_record.setIcon(ui_theme.record_icon(20))
        self._btn_native_record.setIconSize(QSize(20, 20))
        self._btn_native_record.setToolTip("Record video")

        self._lbl_native_cam_timer = QLabel("00:00:00", self._native_hud_right)
        self._lbl_native_cam_timer.setObjectName("camTimer")
        self._lbl_native_cam_timer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_native_cam_timer.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        # The camera's switches, two to a row. A row whose buttons are all hidden takes no room.
        self._camera_top_row = QFrame(self._native_hud_right)
        self._camera_top_row.setObjectName("cameraTopRow")
        ctr_layout = QGridLayout(self._camera_top_row)
        ctr_layout.setContentsMargins(0, 0, 0, 0)
        ctr_layout.setHorizontalSpacing(6)
        ctr_layout.setVerticalSpacing(ui_theme.RAIL_SPACING)

        self._btn_native_split = ui_theme.round_button(
            self._camera_top_row, object_name="camSplitBtn", icon_name="split", size=_cam_small,
            tooltip="Split view (4-up)", checkable=True, tone="info",
        )
        self._btn_native_follow = ui_theme.round_button(
            self._camera_top_row, object_name="camFollowBtn", icon_name="follow_map", size=_cam_small,
            tooltip="Follow vehicle (center map)", checkable=True, tone="info",
        )
        self._btn_native_thermal = ui_theme.round_button(
            self._camera_top_row, object_name="camThermalBtn", icon_name="vama_ir", size=_cam_small,
            tooltip=(
                "Thermal IR camera (C13: one RTSP stream — switches between day and thermal; "
                "gimbal control unchanged)"
            ),
            checkable=True,
        )
        self._btn_native_thermal.hide()
        # Thermal colour mode (M12): a strip of the mode in use; a click lists the others.
        self._btn_native_palette = ui_theme.round_button(
            self._camera_top_row, object_name="camPaletteBtn", size=_cam_small,
        )
        self._btn_native_palette.hide()
        # The camera's torch (row B27): only for a camera with one (_sync_native_torch_button).
        self._btn_native_torch = ui_theme.round_button(
            self._camera_top_row, object_name="camTorchBtn", icon_name="vama_torch", size=_cam_small,
            tooltip="The camera's torch, on or off", checkable=True, tone="torch",
        )
        self._btn_native_torch.hide()
        ctr_layout.addWidget(self._btn_native_split, 0, 0)
        ctr_layout.addWidget(self._btn_native_follow, 0, 1)
        ctr_layout.addWidget(self._btn_native_thermal, 1, 0)
        ctr_layout.addWidget(self._btn_native_palette, 1, 1)
        ctr_layout.addWidget(self._btn_native_torch, 2, 0)

        # Track (the camera follows an object) and Follow Me (the DRONE follows it), named because
        # Follow flies the drone. Not the "Follow vehicle" switch above, which moves the map.
        self._btn_native_m13_track = QPushButton("Track")
        self._btn_native_m13_track.setCheckable(True)
        self._btn_native_m13_track.hide()
        # Follow Me (vgcs/app/object_follow.py): the DRONE follows the tracked object at its distance. Not the
        # "Follow vehicle" button of the top row, which makes the map follow the drone. Checked while Follow runs;
        # a click asks the main window to start (it asks the operator first) or to stop.
        self._btn_follow_object = QPushButton("Follow")
        self._btn_follow_object.setObjectName("followObject")
        self._btn_follow_object.setCheckable(True)
        m13_track_body = CamM13TrackBlock(self._btn_native_m13_track, self._btn_follow_object, self._native_hud_right)
        self._native_m13_track_body = m13_track_body
        m13_track_body.hide()

        # Observe: opens the Observe panel beside the bar, and closes it again.
        self._btn_native_observe = QPushButton("Observe", self._native_hud_right)
        self._btn_native_observe.setObjectName("camObserveBtn")
        self._btn_native_observe.setCheckable(True)
        self._btn_native_observe.setIcon(ui_theme.icon("observe", 16))
        self._btn_native_observe.setIconSize(QSize(16, 16))
        self._btn_native_observe.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_native_observe.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn_native_observe.setToolTip(
            "Observe and DOOAF: Target, Clip, Report, Reset, Set TGT, Set HIT, the laser switch and DOOAF Setup.\n"
            "The panel stays open until you close it."
        )

        self._btn_native_rail_hide = ui_theme.round_button(
            self._native_hud_right, object_name="camRailHideBtn", icon_name="arrow_right", size=28,
            tooltip="Hide camera panel",
        )
        self._btn_native_rail_hide.clicked.connect(lambda: self._set_camera_rail_panel_visible(False))

        _bar = self._native_hud_right_layout
        _bar.addWidget(mode_row, 0, Qt.AlignmentFlag.AlignHCenter)
        _bar.addWidget(self._btn_native_record, 0, Qt.AlignmentFlag.AlignHCenter)
        _bar.addWidget(self._lbl_native_cam_timer, 0, Qt.AlignmentFlag.AlignHCenter)
        _bar.addWidget(self._camera_top_row, 0, Qt.AlignmentFlag.AlignHCenter)
        _bar.addWidget(m13_track_body)
        _bar.addWidget(self._btn_native_observe)
        _bar.addWidget(self._btn_native_rail_hide, 0, Qt.AlignmentFlag.AlignHCenter)

        # --- The gimbal and lens pad ------------------------------------------------------------------
        self._native_gimbal_panel = QFrame(self._native_rail_layer)
        self._native_gimbal_panel.setObjectName("cameraGimbalPanel")
        self._native_gimbal_panel.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_gimbal_panel.setAutoFillBackground(False)
        pad_lay = QGridLayout(self._native_gimbal_panel)
        pad_lay.setContentsMargins(_cam_m + 2, _cam_m + 2, _cam_m + 2, _cam_m + 2)
        pad_lay.setHorizontalSpacing(6)
        pad_lay.setVerticalSpacing(6)

        def _pad_button(icon_name: str, tooltip: str, *, size: int = _cam_btn) -> QPushButton:
            b = QPushButton(self._native_gimbal_panel)
            b.setToolTip(tooltip)
            return ui_theme.style_round_button(b, icon_name=icon_name, size=size, tone="plain")

        self._btn_native_gimbal_up = _pad_button("arrow_up", "Gimbal pitch up — press and hold")
        self._btn_native_gimbal_down = _pad_button("arrow_down", "Gimbal pitch down — press and hold")
        self._btn_native_gimbal_left = _pad_button("arrow_left", "Gimbal yaw left — press and hold")
        self._btn_native_gimbal_right = _pad_button("arrow_right", "Gimbal yaw right — press and hold")
        self._btn_native_gimbal_center = _pad_button("vama_center", "Recenter gimbal (yaw and pitch to 0°)")
        self._btn_native_gimbal_nadir = _pad_button(
            "vama_down",
            "Point gimbal straight down (default −90° pitch; set camera/gimbal_nadir_pitch_deg to 90 if needed)",
        )
        self._btn_native_zoom_plus = _pad_button("vama_zoom_in", "Zoom in")
        self._btn_native_zoom_minus = _pad_button("vama_zoom_out", "Zoom out")
        self._lbl_camera_top_zoom = QLabel(_format_video_zoom_label(1.0), self._native_gimbal_panel)
        self._lbl_camera_top_zoom.setObjectName("camMagnificationLabel")
        self._lbl_camera_top_zoom.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_camera_top_zoom.setToolTip("Camera magnification")
        self._lbl_camera_top_zoom.setFixedWidth(_cam_btn + 12)

        focus_row = QWidget(self._native_gimbal_panel)
        focus_row.setObjectName("cameraFocusRow")
        focus_lay = QHBoxLayout(focus_row)
        focus_lay.setContentsMargins(0, 0, 0, 0)
        focus_lay.setSpacing(6)
        self._btn_native_focus_minus = _pad_button("focus_near", "Focus nearer", size=32)
        self._btn_native_focus_plus = _pad_button("focus_far", "Focus farther", size=32)
        focus_cap = QLabel("Focus", focus_row)
        focus_cap.setObjectName("camPanelTitle")
        focus_lay.addStretch(1)
        focus_lay.addWidget(self._btn_native_focus_minus)
        focus_lay.addWidget(focus_cap)
        focus_lay.addWidget(self._btn_native_focus_plus)
        focus_lay.addStretch(1)

        # A cross of arrows round the centre button, looking down in the corner; the zoom beside it.
        pad_lay.addWidget(self._btn_native_gimbal_nadir, 0, 0)
        pad_lay.addWidget(self._btn_native_gimbal_up, 0, 1)
        pad_lay.addWidget(self._btn_native_gimbal_left, 1, 0)
        pad_lay.addWidget(self._btn_native_gimbal_center, 1, 1)
        pad_lay.addWidget(self._btn_native_gimbal_right, 1, 2)
        pad_lay.addWidget(self._btn_native_gimbal_down, 2, 1)
        pad_lay.setColumnMinimumWidth(3, 4)
        pad_lay.addWidget(self._btn_native_zoom_plus, 0, 4, Qt.AlignmentFlag.AlignHCenter)
        pad_lay.addWidget(self._lbl_camera_top_zoom, 1, 4, Qt.AlignmentFlag.AlignCenter)
        pad_lay.addWidget(self._btn_native_zoom_minus, 2, 4, Qt.AlignmentFlag.AlignHCenter)
        pad_lay.addWidget(focus_row, 3, 0, 1, 5)

        # --- The Observe panel ------------------------------------------------------------------------
        self._native_observe_panel = QFrame(self._native_rail_layer)
        self._native_observe_panel.setObjectName("cameraObservePanel")
        self._native_observe_panel.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._native_observe_panel.setAutoFillBackground(False)
        obs_lay = QVBoxLayout(self._native_observe_panel)
        obs_lay.setContentsMargins(_cam_m + 4, _cam_m, _cam_m + 4, _cam_m + 4)
        obs_lay.setSpacing(6)
        obs_head = QWidget(self._native_observe_panel)
        obs_head.setObjectName("cameraObserveHeader")
        obs_head_lay = QHBoxLayout(obs_head)
        obs_head_lay.setContentsMargins(0, 0, 0, 0)
        obs_title = QLabel("Observe", obs_head)
        obs_title.setObjectName("camPanelTitle")
        self._btn_native_observe_close = ui_theme.round_button(
            obs_head, object_name="camObserveCloseBtn", icon_name="close", size=26, tooltip="Close the Observe panel"
        )
        obs_head_lay.addWidget(obs_title, 1, Qt.AlignmentFlag.AlignVCenter)
        obs_head_lay.addWidget(self._btn_native_observe_close, 0, Qt.AlignmentFlag.AlignVCenter)

        self._btn_native_target = QPushButton("Target")
        self._btn_native_target.setCheckable(True)
        self._btn_native_clip = QPushButton("Clip")
        self._btn_native_report = QPushButton("Report")
        self._btn_native_report.setObjectName("observeReport")
        self._btn_native_reset = QPushButton("Reset")
        self._btn_native_reset.setObjectName("observeReset")
        observe_body = CamObserveBlock(
            self._btn_native_target,
            self._btn_native_clip,
            self._btn_native_report,
            self._btn_native_reset,
            self._native_observe_panel,
        )
        self._native_observe_body = observe_body
        obs_lay.addWidget(obs_head)
        obs_lay.addWidget(observe_body)
        self._native_observe_panel.setFixedWidth(ui_theme.OBSERVE_PANEL_WIDTH)
        self._native_observe_panel.hide()
        self._btn_native_observe.toggled.connect(self._set_observe_panel_open)
        self._btn_native_observe_close.clicked.connect(lambda: self._set_observe_panel_open(False))
        if self._observe_panel_open_pref():
            self._btn_native_observe.setChecked(True)

        self._native_hud_right.hide()
        self._native_rail_layer.hide()

        self._btn_camera_rail_show = CamRailShowHandle(
            self._panel,
            icon=ui_theme.icon("vama_photo", 22),
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
        self._obstacle_radar.bar_range_chosen.connect(self.obstacle_bar_range_chosen.emit)
        self._obstacle_radar.hide()

        # The flight buttons: one strip like QGC's tool strip in the app (2026-11-09 restyle), five
        # buttons of one size, each with its icon and its name. Until then they had three heights, and
        # Land showed an empty space where its icon should be (it asked for a "down" icon the footer's
        # icon class does not draw). Parent `_panel` (not `_map_canvas`), so they stay above the video.
        self._map_action_rail = QFrame(self._panel)
        self._map_action_rail.setObjectName("mapActionRail")
        self._map_action_rail.setStyleSheet(ui_theme.flight_rail_qss())
        ar_l = QVBoxLayout(self._map_action_rail)
        _pad = ui_theme.FLIGHT_RAIL_PADDING
        ar_l.setContentsMargins(_pad, _pad, _pad, _pad)
        ar_l.setSpacing(ui_theme.FLIGHT_RAIL_SPACING)

        def _flight_button(object_name: str, label: str, icon_name: str, tooltip: str) -> QToolButton:
            b = QToolButton(self._map_action_rail)
            b.setObjectName(object_name)
            b.setText(label)
            b.setToolTip(tooltip)
            b.setProperty("vamaIcon", icon_name)
            b.setIcon(ui_theme.icon(icon_name, ui_theme.FLIGHT_ICON_SIZE))
            b.setIconSize(QSize(ui_theme.FLIGHT_ICON_SIZE, ui_theme.FLIGHT_ICON_SIZE))
            b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            b.setFixedSize(ui_theme.FLIGHT_BUTTON_W, ui_theme.FLIGHT_BUTTON_H)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.setAutoRaise(True)
            return b

        self._map_action_takeoff_btn = _flight_button(
            "mapActionTakeoffBtn",
            "Takeoff",
            "takeoff",
            "NAV_TAKEOFF at main Takeoff alt (m). Connect vehicle first (same as dashboard Takeoff).",
        )
        # Land belongs beside Takeoff and Return. Requested 2026-09-10, where
        # the whole flight is described from these buttons: "user can click on
        # the take off button ... after that same for land or return button".
        # Land existed only on the dashboard, so the one screen an operator
        # flies from could start a flight but not finish one.
        self._map_action_land_btn = _flight_button(
            "mapActionLandBtn", "Land", "land", "Land where the aircraft is now (same as dashboard Land)."
        )
        # Stop the motors after a normal landing. Requested 2026-09-11: until
        # now the only disarm in VGCS was EMERGENCY STOP, which forces past
        # every check and is meant for a runaway.
        self._map_action_disarm_btn = _flight_button(
            "mapActionDisarmBtn",
            "Disarm",
            "disarm",
            "Stop the motors after landing. The aircraft refuses this while it\n"
            "still believes it is flying; use EMERGENCY STOP for a runaway.",
        )
        self._map_action_return_btn = _flight_button(
            "mapActionReturnBtn", "Return", "return_home", "Return to launch / RTL (same as dashboard Return)."
        )
        # Type a position and go there. Requested 2026-09-08: "if I put the lat
        # long then I can see that particular location on the map". Unlike the
        # others it needs no vehicle, so it is never disabled.
        self._map_action_goto_btn = _flight_button(
            "mapActionGotoBtn",
            "Go to",
            "goto",
            "Go to a position you type in: latitude and longitude, or a grid\n"
            "reference. Also on Ctrl+G, which works while planning too.",
        )

        ar_l.addWidget(self._map_action_takeoff_btn)
        ar_l.addWidget(self._map_action_land_btn)
        ar_l.addWidget(self._map_action_disarm_btn)
        ar_l.addWidget(self._map_action_return_btn)
        ar_l.addWidget(self._map_action_goto_btn)
        self._map_action_takeoff_btn.setEnabled(False)
        self._map_action_land_btn.setEnabled(False)
        self._map_action_disarm_btn.setEnabled(False)
        self._map_action_return_btn.setEnabled(False)
        self._map_action_takeoff_btn.clicked.connect(lambda: self.takeoff_requested.emit())
        self._map_action_land_btn.clicked.connect(lambda: self.land_requested.emit())
        self._map_action_disarm_btn.clicked.connect(lambda: self.disarm_requested.emit())
        self._map_action_return_btn.clicked.connect(lambda: self.return_requested.emit())
        self._map_action_goto_btn.clicked.connect(self.prompt_goto_location)
        # Ctrl+G reaches it from the plan view too, where the action rail is
        # hidden behind the plan tool rail.
        self._goto_shortcut = QShortcut(QKeySequence("Ctrl+G"), self)
        self._goto_shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
        self._goto_shortcut.activated.connect(self.prompt_goto_location)
        # Map and camera side by side. Reachable from the plan view too, where
        # the action rail is hidden, same as Ctrl+G.
        self._split_shortcut = QShortcut(QKeySequence("Ctrl+D"), self)
        self._split_shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
        self._split_shortcut.activated.connect(self.toggle_map_video_split)
        # Height from the layout rather than arithmetic over the button sizes.
        # Adding Land by hand-summing 54 + 8 + 54 + 8 + 54 + 8 + 34 came out six
        # pixels short and pushed Go to outside its own parent, where it is
        # simply not drawn.
        ar_l.activate()
        self._map_action_rail.setFixedSize(ar_l.sizeHint())
        self._map_action_rail.show()
        self._map_action_rail.raise_()

        self._plan_flight_panel = PlanFlightPanel(panel)
        self._plan_flight_panel.hide()
        self._plan_flight_panel.exit_requested.connect(self._on_plan_panel_exit)
        self._plan_flight_panel.action_requested.connect(self.plan_action_requested.emit)
        self._plan_flight_panel.tool_requested.connect(self._on_plan_panel_tool)
        self._plan_flight_panel.mission_panel_changed.connect(self._on_plan_panel_mission_changed)
        self._plan_flight_panel.mission_start_requested.connect(self.mission_start_requested.emit)
        self._plan_flight_panel.mission_pause_requested.connect(self.mission_pause_requested.emit)
        self._plan_flight_panel.mission_resume_requested.connect(self.mission_resume_requested.emit)
        self._plan_flight_panel.mission_jump_requested.connect(self.mission_jump_requested.emit)
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
            "QFrame#nativeMinimapWrap { background: transparent; border: 1px solid rgba(255, 255, 255, 110); border-radius: 10px; }"
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
        self._btn_native_minimap_plus = QPushButton(self._native_minimap_wrap)
        self._btn_native_minimap_minus = QPushButton(self._native_minimap_wrap)
        # The app's round buttons, as on the main map's zoom (2026-11-09 restyle).
        for b, icon_name, tip in (
            (self._btn_native_minimap_plus, "plus", "Zoom the small map in"),
            (self._btn_native_minimap_minus, "minus", "Zoom the small map out"),
        ):
            ui_theme.style_round_button(b, icon_name=icon_name, size=self._native_minimap_btn_side, tone="plain")
            b.setToolTip(tip)
            b.setAutoDefault(False)
            b.setDefault(False)
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
        self._btn_native_palette.clicked.connect(self._on_native_palette_clicked)
        self._btn_native_torch.toggled.connect(self._on_native_torch_toggled)
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
            # Marking impacts means repeated ranges; hold the laser warm so each
            # one does not pay the cold start.
            self._set_lrf_session_hold(bool(on))
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
            self._set_lrf_session_hold(False)
            self._reset_c13_lrf_for_observe_reset()
            self._stop_m13_track()
            self._clear_observations()

        def _obs_dooaf_setup() -> None:
            print("[VGCS:cam_rail] OBSERVE DOOAF Setup clicked")
            # Gun and target picks come in quick succession from this dialog.
            self._set_lrf_session_hold(True)
            QTimer.singleShot(0, self._show_dooaf_setup_dialog)

        def _obs_laser(role: str, on: bool) -> None:
            print(f"[VGCS:cam_rail] OBSERVE Laser toggled role={role} on={bool(on)}")
            self._set_role_uses_lrf(str(role), bool(on))

        self._btn_native_target.toggled.connect(_obs_target)
        self._btn_native_clip.clicked.connect(_obs_clip)
        self._btn_native_report.clicked.connect(_obs_report)
        self._btn_native_reset.clicked.connect(_obs_reset)
        observe_body.setup_clicked.connect(_obs_dooaf_setup)
        for role in (DOOAF_ROLE_INTENDED, DOOAF_ROLE_IMPACT):
            observe_body.set_laser_for(role, self._role_uses_lrf(role))
        observe_body.laser_changed.connect(_obs_laser)

        def _m13_track_toggle(on: bool) -> None:
            if not on:
                self._stop_m13_track()
            self._set_m13_track_armed(bool(on))

        self._btn_native_m13_track.toggled.connect(_m13_track_toggle)
        self._btn_follow_object.clicked.connect(lambda _checked=False: self.follow_object_requested.emit())

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
        self._btn_m14_detect = QPushButton("Detect")
        self._btn_m14_detect.setToolTip(
            "Run on-demand object + license-plate-region detection on the "
            "current video frame (M14)"
        )
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
        # Payload drop on arrival at the selected waypoint. Requested
        # 2026-09-11: "if i plot the 5 waypoint ... suppose drone is arrived
        # point 1 then servo payload will drop".
        self._wp_drop_payload = QCheckBox("Drop payload here")
        self._wp_drop_payload.setToolTip(
            "Release the payload servo when the aircraft reaches this waypoint.\n"
            "The servo output and its pulse widths: Application Settings,\n"
            "General, Payload release."
        )
        self._btn_apply_all_speed = QPushButton("Set All Speed")
        # Hover on arrival, set before upload. Requested 2026-09-11: "before
        # upload the mission i will set the hover time like 5s or 10s that
        # means drone will hover every point which i set the hover time".
        self._wp_hover = QSpinBox()
        self._wp_hover.setRange(0, MAX_WP_HOVER_S)
        self._wp_hover.setSingleStep(1)
        self._wp_hover.setValue(0)
        # The label lives in the box: row 2 of this grid has three free cells
        # before the tile buttons, and the two Set buttons need two of them.
        self._wp_hover.setPrefix("Hover ")
        self._wp_hover.setSuffix(" s")
        self._wp_hover.setToolTip(
            "Seconds to hold position on arrival before flying to the next\n"
            "waypoint. 0 flies straight through. A waypoint that also drops\n"
            "the payload releases at the end of its hover."
        )
        self._btn_apply_wp_hover = QPushButton("Set WP Hover")
        self._btn_apply_all_hover = QPushButton("Set All Hover")
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
        tools.addWidget(self._wp_drop_payload, 2, 4)
        tools.addWidget(self._wp_hover, 2, 5)
        tools.addWidget(self._btn_apply_wp_hover, 2, 6)
        tools.addWidget(self._btn_apply_all_hover, 2, 7)
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
        tools.addWidget(self._btn_m14_detect, 2, 18)
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
        self._btn_m14_detect.clicked.connect(self._m14_run_detection)
        self._perf_mode.currentIndexChanged.connect(self._on_perf_mode_changed)
        self._wp_selector.currentIndexChanged.connect(self._on_wp_selected)
        self._wp_drop_payload.toggled.connect(self._on_wp_drop_payload_toggled)
        self._btn_apply_wp_alt.clicked.connect(self._apply_altitude_to_selected)
        self._btn_apply_all_alt.clicked.connect(self._apply_altitude_to_all)
        self._btn_apply_wp_speed.clicked.connect(self._apply_speed_to_selected)
        self._btn_apply_wp_hover.clicked.connect(self._apply_hover_to_selected)
        self._btn_apply_all_hover.clicked.connect(self._apply_hover_to_all)
        self._btn_apply_all_speed.clicked.connect(self._apply_speed_to_all)

        self._wp_poll = QTimer(self)
        self._wp_poll.setInterval(1000)
        self._wp_poll.timeout.connect(self._sync_waypoint_count_from_map)
        self._wp_poll.start()

        self._init_map_backend()
