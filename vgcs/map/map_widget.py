"""M2 map scaffold with live position API and WebEngine/Leaflet integration."""

from __future__ import annotations

import base64
import json
from pathlib import Path
from urllib.parse import quote
from urllib.request import Request, urlopen

from PySide6.QtCore import (
    QByteArray,
    QBuffer,
    QObject,
    QPoint,
    QRunnable,
    QSettings,
    QThreadPool,
    QTimer,
    Qt,
    QUrl,
)
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QFileDialog,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtGui import QImage
from vgcs.mission import (
    Waypoint,
    load_waypoints_json,
    save_waypoints_json,
    save_waypoints_kml,
)

# Optional: live camera preview for map overlay.
try:
    from PySide6.QtMultimedia import QCamera, QMediaCaptureSession, QMediaDevices, QVideoSink

    HAS_MULTIMEDIA = True
except Exception:  # pragma: no cover - depends on platform build
    QCamera = None  # type: ignore[assignment]
    QMediaCaptureSession = None  # type: ignore[assignment]
    QMediaDevices = None  # type: ignore[assignment]
    QVideoSink = None  # type: ignore[assignment]
    HAS_MULTIMEDIA = False

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


try:
    from PySide6.QtWebEngineCore import QWebEngineSettings
    from PySide6.QtWebEngineCore import QWebEngineProfile
    from PySide6.QtWebEngineCore import QWebEngineUrlRequestInterceptor
    from PySide6.QtWebEngineCore import QWebEnginePage
    from PySide6.QtWebEngineWidgets import QWebEngineView

    HAS_WEBENGINE = True
except Exception:  # pragma: no cover - environment-specific availability
    QWebEngineSettings = None  # type: ignore[assignment,misc]
    QWebEngineProfile = None  # type: ignore[assignment,misc]
    QWebEngineUrlRequestInterceptor = None  # type: ignore[assignment]
    QWebEnginePage = None  # type: ignore[assignment]
    QWebEngineView = None  # type: ignore[assignment]
    HAS_WEBENGINE = False


LEAFLET_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" onerror="document.title='VGCS_ASSET_ERROR:leaflet_css:'+Date.now()"/>
  <link rel="stylesheet" href="https://unpkg.com/cesium@1.125/Build/Cesium/Widgets/widgets.css" onerror="document.title='VGCS_ASSET_ERROR:cesium_css:'+Date.now()"/>
  <style>
    html, body, #mapWrap, #map2d, #map3d { height:100%; margin:0; background:#1a1d24; }
    /* Single scroll/compositor root; avoid promoted layers that confuse WebEngine on Windows. */
    #mapWrap { position: relative; overflow: hidden; z-index: 0; }
    #map2d, #map3d { position: absolute; inset: 0; }
    #map3d { display: none; }
    .leaflet-control-attribution {
      display: none !important;
    }
    .overlay { position:absolute; z-index:1200; font-family: "Segoe UI", Arial, sans-serif; }
    #linkBanner {
      top:0; left:0; right:0; min-height:46px; padding:8px 12px; border-radius:0;
      background: rgba(24, 30, 40, 0.95); color:#dbe3f3; font-size:17px; font-weight:600;
      border:0;
      border-bottom:1px solid rgba(72, 86, 110, 0.9);
      display:flex; align-items:center; gap:8px;
      box-shadow: 0 1px 2px rgba(0,0,0,0.18);
    }
    #linkBannerLogo {
      height:28px;
      width:auto;
      display:none;
      object-fit:contain;
      flex:0 0 auto;
    }
    #linkBannerDisconnected {
      display:flex;
      align-items:center;
      gap:8px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
      min-width: 0;
      flex: 1 1 auto;
    }
    #linkBannerConnected {
      display:none;
      align-items:center;
      gap:12px;
      color: #f4f7ff;
      font-size: 14px;
      font-weight: 600;
      white-space: nowrap;
      overflow: hidden;
      min-width: 0;
      flex: 1 1 auto;
    }
    #hdrMapModeBtn {
      margin-left:8px;
      min-width:62px;
      height:26px;
      border-radius:13px;
      border:1px solid rgba(210, 220, 240, 0.65);
      background: rgba(20, 30, 42, 0.7);
      color:#f1f6ff;
      font-size:11px;
      font-weight:700;
      letter-spacing:0.02em;
      cursor:pointer;
      padding:0 10px;
      flex:0 0 auto;
    }
    #hdrMapModeBtn:hover {
      background: rgba(36, 50, 69, 0.9);
    }
    .hdrPill {
      display:inline-flex;
      align-items:center;
      gap:6px;
      color:#f4f7ff;
      min-width: 0;
      flex: 0 0 auto;
    }
    #hdrVehiclePill {
      flex: 1 1 0;
      min-width: 0;
      overflow: hidden;
    }
    #hdrVehicleMsg {
      display:inline-block;
      min-width: 0;
      max-width: 100%;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .hdrPillMuted {
      color: #d7deef;
      font-weight: 500;
    }
    .hdrSep {
      width:1px;
      height:24px;
      background: rgba(210, 220, 240, 0.55);
      flex: 0 0 auto;
    }
    .hdrIcon {
      width:22px;
      height:22px;
      object-fit:contain;
      flex:0 0 auto;
    }
    .hdrIconBroadcast {
      width:26px;
      height:26px;
    }
    .hdrIconSmall {
      width:20px;
      height:20px;
    }
    .hdrReadyText {
      color:#d5ff9b;
    }
    .hdrTinyStack {
      display:inline-flex;
      flex-direction:column;
      line-height:1.05;
      font-size:12px;
      color:#f4f7ff;
    }
    #actionRail { top:78px; left:10px; display:flex; flex-direction:column; gap:8px; }
    .actionBtn {
      width:54px; height:54px; border-radius:8px; border:1px solid rgba(255,255,255,0.35);
      background: rgba(34, 42, 56, 0.92); color:#c8d3ea; font-size:11px; text-align:center;
      display:flex; flex-direction:column; justify-content:center; align-items:center;
      cursor:pointer;
      user-select:none;
    }
    .actionBtn:hover {
      background: rgba(46, 58, 78, 0.95);
      border-color: rgba(136, 164, 205, 0.7);
    }
    .actionBtn.disabled {
      opacity: 0.45;
      cursor: not-allowed;
      pointer-events: none;
      border-color: rgba(120, 130, 150, 0.35);
      background: rgba(30, 38, 52, 0.75);
    }
    #planFlightLayer {
      position:absolute;
      inset:0;
      z-index:1250;
      display:none;
      pointer-events:none;
      font-family: "Segoe UI", Arial, sans-serif;
    }
    #planFlightTopBar {
      position:absolute;
      top:0;
      left:0;
      right:0;
      min-height:64px;
      background: rgba(32, 34, 40, 0.97);
      color:#e8eaef;
      display:flex;
      align-items:stretch;
      gap:14px;
      padding: 6px 14px 8px;
      font-size:15px;
      border-bottom:1px solid rgba(70, 76, 88, 0.85);
      pointer-events:auto;
      flex-wrap: nowrap;
    }
    #planBarUpload {
      align-self:center;
      height:32px;
      padding:0 18px;
      border-radius:6px;
      border:1px solid rgba(110, 118, 135, 0.85);
      background: rgba(58, 62, 74, 0.95);
      color:#e8eaef;
      font-size:13px;
      font-weight:600;
      cursor:pointer;
      flex:0 0 auto;
    }
    #planBarUpload:hover {
      background: rgba(72, 78, 92, 0.98);
    }
    #planBarUpload:disabled {
      opacity: 0.4;
      cursor: not-allowed;
    }
    #planFlightTopBar .pfGroup {
      display:flex;
      flex-direction:column;
      justify-content:center;
      gap:3px;
      min-width: 108px;
      padding: 0 2px;
    }
    #planFlightTopBar .pfLabel {
      font-size:11px;
      color:#9ca3b0;
      font-weight:600;
      line-height:1.1;
      white-space: nowrap;
      letter-spacing: 0.01em;
    }
    #planFlightTopBar .pfMetric {
      font-size:13px;
      color:#f3f4f6;
      line-height:1.2;
      white-space:nowrap;
      font-weight: 400;
    }
    #planFlightTopBar .pfMetric b {
      font-weight: 700;
      color:#ffffff;
    }
    #planFlightTopBar .pfMetricGhost {
      visibility: hidden;
    }
    #planFlightTopBar .pfSpacer {
      flex: 0 0 10px;
      min-width: 10px;
    }
    #planExit {
      font-size: 34px;
      display:inline-flex;
      align-items:flex-start;
      padding-top: 6px;
      color:#f3f4f6;
      white-space: nowrap;
      min-width: 92px;
      cursor:pointer;
    }
    #planWorkspace {
      position:absolute;
      top:64px;
      left:0;
      right:0;
      bottom:0;
      display:flex;
      gap:10px;
      padding:10px;
      align-items:flex-start;
      pointer-events:none;
    }
    #planFlightToolRail {
      width:78px;
      border-radius:8px;
      background: rgba(28, 30, 36, 0.96);
      border:1px solid rgba(65, 70, 82, 0.9);
      box-shadow: 0 2px 8px rgba(0,0,0,0.16);
      padding:5px 0;
      display:flex;
      flex-direction:column;
      gap:4px;
      pointer-events:auto;
    }
    .planToolBtn {
      position: relative;
      margin:0 5px;
      min-height:52px;
      border-radius:6px;
      border:1px solid rgba(70, 76, 88, 0.85);
      background: rgba(40, 44, 52, 0.95);
      color:#e8eaef;
      font-size:11px;
      font-weight:600;
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      gap:2px;
      cursor:pointer;
      user-select:none;
      text-align:center;
      padding: 3px 2px;
      letter-spacing: 0.01em;
      pointer-events:auto;
    }
    .planToolIcon {
      width:17px;
      height:17px;
      display:inline-flex;
      align-items:center;
      justify-content:center;
      font-size:15px;
      line-height:1;
      opacity:0.98;
      flex:0 0 auto;
    }
    .planToolBtn.active {
      background:#facc15;
      color:#111827;
      border-color:#ca8a04;
      margin-right: 10px;
    }
    .planToolBtn.active .planToolIcon {
      color:#111827;
    }
    .planToolBtn.active::after {
      content: "";
      position: absolute;
      right: -7px;
      top: 50%;
      transform: translateY(-50%);
      border-width: 9px 0 9px 9px;
      border-style: solid;
      border-color: transparent transparent transparent #facc15;
      filter: drop-shadow(1px 0 0 rgba(0,0,0,0.12));
    }
    .planToolBtn:hover { background: rgba(55, 60, 72, 0.98); }
    .planToolBtn.active:hover { background:#eab308; }
    #planCenterPanel {
      min-width: 540px;
      max-width: 620px;
      border-radius:8px;
      overflow:hidden;
      background: rgba(36, 39, 48, 0.98);
      border:1px solid rgba(70, 76, 88, 0.9);
      box-shadow: 0 8px 24px rgba(0,0,0,0.35);
      pointer-events:auto;
    }
    #planFileFlyout {
      padding: 12px 14px 14px;
      color:#e8eaef;
      font-size:12px;
    }
    .planFileSection {
      margin-bottom: 16px;
    }
    .planFileSection:last-child {
      margin-bottom: 0;
    }
    .planFileSectionTitle {
      font-size: 13px;
      font-weight: 700;
      color: #f9fafb;
      margin-bottom: 10px;
      letter-spacing: 0.02em;
    }
    .planFileCardGrid {
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap:8px;
    }
    .planTplCard {
      border-radius:6px;
      overflow:hidden;
      border:1px solid rgba(90, 96, 110, 0.85);
      cursor:pointer;
      display:flex;
      flex-direction:column;
      background: rgba(28, 30, 36, 0.98);
    }
    .planTplCard:hover {
      border-color: rgba(250, 204, 21, 0.65);
      box-shadow: 0 0 0 1px rgba(250, 204, 21, 0.25);
    }
    .planTplPrev {
      flex: 1 1 82px;
      min-height: 82px;
      position: relative;
      overflow: hidden;
      background: linear-gradient(145deg, #2d3748, #4a5568);
    }
    .planTplPrevImg {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      object-fit: cover;
      object-position: center;
      display: block;
    }
    .planTplLabel {
      background: rgba(24, 26, 32, 0.98);
      border-top:1px solid rgba(70, 76, 88, 0.85);
      text-align:center;
      font-size:11px;
      font-weight:600;
      padding:6px 4px 7px;
      color:#e8eaef;
    }
    .planFileBtnRow {
      display:flex;
      flex-wrap: wrap;
      gap:8px;
      margin-bottom:8px;
    }
    .planFileBtn {
      flex: 1 1 auto;
      min-height:30px;
      padding: 0 12px;
      border-radius:5px;
      border:1px solid rgba(90, 96, 110, 0.9);
      background: rgba(48, 52, 62, 0.95);
      color:#e8eaef;
      font-size:12px;
      font-weight:600;
      cursor:pointer;
    }
    .planFileBtn:hover:not(:disabled) {
      background: rgba(62, 68, 82, 0.98);
    }
    .planFileBtn:disabled {
      opacity:0.38;
      cursor:not-allowed;
    }
    .planFileBtnPrimary {
      background: rgba(88, 82, 118, 0.95);
      border-color: rgba(120, 110, 160, 0.85);
    }
    .planFileBtnPrimary:hover:not(:disabled) {
      background: rgba(100, 94, 132, 0.98);
    }
    .planFileBtnSecondary {
      background: rgba(52, 56, 66, 0.95);
    }
    .planFileBtnWide {
      width:100%;
      display:block;
    }
    #planOtherToolPanel {
      padding: 20px 16px;
      color:#e8eaef;
      font-size: 13px;
      line-height: 1.45;
      min-height: 120px;
    }
    #planRightPanel {
      margin-left:auto;
      width: 340px;
      max-width: min(340px, calc(100vw - 24px));
      max-height: calc(100vh - 120px);
      border-radius:6px;
      background: rgba(36, 38, 46, 0.94);
      border:1px solid rgba(92, 96, 120, 0.85);
      box-shadow: 0 8px 28px rgba(0,0,0,0.35);
      overflow-x: visible;
      overflow-y: auto;
      pointer-events:auto;
      display:flex;
      flex-direction:column;
      font-family: "Segoe UI", Arial, sans-serif;
    }
    #planTabs {
      display:flex;
      flex-shrink:0;
      min-height:38px;
      background:#3a3d4a;
      border-bottom:1px solid rgba(0,0,0,0.35);
    }
    #planTabs:focus-within {
      outline: none;
    }
    .planTab {
      flex: 1 1 0;
      min-width: 0;
      border:none;
      border-right:1px solid rgba(0,0,0,0.28);
      background:#3a3d4a;
      color:#e8eaef;
      font-size:12px;
      font-weight:600;
      cursor:pointer;
      padding:8px 4px;
      position: relative;
    }
    .planTab:focus-visible {
      outline: 2px solid rgba(250, 204, 21, 0.85);
      outline-offset: -2px;
      z-index: 1;
    }
    .planTab:last-child { border-right:none; }
    .planTab:hover:not(.active) {
      background:#45485a;
    }
    .planTab.active {
      background:#f5e6a0;
      color:#111827;
      font-weight:700;
    }
    #planSection {
      padding:0;
    }
    .planTabBody {
      padding: 0;
      flex:1;
      min-height:0;
    }
    .planTabBody[hidden] {
      display: none !important;
    }
    .planTabHint {
      font-size: 12px;
      color: rgba(232, 234, 239, 0.92);
      line-height: 1.5;
      margin: 0 0 12px;
    }
    .planSectionHeader {
      background:#4d5170;
      color:#f9fafb;
      font-size:13px;
      font-weight:600;
      padding:9px 12px;
      letter-spacing:0.02em;
    }
    .planSectionBody {
      padding:12px;
      background:#0c0c0e;
      color:#e8eaef;
      font-size:12px;
    }
    .planSectionBody--fence {
      background:#14151a;
    }
    .planFieldLabel {
      color:rgba(248, 250, 252, 0.88);
      font-size:11px;
      font-weight:600;
      margin:10px 0 5px;
    }
    .planFieldLabel:first-child { margin-top:0; }
    .planRailSelect {
      width:100%;
      box-sizing:border-box;
      min-height:32px;
      padding:6px 28px 6px 10px;
      border-radius:5px;
      border:1px solid rgba(100, 106, 124, 0.65);
      background:#ffffff;
      color:#111827;
      font-size:12px;
      font-weight:500;
      cursor:pointer;
      appearance:none;
      background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='12' viewBox='0 0 12 12'%3E%3Cpath fill='%23374151' d='M3 4.5L6 8l3-3.5z'/%3E%3C/svg%3E");
      background-repeat:no-repeat;
      background-position:right 10px center;
    }
    .planRailInput {
      display:flex;
      align-items:center;
      box-sizing:border-box;
      min-height:32px;
      padding:0 10px;
      border-radius:5px;
      border:1px solid rgba(100, 106, 124, 0.65);
      background:#ffffff;
      color:#111827;
    }
    .planRailInput input {
      flex:1;
      min-width:0;
      border:none;
      background:transparent;
      color:#111827;
      font-size:13px;
      font-weight:500;
      padding:6px 0;
      outline:none;
    }
    .planRailUnit {
      flex:0 0 auto;
      margin-left:8px;
      font-size:12px;
      font-weight:600;
      color:#374151;
    }
    .planKvRow {
      display:flex;
      justify-content:space-between;
      align-items:baseline;
      gap:10px;
      font-size:12px;
      margin:8px 0;
      color:#e8eaef;
    }
    .planKvRow span:first-child { color:rgba(210, 214, 222, 0.88); }
    .planKvRow b { font-weight:700; color:#fff; }
    .planNoteMission {
      font-size:11px;
      line-height:1.5;
      color:rgba(218, 220, 228, 0.88);
      font-weight:400;
      margin:12px 0 14px;
    }
    #planVehicleDetails.planRailDetails summary {
      font-weight:700;
      font-size:13px;
      color:#fff;
      padding:12px 0 10px;
      margin:0;
      border-bottom:1px solid rgba(255,255,255,0.92);
    }
    #planVehicleDetails.planRailDetails .planRailDetailsInner {
      padding-top:12px;
      padding-bottom:4px;
    }
    #planVehicleDetails.planRailDetails .planRailDetailsInner > .planFieldLabel {
      margin-top:2px;
    }
    .planHelpMuted {
      font-size:11px;
      line-height:1.45;
      color:rgba(232,234,239,0.72);
      margin:8px 0 10px;
    }
    .planRailDetails {
      margin-top:4px;
      border-top:1px solid rgba(255,255,255,0.28);
    }
    .planRailDetails + .planRailDetails { margin-top:0; }
    .planRailDetails summary {
      list-style:none;
      cursor:pointer;
      color:#f9fafb;
      display:flex;
      justify-content:space-between;
      align-items:center;
      font-size:13px;
      font-weight:600;
      padding:12px 0 10px;
    }
    .planRailDetails summary::-webkit-details-marker { display:none; }
    .planRailDetails .planRailChev {
      display:inline-block;
      font-size:10px;
      opacity:0.85;
      transition: transform 0.15s ease;
    }
    .planRailDetails[open] .planRailChev { transform: rotate(180deg); }
    .planRailDetailsInner {
      padding-bottom:12px;
    }
    .planGeoLead {
      font-size:11px;
      color:rgba(232,234,239,0.8);
      line-height:1.45;
      margin:0 0 14px;
    }
    .planGeoBlock {
      margin-bottom:14px;
      padding-bottom:12px;
      border-bottom:1px solid rgba(255,255,255,0.22);
    }
    .planGeoBlock:last-child {
      border-bottom:none;
      margin-bottom:0;
      padding-bottom:0;
    }
    .planGeoTitle {
      font-size:12px;
      font-weight:700;
      color:#fff;
      margin:0 0 10px;
    }
    .planGeoStatus {
      font-size:12px;
      color:rgba(232,234,239,0.78);
    }
    .planGeoBtnStack {
      display:flex;
      flex-direction:column;
      gap:8px;
    }
    .planGeoBtn {
      width:100%;
      box-sizing:border-box;
      min-height:34px;
      padding:8px 12px;
      border-radius:6px;
      border:1px solid rgba(80, 86, 102, 0.9);
      background:#3d414d;
      color:#f3f4f6;
      font-size:12px;
      font-weight:600;
      cursor:pointer;
    }
    .planGeoBtn:hover:not(:disabled) {
      background:#4a4f5e;
    }
    .planGeoBtn:disabled {
      opacity:0.45;
      cursor:not-allowed;
    }
    .planRallyInfo {
      margin:0;
      padding:14px 14px 16px;
      border-radius:8px;
      background:#121318;
      border:1px solid rgba(80, 86, 102, 0.55);
      color:#e8eaef;
      font-size:12px;
      line-height:1.5;
    }
    .planFold {
      margin-top:12px;
      border-top:1px solid rgba(94, 99, 109, 0.55);
      padding-top:8px;
      font-size:13px;
      color:#1f2937;
      display:flex;
      justify-content:space-between;
      align-items:center;
    }
    #planStartMissionBtn {
      margin-top:14px;
      width:100%;
      min-height:36px;
      border-radius:6px;
      border:1px solid rgba(120, 90, 40, 0.85);
      background:#9a6b2d;
      color:#ffffff;
      font-size:13px;
      font-weight:700;
      cursor:pointer;
    }
    #planStartMissionBtn:hover {
      background:#b07a34;
    }
    #planSetLaunchMapCenterBtn {
      width:100%;
      margin-top:10px;
      min-height:34px;
      border-radius:6px;
      border:1px solid rgba(80, 86, 102, 0.9);
      background:#3d414d;
      color:#f3f4f6;
      font-size:12px;
      font-weight:600;
      cursor:pointer;
    }
    #planSetLaunchMapCenterBtn:hover {
      background:#4a4f5e;
    }
    #cameraRail {
      top: 78px;
      right: 18px;
      display: none;
      flex-direction: column;
      align-items: center;
      gap: 12px;
      padding: 12px 12px 14px;
      border-radius: 14px;
      background: linear-gradient(180deg, rgba(39, 47, 61, 0.96), rgba(30, 38, 52, 0.96));
      border: 1px solid rgba(188, 202, 224, 0.42);
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.34);
      min-width: 108px;
      z-index: 1215;
    }
    #videoPreview {
      left: 14px;
      bottom: 14px;
      width: 230px;
      height: 130px;
      display: none;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid rgba(206, 220, 242, 0.35);
      background: #0f1623;
      box-shadow: 0 8px 20px rgba(0, 0, 0, 0.32);
      z-index: 1210;
    }
    #videoPreview img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: block;
    }
    #videoPreviewPlaceholder {
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      color: #eef3ff;
      font-size: 20px;
      font-weight: 600;
      letter-spacing: 0.03em;
      background: linear-gradient(180deg, rgba(18, 27, 42, 0.9), rgba(10, 16, 28, 0.92));
      text-shadow: 0 1px 2px rgba(0,0,0,0.45);
      pointer-events: none;
    }
    #cameraTopRow {
      display: flex;
      align-items: center;
      gap: 0;
      padding: 3px;
      border-radius: 22px;
      background: rgba(73, 83, 102, 0.9);
      border: 1px solid rgba(190, 202, 224, 0.35);
    }
    .camSmallBtn {
      width: 48px;
      height: 38px;
      border-radius: 10px;
      border: 1px solid rgba(196, 209, 230, 0.22);
      background: transparent;
      color: #e8edf8;
      font-size: 18px;
      font-weight: 600;
      letter-spacing: 0;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      user-select: none;
      transition: background 120ms ease, border-color 120ms ease, transform 120ms ease;
    }
    .camSmallBtn:hover {
      background: rgba(110, 123, 148, 0.24);
      border-color: rgba(229, 237, 251, 0.4);
    }
    .camSmallBtn.active {
      border-color: rgba(214, 224, 241, 0.9);
      background: rgba(27, 33, 45, 0.96);
      box-shadow: 0 0 0 1px rgba(230, 238, 252, 0.18) inset;
      color: #69e86f;
    }
    #camVideoBtn {
      width: 38px;
      height: 38px;
      border-radius: 50%;
      margin-right: 4px;
      border-color: transparent;
      box-shadow: none;
    }
    #camVideoBtn:hover {
      border-color: transparent;
      background: rgba(110, 123, 148, 0.2);
      transform: none;
    }
    #camVideoBtn.active {
      border-color: rgba(214, 224, 241, 0.9);
      background: rgba(27, 33, 45, 0.96);
      box-shadow: 0 0 0 1px rgba(230, 238, 252, 0.18) inset;
    }
    #camPhotoBtn {
      width: 48px;
      height: 38px;
      border-radius: 9px;
      border-color: transparent;
      box-shadow: none;
      color: #e8edf8;
    }
    #camPhotoBtn:hover {
      border-color: transparent;
      background: rgba(110, 123, 148, 0.2);
      transform: none;
    }
    #camPhotoBtn.active {
      width: 38px;
      height: 38px;
      border-radius: 50%;
      margin-left: 5px;
      margin-right: 5px;
      border-color: rgba(214, 224, 241, 0.9);
      background: rgba(27, 33, 45, 0.96);
      box-shadow: 0 0 0 1px rgba(230, 238, 252, 0.18) inset;
      color: #f2f6ff;
    }
    #camRecordBtn {
      width: 50px;
      height: 50px;
      border-radius: 50%;
      border: 2px solid rgba(231, 239, 255, 0.9);
      background: radial-gradient(circle at 50% 35%, rgba(35, 43, 57, 0.96), rgba(20, 26, 36, 0.96));
      display: inline-flex;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      user-select: none;
      transition: transform 120ms ease, box-shadow 120ms ease;
    }
    #camRecordBtn:hover {
      transform: translateY(-1px);
      box-shadow: 0 3px 8px rgba(0,0,0,0.35);
    }
    #camRecordDot {
      width: 28px;
      height: 28px;
      border-radius: 50%;
      background: linear-gradient(180deg, #ff4b4b, #f62d2d);
      box-shadow: 0 1px 2px rgba(0,0,0,0.4);
    }
    #camRecordBtn.recording #camRecordDot {
      border-radius: 8px;
      width: 22px;
      height: 22px;
      background: linear-gradient(180deg, #ff5555, #ff3939);
    }
    #camTimer {
      font-weight: 700;
      font-size: 15px;
      color: #ffffff;
      background: rgba(255, 65, 65, 0.92);
      border-radius: 10px;
      padding: 4px 10px;
      line-height: 1.2;
      letter-spacing: 0.02em;
      font-variant-numeric: tabular-nums;
      font-family: "Consolas", "Courier New", monospace;
      border: 1px solid rgba(255, 205, 205, 0.5);
      box-shadow: 0 2px 6px rgba(0,0,0,0.25);
    }
    .camLabel {
      opacity: 0.95;
    }
    .camIcon {
      width: 20px;
      height: 20px;
      display: block;
      color: currentColor;
    }
    /* Bottom HUD: compass (racchip) bottom-right; telemetry on a single orbit (no duplicate strips). */
    /* Bottom HUD: lock compass to bottom-right; telemetry strip pinned next to it (Qt WebEngine-safe). */
    #mapFooterHud {
      position: absolute;
      right: 10px;
      bottom: 2px;
      z-index: 1210;
      width: 312px;
      height: 312px;
      pointer-events: none;
      font-family: "Segoe UI", Arial, sans-serif;
      --compass-size: 176px;
      --hud-gap: 12px; /* minimum spacing between telemetry and compass */
    }
    #telemetryLeftStack {
      display: flex;
      flex-direction: column;
      gap: 6px;
      position: absolute;
      /* Anchor relative to the compass circle (not the whole 312px HUD box).
         HUD box: 312px; compass: 176px centered => compass left edge is 68px from HUD left.
         We want telemetry's RIGHT edge to sit left of compass edge by --hud-gap.
         For a HUD box of width W and compass diameter C, this is:
           right = (W + C) / 2 + gap
      */
      left: auto;
      right: calc((100% + var(--compass-size)) / 2 + var(--hud-gap));
      bottom: 44px;              /* move strip up */
      transform: none;
      pointer-events: none;
    }
    #telemetryStrip {
      display: grid;
      grid-template-columns: repeat(3, max-content);
      gap: 6px 8px;
      padding: 6px 10px;
      border-radius: 12px;
      background: rgba(26, 33, 45, 0.84);
      border: 1px solid rgba(80, 92, 118, 0.42);
      box-shadow: 0 1px 4px rgba(0,0,0,0.28);
      backdrop-filter: blur(6px);
      -webkit-backdrop-filter: blur(6px);
      pointer-events: none;
    }
    .telStackItem {
      display: inline-flex;
      align-items: center;
      justify-content: flex-start;
      gap: 8px;
      padding: 2px;
      border-radius: 0;
      background: transparent;
      color: #dce5f5;
      font-size: 15px;
      line-height: 1.25;
      white-space: nowrap;
      box-shadow: none;
      border: 0;
      min-width: 0;
      pointer-events: none;
    }
    .telStackItem .telemetryIcon {
      font-size: 15px;
      opacity: 0.95;
    }
    .telStackItem .telemetryIconHuman {
      font-size: 16px;
      opacity: 0.95;
    }
    #compassHud {
      position: absolute;
      right: 0;
      bottom: -18px; /* move only compass down more */
      width: 312px;
      height: 312px;
      flex-shrink: 0;
      pointer-events: none;
      --orbit-r: 122px;
    }
    .telOrbitItem {
      position: absolute;
      left: 50%;
      top: 50%;
      z-index: 5;
      transform: translate(-50%, -50%) rotate(var(--oa, 0deg)) translateY(calc(-1 * var(--orbit-r))) rotate(calc(-1 * var(--oa, 0deg)));
      display: inline-flex;
      align-items: center;
      gap: 5px;
      padding: 4px 9px;
      border-radius: 8px;
      background: rgba(26, 33, 45, 0.94);
      color: #dce5f5;
      font-size: 13px;
      line-height: 1.25;
      white-space: nowrap;
      box-shadow: 0 1px 4px rgba(0,0,0,0.35);
      border: 1px solid rgba(80, 92, 118, 0.45);
      pointer-events: none;
    }
    .telemetryIcon {
      font-size: 16px;
      line-height: 1;
    }
    .telemetryIconHuman {
      font-size: 19px;
      line-height: 1;
    }
    #compassHud #compass {
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      width: var(--compass-size);
      height: var(--compass-size);
      flex-shrink: 0;
      background: transparent;
      display: flex;
      justify-content: center;
      align-items: center;
      pointer-events: none;
      z-index: 3;
    }
    #compass::before {
      content: "";
      position: absolute;
      inset: 0;
      border-radius: 50%;
      background:
        conic-gradient(
          from -90deg,
          rgba(255,255,255,0.97) 0deg 28deg, transparent 28deg 45deg,
          rgba(255,255,255,0.97) 45deg 73deg, transparent 73deg 90deg,
          rgba(255,255,255,0.97) 90deg 118deg, transparent 118deg 135deg,
          rgba(255,255,255,0.97) 135deg 163deg, transparent 163deg 180deg,
          rgba(255,255,255,0.97) 180deg 208deg, transparent 208deg 225deg,
          rgba(255,255,255,0.97) 225deg 253deg, transparent 253deg 270deg,
          rgba(255,255,255,0.97) 270deg 298deg, transparent 298deg 315deg,
          rgba(255,255,255,0.97) 315deg 343deg, transparent 343deg 360deg
        );
      -webkit-mask: radial-gradient(circle, transparent 0 68px, #000 68px 85px, transparent 85px);
      mask: radial-gradient(circle, transparent 0 68px, #000 68px 85px, transparent 85px);
      pointer-events: none;
      opacity: 0.98;
    }
    #compassInner {
      position:relative; width:152px; height:152px; border-radius:76px;
      background: rgba(22, 26, 34, 0.96);
      border: 2px solid rgba(148, 160, 180, 0.32);
      box-shadow: 0 2px 8px rgba(0,0,0,0.35);
      overflow: hidden;
    }
    #compassInner::before {
      content: "";
      position: absolute;
      inset: 4px;
      border-radius: 50%;
      background:
        repeating-conic-gradient(
          from 0deg,
          rgba(240, 245, 255, 0.88) 0deg 2deg,
          transparent 2deg 30deg
        );
      -webkit-mask: radial-gradient(circle, transparent 0 52px, #000 52px 56px, transparent 56px);
      mask: radial-gradient(circle, transparent 0 52px, #000 52px 56px, transparent 56px);
      pointer-events: none;
    }
    .compassCard {
      position:absolute;
      font-size:16px;
      line-height:1;
      color:#f1f5ff;
      font-weight:600;
      text-shadow:0 1px 0 rgba(0,0,0,0.35);
      z-index: 3;
    }
    #cN { top:11px; left:71px; }
    #cE { top:69px; right:12px; }
    #cS { bottom:12px; left:71px; font-size:15px; }
    #cW { top:69px; left:12px; }
    #compassDeg {
      position:absolute;
      left:0; right:0; bottom:30px;
      text-align:center;
      font-size:13px;
      font-weight:600;
      color:#f4f7ff;
      letter-spacing:0.02em;
      z-index: 3;
    }
    #needle {
      position:absolute;
      left:50px;
      top:46px;
      width:52px;
      height:52px;
      transform-origin: 26px 26px;
      filter: drop-shadow(0 1px 1px rgba(0,0,0,0.45));
      z-index: 2;
    }
    #needle::before {
      content:"";
      position:absolute;
      left:7px;
      top:2px;
      width:38px;
      height:48px;
      background: linear-gradient(180deg, #ff5b4e, #f04336);
      border:2px solid rgba(244,248,255,0.92);
      clip-path: polygon(50% 0%, 100% 58%, 67% 56%, 50% 100%, 33% 56%, 0% 58%);
    }
    /* Vehicle on map: high-contrast heading chevron (matches product reference). */
    .vgcs-vehicle-marker {
      background: transparent !important;
      border: none !important;
    }
    .vgcs-vehicle-marker-inner {
      width: 30px;
      height: 30px;
      margin: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      transform-origin: 50% 50%;
      filter: drop-shadow(0 1px 1px rgba(0,0,0,0.35));
    }
    .vgcs-vehicle-marker-inner svg {
      display: block;
    }
    .vgcs-wp-divicon {
      background: transparent !important;
      border: none !important;
    }
    .vgcs-wp-pin {
      position: relative;
      width: 26px;
      height: 26px;
    }
    .vgcs-wp-disc {
      position: absolute;
      left: 0;
      top: 0;
      width: 26px;
      height: 26px;
      border-radius: 50%;
      background: #facc15;
      border: 2px solid #111827;
      box-sizing: border-box;
    }
    .vgcs-wp-num {
      position: absolute;
      left: 0;
      top: 0;
      width: 26px;
      height: 26px;
      display: flex;
      align-items: center;
      justify-content: center;
      font: 700 12px/1 system-ui, sans-serif;
      color: #111827;
      pointer-events: none;
    }
    .planWpDetails {
      margin: 10px 0 12px 0;
      padding: 10px;
      border-radius: 10px;
      background: rgba(18, 20, 26, 0.96);
      border: 1px solid rgba(70, 76, 92, 0.85);
      overflow: visible;
    }
    .planWpDetailsTitle {
      font-weight: 800;
      color: #e5e7eb;
      margin: 0 0 8px 0;
      font-size: 12px;
      letter-spacing: 0.02em;
      text-transform: uppercase;
    }
    .planWpRow {
      display: grid;
      grid-template-columns: 56px 1fr;
      gap: 8px;
      align-items: center;
      padding: 6px 0;
      border-bottom: 1px solid rgba(70, 76, 92, 0.55);
    }
    .planWpRow:last-child { border-bottom: none; }
    .planWpRow--start .planWpLabel { color: #e7d494; }
    .planWpStartHint {
      font-size: 11px;
      color: #94a3b8;
      line-height: 1.35;
      padding: 0 2px;
    }
    .planWpLabel {
      color: #cbd5e1;
      font-weight: 800;
      font-size: 12px;
    }
    .planWpFields {
      display: grid;
      grid-template-columns: 1fr;
      gap: 6px;
    }
    .planWpField {
      display: flex;
      align-items: center;
      gap: 6px;
      background: rgba(26, 28, 36, 0.85);
      border: 1px solid rgba(70, 76, 92, 0.8);
      border-radius: 10px;
      padding: 7px 10px;
      min-width: 0;
      width: 100%;
      box-sizing: border-box;
    }
    .planWpField input {
      flex: 1;
      background: transparent;
      border: none;
      outline: none;
      color: #f8fafc;
      font-weight: 700;
      font-size: 13px;
      min-width: 0;
    }
    .planWpUnit {
      color: #94a3b8;
      font-weight: 800;
      font-size: 12px;
      flex-shrink: 0;
      white-space: nowrap;
    }
    /* Mission tab: sequence list (Takeoff card + pattern template row), reference UI */
    .planMissionSequence {
      margin-bottom: 14px;
      max-height: 42vh;
      overflow-y: auto;
      padding-right: 2px;
    }
    .planSeqCard {
      border-radius: 6px;
      overflow: hidden;
      border: 1px solid rgba(70, 76, 92, 0.95);
      background: rgba(22, 24, 30, 0.98);
      margin-bottom: 10px;
    }
    .planSeqCardHead {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      min-height: 40px;
      padding: 0 8px 0 6px;
      background: linear-gradient(180deg, #6a5cb8 0%, #4d5696 100%);
      color: #f8fafc;
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.02em;
    }
    .planSeqCardTitle {
      flex: 1;
      text-align: center;
    }
    .planSeqIconBtn {
      width: 32px;
      height: 32px;
      border: none;
      border-radius: 6px;
      background: rgba(255,255,255,0.12);
      color: #f1f5ff;
      font-size: 14px;
      line-height: 1;
      cursor: pointer;
      flex-shrink: 0;
    }
    .planSeqIconBtn:disabled {
      opacity: 0.35;
      cursor: default;
    }
    .planSeqIconBtn:not(:disabled):hover {
      background: rgba(255,255,255,0.22);
    }
    .planSeqCardBody {
      padding: 12px;
      background: #0c0c0e;
    }
    .planSeqCardDesc {
      font-size: 12px;
      line-height: 1.45;
      color: rgba(220, 224, 235, 0.9);
      margin: 0 0 12px;
    }
    #planSeqPatternRow {
      display: none;
      align-items: center;
      gap: 12px;
      margin-bottom: 10px;
      padding: 11px 12px;
      border-radius: 6px;
      background: rgba(36, 38, 48, 0.96);
      border: 2px solid rgba(80, 86, 102, 0.75);
      box-sizing: border-box;
    }
    #planSeqPatternRow.planSeqPatternRow--visible {
      display: flex;
    }
    #planSeqPatternRow.planSeqPatternRow--focus {
      border-color: #e11d48;
      box-shadow: 0 0 0 1px rgba(225, 29, 72, 0.35);
    }
    .planSeqPatternGlyph {
      width: 30px;
      height: 30px;
      border-radius: 50%;
      background: rgba(72, 78, 96, 0.95);
      color: #f9fafb;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 15px;
      font-weight: 800;
      flex-shrink: 0;
    }
    .planSeqPatternLabel {
      font-size: 13px;
      font-weight: 600;
      color: #f3f4f6;
    }
    .planSeqRtlBtn {
      width: 100%;
      box-sizing: border-box;
      min-height: 38px;
      margin-top: 2px;
      border-radius: 6px;
      border: 1px solid rgba(90, 96, 118, 0.85);
      background: rgba(52, 56, 68, 0.96);
      color: #e8eaef;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
    }
    .planSeqRtlBtn:hover {
      background: rgba(64, 68, 82, 0.98);
    }
    /* Empty plan mission style (compact "Mission Start" panel). */
    #planTabPanelMission.planMissionEmpty .planMissionSequence {
      margin-bottom: 8px;
      max-height: none;
      overflow: visible;
      padding-right: 0;
    }
    #planTabPanelMission.planMissionEmpty .planSeqCard {
      border: none;
      background: transparent;
      margin-bottom: 0;
    }
    #planTabPanelMission.planMissionEmpty .planSeqCardHead,
    #planTabPanelMission.planMissionEmpty .planSeqCardDesc,
    #planTabPanelMission.planMissionEmpty #planSeqPatternRow,
    #planTabPanelMission.planMissionEmpty #planSeqRtlBtn {
      display: none !important;
    }
    #planTabPanelMission.planMissionEmpty .planSeqCardBody {
      padding: 0;
      background: transparent;
    }
    #planTabPanelMission.planMissionEmpty .planFieldLabel {
      margin-top: 6px;
      margin-bottom: 4px;
    }
    #planTabPanelMission.planMissionEmpty .planRailSelect,
    #planTabPanelMission.planMissionEmpty .planRailInput {
      min-height: 30px;
    }
    #planSeqCompactList {
      display: none;
      margin-top: 0;
      flex-direction: column;
      gap: 10px;
    }
    #planTabPanelMission.planMissionStack #planSeqCompactList {
      display: flex;
      margin-top: 10px;
    }
    /* Survey stack uses independent mission tabs. Bodies are toggled by JS. */
    #planTabPanelMission.planMissionStack .planSeqCardBody,
    #planTabPanelMission.planMissionStack #planVehicleDetails,
    #planTabPanelMission.planMissionStack #planLaunchDetails {
      display: none;
    }
    /* Each step: title row + detail stacked; groups are spaced in #planSeqCompactList */
    .planSeqCompactGroup {
      display: flex;
      flex-direction: column;
      gap: 0;
      width: 100%;
    }
    .planSeqCompactTab {
      min-height: 34px;
      border-radius: 6px;
      border: 1px solid rgba(86, 93, 116, 0.9);
      background: #4d5170;
      color: #f5f7ff;
      display: inline-flex;
      align-items: center;
      justify-content: flex-start;
      gap: 8px;
      padding: 0 10px;
      font-size: 14px;
      font-weight: 600;
      box-sizing: border-box;
      width: 100%;
      cursor: pointer;
      user-select: none;
      text-align: left;
    }
    .planSeqCompactTab.is-active {
      background: #616790;
      border-color: rgba(120, 132, 170, 0.95);
      box-shadow: 0 0 0 1px rgba(146, 160, 201, 0.24) inset;
    }
    .planSeqCompactRow {
      min-height: 34px;
      border-radius: 6px;
      border: 2px solid rgba(225, 29, 72, 0.9);
      background: rgba(32, 35, 44, 0.9);
      color: #e8eaef;
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 0 10px;
      font-size: 14px;
      font-weight: 600;
      box-sizing: border-box;
    }
    .planSeqCompactTakeoffCard {
      border-radius: 6px;
      border: 2px solid rgba(225, 29, 72, 0.9);
      background: rgba(18, 21, 29, 0.9);
      overflow: hidden;
      display: none;
    }
    .planSeqCompactTakeoffCard.is-active {
      display: block;
      width: 100%;
      box-sizing: border-box;
      box-shadow: 0 0 0 1px rgba(225, 29, 72, 0.32);
    }
    .planSeqCompactTakeoffHead {
      min-height: 34px;
      border-bottom: 1px solid rgba(99, 108, 136, 0.62);
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 0 8px;
      color: #f5f7ff;
      font-size: 14px;
      font-weight: 700;
      background: #4d5170;
    }
    .planSeqCompactTakeoffHeadTitle {
      flex: 1;
    }
    .planSeqCompactHeadBtn {
      width: 24px;
      height: 24px;
      border-radius: 5px;
      border: none;
      background: rgba(88, 96, 120, 0.42);
      color: #f5f7ff;
      font-size: 13px;
      font-weight: 700;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 0;
      cursor: default;
      opacity: 0.9;
    }
    .planSeqCompactTakeoffBody {
      padding: 10px 10px 8px;
      color: #ffffff;
      font-size: 12px;
      line-height: 1.35;
      display: none;
    }
    .planSeqCompactTakeoffCard.is-active .planSeqCompactTakeoffBody { display: block; }
    .planSeqCompactTakeoffBody p {
      margin: 0 0 8px;
    }
    .planSeqCompactDoneBtn {
      width: 100%;
      min-height: 34px;
      border-radius: 6px;
      border: 1px solid rgba(120, 128, 148, 0.65);
      background: rgba(78, 84, 104, 0.75);
      color: #f5f7ff;
      font-size: 14px;
      font-weight: 600;
      cursor: default;
    }
    .planSeqCompactGlyph {
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: rgba(72, 78, 96, 0.95);
      display: inline-flex;
      align-items: center;
      justify-content: center;
      font-size: 12px;
      font-weight: 800;
      flex: 0 0 auto;
    }
    .planSeqCompactRow.is-active {
      box-shadow: 0 0 0 1px rgba(225, 29, 72, 0.32);
    }
    .planSeqCompactBody {
      display: none;
      margin-top: 0;
      margin-bottom: 0;
      border: 2px solid rgba(225, 29, 72, 0.9);
      border-radius: 6px;
      background: rgba(18, 21, 29, 0.9);
      color: #f5f7ff;
      padding: 8px 10px;
      font-size: 12px;
      line-height: 1.35;
    }
    .planSeqCompactBody.is-active {
      display: block;
    }
    #planTabPanelMission.planMissionStack #planSeqRtlBtn {
      display: none !important;
    }
    /* Mission Start stack (Takeoff / Survey / RTL): edge-to-edge black band, square corners */
    #planTabPanelMission.planMissionStack .planSectionBody {
      padding-left: 0;
      padding-right: 0;
      padding-top: 10px;
      padding-bottom: 10px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen .planSectionBody {
      padding-left: 12px;
      padding-right: 12px;
      padding-top: 12px;
      padding-bottom: 12px;
      gap: 8px;
    }
    #planTabPanelMission.planMissionStack .planMissionSequence {
      display: none !important;
      margin-bottom: 0 !important;
    }
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen .planMissionSequence {
      display: block !important;
      margin-bottom: 0 !important;
    }
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen .planSeqCard {
      border: none;
      background: transparent;
      margin-bottom: 0;
    }
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen .planSeqCardBody {
      display: block !important;
      padding: 0;
      background: transparent;
    }
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen #planVehicleDetails,
    #planTabPanelMission.planMissionStack.planMissionDetailsOpen #planLaunchDetails {
      display: block;
    }
    #planTabPanelMission.planMissionStack #planMissionSectionHeader {
      cursor: pointer;
      user-select: none;
    }
    #planTabPanelMission.planMissionStack #planMissionSectionHeader:hover {
      background: #616790;
    }
    #planTabPanelMission.planMissionStack #planStartMissionBtn {
      display: block !important;
    }
    /* Stack tabs already label Takeoff; hide duplicate bar on the detail card */
    #planTabPanelMission.planMissionStack .planSeqCompactTakeoffHead {
      display: none !important;
    }
    #planTabPanelMission.planMissionStack .planSeqCompactGroup,
    #planTabPanelMission.planMissionStack .planSeqCompactTab,
    #planTabPanelMission.planMissionStack .planSeqCompactTakeoffCard,
    #planTabPanelMission.planMissionStack .planSeqCompactBody,
    #planTabPanelMission.planMissionStack .planSeqCompactDoneBtn,
    #planTabPanelMission.planMissionStack .planSeqCompactHeadBtn,
    #planTabPanelMission.planMissionStack .planSeqCompactRow {
      border-radius: 0;
    }
  </style>
</head>
<body>
  <div id="mapWrap">
    <div id="map2d"></div>
    <div id="map3d"></div>
    <div class="overlay" id="linkBanner">
      <img id="linkBannerLogo" src="__LOGO_SRC__" alt="Vama logo"/>
      <div id="linkBannerDisconnected">
        <span id="linkBannerText">Disconnected - Click to manually connect 💬</span>
      </div>
      <div id="linkBannerConnected">
        <span class="hdrPill"><span class="hdrReadyText" id="hdrReadyText">Ready To Fly</span></span>
        <span class="hdrSep"></span>
        <span class="hdrPill"><img class="hdrIcon" src="__ICON_HOLD_SRC__" alt="Hold"/><span id="hdrModeText">Hold</span></span>
        <span class="hdrSep"></span>
        <span class="hdrPill" id="hdrVehiclePill"><img class="hdrIcon hdrIconBroadcast" src="__ICON_LINK_SRC__" alt="Vehicle Message"/><span id="hdrVehicleMsg">Vehicle Msg</span></span>
        <span class="hdrSep"></span>
        <span class="hdrPill" id="hdrGpsPill">
          <img id="hdrGpsIcon" class="hdrIcon hdrIconSmall" src="__ICON_GPS_SRC__" alt="GPS"
               onerror="this.style.display='none'; var e=document.getElementById('hdrGpsEmoji'); if(e) e.style.display='inline';"/>
          <span id="hdrGpsEmoji" style="display:none; font-weight:700;">GPS</span>
          <span class="hdrTinyStack" id="hdrGpsStack"><span id="hdrGpsSat">10</span><span id="hdrGpsHdop">0.7</span></span>
        </span>
        <span class="hdrSep"></span>
        <span class="hdrPill" id="hdrBatteryPill">
          <img id="hdrBatIcon" class="hdrIcon" src="__ICON_BATTERY_SRC__" alt="Battery"
               onerror="this.style.display='none'; var e=document.getElementById('hdrBatEmoji'); if(e) e.style.display='inline';"/>
          <span id="hdrBatEmoji" style="display:none; font-weight:700;">BAT</span>
          <span id="hdrBatteryText">100%</span>
        </span>
        <span class="hdrSep"></span>
        <span class="hdrPill"><img class="hdrIcon" src="__ICON_REMOTE_ID_SRC__" alt="Remote ID"/><span id="hdrRemoteIdText">ID</span></span>
      </div>
      <button id="hdrMapModeBtn" type="button">3D</button>
    </div>
    <div class="overlay" id="actionRail">
      <div class="actionBtn" id="actionTakeoff">⬆<div>Takeoff</div></div>
      <div class="actionBtn" id="actionReturn">↩<div>Return</div></div>
    </div>
    <div id="planFlightLayer">
      <div id="planFlightTopBar">
        <span id="planExit">&lt; Exit Plan</span>
        <button id="planBarUpload" type="button" disabled>Upload</button>
        <div class="pfGroup">
          <span class="pfLabel">Selected Waypoint</span>
          <span class="pfMetric">Alt diff: <b id="pfAltDiff">0.0 ft</b></span>
          <span class="pfMetric">Gradient: <b id="pfGradient">-.-</b></span>
        </div>
        <div class="pfGroup">
          <span class="pfLabel">&nbsp;</span>
          <span class="pfMetric">Azimuth: <b id="pfAzimuth">0</b></span>
          <span class="pfMetric">Heading: <b id="pfHeading">nan</b></span>
        </div>
        <div class="pfGroup">
          <span class="pfLabel">&nbsp;</span>
          <span class="pfMetric">Dist prev WP: <b id="pfDistPrevWp">0.0 ft</b></span>
          <span class="pfMetric pfMetricGhost">Heading: <b>nan</b></span>
        </div>
        <span class="pfSpacer"></span>
        <div class="pfGroup">
          <span class="pfLabel">Total Mission</span>
          <span class="pfMetric">Distance: <b id="pfMissionDistance">0 ft</b></span>
          <span class="pfMetric">Time: <b id="pfMissionTime">00:00:00</b></span>
        </div>
        <div class="pfGroup">
          <span class="pfLabel">&nbsp;</span>
          <span class="pfMetric">Max telem dist: <b id="pfMaxTelemDist">0 ft</b></span>
          <span class="pfMetric pfMetricGhost">Time: <b>00:00:00</b></span>
        </div>
      </div>
      <div id="planWorkspace">
        <div id="planFlightToolRail">
          <div class="planToolBtn active" data-tool="File"><span class="planToolIcon">↻</span><span>File</span></div>
          <div class="planToolBtn" data-tool="Takeoff"><span class="planToolIcon">↑</span><span>Takeoff</span></div>
          <div class="planToolBtn" data-tool="Waypoint"><span class="planToolIcon">⊕</span><span>Waypoint</span></div>
          <div class="planToolBtn" data-tool="ROI"><span class="planToolIcon">⊙</span><span>ROI</span></div>
          <div class="planToolBtn" data-tool="Pattern"><span class="planToolIcon">▦</span><span>Pattern</span></div>
          <div class="planToolBtn" data-tool="Return"><span class="planToolIcon">↩</span><span>Return</span></div>
          <div class="planToolBtn" data-tool="Center"><span class="planToolIcon">✦</span><span>Center</span></div>
        </div>
        <div id="planCenterPanel">
          <div id="planFileFlyout">
            <div class="planFileSection">
              <div class="planFileSectionTitle">Create Plan</div>
              <div class="planFileCardGrid">
                <div class="planTplCard" id="planTplEmpty" role="button" tabindex="0">
                  <div class="planTplPrev">
                    <img class="planTplPrevImg" src="__PLAN_TPL_EMPTY_SRC__" alt="" onerror="this.style.display='none'"/>
                  </div>
                  <div class="planTplLabel">Empty Plan</div>
                </div>
                <div class="planTplCard" id="planTplSurvey" role="button" tabindex="0">
                  <div class="planTplPrev">
                    <img class="planTplPrevImg" src="__PLAN_TPL_SURVEY_SRC__" alt="" onerror="this.style.display='none'"/>
                  </div>
                  <div class="planTplLabel">Survey</div>
                </div>
                <div class="planTplCard" id="planTplCorridor" role="button" tabindex="0">
                  <div class="planTplPrev">
                    <img class="planTplPrevImg" src="__PLAN_TPL_CORRIDOR_SRC__" alt="" onerror="this.style.display='none'"/>
                  </div>
                  <div class="planTplLabel">Corridor Scan</div>
                </div>
                <div class="planTplCard" id="planTplStructure" role="button" tabindex="0">
                  <div class="planTplPrev">
                    <img class="planTplPrevImg" src="__PLAN_TPL_STRUCTURE_SRC__" alt="" onerror="this.style.display='none'"/>
                  </div>
                  <div class="planTplLabel">Structure Scan</div>
                </div>
              </div>
            </div>
            <div class="planFileSection">
              <div class="planFileSectionTitle">Storage</div>
              <div class="planFileBtnRow">
                <button type="button" class="planFileBtn planFileBtnPrimary" id="planStorageOpen">Open...</button>
                <button type="button" class="planFileBtn" id="planStorageSave" disabled>Save</button>
                <button type="button" class="planFileBtn" id="planStorageSaveAs" disabled>Save As...</button>
              </div>
              <button type="button" class="planFileBtn planFileBtnWide" id="planStorageKml" disabled>Save Mission Waypoints As KML...</button>
            </div>
            <div class="planFileSection">
              <div class="planFileSectionTitle">Vehicle</div>
              <div class="planFileBtnRow">
                <button type="button" class="planFileBtn" id="planVehicleUpload" disabled>Upload</button>
                <button type="button" class="planFileBtn planFileBtnSecondary" id="planVehicleDownload">Download</button>
                <button type="button" class="planFileBtn planFileBtnSecondary" id="planVehicleClear">Clear</button>
              </div>
            </div>
          </div>
          <div id="planOtherToolPanel" style="display:none">
            <div id="planOtherToolHint">Select a tool from the rail.</div>
          </div>
        </div>
        <div id="planRightPanel">
          <div id="planTabs" role="tablist" aria-label="Plan configuration">
            <button class="planTab active" type="button" role="tab" id="planTabBtnMission"
              aria-selected="true" aria-controls="planTabPanelMission" tabindex="0" data-plan-tab="mission">Mission</button>
            <button class="planTab" type="button" role="tab" id="planTabBtnFence"
              aria-selected="false" aria-controls="planTabPanelFence" tabindex="-1" data-plan-tab="fence">Fence</button>
            <button class="planTab" type="button" role="tab" id="planTabBtnRally"
              aria-selected="false" aria-controls="planTabPanelRally" tabindex="-1" data-plan-tab="rally">Rally</button>
          </div>
          <div id="planTabPanelMission" class="planTabBody" role="tabpanel" aria-labelledby="planTabBtnMission">
            <div id="planSection">
              <div class="planSectionHeader" id="planMissionSectionHeader" role="button" tabindex="0" aria-label="Mission Start">Mission</div>
              <div class="planSectionBody">
                <div class="planMissionSequence">
                  <div class="planSeqCard planSeqCard--takeoff">
                    <div class="planSeqCardHead">
                      <button type="button" class="planSeqIconBtn" id="planSeqTakeoffTrash" disabled title="Remove (not available)">🗑</button>
                      <span class="planSeqCardTitle">Takeoff</span>
                      <button type="button" class="planSeqIconBtn" id="planSeqTakeoffMenu" disabled title="Options">☰</button>
                    </div>
                    <div class="planSeqCardBody">
                      <p class="planSeqCardDesc">Take off from the ground and ascend to specified altitude.</p>
                      <div class="planFieldLabel">All Altitudes</div>
                      <select id="planAltReferenceSelect" class="planRailSelect" aria-label="Altitude reference">
                        <option value="rel" selected>Altitude Relative To Launch</option>
                        <option value="amsl">AMSL</option>
                        <option value="agl">AGL</option>
                      </select>
                      <div class="planFieldLabel">Initial Waypoint Alt</div>
                      <div class="planRailInput">
                        <input id="planInitialWpAltInput" type="text" value="164.0" inputmode="decimal" autocomplete="off" />
                        <span class="planRailUnit">ft</span>
                      </div>
                    </div>
                  </div>
                  <div id="planSeqPatternRow" class="planSeqPatternRow" role="group" aria-label="Pattern template">
                    <span class="planSeqPatternGlyph" aria-hidden="true">?</span>
                    <span id="planSeqPatternLabel" class="planSeqPatternLabel">Survey</span>
                  </div>
                  <button type="button" id="planSeqRtlBtn" class="planSeqRtlBtn">Return To Launch</button>
                </div>
                <button id="planStartMissionBtn" type="button">Start Mission</button>
                <div id="planWpDetails" class="planWpDetails" style="display:none">
                  <div class="planWpDetailsTitle">Waypoints &amp; start</div>
                  <div id="planWpDetailsList"></div>
                </div>
                <details class="planRailDetails" id="planVehicleDetails">
                  <summary>Vehicle Info <span class="planRailChev">▼</span></summary>
                  <div class="planRailDetailsInner">
                    <div class="planKvRow"><span>Firmware</span><b id="planVehicleFirmwareVal">ArduPilot</b></div>
                    <div class="planKvRow"><span>Vehicle</span><b id="planVehicleTypeVal">Quadrotor</b></div>
                    <p class="planNoteMission">The following speed values are used to calculate total mission time. They do not affect the flight speed for the mission.</p>
                    <div class="planFieldLabel">Hover speed</div>
                    <div class="planRailInput">
                      <input id="planHoverSpeedInput" type="text" value="11.18" inputmode="decimal" autocomplete="off" />
                      <span class="planRailUnit">mph</span>
                    </div>
                  </div>
                </details>
                <details class="planRailDetails" id="planLaunchDetails">
                  <summary>Launch Position <span class="planRailChev">▼</span></summary>
                  <div class="planRailDetailsInner">
                    <div class="planFieldLabel">Altitude</div>
                    <div class="planRailInput">
                      <input id="planLaunchAltInput" type="text" value="0.0" inputmode="decimal" autocomplete="off" />
                      <span class="planRailUnit">ft</span>
                    </div>
                    <p class="planHelpMuted">Actual position set by vehicle at flight time.</p>
                    <div class="planKvRow"><span>Lat</span><b id="planLaunchLatVal">—</b></div>
                    <div class="planKvRow"><span>Lon</span><b id="planLaunchLonVal">—</b></div>
                    <button id="planSetLaunchMapCenterBtn" type="button">Set To Map Center</button>
                  </div>
                </details>
              </div>
            </div>
            <div id="planSeqCompactList" aria-hidden="true" role="group" aria-label="Mission sequence steps">
                  <div class="planSeqCompactGroup">
                    <div class="planSeqCompactTab" id="planSeqCompactTakeoffTab" data-stack-tab="takeoff" role="tab" tabindex="0">
                      <span class="planSeqCompactGlyph">?</span>
                      <span>Takeoff</span>
                    </div>
                    <div class="planSeqCompactTakeoffCard" id="planSeqCompactTakeoffCard">
                      <div class="planSeqCompactTakeoffHead">
                        <span class="planSeqCompactGlyph">?</span>
                        <span class="planSeqCompactHeadBtn" aria-hidden="true">🗑</span>
                        <span class="planSeqCompactTakeoffHeadTitle">Takeoff</span>
                        <span class="planSeqCompactHeadBtn" aria-hidden="true">☰</span>
                      </div>
                      <div class="planSeqCompactTakeoffBody">
                        <p>Take off from the ground and ascend to specified altitude.</p>
                        <p>Move “T” Takeoff to the climbout location.</p>
                        <p>Ensure clear of obstacles and into the wind.</p>
                        <button type="button" class="planSeqCompactDoneBtn" disabled>Done</button>
                      </div>
                    </div>
                  </div>
                  <div class="planSeqCompactGroup">
                    <div class="planSeqCompactTab" id="planSeqCompactSurveyTab" data-stack-tab="survey" role="tab" tabindex="0">
                      <span class="planSeqCompactGlyph">?</span>
                      <span id="planSeqCompactSurveyLabel">Survey</span>
                    </div>
                    <div class="planSeqCompactBody" id="planSeqCompactSurveyBody">Survey pattern selected.</div>
                  </div>
                  <div class="planSeqCompactGroup">
                    <div class="planSeqCompactTab" id="planSeqCompactRtlTab" data-stack-tab="rtl" role="tab" tabindex="0">Return To Launch</div>
                    <div class="planSeqCompactBody" id="planSeqCompactRtlBody">Return leg will be appended after mission actions.</div>
                  </div>
            </div>
          </div>
          <div id="planTabPanelFence" class="planTabBody" role="tabpanel" aria-labelledby="planTabBtnFence" hidden>
            <div class="planSectionHeader">GeoFence</div>
            <div class="planSectionBody planSectionBody--fence">
              <p class="planGeoLead">GeoFencing allows you to set a virtual fence around the area you want to fly in.</p>
              <p class="planTabHint" style="margin-bottom:14px;">Draw a polygon with <b>Polygon Fence</b>, then use <b>Upload fence</b> on the dashboard to send it to the vehicle.</p>
              <div class="planGeoBlock">
                <div class="planGeoTitle">Insert GeoFence</div>
                <div class="planGeoBtnStack">
                  <button type="button" class="planGeoBtn" id="planFenceRoiBtn">Polygon Fence</button>
                  <button type="button" class="planGeoBtn" id="planFenceCircularBtn" disabled title="Not available in this build">Circular Fence</button>
                </div>
              </div>
              <div class="planGeoBlock">
                <div class="planGeoTitle">Polygon Fences</div>
                <div class="planGeoStatus" id="planGeoPolyStatus">None</div>
              </div>
              <div class="planGeoBlock">
                <div class="planGeoTitle">Circular Fences</div>
                <div class="planGeoStatus" id="planGeoCircleStatus">None</div>
              </div>
              <div class="planGeoBlock">
                <div class="planGeoTitle">Breach Return Point</div>
                <button type="button" class="planGeoBtn" id="planBreachReturnBtn" disabled title="Not available in this build">Add Breach Return Point</button>
              </div>
            </div>
          </div>
          <div id="planTabPanelRally" class="planTabBody" role="tabpanel" aria-labelledby="planTabBtnRally" hidden>
            <div class="planSectionHeader">Rally Points</div>
            <div class="planSectionBody planSectionBody--fence" style="padding:12px;">
              <div class="planRallyInfo">Rally Points provide alternate landing points when performing a Return to Launch (RTL). Rally editing is not implemented in M2.</div>
            </div>
          </div>
        </div>
      </div>
    </div>
    <div class="overlay" id="cameraRail">
      <div id="cameraTopRow">
        <div class="camSmallBtn active" id="camVideoBtn" title="Video mode">
          <svg class="camIcon" viewBox="0 0 24 24" aria-hidden="true">
            <rect x="3" y="7" width="12" height="10" rx="2" fill="currentColor"></rect>
            <polygon points="16,10 21,8 21,16 16,14" fill="currentColor"></polygon>
          </svg>
        </div>
        <div class="camSmallBtn" id="camPhotoBtn" title="Take photo">
          <svg class="camIcon" viewBox="0 0 24 24" aria-hidden="true">
            <path d="M8 6h8l1.2 2H20a2 2 0 0 1 2 2v7a3 3 0 0 1-3 3H5a3 3 0 0 1-3-3v-7a2 2 0 0 1 2-2h2.8L8 6z" fill="currentColor"></path>
            <circle cx="12" cy="13.5" r="3.2" fill="rgba(39,47,61,0.95)"></circle>
          </svg>
        </div>
      </div>
      <div id="camRecordBtn" title="Start/Stop recording"><div id="camRecordDot"></div></div>
      <div id="camTimer">00:00:00</div>
      <div class="camSmallBtn" id="camSettingsBtn" title="Camera settings"><span class="camLabel">⚙</span></div>
    </div>
    <div class="overlay" id="videoPreview">
      <img id="videoPreviewImg" alt="Video preview" />
      <div id="videoPreviewPlaceholder">Video</div>
    </div>
    <div id="mapFooterHud" aria-hidden="false">
      <div id="telemetryLeftStack" aria-hidden="true">
        <div id="telemetryStrip">
          <div class="telStackItem"><span class="telemetryIcon">↕</span><span class="telRow1Alt">0.0 ft</span></div>
          <div class="telStackItem"><span class="telemetryIcon">↑</span><span class="telRow1Mph">0.0 mph</span></div>
          <div class="telStackItem"><span class="telemetryIcon">⏱</span><span class="telRow1Time">00:00:00</span></div>
          <div class="telStackItem"><span class="telemetryIcon telemetryIconHuman">&#128100;&#65038;</span><span class="telRow2Msl">0.0 ft</span></div>
          <div class="telStackItem"><span class="telemetryIcon">→</span><span class="telRow2Mph">0.0 mph</span></div>
          <div class="telStackItem"><span class="telemetryIcon">↳</span><span class="telRow2Alt">0.0 ft</span></div>
        </div>
      </div>
      <div id="compassHud">
        <div id="compass">
          <div id="compassInner">
            <span class="compassCard" id="cN">N</span><span class="compassCard" id="cE">E</span><span class="compassCard" id="cS">S</span><span class="compassCard" id="cW">W</span>
            <div id="compassDeg">0°</div>
            <div id="needle"></div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" onerror="document.title='VGCS_ASSET_ERROR:leaflet_js:'+Date.now()"></script>
  <script src="https://unpkg.com/cesium@1.125/Build/Cesium/Cesium.js" onerror="document.title='VGCS_ASSET_ERROR:cesium_js:'+Date.now()"></script>
  <script>
    // Fail fast with a clear signal if core assets are blocked/unreachable.
    setTimeout(() => {
      try {
        if (typeof L === 'undefined') {
          document.title = 'VGCS_ASSET_ERROR:leaflet_missing:' + Date.now();
        }
        if (typeof Cesium === 'undefined') {
          // 3D is optional; this just lets Python disable the toggle on restricted clients.
          document.title = 'VGCS_ASSET_ERROR:cesium_missing:' + Date.now();
        }
      } catch (e) {}
    }, 2500);

    /* preferCanvas false: tile/img renderer composites more reliably with plan HTML overlays in Qt WebEngine. */
    if (typeof L === 'undefined') {
      document.title = 'VGCS_ASSET_ERROR:leaflet_missing:' + Date.now();
      throw new Error('Leaflet missing');
    }
    const map = L.map('map2d', {
      zoomControl: false,
      preferCanvas: false,
      // Qt WebEngine: disabling these animations significantly improves pan smoothness.
      fadeAnimation: false,
      zoomAnimation: false,
      markerZoomAnimation: false,
    }).setView([24.7136, 46.6753], 10);
    L.control.zoom({ position: 'bottomleft' }).addTo(map);
    const linkBanner = document.getElementById('linkBanner');
    const linkBannerLogo = document.getElementById('linkBannerLogo');
    if (linkBannerLogo && linkBannerLogo.getAttribute('src')) {
      linkBannerLogo.style.display = 'block';
    }
    // If header icon assets are missing (src empty), force deterministic text fallbacks.
    function ensureIconFallback(imgId, fallbackId) {
      try {
        const img = document.getElementById(imgId);
        const fb = document.getElementById(fallbackId);
        if (!img || !fb) return 0;
        const src = String(img.getAttribute('src') || '').trim();
        if (!src) {
          img.style.display = 'none';
          fb.style.display = 'inline';
          return 1;
        }
      } catch (e) {}
      return 0;
    }
    ensureIconFallback('hdrGpsIcon', 'hdrGpsEmoji');
    ensureIconFallback('hdrBatIcon', 'hdrBatEmoji');
    try { console.log('[diag] map boot: baseUrl=' + String(document.baseURI || '')); } catch(e) {}
    if (linkBanner) {
      linkBanner.style.cursor = 'pointer';
      linkBanner.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        const target = ev.target;
        if (target && target.id === 'linkBannerLogo') {
          document.title = 'VGCS_MENU_REQUEST:' + ev.clientX + ':' + ev.clientY + ':' + Date.now();
        } else {
          document.title = 'VGCS_CONNECT_REQUEST:' + Date.now();
        }
      });
    }
    const actionTakeoff = document.getElementById('actionTakeoff');
    if (actionTakeoff) {
      actionTakeoff.classList.add('disabled');
      actionTakeoff.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        document.title = 'VGCS_TAKEOFF_REQUEST:' + Date.now();
      });
    }
    const actionReturn = document.getElementById('actionReturn');
    if (actionReturn) {
      actionReturn.classList.add('disabled');
      actionReturn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        document.title = 'VGCS_RETURN_REQUEST:' + Date.now();
      });
    }
    const planLayer = document.getElementById('planFlightLayer');
    const planStartMissionBtn = document.getElementById('planStartMissionBtn');
    const planMissionSectionHeader = document.getElementById('planMissionSectionHeader');
    const hdrMapModeBtn = document.getElementById('hdrMapModeBtn');
    const planExit = document.getElementById('planExit');
    if (planExit) {
      planExit.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        setPlanFlightVisible(false);
        document.title = 'VGCS_PLAN_EXIT:' + Date.now();
      });
    }
    // These must be initialized before any panel binding runs (TDZ-safe).
    let vehicleMarker = null;
    let waypoints = [];
    bindPlanFlyoutActions();
    bindPlanRightTabs();
    bindPlanMissionPanel();
    bindPlanStackTabs();
    setPlanSequenceTemplate('');
    setPlanFlightChromeState(false, 0);
    if (planLayer) {
      for (const btn of planLayer.querySelectorAll('.planToolBtn')) {
        btn.addEventListener('click', function() {
          const tool = btn.getAttribute('data-tool') || '';
          setPlanRailTool(tool);
          document.title = 'VGCS_PLAN_TOOL_REQUEST:' + tool + ':' + Date.now();
        });
      }
    }
    if (planStartMissionBtn) {
      planStartMissionBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        document.title = 'VGCS_MISSION_START_REQUEST:' + Date.now();
      });
    }
    if (planMissionSectionHeader) {
      const onMissionHeaderStart = function(ev) {
        if (!window.__planMissionStartStack) return;
        ev.preventDefault();
        ev.stopPropagation();
        openMissionStartDetails();
      };
      planMissionSectionHeader.addEventListener('click', onMissionHeaderStart);
      planMissionSectionHeader.addEventListener('keydown', function(ev) {
        if (ev.key === 'Enter' || ev.key === ' ') onMissionHeaderStart(ev);
      });
    }
    const planSeqRtlBtn = document.getElementById('planSeqRtlBtn');
    if (planSeqRtlBtn) {
      planSeqRtlBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        document.title = 'VGCS_RETURN_REQUEST:' + Date.now();
      });
    }
    const planSetLaunchMapCenterBtn = document.getElementById('planSetLaunchMapCenterBtn');
    if (planSetLaunchMapCenterBtn) {
      planSetLaunchMapCenterBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        if (!map) return;
        const c = map.getCenter();
        const latEl = document.getElementById('planLaunchLatVal');
        const lonEl = document.getElementById('planLaunchLonVal');
        if (latEl) latEl.textContent = c.lat.toFixed(7);
        if (lonEl) lonEl.textContent = c.lng.toFixed(7);
        schedulePlanMissionPanelEmit();
        updateLaunchMarkerFromPanel();
      });
    }
    if (hdrMapModeBtn) {
      hdrMapModeBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        document.title = 'VGCS_TOGGLE_3D_REQUEST:' + Date.now();
      });
    }
    const cameraRail = document.getElementById('cameraRail');
    const camVideoBtn = document.getElementById('camVideoBtn');
    const camPhotoBtn = document.getElementById('camPhotoBtn');
    const camRecordBtn = document.getElementById('camRecordBtn');
    const camSettingsBtn = document.getElementById('camSettingsBtn');
    const camTimer = document.getElementById('camTimer');
    const videoPreview = document.getElementById('videoPreview');
    const videoPreviewImg = document.getElementById('videoPreviewImg');
    const videoPreviewPlaceholder = document.getElementById('videoPreviewPlaceholder');
    let camTimerId = null;
    let camRecordStartedAt = 0;
    function formatRecordTime(seconds) {
      const s = Math.max(0, Number(seconds) || 0);
      const hh = String(Math.floor(s / 3600)).padStart(2, '0');
      const mm = String(Math.floor((s % 3600) / 60)).padStart(2, '0');
      const ss = String(Math.floor(s % 60)).padStart(2, '0');
      return `${hh}:${mm}:${ss}`;
    }
    function resetCameraTimer() {
      if (camTimerId) {
        clearInterval(camTimerId);
        camTimerId = null;
      }
      camRecordStartedAt = 0;
      if (camRecordBtn) camRecordBtn.classList.remove('recording');
      if (camTimer) camTimer.textContent = '00:00:00';
    }
    let __cameraChromeLinkOk = false;
    function syncCameraChromeVisibility() {
      if (!cameraRail) return 0;
      const inPlan = isPlanFlightLayerVisible();
      const show = __cameraChromeLinkOk && !inPlan;
      cameraRail.style.display = show ? 'flex' : 'none';
      if (videoPreview) videoPreview.style.display = show ? 'block' : 'none';
      if (!show) resetCameraTimer();
      return 1;
    }
    function setCameraControlsVisible(enabled) {
      __cameraChromeLinkOk = !!enabled;
      return syncCameraChromeVisibility();
    }
    function setVideoPreviewImage(dataUrl) {
      if (!videoPreviewImg || !videoPreviewPlaceholder) return 0;
      const src = String(dataUrl || '').trim();
      if (!src) {
        videoPreviewImg.removeAttribute('src');
        videoPreviewPlaceholder.style.display = 'flex';
        return 1;
      }
      videoPreviewImg.src = src;
      videoPreviewPlaceholder.style.display = 'none';
      return 1;
    }
    if (camVideoBtn) {
      camVideoBtn.classList.add('active');
      camVideoBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        camVideoBtn.classList.add('active');
        if (camPhotoBtn) camPhotoBtn.classList.remove('active');
      });
    }
    if (camPhotoBtn) {
      camPhotoBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        camPhotoBtn.classList.add('active');
        if (camVideoBtn) camVideoBtn.classList.remove('active');
      });
    }
    if (camRecordBtn) {
      camRecordBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        const now = Date.now();
        if (camRecordBtn.classList.contains('recording')) {
          resetCameraTimer();
          return;
        }
        camRecordBtn.classList.add('recording');
        camRecordStartedAt = now;
        if (camTimer) camTimer.textContent = '00:00:00';
        if (camTimerId) clearInterval(camTimerId);
        camTimerId = setInterval(() => {
          if (!camTimer || !camRecordStartedAt) return;
          const elapsedSec = Math.floor((Date.now() - camRecordStartedAt) / 1000);
          camTimer.textContent = formatRecordTime(elapsedSec);
        }, 250);
      });
    }
    if (camSettingsBtn) {
      camSettingsBtn.addEventListener('click', function(ev) {
        ev.preventDefault();
        ev.stopPropagation();
        camSettingsBtn.classList.add('active');
        setTimeout(() => camSettingsBtn.classList.remove('active'), 200);
      });
    }
    let tileLayer = null;
    let labelLayer = null;
    let __tileErrorCount = 0;
    let __tileErrorLastSignalAt = 0;
    window.__lowSpec = false;
    window.__tileTemplate = '';
    window.__tileAttribution = '';
    window.__tileMaxZoom = 19;
    window.__tilePlaceholderDetected = false;
    window.__tilePlaceholderLastSignalAt = 0;
    const LABELS_TEMPLATE_ESRI =
      'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}';
    function setTileSource(urlTemplate, attribution, maxZoom) {
      if (tileLayer) map.removeLayer(tileLayer);
      if (labelLayer) map.removeLayer(labelLayer);
      const mz = maxZoom || 19;
      __tileErrorCount = 0;
      window.__tileTemplate = urlTemplate || '';
      try { window.__lastTileTemplate = window.__tileTemplate; } catch(e) {}
      window.__tileAttribution = attribution || '';
      window.__tileMaxZoom = mz;
      const low = !!window.__lowSpec;
      const effectiveMz = low ? Math.min(mz, 17) : mz;
      tileLayer = L.tileLayer(urlTemplate, {
        maxZoom: effectiveMz,
        attribution: attribution || '',
        // Performance: reduce tile churn while panning/zooming in Qt WebEngine.
        updateWhenIdle: true,
        updateWhenZooming: false,
        // Smaller buffer = fewer tiles kept/decoded during pans.
        keepBuffer: low ? 1 : 2,
        // Throttle tile updates a bit to keep panning responsive.
        updateInterval: low ? 260 : 140,
        // Avoid higher-res tile fetches that can double work.
        detectRetina: false
      }).addTo(map);
      try { console.log('[diag] tileSource=' + String(window.__tileTemplate || '') + ' mz=' + String(effectiveMz)); } catch(e) {}
      // Detect Esri "Map data not yet available" placeholder tiles (HTTP 200 with gray image).
      // These do not trigger tileerror, but make the map unusable on some client networks.
      try {
        const checkPlaceholder = (img) => {
          try {
            if (!img || window.__tilePlaceholderDetected) return 0;
            const tmpl = String(window.__tileTemplate || '');
            if (!tmpl.includes('arcgisonline.com')) return 0;
            const c = document.createElement('canvas');
            c.width = 32; c.height = 32;
            const ctx = c.getContext('2d', { willReadFrequently: true });
            if (!ctx) return 0;
            ctx.drawImage(img, 0, 0, 32, 32);
            const d = ctx.getImageData(0, 0, 32, 32).data;
            let sum = 0, sum2 = 0;
            for (let i = 0; i < d.length; i += 4 * 8) {
              const r = d[i], g = d[i + 1], b = d[i + 2];
              const y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
              sum += y;
              sum2 += y * y;
            }
            const n = (d.length / (4 * 8));
            const mean = sum / n;
            const varY = (sum2 / n) - (mean * mean);
            if (mean > 150 && mean < 235 && varY < 120) {
              window.__tilePlaceholderDetected = true;
              const now = Date.now();
              if ((now - window.__tilePlaceholderLastSignalAt) > 8000) {
                window.__tilePlaceholderLastSignalAt = now;
                try { document.title = 'VGCS_TILE_PLACEHOLDER:' + now; } catch (e) {}
              }
              try {
                setTileSource('https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}', 'Tiles © Esri', 19);
              } catch (e2) {}
              return 1;
            }
          } catch (e) {}
          return 0;
        };
        tileLayer.on('tileload', function(ev) {
          try { checkPlaceholder(ev && ev.tile); } catch (e) {}
        });
      } catch (ePH) {}
      try {
        tileLayer.on('tileerror', function() {
          __tileErrorCount += 1;
          const now = Date.now();
          if (__tileErrorCount >= 10 && (now - __tileErrorLastSignalAt) > 8000) {
            __tileErrorLastSignalAt = now;
            document.title = 'VGCS_TILE_ERROR:' + now;
          }
        });
      } catch (e) {}
      // Rate-limited diagnostics to help identify "blocked" vs "offline" cases.
      try {
        window.__diag = window.__diag || {err:0, ok:0, last:0};
        tileLayer.on('tileload', function() {
          window.__diag.ok += 1;
          const now = Date.now();
          if (now - window.__diag.last > 5000) {
            window.__diag.last = now;
            try { console.log('[diag] tiles ok=' + window.__diag.ok + ' err=' + window.__diag.err); } catch(e) {}
          }
        });
        tileLayer.on('tileerror', function(ev) {
          window.__diag.err += 1;
          if (window.__diag.err <= 3) {
            try { console.log('[diag] tileerror url=' + String(ev && ev.tile && ev.tile.src || '')); } catch(e) {}
          }
          if (window.__diag.err === 3) {
            try { console.log('[diag] many tile errors; likely blocked DNS/proxy/firewall. Consider Offline Tiles.'); } catch(e) {}
          }
        });
      } catch (eDiag) {}
      if (!low) {
        // Add borders + place labels overlay (transparent tiles) to match client reference.
        // Works best over satellite imagery; safe to keep enabled for other sources too.
        try {
          labelLayer = L.tileLayer(LABELS_TEMPLATE_ESRI, {
            maxZoom: effectiveMz,
            // Avoid fetching labels at very low zooms (saves a lot of requests).
            minZoom: 3,
            opacity: 0.9,
            attribution: '',
            updateWhenIdle: true,
            updateWhenZooming: false,
            keepBuffer: 1,
            updateInterval: 180,
            detectRetina: false,
            pane: 'overlayPane'
          }).addTo(map);
          try {
            labelLayer.on('tileerror', function() {
              __tileErrorCount += 1;
              const now = Date.now();
              if (__tileErrorCount >= 10 && (now - __tileErrorLastSignalAt) > 8000) {
                __tileErrorLastSignalAt = now;
                document.title = 'VGCS_TILE_ERROR:' + now;
              }
            });
          } catch (e2) {}
        } catch (e) {
          labelLayer = null;
        }
        // Performance: hide labels while interacting, restore when idle.
        // This avoids expensive compositing of semi-transparent tiles during drag.
        try {
          if (!window.__labelsPanHooked) {
            window.__labelsPanHooked = true;
            let __labelsRestoreT = null;
            const hide = () => { try { if (labelLayer) labelLayer.setOpacity(0.0); } catch (e) {} };
            const restoreSoon = () => {
              try {
                if (__labelsRestoreT) clearTimeout(__labelsRestoreT);
                __labelsRestoreT = setTimeout(() => {
                  try { if (labelLayer) labelLayer.setOpacity(0.9); } catch (e) {}
                }, 140);
              } catch (e) {}
            };
            map.on('movestart', hide);
            map.on('zoomstart', hide);
            map.on('moveend', restoreSoon);
            map.on('zoomend', restoreSoon);
          }
          if (labelLayer) labelLayer.setOpacity(0.9);
        } catch (e) {}
      } else {
        labelLayer = null;
      }
      return 1;
    }

    // Provide current map view + active tile template for Python-side probes.
    try {
      window.__vgcsGetMapView = function() {
        try {
          const c = map && map.getCenter ? map.getCenter() : { lat: 0, lng: 0 };
          const z = map && map.getZoom ? map.getZoom() : 0;
          return JSON.stringify({
            z: Number(z) || 0,
            lat: Number(c.lat) || 0,
            lng: Number(c.lng) || 0,
            template: String(window.__tileTemplate || '')
          });
        } catch (e) {
          return JSON.stringify({ z: 0, lat: 0, lng: 0, template: String(window.__tileTemplate || '') });
        }
      };
    } catch (e) {}

    function setLowSpecMode(enabled) {
      window.__lowSpec = !!enabled;
      try {
        if (window.__tileTemplate) {
          setTileSource(window.__tileTemplate, window.__tileAttribution || '', window.__tileMaxZoom || 19);
        }
      } catch (e) {}
      return window.__lowSpec ? 1 : 0;
    }
    // Choose a reliable default tile source for the current network.
    const ESRI_SAT =
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}';
    const OSM =
      'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

    function probeTile(urlTemplate, timeoutMs, cb) {
      try {
        const img = new Image();
        let done = false;
        const finish = (ok) => { if (done) return; done = true; try { cb(!!ok); } catch(e) {} };
        const t = setTimeout(() => finish(false), Math.max(400, Number(timeoutMs) || 2200));
        img.onload = () => { try { clearTimeout(t); } catch(e) {} finish(true); };
        img.onerror = () => { try { clearTimeout(t); } catch(e) {} finish(false); };
        // Use a low zoom tile that should exist everywhere.
        const url = String(urlTemplate || '').replace('{z}','0').replace('{x}','0').replace('{y}','0').replace('{s}','a');
        img.src = url;
      } catch (e) {
        try { cb(false); } catch(e2) {}
      }
    }

    // Default to satellite imagery to match prior VGCS behavior.
    // If the client network blocks imagery, user can switch to Streets or Offline Tiles.
    setTileSource(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      'Tiles © Esri',
      19
    );

    const vehicleMarkerSvg =
      '<svg xmlns="http://www.w3.org/2000/svg" width="30" height="30" viewBox="0 0 30 30" aria-hidden="true">' +
      '<path d="M15 2.5 L26.2 22.5 L17.8 19.8 L15 27.5 L12.2 19.8 L3.8 22.5 Z" ' +
      'fill="#ff2328" stroke="#4a1222" stroke-width="1.35" stroke-linejoin="round"/>' +
      '<path d="M15 5 L15 22" stroke="#3a0f18" stroke-width="1.05" stroke-linecap="round"/>' +
      '</svg>';
    vehicleMarker = L.marker([24.7136, 46.6753], {
      icon: L.divIcon({
        className: 'vgcs-vehicle-marker',
        html: '<div class="vgcs-vehicle-marker-inner">' + vehicleMarkerSvg + '</div>',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
      }),
      interactive: false,
      keyboard: false
    }).addTo(map);
    let headingLine = L.polyline([], {
      color: '#ff7700',
      weight: 5,
      opacity: 0.95,
      lineCap: 'round',
      lineJoin: 'round'
    }).addTo(map);
    let missionRoute = L.polyline([], {
      color: '#ef4444',
      weight: 4,
      opacity: 0.95,
      interactive: false,
      pane: 'overlayPane'
    });
    let flightTrack = L.polyline([], {
      color: '#f97316',
      weight: 2,
      opacity: 0.55,
      interactive: false
    });
    let __lastTrackLat = null;
    let __lastTrackLon = null;
    let addMode = false;
    let addFenceMode = false;
    let fencePoints = [];
    let fencePolygon = null;
    let viewer3d = null;
    let vehicleEntity = null;
    let headingEntity = null;
    window.__lastVehLat = null;
    window.__lastVehLon = null;
    window.__is3d = false;
    window.__heading = 0;
    window.__3dHasInitialFocus = false;
    window.__missionNavSeq = 0;
    const HEADING_MIN_INTERVAL_MS = 125; // ~8 Hz max redraw
    const VFR_HEADING_PRIORITY_MS = 1200; // ignore attitude yaw briefly after VFR_HUD
    let __headingLastApplyMs = 0;
    let __headingLastVfrMs = 0;
    let __headingRafId = null;
    let __headingPending = null;

    function setLinkConnected(connected) {
      try { console.log('[diag] setLinkConnected(' + String(!!connected) + ')'); } catch(e) {}
      if (connected) {
        setFlightStatus('yellow', 'Connected - Not Ready to Arm');
      } else {
        setFlightStatus('red', 'Communication lost - Not Ready to Arm');
      }
      return 1;
    }

    function setFlightStatus(state, detailText) {
      const banner = document.getElementById('linkBanner');
      const bannerText = document.getElementById('linkBannerText');
      const disconnected = document.getElementById('linkBannerDisconnected');
      const connectedRow = document.getElementById('linkBannerConnected');
      if (!banner || !bannerText || !disconnected || !connectedRow) return 0;

      const stateText = (state || '').toLowerCase();
      const detail = detailText || '';
      const isGreen = stateText === 'green';
      const isYellow = stateText === 'yellow';
      const isRed = stateText === 'red';
      const connected = isGreen || isYellow;
      setActionButtonsEnabled(connected);

      if (connected) {
        disconnected.style.display = 'none';
        connectedRow.style.display = 'flex';
      } else {
        connectedRow.style.display = 'none';
        disconnected.style.display = 'flex';
      }

      if (isGreen) {
        bannerText.textContent = detail || 'Parameter downloading... Ready to Arm';
        banner.style.background = 'rgba(24, 82, 38, 0.96)';
        banner.style.borderColor = 'rgba(94, 214, 119, 0.95)';
        banner.style.color = '#e8ffe8';
      } else if (isYellow) {
        bannerText.textContent = detail || 'Connected - Not Ready to Arm';
        banner.style.background = 'rgba(120, 95, 24, 0.96)';
        banner.style.borderColor = 'rgba(247, 211, 92, 0.95)';
        banner.style.color = '#fff7dd';
      } else {
        bannerText.textContent = detail || 'Communication lost - Not Ready to Arm';
        banner.style.background = 'rgba(124, 24, 24, 0.96)';
        banner.style.borderColor = 'rgba(245, 99, 99, 0.95)';
        banner.style.color = '#ffe8e8';
      }
      return 1;
    }

    function setActionButtonsEnabled(enabled) {
      const takeoff = document.getElementById('actionTakeoff');
      const ret = document.getElementById('actionReturn');
      setCameraControlsVisible(enabled);
      for (const el of [takeoff, ret]) {
        if (!el) continue;
        if (enabled) {
          el.classList.remove('disabled');
        } else {
          el.classList.add('disabled');
        }
      }
      return 1;
    }

    function getActivePlanTool() {
      const layer = document.getElementById('planFlightLayer');
      if (!layer) return 'File';
      const act = layer.querySelector('.planToolBtn.active');
      return (act && act.getAttribute('data-tool')) || 'File';
    }
    function updatePlanToolPanel(tool) {
      const t = (tool || '').trim();
      const fileFlyout = document.getElementById('planFileFlyout');
      const other = document.getElementById('planOtherToolPanel');
      const center = document.getElementById('planCenterPanel');
      const isFile = t.toLowerCase() === 'file';
      if (fileFlyout) fileFlyout.style.display = isFile ? 'block' : 'none';
      // Match reference flow: selecting Survey/other tools dismisses file flyout panel.
      if (center) center.style.display = isFile ? 'block' : 'none';
      if (other) {
        other.style.display = 'none';
        const hint = document.getElementById('planOtherToolHint');
        if (hint) {
          const hints = {
            Takeoff: 'Sends takeoff to the vehicle (same as dashboard) using Takeoff alt (m). Vehicle must be connected.',
            Waypoint: 'Click on the map to place waypoints.',
            ROI: 'Click on the map to add fence polygon vertices.',
            Pattern: 'Pattern fills a survey grid from the current vehicle position.',
            Return: 'Return / RTL uses the map Return control.',
            Center: 'Centers the map on the vehicle.'
          };
          hint.textContent = hints[t] || 'Tool active.';
        }
      }
    }
    function setPlanRailTool(name) {
      const layer = document.getElementById('planFlightLayer');
      if (!layer) return 0;
      const want = (name || '').trim();
      if (want) window.__planRailTool = want;
      if (want !== 'Waypoint') addMode = false;
      if (want !== 'ROI') {
        addFenceMode = false;
        if (fencePoints && fencePoints.length > 0 && fencePoints.length < 3) {
          fencePoints = [];
          try {
            if (fencePolygon) {
              map.removeLayer(fencePolygon);
              fencePolygon = null;
            }
          } catch (e) {}
        }
      }
      for (const el of layer.querySelectorAll('.planToolBtn')) {
        const t = el.getAttribute('data-tool') || '';
        el.classList.toggle('active', t === want);
      }
      updatePlanToolPanel(want);
      return 1;
    }

    function disablePlanEditModes() {
      addMode = false;
      addFenceMode = false;
      if (fencePoints && fencePoints.length > 0 && fencePoints.length < 3) {
        fencePoints = [];
        try {
          if (fencePolygon) {
            map.removeLayer(fencePolygon);
            fencePolygon = null;
          }
        } catch (e) {}
      }
    }
    function setPlanFlightChromeState(linked, wpCount) {
      const n = Math.max(0, Number(wpCount) || 0);
      const has = n > 0;
      const link = !!linked;
      const upBar = document.getElementById('planBarUpload');
      const vUp = document.getElementById('planVehicleUpload');
      const vDown = document.getElementById('planVehicleDownload');
      const sav = document.getElementById('planStorageSave');
      const savAs = document.getElementById('planStorageSaveAs');
      const kml = document.getElementById('planStorageKml');
      if (vDown) vDown.disabled = !link;
      if (vUp) vUp.disabled = !link || !has;
      if (upBar) upBar.disabled = !link || !has;
      if (sav) sav.disabled = !has;
      if (savAs) savAs.disabled = !has;
      if (kml) kml.disabled = !has;
      return 1;
    }
    const PLAN_TAB_KEYS = ['mission', 'fence', 'rally'];
    const PLAN_TAB_BTN_IDS = {
      mission: 'planTabBtnMission',
      fence: 'planTabBtnFence',
      rally: 'planTabBtnRally'
    };
    function activatePlanTab(key) {
      const k = PLAN_TAB_KEYS.indexOf(key) >= 0 ? key : 'mission';
      const tabs = document.querySelectorAll('#planTabs .planTab');
      const bodies = {
        mission: document.getElementById('planTabPanelMission'),
        fence: document.getElementById('planTabPanelFence'),
        rally: document.getElementById('planTabPanelRally'),
      };
      for (const tab of tabs) {
        const id = tab.getAttribute('data-plan-tab') || '';
        const on = id === k;
        tab.classList.toggle('active', on);
        tab.setAttribute('aria-selected', on ? 'true' : 'false');
        tab.setAttribute('tabindex', on ? '0' : '-1');
      }
      for (const name of PLAN_TAB_KEYS) {
        const el = bodies[name];
        if (!el) continue;
        if (name === k) {
          el.removeAttribute('hidden');
        } else {
          el.setAttribute('hidden', '');
        }
      }
      return 1;
    }
    function bindPlanRightTabs() {
      const tablist = document.getElementById('planTabs');
      if (!tablist) return;
      const tabs = tablist.querySelectorAll('.planTab');
      for (const tab of tabs) {
        tab.addEventListener('click', function(ev) {
          ev.preventDefault();
          ev.stopPropagation();
          const el = ev.currentTarget;
          const key = el.getAttribute('data-plan-tab') || 'mission';
          activatePlanTab(key);
          try {
            el.focus({ preventScroll: true });
          } catch (e) {
            el.focus();
          }
        });
        tab.addEventListener('keydown', function(ev) {
          const cur = ev.currentTarget.getAttribute('data-plan-tab') || 'mission';
          let idx = PLAN_TAB_KEYS.indexOf(cur);
          if (idx < 0) idx = 0;
          if (ev.key === 'ArrowRight' || ev.key === 'ArrowDown') {
            ev.preventDefault();
            idx = (idx + 1) % PLAN_TAB_KEYS.length;
            activatePlanTab(PLAN_TAB_KEYS[idx]);
            const nextBtn = document.getElementById(PLAN_TAB_BTN_IDS[PLAN_TAB_KEYS[idx]]);
            if (nextBtn) nextBtn.focus();
          } else if (ev.key === 'ArrowLeft' || ev.key === 'ArrowUp') {
            ev.preventDefault();
            idx = (idx + PLAN_TAB_KEYS.length - 1) % PLAN_TAB_KEYS.length;
            activatePlanTab(PLAN_TAB_KEYS[idx]);
            const nextBtn = document.getElementById(PLAN_TAB_BTN_IDS[PLAN_TAB_KEYS[idx]]);
            if (nextBtn) nextBtn.focus();
          } else if (ev.key === 'Home') {
            ev.preventDefault();
            activatePlanTab('mission');
            const b = document.getElementById('planTabBtnMission');
            if (b) b.focus();
          } else if (ev.key === 'End') {
            ev.preventDefault();
            activatePlanTab('rally');
            const b = document.getElementById('planTabBtnRally');
            if (b) b.focus();
          }
        });
      }
    }
    function bindPlanFlyoutActions() {
      const mapActionBtn = (id, action) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.addEventListener('click', function(ev) {
          ev.preventDefault();
          ev.stopPropagation();
          if (el.disabled) return;
          document.title = 'VGCS_PLAN_ACTION:' + action + ':' + Date.now();
        });
      };
      mapActionBtn('planBarUpload', 'bar_upload');
      mapActionBtn('planStorageOpen', 'open');
      mapActionBtn('planStorageSave', 'save');
      mapActionBtn('planStorageSaveAs', 'save_as');
      mapActionBtn('planStorageKml', 'save_kml');
      mapActionBtn('planVehicleUpload', 'vehicle_upload');
      mapActionBtn('planVehicleDownload', 'vehicle_download');
      mapActionBtn('planVehicleClear', 'vehicle_clear');
      mapActionBtn('planFenceRoiBtn', 'fence_roi_tool');
      const tpl = [
        ['planTplEmpty', 'template_empty'],
        ['planTplSurvey', 'template_survey'],
        ['planTplCorridor', 'template_corridor'],
        ['planTplStructure', 'template_structure']
      ];
      for (const [eid, act] of tpl) {
        const node = document.getElementById(eid);
        if (!node) continue;
        node.addEventListener('click', function(ev) {
          ev.preventDefault();
          ev.stopPropagation();
          document.title = 'VGCS_PLAN_ACTION:' + act + ':' + Date.now();
        });
      }
    }
    function clearLaunchMarker() {}

    function isPlanFlightLayerVisible() {
      const layer = document.getElementById('planFlightLayer');
      if (!layer || !window.getComputedStyle) return false;
      return window.getComputedStyle(layer).display !== 'none';
    }

    function updateLaunchMarkerFromPanel() {
      clearLaunchMarker();
    }

    function setPlanFlightVisible(visible) {
      const layer = document.getElementById('planFlightLayer');
      const hud = document.getElementById('mapFooterHud');
      if (!layer) return 0;
      layer.style.display = visible ? 'block' : 'none';
      if (hud) {
        hud.style.display = visible ? 'none' : '';
        hud.setAttribute('aria-hidden', visible ? 'true' : 'false');
      }
      syncCameraChromeVisibility();
      if (!visible) {
        clearLaunchMarker();
      } else {
        const w = window.__planRailTool && String(window.__planRailTool).trim();
        if (w) {
          setPlanRailTool(w);
        } else {
          updatePlanToolPanel(getActivePlanTool());
        }
        updateLaunchMarkerFromPanel();
      }
      return 1;
    }

    function setPlanFlightMetrics(metrics) {
      if (!metrics || typeof metrics !== 'object') return 0;
      const setTxt = (id, value) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.textContent = String(value ?? '');
      };
      setTxt('pfAltDiff', metrics.altDiffFt ?? '0.0 ft');
      setTxt('pfGradient', metrics.gradient ?? '-.-');
      setTxt('pfAzimuth', metrics.azimuth ?? '0');
      setTxt('pfHeading', metrics.heading ?? 'nan');
      setTxt('pfDistPrevWp', metrics.distPrevWpFt ?? '0.0 ft');
      setTxt('pfMissionDistance', metrics.missionDistanceFt ?? '0 ft');
      setTxt('pfMissionTime', metrics.missionTime ?? '00:00:00');
      setTxt('pfMaxTelemDist', metrics.maxTelemDistFt ?? '0 ft');
      return 1;
    }

    let planMissionEmitTimer = null;
    function ftToM(ft) { return (Number(ft) || 0) * 0.3048; }
    function mToFt(m) { return (Number(m) || 0) / 0.3048; }
    function mphToMps(mph) { return (Number(mph) || 0) * 0.44704; }
    function mpsToMph(mps) { return (Number(mps) || 0) / 0.44704; }

    function ensurePlanWpMetaLen(n) {
      const nn = Math.max(0, Number(n) || 0);
      if (!Array.isArray(window.__planWpMeta)) window.__planWpMeta = [];
      const initEl = document.getElementById('planInitialWpAltInput');
      const hoverEl = document.getElementById('planHoverSpeedInput');
      const baseAltM = ftToM(initEl ? initEl.value : 164.0);
      const baseSpdMps = mphToMps(hoverEl ? hoverEl.value : 11.18);
      while (window.__planWpMeta.length < nn) {
        window.__planWpMeta.push({ alt_m: baseAltM, speed_mps: baseSpdMps });
      }
      if (window.__planWpMeta.length > nn) {
        window.__planWpMeta = window.__planWpMeta.slice(0, nn);
      }
    }

    function renderPlanWpDetails() {
      const host = document.getElementById('planWpDetails');
      const list = document.getElementById('planWpDetailsList');
      if (!host || !list) return 0;
      const n = waypoints ? waypoints.length : 0;
      ensurePlanWpMetaLen(n);
      if (n <= 0) {
        host.style.display = 'none';
        list.innerHTML = '';
        return 1;
      }
      host.style.display = '';
      const launchAltEl = document.getElementById('planLaunchAltInput');
      const rawLaunch = launchAltEl ? String(launchAltEl.value || '') : '';
      const lx = parseFloat(rawLaunch.replace(',', '.'));
      const launchFtStr = Number.isFinite(lx) ? lx.toFixed(1) : '0.0';
      let html = '';
      html +=
        '<div class="planWpRow planWpRow--start" data-wp-start="1">' +
          '<div class="planWpLabel">Start</div>' +
          '<div class="planWpFields">' +
            '<div class="planWpField"><input class="planWpStartAlt" type="text" inputmode="decimal" value="' + launchFtStr + '"/><span class="planWpUnit">ft</span></div>' +
            '<div class="planWpStartHint">Takeoff / launch altitude (0 = use WP1 for takeoff).</div>' +
          '</div>' +
        '</div>';
      for (let i = 0; i < n; i++) {
        const m = window.__planWpMeta[i] || {};
        const altFt = mToFt(m.alt_m ?? ftToM(164));
        const spdMph = mpsToMph(m.speed_mps ?? mphToMps(11.18));
        html +=
          '<div class="planWpRow" data-wp-idx="' + i + '">' +
            '<div class="planWpLabel">WP ' + (i+1) + '</div>' +
            '<div class="planWpFields">' +
              '<div class="planWpField"><input class="planWpAlt" type="text" inputmode="decimal" value="' + altFt.toFixed(1) + '"/><span class="planWpUnit">ft</span></div>' +
              '<div class="planWpField"><input class="planWpSpd" type="text" inputmode="decimal" value="' + spdMph.toFixed(1) + '"/><span class="planWpUnit">mph</span></div>' +
            '</div>' +
          '</div>';
      }
      list.innerHTML = html;
      const startAltIn = list.querySelector('.planWpStartAlt');
      if (startAltIn && launchAltEl) {
        const onStartAlt = () => {
          if (window.__planPanelSuppressEmit) return;
          const num = (v, d) => {
            const x = parseFloat(String(v || '').replace(',', '.'));
            return Number.isFinite(x) ? x : d;
          };
          const ft = num(startAltIn.value, 0);
          launchAltEl.value = String(ft);
          schedulePlanMissionPanelEmit();
        };
        startAltIn.addEventListener('input', onStartAlt);
        startAltIn.addEventListener('change', onStartAlt);
      }
      for (const row of list.querySelectorAll('.planWpRow[data-wp-idx]')) {
        const idx = Number(row.getAttribute('data-wp-idx') || '0') || 0;
        const altIn = row.querySelector('.planWpAlt');
        const spdIn = row.querySelector('.planWpSpd');
        const onChange = () => {
          if (window.__planPanelSuppressEmit) return;
          const num = (v, d) => {
            const x = parseFloat(String(v || '').replace(',', '.'));
            return Number.isFinite(x) ? x : d;
          };
          ensurePlanWpMetaLen(waypoints.length);
          const aFt = num(altIn && altIn.value, 164.0);
          const sMph = num(spdIn && spdIn.value, 11.18);
          window.__planWpMeta[idx] = {
            alt_m: Math.max(1.0, ftToM(aFt)),
            speed_mps: Math.max(0.1, mphToMps(sMph)),
          };
          schedulePlanMissionPanelEmit();
        };
        if (altIn) { altIn.addEventListener('input', onChange); altIn.addEventListener('change', onChange); }
        if (spdIn) { spdIn.addEventListener('input', onChange); spdIn.addEventListener('change', onChange); }
      }
      return 1;
    }
    function schedulePlanMissionPanelEmit() {
      if (window.__planPanelSuppressEmit) return;
      if (planMissionEmitTimer) clearTimeout(planMissionEmitTimer);
      planMissionEmitTimer = setTimeout(emitPlanMissionPanel, 320);
    }
    function emitPlanMissionPanel() {
      if (window.__planPanelSuppressEmit) return;
      planMissionEmitTimer = null;
      const sel = document.getElementById('planAltReferenceSelect');
      const initEl = document.getElementById('planInitialWpAltInput');
      const hoverEl = document.getElementById('planHoverSpeedInput');
      const launchAltEl = document.getElementById('planLaunchAltInput');
      const latEl = document.getElementById('planLaunchLatVal');
      const lonEl = document.getElementById('planLaunchLonVal');
      const num = (v, d) => {
        const x = parseFloat(String(v || '').replace(',', '.'));
        return Number.isFinite(x) ? x : d;
      };
      const data = {
        altRef: sel ? String(sel.value || 'rel') : 'rel',
        initialWpAltFt: num(initEl && initEl.value, 164),
        hoverMph: num(hoverEl && hoverEl.value, 11.18),
        launchAltFt: num(launchAltEl && launchAltEl.value, 0),
        launchLat: latEl ? String(latEl.textContent || '').trim() : '',
        launchLon: lonEl ? String(lonEl.textContent || '').trim() : '',
      };
      if (Array.isArray(window.__planWpMeta)) {
        data.wpMeta = window.__planWpMeta.map((m) => ({
          alt_m: Number(m && m.alt_m) || 0,
          speed_mps: Number(m && m.speed_mps) || 0,
        }));
      }
      try {
        const js = JSON.stringify(data);
        document.title = 'VGCS_PLAN_MISSION_PANEL:' + btoa(unescape(encodeURIComponent(js)));
      } catch (e) {}
    }
    function setPlanSequenceTemplate(templateId) {
      const row = document.getElementById('planSeqPatternRow');
      const label = document.getElementById('planSeqPatternLabel');
      const missionPanel = document.getElementById('planTabPanelMission');
      const missionHeader = document.getElementById('planMissionSectionHeader');
      const compactList = document.getElementById('planSeqCompactList');
      const compactSurveyLabel = document.getElementById('planSeqCompactSurveyLabel');
      const compactSurveyBody = document.getElementById('planSeqCompactSurveyBody');
      const compactTakeoffTab = document.getElementById('planSeqCompactTakeoffTab');
      const compactSurveyTab = document.getElementById('planSeqCompactSurveyTab');
      const compactRtlTab = document.getElementById('planSeqCompactRtlTab');
      const compactRtlBody = document.getElementById('planSeqCompactRtlBody');
      const compactTakeoff = document.getElementById('planSeqCompactTakeoffCard');
      const takeoffHead = document.querySelector('#planTabPanelMission .planSeqCardHead');
      const takeoffDesc = document.querySelector('#planTabPanelMission .planSeqCardDesc');
      const startMissionBtn = document.getElementById('planStartMissionBtn');
      const seqRtlBtn = document.getElementById('planSeqRtlBtn');
      if (!row || !label) return 0;
      const t = String(templateId || '').toLowerCase().trim();
      const labels = {
        survey: 'Survey',
        corridor: 'Corridor Scan',
        structure: 'Structure Scan'
      };
      row.classList.remove('planSeqPatternRow--visible', 'planSeqPatternRow--focus');
      const isEmpty = !t || !labels[t];
      const isSurveyMode = t === 'survey';
      window.__planMissionStartStack = !!isSurveyMode;
      if (missionPanel) {
        missionPanel.classList.toggle('planMissionEmpty', isEmpty || isSurveyMode);
        missionPanel.classList.toggle('planMissionStack', isSurveyMode);
      }
      if (compactList) compactList.setAttribute('aria-hidden', isSurveyMode ? 'false' : 'true');
      if (compactSurveyLabel && labels[t]) compactSurveyLabel.textContent = labels[t];
      if (compactSurveyBody && labels[t]) compactSurveyBody.textContent = labels[t] + ' pattern selected.';
      if (missionHeader) missionHeader.textContent = (isEmpty || isSurveyMode) ? 'Mission Start' : 'Mission';
      // Hard show/hide to keep Survey state stable regardless CSS cache/state.
      if (takeoffHead) takeoffHead.style.display = (isEmpty || isSurveyMode) ? 'none' : '';
      if (takeoffDesc) takeoffDesc.style.display = (isEmpty || isSurveyMode) ? 'none' : '';
      if (seqRtlBtn) seqRtlBtn.style.display = isSurveyMode ? 'none' : (isEmpty ? 'none' : '');
      // Keep Start Mission visible regardless of template state.
      if (startMissionBtn) startMissionBtn.style.display = '';
      if (compactList) compactList.style.display = isSurveyMode ? 'flex' : 'none';
      if (compactTakeoffTab) compactTakeoffTab.classList.remove('is-active');
      if (compactTakeoff) compactTakeoff.classList.remove('is-active');
      if (compactSurveyTab) compactSurveyTab.classList.remove('is-active');
      if (compactRtlTab) compactRtlTab.classList.remove('is-active');
      if (compactSurveyBody) compactSurveyBody.classList.remove('is-active');
      if (compactRtlBody) compactRtlBody.classList.remove('is-active');
      if (!t || !labels[t]) {
        return 1;
      }
      label.textContent = labels[t];
      row.classList.add('planSeqPatternRow--visible', 'planSeqPatternRow--focus');
      setPlanStackTab('takeoff');
      return 1;
    }
    function setPlanStackTab(name) {
      const missionPanel = document.getElementById('planTabPanelMission');
      if (!missionPanel) return 0;
      missionPanel.classList.remove('planMissionDetailsOpen');
      const tab = String(name || 'takeoff').toLowerCase();
      const compactTakeoffTab = document.getElementById('planSeqCompactTakeoffTab');
      const compactSurveyTab = document.getElementById('planSeqCompactSurveyTab');
      const compactRtlTab = document.getElementById('planSeqCompactRtlTab');
      const compactTakeoff = document.getElementById('planSeqCompactTakeoffCard');
      const compactSurveyBody = document.getElementById('planSeqCompactSurveyBody');
      const compactRtlBody = document.getElementById('planSeqCompactRtlBody');
      const takeoffBody = document.querySelector('#planTabPanelMission .planSeqCardBody');
      const vehicleDetails = document.getElementById('planVehicleDetails');
      const launchDetails = document.getElementById('planLaunchDetails');
      if (compactTakeoffTab) compactTakeoffTab.classList.toggle('is-active', tab === 'takeoff');
      if (compactTakeoff) compactTakeoff.classList.toggle('is-active', tab === 'takeoff');
      if (compactSurveyTab) compactSurveyTab.classList.toggle('is-active', tab === 'survey');
      if (compactRtlTab) compactRtlTab.classList.toggle('is-active', tab === 'rtl');
      if (compactSurveyBody) compactSurveyBody.classList.toggle('is-active', tab === 'survey');
      if (compactRtlBody) compactRtlBody.classList.toggle('is-active', tab === 'rtl');
      // Keep visibility deterministic even if cached CSS state is stale.
      if (compactTakeoff) compactTakeoff.style.display = (tab === 'takeoff') ? 'block' : 'none';
      if (compactSurveyBody) compactSurveyBody.style.display = (tab === 'survey') ? 'block' : 'none';
      if (compactRtlBody) compactRtlBody.style.display = (tab === 'rtl') ? 'block' : 'none';
      if (takeoffBody) takeoffBody.style.display = 'none';
      if (vehicleDetails) vehicleDetails.style.display = 'none';
      if (launchDetails) launchDetails.style.display = 'none';
      return 1;
    }
    function bindPlanStackTabs() {
      const takeoff = document.getElementById('planSeqCompactTakeoffTab');
      const survey = document.getElementById('planSeqCompactSurveyTab');
      const rtl = document.getElementById('planSeqCompactRtlTab');
      const bind = (el, tab) => {
        if (!el) return;
        const onAct = (ev) => {
          if (ev) { ev.preventDefault(); ev.stopPropagation(); }
          setPlanStackTab(tab);
        };
        el.addEventListener('click', onAct);
        el.addEventListener('keydown', (ev) => {
          if (ev.key === 'Enter' || ev.key === ' ') onAct(ev);
        });
      };
      bind(takeoff, 'takeoff');
      bind(survey, 'survey');
      bind(rtl, 'rtl');
    }
    function openMissionStartDetails() {
      const missionPanel = document.getElementById('planTabPanelMission');
      const missionHeader = document.getElementById('planMissionSectionHeader');
      const compactList = document.getElementById('planSeqCompactList');
      const takeoffHead = document.querySelector('#planTabPanelMission .planSeqCardHead');
      const takeoffBody = document.querySelector('#planTabPanelMission .planSeqCardBody');
      const vehicleDetails = document.getElementById('planVehicleDetails');
      const launchDetails = document.getElementById('planLaunchDetails');
      const row = document.getElementById('planSeqPatternRow');
      const seqRtlBtn = document.getElementById('planSeqRtlBtn');
      const startMissionBtn = document.getElementById('planStartMissionBtn');
      const compactTakeoff = document.getElementById('planSeqCompactTakeoffCard');
      const compactSurveyBody = document.getElementById('planSeqCompactSurveyBody');
      const compactRtlBody = document.getElementById('planSeqCompactRtlBody');
      if (!missionPanel) return 0;
      // Keep stack layout so Takeoff/Survey/RTL tabs preserve their existing UI.
      missionPanel.classList.add('planMissionStack');
      missionPanel.classList.remove('planMissionEmpty');
      missionPanel.classList.add('planMissionDetailsOpen');
      if (missionHeader) missionHeader.textContent = 'Mission Start';
      if (compactList) {
        // Keep plan step tabs visible while Mission Start details are open.
        compactList.style.display = 'flex';
        compactList.setAttribute('aria-hidden', 'false');
      }
      if (takeoffHead) takeoffHead.style.display = 'none';
      if (takeoffBody) takeoffBody.style.display = '';
      if (vehicleDetails) vehicleDetails.style.display = '';
      if (launchDetails) launchDetails.style.display = '';
      if (row) row.classList.remove('planSeqPatternRow--visible', 'planSeqPatternRow--focus');
      if (seqRtlBtn) seqRtlBtn.style.display = 'none';
      if (startMissionBtn) startMissionBtn.style.display = '';
      // Show tabs only (no expanded compact detail card) while editing mission details.
      if (compactTakeoff) compactTakeoff.style.display = 'none';
      if (compactSurveyBody) compactSurveyBody.style.display = 'none';
      if (compactRtlBody) compactRtlBody.style.display = 'none';
      window.__planMissionStartStack = true;
      return 1;
    }
    function setPlanMissionStartStack(enabled, surveyLabel) {
      const missionPanel = document.getElementById('planTabPanelMission');
      const missionHeader = document.getElementById('planMissionSectionHeader');
      const compactList = document.getElementById('planSeqCompactList');
      const compactSurveyLabel = document.getElementById('planSeqCompactSurveyLabel');
      const compactSurveyBody = document.getElementById('planSeqCompactSurveyBody');
      const compactRtlBody = document.getElementById('planSeqCompactRtlBody');
      const takeoffHead = document.querySelector('#planTabPanelMission .planSeqCardHead');
      const takeoffDesc = document.querySelector('#planTabPanelMission .planSeqCardDesc');
      const takeoffBody = document.querySelector('#planTabPanelMission .planSeqCardBody');
      const vehicleDetails = document.getElementById('planVehicleDetails');
      const launchDetails = document.getElementById('planLaunchDetails');
      const startMissionBtn = document.getElementById('planStartMissionBtn');
      const seqRtlBtn = document.getElementById('planSeqRtlBtn');
      const row = document.getElementById('planSeqPatternRow');
      const on = !!enabled;
      window.__planMissionStartStack = on;
      if (missionPanel) {
        missionPanel.classList.toggle('planMissionEmpty', on);
        missionPanel.classList.toggle('planMissionStack', on);
        missionPanel.classList.toggle('planMissionDetailsOpen', on);
      }
      if (missionHeader) missionHeader.textContent = on ? 'Mission Start' : 'Mission';
      if (takeoffHead) takeoffHead.style.display = on ? 'none' : '';
      if (takeoffDesc) takeoffDesc.style.display = on ? 'none' : '';
      if (seqRtlBtn) seqRtlBtn.style.display = on ? 'none' : '';
      // Keep Start Mission visible in Mission tab for both normal and stack layouts.
      if (startMissionBtn) startMissionBtn.style.display = '';
      if (compactList) {
        compactList.setAttribute('aria-hidden', on ? 'false' : 'true');
        compactList.style.display = on ? 'flex' : 'none';
      }
      if (compactSurveyLabel) compactSurveyLabel.textContent = String(surveyLabel || 'Survey');
      if (compactSurveyBody) compactSurveyBody.textContent = String(surveyLabel || 'Survey') + ' pattern selected.';
      if (row && on) {
        row.classList.remove('planSeqPatternRow--visible', 'planSeqPatternRow--focus');
      }
      if (on) {
        // Mission Start keeps its original details UI; only append plan rows below.
        const compactTakeoffTab = document.getElementById('planSeqCompactTakeoffTab');
        const compactSurveyTab = document.getElementById('planSeqCompactSurveyTab');
        const compactRtlTab = document.getElementById('planSeqCompactRtlTab');
        const compactTakeoff = document.getElementById('planSeqCompactTakeoffCard');
        if (takeoffBody) takeoffBody.style.display = '';
        if (vehicleDetails) vehicleDetails.style.display = '';
        if (launchDetails) launchDetails.style.display = '';
        if (compactTakeoffTab) compactTakeoffTab.classList.remove('is-active');
        if (compactSurveyTab) compactSurveyTab.classList.remove('is-active');
        if (compactRtlTab) compactRtlTab.classList.remove('is-active');
        if (compactTakeoff) {
          compactTakeoff.classList.remove('is-active');
          compactTakeoff.style.display = 'none';
        }
        if (compactSurveyBody) {
          compactSurveyBody.classList.remove('is-active');
          compactSurveyBody.style.display = 'none';
        }
        if (compactRtlBody) {
          compactRtlBody.classList.remove('is-active');
          compactRtlBody.style.display = 'none';
        }
      } else {
        missionPanel.classList.remove('planMissionDetailsOpen');
        if (takeoffBody) takeoffBody.style.display = '';
        if (vehicleDetails) vehicleDetails.style.display = '';
        if (launchDetails) launchDetails.style.display = '';
        if (compactSurveyBody) compactSurveyBody.classList.remove('is-active');
        if (compactRtlBody) compactRtlBody.classList.remove('is-active');
        const compactTakeoffTab = document.getElementById('planSeqCompactTakeoffTab');
        const compactSurveyTab = document.getElementById('planSeqCompactSurveyTab');
        const compactRtlTab = document.getElementById('planSeqCompactRtlTab');
        const compactTakeoff = document.getElementById('planSeqCompactTakeoffCard');
        if (compactTakeoffTab) compactTakeoffTab.classList.remove('is-active');
        if (compactSurveyTab) compactSurveyTab.classList.remove('is-active');
        if (compactRtlTab) compactRtlTab.classList.remove('is-active');
        if (compactTakeoff) compactTakeoff.classList.remove('is-active');
      }
      return 1;
    }
    function applyPlanMissionPanelState(s) {
      if (!s || typeof s !== 'object') return 0;
      window.__planPanelSuppressEmit = true;
      try {
        const sel = document.getElementById('planAltReferenceSelect');
        if (sel && s.altRef) sel.value = String(s.altRef);
        const initEl = document.getElementById('planInitialWpAltInput');
        if (initEl && s.initialWpAltFt != null) initEl.value = String(s.initialWpAltFt);
        const hoverEl = document.getElementById('planHoverSpeedInput');
        if (hoverEl && s.hoverMph != null) hoverEl.value = String(s.hoverMph);
        const launchAltEl = document.getElementById('planLaunchAltInput');
        if (launchAltEl && s.launchAltFt != null) launchAltEl.value = String(s.launchAltFt);
        const latEl = document.getElementById('planLaunchLatVal');
        const lonEl = document.getElementById('planLaunchLonVal');
        const lt = s.launchLat != null ? String(s.launchLat).trim() : '';
        const ln = s.launchLon != null ? String(s.launchLon).trim() : '';
        if (latEl) latEl.textContent = lt || '—';
        if (lonEl) lonEl.textContent = ln || '—';
        updateLaunchMarkerFromPanel();
        if (Array.isArray(s.wpMeta)) {
          window.__planWpMeta = s.wpMeta.map((m) => ({
            alt_m: Number(m && m.alt_m) || 0,
            speed_mps: Number(m && m.speed_mps) || 0,
          }));
        }
        renderPlanWpDetails();
      } finally {
        setTimeout(function() { window.__planPanelSuppressEmit = false; }, 0);
      }
      return 1;
    }
    function setPlanVehicleInfo(fw, veh) {
      const a = document.getElementById('planVehicleFirmwareVal');
      const b = document.getElementById('planVehicleTypeVal');
      if (a) a.textContent = fw || '—';
      if (b) b.textContent = veh || '—';
      return 1;
    }
    function bindPlanMissionPanel() {
      const sel = document.getElementById('planAltReferenceSelect');
      if (sel) sel.addEventListener('change', schedulePlanMissionPanelEmit);
      for (const id of ['planInitialWpAltInput', 'planHoverSpeedInput', 'planLaunchAltInput']) {
        const el = document.getElementById(id);
        if (!el) continue;
        el.addEventListener('input', schedulePlanMissionPanelEmit);
        el.addEventListener('change', schedulePlanMissionPanelEmit);
      }
      for (const id of ['planInitialWpAltInput', 'planHoverSpeedInput', 'planLaunchAltInput']) {
        const el = document.getElementById(id);
        if (!el) continue;
        el.addEventListener('input', function() { renderPlanWpDetails(); });
        el.addEventListener('change', function() { renderPlanWpDetails(); });
      }
      renderPlanWpDetails();
    }

    function centerOnVehicle() {
      if (!vehicleMarker) return 0;
      const p = vehicleMarker.getLatLng();
      if (!p) return 0;
      // 3D mode: center/focus Cesium camera on the vehicle entity.
      if (window.__is3d) {
        try {
          if (viewer3d && vehicleEntity) {
            try { vehicleEntity.show = true; } catch (e) {}
            try {
              viewer3d.flyTo(vehicleEntity, {
                duration: 0.65,
                offset: new Cesium.HeadingPitchRange(
                  Cesium.Math.toRadians(Number(window.__heading || 0)),
                  Cesium.Math.toRadians(-40.0),
                  1400.0
                )
              });
            } catch (e) {
              try { focus3DCamera(true); } catch (e2) {}
            }
            try { viewer3d.scene && viewer3d.scene.requestRender && viewer3d.scene.requestRender(); } catch (e) {}
            return 1;
          }
          // Fallback: if entity isn't ready, at least fly to lat/lon.
          if (viewer3d && window.Cesium) {
            viewer3d.camera.flyTo({
              destination: Cesium.Cartesian3.fromDegrees(Number(p.lng), Number(p.lat), 1800),
              orientation: {
                heading: Cesium.Math.toRadians(Number(window.__heading || 0)),
                pitch: Cesium.Math.toRadians(-35.0),
                roll: 0.0
              },
              duration: 0.7
            });
            try { viewer3d.scene && viewer3d.scene.requestRender && viewer3d.scene.requestRender(); } catch (e) {}
            return 1;
          }
        } catch (e) {}
        return 0;
      }
      // 2D mode: center Leaflet map on the vehicle marker.
      if (!map) return 0;
      map.setView([p.lat, p.lng], Math.max(map.getZoom(), 16), { animate: true });
      return 1;
    }

    function setHeaderMode(text) {
      const el = document.getElementById('hdrModeText');
      if (!el) return 0;
      el.textContent = text || 'Hold';
      return 1;
    }

    function setHeaderVehicleMsg(text) {
      const el = document.getElementById('hdrVehicleMsg');
      if (!el) return 0;
      const raw = String(text || '').trim();
      // Do not hard-truncate here; let CSS ellipsis handle tight layouts.
      // Keep full text available via tooltip.
      el.textContent = raw || 'Vehicle Msg';
      try { el.title = raw || ''; } catch (e0) {}
      // Client UX: if PreArm reports bad GPS fix, keep GPS visible but visually mute it.
      try {
        const low = raw.toLowerCase();
        const badFix =
          (low.includes('prearm') && low.includes('gps') && low.includes('bad fix')) ||
          low.includes('bad fix');
        const gpsPill = document.getElementById('hdrGpsPill');
        if (gpsPill && gpsPill.classList) {
          gpsPill.classList.toggle('hdrPillMuted', !!badFix);
          try { gpsPill.title = badFix ? 'GPS: Bad fix' : ''; } catch (e2) {}
        }
      } catch (e) {}
      return 1;
    }

    function setHeaderGps(sat, hdop) {
      const satEl = document.getElementById('hdrGpsSat');
      const hdopEl = document.getElementById('hdrGpsHdop');
      if (!satEl || !hdopEl) return 0;
      satEl.textContent = String(sat || '0');
      hdopEl.textContent = String(hdop || 'N/A');
      return 1;
    }

    function setHeaderBattery(text) {
      const el = document.getElementById('hdrBatteryText');
      if (!el) return 0;
      el.textContent = text || 'N/A';
      return 1;
    }

    function setHeaderRemoteId(text) {
      const el = document.getElementById('hdrRemoteIdText');
      if (!el) return 0;
      el.textContent = text || 'ID';
      return 1;
    }

    function updateCompassNeedle(deg) {
      const needle = document.getElementById('needle');
      const degLabel = document.getElementById('compassDeg');
      if (!needle || !degLabel) return;
      const normalized = ((Number(deg) || 0) % 360 + 360) % 360;
      needle.style.transform = `rotate(${normalized}deg)`;
      degLabel.textContent = `${Math.round(normalized)}°`;
    }

    function setTelemetryOverlay(relAltM, groundSpeedMps, timeText, mslAltM) {
      const ft = (Number(relAltM || 0.0) * 3.28084).toFixed(1);
      const mph = (Number(groundSpeedMps || 0.0) * 2.23694).toFixed(1);
      const mslFt = (Number(mslAltM || 0.0) * 3.28084).toFixed(1);
      const ttime = timeText || '00:00:00';
      const prev = window.__telHudSig || '';
      const sig = ft + '|' + mph + '|' + ttime + '|' + mslFt;
      if (sig === prev) return 1;
      window.__telHudSig = sig;
      const setAll = (className, text) => {
        document.querySelectorAll('.' + className).forEach((el) => {
          el.textContent = text;
        });
      };
      setAll('telRow1Alt', `${ft} ft`);
      setAll('telRow1Mph', `${mph} mph`);
      setAll('telRow1Time', ttime);
      setAll('telRow2Alt', `${ft} ft`);
      setAll('telRow2Mph', `${mph} mph`);
      setAll('telRow2Msl', `${mslFt} ft`);
      return 1;
    }

    function preferTelemetryStrip() {
      // Keep the original bottom telemetry strip UI (vs. orbit "bubbles").
      const stack = document.getElementById('telemetryLeftStack');
      if (stack) stack.style.display = '';
      document.querySelectorAll('.telOrbitItem').forEach((el) => el.remove());
      return 1;
    }

    // Run once at startup (safe if orbit items don't exist).
    preferTelemetryStrip();

    function ensure3D() {
      if (viewer3d) return true;
      if (!window.Cesium) return false;
      try {
        const cesiumCreditHost = document.createElement('div');
        cesiumCreditHost.style.display = 'none';
        cesiumCreditHost.setAttribute('aria-hidden', 'true');
        document.body.appendChild(cesiumCreditHost);
        viewer3d = new Cesium.Viewer('map3d', {
          creditContainer: cesiumCreditHost,
          timeline: false,
          animation: false,
          geocoder: false,
          baseLayerPicker: false,
          homeButton: false,
          sceneModePicker: true,
          navigationHelpButton: false,
          fullscreenButton: false,
          infoBox: false,
          selectionIndicator: false,
          terrainProvider: new Cesium.EllipsoidTerrainProvider(),
          imageryProvider: new Cesium.OpenStreetMapImageryProvider({
            url: 'https://tile.openstreetmap.org/'
          })
        });
        // Make sure the vehicle marker stays visible on top of imagery/terrain.
        try { viewer3d.scene.globe.depthTestAgainstTerrain = false; } catch (e) {}
        // Prevent excessive zoom-in (can cause ground/imagery artifacts in Qt WebEngine builds).
        try {
          const ssc = viewer3d.scene && viewer3d.scene.screenSpaceCameraController
            ? viewer3d.scene.screenSpaceCameraController
            : null;
          if (ssc) {
            ssc.minimumZoomDistance = 250.0; // meters
            ssc.enableCollisionDetection = true;
          }
        } catch (e) {}
        // Hard clamp camera height too (minimumZoomDistance isn't always sufficient at shallow pitch).
        try {
          const MIN_CAM_H_M = 250.0;
          let __clamping = false;
          viewer3d.camera.changed.addEventListener(function() {
            if (__clamping) return;
            try {
              const c = viewer3d.camera.positionCartographic;
              if (!c || !Number.isFinite(c.height)) return;
              if (c.height >= MIN_CAM_H_M) return;
              __clamping = true;
              viewer3d.camera.setView({
                destination: Cesium.Cartesian3.fromRadians(c.longitude, c.latitude, MIN_CAM_H_M),
                orientation: {
                  heading: viewer3d.camera.heading,
                  pitch: viewer3d.camera.pitch,
                  roll: viewer3d.camera.roll
                }
              });
            } catch (e) {
              // ignore
            } finally {
              __clamping = false;
            }
          });
        } catch (e) {}
        // Force a known imagery layer for stable no-key rendering in WebEngine.
        try {
          viewer3d.imageryLayers.removeAll();
          viewer3d.imageryLayers.addImageryProvider(
            new Cesium.UrlTemplateImageryProvider({
              url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
              credit: 'Tiles © Esri'
            })
          );
          // Overlay: borders + place labels (transparent) to match client reference.
          try {
            viewer3d.imageryLayers.addImageryProvider(
              new Cesium.UrlTemplateImageryProvider({
                url: 'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
                credit: ''
              })
            );
          } catch (e2) {}
        } catch (e) {
          // Fallback to OSM if ArcGIS provider fails.
          try {
            viewer3d.imageryLayers.removeAll();
            viewer3d.imageryLayers.addImageryProvider(
              new Cesium.OpenStreetMapImageryProvider({
                url: 'https://tile.openstreetmap.org/'
              })
            );
          } catch (e2) {}
        }
        // Vehicle marker for 3D: use the same arrow/chevron SVG as 2D, as a Cesium billboard.
        const seed = vehicleMarker ? vehicleMarker.getLatLng() : null;
        const seedLat = (window.__lastVehLat != null ? window.__lastVehLat : (seed ? seed.lat : 24.7136));
        const seedLon = (window.__lastVehLon != null ? window.__lastVehLon : (seed ? seed.lng : 46.6753));
        // Convert inline SVG to a data URL for Cesium billboard rendering.
        const __vehSvg = (typeof vehicleMarkerSvg === 'string' && vehicleMarkerSvg.length)
          ? vehicleMarkerSvg
          : '<svg xmlns="http://www.w3.org/2000/svg" width="30" height="30" viewBox="0 0 30 30" aria-hidden="true">' +
            '<path d="M15 2.5 L26.2 22.5 L17.8 19.8 L15 27.5 L12.2 19.8 L3.8 22.5 Z" ' +
            'fill="#ff2328" stroke="#4a1222" stroke-width="1.35" stroke-linejoin="round"/>' +
            '<path d="M15 5 L15 22" stroke="#3a0f18" stroke-width="1.05" stroke-linecap="round"/>' +
            '</svg>';
        const __vehSvgUrl = 'data:image/svg+xml;charset=UTF-8,' + encodeURIComponent(__vehSvg);
        vehicleEntity = viewer3d.entities.add({
          // Use a small fixed height above ellipsoid to ensure visibility even when clamping fails.
          position: Cesium.Cartesian3.fromDegrees(Number(seedLon) || 46.6753, Number(seedLat) || 24.7136, 30),
          billboard: {
            image: __vehSvgUrl,
            width: 30,
            height: 30,
            verticalOrigin: Cesium.VerticalOrigin.CENTER,
            horizontalOrigin: Cesium.HorizontalOrigin.CENTER,
            rotation: Cesium.Math.toRadians(Number(window.__heading || 0)),
            alignedAxis: Cesium.Cartesian3.UNIT_Z,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            scaleByDistance: new Cesium.NearFarScalar(250.0, 1.15, 4500.0, 0.55)
          }
        });
        headingEntity = viewer3d.entities.add({
          polyline: {
            positions: [],
            width: 0,
            material: Cesium.Color.fromCssColorString('#00000000')
          }
        });
        window.__3dHasInitialFocus = false;
        return true;
      } catch (e) {
        return false;
      }
    }

    function focus3DCamera(force) {
      if (!viewer3d || !window.Cesium) return 0;
      if (!force && window.__3dHasInitialFocus) return 0;
      const p = vehicleMarker ? vehicleMarker.getLatLng() : null;
      const lat = p ? Number(p.lat) : 24.7136;
      const lon = p ? Number(p.lng) : 46.6753;
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) return 0;
      try {
        viewer3d.camera.flyTo({
          destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1800),
          orientation: {
            heading: Cesium.Math.toRadians(Number(window.__heading || 0)),
            pitch: Cesium.Math.toRadians(-35.0),
            roll: 0.0
          },
          duration: 0.9
        });
        window.__3dHasInitialFocus = true;
        return 1;
      } catch (e) {
        return 0;
      }
    }

    function syncVehicleHeadingLine() {
      clearVehicleHeadingLine();
    }

    function clearVehicleHeadingLine() {
      headingLine.setLatLngs([]);
      if (headingEntity) {
        headingEntity.polyline.positions = [];
      }
    }

    function updateHeadingLineGeometry(lat, lon, deg) {
      clearVehicleHeadingLine();
    }

    function haversine_m(lat1, lon1, lat2, lon2) {
      const R = 6371000;
      const toR = (x) => x * Math.PI / 180;
      const dLat = toR(lat2 - lat1);
      const dLon = toR(lon2 - lon1);
      const a = Math.sin(dLat / 2) ** 2 +
        Math.cos(toR(lat1)) * Math.cos(toR(lat2)) * Math.sin(dLon / 2) ** 2;
      return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
    }

    function appendFlightTrack(lat, lon) {
      try {
        if (__lastTrackLat != null && __lastTrackLon != null) {
          const d = haversine_m(__lastTrackLat, __lastTrackLon, lat, lon);
          if (d < 0.3) return;
        }
        __lastTrackLat = lat;
        __lastTrackLon = lon;
        let pts = (flightTrack.getLatLngs() || []).concat([[lat, lon]]);
        const trimM = 10;
        while (pts.length > 1) {
          const a = pts[0];
          if (haversine_m(a.lat, a.lng, lat, lon) < trimM) pts = pts.slice(1);
          else break;
        }
        const maxPts = 200;
        while (pts.length > maxPts) pts = pts.slice(pts.length - maxPts);
        flightTrack.setLatLngs(pts);
        if (pts.length && !map.hasLayer(flightTrack)) flightTrack.addTo(map);
      } catch (e) {}
    }

    function clearFlightTrack() {
      __lastTrackLat = null;
      __lastTrackLon = null;
      try { flightTrack.setLatLngs([]); } catch (e) {}
      try { if (map.hasLayer(flightTrack)) map.removeLayer(flightTrack); } catch (e) {}
    }

    function updateMissionRoutePolyline() {
      try {
        const navSeq = Number(window.__missionNavSeq) || 0;
        let startIdx = 0;
        // VGCS mission layout sent to ArduPilot:
        //   seq 0: dummy (home slot / protocol placeholder)
        //   seq 1: TAKEOFF
        //   seq 2: DO_CHANGE_SPEED for WP1
        //   seq 3: WP1
        //   seq 4: DO_CHANGE_SPEED for WP2
        //   seq 5: WP2
        // Therefore map MISSION_CURRENT.seq -> waypoint array index:
        //   - before seq 3: start at WP1 (index 0)
        //   - seq>=3: floor((seq-3)/2)
        if (navSeq >= 3) {
          startIdx = Math.max(0, Math.min(waypoints.length, Math.floor((navSeq - 3) / 2)));
        } else {
          startIdx = 0;
        }
        const veh = vehicleMarker && vehicleMarker.getLatLng();
        const rest = waypoints.slice(startIdx).map((w) => w.getLatLng());
        let ll = [];
        if (veh && rest.length >= 1) {
          ll = [[veh.lat, veh.lng]].concat(rest);
        } else if (rest.length >= 2) {
          ll = rest;
        } else if (veh && rest.length === 1) {
          ll = [[veh.lat, veh.lng], rest[0]];
        } else if (rest.length === 1) {
          ll = rest;
        }
        if (ll.length < 2) {
          if (map.hasLayer(missionRoute)) map.removeLayer(missionRoute);
          return;
        }
        missionRoute.setLatLngs(ll);
        if (!map.hasLayer(missionRoute)) missionRoute.addTo(map);
        try { missionRoute.bringToFront(); } catch (e) {}
      } catch (e) {}
    }

    function setVehicle(lat, lon) {
      vehicleMarker.setLatLng([lat, lon]);
      window.__lastVehLat = lat;
      window.__lastVehLon = lon;
      appendFlightTrack(lat, lon);
      updateHeadingLineGeometry(lat, lon, window.__heading || 0);
      updateMissionRoutePolyline();
      if (vehicleEntity) {
        // Cesium stores `entity.position` as a Property; in some Qt WebEngine builds,
        // reassigning `entity.position = Cartesian3` doesn't reliably update the visual.
        // Prefer `setValue()` when available.
        const p3 = Cesium.Cartesian3.fromDegrees(lon, lat, 30);
        try {
          if (vehicleEntity.position && typeof vehicleEntity.position.setValue === 'function') {
            vehicleEntity.position.setValue(p3);
          } else {
            vehicleEntity.position = p3;
          }
        } catch (e) {
          try { vehicleEntity.position = p3; } catch (e2) {}
        }
        try { vehicleEntity.show = true; } catch (e) {}
        try { viewer3d && viewer3d.scene && viewer3d.scene.requestRender && viewer3d.scene.requestRender(); } catch (e) {}
      }
      if (window.__is3d && !window.__3dHasInitialFocus) {
        focus3DCamera(false);
      }
    }

    function nowMs() {
      return (typeof performance !== 'undefined' && performance.now)
        ? performance.now()
        : Date.now();
    }

    function cancelHeadingSchedule() {
      if (__headingRafId != null) {
        clearTimeout(__headingRafId);
        __headingRafId = null;
      }
      __headingPending = null;
    }

    function applyHeadingVisuals(deg, latArg, lonArg) {
      const d = ((Number(deg) || 0) % 360 + 360) % 360;
      window.__heading = d;
      updateCompassNeedle(d);
      const p = vehicleMarker.getLatLng();
      const lat = latArg !== undefined ? latArg : p.lat;
      const lon = lonArg !== undefined ? lonArg : p.lng;
      updateHeadingLineGeometry(lat, lon, d);
      try {
        if (vehicleEntity && vehicleEntity.billboard) {
          vehicleEntity.billboard.rotation = Cesium.Math.toRadians(d);
        }
      } catch (e) {}
      try {
        const el = vehicleMarker.getElement && vehicleMarker.getElement();
        if (el) {
          const inner = el.querySelector('.vgcs-vehicle-marker-inner');
          if (inner) inner.style.transform = 'rotate(' + d + 'deg)';
        }
      } catch (e) {}
      __headingLastApplyMs = nowMs();
    }

    function flushHeadingPending() {
      __headingRafId = null;
      if (!__headingPending) return;
      const { deg, latArg, lonArg, source } = __headingPending;
      __headingPending = null;
      if (source === 'att' && (__headingLastVfrMs > 0) && (nowMs() - __headingLastVfrMs < VFR_HEADING_PRIORITY_MS)) {
        return;
      }
      if (source === 'vfr' || source === 'gpi') {
        __headingLastVfrMs = nowMs();
      }
      applyHeadingVisuals(deg, latArg, lonArg);
    }

    function scheduleHeadingUpdate(deg, latArg, lonArg, source) {
      __headingPending = { deg, latArg, lonArg, source: source || 'mixed' };
      const t = nowMs();
      const wait = Math.max(0, HEADING_MIN_INTERVAL_MS - (t - __headingLastApplyMs));
      if (__headingRafId != null) {
        clearTimeout(__headingRafId);
      }
      __headingRafId = setTimeout(flushHeadingPending, wait);
    }

    function updateHeading(deg, latArg, lonArg, source) {
      if (latArg !== undefined || lonArg !== undefined) {
        applyHeadingVisuals(deg, latArg, lonArg);
        return;
      }
      scheduleHeadingUpdate(deg, latArg, lonArg, source);
    }

    function enableAddWaypoint() { addMode = true; addFenceMode = false; }
    function enableFencePolygon() { addFenceMode = true; addMode = false; }

    function waypointNumberIcon(n) {
      const html =
        '<div class="vgcs-wp-pin" aria-label="Waypoint ' + n + '">' +
        '<span class="vgcs-wp-disc"></span>' +
        '<span class="vgcs-wp-num">' + n + '</span></div>';
      return L.divIcon({
        html: html,
        className: 'vgcs-wp-divicon',
        iconSize: [26, 26],
        iconAnchor: [13, 13],
      });
    }

    function refreshWaypointLabels() {
      waypoints.forEach((w, i) => {
        try {
          w.setIcon(waypointNumberIcon(i + 1));
        } catch (e) {}
      });
    }

    function attachWaypointDeleteHandlers(marker) {
      if (!marker) return;
      const removeThis = () => {
        try { map.removeLayer(marker); } catch (e) {}
        waypoints = waypoints.filter(w => w !== marker);
        refreshWaypointLabels();
        syncVehicleHeadingLine();
        updateMissionRoutePolyline();
      };
      // Fast delete gesture for planning workflows.
      marker.on('dblclick', function(ev) {
        if (ev) {
          ev.originalEvent?.preventDefault?.();
          ev.originalEvent?.stopPropagation?.();
        }
        removeThis();
      });
      marker.on('contextmenu', function(ev) {
        if (ev) {
          ev.originalEvent?.preventDefault?.();
          ev.originalEvent?.stopPropagation?.();
        }
        removeThis();
      });
    }

    function addWaypointMarker(latlng) {
      const idx = waypoints.length + 1;
      const m = L.marker(latlng, {
        icon: waypointNumberIcon(idx),
        interactive: true,
        bubblingMouseEvents: false,
        keyboard: false,
        pane: 'overlayPane',
      }).addTo(map);
      attachWaypointDeleteHandlers(m);
      waypoints.push(m);
      refreshWaypointLabels();
      syncVehicleHeadingLine();
      renderPlanWpDetails();
      return m;
    }

    function clearWaypoints() {
      for (const wp of waypoints) map.removeLayer(wp);
      waypoints = [];
      window.__missionNavSeq = 0;
      window.__planWpMeta = [];
      syncVehicleHeadingLine();
      updateMissionRoutePolyline();
      return 0;
    }

    let fenceCircle = null;
    function setFence(lat, lon, radiusM) {
      if (fenceCircle) map.removeLayer(fenceCircle);
      fenceCircle = L.circle([lat, lon], {
        radius: radiusM,
        color: '#f87171',
        fillColor: '#f87171',
        fillOpacity: 0.08,
        weight: 2
      }).addTo(map);
    }

    function clearFence() {
      if (fenceCircle) {
        map.removeLayer(fenceCircle);
        fenceCircle = null;
      }
      if (fencePolygon) {
        map.removeLayer(fencePolygon);
        fencePolygon = null;
      }
      fencePoints = [];
      return 1;
    }

    function getFencePoints() { return fencePoints.slice(); }

    function setFencePolygon(points) {
      if (fencePolygon) map.removeLayer(fencePolygon);
      fencePoints = points || [];
      if (fencePoints.length >= 3) {
        fencePolygon = L.polygon(fencePoints, {
          color: '#f97316',
          fillColor: '#f97316',
          fillOpacity: 0.08,
          weight: 2
        }).addTo(map);
      }
      return fencePoints.length;
    }

    function getWaypoints() {
      return waypoints.map(w => [w.getLatLng().lat, w.getLatLng().lng]);
    }

    function setWaypoints(points) {
      clearWaypoints();
      for (const p of points) {
        addWaypointMarker([p[0], p[1]]);
      }
      updateMissionRoutePolyline();
      renderPlanWpDetails();
    }

    function getWaypointCount() { return waypoints.length; }

    function set3DEnabled(enabled) {
      if (enabled) {
        if (!ensure3D()) return false;
        document.getElementById('map2d').style.display = 'none';
        document.getElementById('map3d').style.display = 'block';
        window.__is3d = true;
        if (hdrMapModeBtn) hdrMapModeBtn.textContent = '2D';
        // Ensure vehicle marker is in view and updating when switching modes.
        try {
          if (viewer3d && vehicleEntity) {
            try { vehicleEntity.show = true; } catch (e) {}
            viewer3d.trackedEntity = vehicleEntity;
            setTimeout(() => { try { viewer3d.trackedEntity = undefined; } catch (e) {} }, 900);
            // Hard focus: trackedEntity can be flaky in some WebEngine builds; fly/zoom as fallback.
            try {
              viewer3d.flyTo(vehicleEntity, {
                duration: 0.6,
                offset: new Cesium.HeadingPitchRange(
                  Cesium.Math.toRadians(Number(window.__heading || 0)),
                  Cesium.Math.toRadians(-40.0),
                  1800.0
                )
              });
            } catch (e) {
              try { viewer3d.zoomTo(vehicleEntity); } catch (e2) {}
            }
            try { viewer3d.scene && viewer3d.scene.requestRender && viewer3d.scene.requestRender(); } catch (e) {}
          }
        } catch (e) {}
        focus3DCamera(true);
        return true;
      }
      document.getElementById('map3d').style.display = 'none';
      document.getElementById('map2d').style.display = 'block';
      window.__is3d = false;
      if (hdrMapModeBtn) hdrMapModeBtn.textContent = '3D';
      return false;
    }

    map.on('click', function(e) {
      if (addMode) {
        addWaypointMarker(e.latlng);
        updateMissionRoutePolyline();
        return;
      }
      if (addFenceMode) {
        fencePoints.push([e.latlng.lat, e.latlng.lng]);
        setFencePolygon(fencePoints);
      }
    });
  </script>
</body>
</html>
"""


class _VideoEncodeBridge(QObject):
    encoded = Signal(str)


class _VideoEncodeTask(QRunnable):
    def __init__(self, img, bridge: _VideoEncodeBridge) -> None:
        super().__init__()
        self._img = img
        self._bridge = bridge

    def run(self) -> None:
        try:
            img = self._img
            if img is None or img.isNull():
                return
            try:
                img = img.scaled(
                    230,
                    130,
                    Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                    Qt.TransformationMode.SmoothTransformation,
                )
            except Exception:
                pass
            ba = QByteArray()
            buf = QBuffer(ba)
            if not buf.open(QBuffer.OpenModeFlag.WriteOnly):
                return
            try:
                img.save(buf, "PNG")
            finally:
                buf.close()
            raw = bytes(ba)
            if not raw:
                return
            data_url = "data:image/png;base64," + base64.b64encode(raw).decode("ascii")
            self._bridge.encoded.emit(data_url)
        except Exception:
            return


class _TileHeaderInterceptor(QWebEngineUrlRequestInterceptor):  # type: ignore[misc]
    """Attach browser-like headers to tile requests.

    Some tile providers block desktop apps when the referrer is file:// or missing.
    """

    def __init__(self) -> None:
        super().__init__()

    def interceptRequest(self, info) -> None:  # pragma: no cover - runtime/Qt dependent
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
            # OSM blocks many desktop apps unless a proper https referrer is present.
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
    """Forward JS console messages to Python stdout for client-side diagnostics."""

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID) -> None:  # pragma: no cover
        try:
            src = str(sourceID or "")
            msg = str(message or "")
            print(f"[VGCS:map] {src}:{int(lineNumber)} {msg}")
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
            req = Request(
                url,
                headers={
                    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36",
                    "Referer": "https://www.arcgis.com/" if "arcgisonline.com" in url.lower() else "https://www.openstreetmap.org/",
                },
                method="GET",
            )
            with urlopen(req, timeout=3.0) as resp:
                code = getattr(resp, "status", None) or resp.getcode()
                ctype = str(getattr(resp, "headers", {}).get("Content-Type", "") or "")
                raw = resp.read()
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
    mission_start_requested = Signal()
    plan_mission_panel_changed = Signal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._lat: float | None = None
        self._lon: float | None = None
        self._heading: float | None = None
        self._waypoint_count = 0
        self._waypoints_model: list[Waypoint] = []
        self._web_ready = False
        self._is_3d_mode = False
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
        tools.addWidget(self._btn_fence_apply, 0, 12)
        tools.addWidget(self._btn_fence_clear, 0, 13)
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

        panel_layout.addWidget(self._map_canvas)
        panel_layout.addWidget(toolbar)
        panel_layout.addWidget(status_box)
        panel.setLayout(panel_layout)
        root.addWidget(panel)
        self.setLayout(root)

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

    def set_link_connected(self, connected: bool) -> None:
        c = bool(connected)
        if self._last_link_connected == c:
            return
        self._last_link_connected = c
        try:
            print(f"[VGCS:map] link_connected={c}")
        except Exception:
            pass
        self._run_js("setLinkConnected(true);" if c else "setLinkConnected(false);")
        # Run a one-time tile probe after connect to log "blocked vs placeholder" clearly.
        if c and not getattr(self, "_tile_probe_ran", False):
            self._tile_probe_ran = True
            try:
                QTimer.singleShot(1500, lambda: self._probe_current_tiles(reason="connect"))
            except Exception:
                pass
        # Keep camera preview in sync with link status, but never auto-enable the webcam.
        if not c:
            self._stop_video_preview(clear_overlay=True)
            return
        if bool(getattr(self, "_btn_webcam", None)) and bool(self._btn_webcam.isChecked()):
            self._start_video_preview()

    def _probe_current_tiles(self, *, reason: str) -> None:
        # Probe the *current* view tile (not just z=0), because placeholders often occur only at higher zooms.
        def _kick(payload: str | None) -> None:
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
                        ("esri_imagery_view", "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/10/0/0"),
                        ("esri_streets_view", "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/10/0/0"),
                    ]
                )

            # Always also probe z=0 as a baseline.
            if tmpl:
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

        try:
            self._run_js("window.__vgcsGetMapView ? window.__vgcsGetMapView() : '';", callback=_kick)
        except Exception:
            _kick(None)

    def _on_tile_probe_result(self, provider_label: str, outcome: str, detail: str) -> None:
        try:
            print(f"[VGCS:map] tile_probe {provider_label} -> {outcome} ({detail})")
        except Exception:
            pass
        # If the *active view* tile is a placeholder, auto-fallback to Streets and guide the user.
        if "active_view" in str(provider_label) and outcome == "placeholder_suspected":
            self._set_status("Satellite tiles are placeholders — auto-switching to Streets")
            try:
                self.activate_esri_street_tiles()
            except Exception:
                pass
        if "esri_imagery" in str(provider_label) and outcome == "placeholder_suspected":
            self._set_status("Satellite tiles blocked/placeholder — switch to Streets or Offline Tiles…")

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
        # Only start if we're linked and the page is ready.
        if bool(getattr(self, "_last_link_connected", False)) and bool(getattr(self, "_web_ready", False)):
            self._start_video_preview()
            self._set_status("Webcam enabled")
        else:
            self._set_status("Webcam enabled (will start when connected)")

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
                self._video_push_timer.setInterval(400 if on else 200)
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
        if st not in {"green", "yellow", "red"}:
            st = "red"
        d = str(detail)
        key = (st, d)
        if self._last_flight_status_key == key:
            return
        self._last_flight_status_key = key
        self._run_js(f"setFlightStatus({json.dumps(st)}, {json.dumps(detail)});")

    def set_header_mode(self, mode_text: str) -> None:
        t = str(mode_text)
        if t == self._last_header_mode:
            return
        self._last_header_mode = t
        self._run_js(f"setHeaderMode({json.dumps(mode_text)});")

    def set_header_vehicle_msg(self, msg_text: str) -> None:
        self._run_js(f"setHeaderVehicleMsg({json.dumps(msg_text)});")

    def set_header_gps(self, satellites: int | str, hdop_text: str) -> None:
        key = (str(satellites), str(hdop_text))
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

    def set_plan_flight_visible(self, visible: bool) -> None:
        if visible:
            t = self._plan_rail_tool_state
            self._run_js(
                f"window.__planRailTool = {json.dumps(t)}; setPlanFlightVisible(true);"
            )
        else:
            self._run_js("setPlanFlightVisible(false);")

    def set_plan_flight_metrics(
        self,
        *,
        alt_diff_ft: str,
        gradient: str,
        azimuth: str,
        heading: str,
        dist_prev_wp_ft: str,
        mission_distance_ft: str,
        mission_time: str,
        max_telem_dist_ft: str,
    ) -> None:
        payload = {
            "altDiffFt": alt_diff_ft,
            "gradient": gradient,
            "azimuth": azimuth,
            "heading": heading,
            "distPrevWpFt": dist_prev_wp_ft,
            "missionDistanceFt": mission_distance_ft,
            "missionTime": mission_time,
            "maxTelemDistFt": max_telem_dist_ft,
        }
        if payload == self._last_plan_flight_metrics_payload:
            return
        self._last_plan_flight_metrics_payload = payload
        self._run_js(f"setPlanFlightMetrics({json.dumps(payload)});")

    def refresh_plan_flight_chrome(self, *, link_ok: bool, waypoint_count: int) -> None:
        self._run_js(
            "setPlanFlightChromeState("
            f"{str(bool(link_ok)).lower()}, {max(0, int(waypoint_count))});"
        )

    def center_on_vehicle(self) -> None:
        self._run_js("centerOnVehicle();")

    def set_plan_rail_tool(self, tool: str) -> None:
        t = (tool or "").strip()
        if not t:
            return
        self._plan_rail_tool_state = t
        self._run_js(
            f"window.__planRailTool = {json.dumps(t)}; setPlanRailTool({json.dumps(t)});"
        )

    def apply_plan_mission_panel_state(self, state: dict[str, object]) -> None:
        self._run_js(f"applyPlanMissionPanelState({json.dumps(state)});")

    def set_plan_sequence_template(self, template_id: str | None) -> None:
        """Show/hide Mission tab pattern row (Survey / Corridor / Structure) to match template picks."""
        tid = (template_id or "").strip().lower()
        self._run_js(f"setPlanSequenceTemplate({json.dumps(tid)});")

    def set_plan_mission_start_stack(self, enabled: bool, survey_label: str = "Survey") -> None:
        self._run_js(
            "setPlanMissionStartStack("
            f"{str(bool(enabled)).lower()}, {json.dumps(str(survey_label))}"
            ");"
        )

    def set_plan_vehicle_info(self, firmware: str, vehicle: str) -> None:
        key = (str(firmware), str(vehicle))
        if key == self._last_plan_vehicle_info_key:
            return
        self._last_plan_vehicle_info_key = key
        self._run_js(
            f"setPlanVehicleInfo({json.dumps(firmware)}, {json.dumps(vehicle)});"
        )

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
        if HAS_WEBENGINE and QWebEngineView is not None:
            self._web = QWebEngineView()
            self._web.setMinimumHeight(260)
            self._web.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
            self._map_canvas.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
            self._map_canvas.setAutoFillBackground(True)
            # Enable persistent HTTP cache so 2D tiles load much faster after first view.
            try:
                if QWebEngineProfile is not None:
                    prof = QWebEngineProfile.defaultProfile()
                    # Attach headers to avoid tile-provider blocks on some client networks.
                    try:
                        if QWebEngineUrlRequestInterceptor is not None:
                            self._tile_interceptor = _TileHeaderInterceptor()
                            prof.setUrlRequestInterceptor(self._tile_interceptor)
                    except Exception:
                        pass
                    cache_root = (Path.home() / ".vgcs-webengine-cache").resolve()
                    cache_root.mkdir(parents=True, exist_ok=True)
                    prof.setCachePath(str(cache_root))
                    prof.setPersistentStoragePath(str(cache_root))
                    # Disk cache; 512MB cap (tunable).
                    try:
                        prof.setHttpCacheType(QWebEngineProfile.HttpCacheType.DiskHttpCache)
                        prof.setHttpCacheMaximumSize(512 * 1024 * 1024)
                    except Exception:
                        pass
            except Exception:
                pass
            # Print JS console logs to the VGCS terminal (client diagnostics).
            try:
                if QWebEnginePage is not None:
                    self._web.setPage(_LoggingWebPage(self._web))
            except Exception:
                pass
            if QWebEngineSettings is not None:
                settings = self._web.settings()
                settings.setAttribute(
                    QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True
                )
                # Required for offline tiles (file:///.../z/x/y.png) while page baseUrl is assets/.
                try:
                    settings.setAttribute(
                        QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True
                    )
                except Exception:
                    pass
                settings.setAttribute(
                    QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled, False
                )
                settings.setAttribute(
                    QWebEngineSettings.WebAttribute.ScrollAnimatorEnabled, False
                )
            assets_root = (Path(__file__).resolve().parents[1] / "assets").resolve()
            base = QUrl.fromLocalFile(str(assets_root) + "/")
            self._web.setHtml(self._build_leaflet_html(), base)
            self._web.loadFinished.connect(self._on_map_loaded)
            self._web.titleChanged.connect(self._on_web_title_changed)
            self._map_canvas_layout.addWidget(self._web)
            self._set_status("Map backend: Leaflet (WebEngine)")
        # Apply saved performance preference immediately; Auto detection will run after load.
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
            return

        backend_notice = QLabel(
            "Qt WebEngine is not available. Install PySide6 WebEngine modules to enable the interactive map."
        )
        backend_notice.setWordWrap(True)
        backend_notice.setAlignment(Qt.AlignCenter)
        backend_notice.setStyleSheet("color: #a8b0c4; padding: 20px;")
        self._map_canvas_layout.addWidget(backend_notice)
        self._set_status("Map backend unavailable")

    def _build_leaflet_html(self) -> str:
        """Build map HTML. Image refs are paths relative to the assets/ base URL (see _init_map_backend).

        Large PNGs are not base64-inlined: multi-megabyte data URIs choke Qt WebEngine and yield a blank page.
        """
        assets_dir = Path(__file__).resolve().parents[1] / "assets"
        assets_root = assets_dir.resolve()

        def src_under_assets(path: Path) -> str:
            rel = path.resolve().relative_to(assets_root)
            return "/".join(quote(part, safe="") for part in rel.parts)

        logo_candidates = [
            assets_dir / "Vama Logo.png",
            assets_dir / "vama_logo.jpg",
            Path(__file__).resolve().parents[2] / "Vama Logo New.png",
        ]
        logo_src = ""
        for p in logo_candidates:
            if not p.is_file():
                continue
            pr = p.resolve()
            try:
                logo_src = src_under_assets(pr)
                break
            except ValueError:
                try:
                    raw = pr.read_bytes()
                except Exception:
                    continue
                if not raw:
                    continue
                mime = "image/png" if pr.suffix.lower() == ".png" else "image/jpeg"
                logo_src = f"data:{mime};base64,{base64.b64encode(raw).decode('ascii')}"
                break
        icon_files = {
            "__ICON_HOLD_SRC__": assets_dir / "header_icons" / "hold.svg",
            "__ICON_LINK_SRC__": assets_dir / "header_icons" / "link.svg",
            "__ICON_GPS_SRC__": assets_dir / "header_icons" / "gps.svg",
            "__ICON_BATTERY_SRC__": assets_dir / "header_icons" / "battery.svg",
            "__ICON_REMOTE_ID_SRC__": assets_dir / "header_icons" / "remote_id.svg",
        }
        icon_data: dict[str, str] = {}
        for token, icon_path in icon_files.items():
            if not icon_path.is_file():
                icon_data[token] = ""
                continue
            try:
                icon_data[token] = src_under_assets(icon_path)
            except ValueError:
                icon_data[token] = ""

        empty_plan_src = ""
        # Prefer the corrected filename, but keep typo fallback for older local worktrees.
        for _empty_name in ("empty plan.png", "emtpy plan.png"):
            ep = assets_dir / _empty_name
            if ep.is_file():
                try:
                    empty_plan_src = src_under_assets(ep)
                except ValueError:
                    empty_plan_src = quote(_empty_name, safe="")
                break
        survey_p = assets_dir / "survey.png"
        corr_p = assets_dir / "Corridor Scan.png"
        stru_p = assets_dir / "Structure Scan.png"
        plan_tpl_images = {
            "__PLAN_TPL_EMPTY_SRC__": empty_plan_src,
            "__PLAN_TPL_SURVEY_SRC__": src_under_assets(survey_p) if survey_p.is_file() else "",
            "__PLAN_TPL_CORRIDOR_SRC__": src_under_assets(corr_p) if corr_p.is_file() else "",
            "__PLAN_TPL_STRUCTURE_SRC__": src_under_assets(stru_p) if stru_p.is_file() else "",
        }

        html = LEAFLET_HTML.replace("__LOGO_SRC__", logo_src)
        for token, data_uri in icon_data.items():
            html = html.replace(token, data_uri)
        for token, data_uri in plan_tpl_images.items():
            html = html.replace(token, data_uri)
        return html

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
            # Tile selection strategy:
            # - If offline tiles are configured, always prefer them (guaranteed).
            # - Otherwise, always TRY satellite first on every launch (matches desired default),
            #   then allow fallback (probe) to Streets if the client network returns placeholders.
            try:
                s = QSettings(_QS_NS, _QS_APP)
                root = str(s.value(_KEY_MAP_OFFLINE_TILE_ROOT, "") or "").strip()
                if root and Path(root).is_dir():
                    self.activate_offline_tiles(root)
                else:
                    # Always try Satellite first (even if a client previously used Streets).
                    self.activate_satellite_tiles()
                    # Run a quick probe to detect placeholders and fallback automatically if needed.
                    try:
                        QTimer.singleShot(1200, lambda: self._probe_current_tiles(reason="startup"))
                    except Exception:
                        pass
            except Exception:
                pass
            self.map_page_ready.emit()
            # If we became ready after link-up, only start preview if user enabled it.
            try:
                if bool(self._last_link_connected) and bool(self._btn_webcam.isChecked()):
                    self._start_video_preview()
            except Exception:
                pass
            # Auto-detect low-spec devices and reduce map workload if needed.
            try:
                QTimer.singleShot(250, self._maybe_autodetect_low_spec)
            except Exception:
                pass
        else:
            self._set_status("Map failed to load")

    def _ensure_video_preview_backend(self) -> bool:
        if not HAS_MULTIMEDIA:
            return False
        if getattr(self, "_video_inited", False):
            return bool(getattr(self, "_camera", None)) and bool(getattr(self, "_video_sink", None))

        self._video_inited = True
        self._camera = None
        self._capture_session = None
        self._video_sink = None
        self._video_last_data_url = ""
        self._video_encode_bridge = _VideoEncodeBridge(self)
        self._video_encode_bridge.encoded.connect(self._on_video_frame_encoded)
        self._video_encode_inflight = False
        self._video_encode_pending = None
        self._video_pool = QThreadPool.globalInstance()
        self._video_push_timer = QTimer(self)
        self._video_push_timer.setInterval(200)  # 5 fps push to WebEngine
        self._video_push_timer.timeout.connect(self._push_video_preview_to_overlay)

        try:
            devices = list(QMediaDevices.videoInputs()) if QMediaDevices is not None else []
        except Exception:
            devices = []
        if not devices:
            return False

        try:
            self._video_sink = QVideoSink(self)
            self._video_sink.videoFrameChanged.connect(self._on_video_frame_changed)
            self._capture_session = QMediaCaptureSession(self)
            self._capture_session.setVideoSink(self._video_sink)
            self._camera = QCamera(devices[0])
            self._capture_session.setCamera(self._camera)
        except Exception:
            self._camera = None
            self._capture_session = None
            self._video_sink = None
            return False
        return True

    def _start_video_preview(self) -> None:
        if not getattr(self, "_web_ready", False):
            return
        if not self._ensure_video_preview_backend():
            # No multimedia backend / camera: keep placeholder visible.
            self._run_js("setVideoPreviewImage('');")
            return
        try:
            if self._camera is not None:
                self._camera.start()
            if hasattr(self, "_video_push_timer") and not self._video_push_timer.isActive():
                self._video_push_timer.start()
        except Exception:
            self._run_js("setVideoPreviewImage('');")

    def _stop_video_preview(self, *, clear_overlay: bool) -> None:
        if hasattr(self, "_video_push_timer") and self._video_push_timer.isActive():
            self._video_push_timer.stop()
        try:
            if getattr(self, "_camera", None) is not None:
                self._camera.stop()
        except Exception:
            pass
        if clear_overlay and getattr(self, "_web_ready", False):
            self._run_js("setVideoPreviewImage('');")

    def _on_video_frame_changed(self, frame) -> None:
        # Called on the GUI thread; offload encoding to a worker to avoid UI freezes on low-end devices.
        try:
            img = frame.toImage()
        except Exception:
            return
        if img is None or img.isNull():
            return
        try:
            img2 = img.copy()
        except Exception:
            img2 = img

        if bool(getattr(self, "_video_encode_inflight", False)):
            self._video_encode_pending = img2
            return

        self._video_encode_inflight = True
        task = _VideoEncodeTask(img2, self._video_encode_bridge)
        try:
            self._video_pool.start(task)
        except Exception:
            # If threadpool is unavailable, drop rather than blocking the GUI.
            self._video_encode_inflight = False
            return

    def _on_video_frame_encoded(self, data_url: str) -> None:
        self._video_last_data_url = str(data_url or "")
        self._video_encode_inflight = False
        pending = getattr(self, "_video_encode_pending", None)
        if pending is None:
            return
        self._video_encode_pending = None
        self._video_encode_inflight = True
        task = _VideoEncodeTask(pending, self._video_encode_bridge)
        try:
            self._video_pool.start(task)
        except Exception:
            self._video_encode_inflight = False
            return

    def _push_video_preview_to_overlay(self) -> None:
        if not getattr(self, "_web_ready", False):
            return
        src = str(getattr(self, "_video_last_data_url", "") or "")
        if not src:
            return
        # Avoid spamming WebEngine with identical payloads.
        last = str(getattr(self, "_last_video_pushed", "") or "")
        if src == last:
            return
        self._last_video_pushed = src
        self._run_js(f"setVideoPreviewImage({json.dumps(src)});")

    def _on_web_title_changed(self, title: str) -> None:
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
                    if hasattr(self, "_web"):
                        gp = self._web.mapToGlobal(QPoint(vx, vy))
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

    def _run_js(self, script: str, callback=None) -> None:
        if not getattr(self, "_web_ready", False):
            return
        if not hasattr(self, "_web"):
            return
        if callback is None:
            self._web.page().runJavaScript(script)
            return
        self._web.page().runJavaScript(script, callback)
        # Best-effort: capture tile template when JS reports it.
        try:
            self._web.page().runJavaScript("window.__lastTileTemplate || '';", lambda v: setattr(self, "_last_tile_template", str(v or "")))
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
        lat = float(self._lat)
        lon = float(self._lon)
        hd = float(self._heading) if self._heading is not None else 0.0
        src = self._heading_js_source or "mixed"
        self._run_js(
            f"setVehicle({lat:.8f}, {lon:.8f}); "
            f"updateHeading({hd:.2f}, undefined, undefined, {json.dumps(src)});"
        )

    def set_vehicle_position(self, lat: float, lon: float, *, relative_alt_m: float | None = None) -> None:
        first_fix = self._lat is None or self._lon is None
        self._lat = lat
        self._lon = lon
        if relative_alt_m is None:
            self._coords.setText(f"Lat/Lon: {lat:.7f}, {lon:.7f}")
        else:
            self._coords.setText(
                f"Lat/Lon: {lat:.7f}, {lon:.7f}  |  Rel Alt: {relative_alt_m:.1f} m"
            )
        self._schedule_vehicle_pose_js(immediate=first_fix)

    def set_vehicle_heading(self, heading_deg: float, *, source: str = "mixed") -> None:
        self._heading = heading_deg % 360.0
        self._heading_js_source = source or "mixed"
        self._heading_label.setText(f"Heading: {self._heading:.1f}°")
        self._schedule_vehicle_pose_js(immediate=False)

    def clear_flight_track(self) -> None:
        """Clear the orange breadcrumb trail (e.g. on reconnect / disconnect)."""
        self._run_js("clearFlightTrack();")

    def set_mission_nav_seq(self, seq: int) -> None:
        """MAVLink MISSION_CURRENT.seq: trim planned route / sync with vehicle progress."""
        self._run_js(
            f"window.__missionNavSeq = {max(0, int(seq))}; updateMissionRoutePolyline();"
        )

    def set_flight_telemetry(
        self,
        *,
        relative_alt_m: float,
        ground_speed_mps: float,
        flight_time_text: str,
        msl_alt_m: float,
    ) -> None:
        ft = f"{float(relative_alt_m) * 3.28084:.1f}"
        mph = f"{float(ground_speed_mps) * 2.23694:.1f}"
        ttime = str(flight_time_text)
        msl_ft = f"{float(msl_alt_m) * 3.28084:.1f}"
        sig = f"{ft}|{mph}|{ttime}|{msl_ft}"
        if sig == self._last_flight_telemetry_sig:
            return
        self._last_flight_telemetry_sig = sig
        self._run_js(
            "setTelemetryOverlay("
            f"{float(relative_alt_m):.3f}, "
            f"{float(ground_speed_mps):.3f}, "
            f"{json.dumps(ttime)}, "
            f"{float(msl_alt_m):.3f}"
            ");"
        )

    def set_mission_waypoint_count(self, count: int) -> None:
        self._waypoint_count = max(0, int(count))
        self._mission.setText(f"Mission WPs: {self._waypoint_count}")

    def _enable_add_waypoint_mode(self) -> None:
        self._run_js("enableAddWaypoint();")
        self._set_status("click on map to add waypoint")

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

    def _request_upload(self) -> None:
        self._run_js(
            "JSON.stringify(getWaypoints());",
            callback=lambda payload: self._emit_upload_from_json(payload),
        )

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

    def _emit_upload_from_json(self, payload: str | None) -> None:
        waypoints = self._waypoints_from_map_json(payload)
        if not waypoints:
            self._set_status("No waypoints to upload")
            return
        self.mission_upload_requested.emit(waypoints)
        self._set_status(f"Mission upload requested ({len(waypoints)} WPs)")

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
        def cb(payload: str | None) -> None:
            wps = self._waypoints_from_map_json(payload)
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

        self._run_js("JSON.stringify(getWaypoints());", callback=cb)

    def save_plan_mission_kml(self) -> None:
        def cb(payload: str | None) -> None:
            wps = self._waypoints_from_map_json(payload)
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

        self._run_js("JSON.stringify(getWaypoints());", callback=cb)


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

        def cb(payload: str | None) -> None:
            waypoints = self._waypoints_from_map_json(payload)
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

        self._run_js("JSON.stringify(getWaypoints());", callback=cb)

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
        self._run_js(
            "setTileSource('https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}', "
            "'Tiles © Esri', 19);"
        )
        self._set_status("Online tiles active (Esri Streets)")

    def activate_osm_tiles(self) -> None:
        """OSM tiles are often blocked for desktop apps (referrer policy). Keep optional."""
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "osm")
        except Exception:
            pass
        self._run_js(
            "setTileSource('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', "
            "'&copy; OpenStreetMap contributors', 19);"
        )
        self._set_status("Online tiles active (OSM)")

    def activate_satellite_tiles(self) -> None:
        try:
            QSettings(_QS_NS, _QS_APP).setValue(_KEY_MAP_TILE_MODE, "sat")
        except Exception:
            pass
        self._run_js(
            "setTileSource('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', "
            "'Tiles © Esri', 19);"
        )
        self._set_status("Satellite tiles active")

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
        self._run_js(f"setTileSource({json.dumps(tmpl)}, 'Offline tile cache', 19);")
        self._set_status("Offline tiles active")

    def _apply_geofence(self) -> None:
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
            self.geofence_upload_requested.emit(
                {
                    "radius_m": radius,
                    "alt_max_m": float(self._fence_alt_max.value()),
                    "center_lat": self._lat,
                    "center_lon": self._lon,
                }
            )
            self._set_status(f"Fence requested (r={radius:.0f}m)")

        self._run_js("JSON.stringify(getFencePoints());", callback=_after_fence_points)

    def _clear_geofence(self) -> None:
        self._run_js("clearFence();")
        self.geofence_upload_requested.emit({"disable": True})
        self._set_status("Fence cleared")

    def set_3d_enabled(self, enabled: bool) -> bool:
        self._is_3d_mode = bool(enabled and HAS_WEBENGINE)
        if not self._web_ready:
            if enabled:
                self._set_status("3D view unavailable: map backend not ready")
            else:
                self._set_status("2D mode active")
            return False
        desired = "true" if enabled else "false"
        self._run_js(
            f"set3DEnabled({desired});",
            callback=lambda ok: self._on_3d_toggle_result(enabled, ok),
        )
        return bool(enabled)

    def _on_3d_toggle_result(self, requested: bool, result: object) -> None:
        active = bool(result)
        self._is_3d_mode = active
        self._btn_3d.blockSignals(True)
        self._btn_3d.setChecked(active)
        self._btn_3d.blockSignals(False)
        if requested and active:
            self._set_status("3D mode active")
        elif requested:
            self._set_status("3D mode unavailable; using 2D")
        else:
            self._set_status("2D mode active")

