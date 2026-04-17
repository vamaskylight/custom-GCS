"""M2 map scaffold with live position API and WebEngine/Leaflet integration."""

from __future__ import annotations

import base64
import json
from pathlib import Path

from PySide6.QtCore import QPoint, QTimer, Qt, QUrl
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QFileDialog,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from vgcs.mission import Waypoint, load_waypoints_json, save_waypoints_json


try:
    from PySide6.QtWebEngineWidgets import QWebEngineView

    HAS_WEBENGINE = True
except Exception:  # pragma: no cover - environment-specific availability
    QWebEngineView = None  # type: ignore[assignment]
    HAS_WEBENGINE = False


LEAFLET_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <link rel="stylesheet" href="https://unpkg.com/cesium@1.125/Build/Cesium/Widgets/widgets.css"/>
  <style>
    html, body, #mapWrap, #map2d, #map3d { height:100%; margin:0; background:#1a1d24; }
    #mapWrap { position: relative; overflow: hidden; }
    #map2d, #map3d { position: absolute; inset: 0; }
    #map3d { display: none; }
    .leaflet-control-attribution { background: rgba(26,29,36,0.7); color:#a8b0c4; }
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
      background: rgba(250, 251, 253, 0.96);
      color:#111827;
      display:flex;
      align-items:stretch;
      gap:14px;
      padding: 6px 14px 8px;
      font-size:15px;
      border-bottom:1px solid rgba(108, 114, 126, 0.30);
      pointer-events:auto;
      flex-wrap: nowrap;
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
      color:#4b5563;
      font-weight:600;
      line-height:1.1;
      white-space: nowrap;
      letter-spacing: 0.01em;
    }
    #planFlightTopBar .pfMetric {
      font-size:13px;
      color:#111827;
      line-height:1.2;
      white-space:nowrap;
      font-weight: 400;
    }
    #planFlightTopBar .pfMetric b {
      font-weight: 700;
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
      color:#111827;
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
      width:74px;
      border-radius:8px;
      background: rgba(244, 245, 247, 0.97);
      border:1px solid rgba(130, 136, 146, 0.36);
      box-shadow: 0 2px 8px rgba(0,0,0,0.16);
      padding:5px 0;
      display:flex;
      flex-direction:column;
      gap:4px;
      pointer-events:auto;
    }
    .planToolBtn {
      margin:0 5px;
      min-height:52px;
      border-radius:6px;
      border:1px solid rgba(158, 164, 176, 0.34);
      background:#f8f8f8;
      color:#1f2937;
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
      background:#ad6a05;
      color:#ffffff;
      border-color:#8d5405;
    }
    .planToolBtn:hover { background:#eceff3; }
    #planCenterPanel {
      width:420px;
      border-radius:6px;
      overflow:hidden;
      background: rgba(238, 239, 241, 0.95);
      border:1px solid rgba(145, 150, 160, 0.45);
      box-shadow: 0 2px 10px rgba(0,0,0,0.16);
      pointer-events:auto;
    }
    #planCenterBanner {
      padding:8px 12px;
      font-size:14px;
      font-weight:500;
      color:#0f172a;
      background:#ffffff;
      border-bottom:1px solid rgba(165, 170, 182, 0.45);
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    #planCenterBody {
      padding:10px 12px 12px;
      color:#111827;
      font-size:12px;
    }
    #planCenterBody .title {
      font-size:30px;
      font-weight:700;
      margin-bottom:8px;
    }
    #planCards {
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap:7px;
      margin-top:6px;
    }
    .planCard {
      background:#e5e7eb;
      border:1px solid rgba(143, 149, 159, 0.45);
      border-radius:4px;
      min-height:104px;
      overflow:hidden;
      display:flex;
      flex-direction:column;
    }
    .planCardPreview {
      flex:1;
      min-height:76px;
      background: linear-gradient(140deg, #6f7783, #a2aab6);
    }
    .planCardLabel {
      background:#f3f4f6;
      border-top:1px solid rgba(143, 149, 159, 0.35);
      text-align:center;
      font-size:11px;
      font-weight:600;
      padding:4px 4px 6px;
      color:#111827;
    }
    #planRightPanel {
      margin-left:auto;
      width:282px;
      border-radius:5px;
      background: rgba(236, 239, 242, 0.95);
      border:1px solid rgba(135, 142, 153, 0.42);
      box-shadow: 0 2px 10px rgba(0,0,0,0.14);
      overflow:hidden;
      pointer-events:auto;
    }
    #planTabs {
      display:flex;
      height:34px;
      background:#eceef1;
      border-bottom:1px solid rgba(150, 156, 167, 0.42);
    }
    .planTab {
      flex:1;
      border:none;
      background:transparent;
      color:#1f2937;
      font-size:12px;
      font-weight:500;
    }
    .planTab.active {
      background:#b7791f;
      color:#ffffff;
      font-weight:600;
    }
    #planSection {
      padding:0;
    }
    .planSectionHeader {
      background:#b8dddd;
      color:#0f172a;
      font-size:13px;
      font-weight:600;
      padding:8px 10px;
    }
    .planSectionBody {
      padding:10px;
      background:#c8c8cb;
      color:#111827;
      font-size:12px;
    }
    .planFieldLabel {
      color:#374151;
      font-size:11px;
      margin:8px 0 3px;
    }
    .planFieldValue {
      background:#f3f4f6;
      border:1px solid rgba(136, 141, 151, 0.45);
      border-radius:4px;
      min-height:30px;
      display:flex;
      align-items:center;
      padding:0 10px;
      font-size:12px;
      color:#111827;
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
      margin-top:10px;
      width:100%;
      min-height:30px;
      border-radius:4px;
      border:1px solid rgba(140, 146, 156, 0.55);
      background:#ad6a05;
      color:#ffffff;
      font-size:12px;
      font-weight:700;
      cursor:pointer;
    }
    #planStartMissionBtn:hover {
      background:#c27a09;
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
      background: linear-gradient(180deg, rgba(39, 47, 61, 0.92), rgba(30, 38, 52, 0.92));
      border: 1px solid rgba(188, 202, 224, 0.42);
      backdrop-filter: blur(3px);
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
    #telemetryStrip {
      right:188px; bottom:12px; padding:10px 14px; border-radius:8px;
      background: rgba(26, 33, 45, 0.94); color:#dce5f5; font-size:15px;
      line-height: 1.35;
      display:flex; flex-direction:column; gap:4px;
      white-space: nowrap;
    }
    .telemetryRow {
      display:flex; align-items:center; gap:12px;
    }
    .telemetryItem {
      display:inline-flex;
      align-items:center;
      gap:6px;
    }
    .telemetryIcon {
      font-size:18px;
      line-height:1;
    }
    .telemetryIconHuman {
      font-size:22px;
      line-height:1;
    }
    #compass {
      right:8px; bottom:4px; width:176px; height:176px;
      background: transparent;
      display:flex; justify-content:center; align-items:center;
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
        <span class="hdrPill"><img class="hdrIcon hdrIconBroadcast" src="__ICON_LINK_SRC__" alt="Vehicle Message"/><span id="hdrVehicleMsg">Vehicle Msg</span></span>
        <span class="hdrSep"></span>
        <span class="hdrPill"><img class="hdrIcon hdrIconSmall" src="__ICON_GPS_SRC__" alt="GPS"/><span class="hdrTinyStack"><span id="hdrGpsSat">10</span><span id="hdrGpsHdop">0.7</span></span></span>
        <span class="hdrSep"></span>
        <span class="hdrPill"><img class="hdrIcon" src="__ICON_BATTERY_SRC__" alt="Battery"/><span id="hdrBatteryText">100%</span></span>
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
        <div class="pfGroup">
          <span class="pfLabel">Selected Waypoint</span>
          <span class="pfMetric">Alt diff: <b id="pfAltDiff">0.0 ft</b></span>
          <span class="pfMetric">Gradient: <b id="pfGradient">--</b></span>
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
          <div class="planToolBtn active" data-tool="File"><span class="planToolIcon">🗂</span><span>File</span></div>
          <div class="planToolBtn" data-tool="Takeoff"><span class="planToolIcon">↑</span><span>Takeoff</span></div>
          <div class="planToolBtn" data-tool="Waypoint"><span class="planToolIcon">⊕</span><span>Waypoint</span></div>
          <div class="planToolBtn" data-tool="ROI"><span class="planToolIcon">◉</span><span>ROI</span></div>
          <div class="planToolBtn" data-tool="Pattern"><span class="planToolIcon">▦</span><span>Pattern</span></div>
          <div class="planToolBtn" data-tool="Return"><span class="planToolIcon">↩</span><span>Return</span></div>
          <div class="planToolBtn" data-tool="Center"><span class="planToolIcon">✦</span><span>Center</span></div>
        </div>
        <div id="planCenterPanel">
          <div id="planCenterBanner">You have unsaved changes.</div>
          <div id="planCenterBody">
            <div class="title">Create Plan</div>
            <div id="planCards">
              <div class="planCard">
                <div class="planCardPreview"></div>
                <div class="planCardLabel">Empty Plan</div>
              </div>
              <div class="planCard">
                <div class="planCardPreview"></div>
                <div class="planCardLabel">Survey</div>
              </div>
              <div class="planCard">
                <div class="planCardPreview"></div>
                <div class="planCardLabel">Corridor</div>
              </div>
              <div class="planCard">
                <div class="planCardPreview"></div>
                <div class="planCardLabel">Structure</div>
              </div>
            </div>
          </div>
        </div>
        <div id="planRightPanel">
          <div id="planTabs">
            <button class="planTab active" type="button">Mission</button>
            <button class="planTab" type="button">Fence</button>
            <button class="planTab" type="button">Rally</button>
          </div>
          <div id="planSection">
            <div class="planSectionHeader">Mission Start</div>
            <div class="planSectionBody">
              <div class="planFieldLabel">All Altitudes</div>
              <div class="planFieldValue">Relative To Launch ▼</div>
              <div class="planFieldLabel">Initial Waypoint Alt</div>
              <div class="planFieldValue">164.0 ft</div>
              <button id="planStartMissionBtn" type="button">Start Mission</button>
              <div class="planFold">Vehicle Info <span>▼</span></div>
              <div class="planFold">Launch Position <span>▼</span></div>
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
    <div class="overlay" id="telemetryStrip"><div class="telemetryRow"><span class="telemetryItem"><span class="telemetryIcon">↕</span><span>0.0 ft</span></span><span class="telemetryItem"><span class="telemetryIcon">↑</span><span>0.0 mph</span></span><span class="telemetryItem"><span class="telemetryIcon">⏱</span><span>00:00:00</span></span></div><div class="telemetryRow"><span class="telemetryItem"><span class="telemetryIcon">↳</span><span>0.0 ft</span></span><span class="telemetryItem"><span class="telemetryIcon">→</span><span>0.0 mph</span></span><span class="telemetryItem"><span class="telemetryIcon telemetryIconHuman">&#128100;&#65038;</span><span>0.0 ft</span></span></div></div>
    <div class="overlay" id="compass">
      <div id="compassInner">
        <span class="compassCard" id="cN">N</span><span class="compassCard" id="cE">E</span><span class="compassCard" id="cS">S</span><span class="compassCard" id="cW">W</span>
        <div id="compassDeg">0°</div>
        <div id="needle"></div>
      </div>
    </div>
  </div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script src="https://unpkg.com/cesium@1.125/Build/Cesium/Cesium.js"></script>
  <script>
    const map = L.map('map2d', { zoomControl: false }).setView([24.7136, 46.6753], 10);
    L.control.zoom({ position: 'bottomleft' }).addTo(map);
    const linkBanner = document.getElementById('linkBanner');
    const linkBannerLogo = document.getElementById('linkBannerLogo');
    if (linkBannerLogo && linkBannerLogo.getAttribute('src')) {
      linkBannerLogo.style.display = 'block';
    }
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
    const hdrMapModeBtn = document.getElementById('hdrMapModeBtn');
    const planExit = document.getElementById('planExit');
    if (planExit) {
      planExit.addEventListener('click', function() {
        setPlanFlightVisible(false);
      });
    }
    if (planLayer) {
      for (const btn of planLayer.querySelectorAll('.planToolBtn')) {
        btn.addEventListener('click', function() {
          for (const el of planLayer.querySelectorAll('.planToolBtn')) {
            el.classList.remove('active');
          }
          btn.classList.add('active');
          const tool = btn.getAttribute('data-tool') || '';
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
    function setCameraControlsVisible(enabled) {
      if (!cameraRail) return 0;
      cameraRail.style.display = enabled ? 'flex' : 'none';
      if (videoPreview) videoPreview.style.display = enabled ? 'block' : 'none';
      if (!enabled) resetCameraTimer();
      return 1;
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
    function setTileSource(urlTemplate, attribution, maxZoom) {
      if (tileLayer) map.removeLayer(tileLayer);
      tileLayer = L.tileLayer(urlTemplate, {
        maxZoom: maxZoom || 19,
        attribution: attribution || ''
      }).addTo(map);
      return 1;
    }
    // Satellite-like default view similar to the provided reference.
    setTileSource(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      'Tiles © Esri',
      19
    );

    let vehicleMarker = L.circleMarker([24.7136, 46.6753], {
      radius: 7, color: '#4ade80', fillColor: '#4ade80', fillOpacity: 0.8
    }).addTo(map);
    let headingLine = L.polyline([[20,0], [20,0]], { color:'#fbbf24', weight:3 }).addTo(map);
    let waypoints = [];
    let addMode = false;
    let addFenceMode = false;
    let fencePoints = [];
    let fencePolygon = null;
    let viewer3d = null;
    let vehicleEntity = null;
    let headingEntity = null;
    window.__is3d = false;
    window.__heading = 0;
    window.__3dHasInitialFocus = false;

    function setLinkConnected(connected) {
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

    function setPlanFlightVisible(visible) {
      const layer = document.getElementById('planFlightLayer');
      if (!layer) return 0;
      layer.style.display = visible ? 'block' : 'none';
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
      setTxt('pfGradient', metrics.gradient ?? '--');
      setTxt('pfAzimuth', metrics.azimuth ?? '0');
      setTxt('pfHeading', metrics.heading ?? 'nan');
      setTxt('pfDistPrevWp', metrics.distPrevWpFt ?? '0.0 ft');
      setTxt('pfMissionDistance', metrics.missionDistanceFt ?? '0 ft');
      setTxt('pfMissionTime', metrics.missionTime ?? '00:00:00');
      setTxt('pfMaxTelemDist', metrics.maxTelemDistFt ?? '0 ft');
      return 1;
    }

    function centerOnVehicle() {
      if (!map || !vehicleMarker) return 0;
      const p = vehicleMarker.getLatLng();
      if (!p) return 0;
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
      el.textContent = raw ? raw.slice(0, 20) : 'Vehicle Msg';
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
      const strip = document.getElementById('telemetryStrip');
      if (!strip) return 0;
      strip.innerHTML = `<div class="telemetryRow"><span class="telemetryItem"><span class="telemetryIcon">↕</span><span>${ft} ft</span></span><span class="telemetryItem"><span class="telemetryIcon">↑</span><span>${mph} mph</span></span><span class="telemetryItem"><span class="telemetryIcon">⏱</span><span>${timeText || '00:00:00'}</span></span></div><div class="telemetryRow"><span class="telemetryItem"><span class="telemetryIcon">↳</span><span>${ft} ft</span></span><span class="telemetryItem"><span class="telemetryIcon">→</span><span>${mph} mph</span></span><span class="telemetryItem"><span class="telemetryIcon telemetryIconHuman">&#128100;&#65038;</span><span>${mslFt} ft</span></span></div>`;
      return 1;
    }

    function ensure3D() {
      if (viewer3d) return true;
      if (!window.Cesium) return false;
      try {
        viewer3d = new Cesium.Viewer('map3d', {
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
        // Force a known imagery layer for stable no-key rendering in WebEngine.
        try {
          viewer3d.imageryLayers.removeAll();
          viewer3d.imageryLayers.addImageryProvider(
            new Cesium.UrlTemplateImageryProvider({
              url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
              credit: 'Tiles © Esri'
            })
          );
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
        vehicleEntity = viewer3d.entities.add({
          position: Cesium.Cartesian3.fromDegrees(46.6753, 24.7136, 20),
          point: { pixelSize: 10, color: Cesium.Color.LIME }
        });
        headingEntity = viewer3d.entities.add({
          polyline: { positions: [], width: 2, material: Cesium.Color.ORANGE }
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

    function setVehicle(lat, lon) {
      vehicleMarker.setLatLng([lat, lon]);
      updateHeading(window.__heading || 0, lat, lon);
      if (vehicleEntity) {
        vehicleEntity.position = Cesium.Cartesian3.fromDegrees(lon, lat, 20);
      }
      if (window.__is3d && !window.__3dHasInitialFocus) {
        focus3DCamera(false);
      }
    }

    function updateHeading(deg, latArg, lonArg) {
      window.__heading = deg;
      updateCompassNeedle(deg);
      const p = vehicleMarker.getLatLng();
      const lat = latArg !== undefined ? latArg : p.lat;
      const lon = lonArg !== undefined ? lonArg : p.lng;
      const len = 0.01;
      const rad = deg * Math.PI / 180.0;
      const lat2 = lat + len * Math.cos(rad);
      const lon2 = lon + len * Math.sin(rad);
      headingLine.setLatLngs([[lat, lon], [lat2, lon2]]);
      if (headingEntity) {
        headingEntity.polyline.positions = Cesium.Cartesian3.fromDegreesArray([
          lon, lat, lon2, lat2
        ]);
      }
    }

    function enableAddWaypoint() { addMode = true; addFenceMode = false; }
    function enableFencePolygon() { addFenceMode = true; addMode = false; }

    function attachWaypointDeleteHandlers(marker) {
      if (!marker) return;
      const removeThis = () => {
        try { map.removeLayer(marker); } catch (e) {}
        waypoints = waypoints.filter(w => w !== marker);
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
      const m = L.marker(latlng).addTo(map);
      attachWaypointDeleteHandlers(m);
      waypoints.push(m);
      return m;
    }

    function clearWaypoints() {
      for (const wp of waypoints) map.removeLayer(wp);
      waypoints = [];
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
    }

    function getWaypointCount() { return waypoints.length; }

    function set3DEnabled(enabled) {
      if (enabled) {
        if (!ensure3D()) return false;
        document.getElementById('map2d').style.display = 'none';
        document.getElementById('map3d').style.display = 'block';
        window.__is3d = true;
        if (hdrMapModeBtn) hdrMapModeBtn.textContent = '2D';
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
    toggle_3d_requested = Signal()
    mission_start_requested = Signal()

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
        self._btn_export = QPushButton("Export Mission")
        self._btn_import = QPushButton("Import Mission")
        self._btn_3d = QPushButton("3D Toggle")
        self._btn_3d.setCheckable(True)
        self._btn_fence_poly = QPushButton("Fence Polygon")
        self._btn_tiles_online = QPushButton("Online Tiles")
        self._btn_tiles_pick = QPushButton("Offline Tiles…")
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
        self._wp_selector = QComboBox()
        self._wp_selector.setMinimumWidth(90)
        self._wp_alt = QDoubleSpinBox()
        self._wp_alt.setRange(1.0, 500.0)
        self._wp_alt.setDecimals(1)
        self._wp_alt.setSingleStep(1.0)
        self._wp_alt.setValue(20.0)
        self._btn_apply_wp_alt = QPushButton("Set WP Alt")
        self._btn_apply_all_alt = QPushButton("Set All Alt")
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
        tools.addWidget(QLabel("WP"), 1, 2)
        tools.addWidget(self._wp_selector, 1, 3)
        tools.addWidget(QLabel("Alt (m)"), 1, 4)
        tools.addWidget(self._wp_alt, 1, 5)
        tools.addWidget(self._btn_apply_wp_alt, 1, 6)
        tools.addWidget(self._btn_apply_all_alt, 1, 7)
        tools.addWidget(self._btn_tiles_online, 1, 8)
        tools.addWidget(self._btn_tiles_pick, 1, 9)
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
        self._btn_tiles_online.clicked.connect(self._set_online_tiles)
        self._btn_tiles_pick.clicked.connect(self._pick_offline_tiles)
        self._wp_selector.currentIndexChanged.connect(self._on_wp_selected)
        self._btn_apply_wp_alt.clicked.connect(self._apply_altitude_to_selected)
        self._btn_apply_all_alt.clicked.connect(self._apply_altitude_to_all)

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
        self._run_js("setLinkConnected(true);" if connected else "setLinkConnected(false);")

    def set_flight_status(self, status: str, detail: str = "") -> None:
        st = (status or "").strip().lower()
        if st not in {"green", "yellow", "red"}:
            st = "red"
        self._run_js(f"setFlightStatus({json.dumps(st)}, {json.dumps(detail)});")

    def set_header_mode(self, mode_text: str) -> None:
        self._run_js(f"setHeaderMode({json.dumps(mode_text)});")

    def set_header_vehicle_msg(self, msg_text: str) -> None:
        self._run_js(f"setHeaderVehicleMsg({json.dumps(msg_text)});")

    def set_header_gps(self, satellites: int | str, hdop_text: str) -> None:
        self._run_js(f"setHeaderGps({json.dumps(str(satellites))}, {json.dumps(hdop_text)});")

    def set_header_battery(self, battery_text: str) -> None:
        self._run_js(f"setHeaderBattery({json.dumps(battery_text)});")

    def set_header_remote_id(self, rid_text: str) -> None:
        self._run_js(f"setHeaderRemoteId({json.dumps(rid_text)});")

    def set_plan_flight_visible(self, visible: bool) -> None:
        self._run_js("setPlanFlightVisible(true);" if visible else "setPlanFlightVisible(false);")

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
        self._run_js(f"setPlanFlightMetrics({json.dumps(payload)});")

    def center_on_vehicle(self) -> None:
        self._run_js("centerOnVehicle();")

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
            self._web.setHtml(self._build_leaflet_html(), QUrl("https://vgcs.local/"))
            self._web.loadFinished.connect(self._on_map_loaded)
            self._web.titleChanged.connect(self._on_web_title_changed)
            self._map_canvas_layout.addWidget(self._web)
            self._set_status("Map backend: Leaflet (WebEngine)")
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
        assets_dir = Path(__file__).resolve().parents[1] / "assets"
        logo_candidates = [
            assets_dir / "Vama Logo.png",
            assets_dir / "vama_logo.jpg",
            Path(__file__).resolve().parents[2] / "Vama Logo New.png",
        ]
        logo_src = ""
        for p in logo_candidates:
            if not p.exists():
                continue
            try:
                raw = p.read_bytes()
            except Exception:
                continue
            if not raw:
                continue
            mime = "image/png" if p.suffix.lower() == ".png" else "image/jpeg"
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
            encoded = ""
            try:
                raw = icon_path.read_bytes()
                if raw:
                    encoded = f"data:image/svg+xml;base64,{base64.b64encode(raw).decode('ascii')}"
            except Exception:
                encoded = ""
            icon_data[token] = encoded

        html = LEAFLET_HTML.replace("__LOGO_SRC__", logo_src)
        for token, data_uri in icon_data.items():
            html = html.replace(token, data_uri)
        return html

    def _on_map_loaded(self, ok: bool) -> None:
        self._web_ready = bool(ok)
        if self._web_ready:
            self._set_status("Map ready")
        else:
            self._set_status("Map failed to load")

    def _on_web_title_changed(self, title: str) -> None:
        if title.startswith("VGCS_PLAN_TOOL_REQUEST:"):
            parts = title.split(":")
            tool = parts[1] if len(parts) >= 2 else ""
            self.plan_tool_requested.emit(tool)
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

    def _set_status(self, text: str) -> None:
        self._status.setText(f"Map status: {text}")

    def set_vehicle_position(self, lat: float, lon: float, *, relative_alt_m: float | None = None) -> None:
        self._lat = lat
        self._lon = lon
        if relative_alt_m is None:
            self._coords.setText(f"Lat/Lon: {lat:.7f}, {lon:.7f}")
        else:
            self._coords.setText(
                f"Lat/Lon: {lat:.7f}, {lon:.7f}  |  Rel Alt: {relative_alt_m:.1f} m"
            )
        self._run_js(f"setVehicle({lat:.8f}, {lon:.8f});")
        self._set_status("vehicle marker updated")

    def set_vehicle_heading(self, heading_deg: float) -> None:
        self._heading = heading_deg % 360.0
        self._heading_label.setText(f"Heading: {self._heading:.1f}°")
        self._run_js(f"updateHeading({self._heading:.2f});")

    def set_flight_telemetry(
        self,
        *,
        relative_alt_m: float,
        ground_speed_mps: float,
        flight_time_text: str,
        msl_alt_m: float,
    ) -> None:
        self._run_js(
            "setTelemetryOverlay("
            f"{float(relative_alt_m):.3f}, "
            f"{float(ground_speed_mps):.3f}, "
            f"{json.dumps(str(flight_time_text))}, "
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
            callback=lambda _: self._after_waypoints_mutated(),
        )
        self._set_status("waypoints cleared")

    def _sync_waypoint_count_from_map(self) -> None:
        self._run_js("JSON.stringify(getWaypoints());", callback=self._on_waypoints_json)

    def _on_waypoints_json(self, payload: str | None) -> None:
        if not payload:
            self.set_mission_waypoint_count(0)
            self._waypoints_model = []
            self._rebuild_wp_selector()
            self.waypoints_changed.emit([])
            return
        try:
            rows = json.loads(payload)
        except Exception:
            return
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
            waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
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

    def _emit_upload_from_json(self, payload: str | None) -> None:
        if not payload:
            self._set_status("No waypoints to upload")
            return
        try:
            rows = json.loads(payload)
            waypoints = []
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
                waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
        except Exception:
            self._set_status("Mission parse error")
            return
        if not waypoints:
            self._set_status("No waypoints to upload")
            return
        self.mission_upload_requested.emit(waypoints)
        self._set_status(f"Mission upload requested ({len(waypoints)} WPs)")

    def _request_download(self) -> None:
        self.mission_download_requested.emit()
        self._set_status("Mission download requested")

    def _export_mission(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export mission waypoints",
            "mission-waypoints.json",
            "JSON files (*.json)",
        )
        if not path:
            return

        def cb(payload: str | None) -> None:
            if not payload:
                self._set_status("No waypoints to export")
                return
            try:
                rows = json.loads(payload)
                waypoints = []
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
                    waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=alt))
                save_waypoints_json(path, waypoints)
                self._set_status(f"Mission exported ({len(waypoints)} WPs)")
            except Exception:
                self._set_status("Export failed")

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
        self._set_status(f"Mission imported ({len(waypoints)} WPs)")

    def set_waypoints(self, waypoints: list[Waypoint]) -> None:
        rows = [[wp.lat, wp.lon] for wp in waypoints]
        self._waypoints_model = list(waypoints)
        self._run_js(
            f"setWaypoints({json.dumps(rows)});",
            callback=lambda _: self._after_waypoints_mutated(),
        )
        self._set_status(f"Mission loaded ({len(waypoints)} WPs)")

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

    def _on_wp_selected(self, index: int) -> None:
        if 0 <= index < len(self._waypoints_model):
            self._wp_alt.setValue(float(self._waypoints_model[index].alt_m))

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

    def _toggle_3d_mode(self, enabled: bool) -> None:
        active = self.set_3d_enabled(enabled)
        if active != enabled:
            self._btn_3d.blockSignals(True)
            self._btn_3d.setChecked(active)
            self._btn_3d.blockSignals(False)

    def _enable_fence_polygon_mode(self) -> None:
        self._run_js("enableFencePolygon();")
        self._set_status("Fence polygon mode: click map to add points")

    def _set_online_tiles(self) -> None:
        self.activate_online_tiles()

    def _pick_offline_tiles(self) -> None:
        root = QFileDialog.getExistingDirectory(
            self,
            "Select offline tile root (contains z/x/y.png)",
            "",
        )
        if not root:
            return
        self.activate_offline_tiles(root)

    def activate_online_tiles(self) -> None:
        self._run_js(
            "setTileSource('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', "
            "'&copy; OpenStreetMap contributors', 19);"
        )
        self._set_status("Online tiles active")

    def activate_offline_tiles(self, root: str) -> None:
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

