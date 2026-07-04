"""MapWidget mixin — see vgcs.map.observation package."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

from PySide6.QtCore import QSettings, QThreadPool, QTimer, Qt
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QFileDialog, QMessageBox

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.image_io import save_qimage_to_path
from vgcs.map.observation.types import (
    ObservationExportBridge,
    ObservationExportTask,
    ObservationSnapshotBridge,
    ObservationSnapshotTask,
    PendingLrfVideoPick,
)
from vgcs.observe.dooaf import (
    DOOAF_ROLE_IMPACT,
    DOOAF_ROLE_INTENDED,
    DOOAF_ROLE_SURVEY,
    apply_dooaf_impact_geo_fallback,
    apply_facade_slant_to_mark_row,
    assemble_observation_report_html,
    build_dooaf_session,
    dooaf_export_blockers,
    format_dooaf_html_summary,
    format_dooaf_status,
    format_observation_detailed_log_html,
    latest_mark,
    latest_mark_row,
)
from vgcs.observe.grid_reference import format_grid_reference
from vgcs.observe.target_measure import (
    MARKS_NOT_LEVEL_HINT,
    band_width_partner_row,
    clear_tape_pair_override,
    format_target_segment_label,
    haversine_m,
    marks_need_level_warning,
    marks_same_height_band,
    measure_agl_ok,
    observation_building_height_segments,
    observation_facade_video_segments,
    observation_target_latlon,
    segment_distance_between_rows,
    segment_distance_video_fallback,
    session_facade_reference_range_m,
    session_peak_geo_range_m,
    session_rangefinder_reference_m,
    target_track_from_observations,
    video_mark_span_norm,
)


def _open_path_in_system_viewer(path: str) -> None:
    """Open a file in the default OS viewer without routing through Qt URL handlers."""
    target = Path(path).resolve()
    if not target.is_file():
        return
    p = str(target)
    if sys.platform == "win32":
        os.startfile(p)  # noqa: S606 — intentional Windows shell open
        return
    if sys.platform == "darwin":
        subprocess.Popen(["open", p], close_fds=True, start_new_session=True)
        return
    subprocess.Popen(["xdg-open", p], close_fds=True, start_new_session=True)


def _refocus_vgcs_window(host: object) -> None:
    """Bring the main VGCS window back after opening an external report viewer."""
    try:
        win = host.window() if hasattr(host, "window") else host
        if win is None:
            return
        if hasattr(win, "isMinimized") and win.isMinimized():
            win.showNormal()
        if hasattr(win, "show"):
            win.show()
        if hasattr(win, "raise_"):
            win.raise_()
        if hasattr(win, "activateWindow"):
            win.activateWindow()
    except Exception:
        pass


class ObservationSessionMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _warn_gps_unavailable_for_pick(self) -> bool:
        """Return True when pick should be blocked (GPS popup shown)."""
        if self._gps_available_for_geo_pick():
            return True
        QMessageBox.warning(
            self,
            "GPS unavailable",
            "GPS is not available (need 3D fix and vehicle position).\n\n"
            "You cannot pick coordinates on the map or video until GPS is ready.",
        )
        return False

    def _rebuild_observation_map_markers(self) -> None:
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "clear_observation_marks"):
                nm.clear_observation_marks()
            if nm is not None:
                for row in self._observations:
                    kind = str(row.get("kind") or "")
                    if kind == "map_mark":
                        la = row.get("map_lat")
                        lo = row.get("map_lon")
                        if la is None or lo is None:
                            continue
                        if hasattr(nm, "add_observation_map_marker"):
                            nm.add_observation_map_marker(float(la), float(lo))
                    elif kind == "video_mark":
                        la = row.get("target_lat")
                        lo = row.get("target_lon")
                        if la is None or lo is None:
                            continue
                        if hasattr(nm, "add_geo_referenced_marker"):
                            nm.add_geo_referenced_marker(float(la), float(lo))
        except Exception:
            pass
        self._refresh_observation_measure_overlays()
        self._refresh_dooaf_map_overlay()

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
        dooaf_role = self._current_observe_dooaf_role()
        if kind in ("video_mark", "map_mark") and not self._warn_gps_unavailable_for_pick():
            return
        if (
            dooaf_role == DOOAF_ROLE_IMPACT
            and kind in ("video_mark", "map_mark")
            and latest_mark(self._observations, DOOAF_ROLE_IMPACT) is not None
        ):
            QMessageBox.warning(
                self,
                "DOOAF",
                "Only one Impact Target is allowed per session.\n\n"
                "Press Reset to clear the previous mark and try again.",
            )
            return
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
        row["dooaf_role"] = dooaf_role
        if (
            kind == "video_mark"
            and dooaf_role == DOOAF_ROLE_IMPACT
            and video_x is not None
            and video_y is not None
            and self._dooaf_lrf_geo_enabled()
        ):
            if self._lrf_lock_in_progress or self._pending_lrf_video_pick is not None:
                self._set_status("LRF lock in progress — wait before marking impact…")
                return
            if self._dooaf_facade_session_has_lock():
                row.update(self._observation_context())
                geo = self._geo_from_facade_uv_pick(float(video_x), float(video_y))
                if geo is not None:
                    lat, lon, alt_m = geo
                    row["target_lat"] = float(lat)
                    row["target_lon"] = float(lon)
                    if alt_m is not None:
                        row["target_alt_m"] = float(alt_m)
                    row["geo_quality"] = "fair"
                    row["geo_method"] = "lrf_facade_plane"
                    geo_warn = self._facade_geo_warning_from_uv(
                        float(video_x), float(video_y)
                    )
                    stale = not self._dooaf_facade_uv_pick_ready()
                    parts: list[str] = []
                    if geo_warn:
                        parts.append(geo_warn)
                    if stale:
                        parts.append(
                            "facade lock stale (gimbal/drone moved) — geo from lock pose"
                        )
                    row["geo_warning"] = "; ".join(parts)
                    slant = self._dooaf_facade_session.slant_range_m
                    if slant is not None:
                        row["lrf_slant_range_m"] = float(slant)
                    lock_att = self._facade_lock_gimbal_att()
                    row["video_mark_frozen_u"] = float(video_x)
                    row["video_mark_frozen_v"] = float(video_y)
                    self._apply_video_mark_gimbal_track_to_row(
                        row,
                        float(video_x),
                        float(video_y),
                        ref_att=lock_att,
                        lock_att=lock_att,
                        used_lrf_slew=False,
                    )
                    glat = row.get("video_mark_geo_lat")
                    if glat is None:
                        row["video_mark_geo_lat"] = float(lat)
                        row["video_mark_geo_lon"] = float(lon)
                        if alt_m is not None:
                            row["video_mark_geo_alt_m"] = float(alt_m)
                    self._log_observation_after_geo(
                        row,
                        kind=kind,
                        map_lat=map_lat,
                        map_lon=map_lon,
                        video_x=float(video_x),
                        video_y=float(video_y),
                        clip_path=clip_path,
                        capture_snapshot=capture_snapshot,
                    )
                    print(
                        f"[VGCS:observe] impact facade uv pick ok "
                        f"lat={lat:.7f} lon={lon:.7f} "
                        f"video=({float(video_x):.3f},{float(video_y):.3f})"
                    )
                    return
                print(
                    "[VGCS:observe] impact blocked — facade lock active but geo "
                    "missing; re-lock TARGET LRF on the building face"
                )
                self._set_status(
                    "Impact needs facade geo — re-lock TARGET on the building, "
                    "then click impact (no gimbal slew)"
                )
                return
            if self._dooaf_setup_is_ground_workflow():
                row.update(self._observation_context())
                if hasattr(self, "_warn_ground_pick_facade_risk"):
                    self._warn_ground_pick_facade_risk(
                        float(video_y),
                        pick_role=DOOAF_ROLE_IMPACT,
                        label="Impact Target",
                    )
                self._enrich_observation_geo_reference(row)
                if str(row.get("geo_method") or "") in ("", "insufficient"):
                    row["geo_method"] = "dooaf_ground_video"
                att = self._read_gimbal_attitude_pair()
                self._apply_video_mark_gimbal_track_to_row(
                    row,
                    float(video_x),
                    float(video_y),
                    ref_att=att,
                    lock_att=att,
                    used_lrf_slew=False,
                )
                self._log_observation_after_geo(
                    row,
                    kind=kind,
                    map_lat=map_lat,
                    map_lon=map_lon,
                    video_x=float(video_x),
                    video_y=float(video_y),
                    clip_path=clip_path,
                    capture_snapshot=capture_snapshot,
                )
                print(
                    f"[VGCS:observe] impact ground video pick ok "
                    f"lat={row.get('target_lat')} lon={row.get('target_lon')} "
                    f"video=({float(video_x):.3f},{float(video_y):.3f})"
                )
                return
            self._pending_lrf_video_pick = PendingLrfVideoPick(
                purpose="observation",
                u=float(video_x),
                v=float(video_y),
                label="Impact Target",
                observation_row=row,
                obs_kind=str(kind),
                obs_map_lat=map_lat,
                obs_map_lon=map_lon,
                obs_clip_path=str(clip_path or "").strip(),
                obs_capture_snapshot=bool(capture_snapshot),
            )
            self._begin_c13_lrf_video_lock_for_pick(
                float(video_x), float(video_y), label="Impact Target"
            )
            return
        self._enrich_observation_geo_reference(row)
        if kind == "video_mark" and video_x is not None and video_y is not None:
            att = self._read_gimbal_attitude_pair()
            self._apply_video_mark_gimbal_track_to_row(
                row,
                float(video_x),
                float(video_y),
                ref_att=att,
                lock_att=att,
                used_lrf_slew=False,
            )
        self._log_observation_after_geo(
            row,
            kind=kind,
            map_lat=map_lat,
            map_lon=map_lon,
            video_x=video_x,
            video_y=video_y,
            clip_path=clip_path,
            capture_snapshot=capture_snapshot,
        )

    def _complete_pending_observation_lrf_pick(
        self,
        slant_m: float | None,
        pending: PendingLrfVideoPick,
    ) -> None:
        row = pending.observation_row
        if row is None:
            return
        video_x, video_y = float(pending.u), float(pending.v)
        row.update(self._observation_context())
        used_lrf = False
        if slant_m is not None:
            row["lrf_slant_range_m"] = float(slant_m)
            used_lrf = self._apply_lrf_slant_geo_to_row(
                row,
                float(slant_m),
                video_x,
                video_y,
                boresight_after_slew=True,
            )
        if not used_lrf:
            self._enrich_observation_geo_reference(row)
            if slant_m is None:
                self._append_lrf_fallback_warning(
                    row,
                    "LRF lock failed — impact from DEM ray estimate",
                )
            else:
                self._append_lrf_fallback_warning(
                    row,
                    "LRF geo failed — impact from DEM ray estimate",
                )
        row["video_mark_frozen_u"] = float(video_x)
        row["video_mark_frozen_v"] = float(video_y)
        click_att = getattr(self, "_lrf_click_att", None)
        self._apply_video_mark_gimbal_track_to_row(
            row,
            video_x,
            video_y,
            ref_att=click_att,
            lock_att=self._read_gimbal_attitude_pair(),
            used_lrf_slew=False,
        )
        self._log_observation_after_geo(
            row,
            kind=str(pending.obs_kind or "video_mark"),
            map_lat=pending.obs_map_lat,
            map_lon=pending.obs_map_lon,
            video_x=video_x,
            video_y=video_y,
            clip_path=pending.obs_clip_path or None,
            capture_snapshot=bool(pending.obs_capture_snapshot),
        )

    def _log_observation_after_geo(
        self,
        row: dict[str, object],
        *,
        kind: str,
        map_lat: float | None = None,
        map_lon: float | None = None,
        video_x: float | None = None,
        video_y: float | None = None,
        clip_path: str | None = None,
        capture_snapshot: bool = True,
    ) -> None:
        """Append observation row after geo (DEM ray or LRF) is on ``row``."""
        dooaf_role = str(row.get("dooaf_role") or "")
        track_before = target_track_from_observations(self._observations)
        seg_m = None
        pt = observation_target_latlon(row)
        cross_band = False
        partner: dict[str, object] | None = None
        if dooaf_role == DOOAF_ROLE_IMPACT and pt is not None:
            intended = latest_mark(self._observations, DOOAF_ROLE_INTENDED)
            if intended is not None:
                seg_m = haversine_m(
                    intended.lat, intended.lon, pt[0], pt[1]
                )
            else:
                rs = self._resolved_dooaf_settings()
                if rs.target_lat is not None and rs.target_lon is not None:
                    seg_m = haversine_m(
                        float(rs.target_lat),
                        float(rs.target_lon),
                        pt[0],
                        pt[1],
                    )
            row["segment_distance_m"] = seg_m
        elif dooaf_role == DOOAF_ROLE_SURVEY and pt is not None and track_before:
            hfov, _, _ = self._m8_geo_settings()
            peak = session_peak_geo_range_m(self._observations)
            facade_ref = session_facade_reference_range_m(
                self._observations, hfov_deg=hfov
            )
            partner = band_width_partner_row(self._observations, row)
            if partner is not None:
                seg_m = segment_distance_between_rows(
                    partner,
                    row,
                    hfov_deg=hfov,
                    session_peak_range_m=peak,
                    facade_reference_range_m=facade_ref,
                )
                if seg_m is None:
                    rf = session_rangefinder_reference_m(
                        self._observations + [row]
                    )
                    seg_m = segment_distance_video_fallback(
                        partner, row, hfov_deg=hfov, range_m=rf
                    )
            else:
                prev_row = self._observations[-1]
                if marks_same_height_band(prev_row, row):
                    seg_m = segment_distance_between_rows(
                        prev_row,
                        row,
                        hfov_deg=hfov,
                        session_peak_range_m=peak,
                        facade_reference_range_m=facade_ref,
                    )
                    if seg_m is None:
                        seg_m = haversine_m(
                            track_before[-1][0],
                            track_before[-1][1],
                            pt[0],
                            pt[1],
                        )
                else:
                    cross_band = True
                    seg_m = None
            row["segment_distance_m"] = seg_m
        else:
            row["segment_distance_m"] = None
        self._observations.append(row)
        idx = len(self._observations) - 1
        if kind == "video_mark" and video_x is not None and video_y is not None:
            try:
                self._video_obs_marks.append((float(video_x), float(video_y)))
            except Exception:
                pass
        if capture_snapshot:
            self._schedule_observation_snapshot(idx)
        try:
            print(
                f"[VGCS:observe] logged {kind} count={len(self._observations)} "
                f"video=({video_x},{video_y}) map=({map_lat},{map_lon}) "
                f"geo=({row.get('target_lat')},{row.get('target_lon')}) q={row.get('geo_quality')} "
                f"ekf={row.get('ekf_rel_alt_m')} rf={row.get('rangefinder_down_m')} "
                f"agl={row.get('measure_agl_m')}({row.get('agl_source')})"
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
        elif dooaf_role != DOOAF_ROLE_SURVEY:
            session = build_dooaf_session(
                self._observations, **self._dooaf_session_kwargs()
            )
            msg += f" — {format_dooaf_status(session)}"
            if dooaf_role == DOOAF_ROLE_IMPACT:
                rs = self._resolved_dooaf_settings()
                if rs.gun_lat is None or rs.target_lat is None:
                    msg += " — complete DOOAF Setup (gun + target) for correction"
                elif seg_m is not None:
                    msg += f" (miss {float(seg_m):.0f} m)"
        elif kind in ("video_mark", "map_mark"):
            gq = str(row.get("geo_quality") or "")
            if gq in ("good", "fair", "map_direct"):
                rng = row.get("geo_range_m")
                if rng is not None:
                    msg += f" — drone→target {float(rng):.0f} m"
                if row.get("target_lat") is not None:
                    msg += f" @ {float(row['target_lat']):.6f},{float(row['target_lon']):.6f}"
                agl_ok, agl_msg = measure_agl_ok(self._observations + [row])
                if not agl_ok and kind == "video_mark":
                    msg += f" — {agl_msg}"
                elif seg_m is not None:
                    est = (
                        " (RF est)"
                        if str(row.get("geo_quality") or "") == "insufficient"
                        else ""
                    )
                    msg += f" — targets {float(seg_m):.1f} m apart{est}"
                    warn_row = partner if partner is not None else (
                        self._observations[-1] if self._observations else None
                    )
                    if warn_row is not None and marks_need_level_warning(warn_row, row):
                        msg += f" — {MARKS_NOT_LEVEL_HINT}"
                elif cross_band:
                    msg += f" — {MARKS_NOT_LEVEL_HINT}"
            elif kind == "video_mark":
                warn = str(row.get("geo_warning") or "geo insufficient")
                msg += f" — {warn}"
                if dooaf_role == DOOAF_ROLE_IMPACT and row.get("target_lat") is None:
                    msg += " — no HIT on map (click ground in lower video, not sky/horizon)"
        self._set_status(msg)
        self._refresh_observation_measure_overlays()
        self._refresh_dooaf_map_overlay()
        if dooaf_role == DOOAF_ROLE_IMPACT:
            self._ensure_dooaf_impact_visible_on_map(row)

        # Native OBSERVE -> Target needs a visible marker on the Qt map.
        if kind == "map_mark" and map_lat is not None and map_lon is not None:
            try:
                nm = getattr(self, "_native_map", None)
                if nm is not None and hasattr(nm, "add_observation_map_marker"):
                    nm.add_observation_map_marker(float(map_lat), float(map_lon))
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
        try:
            self._flush_video_marks_overlay()
        except Exception:
            pass

    def _flush_video_marks_overlay(self) -> None:
        try:
            if self._lrf_reticle_tracking_active():
                self._update_lrf_reticle_track()
            marks = self._video_overlay_marks()
            self._video_obs_marks = [(m.x, m.y) for m in marks]
            ly = self._native_video_overlay
            ly.set_video_marks(marks)
            ly.set_offscreen_hints(self._video_overlay_offscreen_hints())
            ly.set_target_measure_segments(self._observation_video_measure_segments())
            self._refresh_dooaf_facade_overlay_hint()
            if bool(getattr(self, "_video_preview_enabled", False)):
                ly.show()
                ly.raise_()
            self._sync_native_video_overlay()
        except Exception:
            pass
        self._sync_video_mark_track_timer()

    def _observation_measure_labels_and_segments(
        self,
    ) -> tuple[list[str], list[tuple[float, float, float, float, str]]]:
        """Map labels (one per track edge) and video measure lines."""
        labels: list[str] = []
        segs: list[tuple[float, float, float, float, str]] = []
        hfov, _, _ = self._m8_geo_settings()
        peak = session_peak_geo_range_m(self._observations)
        facade_ref = session_facade_reference_range_m(
            self._observations, hfov_deg=hfov
        )
        prev_row: dict[str, object] | None = None
        prev_xy: tuple[float, float] | None = None
        for row in self._observations:
            if observation_target_latlon(row) is None:
                continue
            vx = row.get("video_x_norm")
            vy = row.get("video_y_norm")
            if prev_row is not None:
                if marks_same_height_band(prev_row, row):
                    d = segment_distance_between_rows(
                        prev_row,
                        row,
                        hfov_deg=hfov,
                        session_peak_range_m=peak,
                        facade_reference_range_m=facade_ref,
                    )
                    pix = None
                    if (
                        prev_xy is not None
                        and vx is not None
                        and vy is not None
                    ):
                        pix = video_mark_span_norm(
                            prev_xy[0], prev_xy[1], float(vx), float(vy)
                        )
                    label = (
                        format_target_segment_label(d, video_span_norm=pix)
                        if d is not None
                        else ""
                    )
                    labels.append(label)
                    if label and prev_xy is not None and vx is not None and vy is not None:
                        segs.append(
                            (prev_xy[0], prev_xy[1], float(vx), float(vy), label)
                        )
                else:
                    labels.append("")
            prev_row = row
            if vx is not None and vy is not None:
                prev_xy = (float(vx), float(vy))
        return labels, segs

    def _observation_video_measure_segments(self) -> list[tuple[float, float, float, float, str]]:
        """Dashed lines: building height, facade width, intended→impact."""
        dooaf_seg = dooaf_intended_impact_video_segment(self._observations)
        hfov, _, _ = self._m8_geo_settings()
        segs = list(
            observation_facade_video_segments(self._observations, hfov_deg=hfov)
        )
        segs.extend(
            observation_building_height_segments(self._observations, hfov_deg=hfov)
        )
        if dooaf_seg is not None:
            segs.append(dooaf_seg)
        return segs

    def _refresh_observation_measure_overlays(self) -> None:
        """Sync map measure lines + video segment labels from logged observations."""
        labels, _ = self._observation_measure_labels_and_segments()
        track = target_track_from_observations(self._observations)
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "set_observation_target_track"):
                nm.set_observation_target_track(track, segment_labels=labels)
        except Exception:
            pass
        self._schedule_video_marks_overlay_refresh()
        self._sync_3d_map_overlays()

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
        pool.start(ObservationSnapshotTask(img, dest, idx, self._obs_snapshot_bridge))

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
                self._apply_video_recording_preview_transform(clip_sid)
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
        s = QSettings(QS_ORG, QS_APP)
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
        try:
            clear_tape_pair_override()
        except Exception:
            pass
        # Clear native markers (Qt) + web markers (if any).
        try:
            nm = getattr(self, "_native_map", None)
            if nm is not None and hasattr(nm, "clear_observation_marks"):
                nm.clear_observation_marks()
        except Exception:
            pass
        try:
            self._native_video_overlay.clear_video_marks()
            self._native_video_overlay.clear_detections()
            self._native_video_overlay.set_target_measure_segments([])
            self._refresh_lrf_lock_overlay()
        except Exception:
            pass
        self._run_js("if (window.clearObservationMarks) clearObservationMarks();")
        self._refresh_observation_measure_overlays()
        self._refresh_dooaf_map_overlay()
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
        rows = [dict(r) for r in self._observations]
        dooaf = self._dooaf_session_kwargs()
        facade_slant = dooaf.get("facade_slant_range_m")
        if facade_slant is not None:
            try:
                slant_f = float(facade_slant)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                slant_f = None
            if slant_f is not None:
                for row in rows:
                    apply_facade_slant_to_mark_row(row, slant_f)
        for row in rows:
            if str(row.get("dooaf_role") or "") == DOOAF_ROLE_IMPACT:
                apply_dooaf_impact_geo_fallback(
                    row,
                    target_lat=dooaf.get("target_lat"),  # type: ignore[arg-type]
                    target_lon=dooaf.get("target_lon"),  # type: ignore[arg-type]
                    setup_video_marks=dooaf.get("setup_video_marks"),  # type: ignore[arg-type]
                    dem_path=dooaf.get("dem_path"),  # type: ignore[arg-type]
                    vehicle_alt_msl_m=self._vehicle_alt_msl_m,
                )
        export_warnings = dooaf_export_blockers(
            rows,
            gun_lat=dooaf.get("gun_lat"),  # type: ignore[arg-type]
            gun_lon=dooaf.get("gun_lon"),  # type: ignore[arg-type]
            target_lat=dooaf.get("target_lat"),  # type: ignore[arg-type]
            target_lon=dooaf.get("target_lon"),  # type: ignore[arg-type]
            setup_video_marks=dooaf.get("setup_video_marks"),  # type: ignore[arg-type]
            dem_path=dooaf.get("dem_path"),  # type: ignore[arg-type]
        )
        self._obs_export_warnings = export_warnings
        if export_warnings:
            note = " | ".join(export_warnings)
            self._set_status(f"Exporting with warnings: {note[:120]}")
            print(f"[VGCS:observe] export warnings: {note}")
        self._obs_export_warning_text = (
            "\n\n".join(export_warnings) if export_warnings else ""
        )
        print(f"[VGCS:observe] export started -> {csv_path}")
        self._set_status("Exporting observation report…")
        pool = getattr(self, "_video_pool", None) or QThreadPool.globalInstance()
        pool.start(
            ObservationExportTask(
                rows=rows,
                csv_path=csv_path,
                html_path=html_path,
                obs_cell_fn=self._obs_cell,
                bridge=self._obs_export_bridge,
                gun_lat=dooaf.get("gun_lat"),
                gun_lon=dooaf.get("gun_lon"),
                gun_alt_m=dooaf.get("gun_alt_m"),
                target_lat=dooaf.get("target_lat"),
                target_lon=dooaf.get("target_lon"),
                target_alt_m=dooaf.get("target_alt_m"),
                dem_path=dooaf.get("dem_path"),
                setup_video_marks=dooaf.get("setup_video_marks"),
                facade_slant_range_m=dooaf.get("facade_slant_range_m"),
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
            warn = str(getattr(self, "_obs_export_warning_text", "") or "").strip()
            self._obs_export_warning_text = ""

            def _prompt_open_report() -> None:
                try:
                    lines = [ln.strip() for ln in str(summary).splitlines() if ln.strip()]
                    html_path: Path | None = None
                    folder = ""
                    for ln in lines[1:]:
                        p = Path(ln)
                        if p.suffix.lower() in (".html", ".htm") and p.is_file():
                            html_path = p.resolve()
                            folder = str(p.parent)
                            break
                        folder = str(p.parent if p.suffix else p)
                    body = (
                        f"Report exported.\n\n{summary}\n\n"
                        "VGCS keeps running — use Alt+Tab to return after viewing the report."
                    )
                    if warn:
                        body = f"Report exported.\n\nWarnings:\n{warn}\n\n{summary}\n\nVGCS keeps running."
                    parent = self.window() or self
                    box = QMessageBox(parent)
                    box.setWindowTitle("Observation Report")
                    box.setText(body)
                    box.setIcon(
                        QMessageBox.Icon.Warning if warn else QMessageBox.Icon.Information
                    )
                    open_btn = box.addButton(
                        "Open HTML report",
                        QMessageBox.ButtonRole.AcceptRole,
                    )
                    stay_btn = box.addButton(
                        "Stay in VGCS",
                        QMessageBox.ButtonRole.RejectRole,
                    )
                    folder_btn = None
                    if folder:
                        folder_btn = box.addButton(
                            "Open folder",
                            QMessageBox.ButtonRole.ActionRole,
                        )
                    box.exec()
                    clicked = box.clickedButton()
                    if clicked is open_btn and html_path is not None:
                        from vgcs.video.pipeline import notify_companion_report_viewer_opened

                        notify_companion_report_viewer_opened(duration_s=180.0)
                        _open_path_in_system_viewer(str(html_path))
                        print(
                            "[VGCS:observe] report opened in external browser — "
                            "VGCS still running (Alt+Tab to return)"
                        )
                        self._set_status(
                            "Report opened in browser — VGCS still running (Alt+Tab to return)"
                        )
                        QTimer.singleShot(1200, lambda: _refocus_vgcs_window(self))
                    elif clicked is folder_btn and folder:
                        _open_path_in_system_viewer(folder)
                        QTimer.singleShot(400, lambda: _refocus_vgcs_window(self))
                    elif clicked is stay_btn:
                        QTimer.singleShot(0, lambda: _refocus_vgcs_window(self))
                except Exception as exc:
                    try:
                        print(f"[VGCS:observe] report open failed: {exc}")
                    except Exception:
                        pass

            QTimer.singleShot(400, _prompt_open_report)

    def _write_observation_html_summary(self, path: str) -> None:
        export_rows: list[dict[str, object]] = []
        for row in self._observations:
            out = dict(row)
            out["map_grid_ref"] = format_grid_reference(
                out.get("map_lat"), out.get("map_lon")
            )
            out["vehicle_grid_ref"] = format_grid_reference(
                out.get("vehicle_lat"), out.get("vehicle_lon")
            )
            out["target_grid_ref"] = format_grid_reference(
                out.get("target_lat"), out.get("target_lon")
            )
            export_rows.append(out)
        session = build_dooaf_session(
            list(self._observations),
            **self._dooaf_session_kwargs(),
        )
        obs_row = latest_mark_row(self._observations, DOOAF_ROLE_IMPACT)
        if obs_row is None and self._observations:
            obs_row = self._observations[-1]
        html = assemble_observation_report_html(
            len(self._observations),
            format_dooaf_html_summary(
                session,
                observation_row=obs_row,
                observation_rows=list(self._observations),
            ),
            format_observation_detailed_log_html(export_rows, self._obs_cell),
            session=session,
        )
        Path(path).write_text(html, encoding="utf-8")
