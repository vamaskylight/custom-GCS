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
from vgcs.observe.geo_reference import row_point_is_measured
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


# "Laser HIT": how far from the cross a click may be for the laser to stand
# for it. The laser measures at the cross and nowhere else. This is the
# C13's own limit for a lock with the camera held still
# (adapter._LRF_HOLD_MAX_CLICK_OFFSET_DEG). The Viewpro's is wider.
LASER_HIT_MAX_OFF_CROSS_DEG = 4.0


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
        # A second fall of shot used to be refused outright: "Only one Impact
        # Target is allowed per session. Press Reset to clear the previous mark
        # and try again." That made the multi-round averaging built on
        # 2026-09-01 unreachable, for a question the client had asked directly
        # ("if we did the 2 DOOAF test then how can we calculate the average"),
        # and it made adjusting round after round in one shoot impossible.
        #
        # Rounds accumulate now. The correction still comes off the newest
        # round, which is how fire is adjusted; the earlier ones give the bias
        # and dispersion across the shoot. The round number goes in the status
        # line so a stray second click reads as a new round rather than
        # silently rewriting the correction.
        # The round number is appended in _log_observation_after_geo, once this
        # row is actually in the session, because that is where the status line
        # is assembled.
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
            and self._impact_uses_lrf()
        ):
            # "Laser HIT" is on: the fall of shot is what the laser measures
            # under the cross. One way, whatever was set up before.
            #
            # Until 2026-10-08 there were three, chosen by how the gun and the
            # target had been set: the click put on the wall plane of an
            # earlier lock, the picture alone, or a lock with the camera turned
            # to the click. The first is right only on one wall face seen from
            # an unmoved camera. Over open ground it puts a round that fell
            # 100 m short at the target's own lat long, 10 m lower.
            row["laser_asked"] = True
            why_not = self._why_the_laser_cannot_take_this_click(
                float(video_x), float(video_y), row
            )
            if why_not:
                # No laser on this camera, or a click away from the cross:
                # the click goes on from the picture, and the operator is told.
                row["laser_not_used_why"] = why_not
            else:
                if self._lrf_lock_in_progress or self._pending_lrf_video_pick is not None:
                    self._set_status("LRF lock in progress — wait before marking impact…")
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
                # The camera is held still and the laser reads along the
                # cross: the same lock as "LRF lock (facade)" in DOOAF Setup,
                # which is the one the crew use for the target.
                self._begin_c13_lrf_video_lock_for_pick(
                    float(video_x),
                    float(video_y),
                    label="Impact Target",
                    hold_gimbal=True,
                    hold_slant_boresight=True,
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
            # The point of this row is not the laser's. The laser's range does
            # not stay on it as if it were, and the operator is told why the
            # laser did not give the point (dooaf_popup.laser_note).
            row["lrf_slant_range_m"] = None
            row["laser_not_used_why"] = self._why_the_laser_gave_no_point(slant_m)
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
        else:
            # The laser measured what is under the cross. That is where the
            # mark belongs. The click, at most a few degrees from it, is kept.
            row["laser_not_used_why"] = ""
            row["click_x_norm"] = float(video_x)
            row["click_y_norm"] = float(video_y)
            video_x, video_y = 0.5, 0.5
            row["video_x_norm"] = video_x
            row["video_y_norm"] = video_y
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
        if observation_target_latlon(row) is not None and row_point_is_measured(row):
            # The click was placed, with the laser or without it. No red
            # "LRF failed" mark is put beside a position (lrf_mixin).
            self._lrf_pick_placed_without_laser = not used_lrf

    def _why_the_laser_cannot_take_this_click(
        self, video_x: float, video_y: float, row: dict[str, object]
    ) -> str:
        """Why no laser lock is started for a click as Set HIT, or "" when one is."""
        from vgcs.map.dooaf_popup import (
            LASER_NOT_ON_THIS_CAMERA,
            laser_not_at_the_cross_text,
        )

        if not self._dooaf_lrf_geo_enabled():
            return LASER_NOT_ON_THIS_CAMERA
        try:
            away_deg, _down_deg = self._facade_click_offset_deg(video_x, video_y)
        except Exception:
            return ""
        if float(away_deg) > LASER_HIT_MAX_OFF_CROSS_DEG:
            row["laser_click_off_cross_deg"] = float(away_deg)
            return laser_not_at_the_cross_text(float(away_deg))
        return ""

    def _why_the_laser_gave_no_point(self, slant_m: float | None) -> str:
        """The words for a lock that did not give the fall of shot."""
        from vgcs.map.dooaf_popup import (
            LASER_GAVE_NO_RANGE,
            LASER_POINT_NOT_WORKED_OUT,
        )

        if slant_m is not None:
            return LASER_POINT_NOT_WORKED_OUT
        in_the_air = getattr(self, "_lrf_range_in_the_air", None)
        short_text = getattr(in_the_air, "short_text", None)
        if callable(short_text):
            # A range that is no point on the ground (lrf_mixin): "Laser 18.5 m
            # is in the air (drone 83 m up)".
            return f"{short_text()}."
        return LASER_GAVE_NO_RANGE

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
        not_measured_note = ""
        if kind == "video_mark" and dooaf_role in (DOOAF_ROLE_IMPACT, DOOAF_ROLE_INTENDED):
            if observation_target_latlon(row) is None:
                self._dooaf_click_not_placed(row, dooaf_role)
                return
            if not row_point_is_measured(row):
                # The point rests on a guess: no target and no fall of shot.
                # The click goes on from here as a measuring mark.
                not_measured_note = self._dooaf_click_not_measured(row, dooaf_role)
                dooaf_role = DOOAF_ROLE_SURVEY
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
                if rs.assumed_gun_bearing_deg is not None:
                    # Gun deliberately not surveyed — only the target is outstanding.
                    if rs.target_lat is None:
                        msg += " — set the target in DOOAF Setup for correction"
                elif rs.gun_lat is None or rs.target_lat is None:
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
        if dooaf_role == DOOAF_ROLE_IMPACT and kind in ("video_mark", "map_mark"):
            # Only from the second onwards: saying "round 1" on a single-round
            # shoot would imply more are expected.
            rounds = self._impact_round_count()
            if rounds > 1:
                msg += f" — round {rounds}"
        self._show_dooaf_mark_popup(dooaf_role, kind)
        self._set_status(f"{not_measured_note} | {msg}" if not_measured_note else msg)
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

    def _dooaf_click_not_placed(self, row: dict[str, object], dooaf_role: str) -> None:
        """A click on the video that got no position: say so, and keep no mark.

        It used to be kept all the same: a mark on the video, a line in the
        report, one more round in the count, and a popup that still showed the
        round before. Nothing was measured, so nothing is recorded, and the
        operator is told why and what to do (client photo, 2026-10-08).
        """
        from vgcs.map.dooaf_popup import (
            IMPACT_HEADING,
            NOT_MARKED,
            TARGET_HEADING,
            not_marked_text,
            why_not_marked,
        )

        heading = TARGET_HEADING if dooaf_role == DOOAF_ROLE_INTENDED else IMPACT_HEADING
        print(
            f"[VGCS:observe] {heading} {NOT_MARKED} "
            f"video=({row.get('video_x_norm')},{row.get('video_y_norm')}) "
            f"reason={str(row.get('geo_warning') or row.get('geo_quality') or '')!r} "
            f"look={row.get('geo_depression_deg')} agl={row.get('measure_agl_m')} "
            f"gimbal=({row.get('gimbal_yaw_deg')},{row.get('gimbal_pitch_deg')})"
        )
        kept = ""
        try:
            if dooaf_role == DOOAF_ROLE_IMPACT:
                rounds = self._impact_round_count()
                if rounds == 1:
                    kept = "The round marked before stays as it was."
                elif rounds > 1:
                    kept = f"The {rounds} rounds marked before stay as they were."
            elif self._resolved_dooaf_settings().target_lat is not None:
                kept = "The target set before stays as it was."
        except Exception:
            kept = ""
        self._show_dooaf_not_marked(
            not_marked_text(heading, row, kept, self._laser_advice_for(row, dooaf_role))
        )
        why = why_not_marked(row).replace("\n", " ")
        self._set_status(f"{heading} {NOT_MARKED}: {why}")

    def _laser_advice_for(self, row: dict[str, object], dooaf_role: str) -> str:
        """The way that is left for a fall of shot the picture could not place.

        Said only where it is true and new: the click was a fall of shot, the
        switch "Laser HIT" was off, and this camera has a laser.
        """
        from vgcs.map.dooaf_popup import LASER_ADVICE

        try:
            if dooaf_role != DOOAF_ROLE_IMPACT or row.get("laser_asked"):
                return ""
            if str(row.get("kind") or "video_mark") != "video_mark":
                return ""
            return LASER_ADVICE if self._dooaf_lrf_geo_enabled() else ""
        except Exception:
            return ""

    def _show_dooaf_not_marked(self, text: str) -> None:
        """Shown whether or not the read-out popup is switched on.

        That switch is for the numbers of a mark, and this is the news that
        there is no mark: the status line, the only other place, is hidden in
        flight.
        """
        try:
            from vgcs.map.dooaf_popup import DooafPopup

            popup = getattr(self, "_dooaf_popup", None)
            if popup is None:
                popup = DooafPopup(self)
                self._dooaf_popup = popup
            popup.show_text(text)
        except Exception:
            pass

    def _dooaf_click_not_measured(self, row: dict[str, object], dooaf_role: str) -> str:
        """A click whose point rests on a guess: no DOOAF point, a measuring mark.

        Below the flying height the ground point of a click is still worked
        out the bench's way when the camera's own angle gives none: a camera
        within 15 degrees of level is taken to look 18 or 35 degrees down, a
        drone on the ground is taken to be half a metre up. The measuring
        marks on the video were tuned with those points. As a target or a
        fall of shot such a point read as a measurement: on the ground it
        lies about one drone height in front of the drone whatever was
        clicked, and the correction was worked out from there (the client's
        "static mode", 2026-10-08).

        So the click loses its DOOAF role and stays as a measuring mark. The
        row keeps what it was clicked as, and the operator is told. Returns
        the words for the status line.
        """
        from vgcs.map.dooaf_popup import (
            IMPACT_HEADING,
            MEASURING_MARK_ONLY,
            NOT_MARKED,
            TARGET_HEADING,
            not_marked_text,
            why_not_marked,
        )

        heading = TARGET_HEADING if dooaf_role == DOOAF_ROLE_INTENDED else IMPACT_HEADING
        row["dooaf_clicked_as"] = dooaf_role
        row["dooaf_role"] = DOOAF_ROLE_SURVEY
        print(
            f"[VGCS:observe] {heading} {NOT_MARKED} (point not measured, kept as a measuring mark) "
            f"video=({row.get('video_x_norm')},{row.get('video_y_norm')}) "
            f"why={str(row.get('geo_not_measured_why') or '')!r} "
            f"look={row.get('geo_not_measured_look_deg')} agl={row.get('measure_agl_m')} "
            f"gimbal=({row.get('gimbal_yaw_deg')},{row.get('gimbal_pitch_deg')}) "
            f"guess=({row.get('target_lat')},{row.get('target_lon')}) by {row.get('geo_method')}"
        )
        self._show_dooaf_not_marked(
            not_marked_text(
                heading, row, MEASURING_MARK_ONLY, self._laser_advice_for(row, dooaf_role)
            )
        )
        why = why_not_marked(row).replace("\n", " ")
        return f"{heading} {NOT_MARKED}: {why} {MEASURING_MARK_ONLY}"

    def dooaf_popup_enabled(self) -> bool:
        """On by default; the crew asked for the read-out on every mark."""
        raw = QSettings(QS_ORG, QS_APP).value("dooaf/mark_popup", True)
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() not in ("0", "false", "no", "off")

    def _show_dooaf_mark_popup(self, dooaf_role: str, kind: str) -> None:
        """Put the marked point, its grid reference and the correction on screen.

        Everything here already went to the map status line, which dashboard
        mode hides, so in practice it went nowhere. Requested 2026-09-09.
        """
        if kind not in ("video_mark", "map_mark"):
            return
        if dooaf_role not in (DOOAF_ROLE_INTENDED, DOOAF_ROLE_IMPACT):
            return
        if not self.dooaf_popup_enabled():
            if dooaf_role == DOOAF_ROLE_IMPACT:
                try:
                    from vgcs.map.dooaf_popup import marked_without_the_laser_text

                    said = marked_without_the_laser_text(
                        latest_mark_row(self._observations, DOOAF_ROLE_IMPACT)
                    )
                    if said:
                        self._show_dooaf_not_marked(said)
                except Exception:
                    pass
            return
        try:
            from vgcs.map.dooaf_popup import (
                DooafPopup,
                dooaf_popup_text,
                gun_note,
                laser_note,
            )

            session = build_dooaf_session(
                self._observations, **self._dooaf_session_kwargs()
            )
            text = dooaf_popup_text(session)
            if not text:
                return
            popup = getattr(self, "_dooaf_popup", None)
            if popup is None:
                popup = DooafPopup(self)
                self._dooaf_popup = popup
            # With "Laser HIT" on: whether the fall of shot shown is the
            # laser's point, with its range, or the picture's, and why.
            notes = [
                gun_note(session),
                laser_note(latest_mark_row(self._observations, DOOAF_ROLE_IMPACT)),
            ]
            popup.show_text(text, "\n".join(n for n in notes if n))
        except Exception:
            # A read-out must never take the mark down with it.
            pass

    def _impact_uses_lrf(self) -> bool:
        """Whether a fall of shot is measured by the laser: "Laser HIT" on the camera rail.

        Off by default. Requested 2026-09-11: "for impact Target we don't want
        to use the LRF". With it off, an impact pick is placed from GPS, gimbal
        angle and terrain, and the camera never slews. The actual target is
        untouched by this and can still be laser-locked, which is what they
        asked for.

        The switch is for the fall of shot that the picture cannot measure: a
        drone on the ground, a look flatter than 8 degrees. Until 2026-10-08
        this was a setting with no place on the screen.
        """
        raw = self._dooaf_settings_store().value("dooaf/impact_uses_lrf", False)
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() in ("1", "true", "yes", "on")

    def _set_impact_uses_lrf(self, on: bool) -> None:
        """The operator pressed "Laser HIT" on the camera rail."""
        try:
            self._dooaf_settings_store().setValue("dooaf/impact_uses_lrf", bool(on))
        except Exception:
            pass
        if not on:
            self._set_status("Laser HIT off: the fall of shot is placed from the picture")
        elif not self._dooaf_lrf_geo_enabled():
            self._set_status(
                "Laser HIT on, but this camera has no laser that VGCS can use: "
                "the fall of shot is placed from the picture"
            )
        else:
            self._set_status(
                "Laser HIT on: put the cross on the fall of shot, then click it"
            )

    def _impact_round_count(self) -> int:
        """How many rounds have been marked in this session so far."""
        n = 0
        for row in getattr(self, "_observations", None) or []:
            if not isinstance(row, dict):
                continue
            if str(row.get("dooaf_role") or "") != DOOAF_ROLE_IMPACT:
                continue
            if str(row.get("kind") or "") not in ("video_mark", "map_mark"):
                continue
            # A click that got no position is no round (a session saved before
            # 2026-10-08 can still hold one).
            if observation_target_latlon(row) is not None:
                n += 1
        return n

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
            assumed_gun_bearing_deg=dooaf.get("assumed_gun_bearing_deg"),  # type: ignore[arg-type]
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
            format_observation_detailed_log_html(
                export_rows,
                self._obs_cell,
                dem_available=bool(getattr(session, "dem_available", False)),
            ),
            session=session,
        )
        Path(path).write_text(html, encoding="utf-8")
