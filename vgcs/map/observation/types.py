"""Observation / LRF async tasks and pick descriptors."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, QRunnable, Signal
from PySide6.QtGui import QImage

from vgcs.map.image_io import save_qimage_to_path
from vgcs.observe.dooaf import (
    DOOAF_ROLE_IMPACT,
    assemble_observation_report_html,
    build_dooaf_session,
    fire_correction_en_miss_m,
    fire_correction_miss_consistency_gap_m,
    fire_correction_miss_is_consistent,
    format_dooaf_html_summary,
    format_gimbal_pitch_direction,
    format_gimbal_yaw_direction,
    format_observation_detailed_log_html,
    latest_mark_row,
    merge_setup_video_marks,
)
from vgcs.observe.grid_reference import format_grid_reference


class ObservationSnapshotBridge(QObject):
    finished = Signal(int, str)  # observation index, snapshot path (may be empty)


@dataclass
class PendingLrfVideoPick:
    """DOOAF / observation video pick waiting on C13 LRF lock."""

    purpose: str  # "dooaf_setup" | "observation"
    u: float
    v: float
    label: str = ""
    pick_role: str = ""
    observation_row: dict[str, object] | None = None
    obs_kind: str = ""
    obs_map_lat: float | None = None
    obs_map_lon: float | None = None
    obs_clip_path: str = ""
    obs_capture_snapshot: bool = True


class LrfLockBridge(QObject):
    finished = Signal(object, float, float)  # distance_m | None, u, v
    progress = Signal(float)  # live SLR sample while locking


class LrfLockTask(QRunnable):
    """GOT + SUM + SLR on worker thread (UDP can block)."""

    def __init__(
        self,
        cc: object,
        u: float,
        v: float,
        bridge: LrfLockBridge,
        *,
        frame_w: int = 1280,
        frame_h: int = 720,
        hold_gimbal: bool | None = None,
        hold_slant_boresight: bool = False,
    ) -> None:
        super().__init__()
        self._cc = cc
        self._u = float(u)
        self._v = float(v)
        self._bridge = bridge
        self._frame_w = int(frame_w)
        self._frame_h = int(frame_h)
        self._hold_gimbal = hold_gimbal
        self._hold_slant_boresight = bool(hold_slant_boresight)

    def run(self) -> None:
        dist = None

        def _on_sample(value_m: float) -> None:
            try:
                self._bridge.progress.emit(float(value_m))
            except Exception:
                pass

        try:
            lock_fn = getattr(self._cc, "lock_lrf_at_video_norm", None)
            if callable(lock_fn):
                dist = lock_fn(
                    self._u,
                    self._v,
                    frame_w=self._frame_w,
                    frame_h=self._frame_h,
                    on_sample=_on_sample,
                    hold_gimbal=self._hold_gimbal,
                    hold_slant_boresight=self._hold_slant_boresight,
                )
        except Exception as exc:
            print(f"[VGCS:lrf] lock failed: {exc}")
        try:
            self._bridge.finished.emit(dist, self._u, self._v)
        except Exception:
            pass


class ObservationSnapshotTask(QRunnable):
    """Save a preview still off the GUI thread (Target / Report must not freeze the app)."""

    def __init__(
        self,
        img: QImage,
        dest: Path,
        idx: int,
        bridge: ObservationSnapshotBridge,
    ) -> None:
        super().__init__()
        self._img = img
        self._dest = dest
        self._idx = int(idx)
        self._bridge = bridge

    def run(self) -> None:
        path = ""
        try:
            if save_qimage_to_path(self._img, self._dest):
                path = str(self._dest)
        except Exception:
            path = ""
        try:
            self._bridge.finished.emit(self._idx, path)
        except Exception:
            pass


class ObservationExportBridge(QObject):
    finished = Signal(bool, str)  # ok, summary message


class ObservationExportTask(QRunnable):
    def __init__(
        self,
        *,
        rows: list[dict[str, object]],
        csv_path: str,
        html_path: str,
        obs_cell_fn,
        bridge: ObservationExportBridge,
        gun_lat: float | None = None,
        gun_lon: float | None = None,
        gun_alt_m: float | None = None,
        target_lat: float | None = None,
        target_lon: float | None = None,
        target_alt_m: float | None = None,
        dem_path: str | None = None,
        setup_video_marks: dict[str, tuple[float, float]] | None = None,
        facade_slant_range_m: float | None = None,
    ) -> None:
        super().__init__()
        self._rows = list(rows)
        self._csv_path = str(csv_path)
        self._html_path = str(html_path)
        self._obs_cell_fn = obs_cell_fn
        self._bridge = bridge
        self._gun_lat = gun_lat
        self._gun_lon = gun_lon
        self._gun_alt_m = gun_alt_m
        self._target_lat = target_lat
        self._target_lon = target_lon
        self._target_alt_m = target_alt_m
        self._dem_path = dem_path
        self._setup_video_marks = setup_video_marks
        self._facade_slant_range_m = facade_slant_range_m

    def run(self) -> None:
        fields = [
            "timestamp_utc",
            "kind",
            "dooaf_role",
            "map_lat",
            "map_lon",
            "map_grid_ref",
            "video_x_norm",
            "video_y_norm",
            "vehicle_lat",
            "vehicle_lon",
            "vehicle_grid_ref",
            "vehicle_heading_deg",
            "vehicle_roll_deg",
            "vehicle_pitch_deg",
            "vehicle_rel_alt_m",
            "vehicle_alt_msl_m",
            "gimbal_yaw_deg",
            "gimbal_pitch_deg",
            "gimbal_yaw_direction",
            "gimbal_pitch_direction",
            "gps_fix_type",
            "gps_satellites",
            "gps_hdop",
            "target_lat",
            "target_lon",
            "target_grid_ref",
            "target_alt_m",
            "geo_quality",
            "geo_warning",
            "geo_method",
            "geo_range_m",
            "geo_bearing_deg",
            "geo_depression_deg",
            "lrf_slant_range_m",
            "segment_distance_m",
            "measure_agl_m",
            "agl_source",
            "geo_agl_source",
            "snapshot_path",
            "clip_path",
            "dooaf_range_correction_m",
            "dooaf_deflection_correction_m",
            "dooaf_miss_m",
            "dooaf_miss_east_m",
            "dooaf_miss_north_m",
            "dooaf_miss_en_m",
            "dooaf_miss_consistency_gap_m",
            "dooaf_miss_vertical_m",
            "dooaf_east_correction_m",
            "dooaf_north_correction_m",
            "dooaf_elevation_correction_m",
            "dooaf_target_dem_alt_m",
            "dooaf_impact_dem_alt_m",
            "dooaf_height_correction_m",
        ]
        session = build_dooaf_session(
            self._rows,
            gun_lat=self._gun_lat,
            gun_lon=self._gun_lon,
            gun_alt_m=self._gun_alt_m,
            target_lat=self._target_lat,
            target_lon=self._target_lon,
            target_alt_m=self._target_alt_m,
            dem_path=self._dem_path,
            setup_video_marks=merge_setup_video_marks(self._setup_video_marks),
            facade_slant_range_m=self._facade_slant_range_m,
        )
        corr = session.correction
        if corr is not None and not fire_correction_miss_is_consistent(corr):
            en = fire_correction_en_miss_m(corr)
            gap = fire_correction_miss_consistency_gap_m(corr)
            print(
                f"[VGCS:observe] report sanity: target→impact "
                f"{corr.impact_to_intended_m:.1f} m vs E/N √(E²+N²) {en:.1f} m "
                f"(gap {gap:.1f} m) — mixed geometry; "
                "check gun/target/impact pick modes",
                flush=True,
            )
        export_rows: list[dict[str, object]] = []
        for row in self._rows:
            out = dict(row)
            yaw = out.get("gimbal_yaw_deg")
            pitch = out.get("gimbal_pitch_deg")
            try:
                out["gimbal_yaw_direction"] = format_gimbal_yaw_direction(
                    float(yaw) if yaw is not None else None
                )
            except (TypeError, ValueError):
                out["gimbal_yaw_direction"] = "N/A"
            try:
                out["gimbal_pitch_direction"] = format_gimbal_pitch_direction(
                    float(pitch) if pitch is not None else None
                )
            except (TypeError, ValueError):
                out["gimbal_pitch_direction"] = "N/A"
            out["map_grid_ref"] = format_grid_reference(
                out.get("map_lat"), out.get("map_lon")
            )
            out["vehicle_grid_ref"] = format_grid_reference(
                out.get("vehicle_lat"), out.get("vehicle_lon")
            )
            out["target_grid_ref"] = format_grid_reference(
                out.get("target_lat"), out.get("target_lon")
            )
            if corr is not None:
                out["dooaf_range_correction_m"] = corr.range_correction_m
                out["dooaf_deflection_correction_m"] = corr.deflection_correction_m
                out["dooaf_miss_m"] = corr.impact_to_intended_m
                out["dooaf_miss_east_m"] = corr.miss_east_m
                out["dooaf_miss_north_m"] = corr.miss_north_m
                out["dooaf_miss_en_m"] = fire_correction_en_miss_m(corr)
                out["dooaf_miss_consistency_gap_m"] = (
                    fire_correction_miss_consistency_gap_m(corr)
                )
                out["dooaf_miss_vertical_m"] = corr.miss_vertical_m
                out["dooaf_east_correction_m"] = -corr.miss_east_m
                out["dooaf_north_correction_m"] = -corr.miss_north_m
                out["dooaf_elevation_correction_m"] = corr.elevation_correction_m
                out["dooaf_target_dem_alt_m"] = session.intended_dem_alt_m
                out["dooaf_impact_dem_alt_m"] = session.impact_dem_alt_m
                out["dooaf_height_correction_m"] = session.height_correction_m
            export_rows.append(out)
        obs_row = latest_mark_row(self._rows, DOOAF_ROLE_IMPACT)
        if obs_row is None and self._rows:
            obs_row = self._rows[-1]
        ok = False
        summary = ""
        try:
            with open(self._csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                for row in export_rows:
                    w.writerow({k: row.get(k) for k in fields})
            detailed_log = format_observation_detailed_log_html(
                export_rows, self._obs_cell_fn
            )
            dooaf_summary = format_dooaf_html_summary(
                session,
                observation_row=obs_row,
                observation_rows=list(self._rows),
            )
            html = assemble_observation_report_html(
                len(self._rows),
                dooaf_summary,
                detailed_log,
                session=session,
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


__all__ = [
    "LrfLockBridge",
    "LrfLockTask",
    "ObservationExportBridge",
    "ObservationExportTask",
    "ObservationSnapshotBridge",
    "ObservationSnapshotTask",
    "PendingLrfVideoPick",
]
