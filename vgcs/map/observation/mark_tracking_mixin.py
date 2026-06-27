"""MapWidget mixin — see vgcs.map.observation package."""

from __future__ import annotations

from PySide6.QtCore import QTimer

from vgcs.map.native_video_overlay import VideoOverlayMark, VideoOverlayOffscreenHint, offscreen_hint_edge_uv
from vgcs.observe.dooaf import dooaf_role_display
from vgcs.observe.dooaf_flight_session import mark_track_use_geo_in_flight
from vgcs.observe.geo_reference import project_wgs84_to_video_norm, should_project_lrf_mark_via_geo
from vgcs.observe.target_measure import haversine_m


class VideoMarkTrackingMixin:
    """Extracted from MapWidget — uses host widget state via self."""

    def _build_video_mark_track(
        self,
        ref_uv: tuple[float, float],
        ref_att: tuple[float, float] | None,
        lock_att: tuple[float, float] | None,
        *,
        used_lrf_slew: bool,
    ) -> dict[str, object] | None:
        if ref_att is None:
            return None
        from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

        track_uv = (float(ref_uv[0]), float(ref_uv[1]))
        track_att = (float(ref_att[0]), float(ref_att[1]))
        h_scale = 1.0
        v_scale = 1.0
        if used_lrf_slew and lock_att is not None:
            h_scale = float(getattr(self, "_lrf_track_gac_h_scale", 1.0) or 1.0)
            v_scale = float(getattr(self, "_lrf_track_gac_v_scale", 1.0) or 1.0)
            if abs(h_scale - 1.0) < 1e-6 and abs(v_scale - 1.0) < 1e-6:
                h_scale, v_scale = SkydroidTopUdpAdapter.calibrate_track_gac_scales(
                    track_uv,
                    track_att,
                    lock_att,
                )
            track_uv = (0.5, 0.5)
            track_att = (float(lock_att[0]), float(lock_att[1]))
        out: dict[str, object] = {
            "ref_uv": track_uv,
            "ref_att": track_att,
            "h_scale": float(h_scale),
            "v_scale": float(v_scale),
            "lrf_slew": bool(used_lrf_slew),
        }
        return out

    def _pending_lrf_dooaf_pick_role(pending: object | None) -> str | None:
        """DOOAF Setup pick role during LRF slew, or None for observation impact picks."""
        if pending is None:
            return None
        if str(getattr(pending, "purpose", "") or "") != "dooaf_setup":
            return None
        role = str(getattr(pending, "pick_role", "") or "").strip()
        return role if role else None

    def _sync_dooaf_setup_mark_from_lrf_slew_progress(self, role: str | None = None) -> None:
        """Keep DOOAF gun mark aligned with LRF reticle while click-to-aim slew runs."""
        if not bool(getattr(self, "_lrf_lock_in_progress", False)):
            return
        click_uv = getattr(self, "_lrf_click_uv", None)
        click_att = getattr(self, "_lrf_click_att", None)
        if click_uv is None or click_att is None:
            return
        h_scale = float(getattr(self, "_lrf_track_gac_h_scale", 1.0) or 1.0)
        v_scale = float(getattr(self, "_lrf_track_gac_v_scale", 1.0) or 1.0)
        roles = [str(role)] if role else list(self._dooaf_setup_mark_track.keys())
        for rk in roles:
            built = self._dooaf_setup_mark_track.get(str(rk))
            if built is None:
                continue
            built["ref_uv"] = (float(click_uv[0]), float(click_uv[1]))
            built["ref_att"] = (float(click_att[0]), float(click_att[1]))
            built["h_scale"] = h_scale
            built["v_scale"] = v_scale
            built["lrf_slew"] = True

    def _sync_dooaf_setup_track_from_lrf_lock(self, role: str) -> None:
        """Copy post-lock GAC scale factors into one DOOAF setup mark track.

        ref_uv/ref_att stay at boresight+lock pose from _build_video_mark_track;
        overwriting them with pre-lock calibration residuals caused post-OK drift.
        Each role keeps its own scales — never copy the latest global lock onto gun+target.
        """
        h_scale = float(getattr(self, "_lrf_track_gac_h_scale", 1.0) or 1.0)
        v_scale = float(getattr(self, "_lrf_track_gac_v_scale", 1.0) or 1.0)
        built = self._dooaf_setup_mark_track.get(str(role))
        if not built or not built.get("lrf_slew"):
            return
        built["h_scale"] = h_scale
        built["v_scale"] = v_scale

    def _hide_lrf_video_reticle_keep_range(self) -> None:
        """Hide duplicate cyan LRF box on video; keep slant range on PROXIMITY."""
        self._lrf_lock_uv = None
        self._lrf_click_uv = None
        self._lrf_click_att = None
        self._clear_lrf_track_ref()
        try:
            ly = self._native_video_overlay
            ly.set_lrf_lock(None)
            ly.update()
        except Exception:
            pass

    def _register_dooaf_setup_mark_track(
        self,
        role: str,
        *,
        ref_uv: tuple[float, float],
        ref_att: tuple[float, float] | None,
        lock_att: tuple[float, float] | None,
        used_lrf_slew: bool,
        geo_lat: float | None = None,
        geo_lon: float | None = None,
        geo_alt_m: float | None = None,
        lrf_slant_range_m: float | None = None,
    ) -> None:
        """Remember gimbal attitude at pick so marks stay on the world point when camera moves."""
        built = self._build_video_mark_track(
            ref_uv,
            ref_att,
            lock_att,
            used_lrf_slew=used_lrf_slew,
        )
        key = str(role)
        if built is None:
            self._dooaf_setup_mark_track.pop(key, None)
            self._sync_video_mark_track_timer()
            return
        if geo_lat is not None and geo_lon is not None:
            built["geo_lat"] = float(geo_lat)
            built["geo_lon"] = float(geo_lon)
            if geo_alt_m is not None:
                built["geo_alt_m"] = float(geo_alt_m)
        if lrf_slant_range_m is not None:
            try:
                built["lrf_slant_range_m"] = float(lrf_slant_range_m)
            except (TypeError, ValueError):
                pass
        for sk in (
            "smooth_vehicle_lat",
            "smooth_vehicle_lon",
            "smooth_vehicle_heading_deg",
        ):
            built.pop(sk, None)
        self._attach_lock_vehicle_pose_to_track(built)
        self._dooaf_setup_mark_track[key] = built
        self._sync_video_mark_track_timer()

    def _attach_lock_vehicle_pose_to_track(self, track: dict[str, object]) -> None:
        ctx = self._observation_context()
        vlat = ctx.get("vehicle_lat")
        vlon = ctx.get("vehicle_lon")
        if vlat is not None and vlon is not None:
            track["lock_vehicle_lat"] = float(vlat)
            track["lock_vehicle_lon"] = float(vlon)
        hdg = ctx.get("vehicle_heading_deg")
        if hdg is not None:
            try:
                track["lock_vehicle_heading_deg"] = float(hdg)
            except (TypeError, ValueError):
                pass

    def _vehicle_airborne_for_mark_track(self, *, min_rel_alt_m: float = 8.0) -> bool:
        """True when the aircraft is airborne enough that mark tracking needs geo, not gimbal-only."""
        raw = getattr(self, "_vehicle_rel_alt_m", None)
        if raw is None:
            ctx = self._observation_context()
            raw = ctx.get("vehicle_rel_alt_m") or ctx.get("ekf_rel_alt_m")
        if raw is None:
            return False
        try:
            return float(raw) >= float(min_rel_alt_m)
        except (TypeError, ValueError):
            return False

    def _mark_track_use_geo_projection(self, track: dict[str, object]) -> bool:
        """Use geo when no LRF slew, clearly airborne, or when the aircraft has moved since lock."""
        from vgcs.observe.geo_reference import should_project_lrf_mark_via_geo

        glat = track.get("geo_lat")
        glon = track.get("geo_lon")
        has_geo = glat is not None and glon is not None
        if not has_geo:
            return False
        ctx = self._observation_context()
        rel_alt = getattr(self, "_vehicle_rel_alt_m", None)
        if rel_alt is None:
            rel_alt = ctx.get("vehicle_rel_alt_m") or ctx.get("ekf_rel_alt_m")
        if mark_track_use_geo_in_flight(has_geo=True, rel_alt_m=rel_alt):  # type: ignore[arg-type]
            return True
        lrf_slew = bool(track.get("lrf_slew"))
        if not lrf_slew:
            return True
        shift_m = 0.0
        heading_delta: float | None = None
        lock_lat = track.get("lock_vehicle_lat")
        lock_lon = track.get("lock_vehicle_lon")
        clat = ctx.get("vehicle_lat")
        clon = ctx.get("vehicle_lon")
        if lock_lat is not None and lock_lon is not None and clat is not None and clon is not None:
            try:
                shift_m = float(
                    self._haversine_m(
                        float(lock_lat), float(lock_lon), float(clat), float(clon)
                    )
                )
            except (TypeError, ValueError):
                shift_m = 0.0
        lock_h = track.get("lock_vehicle_heading_deg")
        cur_h = ctx.get("vehicle_heading_deg")
        if lock_h is not None and cur_h is not None:
            try:
                heading_delta = float(
                    ((float(cur_h) - float(lock_h) + 180.0) % 360.0) - 180.0
                )
            except (TypeError, ValueError):
                heading_delta = None
        slant_raw = track.get("lrf_slant_range_m")
        slant_m: float | None = None
        if slant_raw is not None:
            try:
                slant_m = float(slant_raw)
            except (TypeError, ValueError):
                slant_m = None
        return should_project_lrf_mark_via_geo(
            lrf_slew=True,
            has_geo=True,
            rel_alt_m=rel_alt,  # type: ignore[arg-type]
            vehicle_shift_m=shift_m,
            heading_delta_deg=heading_delta,
            slant_range_m=slant_m,
        )

    def _video_mark_tracking_active(self) -> bool:
        if bool(getattr(self, "_lrf_lock_in_progress", False)):
            return True
        if getattr(self, "_dooaf_setup_mark_track", None):
            return True
        for row in self._observations:
            if row.get("video_mark_track_ref_u") is not None:
                return True
        return False

    def _sync_video_mark_track_timer(self) -> None:
        """Start/stop throttled mark tracking (~12 Hz) — never on every video frame."""
        if not self._video_mark_tracking_active():
            t = getattr(self, "_video_mark_track_timer", None)
            if t is not None:
                t.stop()
            return
        t = getattr(self, "_video_mark_track_timer", None)
        if t is None:
            t = QTimer(self)
            t.setInterval(50)
            t.timeout.connect(self._refresh_tracked_video_marks_light)
            self._video_mark_track_timer = t
        fast = bool(getattr(self, "_lrf_lock_in_progress", False))
        t.setInterval(33 if fast else 50)
        if not t.isActive():
            t.start()

    def _refresh_tracked_video_marks_light(self) -> None:
        """Reproject tracked marks and LRF reticle during slew (~20 Hz)."""
        if bool(getattr(self, "_lrf_lock_in_progress", False)):
            try:
                pending = getattr(self, "_pending_lrf_video_pick", None)
                pick_role = self._pending_lrf_dooaf_pick_role(pending)
                if pick_role:
                    self._sync_dooaf_setup_mark_from_lrf_slew_progress(pick_role)
                self._refresh_lrf_lock_overlay(sync_geometry=False)
            except Exception:
                pass
        if not (
            getattr(self, "_dooaf_setup_mark_track", None)
            or any(
                row.get("video_mark_track_ref_u") is not None
                for row in self._observations
            )
        ):
            if not bool(getattr(self, "_lrf_lock_in_progress", False)):
                self._sync_video_mark_track_timer()
            return
        try:
            marks = self._video_overlay_marks()
            self._video_obs_marks = [(m.x, m.y) for m in marks]
            ly = self._native_video_overlay
            ly.set_video_marks(marks)
            ly.set_offscreen_hints(self._video_overlay_offscreen_hints())
            self._refresh_dooaf_facade_overlay_hint()
            ly.update()
        except Exception:
            pass

    def _apply_video_mark_gimbal_track_to_row(
        self,
        row: dict[str, object],
        u: float,
        v: float,
        *,
        ref_att: tuple[float, float] | None,
        lock_att: tuple[float, float] | None,
        used_lrf_slew: bool,
    ) -> None:
        built = self._build_video_mark_track(
            (float(u), float(v)),
            ref_att,
            lock_att,
            used_lrf_slew=used_lrf_slew,
        )
        if built is None:
            for key in (
                "video_mark_track_ref_u",
                "video_mark_track_ref_v",
                "video_mark_track_ref_yaw",
                "video_mark_track_ref_pitch",
                "video_mark_track_h_scale",
                "video_mark_track_v_scale",
            ):
                row.pop(key, None)
            self._sync_video_mark_track_timer()
            return
        ref_uv = built["ref_uv"]
        ref_att_out = built["ref_att"]
        assert isinstance(ref_uv, tuple) and isinstance(ref_att_out, tuple)
        row["video_mark_track_ref_u"] = float(ref_uv[0])
        row["video_mark_track_ref_v"] = float(ref_uv[1])
        row["video_mark_track_ref_yaw"] = float(ref_att_out[0])
        row["video_mark_track_ref_pitch"] = float(ref_att_out[1])
        row["video_mark_track_h_scale"] = float(built["h_scale"])
        row["video_mark_track_v_scale"] = float(built["v_scale"])
        row["video_mark_lrf_slew"] = bool(used_lrf_slew)
        ctx = self._observation_context()
        vlat = ctx.get("vehicle_lat")
        vlon = ctx.get("vehicle_lon")
        if vlat is not None and vlon is not None:
            row["lock_vehicle_lat"] = float(vlat)
            row["lock_vehicle_lon"] = float(vlon)
        hdg = ctx.get("vehicle_heading_deg")
        if hdg is not None:
            try:
                row["lock_vehicle_heading_deg"] = float(hdg)
            except (TypeError, ValueError):
                pass
        tlat = row.get("target_lat")
        tlon = row.get("target_lon")
        if tlat is not None and tlon is not None:
            row["video_mark_geo_lat"] = float(tlat)
            row["video_mark_geo_lon"] = float(tlon)
            talt = row.get("target_alt_m")
            if talt is not None:
                try:
                    row["video_mark_geo_alt_m"] = float(talt)
                except (TypeError, ValueError):
                    pass
        self._sync_video_mark_track_timer()

    def _project_geo_to_video_norm(
        self,
        lat: float,
        lon: float,
        alt_m: float | None = None,
        *,
        pose_store: dict[str, object] | None = None,
        smooth_pose: bool = False,
    ) -> tuple[float, float] | None:
        ctx = self._observation_context()
        hfov, vfov = self._c13_lrf_geo_fov()
        vlat = ctx.get("vehicle_lat")
        vlon = ctx.get("vehicle_lon")
        vhdg = ctx.get("vehicle_heading_deg")
        if smooth_pose and pose_store is not None:
            from vgcs.observe.geo_reference import smooth_vehicle_pose_ema

            slat, slon, shdg = smooth_vehicle_pose_ema(
                pose_store,
                vehicle_lat=vlat,  # type: ignore[arg-type]
                vehicle_lon=vlon,  # type: ignore[arg-type]
                vehicle_heading_deg=vhdg,  # type: ignore[arg-type]
            )
            if slat is not None:
                vlat = slat
            if slon is not None:
                vlon = slon
            if shdg is not None:
                vhdg = shdg
        try:
            return project_wgs84_to_video_norm(
                target_lat=float(lat),
                target_lon=float(lon),
                target_alt_m=alt_m,
                vehicle_lat=vlat,  # type: ignore[arg-type]
                vehicle_lon=vlon,  # type: ignore[arg-type]
                vehicle_heading_deg=vhdg,  # type: ignore[arg-type]
                vehicle_roll_deg=ctx.get("vehicle_roll_deg"),  # type: ignore[arg-type]
                vehicle_pitch_deg=ctx.get("vehicle_pitch_deg"),  # type: ignore[arg-type]
                vehicle_alt_msl_m=ctx.get("vehicle_alt_msl_m"),  # type: ignore[arg-type]
                gimbal_yaw_deg=ctx.get("gimbal_yaw_deg"),  # type: ignore[arg-type]
                gimbal_pitch_deg=ctx.get("gimbal_pitch_deg"),  # type: ignore[arg-type]
                camera_hfov_deg=hfov,
                camera_vfov_deg=vfov,
            )
        except Exception:
            return None

    def _attitude_mark_uv_from_track(
        self,
        track: dict[str, object],
        stored_uv: tuple[float, float],
    ) -> tuple[float, float] | None:
        cur = self._read_gimbal_attitude_pair()
        if cur is None:
            return None
        ref_uv = track.get("ref_uv")
        ref_att = track.get("ref_att")
        if not isinstance(ref_uv, tuple) or not isinstance(ref_att, tuple):
            return None
        try:
            from vgcs.skydroid.adapter import SkydroidTopUdpAdapter

            u, v = SkydroidTopUdpAdapter.lrf_track_uv_from_attitude(
                (float(ref_uv[0]), float(ref_uv[1])),
                (float(ref_att[0]), float(ref_att[1])),
                cur,
                gac_h_scale=float(track.get("h_scale", 1.0) or 1.0),
                gac_v_scale=float(track.get("v_scale", 1.0) or 1.0),
                clamp=False,
            )
            return (float(u), float(v))
        except Exception:
            return None

    def _project_mark_uv_unclamped(
        self,
        track: dict[str, object] | None,
        stored_uv: tuple[float, float],
    ) -> tuple[float, float]:
        att_uv: tuple[float, float] | None = None
        if track is not None:
            att_uv = self._attitude_mark_uv_from_track(track, stored_uv)
        if track is not None and self._mark_track_use_geo_projection(track):
            glat = track.get("geo_lat")
            glon = track.get("geo_lon")
            galt = track.get("geo_alt_m")
            try:
                alt = float(galt) if galt is not None else None
            except (TypeError, ValueError):
                alt = None
            geo_uv = self._project_geo_to_video_norm(
                float(glat), float(glon), alt_m=alt,  # type: ignore[arg-type]
                pose_store=track,
                smooth_pose=bool(
                    track.get("lrf_slew")
                    and self._vehicle_airborne_for_mark_track()
                ),
            )
            if geo_uv is not None:
                gu, gv = float(geo_uv[0]), float(geo_uv[1])
                airborne = self._vehicle_airborne_for_mark_track(min_rel_alt_m=3.0)
                if track.get("lrf_slew") and att_uv is not None and not airborne:
                    au, av = att_uv
                    if abs(gu - au) > 0.10 or abs(gv - av) > 0.10:
                        return (float(au), float(av))
                return (gu, gv)
        if att_uv is not None:
            return att_uv
        if not track:
            return (float(stored_uv[0]), float(stored_uv[1]))
        return (float(stored_uv[0]), float(stored_uv[1]))

    def _mark_uv_on_screen(u: float, v: float, *, margin: float = 0.02) -> bool:
        return (
            float(u) >= -float(margin)
            and float(u) <= 1.0 + float(margin)
            and float(v) >= -float(margin)
            and float(v) <= 1.0 + float(margin)
        )

    def _make_offscreen_hint(
        self,
        role: str,
        raw_uv: tuple[float, float],
        *,
        index: int = 0,
    ) -> VideoOverlayOffscreenHint:
        edge_x, edge_y, angle = offscreen_hint_edge_uv(
            float(raw_uv[0]),
            float(raw_uv[1]),
        )
        name = dooaf_role_display(str(role or "")).strip() or "Mark"
        return VideoOverlayOffscreenHint(
            edge_x=float(edge_x),
            edge_y=float(edge_y),
            angle_deg=float(angle),
            label=f"{name} off-screen — see map",
            role=str(role or ""),
            index=int(index),
        )

    def _tracked_uv_from_store(
        self,
        track: dict[str, object] | None,
        stored_uv: tuple[float, float],
    ) -> tuple[float, float] | None:
        """Project a world-fixed mark to screen UV; None when the point is outside the frame."""
        u, v = self._project_mark_uv_unclamped(track, stored_uv)
        if track is not None and not self._mark_uv_on_screen(u, v):
            return None
        if track is not None:
            return (
                max(0.0, min(1.0, float(u))),
                max(0.0, min(1.0, float(v))),
            )
        return (float(u), float(v))

    def _dooaf_mark_display_uv(
        self, role: str, stored_uv: tuple[float, float]
    ) -> tuple[float, float] | None:
        """Screen UV for a DOOAF setup mark — tracks gimbal; hidden when off-screen."""
        if bool(getattr(self, "_lrf_lock_in_progress", False)):
            pending = getattr(self, "_pending_lrf_video_pick", None)
            active_role = self._pending_lrf_dooaf_pick_role(pending)
            if active_role and str(role) == active_role:
                lock_uv = getattr(self, "_lrf_lock_uv", None)
                if lock_uv is not None:
                    u, v = float(lock_uv[0]), float(lock_uv[1])
                    if self._mark_uv_on_screen(u, v):
                        return (u, v)
                    return None
        track = self._dooaf_setup_mark_track.get(str(role))
        return self._tracked_uv_from_store(track, stored_uv)

    def _persist_mark_track_smooth_keys(
        track: dict[str, object], row: dict[str, object]
    ) -> None:
        for key in (
            "smooth_vehicle_lat",
            "smooth_vehicle_lon",
            "smooth_vehicle_heading_deg",
        ):
            if key in track:
                row[key] = track[key]

    def _observation_mark_display_uv(
        self, row: dict[str, object], stored_u: float, stored_v: float
    ) -> tuple[float, float] | None:
        ref_u = row.get("video_mark_track_ref_u")
        if ref_u is None:
            return (float(stored_u), float(stored_v))
        try:
            track: dict[str, object] = {
                "ref_uv": (
                    float(ref_u),
                    float(row.get("video_mark_track_ref_v") or 0.0),
                ),
                "ref_att": (
                    float(row.get("video_mark_track_ref_yaw") or 0.0),
                    float(row.get("video_mark_track_ref_pitch") or 0.0),
                ),
                "h_scale": float(row.get("video_mark_track_h_scale") or 1.0),
                "v_scale": float(row.get("video_mark_track_v_scale") or 1.0),
                "lrf_slew": bool(row.get("video_mark_lrf_slew")),
            }
            glat = row.get("video_mark_geo_lat")
            glon = row.get("video_mark_geo_lon")
            if glat is not None and glon is not None:
                track["geo_lat"] = float(glat)
                track["geo_lon"] = float(glon)
                galt = row.get("video_mark_geo_alt_m")
                if galt is not None:
                    track["geo_alt_m"] = float(galt)
            for key in (
                "lock_vehicle_lat",
                "lock_vehicle_lon",
                "lock_vehicle_heading_deg",
            ):
                if row.get(key) is not None:
                    track[key] = row.get(key)
            for sk in (
                "smooth_vehicle_lat",
                "smooth_vehicle_lon",
                "smooth_vehicle_heading_deg",
            ):
                if row.get(sk) is not None:
                    track[sk] = row.get(sk)
            slant = row.get("lrf_slant_range_m")
            if slant is not None:
                try:
                    track["lrf_slant_range_m"] = float(slant)
                except (TypeError, ValueError):
                    pass
        except (TypeError, ValueError):
            return (float(stored_u), float(stored_v))
        uv = self._tracked_uv_from_store(track, (float(stored_u), float(stored_v)))
        self._persist_mark_track_smooth_keys(track, row)
        return uv

    def _video_overlay_offscreen_hints(self) -> list[VideoOverlayOffscreenHint]:
        """Edge arrows for tracked marks that left the current video frame."""
        out: list[VideoOverlayOffscreenHint] = []
        for role, pt in self._dooaf_setup_video_marks.items():
            track = self._dooaf_setup_mark_track.get(str(role))
            if not track:
                continue
            try:
                raw = self._project_mark_uv_unclamped(track, pt)
                if self._mark_uv_on_screen(raw[0], raw[1]):
                    continue
                out.append(self._make_offscreen_hint(str(role), raw, index=0))
            except (TypeError, ValueError, IndexError):
                continue
        for idx, row in enumerate(self._observations):
            if str(row.get("kind") or "") != "video_mark":
                continue
            if row.get("video_mark_track_ref_u") is None:
                continue
            vx = row.get("video_x_norm")
            vy = row.get("video_y_norm")
            if vx is None or vy is None:
                continue
            try:
                track: dict[str, object] = {
                    "ref_uv": (
                        float(row.get("video_mark_track_ref_u") or 0.0),
                        float(row.get("video_mark_track_ref_v") or 0.0),
                    ),
                    "ref_att": (
                        float(row.get("video_mark_track_ref_yaw") or 0.0),
                        float(row.get("video_mark_track_ref_pitch") or 0.0),
                    ),
                    "h_scale": float(row.get("video_mark_track_h_scale") or 1.0),
                    "v_scale": float(row.get("video_mark_track_v_scale") or 1.0),
                    "lrf_slew": bool(row.get("video_mark_lrf_slew")),
                }
                glat = row.get("video_mark_geo_lat")
                glon = row.get("video_mark_geo_lon")
                if glat is not None and glon is not None:
                    track["geo_lat"] = float(glat)
                    track["geo_lon"] = float(glon)
                    galt = row.get("video_mark_geo_alt_m")
                    if galt is not None:
                        track["geo_alt_m"] = float(galt)
                raw = self._project_mark_uv_unclamped(
                    track, (float(vx), float(vy))
                )
                if self._mark_uv_on_screen(raw[0], raw[1]):
                    continue
                out.append(
                    self._make_offscreen_hint(
                        str(row.get("dooaf_role") or ""),
                        raw,
                        index=idx + 1,
                    )
                )
            except (TypeError, ValueError):
                continue
        return out

    def _video_overlay_marks(self) -> list[VideoOverlayMark]:
        """Video clicks: DOOAF Setup picks (green target) + observation fall-of-shot (red)."""
        out: list[VideoOverlayMark] = []
        seen: set[tuple[float, float]] = set()
        in_slew = bool(getattr(self, "_lrf_lock_in_progress", False))
        pending = getattr(self, "_pending_lrf_video_pick", None)
        slew_role = (
            self._pending_lrf_dooaf_pick_role(pending) if in_slew else None
        )
        for role, pt in self._dooaf_setup_video_marks.items():
            if slew_role and str(role) == slew_role:
                continue
            try:
                disp = self._dooaf_mark_display_uv(str(role), pt)
                if disp is None:
                    continue
                key = (round(float(disp[0]), 4), round(float(disp[1]), 4))
                if key in seen:
                    continue
                seen.add(key)
                out.append(
                    VideoOverlayMark(
                        x=float(disp[0]),
                        y=float(disp[1]),
                        role=str(role),
                        index=0,
                    )
                )
            except (TypeError, ValueError, IndexError):
                continue
        for idx, row in enumerate(self._observations):
            if str(row.get("kind") or "") != "video_mark":
                continue
            vx = row.get("video_x_norm")
            vy = row.get("video_y_norm")
            if vx is None or vy is None:
                continue
            try:
                disp = self._observation_mark_display_uv(row, float(vx), float(vy))
                if disp is None:
                    continue
                out.append(
                    VideoOverlayMark(
                        x=float(disp[0]),
                        y=float(disp[1]),
                        role=str(row.get("dooaf_role") or ""),
                        index=idx + 1,
                    )
                )
                seen.add((round(float(disp[0]), 4), round(float(disp[1]), 4)))
            except (TypeError, ValueError):
                continue
        return out
