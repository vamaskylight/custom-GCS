"""MapWidget video mixin — see vgcs.map.video package."""

from __future__ import annotations

import base64
import json
import os
import time

from PySide6.QtCore import QSettings, QThreadPool, Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QMessageBox

from vgcs.map.app_settings import QS_APP, QS_ORG
from vgcs.map.video.encode_bridge import VideoEncodeBridge, VideoEncodeTask
from vgcs.map.video.helpers import _format_video_zoom_label
from vgcs.map.video.settings_keys import (
    KEY_VIDEO_DEFAULT_VIEW,
    KEY_VIDEO_ENABLED,
    KEY_VIDEO_LOW_LATENCY,
    KEY_VIDEO_RECORD_FORMAT,
    KEY_VIDEO_RTSP_DAY,
    KEY_VIDEO_RTSP_THERMAL,
    KEY_VIDEO_RTSP_TRANSPORT,
    KEY_VIDEO_SOURCE,
)
from vgcs.video.pipeline import (
    HAS_MULTIMEDIA,
    VideoFrame,
    VideoPipeline,
    QS_KEY_LAST_PHOTO_SAVE_DIR,
    notify_companion_app_background,
    notify_companion_app_foreground,
    notify_companion_preview_motion,
    release_all_companion_rtsp_hosts,
    release_companion_rtsp_host,
    set_companion_decode_gate,
    suggested_photo_save_path,
    suggested_recording_save_path,
    wait_qmedia_recorder_stopped,
)


class VideoPipelineMixin:
    """Extracted from MapWidget — uses host widget state via self."""

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
            try:
                cur_active = str(vp.active_source_id() or "").strip()
            except Exception:
                cur_active = ""
            if cur_active in src_ids:
                preferred_id = cur_active
            elif "day" in src_ids:
                preferred_id = "day"
            elif "thermal" in src_ids:
                preferred_id = "thermal"
            elif src_ids:
                preferred_id = str(next(iter(src_ids)))
            if preferred_id and preferred_id != cur_active:
                try:
                    vp.set_active_source(preferred_id)
                    self._video_active_source = vp.active_source()
                except Exception:
                    pass
            elif preferred_id and self._video_active_source is None:
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
                    bridge = VideoEncodeBridge(self)
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
        if not self._mini_video_pip_allowed():
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
        self._video_zoom = 1.0
        try:
            self._sync_native_video_zoom_label()
        except Exception:
            pass
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
        try:
            old_br = getattr(self, "_video_encode_bridge", None)
            if old_br is not None:
                try:
                    old_br.encoded.disconnect(self)
                except Exception:
                    pass
        except Exception:
            pass
        self._video_encode_bridge = VideoEncodeBridge(self)
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
        s = QSettings(QS_ORG, QS_APP)
        source = str(s.value(KEY_VIDEO_SOURCE, "rtsp") or "rtsp").strip().lower()
        self._video_settings_source = source
        self._video_settings_day = str(s.value(KEY_VIDEO_RTSP_DAY, "") or "").strip()
        self._video_settings_thermal = str(s.value(KEY_VIDEO_RTSP_THERMAL, "") or "").strip()
        has_stream = bool(self._video_settings_day or self._video_settings_thermal) or source in (
            "udp_h264",
            "udp_h265",
        )
        explicit_on = bool(s.value(KEY_VIDEO_ENABLED, False))
        self._video_settings_enabled = (explicit_on or has_stream) and source != "disabled"
        self._video_settings_rtsp_transport = str(s.value(KEY_VIDEO_RTSP_TRANSPORT, "auto") or "auto").strip().lower()
        self._video_settings_low_latency = bool(s.value(KEY_VIDEO_LOW_LATENCY, False))
        rec_fmt = str(s.value(KEY_VIDEO_RECORD_FORMAT, "mp4") or "mp4").strip().lower()
        self._video_settings_record_format = rec_fmt if rec_fmt in ("mp4", "mkv") else "mp4"
        self._video_settings_default_view = str(s.value(KEY_VIDEO_DEFAULT_VIEW, "Single") or "Single")

    def _video_record_suffix(self) -> str:
        return str(getattr(self, "_video_settings_record_format", "mp4") or "mp4")

    def _video_preview_should_run(self) -> bool:
        """True when a stream is configured and the map is ready (no toolbar toggle required)."""
        return bool(getattr(self, "_video_settings_enabled", False)) and bool(
            getattr(self, "_web_ready", False)
        )

    def _mini_video_pip_allowed(self) -> bool:
        """Bottom-left PiP / 2×2 split is shown only after the vehicle MAVLink link is up."""
        return bool(getattr(self, "_last_link_connected", False))

    def _auto_start_mini_video_pip(
        self,
        *,
        force_decode: bool = False,
        preserve_layout: bool = False,
    ) -> None:
        """Show bottom-left mini-video automatically when RTSP/UDP is configured."""
        if not self._mini_video_pip_allowed():
            return
        if not self._video_preview_should_run():
            return
        self._show_mini_video_pip_shell()
        if preserve_layout:
            reset_swapped = False
        else:
            reset_swapped = not bool(getattr(self, "_video_swap_user_map_main", False))
        self._start_video_preview(reset_swapped=reset_swapped, force_decode=force_decode)

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
                if self._uses_companion_rtsp():
                    dv = (
                        str(getattr(self, "_video_settings_default_view", "Single") or "Single")
                        .strip()
                        .lower()
                    )
                    if dv == "split":
                        try:
                            print(
                                "[VGCS:video] C13 companion: split layout OK — only one RTSP "
                                "decodes at a time; use IR or tap Thermal cell to switch feed"
                            )
                        except Exception:
                            pass
                    self._video_split_enabled = dv == "split"
                else:
                    self._video_split_enabled = (
                        str(
                            getattr(self, "_video_settings_default_view", "Single") or "Single"
                        )
                        .strip()
                        .lower()
                        == "split"
                    )
            except Exception:
                pass
            self._sync_native_camera_rail_toggles()
            self._sync_native_thermal_feed_button()
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
            dv = (
                str(getattr(self, "_video_settings_default_view", "Single") or "Single")
                .strip()
                .lower()
            )
            self._video_split_enabled = dv == "split"
            if self._uses_companion_rtsp() and dv == "split":
                try:
                    print(
                        "[VGCS:video] C13 companion: split layout — one RTSP at a time; "
                        "IR button or Thermal cell switches feed"
                    )
                except Exception:
                    pass
        except Exception:
            pass
        self._sync_native_camera_rail_toggles()
        self._sync_native_thermal_feed_button()

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
                self._auto_start_mini_video_pip(force_decode=False)
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
        # C13 / companion: hardware allows one RTSP client — never decode day+thermal together.
        if self._uses_companion_rtsp():
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

    def _refresh_companion_video_after_foreground(self, *, elapsed_bg_s: float) -> None:
        last = float(getattr(self, "_last_foreground_video_refresh_mono", 0.0) or 0.0)
        if time.monotonic() - last < 6.0:
            return
        vp = getattr(self, "_video", None)
        if vp is None:
            return
        want = self._video_preview_source_ids_to_run(vp)
        if not want:
            return
        # Alt-tab during link-up: don't schedule a 2–3s RTSP restart while still waiting
        # for the first preview frame (that was adding 10–20s to "camera connect").
        try:
            for sid in want:
                src = vp.sources().get(sid)
                if src is None:
                    continue
                if bool(getattr(src, "_ffmpeg_had_frame", False)):
                    break
                if hasattr(src, "decode_recently_active") and src.decode_recently_active(
                    max_age_s=45.0
                ):
                    try:
                        print(
                            "[VGCS:video] foreground refresh skipped — "
                            "still waiting for first RTSP preview frame"
                        )
                    except Exception:
                        pass
                    return
        except Exception:
            pass
        self._last_foreground_video_refresh_mono = time.monotonic()
        delay_ms = 3200 if elapsed_bg_s >= 8.0 else 2200
        try:
            print(
                f"[VGCS:video] foreground RTSP refresh for {want!r} "
                f"(background {elapsed_bg_s:.0f}s, delay={delay_ms}ms)"
            )
        except Exception:
            pass
        for sid in want:
            try:
                src = vp.sources().get(sid)
                if src is not None and hasattr(src, "restart_decode"):
                    src.restart_decode(delay_ms=delay_ms)
            except Exception:
                pass

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
        if self._uses_companion_rtsp():
            try:
                print(
                    f"[VGCS:video] companion decode plan: split="
                    f"{bool(getattr(self, '_video_split_enabled', False))} "
                    f"sources={sorted(want)}"
                )
            except Exception:
                pass
        self._stop_idle_video_decode_sources(vp)
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
        if not self._mini_video_pip_allowed():
            try:
                print("[VGCS:video] preview start skipped: vehicle not connected")
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
            try:
                self._native_video_overlay.clear_detections()
            except Exception:
                pass
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
        try:
            self._native_video_overlay.clear_all()
            self._lrf_lock_uv = None
            self._lrf_lock_distance_m = None
            self._lrf_lock_armed = False
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
            if self._uses_companion_rtsp() and self._mini_video_pip_allowed():
                self._video_preview_enabled = True
                if not bool(getattr(self, "_video_swap_user_map_main", False)):
                    self._video_swapped = False
                try:
                    self._native_video_preview.show()
                    if bool(getattr(self, "_web_ready", False)):
                        self._native_compass.show()
                        self._native_telemetry.show()
                        mz = getattr(self, "_native_map_zoom_ctrl", None)
                        if mz is not None:
                            mz.show()
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

        sid = self._operator_preview_source_id()
        try:
            self._cache_preview_raw_frame(sid or "day", img)
        except Exception:
            pass

        img = self._apply_digital_zoom(
            img,
            self._effective_preview_digital_zoom(sid),
        )
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
        try:
            self._cache_preview_raw_frame(sid, img)
        except Exception:
            pass
        img = self._apply_digital_zoom(img, self._effective_preview_digital_zoom(sid))
        try:
            img2 = img.copy()
        except Exception:
            img2 = img

        try:
            self._split_last_images[str(source_id)] = img2
        except Exception:
            pass

        try:
            vp_act = getattr(self, "_video", None)
            active_sid = str(vp_act.active_source_id() or "").strip() if vp_act is not None else ""
        except Exception:
            active_sid = ""
        if active_sid and str(source_id) == active_sid:
            try:
                self._native_pip_last_source_frame = img2
            except Exception:
                pass
        if not bool(getattr(self, "_video_gui_logged_frame", False)):
            self._video_gui_logged_frame = True
            try:
                print(
                    "[VGCS:video] GUI preview receiving frames "
                    f"(split=True source={source_id!r} swapped="
                    f"{bool(getattr(self, '_video_swapped', False))})"
                )
            except Exception:
                pass

        # C13 IR switch: when split/fullscreen, paint the active feed immediately (not stale day).
        try:
            vp = getattr(self, "_video", None)
            active = str(vp.active_source_id() or "").strip() if vp is not None else ""
        except Exception:
            active = ""
        if (
            self._uses_companion_rtsp()
            and active
            and str(source_id) == active
            and bool(getattr(self, "_video_swapped", False))
        ):
            focus = str(getattr(self, "_split_fullscreen_source_id", None) or active)
            if str(source_id) == focus:
                last_render = float(getattr(self, "_video_ui_render_mono", 0.0) or 0.0)
                if now_frame - last_render >= 0.04:
                    self._video_ui_render_mono = now_frame
                    try:
                        self._native_pip_last_source_frame = img2
                        self._render_native_video_preview(img2)
                    except RuntimeError:
                        pass
                return

        if bool(getattr(self, "_video_swapped", False)):
            focus = self._ensure_split_fullscreen_focus()
            if focus and str(source_id) == str(focus):
                last_render = float(getattr(self, "_video_ui_render_mono", 0.0) or 0.0)
                if now_frame - last_render >= 0.04:
                    self._video_ui_render_mono = now_frame
                    try:
                        self._render_native_video_preview(img2)
                    except RuntimeError:
                        pass
                return

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
        task = VideoEncodeTask(
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
        task = VideoEncodeTask(
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

    def _cache_preview_raw_frame(self, source_id: str, img: QImage) -> None:
        """Keep the latest uncropped preview frame so rail zoom can repaint immediately."""
        sid = str(source_id or "").strip() or "day"
        if img is None or img.isNull():
            return
        try:
            raw = img.copy()
        except Exception:
            raw = img
        cache = getattr(self, "_split_last_raw_images", None)
        if not isinstance(cache, dict):
            cache = {}
            self._split_last_raw_images = cache
        cache[sid] = raw
        try:
            active = self._operator_preview_source_id()
        except Exception:
            active = sid
        if not active or sid == active:
            self._native_pip_last_raw_frame = raw

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

    def set_video_follow_enabled(self, enabled: bool) -> None:
        """Same Follow behavior as the map camera rail (center map on vehicle while on)."""
        self._on_web_title_changed(f"VGCS_CAM_FOLLOW_TOGGLE:{1 if bool(enabled) else 0}:0")
        self._sync_native_camera_rail_toggles()

    def _notify_companion_gimbal_motion(self, *, duration_s: float = 2.5) -> None:
        if not self._uses_companion_rtsp():
            return
        try:
            notify_companion_preview_motion(duration_s=float(duration_s))
        except Exception:
            pass
