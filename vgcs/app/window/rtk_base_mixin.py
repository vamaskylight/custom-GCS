"""MainWindow mixin: the RTK base station (milestone M11).

Reads corrections from a base station on this PC and passes them to the drone
over the MAVLink link. See vgcs.link.rtk_base (the reader), vgcs.link.rtcm (the
message format) and vgcs.app.rtk_corrections (what is shown).
"""

from __future__ import annotations

import time

from vgcs.app.rtk_corrections import STATE_CONNECTED, STATE_ERROR, STATE_OFF
from vgcs.link.rtk_base import BaseSource, RtkBaseThread, parse_source

# QSettings key: where the base is, as typed ("COM7:115200", "192.168.1.50:2101"). Empty means no base.
KEY_RTK_BASE_SOURCE = "rtk/base_source"


class MainWindowRtkBaseMixin:
    """Part of MainWindow. Uses the window's state through self."""

    def _rtk_base_source_setting(self) -> str:
        return str(self._settings.value(KEY_RTK_BASE_SOURCE, "") or "").strip()

    def _apply_rtk_base_setting(self) -> None:
        """Start, stop or restart the base reader so that it matches the saved setting."""
        source = parse_source(self._rtk_base_source_setting())
        running = getattr(self, "_rtk_base_thread", None)
        if running is not None and source is not None and running.source() == source:
            return
        self._stop_rtk_base()
        if source is None:
            typed = self._rtk_base_source_setting()
            if typed:
                # Something is typed, but it is not an address. Say so, do not just stay "Off".
                self._rtk_corrections.set_state(STATE_ERROR, typed, "not a port or address")
            self._publish_rtk_corrections()
            return
        self._start_rtk_base(source)

    def _start_rtk_base(self, source: BaseSource) -> None:
        thread = RtkBaseThread(source, self)
        thread.state.connect(self._on_rtk_base_state)
        thread.corrections.connect(self._on_rtk_base_frame)
        thread.station.connect(self._on_rtk_base_station)
        self._rtk_base_thread = thread
        self._rtk_corrections.set_state("connecting", source.label())
        self._publish_rtk_corrections()
        self._append_log(f"RTK base: reading {source.label()}")
        thread.start()

    def _stop_rtk_base(self) -> None:
        thread = getattr(self, "_rtk_base_thread", None)
        self._rtk_base_thread = None
        if thread is not None:
            try:
                thread.state.disconnect(self._on_rtk_base_state)
                thread.corrections.disconnect(self._on_rtk_base_frame)
                thread.station.disconnect(self._on_rtk_base_station)
            except Exception:
                pass
            thread.stop()
            thread.wait(3000)
        self._rtk_corrections.set_state(STATE_OFF)
        self._publish_rtk_corrections()

    def _on_rtk_base_state(self, state: str, detail: str) -> None:
        thread = getattr(self, "_rtk_base_thread", None)
        if thread is None:
            return
        self._rtk_corrections.set_state(state, thread.source().label(), detail)
        if state == STATE_ERROR:
            self._append_log(f"RTK base: {thread.source().label()}: {detail}")
        elif state == STATE_CONNECTED:
            self._append_log(f"RTK base: connected to {thread.source().label()}")
        self._publish_rtk_corrections()

    def _on_rtk_base_frame(self, data: bytes, message_type: int) -> None:
        """One whole correction message from the base: count it, and send it to the drone."""
        if getattr(self, "_rtk_base_thread", None) is None:
            return
        self._rtk_corrections.add_frame(int(message_type), len(data))
        link = getattr(self, "_thread", None)
        live = bool(self._preflight_link_is_live())
        self._rtk_corrections.set_link_up(live)
        if live and link is not None:
            link.queue_rtcm(bytes(data))

    def _on_rtk_base_station(self, lat: float, lon: float, height_m: float) -> None:
        first = self._rtk_corrections.station() is None
        self._rtk_corrections.set_station(lat, lon, height_m)
        if first:
            self._append_log(f"RTK base: position {lat:.6f}, {lon:.6f}, height {height_m:.1f} m")

    def _publish_rtk_corrections(self) -> None:
        """Paint the "RTK corrections" value. Called on changes and once a second."""
        stream = self._rtk_corrections
        now = time.monotonic()
        if stream.state != STATE_OFF:
            stream.set_link_up(bool(self._preflight_link_is_live()))
        field = self._fields["rtk_corrections"]
        text = stream.text(now)
        if field.text() != text:
            field.setText(text)
        level = stream.level(now)
        if str(field.property("state_role") or "") != level:
            self._apply_state_style(field, level)
        field.setToolTip(stream.tooltip(now))
