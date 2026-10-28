"""Recording while matching (--record): the flight recorder's folders, from the same pictures.

The Pi has one down camera and one serial port to the flight controller, and only one program can
hold each. So with --record vama_nav keeps the pictures it takes, each with the drone's state at
its moment, in the same flight folders as vama_recorder (images/, frames.csv, telemetry.jsonl,
flight.json, see vama_recorder/flight.py), by the recorder's own rules: from start_height_m while
armed, not when the card is nearly full. Those flights are step 2's data, matched or not.

The writing has its own thread. The flight folder's rows are forced to the card every ten rows
(fsync), and that can take a second or more on a busy card: in the link's thread it held up
GPS_INPUT for 1.8 s in the simulator (2026-10-29). The link's thread only hands messages over.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

from vama_recorder import __version__ as recorder_version
from vama_recorder.flight import LOGGED_MESSAGES, Flight
from vama_recorder.recorder import DISK_CHECK_NS, disk_free_mb, recording_wanted
from vama_recorder.settings import Settings
from vama_recorder.telemetry import Telemetry

log = logging.getLogger("vama_nav.recording")

STEP_S = 0.05


class Recording:
    """Thread safe: the camera thread keeps pictures, the link's thread hands messages over, and the
    recording's own thread (or ``poll`` without threads) opens, closes and writes."""

    def __init__(self, settings: Settings, telemetry: Telemetry, describe_camera: Callable[[], dict],
                 free_mb: Callable[[Path], int | None] = disk_free_mb, interval_s: float = 1.0,
                 clock: Callable[[], int] = time.monotonic_ns) -> None:
        self.settings = settings
        self.telemetry = telemetry
        self.describe_camera = describe_camera
        self.free_mb = free_mb
        self.interval_s = float(interval_s)
        self.clock = clock
        self.root = settings.flights_dir
        self.flight: Flight | None = None
        self.active = False                 # a flight is open: pictures are wanted
        self.flights_closed = 0
        self.save_errors = 0
        self.dropped = 0                    # messages not logged (the writing fell behind)
        self.why = ""
        self._lock = threading.Lock()
        self._messages: queue.Queue = queue.Queue(maxsize=5000)
        self._free: int | None = None
        self._next_disk = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # --- the other threads' calls ---------------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="recording", daemon=True)
        self._thread.start()

    def hear(self, msg, arrival_ns: int) -> None:
        """A message for the flight's telemetry.jsonl. Never waits."""
        if self.active and msg.get_type() in LOGGED_MESSAGES:
            try:
                self._messages.put_nowait((msg, int(arrival_ns)))
            except queue.Full:
                self.dropped += 1

    def keep(self, shot, now_ns: int) -> str | None:
        """Save a picture into the open flight, with its row (the camera's thread). Its name, or None."""
        t = self.telemetry
        with self._lock:
            if self.flight is None:
                return None
            path, image = self.flight.image_path(shot.extension)
            try:
                shot.save(path)
            except Exception as exc:
                self.save_errors += 1
                log.error("picture %s not saved: %s", image, exc)
                return None
            gps = t.gps_now(now_ns) or {}
            self.flight.add_frame(image, shot, {"gps_fix": gps.get("fix"), "gps_sats": gps.get("sats"),
                                                "gps_hdop": gps.get("hdop"), "rangefinder_m": t.rangefinder_now(now_ns),
                                                "armed": t.armed, "mode": t.mode_name()})
            return image

    def poll(self, now_ns: int) -> bool:
        """One turn of the recording's work, for a caller without threads. True while recording."""
        self._step(now_ns)
        self._drain()
        return self.active

    def close(self, now_ns: int, reason: str) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
            self._thread = None
        self._drain()
        self._close(now_ns, reason)

    # --- the recording's own thread -------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._step(self.clock())
                self._drain()
            except Exception:
                log.exception("recording")
            self._stop.wait(STEP_S)

    def _drain(self) -> None:
        while True:
            try:
                msg, arrival = self._messages.get_nowait()
            except queue.Empty:
                return
            with self._lock:
                if self.flight is not None:
                    self.flight.log_message(msg, arrival)

    def _step(self, now_ns: int) -> None:
        """Open or close the flight as the rules say, write the rows that are ready."""
        t = self.telemetry
        if now_ns >= self._next_disk:
            self.root.mkdir(parents=True, exist_ok=True)
            self._free = self.free_mb(self.root)
            self._next_disk = now_ns + DISK_CHECK_NS
        want, why = recording_wanted(link_alive=t.link_alive(now_ns), armed=t.armed, rel_alt_m=t.rel_alt_now(now_ns),
                                     start_height_m=self.settings.start_height_m, free_mb=self._free,
                                     min_free_mb=self.settings.min_free_mb, recording=self.active)
        if want and not self.active:
            self._open(now_ns)
        elif not want and self.active:
            self._close(now_ns, why)
        if why != self.why:
            if why:
                log.info("not recording: %s", why)
            self.why = why
        with self._lock:
            if self.flight is not None:
                self.flight.finalize(t, now_ns)

    def _open(self, now_ns: int) -> None:
        t = self.telemetry
        boot = t.boot_ms_at(now_ns)
        utc = t.utc_at(boot) if boot is not None else None
        stamp = (utc[:19].replace("T", "_").replace(":", "-") + "Z") if utc \
            else datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
        settings = self.settings.as_dict()
        settings["interval_s"] = self.interval_s           # vama_nav's: one camera, one rate
        info = {"settings": settings, "settings_file": self.settings.source, "camera": self.describe_camera(),
                "flight_controller": list(t.autopilot or ()), "vehicle_type": t.vehicle_type, "started_utc": utc,
                "clock_offset_ms_start": t.offset_ms(), "written_by": f"vama_nav --record (recorder {recorder_version})",
                "frames_csv_time": "local_ns is the camera's own frame time on this computer's monotonic clock; "
                                   "ap_boot_ms is that moment on the flight controller's clock; the row's state is "
                                   "interpolated there"}
        with self._lock:
            self.flight = Flight(self.root, Flight.next_name(self.root, stamp), info)
            name = self.flight.folder.name
        self.active = True
        log.info("flight %s started (height %s m)", name, t.rel_alt_now(now_ns))

    def _close(self, now_ns: int, reason: str) -> None:
        t = self.telemetry
        self.active = False
        with self._lock:
            flight, self.flight = self.flight, None
            if flight is None:
                return
            flight.close(t, now_ns, reason, {"clock_offset_ms_end": t.offset_ms(), "controller_restarts": t.clock.restarts,
                                             "messages_heard": dict(t.counts), "save_errors": self.save_errors,
                                             "messages_not_logged": self.dropped})
        self.flights_closed += 1
        log.info("flight %s ended: %s, %d pictures", flight.folder.name, reason, flight.frames)
