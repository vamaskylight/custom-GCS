"""The recorder: pictures while the drone flies, each with the drone's state at its own moment.

A flight starts when the drone is armed and above start_height_m (as soon as it is armed with 0),
and ends when it disarms, when the link to the flight controller is lost for 5 s, or when the card
is nearly full. Each flight is one folder (see flight.py).
"""

from __future__ import annotations

import logging
import math
import os
import re
import shutil
import subprocess
import threading
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

from . import __version__
from .flight import Flight
from .settings import Settings
from .telemetry import Telemetry

log = logging.getLogger("vama_recorder")

DISK_CHECK_NS = 5_000_000_000
STATUS_EVERY_NS = 10_000_000_000
GPS_AFTER_ARMING_NS = 2_000_000_000     # the GPS is judged this long after arming, once its messages are in

# GPS_RAW_INT's fix_type: 0 is MAVLink's GPS_FIX_TYPE_NO_GPS, no receiver at all, not one that still waits for
# satellites (1). The client's first flight (2026-11-03) had fix 0 from start to end, after a bench check that
# called it "no 3D fix yet", which is normal indoors: its pictures got no true positions.
GPS_FIX_NO_GPS = 0
GPS_FIX_3D = 3

GPS_RECEIVER_HELP = ("Check the GPS cable, GPS1_TYPE 1 (auto; GPS_TYPE before ArduPilot 4.6), and that the GPS "
                     "port's SERIALn_PROTOCOL is 5 (GPS), then restart the flight controller")


def disk_free_mb(folder: Path) -> int | None:
    try:
        return int(shutil.disk_usage(folder).free // (1024 * 1024))
    except OSError:
        return None


def recording_wanted(*, link_alive: bool, armed: bool, rel_alt_m: float | None, start_height_m: float,
                     free_mb: int | None, min_free_mb: int, recording: bool) -> tuple[bool, str]:
    """Whether to take pictures now, and if not, why not (in plain words, for the log)."""
    if not link_alive:
        return False, "no heartbeat from the flight controller"
    if not armed:
        return False, "disarmed"
    if free_mb is not None and free_mb < min_free_mb:
        return False, f"the card is nearly full ({free_mb} MB free)"
    if recording or start_height_m <= 0:
        return True, ""
    if rel_alt_m is None:
        return False, "the height is not known yet"
    if rel_alt_m < start_height_m:
        return False, f"below {start_height_m:g} m above home"
    return True, ""


def gps_check_line(gps: dict | None) -> tuple[bool, str]:
    """The bench check's line on the GPS, and True when it is a problem."""
    if gps is None:
        return False, "NOTE    GPS: no GPS_RAW_INT heard (see its line above)"
    fix, sats = gps.get("fix"), gps.get("sats")
    if fix == GPS_FIX_NO_GPS:
        return True, ("PROBLEM GPS: the flight controller sees no GPS receiver at all (fix 0, not only no fix yet). "
                      "A flight like that has no true positions and is no use for the navigation. "
                      f"{GPS_RECEIVER_HELP}")
    if fix is not None and fix >= GPS_FIX_3D:
        return False, f"OK      GPS: fix {fix}, {sats} satellites"
    return False, (f"NOTE    GPS: the receiver is there, no 3D fix yet (fix {fix}, {sats} satellites). "
                   "Indoors that is normal; outdoors it comes after a minute or two")


def armed_gps_warning(gps: dict | None) -> str:
    """What the log says when the drone was armed without a GPS ready to give positions, else ""."""
    if gps is None:
        return "armed, but the flight controller sends no GPS_RAW_INT: its GPS is not known"
    fix, sats = gps.get("fix"), gps.get("sats")
    if fix == GPS_FIX_NO_GPS:
        return ("armed with NO GPS RECEIVER (fix 0): the flight controller sees no GPS, so the pictures of this "
                f"flight get no true positions and are no use for the navigation. {GPS_RECEIVER_HELP}, and run "
                "the bench check before the next flight (RECORDER.md part 7)")
    if fix is None or fix < GPS_FIX_3D:
        return (f"armed without a 3D GPS fix (fix {fix}, {sats} satellites): the pictures' positions are not sure "
                "until it has one. Wait for the fix before take-off")
    return ""


def _default_link_factory(*args, **kwargs):
    from .link import Link
    return Link(*args, **kwargs)


class Recorder:
    def __init__(self, settings: Settings, camera, link_factory: Callable | None = None,
                 clock: Callable[[], int] = time.monotonic_ns,
                 free_mb: Callable[[Path], int | None] = disk_free_mb, stop_after_flights: int = 0) -> None:
        self.settings = settings
        self.camera = camera
        self.clock = clock
        self.telemetry = Telemetry()
        self._link_factory = link_factory or _default_link_factory
        self._free_mb = free_mb
        self.stop_after_flights = int(stop_after_flights)
        self.stop_event = threading.Event()
        self.link = None
        self.flight: Flight | None = None
        self._flight_lock = threading.Lock()
        self.flights_closed = 0
        self.camera_errors = 0
        self.save_errors = 0
        self.why_not = ""
        self.frames_without_3d_fix = 0        # in the flight now
        self._was_armed = False
        self._gps_check_at: int | None = None

    # --- the link's thread ------------------------------------------------------------------

    def _on_message(self, msg, arrival_ns: int) -> None:
        if self.telemetry.hear(msg, arrival_ns):
            with self._flight_lock:
                if self.flight is not None:
                    self.flight.log_message(msg, arrival_ns)

    # --- the main thread --------------------------------------------------------------------

    def run(self) -> int:
        self.begin()
        try:
            while not self.stop_event.is_set():
                if not self.step(self.clock()):
                    break
                self.stop_event.wait(0.02)
            return 0
        finally:
            self.end()

    def begin(self) -> None:
        """Start the camera and the link. ``step`` then does the work, ``end`` stops it all."""
        self._root = self.settings.flights_dir
        self._root.mkdir(parents=True, exist_ok=True)
        self._next_shot = self._next_disk = self._next_status = 0
        self._free = None
        self._counts_before: dict[str, int] = {}
        self.camera.start()
        try:
            self.link = self._link_factory(self.settings, self._on_message, lambda: self.telemetry.autopilot,
                                           self.clock)
            self.link.open()
            self.link.start()
        except BaseException:
            self.camera.stop()
            raise
        log.info("recorder %s started: camera %s, link %s at %s, flights in %s", __version__,
                 self.camera.describe().get("model"), self.settings.device, self.settings.baud, self._root)

    def end(self) -> None:
        try:
            if self.flight is not None:
                self._close_flight(self.clock(), "the recorder stopped")
        finally:
            if self.link is not None:
                self.link.stop()
            self.camera.stop()

    def step(self, now: int) -> bool:
        """One turn of the loop at ``now``. False when the recorder should stop (--flights reached)."""
        s = self.settings
        t = self.telemetry
        if now >= self._next_disk:
            self._free = self._free_mb(self._root)
            self._next_disk = now + DISK_CHECK_NS
        self._watch_arming(now)
        want, why = recording_wanted(link_alive=t.link_alive(now), armed=t.armed, rel_alt_m=t.rel_alt_now(now),
                                     start_height_m=s.start_height_m, free_mb=self._free,
                                     min_free_mb=s.min_free_mb, recording=self.flight is not None)
        if want and self.flight is None:
            self._open_flight(self._root, now)
            self._next_shot = now
        elif not want and self.flight is not None:
            self._close_flight(now, why)
            if self.stop_after_flights and self.flights_closed >= self.stop_after_flights:
                return False
        if why != self.why_not:
            if why:
                log.info("not recording: %s", why)
            self.why_not = why
        if self.flight is not None:
            if now >= self._next_shot:
                self._take(now)
                self._next_shot += int(s.interval_s * 1e9)
                if self._next_shot <= now:           # fell behind (a slow save): no burst of pictures
                    self._next_shot = now + int(s.interval_s * 1e9)
            self.flight.finalize(t, self.clock())
        if now >= self._next_status:
            if self._next_status:
                self._counts_before = self._status(now, self._counts_before, self._free)
            else:
                self._counts_before = dict(t.counts)
            self._next_status = now + STATUS_EVERY_NS
        return True

    def _watch_arming(self, now: int) -> None:
        """Once in each arming, a little after it: a warning in the log when the GPS cannot give positions."""
        armed = bool(self.telemetry.armed) and self.telemetry.link_alive(now)
        if armed and not self._was_armed:
            self._gps_check_at = now + GPS_AFTER_ARMING_NS
        elif not armed:
            self._gps_check_at = None
        self._was_armed = armed
        if self._gps_check_at is not None and now >= self._gps_check_at:
            self._gps_check_at = None
            warning = armed_gps_warning(self.telemetry.gps_now(now))
            if warning:
                log.warning(warning)

    def _open_flight(self, root: Path, now: int) -> None:
        t = self.telemetry
        boot = t.boot_ms_at(now)
        utc = t.utc_at(boot) if boot is not None else None
        if utc:
            stamp = utc[:19].replace("T", "_").replace(":", "-") + "Z"
        else:
            stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        info = {"settings": self.settings.as_dict(), "settings_file": self.settings.source,
                "camera": self.camera.describe(), "flight_controller": list(t.autopilot or ()),
                "vehicle_type": t.vehicle_type, "started_utc": utc, "clock_offset_ms_start": t.offset_ms(),
                "frames_csv_time": "local_ns is the camera's own frame time (libcamera SensorTimestamp) on this "
                                   "computer's monotonic clock; ap_boot_ms is that moment on the flight "
                                   "controller's clock; the row's state is interpolated there"}
        with self._flight_lock:
            self.flight = Flight(root, Flight.next_name(root, stamp), info)
        self.frames_without_3d_fix = 0
        gps = t.gps_now(now) or {}
        log.info("flight %s started (height %s m, GPS fix %s with %s satellites)", self.flight.folder.name,
                 t.rel_alt_now(now), gps.get("fix"), gps.get("sats"))

    def _close_flight(self, now: int, reason: str) -> None:
        t = self.telemetry
        with self._flight_lock:
            flight, self.flight = self.flight, None
            if flight is None:
                return
            flight.close(t, now, reason, {"clock_offset_ms_end": t.offset_ms(), "controller_restarts": t.clock.restarts,
                                          "messages_heard": dict(t.counts), "camera_errors": self.camera_errors,
                                          "save_errors": self.save_errors,
                                          "frames_without_3d_gps_fix": self.frames_without_3d_fix})
        self.flights_closed += 1
        log.info("flight %s ended: %s, %d pictures", flight.folder.name, reason, flight.frames)
        if self.frames_without_3d_fix:
            log.warning("flight %s: %d of its %d pictures were taken without a 3D GPS fix: their positions are not "
                        "true", flight.folder.name, self.frames_without_3d_fix, flight.frames)

    def _take(self, now: int) -> None:
        t = self.telemetry
        try:
            shot = self.camera.capture()
        except Exception as exc:
            self.camera_errors += 1
            log.error("camera: %s", exc)
            return
        gps = t.gps_now(now) or {}
        snapshot = {"gps_fix": gps.get("fix"), "gps_sats": gps.get("sats"), "gps_hdop": gps.get("hdop"),
                    "rangefinder_m": t.rangefinder_now(now), "armed": t.armed, "mode": t.mode_name()}
        path, image = self.flight.image_path(shot.extension)
        try:
            shot.save(path)
        except Exception as exc:
            self.save_errors += 1
            log.error("picture %s not saved: %s", image, exc)
            return
        self.flight.add_frame(image, shot, snapshot)
        if (gps.get("fix") or 0) < GPS_FIX_3D:
            self.frames_without_3d_fix += 1

    def _status(self, now: int, before: dict[str, int], free: int | None) -> dict[str, int]:
        t = self.telemetry
        counts = dict(t.counts)
        rate = {k: (counts.get(k, 0) - before.get(k, 0)) / (STATUS_EVERY_NS / 1e9)
                for k in ("ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT")}
        rates = ", ".join(f"{k} {v:.1f}/s" for k, v in rate.items())
        offset = t.offset_ms()
        offset_text = "-" if offset is None else f"{offset:.1f} ms"
        if self.flight is not None:
            log.info("recording %s: %d pictures, %d rows, clock offset %s, %s, %s MB free",
                     self.flight.folder.name, self.flight.frames, self.flight.rows, offset_text, rates, free)
        else:
            gps = t.gps_now(now) or {}
            log.info("waiting (%s): height %s, GPS fix %s with %s satellites, %s", self.why_not or "-",
                     t.rel_alt_now(now), gps.get("fix"), gps.get("sats"), rates)
        return counts


# --- the bench check --------------------------------------------------------------------------

def port_hint(device: str) -> str:
    """A word on the serial port when it is likely the wrong one, else "". On a Raspberry Pi 5, /dev/serial0
    is the 3-pin debug connector (ttyAMA10), not pins 8 and 10, and pins 8 and 10 (/dev/ttyAMA0) need an
    overlay (the client's bench check, 2026-10-30: no heartbeat on /dev/serial0)."""
    if os.path.basename(os.path.realpath(device)) == "ttyAMA10":
        return (f" {device} is the Raspberry Pi 5's small 3-pin debug connector, not pins 8 and 10: add the line "
                "dtoverlay=uart0-pi5 to /boot/firmware/config.txt, reboot, and set device = /dev/ttyAMA0 "
                "(RECORDER.md part 3).")
    if device == "/dev/ttyAMA0" and not os.path.exists(device):
        return (" On a Raspberry Pi 5, pins 8 and 10 are /dev/ttyAMA0 only with the line dtoverlay=uart0-pi5 in "
                "/boot/firmware/config.txt, and a reboot (RECORDER.md part 3).")
    return ""


def service_running() -> bool:
    """Whether the recorder's own service runs. It then holds the camera and the serial port, and a check beside
    it reports a missing camera and a dead link that are neither (the client's bench check, 2026-10-31)."""
    if shutil.which("systemctl") is None:
        return False
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", "vama-recorder"], timeout=10).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


# Without a heartbeat, what arrives on the port tells a dead wire from a port that carries something else. On the
# client's Brahma H7 (2026-11-04) the Pi's link worked only while SERIAL4 was MAVLink and the GPS only while it was
# GPS: the Pi and the GPS shared one port, and "no heartbeat" alone pointed at the wires.
LISTEN_BAUDS = (230400, 57600, 38400, 9600, 460800, 921600, 115200)   # after the recorder's own; 230400: ArduPilot's u-blox
LISTEN_S = 2.5                           # ArduPilot asks a u-blox GPS something about every 2 s once it is set up
SERIAL_BAUD_CODES = {9600: 9, 19200: 19, 38400: 38, 57600: 57, 115200: 115, 230400: 230, 460800: 460, 921600: 921}
NETWORK_DEVICES = ("tcp:", "tcpin:", "udp:", "udpin:", "udpout:", "udpbcast:")
_NMEA = re.compile(rb"\$([A-Z]{5}|PUBX),[^$*\r\n]{0,79}\*([0-9A-F]{2})")


def read_port(device: str, baud: int, seconds: float) -> bytes:
    """What arrives on a serial port in so many seconds at that speed. It sends nothing."""
    import serial  # pyserial, as pymavlink's own serial link

    data = bytearray()
    with serial.Serial(device, baudrate=baud, timeout=0.1) as port:
        port.reset_input_buffer()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            data += port.read(4096)
    return bytes(data)


def _ubx_frames(data: bytes) -> int:
    """Whole u-blox UBX frames in the bytes: B5 62, class, id, length, payload, a right checksum."""
    count, i = 0, data.find(b"\xb5\x62")
    while 0 <= i <= len(data) - 8:
        end = i + 6 + (data[i + 4] | data[i + 5] << 8)
        if end + 2 <= len(data):
            a = b = 0
            for byte in data[i + 2:end]:
                a = (a + byte) & 0xFF
                b = (b + a) & 0xFF
            if data[end] == a and data[end + 1] == b:
                count += 1
                i = data.find(b"\xb5\x62", end + 2)
                continue
        i = data.find(b"\xb5\x62", i + 1)
    return count


def _nmea_sentences(data: bytes) -> int:
    """NMEA sentences with a right checksum: a GPS's $GPRMC and the like, and u-blox's $PUBX, which ArduPilot sends to
    set a GPS's speed."""
    count = 0
    for m in _NMEA.finditer(data):
        x = 0
        for byte in m.group(0)[1:-3]:
            x ^= byte
        count += x == int(m.group(2), 16)
    return count


def _mavlink_messages(data: bytes) -> int:
    """MAVLink 1 or 2 messages of a known kind with a right checksum, tried at every start byte. pymavlink's own
    stream parser is not used: after a bad frame it skips that frame's whole length (a true frame behind noise is
    lost), and it hands out a message of an unknown kind without a checksum check (noise looks like MAVLink)."""
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2

    parser = mavlink2.MAVLink(None)
    count = free_from = 0
    for marker in re.finditer(rb"[\xfd\xfe]", data):
        i = marker.start()
        if i < free_from or i + 8 > len(data):
            continue
        if data[i] == 0xFE:
            size = data[i + 1] + 8                           # v1: 6 header bytes, the payload, 2 checksum bytes
        elif data[i + 2] in (0, 1):
            size = data[i + 1] + 12 + 13 * data[i + 2]       # v2: 10 header bytes, the payload, 2, a signature
        else:
            continue
        if i + size > len(data):
            continue
        try:
            msg = parser.decode(bytearray(data[i:i + size]))
        except Exception:  # noqa: BLE001, S112 (noise fails pymavlink's decode in many ways: checksum, length, fields)
            continue
        if msg.get_msgId() in mavlink2.mavlink_map:
            count += 1
            free_from = i + size
    return count


def port_traffic(data: bytes) -> str:
    """What bytes from a serial port are: "nothing", "ubx" or "nmea" (GPS data), "mavlink", or "other". MAVLink needs
    two messages: one in noise passes its 16-bit checksum by chance about once in 65536 tries, and a flight
    controller sends at least a heartbeat a second on every MAVLink port."""
    if not data:
        return "nothing"
    if _ubx_frames(data):
        return "ubx"
    if _nmea_sentences(data):
        return "nmea"
    if _mavlink_messages(data) >= 2:
        return "mavlink"
    return "other"


def what_the_port_carries(device: str, baud: int, read: Callable[[str, int, float], bytes] = read_port,
                          seconds: float = LISTEN_S) -> tuple[str, int | None]:
    """What arrives on a serial port, at the recorder's speed first and then at the usual others, until one shows
    GPS data or MAVLink: (what, at which speed). ("unknown", None) for a network link or a port that cannot be read."""
    if device.lower().startswith(NETWORK_DEVICES):
        return "unknown", None
    read_once = data_seen = False
    for b in [baud] + [x for x in LISTEN_BAUDS if x != baud]:
        try:
            data = read(device, b, seconds)
        except Exception as exc:  # noqa: BLE001 (no pyserial, a speed the port cannot do; pyserial lets termios.error out)
            log.debug("listening on %s at %s: %s", device, b, exc)
            continue
        read_once = True
        kind = port_traffic(data)
        if kind not in ("nothing", "other"):
            return kind, b
        data_seen = data_seen or kind == "other"
    if not read_once:
        return "unknown", None
    return ("other" if data_seen else "nothing"), None


def no_heartbeat_line(device: str, baud: int, kind: str, found_at: int | None, waited_s: float) -> str:
    """The bench check's PROBLEM line when no heartbeat came, from what the port carries (what_the_port_carries)."""
    head = f"PROBLEM link: no heartbeat from the flight controller on {device} at {baud}"
    code = SERIAL_BAUD_CODES.get(baud)
    speed = f"SERIALn_BAUD {code}" if code else f"the speed of baud = {baud}"
    if kind in ("ubx", "nmea"):
        return (f"{head}. The port carries GPS data ({'u-blox' if kind == 'ubx' else 'NMEA'} at {found_at}), not MAVLink: "
                "the flight controller uses this port for a GPS (SERIALn_PROTOCOL 5), or the Pi shares the GPS's wires. "
                "The Pi needs a port of its own: keep the GPS's port at 5, and wire the Pi to a free port with "
                "SERIALn_PROTOCOL 2 (RECORDER.md part 4)")
    if kind == "mavlink" and found_at != baud:
        fix = f"Set that port's SERIALn_BAUD to {code} on the flight controller, or" if code else "Set"
        return f"{head}, but MAVLink arrives on this port at {found_at}. {fix} baud = {found_at} in the recorder's settings"
    if kind == "mavlink":
        return (f"{head} in {waited_s:g} s, but MAVLink arrives now. If the flight controller was still starting, run "
                "the check again. If not, the Pi is on another MAVLink device, not on the flight controller")
    if kind == "nothing":
        return (f"{head}, and nothing at all arrives on the port, at any usual speed. Check the wires (the Pi's TX, "
                "pin 8, to the flight controller's RX, the Pi's RX, pin 10, to its TX, and ground), that the flight "
                "controller is on, and that its port has SERIALn_PROTOCOL 2 (MAVLink 2): a port at -1 is off, and a "
                "port set for a GPS (5) can be silent too")
    if kind == "other":
        return (f"{head}. Data arrives, but it is not MAVLink at any usual speed. Check that the port has "
                f"SERIALn_PROTOCOL 2 (MAVLink 2, not GPS) and {speed}, and that no other device shares its wires")
    return (f"{head}. Check the wires (Pi TX to the Cube's RX, Pi RX to the Cube's TX, and ground), and on the Cube "
            "that port's SERIALn_PROTOCOL 2 (MAVLink 2, not GPS) and SERIALn_BAUD")


def check(settings: Settings, camera, link_factory: Callable | None = None, listen_s: float = 6.0,
          out: Callable[[str], None] = print, clock: Callable[[], int] = time.monotonic_ns,
          heartbeat_wait_s: float = 10.0, settle_s: float = 1.5,
          read: Callable[[str, int, float], bytes] = read_port, port_listen_s: float = LISTEN_S) -> int:
    """Check the camera, the card and the link on the ground. 0 when all is well. Without a heartbeat it listens to
    what arrives on the port (``read``, ``port_listen_s`` at each speed) to say what is wrong."""
    from .link import STREAMS_HZ

    problems = 0
    out(f"VAMA recorder {__version__}, settings: {settings.source}")
    if service_running():
        out("PROBLEM service: the recorder service is running, and it holds the camera and the serial port. "
            "Run: sudo systemctl stop vama-recorder, then this check again, then: sudo systemctl start vama-recorder")
        return 1
    root = settings.flights_dir
    try:
        root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        out(f"PROBLEM folder {root}: {exc}")
        return 1
    try:
        camera.start()
        d = camera.describe()
        size = d.get("picture_size") or []
        out(f"OK      camera: {d.get('model') or d.get('kind')}, picture {' x '.join(str(v) for v in size)}")
        shot = camera.capture()
        path = root / f"check.{shot.extension}"
        shot.save(path)
        out(f"OK      a test picture: {path} ({path.stat().st_size // 1024} kB)")
    except Exception as exc:
        problems += 1
        out(f"PROBLEM camera: {exc}")
    finally:
        try:
            camera.stop()
        except Exception:
            pass
    free = disk_free_mb(root)
    if free is None or free < settings.min_free_mb:
        problems += 1
        out(f"PROBLEM card: {free} MB free in {root}, the recorder needs more than {settings.min_free_mb} MB")
    else:
        out(f"OK      card: {free} MB free in {root}")

    telemetry = Telemetry()
    factory = link_factory or _default_link_factory
    link = factory(settings, telemetry.hear, lambda: telemetry.autopilot, clock)
    try:
        link.open()
    except Exception as exc:
        out(f"PROBLEM link: {str(exc).rstrip('.')}.{port_hint(settings.device)}")
        return problems + 1
    link.start()
    try:
        end = time.monotonic() + heartbeat_wait_s
        while telemetry.autopilot is None and time.monotonic() < end:
            time.sleep(0.1)
        if telemetry.autopilot is None:
            link.stop()                  # the port is free again for the listening
            if not settings.device.lower().startswith(NETWORK_DEVICES):
                speeds = len({settings.baud, *LISTEN_BAUDS})
                out(f"NOTE    link: no heartbeat in {heartbeat_wait_s:g} s, listening to what arrives on "
                    f"{settings.device} at {speeds} speeds (up to {math.ceil(speeds * port_listen_s)} s)")
            kind, found_at = what_the_port_carries(settings.device, settings.baud, read=read, seconds=port_listen_s)
            out(f"{no_heartbeat_line(settings.device, settings.baud, kind, found_at, heartbeat_wait_s)}."
                f"{port_hint(settings.device)}")
            return problems + 1
        out(f"OK      flight controller: system {telemetry.autopilot[0]}, component {telemetry.autopilot[1]}")
        time.sleep(settle_s)                # the stream request goes out and the rates change
        before = dict(telemetry.counts)
        time.sleep(listen_s)
        after = dict(telemetry.counts)
        names = {30: "ATTITUDE", 33: "GLOBAL_POSITION_INT", 24: "GPS_RAW_INT", 2: "SYSTEM_TIME"}
        for msg_id, name in names.items():
            got = (after.get(name, 0) - before.get(name, 0)) / listen_s
            wanted = STREAMS_HZ[msg_id]
            if got < 0.6 * wanted:
                problems += 1
                out(f"PROBLEM {name}: {got:.1f} a second, the recorder asked for {wanted:g}. "
                    "A slow link (SERIALn_BAUD) or a busy port")
            else:
                out(f"OK      {name}: {got:.1f} a second")
        now = clock()
        gps_problem, gps_line = gps_check_line(telemetry.gps_now(now))
        out(gps_line)
        if gps_problem:
            problems += 1
        offset = telemetry.offset_ms()
        out(f"OK      clock: the flight controller's time is known (offset {offset:.1f} ms)" if offset is not None
            else "PROBLEM clock: no time from the flight controller")
        if offset is None:
            problems += 1
        out(f"NOTE    the drone is {'ARMED' if telemetry.armed else 'disarmed'}, mode {telemetry.mode_name()}, "
            f"height above home {telemetry.rel_alt_now(now)} m")
    finally:
        link.stop()
    out("All checks passed." if problems == 0 else f"{problems} problem(s): see the PROBLEM lines.")
    return 0 if problems == 0 else 1
