"""What the flight controller says, kept for some seconds, so each picture gets the drone's state at
its own moment: position, heights, speed and heading from GLOBAL_POSITION_INT, the angles from
ATTITUDE, both interpolated between the controller's own samples (see clock.py)."""

from __future__ import annotations

import bisect
import math
import threading
from datetime import UTC, datetime

from .clock import NS_PER_MS, AutopilotClock

MAV_AUTOPILOT_INVALID = 8
MAV_COMP_ID_AUTOPILOT1 = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_SENSOR_ROTATION_PITCH_270 = 25        # a rangefinder looking down

KEEP_MS = 20_000                          # samples older than this behind the newest go
LINK_LOST_NS = 5_000_000_000              # no heartbeat from the controller for this long: no link
FRESH_NS = 3_000_000_000                  # a height or GPS older than this is not "now"
RANGEFINDER_PREFER_NS = 2_000_000_000     # DISTANCE_SENSOR this recent: ignore the older RANGEFINDER message

# ArduCopter's mode numbers (custom_mode), for the rows. Other numbers are written as "mode N".
COPTER_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL",
                7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
                17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD",
                23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE"}

# Indexes of angles among a sample's values (after its time), interpolated the short way round.
_ATTITUDE_ANGLES = (0, 2)                 # roll, yaw (pitch stays within +-90)
_POSITION_ANGLES = (7,)                   # heading


def _interpolate(samples: list[tuple], t: float, angles: tuple[int, ...]) -> tuple[tuple | None, str, float | None]:
    """(values at t, "interpolated" or "held" or "none", gap in ms).

    interpolated: between two samples, the gap is the time between them.
    held: t is outside the samples, the nearest is taken, the gap is how far it is from t.
    """
    if not samples:
        return None, "none", None
    i = bisect.bisect_left(samples, t, key=lambda s: s[0])
    if i < len(samples) and samples[i][0] == t:
        return samples[i][1:], "interpolated", 0.0
    if i == 0:
        return samples[0][1:], "held", float(samples[0][0] - t)
    if i == len(samples):
        return samples[-1][1:], "held", float(t - samples[-1][0])
    a, b = samples[i - 1], samples[i]
    span = float(b[0] - a[0])
    f = (t - a[0]) / span if span > 0 else 0.0
    out = []
    for k, (va, vb) in enumerate(zip(a[1:], b[1:], strict=True)):
        if va is None or vb is None:
            out.append(va if f < 0.5 else vb)
        elif k in angles:
            out.append(va + f * (((vb - va + 180.0) % 360.0) - 180.0))
        else:
            out.append(va + f * (vb - va))
    return tuple(out), "interpolated", span


def _pm180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


class Telemetry:
    """Thread safe: the link's reader thread calls hear(), the recorder reads."""

    def __init__(self, clock: AutopilotClock | None = None) -> None:
        self.clock = clock or AutopilotClock()
        self._lock = threading.Lock()
        self._attitude: list[tuple] = []      # (boot_ms, roll, pitch, yaw) degrees
        self._position: list[tuple] = []      # (boot_ms, lat, lon, alt_msl, rel_alt, vn, ve, vd, heading)
        self.autopilot: tuple[int, int] | None = None
        self.vehicle_type: int | None = None
        self.armed = False
        self.mode: int | None = None
        self.heartbeat_ns: int | None = None
        self.gps: dict | None = None
        self.rangefinder_m: float | None = None
        self.rangefinder_ns: int | None = None
        self._distance_down_ns: int | None = None
        self._utc: tuple[int, int] | None = None   # (unix_usec, boot_ms) from SYSTEM_TIME
        self.counts: dict[str, int] = {}

    # --- what arrives -----------------------------------------------------------------------

    def hear(self, msg, arrival_ns: int) -> bool:
        """One MAVLink message and when it arrived (ns). True when it was the flight controller's."""
        kind = msg.get_type()
        src = (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
        with self._lock:
            if kind == "HEARTBEAT" and self.autopilot is None:
                if (int(getattr(msg, "autopilot", MAV_AUTOPILOT_INVALID)) != MAV_AUTOPILOT_INVALID
                        and src[1] == MAV_COMP_ID_AUTOPILOT1):
                    self.autopilot = src
            if self.autopilot is None or src != self.autopilot:
                return False
            self.counts[kind] = self.counts.get(kind, 0) + 1
            if kind == "HEARTBEAT":
                self.heartbeat_ns = int(arrival_ns)
                self.armed = bool(int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = int(msg.custom_mode)
                self.vehicle_type = int(msg.type)
            elif kind == "ATTITUDE":
                t = int(msg.time_boot_ms)
                self.clock.hear(t, arrival_ns)
                self._insert(self._attitude, (t, _pm180(math.degrees(msg.roll)), math.degrees(msg.pitch),
                                              math.degrees(msg.yaw) % 360.0))
            elif kind == "GLOBAL_POSITION_INT":
                t = int(msg.time_boot_ms)
                self.clock.hear(t, arrival_ns)
                heading = None if int(msg.hdg) == 65535 else int(msg.hdg) / 100.0
                self._insert(self._position, (t, msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0,
                                              msg.relative_alt / 1000.0, msg.vx / 100.0, msg.vy / 100.0,
                                              msg.vz / 100.0, heading))
            elif kind == "GPS_RAW_INT":
                eph = int(msg.eph)
                sats = int(msg.satellites_visible)
                self.gps = {"fix": int(msg.fix_type), "sats": None if sats == 255 else sats,
                            "hdop": None if eph == 65535 else eph / 100.0, "lat": msg.lat / 1e7,
                            "lon": msg.lon / 1e7, "alt_msl_m": msg.alt / 1000.0, "ns": int(arrival_ns)}
            elif kind == "SYSTEM_TIME":
                t = int(msg.time_boot_ms)
                self.clock.hear(t, arrival_ns)
                if int(msg.time_unix_usec) > 0:
                    self._utc = (int(msg.time_unix_usec), t)
            elif kind == "DISTANCE_SENSOR":
                if int(msg.orientation) == MAV_SENSOR_ROTATION_PITCH_270:
                    cm, low, high = int(msg.current_distance), int(msg.min_distance), int(msg.max_distance)
                    good = 0 < cm < 65535 and cm >= low and (high <= 0 or cm <= high)
                    self.rangefinder_m = cm / 100.0 if good else None
                    self.rangefinder_ns = self._distance_down_ns = int(arrival_ns)
            elif kind == "RANGEFINDER":
                if self._distance_down_ns is None or arrival_ns - self._distance_down_ns > RANGEFINDER_PREFER_NS:
                    d = float(msg.distance)
                    self.rangefinder_m = d if d > 0 else None
                    self.rangefinder_ns = int(arrival_ns)
            return True

    @staticmethod
    def _insert(samples: list[tuple], sample: tuple) -> None:
        if samples and sample[0] < samples[-1][0] - AutopilotClock.RESTART_MS:
            samples.clear()               # the controller restarted: its clock began again
        bisect.insort(samples, sample, key=lambda s: s[0])
        newest = samples[-1][0]
        cut = bisect.bisect_left(samples, newest - KEEP_MS, key=lambda s: s[0])
        if cut:
            del samples[:cut]

    # --- what the recorder reads ------------------------------------------------------------

    def boot_ms_at(self, local_ns: int) -> float | None:
        with self._lock:
            return self.clock.boot_ms(local_ns)

    def offset_ms(self) -> float | None:
        with self._lock:
            offset = self.clock.offset_ns
            return None if offset is None else offset / NS_PER_MS

    def covered(self, boot_ms: float) -> bool:
        """True when both streams have a sample at or after ``boot_ms``: its state can be interpolated."""
        with self._lock:
            return bool(self._attitude and self._position and self._attitude[-1][0] >= boot_ms
                        and self._position[-1][0] >= boot_ms)

    def state_at(self, boot_ms: float) -> dict:
        """The drone at the controller's time ``boot_ms``: position, heights, speed, heading, angles."""
        with self._lock:
            att, att_how, att_gap = _interpolate(self._attitude, boot_ms, _ATTITUDE_ANGLES)
            pos, pos_how, pos_gap = _interpolate(self._position, boot_ms, _POSITION_ANGLES)
        out: dict = {"position_how": pos_how, "position_gap_ms": pos_gap,
                     "attitude_how": att_how, "attitude_gap_ms": att_gap}
        if pos is not None:
            lat, lon, alt, rel, vn, ve, vd, heading = pos
            out.update(lat=lat, lon=lon, alt_msl_m=alt, rel_alt_m=rel, vn_mps=vn, ve_mps=ve, vd_mps=vd,
                       heading_deg=None if heading is None else heading % 360.0)
        if att is not None:
            roll, pitch, yaw = att
            out.update(roll_deg=_pm180(roll), pitch_deg=pitch, yaw_deg=yaw % 360.0)
        return out

    def utc_at(self, boot_ms: float) -> str | None:
        """The UTC time at the controller's ``boot_ms``, from its GPS time (SYSTEM_TIME), or None."""
        with self._lock:
            pair = self._utc
        if pair is None:
            return None
        usec = pair[0] + (float(boot_ms) - pair[1]) * 1000.0
        stamp = datetime.fromtimestamp(usec / 1e6, tz=UTC)
        return stamp.isoformat(timespec="milliseconds").replace("+00:00", "Z")

    def link_alive(self, now_ns: int) -> bool:
        with self._lock:
            return self.heartbeat_ns is not None and now_ns - self.heartbeat_ns <= LINK_LOST_NS

    def rel_alt_now(self, now_ns: int) -> float | None:
        """The drone's height above home now, or None when not known or old."""
        with self._lock:
            if not self._position:
                return None
            newest_local = self.clock.local_ns(self._position[-1][0])
            if newest_local is None or now_ns - newest_local > FRESH_NS:
                return None
            return self._position[-1][4]

    def gps_now(self, now_ns: int) -> dict | None:
        with self._lock:
            if self.gps is None or now_ns - self.gps["ns"] > FRESH_NS:
                return None
            return dict(self.gps)

    def rangefinder_now(self, now_ns: int) -> float | None:
        with self._lock:
            if self.rangefinder_ns is None or now_ns - self.rangefinder_ns > FRESH_NS:
                return None
            return self.rangefinder_m

    def mode_name(self) -> str:
        with self._lock:
            mode = self.mode
        if mode is None:
            return ""
        return COPTER_MODES.get(mode, f"mode {mode}")
