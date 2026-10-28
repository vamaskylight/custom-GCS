"""What goes into GPS_INPUT, ten times a second, and the flight controller settings it needs.

ArduPilot (AP_GPS_MAV, 4.6 and 4.7) takes a MAVLink GPS as it comes. What it does with the fields,
and so how they are filled:

- One 3D message per camera fix. EKF3 takes every 3D message as a new measurement with its own
  error. Ten copies of one fix a second (one error, ten times) made it chase each fix's jump: in the
  simulator it drifted 15 m and landed 25 s after GPS 1 went off, where ten fixes a second with
  their own errors held within 1 m (2026-10-29). So the other nine messages of each second carry the
  fix forward as a 2D fix, which EKF3 does not fuse (it waits for a 3D fix). They keep GPS 2 healthy
  (ArduPilot wants about 5 messages a second) and on the ground station's screen, and with the
  stand-in satellites a GPS 1 that falls back to 2D does not take GPS 2's place.
- Every field is sent in every message. AP_GPS_MAV keeps a field as "known" once it was sent (the
  accuracies, the vertical speed), so a later message without it would leave the old value standing
  as if new. Only the vertical speed is never sent: GPS_RAW_INT has none, a camera does not measure
  it, and the EKF takes heights from the barometer.
- The altitude is always sent. With the ALT ignore flag AP_GPS_MAV puts 0 m above sea level.
- time_week 0: ArduPilot then times each message by its arrival, minus GPS2_DELAY_MS. The position
  is carried forward to the moment of sending, so GPS2_DELAY_MS is only the way over the wire and
  ArduPilot's 50 Hz GPS update (20 ms). A GPS week from this computer's clock would also set the
  flight controller's clock (AP_RTC takes a later GPS time while disarmed), and the Pi's clock is not
  known to be right in the field.
- satellites: a camera sees none, but EKF3 stops trusting a GPS that reports fewer than 6
  (EK3_GPS_CHECK), so a camera fix reports SATELLITES_STAND_IN.
- hdop: EKF3 wants 2.5 or less. A camera fix reports its accuracy / 3, a passed-on fix GPS 1's own.
- yaw 0: no heading.
- fix type at most 3: a passed-on RTK fix is not claimed as RTK.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass

log = logging.getLogger("vama_nav.gps_input")

GPS_ID = 1                          # GPS 2 (GPS2_TYPE 14): instance 1
RATE_HZ = 10.0                      # ArduPilot calls a GPS unhealthy below about 5 messages a second
IGNORE_VEL_VERT = 16                # GPS_INPUT_IGNORE_FLAG_VEL_VERT, the one field never sent
MAX_AGE_S = 2.0                     # a camera fix older than this: no fix, the EKF holds on its own sensors
FUTURE_S = 0.05                     # a fix this far ahead of now is a clock fault, not a fix
GPS1_MAX_AGE_NS = 1_000_000_000     # GPS 1's GPS_RAW_INT older than this is not passed on
SATELLITES_STAND_IN = 10
ACCEL_UNKNOWN = 1.0                 # m/s2 the drone may speed up or turn by between two pictures
M_PER_DEG = 111_318.84502145034     # metres per degree of latitude, as ArduPilot (LOCATION_SCALING_FACTOR)
UNKNOWN_ACC = 999.0                 # an accuracy that is not known (no fix, or GPS 1 did not say)
UNKNOWN_DOP = 99.99

# The flight controller settings without which camera positions are held back (``settings_problem``).
REQUIRED = {"GPS2_TYPE": 14, "GPS_AUTO_SWITCH": 4, "GPS_PRIMARY": 0}
# Advised: the sender's positions are for the moment they are sent (see the module's notes).
ADVISED = {"GPS2_DELAY_MS": 20}
NOT_READ = "not read yet"

# Where a message comes from: nothing (no fix), GPS 1 passed on while disarmed, a new camera fix (3D,
# fused by the EKF), the same fix carried forward until the next one (2D, not fused).
NONE, GPS1, CAMERA, CARRIED = "none", "gps1", "camera", "carried"


@dataclass(frozen=True)
class Fix:
    """A position measured from a picture, at the picture's moment on this computer's clock
    (time.monotonic_ns), with the drone's speed over the ground then (the shift between pictures)."""

    local_ns: int
    lat: float
    lon: float
    alt_msl_m: float
    vn: float
    ve: float
    horiz_acc_m: float
    vert_acc_m: float
    speed_acc_mps: float

    def problem(self) -> str | None:
        """Why this fix must not reach the flight controller, or None."""
        numbers = (self.lat, self.lon, self.alt_msl_m, self.vn, self.ve, self.horiz_acc_m, self.vert_acc_m,
                   self.speed_acc_mps)
        if not all(math.isfinite(float(x)) for x in numbers):
            return "a value is not a number"
        if not (-89.0 <= self.lat <= 89.0 and -180.0 <= self.lon <= 180.0):
            return f"position {self.lat:.6f}, {self.lon:.6f} is off the map"
        if min(self.horiz_acc_m, self.vert_acc_m, self.speed_acc_mps) <= 0:
            return "an accuracy of 0 or less"
        if math.hypot(self.vn, self.ve) > 100.0:
            return f"speed {math.hypot(self.vn, self.ve):.0f} m/s"
        return None


@dataclass(frozen=True)
class GpsInput:
    """One GPS_INPUT message, in plain units."""

    source: str                     # NONE, GPS1, CAMERA or CARRIED
    fix_type: int
    lat: float = 0.0
    lon: float = 0.0
    alt_msl_m: float = 0.0
    vn: float = 0.0
    ve: float = 0.0
    hdop: float = UNKNOWN_DOP
    vdop: float = UNKNOWN_DOP
    speed_acc: float = UNKNOWN_ACC
    horiz_acc: float = UNKNOWN_ACC
    vert_acc: float = UNKNOWN_ACC
    satellites: int = 0

    def send(self, mav, time_usec: int) -> None:
        """Send it with a pymavlink ``conn.mav``."""
        mav.gps_input_send(int(time_usec), GPS_ID, IGNORE_VEL_VERT, 0, 0, int(self.fix_type),
                           round(self.lat * 1e7), round(self.lon * 1e7), float(self.alt_msl_m),
                           float(self.hdop), float(self.vdop), float(self.vn), float(self.ve), 0.0,
                           float(self.speed_acc), float(self.horiz_acc), float(self.vert_acc), int(self.satellites))


NO_FIX = GpsInput(NONE, 1)


def _dop(acc_m: float) -> float:
    return min(max(acc_m / 3.0, 0.5), UNKNOWN_DOP)


def carried_forward(fix: Fix, now_ns: int, satellites: int = SATELLITES_STAND_IN, max_age_s: float = MAX_AGE_S,
                    new: bool = True) -> GpsInput | None:
    """The camera's fix moved on with its own speed to ``now_ns``, its accuracy grown with the time
    since its picture: a 3D fix when ``new``, else 2D (see the module's notes). None when it is
    older than ``max_age_s`` (or from the future)."""
    age = (now_ns - fix.local_ns) / 1e9
    if age > max_age_s or age < -FUTURE_S:
        return None
    age = max(age, 0.0)
    lat = fix.lat + fix.vn * age / M_PER_DEG
    lon = fix.lon + fix.ve * age / (M_PER_DEG * max(math.cos(math.radians(fix.lat)), 0.01))
    horiz = fix.horiz_acc_m + fix.speed_acc_mps * age + 0.5 * ACCEL_UNKNOWN * age * age
    return GpsInput(CAMERA if new else CARRIED, 3 if new else 2, lat, lon, fix.alt_msl_m, fix.vn, fix.ve,
                    hdop=_dop(horiz), vdop=_dop(fix.vert_acc_m), speed_acc=fix.speed_acc_mps, horiz_acc=horiz,
                    vert_acc=fix.vert_acc_m, satellites=satellites)


def passed_on(raw, arrival_ns: int, now_ns: int) -> GpsInput | None:
    """GPS 1's own fix (its GPS_RAW_INT) as GPS 2, for the arming checks while disarmed. None unless
    it is a 3D fix that arrived less than a second ago."""
    if raw is None or now_ns - arrival_ns > GPS1_MAX_AGE_NS or int(raw.fix_type) < 3:
        return None
    vel, cog = int(raw.vel), int(raw.cog)
    if vel == 65535 or cog == 65535:
        vn = ve = 0.0
    else:
        vn = vel / 100.0 * math.cos(math.radians(cog / 100.0))
        ve = vel / 100.0 * math.sin(math.radians(cog / 100.0))
    eph, epv, sats = int(raw.eph), int(raw.epv), int(raw.satellites_visible)
    h_acc, v_acc, vel_acc = (int(getattr(raw, name, 0) or 0) for name in ("h_acc", "v_acc", "vel_acc"))
    return GpsInput(GPS1, 3, raw.lat / 1e7, raw.lon / 1e7, raw.alt / 1000.0, vn, ve,
                    hdop=UNKNOWN_DOP if eph == 65535 else eph / 100.0,
                    vdop=UNKNOWN_DOP if epv == 65535 else epv / 100.0,
                    speed_acc=vel_acc / 1000.0 if vel_acc > 0 else UNKNOWN_ACC,
                    horiz_acc=h_acc / 1000.0 if h_acc > 0 else UNKNOWN_ACC,
                    vert_acc=v_acc / 1000.0 if v_acc > 0 else UNKNOWN_ACC,
                    satellites=0 if sats == 255 else sats)


def settings_problem(values: dict[str, float]) -> str | None:
    """Why camera positions must not go to this flight controller, or None. ``values``: the
    parameters read from it. With other settings ArduPilot could take the camera's position while
    the real GPS works (GPS_AUTO_SWITCH 1 picks the GPS with more satellites, 2 blends them)."""
    missing = [name for name in REQUIRED if name not in values]
    if missing:
        return f"{NOT_READ}: " + ", ".join(missing) + " (ArduPilot 4.6 or newer)"
    wrong = [f"{name} is {values[name]:g}, needs {want}" for name, want in REQUIRED.items() if values[name] != want]
    return "; ".join(wrong) or None


class Sender:
    """Decides each GPS_INPUT. Thread safe: the matching hands its fixes over from its own thread."""

    def __init__(self, satellites: int = SATELLITES_STAND_IN, max_age_s: float = MAX_AGE_S) -> None:
        self.satellites = int(satellites)
        self.max_age_s = float(max_age_s)
        self._lock = threading.Lock()
        self._fix: Fix | None = None
        self._fix_sent = False              # the newest fix went out as its one 3D message
        self.offered = self.refused = 0

    def offer(self, fix: Fix) -> bool:
        """A new camera fix. Refused when it is not sane or older than the one already held."""
        problem = fix.problem()
        with self._lock:
            self.offered += 1
            if problem is None and self._fix is not None and fix.local_ns <= self._fix.local_ns:
                problem = "older than the one already held"
            if problem is not None:
                self.refused += 1
            else:
                self._fix, self._fix_sent = fix, False
        if problem is not None:
            log.warning("a camera position was not used: %s", problem)
            return False
        return True

    def message(self, now_ns: int, armed: bool, gps1: tuple | None, held_back: str | None) -> tuple[GpsInput, str]:
        """(the message to send at ``now_ns``, why). ``gps1``: GPS 1's newest (GPS_RAW_INT, arrival_ns)
        or None. ``held_back``: why camera positions must not be sent (``settings_problem``), or None."""
        if not armed:
            out = passed_on(gps1[0], gps1[1], now_ns) if gps1 is not None else None
            if out is None:
                return NO_FIX, "disarmed, and GPS 1 has no fresh 3D fix to pass on"
            return out, "disarmed: GPS 1's fix passed on, for the arming checks"
        if held_back is not None:
            return NO_FIX, f"armed, camera positions held back: {held_back}"
        with self._lock:
            fix, new = self._fix, not self._fix_sent
        if fix is None:
            return NO_FIX, "armed, no camera position yet"
        out = carried_forward(fix, now_ns, self.satellites, self.max_age_s, new=new)
        if out is None and fix.local_ns > now_ns:
            return NO_FIX, "armed, the newest camera position is ahead of this computer's clock"
        if out is None:
            return NO_FIX, f"armed, the newest camera position is over {self.max_age_s:g} s old"
        if new:
            with self._lock:
                if self._fix is fix:
                    self._fix_sent = True
        return out, "armed: the camera's positions (each fix once as 3D, carried forward as 2D)"
