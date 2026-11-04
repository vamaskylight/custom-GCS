"""Follow Me: the drone moves with the object the operator has locked, and keeps its distance to it.

A port of the app's FollowGuide (apk/custom/src/Skydroid/FollowGuide.h and .cc, 2026-10-13), rule for rule. The
tests run both on the same scenarios (tests/test_follow_guide_matches_app.py). No Qt here.

What it does:
 - It takes fixes of the object's position (from the lock: the camera's angles with the laser's range or with the
   height above the ground) and estimates where the object is and how fast it moves.
 - It keeps the drone where it was relative to the object when Follow was started: the same distance, on the same
   side, at the same height.
 - It answers with a speed over the ground for the drone, five times a second. The drone flies it in Guided mode
   (ArduCopter stops by itself when the speeds stop coming, GUID_TIMEOUT).

What it never does:
 - It never flies the drone nearer to the object than MIN_DISTANCE_M. Inside that distance the drone flies straight
   away from the object, or stands: no speed towards the object, and none beside it. With the object under the drone,
   where "away" is not sure, it stands. (The distance is our own estimate, which is why Follow starts only
   START_DISTANCE_M away: the estimate can be a few metres off. For a fast object and an unsure position, speeds
   towards the object stop a little further out.)
 - It never flies the drone over the object, or round it, to get back to its side. When the object has passed the
   drone, the drone follows on from the side it is on then, at the same distance.
 - It never turns the drone: every speed comes with the heading the drone had at the start, to be kept.
 - It never changes the drone's height, and it ends when the drone is lower than END_HEIGHT_M: above the take-off
   point, or above the object where the laser measured that (rising ground).
 - It never flies faster than the speed limit, and never further from where Follow was started than the range limit.
 - It never goes on without the object: when the lock does not see it, or its position is old, the drone is told to
   stop.
 - It never flies after a jump: when the object's position jumps (the lock took another object, say), the drone
   stands until the new position has held for a few seconds.
 - It never goes on with a drone that does not fly what it is asked, and never with another drone than the one it
   was started on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

NAN = float("nan")

MIN_DISTANCE_M = 15.0       # the drone never flies itself nearer to the object than this
START_DISTANCE_M = 20.0     # and Follow starts only this far from it, over the ground
MIN_HEIGHT_M = 10.0         # Follow starts only this high (see Drone.height_m and Fix.below_m)
END_HEIGHT_M = 8.0          # and ends when the drone is lower than this
SPEED_LIMIT_LOW_MPS = 2.0   # the speed limit setting: its range and its usual value
SPEED_LIMIT_HIGH_MPS = 15.0
SPEED_LIMIT_DEFAULT_MPS = 8.0
RANGE_LIMIT_LOW_M = 50.0    # the range limit setting, from where Follow was started
RANGE_LIMIT_HIGH_M = 2000.0
RANGE_LIMIT_DEFAULT_M = 500.0

TICK_S = 0.2                # tick() is called this often
DRONE_LATE_S = 1.0          # no telemetry this long: the drone is told to stand
DRONE_SILENT_S = 2.0        # and this long: Follow ends
FIX_FRESH_S = 2.0           # the object's position older than this: the drone waits
WAIT_MAX_S = 10.0           # waiting this long: Follow ends
GUIDED_GRACE_S = 4.0        # the drone has this long to change to Guided after the start
BRAKE_S = 1.5               # after the end, "stop" is sent this long, then nothing
NOT_OBEYED_S = 5.0          # the drone flies another speed than asked this long: Follow ends
BELOW_FRESH_S = 15.0        # a measured height above the object counts this long

# How the speed goes to ArduCopter: SET_POSITION_TARGET_LOCAL_NED, five times a second (measured in its simulator,
# 4.6.2 and 4.7.0, for the app): with the speed alone (type mask 0x0DC7) ArduCopter turns its nose into the flight
# direction, and with a yaw rate of 0 (0x05C7) it drifted 11.8 degrees on a curve; with a yaw angle (0x09C7) it keeps
# that heading. Outside Guided it does not fly the speed at all. After the last message it flies on for GUID_TIMEOUT
# (3 s as shipped), then brakes and holds its place, still in Guided.
SPEED_TYPE_MASK = 0x09C7              # the speed and the yaw angle
SPEED_TYPE_MASK_NO_HEADING = 0x05C7   # the heading is not known: the speed and a yaw rate of 0
SPEED_FRAME = 1                       # MAV_FRAME_LOCAL_NED: north and east over the ground, yaw from north
COPTER_GUIDED = 4                     # ArduCopter's own mode numbers (HEARTBEAT custom_mode)
COPTER_LOITER = 5
COPTER_POSHOLD = 16
COPTER_BRAKE = 17

# The numbers below were chosen on the app's test bench, whose drone answers as ArduCopter did in its simulator.
_OBJECT_ACCEL_MPS2 = 0.6     # the object may change its speed by about this much each second
_FIRST_SPEED_SIGMA_MPS = 3.0  # at the first fix nothing is known of its speed
_LEAST_SIGMA_M = 0.5         # no fix is taken as more exact than this
_GATE_SIGMAS = 4.0           # a fix further from the estimate than this is not believed ...
_GATE_RESET_AFTER = 5        # ... until this many in a row say so: then the object is there
_FIXES_TO_KNOW = 8           # the object is known after this many fixes over this long
_KNOWN_AFTER_S = 3.0
_POSITION_GAIN = 0.3         # m/s for each metre the drone is away from its place
_OBJECT_SPEED_SHARE = 1.0    # how much of the object's own speed is flown with
_STILL_ERROR_M = 1.5         # the object stands and the drone is this near its place: no speed ...
_STILL_OBJECT_MPS = 0.4
_MOVE_ERROR_M = 2.0 * _STILL_ERROR_M      # ... until it is this far off, or the object moves this fast
_MOVE_OBJECT_MPS = 2.0 * _STILL_OBJECT_MPS
_ASKED_ACCEL_MPS2 = 2.0      # the speed asked changes no faster than this
_ASKED_SMOOTH_S = 0.7        # and it is smoothed over about this long
_DRONE_AHEAD_MAX_S = 0.6     # the drone's place is carried forward by its speed at most this long
_UNSURE_S = 0.5              # a fast object's place is less sure: the least distance grows by this many seconds of
_UNSURE_SIGMAS = 2.0         # its speed, and this many standard deviations of its place
_KNOWN_SIGMA_M = 2.5         # the object's place is known well enough to start under this
_SIDE_SURE_M = 2.0           # the side the drone keeps is given up only when the direction to the object is sure
_BESIDE_BAND_M = 3.0         # speeds beside the object: none inside the least distance, all this much further out
_OBEY_LAG_S = 1.0            # the drone needs about this long to take up a new speed; flying this much (or this
_OBEY_MISS_MPS = 2.0         # share) off the speed it should have by now is "another speed than asked"
_OBEY_MISS_SHARE = 0.6

_EARTH_RADIUS_M = 6371000.0


class Mode(IntEnum):
    """The drone's flight mode, as far as Follow cares."""
    GUIDED = 0    # the mode Follow flies in
    HOLDING = 1   # the drone holds its place by itself: Loiter, Position Hold, Brake. Follow may start from here
    OTHER = 2     # anything else (a mission, RTL, Land, a pilot's mode): Follow does not start, and ends if it runs


class State(IntEnum):
    OFF = 0
    FOLLOWING = 1
    WAITING = 2   # started, but the object is not seen or its position is old: the drone is told to stand still


class Why(IntEnum):
    """Why Follow did not start, why it waits, or why it ended (the app's numbers)."""
    NONE = 0
    NO_LOCK = 1          # it did not start: no lock that sees the object
    DRONE_SILENT = 2     # no telemetry from the drone (also an end)
    NOT_FLYING = 3       # not armed or not in the air (also an end)
    NO_GPS = 4           # the drone has no 3D GPS position
    WRONG_MODE = 5       # the drone is not holding its place (Loiter, Position Hold, Brake) or in Guided
    TOO_LOW = 6          # under MIN_HEIGHT_M (also an end: under END_HEIGHT_M)
    NO_FIX = 7           # the object's position is not known yet, or not well enough
    TOO_CLOSE = 8        # nearer to the object than START_DISTANCE_M
    NEAR = 9             # it follows, and the object is nearer than MIN_DISTANCE_M: no flying towards it
    DRONE_LATE = 10      # it waits: the drone's telemetry is late
    NOT_SEEN = 11        # the lock does not see the object right now
    FIX_OLD = 12         # no new position of the object
    JUMPED = 13          # the object's position jumped: it is not known well enough again yet
    STOPPED = 14         # it ended: by the operator
    LOCK_ENDED = 15      # the lock ended
    NOT_GUIDED = 16      # the drone did not change to Guided after the start
    PILOT_TOOK_OVER = 17  # the flight mode was changed while following
    TOO_FAR = 18         # the range limit
    FIX_LOST = 19        # waited too long for the object
    NOT_OBEYED = 20      # the drone does not fly the speeds it is asked
    DRONE_CHANGED = 21   # another drone is the active one now


@dataclass
class Drone:
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    height_m: float = 0.0     # above the take-off point
    north_mps: float = NAN    # its own speed over the ground when it reports one; NaN: taken from the positions
    east_mps: float = NAN
    heading_deg: float = NAN  # where its nose points, degrees from north; NaN: not known
    armed: bool = False
    flying: bool = False
    gps3d: bool = False
    mode: Mode = Mode.OTHER


@dataclass
class Fix:
    """Where the object is, and how sure that is (one standard deviation, metres)."""
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    sigma_m: float = 3.0
    # How far the object is below the drone, when that was measured (the laser's range and the camera's angle).
    # NaN: not measured, the ground is taken as level.
    below_m: float = NAN


@dataclass(frozen=True)
class Velocity:
    """What to send to the drone now."""
    send: bool = False        # False: say nothing
    north_mps: float = 0.0
    east_mps: float = 0.0
    # The heading to keep, degrees from north: the drone's own at the start. NaN: it was not known then, and the
    # speed goes with a yaw rate of 0.
    heading_deg: float = NAN


def _clamp(value: float, low: float, high: float) -> float:
    """std::clamp: low when under it, high when over it, else the value (a NaN stays NaN)."""
    if value < low:
        return low
    if high < value:
        return high
    return value


def _clamp_or(value: float, low: float, high: float, otherwise: float) -> float:
    return _clamp(value, low, high) if math.isfinite(value) else otherwise


class _Axis:
    """One direction of the object's motion: place and speed, with their covariance."""

    __slots__ = ("p", "ppp", "ppv", "pvv", "v")

    def __init__(self) -> None:
        self.p = 0.0
        self.v = 0.0
        self.ppp = 0.0
        self.ppv = 0.0
        self.pvv = 0.0


class FollowGuide:
    def __init__(self) -> None:
        self._speed_limit = SPEED_LIMIT_DEFAULT_MPS
        self._range_limit = RANGE_LIMIT_DEFAULT_M
        # The local frame: metres north and east of the first position seen.
        self._have_origin = False
        self._origin_lat = 0.0
        self._origin_lon = 0.0
        self._cos_lat = 1.0
        # The drone.
        self._have_drone = False
        self._drone = Drone()
        self._drone_at_s = 0.0       # the time its place holds for
        self._drone_seen_at_s = 0.0  # the last time telemetry came at all
        self._drone_n = 0.0
        self._drone_e = 0.0
        self._drone_vn = 0.0
        self._drone_ve = 0.0
        # The object.
        self._have_object = False
        self._north = _Axis()
        self._east = _Axis()
        self._object_at_s = 0.0      # the time the estimate holds for
        self._first_fix_at_s = 0.0
        self._last_fix_at_s = 0.0    # the last fix that was believed
        self._fixes = 0
        self._rejected = 0
        self._jumps = 0
        self._have_below = False     # the height above the object was measured
        self._below_m = 0.0
        self._below_at_s = 0.0
        # Follow itself.
        self._state = State.OFF
        self._why = Why.NONE
        self._offset_n = 0.0         # where the drone stays, seen from the object
        self._offset_e = 0.0
        self._kept_distance_m = 0.0
        self._start_n = 0.0          # where the drone was when Follow started
        self._start_e = 0.0
        self._started_at_s = 0.0
        self._heading_deg = NAN      # the heading the drone keeps
        self._guided_seen = False
        self._wait_since_s = 0.0
        self._asked_vn = 0.0
        self._asked_ve = 0.0
        self._still = False          # the object stands and the drone is at its place: no speed is asked
        self._expect_vn = 0.0        # the speed the drone should have by now
        self._expect_ve = 0.0
        self._off_asked_mps = 0.0    # how far its speed is from that
        self._miss_s = 0.0           # for how long it has flown another speed
        self._last_tick_s = 0.0
        self._brake_until_s = 0.0

    # --- the settings: values outside their range are brought into it ---

    def set_speed_limit(self, mps: float) -> None:
        self._speed_limit = _clamp_or(mps, SPEED_LIMIT_LOW_MPS, SPEED_LIMIT_HIGH_MPS, SPEED_LIMIT_DEFAULT_MPS)

    def set_range_limit(self, metres: float) -> None:
        self._range_limit = _clamp_or(metres, RANGE_LIMIT_LOW_M, RANGE_LIMIT_HIGH_M, RANGE_LIMIT_DEFAULT_M)

    @property
    def speed_limit(self) -> float:
        return self._speed_limit

    @property
    def range_limit(self) -> float:
        return self._range_limit

    # --- the local frame ---

    def _set_origin(self, lat_deg: float, lon_deg: float) -> None:
        self._have_origin = True
        self._origin_lat = lat_deg
        self._origin_lon = lon_deg
        self._cos_lat = max(0.01, math.cos(lat_deg * math.pi / 180.0))

    def _to_local(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        north = (lat_deg - self._origin_lat) * math.pi / 180.0 * _EARTH_RADIUS_M
        east = (lon_deg - self._origin_lon) * math.pi / 180.0 * _EARTH_RADIUS_M * self._cos_lat
        return north, east

    def _to_geo(self, north: float, east: float) -> tuple[float, float]:
        lat = self._origin_lat + north / _EARTH_RADIUS_M * 180.0 / math.pi
        lon = self._origin_lon + east / (_EARTH_RADIUS_M * self._cos_lat) * 180.0 / math.pi
        return lat, lon

    # --- what comes in ---

    def drone(self, d: Drone, at_s: float) -> None:
        """Telemetry: where the drone is at the time at_s (seconds on any steady clock)."""
        if not (math.isfinite(d.lat_deg) and math.isfinite(d.lon_deg) and math.isfinite(at_s)):
            return
        if not self._have_origin:
            self._set_origin(d.lat_deg, d.lon_deg)
        n, e = self._to_local(d.lat_deg, d.lon_deg)
        if not self._have_drone:
            self._drone_n, self._drone_e = n, e
            self._drone_vn = self._drone_ve = 0.0
            self._drone_at_s = at_s
        elif math.isfinite(d.north_mps) and math.isfinite(d.east_mps):
            # The drone's own speed estimate: better than two positions.
            self._drone_n, self._drone_e = n, e
            self._drone_vn, self._drone_ve = d.north_mps, d.east_mps
            self._drone_at_s = at_s
        elif n != self._drone_n or e != self._drone_e:
            # A new position. Telemetry repeats the last one until the next comes, so a position that did not
            # change is no news about the speed.
            dt = at_s - self._drone_at_s
            if dt > 0.02:
                keep = 0.0 if dt >= 1.0 else 0.4
                self._drone_vn = keep * self._drone_vn + (1.0 - keep) * (n - self._drone_n) / dt
                self._drone_ve = keep * self._drone_ve + (1.0 - keep) * (e - self._drone_e) / dt
            self._drone_n, self._drone_e = n, e
            self._drone_at_s = at_s
        elif at_s - self._drone_at_s >= 1.0:
            # The same place for a second: it stands.
            self._drone_vn = self._drone_ve = 0.0
            self._drone_at_s = at_s
        self._drone = d
        self._drone_seen_at_s = at_s
        self._have_drone = True

    def forget_object(self) -> None:
        """Forget the object (the lock ended, or a new object was locked)."""
        self._have_object = False
        self._fixes = 0
        self._rejected = 0
        self._jumps = 0
        self._have_below = False

    def forget_drone(self) -> None:
        """Forget the drone: the telemetry so far was another drone's."""
        self._have_drone = False

    def _predict(self, dt: float) -> None:
        q = _OBJECT_ACCEL_MPS2 * _OBJECT_ACCEL_MPS2
        for a in (self._north, self._east):
            a.p += a.v * dt
            a.ppp += dt * (2.0 * a.ppv + dt * a.pvv) + q * dt * dt * dt * dt / 4.0
            a.ppv += dt * a.pvv + q * dt * dt * dt / 2.0
            a.pvv += q * dt * dt

    def fix(self, f: Fix, at_s: float) -> None:
        """A fix of the object's position at the time at_s. Fixes come while a lock runs, with or without Follow."""
        if not (math.isfinite(f.lat_deg) and math.isfinite(f.lon_deg) and math.isfinite(at_s)):
            return
        if not self._have_origin:
            self._set_origin(f.lat_deg, f.lon_deg)
        n, e = self._to_local(f.lat_deg, f.lon_deg)
        sigma = max(_LEAST_SIGMA_M, f.sigma_m if math.isfinite(f.sigma_m) else 3.0)
        r = sigma * sigma
        if math.isfinite(f.below_m) and (not self._have_below or at_s >= self._below_at_s):
            self._have_below = True
            self._below_m = f.below_m
            self._below_at_s = at_s

        def begin() -> None:
            for a in (self._north, self._east):
                a.v = 0.0
                a.ppp = r
                a.ppv = 0.0
                a.pvv = _FIRST_SPEED_SIGMA_MPS * _FIRST_SPEED_SIGMA_MPS
            self._north.p = n
            self._east.p = e
            self._have_object = True
            self._object_at_s = at_s
            self._first_fix_at_s = at_s
            self._last_fix_at_s = at_s
            self._fixes = 1
            self._rejected = 0

        if not self._have_object:
            begin()
            return
        dt = at_s - self._object_at_s
        if dt < 0.0:
            return  # older than what is known already
        self._predict(dt)
        self._object_at_s = at_s
        yn = n - self._north.p
        ye = e - self._east.p
        sn = self._north.ppp + r
        se = self._east.ppp + r
        if yn * yn / sn + ye * ye / se > _GATE_SIGMAS * _GATE_SIGMAS:
            # Far from where the object should be: one such fix is a wrong reading. Several in a row mean the
            # estimate is wrong, and it starts again from there.
            self._rejected += 1
            if self._rejected >= _GATE_RESET_AFTER:
                begin()
                self._jumps += 1
            return

        def update(a: _Axis, y: float) -> None:
            s = a.ppp + r
            kp = a.ppp / s
            kv = a.ppv / s
            a.p += kp * y
            a.v += kv * y
            ppp = (1.0 - kp) * a.ppp
            ppv = (1.0 - kp) * a.ppv
            pvv = a.pvv - kv * a.ppv
            a.ppp, a.ppv, a.pvv = ppp, ppv, pvv

        update(self._north, yn)
        update(self._east, ye)
        self._rejected = 0
        self._fixes += 1
        self._last_fix_at_s = at_s

    # --- what is known ---

    def object_known(self, now_s: float) -> bool:
        """The object's position is known well enough to start (a fresh fix, a few of them, and they agree)."""
        return (self._have_object and self._fixes >= _FIXES_TO_KNOW
                and self._last_fix_at_s - self._first_fix_at_s >= _KNOWN_AFTER_S
                and now_s - self._last_fix_at_s <= FIX_FRESH_S and self.place_sigma_m(now_s) <= _KNOWN_SIGMA_M)

    def place_sigma_m(self, now_s: float) -> float:
        """How sure the object's place is now: one standard deviation, metres. Negative when not known."""
        if not self._have_object:
            return -1.0
        dt = _clamp(now_s - self._object_at_s, 0.0, FIX_FRESH_S)

        def variance(a: _Axis) -> float:
            return a.ppp + dt * (2.0 * a.ppv + dt * a.pvv)

        return math.sqrt(max(0.0, max(variance(self._north), variance(self._east))))

    def _object_at(self, at_s: float) -> tuple[float, float]:
        dt = _clamp(at_s - self._object_at_s, 0.0, FIX_FRESH_S)
        return self._north.p + self._north.v * dt, self._east.p + self._east.v * dt

    def _drone_at(self, at_s: float) -> tuple[float, float]:
        dt = _clamp(at_s - self._drone_at_s, 0.0, _DRONE_AHEAD_MAX_S)
        return self._drone_n + self._drone_vn * dt, self._drone_e + self._drone_ve * dt

    def distance_m(self, now_s: float) -> float:
        """Drone to object over the ground, metres. Negative when not known."""
        if not self._have_object or not self._have_drone:
            return -1.0
        on, oe = self._object_at(now_s)
        dn, de = self._drone_at(now_s)
        return math.hypot(on - dn, oe - de)

    def object_from_drone(self, now_s: float) -> tuple[float, float] | None:
        """The object as the drone sees it: metres north and east of the drone. None when not known."""
        if not self._have_object or not self._have_drone:
            return None
        on, oe = self._object_at(now_s)
        dn, de = self._drone_at(now_s)
        return on - dn, oe - de

    @property
    def kept_distance_m(self) -> float:
        """The distance Follow keeps (set at the start)."""
        return self._kept_distance_m

    def kept_place(self) -> tuple[float, float] | None:
        """Where the drone stays, seen from the object: metres north and east. None when Follow is off."""
        if self._state == State.OFF:
            return None
        return self._offset_n, self._offset_e

    @property
    def last_drone(self) -> Drone:
        return self._drone

    def drone_age_s(self, now_s: float) -> float:
        """How old the last telemetry is. Negative: none yet."""
        return now_s - self._drone_seen_at_s if self._have_drone else -1.0

    def below_m(self, now_s: float) -> float:
        """The height above the object as the laser last measured it. Negative: not measured, or too long ago."""
        return self._below_m if self._have_below and now_s - self._below_at_s <= BELOW_FRESH_S else -1.0

    def asked_speed_mps(self) -> float:
        """The speed last asked from the drone."""
        return math.hypot(self._asked_vn, self._asked_ve)

    def object_speed_mps(self) -> float:
        """The object's own estimated speed."""
        return math.hypot(self._north.v, self._east.v) if self._have_object else 0.0

    def from_start_m(self, now_s: float) -> float:
        """How far the drone is from where Follow started."""
        if self._state == State.OFF or not self._have_drone:
            return 0.0
        dn, de = self._drone_at(now_s)
        return math.hypot(dn - self._start_n, de - self._start_e)

    @property
    def off_asked_mps(self) -> float:
        """How far the drone's own speed is from the speed it should have by now (see NOT_OBEYED_S)."""
        return self._off_asked_mps

    def object_position(self, now_s: float) -> tuple[float, float] | None:
        """The object's estimated position now (latitude, longitude). None when not known."""
        if not self._have_object:
            return None
        return self._to_geo(*self._object_at(now_s))

    @property
    def state(self) -> State:
        return self._state

    @property
    def why(self) -> Why:
        """While following NONE, or NEAR; why it waits; after start() refused or after the end, the reason."""
        return self._why

    @property
    def jumps(self) -> int:
        """How often the object's position has jumped since it was last forgotten."""
        return self._jumps

    def braking(self, now_s: float) -> bool:
        """True while tick() still has "stop" to send after the end."""
        return self._state == State.OFF and self._brake_until_s > now_s

    # --- starting, ending ---

    def would_start(self, lock_seen: bool, now_s: float) -> Why:
        """Why Follow would not start now. Why.NONE: it would."""
        if not lock_seen:
            return Why.NO_LOCK
        if not self._have_drone or now_s - self._drone_seen_at_s > DRONE_SILENT_S:
            return Why.DRONE_SILENT
        if not self._drone.armed or not self._drone.flying:
            return Why.NOT_FLYING
        if not self._drone.gps3d:
            return Why.NO_GPS
        if self._drone.mode == Mode.OTHER:
            return Why.WRONG_MODE
        if not (self._drone.height_m >= MIN_HEIGHT_M):
            return Why.TOO_LOW
        if not self.object_known(now_s):
            return Why.NO_FIX
        if self._have_below and now_s - self._below_at_s <= BELOW_FRESH_S and self._below_m < MIN_HEIGHT_M:
            return Why.TOO_LOW  # the ground at the object is higher than the take-off point
        if not (self.distance_m(now_s) >= START_DISTANCE_M):
            return Why.TOO_CLOSE
        return Why.NONE

    def start(self, lock_seen: bool, now_s: float) -> Why:
        """The operator starts Follow. Why.NONE when it started; the caller then asks the drone for Guided."""
        if self._state != State.OFF:
            return Why.NONE  # it runs already
        why = self.would_start(lock_seen, now_s)
        if why != Why.NONE:
            self._why = why
            return why
        on, oe = self._object_at(now_s)
        dn, de = self._drone_at(now_s)
        # The drone stays where it is now, seen from the object.
        self._offset_n = dn - on
        self._offset_e = de - oe
        self._kept_distance_m = math.hypot(dn - on, de - oe)
        self._start_n, self._start_e = dn, de
        self._started_at_s = now_s
        self._heading_deg = self._drone.heading_deg
        self._guided_seen = self._drone.mode == Mode.GUIDED
        self._asked_vn = self._asked_ve = 0.0
        self._still = False
        self._expect_vn = self._drone_vn
        self._expect_ve = self._drone_ve
        self._off_asked_mps = 0.0
        self._miss_s = 0.0
        self._last_tick_s = now_s
        self._brake_until_s = 0.0
        self._state = State.FOLLOWING
        self._why = Why.NONE
        return Why.NONE

    def _end(self, why: Why, now_s: float, brake: bool) -> None:
        self._state = State.OFF
        self._why = why
        self._asked_vn = self._asked_ve = 0.0
        self._brake_until_s = now_s + BRAKE_S if brake else 0.0

    def stop(self, why: Why, now_s: float) -> None:
        """The end, by the operator or for a reason found outside (the lock ended)."""
        if self._state != State.OFF:
            self._end(why, now_s, True)

    def drop(self, why: Why) -> None:
        """The end without a word to the drone: it is not Follow's to fly any more."""
        if self._state != State.OFF:
            self._end(why, 0.0, False)

    def _not_obeyed(self, asked_vn: float, asked_ve: float, dt: float) -> bool:
        """True when the drone has not flown what it was asked for NOT_OBEYED_S."""
        # The speed the drone should have by now: what was asked, a second ago.
        share = min(1.0, dt / _OBEY_LAG_S)
        self._expect_vn += (asked_vn - self._expect_vn) * share
        self._expect_ve += (asked_ve - self._expect_ve) * share
        if not self._guided_seen:
            return False  # not Follow's to fly yet
        self._off_asked_mps = math.hypot(self._drone_vn - self._expect_vn, self._drone_ve - self._expect_ve)
        allowed = max(_OBEY_MISS_MPS, _OBEY_MISS_SHARE * math.hypot(self._expect_vn, self._expect_ve))
        self._miss_s = self._miss_s + dt if self._off_asked_mps > allowed else 0.0
        return self._miss_s > NOT_OBEYED_S

    def tick(self, lock_active: bool, lock_seen: bool, now_s: float) -> Velocity:
        """Every TICK_S while state is not OFF, and for BRAKE_S after the end. Gives the speed to send."""
        nothing = Velocity(False, 0.0, 0.0, NAN)
        stand = Velocity(True, 0.0, 0.0, self._heading_deg)
        if self._state == State.OFF:
            # After the end the drone is told to stop for a moment. Then nothing more is said.
            return stand if self._brake_until_s > now_s else nothing
        dt = _clamp(now_s - self._last_tick_s, 0.0, 1.0)
        self._last_tick_s = now_s

        # The drone itself first. When it is not Follow's to fly (the pilot has it, or it has landed), nothing is
        # sent to it.
        if now_s - self._drone_seen_at_s > DRONE_SILENT_S:
            self._end(Why.DRONE_SILENT, now_s, True)
            return stand
        if not self._drone.armed or not self._drone.flying:
            self._end(Why.NOT_FLYING, now_s, False)
            return nothing
        if self._drone.mode == Mode.GUIDED:
            self._guided_seen = True
        elif self._guided_seen:
            self._end(Why.PILOT_TOOK_OVER, now_s, False)
            return nothing
        elif now_s - self._started_at_s > GUIDED_GRACE_S:
            self._end(Why.NOT_GUIDED, now_s, False)
            return nothing
        if not lock_active:
            self._end(Why.LOCK_ENDED, now_s, True)
            return stand
        # The drone holds its height by itself. Lower than it may be: the end.
        if not (self._drone.height_m >= END_HEIGHT_M) or (
                self._have_below and now_s - self._below_at_s <= BELOW_FRESH_S and self._below_m < END_HEIGHT_M):
            self._end(Why.TOO_LOW, now_s, True)
            return stand

        # Without the object the drone stands.
        wait = Why.NONE
        if now_s - self._drone_seen_at_s > DRONE_LATE_S:
            wait = Why.DRONE_LATE  # where the drone is now is not known: no speed is made up from an old place
        elif not lock_seen:
            wait = Why.NOT_SEEN
        elif not self._have_object or now_s - self._last_fix_at_s > FIX_FRESH_S:
            wait = Why.FIX_OLD
        elif self._fixes < _FIXES_TO_KNOW or self._last_fix_at_s - self._first_fix_at_s < _KNOWN_AFTER_S:
            # The estimate was started anew, because the positions jumped. Where the object is and how fast it
            # moves has to hold for a few seconds again, as before the start.
            wait = Why.JUMPED
        if wait != Why.NONE:
            if self._state != State.WAITING:
                self._state = State.WAITING
                self._wait_since_s = now_s
            self._why = wait
            self._asked_vn = self._asked_ve = 0.0
            if now_s - self._wait_since_s > WAIT_MAX_S:
                self._end(Why.FIX_LOST, now_s, True)
            elif wait != Why.DRONE_LATE and self._not_obeyed(0.0, 0.0, dt):
                self._end(Why.NOT_OBEYED, now_s, True)  # told to stand, and it flies on
            return stand
        self._state = State.FOLLOWING
        self._why = Why.NONE

        on, oe = self._object_at(now_s)
        dn, de = self._drone_at(now_s)
        if math.hypot(dn - self._start_n, de - self._start_e) > self._range_limit:
            self._end(Why.TOO_FAR, now_s, True)
            return stand

        # The object has come nearer than the least distance, or it has passed the drone (the place the drone keeps
        # is on the far side of the object now). The drone does not fly round the object or over it to get to its
        # place. It gives up its side and keeps its distance where it is: straight out from the object.
        from_n = dn - on
        from_e = de - oe
        distance = math.hypot(from_n, from_e)
        # How far off the object's place may be: more for a fast object (the fixes are a little old).
        unsure = _UNSURE_S * math.hypot(self._north.v, self._east.v) + _UNSURE_SIGMAS * self.place_sigma_m(now_s)
        if ((distance < MIN_DISTANCE_M or from_n * self._offset_n + from_e * self._offset_e < 0.0)
                and distance > max(_SIDE_SURE_M, unsure)):
            self._offset_n = from_n / distance * self._kept_distance_m
            self._offset_e = from_e / distance * self._kept_distance_m

        # The drone's place is the object's place plus the offset of the start. It flies the object's own speed,
        # and a little more or less for what it is away from its place.
        en = on + self._offset_n - dn
        ee = oe + self._offset_e - de
        vn = _OBJECT_SPEED_SHARE * self._north.v + _POSITION_GAIN * en
        ve = _OBJECT_SPEED_SHARE * self._east.v + _POSITION_GAIN * ee
        error = math.hypot(en, ee)
        object_speed = math.hypot(self._north.v, self._east.v)
        if self._still:
            self._still = error < _MOVE_ERROR_M and object_speed < _MOVE_OBJECT_MPS
        else:
            self._still = error < _STILL_ERROR_M and object_speed < _STILL_OBJECT_MPS
        if self._still:
            vn = ve = 0.0
        # Gentle changes of the speed asked: the fixes are noisy, and the drone should not twitch with them.
        if _ASKED_SMOOTH_S > 0.0:
            share = min(1.0, max(dt, 0.05) / _ASKED_SMOOTH_S)
            vn = self._asked_vn + (vn - self._asked_vn) * share
            ve = self._asked_ve + (ve - self._asked_ve) * share
        step = _ASKED_ACCEL_MPS2 * max(dt, 0.05)
        change = math.hypot(vn - self._asked_vn, ve - self._asked_ve)
        if change > step:
            vn = self._asked_vn + (vn - self._asked_vn) * step / change
            ve = self._asked_ve + (ve - self._asked_ve) * step / change
        # Never nearer than the least distance by the drone's own flying. The speed has a part straight away from
        # the object and a part beside it. Near the object, the part that points at it is taken out. Inside the
        # least distance the part beside it goes as well (where the object is exactly is a few metres unsure, so
        # "beside" is not sure either). And with the object under the drone, nearer than its place is unsure, no
        # way is "away": the drone stands until the object is to one side of it.
        if distance > 1e-6:
            out_n = from_n / distance
            out_e = from_e / distance
            away = vn * out_n + ve * out_e
            beside_n = vn - away * out_n
            beside_e = ve - away * out_e
            if distance < MIN_DISTANCE_M + unsure:
                away = max(0.0, away)
            if unsure > 1e-6:
                away *= _clamp(distance / unsure - 1.0, 0.0, 1.0)
            beside = _clamp((distance - MIN_DISTANCE_M) / _BESIDE_BAND_M, 0.0, 1.0)
            vn = away * out_n + beside * beside_n
            ve = away * out_e + beside * beside_e
        else:
            vn = ve = 0.0
        self._why = Why.NEAR if distance < MIN_DISTANCE_M else Why.NONE
        # The speed limit.
        speed = math.hypot(vn, ve)
        if speed > self._speed_limit:
            vn *= self._speed_limit / speed
            ve *= self._speed_limit / speed
        self._asked_vn = vn
        self._asked_ve = ve
        if self._not_obeyed(vn, ve, dt):
            self._end(Why.NOT_OBEYED, now_s, True)
            return stand
        return Velocity(True, vn, ve, self._heading_deg)
