"""Follow Me in VGCS: the drone moves with the object the operator has locked on the video, and keeps its distance.

A port of the app's ObjectFollow (apk/custom/src/Skydroid/ObjectFollow.*, 2026-10-13). The pieces:
 - FollowLock: the lock on the video. It places the locked object on the ground, picture by picture
   (object_placed), and says whether it still sees it.
 - FollowGuide (follow_guide.py): every rule of Follow. It turns those places and the drone's telemetry into a speed
   over the ground, five times a second.
 - FollowDrone: the drone, telemetry in and a speed out (VGCS's link to the active drone).
 - ObjectFollow (this class) runs the clock, starts and stops Follow for the operator, and says in plain words what
   is going on.
It is a follow that keeps its distance to watch an object: it never flies the drone nearer than MIN_DISTANCE_M, never
changes its height, and stops when the lock is lost. There is no option that closes in on the object.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QElapsedTimer, QObject, QSettings, QTimer, Signal

from vgcs.app import follow_guide as fg
from vgcs.app.follow_guide import Mode, State, Why
from vgcs.map.app_settings import QS_APP, QS_ORG

SETTINGS_GROUP = "follow"   # in VGCS's own settings (QS_ORG, QS_APP), as every other VGCS setting
TICK_MS = 200   # follow_guide.TICK_S


class FollowDrone(QObject):
    """The drone, as far as Follow needs it: telemetry in, a speed out."""

    telemetry = Signal(object)   # a follow_guide.Drone, each time the drone says where it is
    other_drone = Signal()       # this is another drone now, or none: nothing told before holds for it

    def why_not(self) -> str:
        """Why Follow cannot fly this drone at all (no drone, or not an ArduPilot multicopter). Empty: it can."""
        raise NotImplementedError

    def mode_name(self) -> str:
        """The name of its flight mode now, for a message."""
        raise NotImplementedError

    def ask_guided(self) -> None:
        """Ask the drone to change to Guided, the mode it takes speeds in."""
        raise NotImplementedError

    def send_speed(self, north_mps: float, east_mps: float, heading_deg: float) -> None:
        """A speed over the ground, north and east, m/s, and the heading to keep, degrees from north (NaN: the
        heading it has). The drone keeps its height."""
        raise NotImplementedError

    def flies_on_s(self) -> float:
        """How long the drone flies on after the last speed it got (GUID_TIMEOUT), seconds. Negative: not known."""
        return -1.0

    def has_drone(self) -> bool:
        """A drone's link is there (for the words when Follow ends on other_drone: another drone, or none)."""
        return True


class FollowLock(QObject):
    """The lock on the video, as far as Follow needs it."""

    lock_changed = Signal()
    # latitude, longitude, how sure (one standard deviation, metres), height of the drone above the object when the
    # laser measured it (NaN: level ground was assumed), how old the picture is (milliseconds)
    object_placed = Signal(float, float, float, float, int)

    def lock_active(self) -> bool:
        raise NotImplementedError

    def lock_seen(self) -> bool:
        """The lock sees the object in the last picture."""
        raise NotImplementedError

    def lock_by_us(self) -> bool:
        """The lock is VGCS's own tracker (a camera's own tracker gives no places to follow)."""
        return True

    def not_ours_text(self) -> str:
        return "Follow needs VGCS's own lock on the object."


class ObjectFollow(QObject):
    changed = Signal()            # something the screen shows has changed
    settings_changed = Signal()

    def __init__(self, lock: FollowLock, drone: FollowDrone, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._lock = lock
        self._drone = drone
        self._guide = fg.FollowGuide()
        self._clock = QElapsedTimer()
        self._clock.start()
        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)
        self._lock_was_active = False
        self._was_active = False
        self._message = ""
        self._ready = False
        self._not_ready_text = ""
        self._distance_m = -1.0
        self._height_m = -1.0
        self._by_laser = False
        self._told = ""
        drone.telemetry.connect(self._telemetry)
        drone.other_drone.connect(self._other_drone)
        lock.object_placed.connect(self._placed)
        lock.lock_changed.connect(self._lock_changed_slot)
        settings = QSettings(QS_ORG, QS_APP)
        settings.beginGroup(SETTINGS_GROUP)
        # A saved value that is no number gives the usual one; a number outside the range is brought into it.
        self._guide.set_speed_limit(_number(settings.value("speedLimit")))
        self._guide.set_range_limit(_number(settings.value("rangeLimit")))
        settings.endGroup()
        self._lock_changed_slot()

    # --- what the screen shows ---

    def active(self) -> bool:
        """Follow runs: the drone is flown by VGCS (following, or waiting for the object)."""
        return self._guide.state != State.OFF

    def waiting(self) -> bool:
        """It runs, but the drone is told to stand: the object is not seen, or its position is old."""
        return self._guide.state == State.WAITING

    def message(self) -> str:
        """What Follow does, why it does not start, or why it ended. Empty: nothing to say."""
        return self._message

    def ready(self) -> bool:
        """Follow would start now."""
        return self._ready

    def not_ready_text(self) -> str:
        """Why it would not: one sentence for the operator. Empty when ready."""
        return self._not_ready_text

    def distance_m(self) -> float:
        """Drone to object over the ground, metres, as VGCS works it out. Negative: not known."""
        return self._distance_m

    def kept_distance_m(self) -> float:
        return self._guide.kept_distance_m if self.active() else -1.0

    def height_m(self) -> float:
        """The drone's height above its take-off point, metres. Negative: not known."""
        return self._height_m

    def asked_speed_mps(self) -> float:
        return self._guide.asked_speed_mps() if self.active() else 0.0

    def by_laser(self) -> bool:
        """The object's place comes with a height above it that the laser measured (else level ground is assumed)."""
        return self._by_laser

    def flies_on_s(self) -> float:
        """How long the drone flies on if VGCS or the radio link stops, seconds. Negative: not known."""
        return self._drone.flies_on_s()

    @property
    def guide(self) -> fg.FollowGuide:
        return self._guide

    # --- the settings, saved between runs ---

    def speed_limit(self) -> float:
        return self._guide.speed_limit

    def set_speed_limit(self, mps: float) -> None:
        before = self._guide.speed_limit
        self._guide.set_speed_limit(mps)
        if self._guide.speed_limit != before:
            self._save("speedLimit", self._guide.speed_limit)

    def range_limit(self) -> float:
        return self._guide.range_limit

    def set_range_limit(self, metres: float) -> None:
        before = self._guide.range_limit
        self._guide.set_range_limit(metres)
        if self._guide.range_limit != before:
            self._save("rangeLimit", self._guide.range_limit)

    def _save(self, name: str, value: float) -> None:
        settings = QSettings(QS_ORG, QS_APP)
        settings.beginGroup(SETTINGS_GROUP)
        settings.setValue(name, value)
        settings.endGroup()
        self.settings_changed.emit()

    # --- the operator ---

    def start(self) -> None:
        """The operator starts Follow on the locked object. When it cannot start, message() says why."""
        if self.active():
            return
        why_not = self._drone.why_not()
        if why_not:
            self._set_message(why_not)
            return
        if self._lock.lock_active() and not self._lock.lock_by_us():
            self._set_message(self._lock.not_ours_text())
            return
        lock = self._lock.lock_active() and self._lock.lock_by_us()
        why = self._guide.start(lock and self._lock.lock_seen(), self._now())
        if why != Why.NONE:
            self._set_message(self.text(why))
            return
        if self._guide.last_drone.mode != Mode.GUIDED:
            self._drone.ask_guided()
        self._was_active = True
        self._keep_ticking()
        self._refresh()

    def stop(self) -> None:
        """The operator stops it. The drone is told to stop, and holds its place in Guided."""
        if not self.active():
            return
        self._guide.stop(Why.STOPPED, self._now())
        self._tick()  # the stop goes out now

    def clear_message(self) -> None:
        """The message was read."""
        if not self.active():
            self._set_message("")

    def refresh(self) -> None:
        """Work out now what the screen shows (ready, why not). The operator's click asks this first: without a
        lock the clock does not run, and a reason worked out before may be old."""
        self._refresh()

    def shutdown(self) -> None:
        """Never leave the drone flying a speed when VGCS closes."""
        if self.active():
            self._drone.send_speed(0.0, 0.0, math.nan)
            self._guide.drop(Why.STOPPED)
        self._timer.stop()

    # --- inside ---

    def _now(self) -> float:
        return self._clock.elapsed() / 1000.0

    def _telemetry(self, drone: fg.Drone) -> None:
        self._guide.drone(drone, self._now())
        if not self._timer.isActive():
            # Without a lock the clock does not run: the drone's data keeps "why not" current (a drone that has
            # just connected is no longer "no drone"). The screen is told only when something shown changes.
            self._refresh()

    def _other_drone(self) -> None:
        # Follow never goes on with another drone than the one it was started on. And it says nothing to the new
        # one, not even "stop": that drone may be doing something else.
        self._guide.drop(Why.DRONE_CHANGED)
        self._guide.forget_drone()  # where the last drone was says nothing of this one
        self._refresh()
        self._keep_ticking()

    def _placed(self, lat_deg: float, lon_deg: float, sigma_m: float, below_m: float, age_ms: int) -> None:
        self._guide.fix(fg.Fix(lat_deg, lon_deg, sigma_m, below_m), self._now() - max(0, age_ms) / 1000.0)
        self._by_laser = math.isfinite(below_m)

    def _lock_changed_slot(self) -> None:
        lock = self._lock.lock_active() and self._lock.lock_by_us()
        if lock != self._lock_was_active:
            self._lock_was_active = lock
            if not lock and self.active():
                self._tick()  # the lock ended: the drone is told to stop now, not on the next tick
            # The positions belong to the object that was locked, not to the next one.
            self._guide.forget_object()
            self._by_laser = False
        self._keep_ticking()
        self._refresh()

    def _keep_ticking(self) -> None:
        # While Follow runs, while the stop at its end goes out, and while a lock could be followed (so the screen
        # says whether Follow would start).
        needed = self.active() or self._guide.braking(self._now()) or self._lock_was_active
        if needed and not self._timer.isActive():
            self._timer.start()
        elif not needed and self._timer.isActive():
            self._timer.stop()

    def _tick(self) -> None:
        lock = self._lock.lock_active() and self._lock.lock_by_us()
        v = self._guide.tick(lock, lock and self._lock.lock_seen(), self._now())
        if v.send:
            self._drone.send_speed(v.north_mps, v.east_mps, v.heading_deg)
        self._refresh()
        self._keep_ticking()

    def _set_message(self, message: str) -> None:
        if message != self._message:
            self._message = message
            self._refresh()

    def text(self, why: Why) -> str:
        """One sentence for a reason. During Follow: what it does."""
        now = self._now()
        drone = self._guide.last_drone
        hold = "The drone holds its place in Guided."
        g = self._guide
        if why == Why.NONE:
            return f"Following. The object is {g.distance_m(now):.0f} m away." if self.active() else ""
        if why == Why.NO_LOCK:
            return "Follow needs a locked object that VGCS sees. Lock the object first."
        if why == Why.DRONE_SILENT:
            return ("Follow ended: the drone sends no data. It stops by itself." if self.active() or self._was_active
                    else "Follow cannot start: the drone sends no position.")
        if why == Why.NOT_FLYING:
            return ("Follow ended: the drone is not flying." if self.active() or self._was_active
                    else "Follow cannot start: the drone is not flying.")
        if why == Why.NO_GPS:
            return "Follow cannot start: the drone has no 3D GPS position."
        if why == Why.WRONG_MODE:
            return f"Follow starts from Loiter, Position Hold, Brake or Guided. The drone is in {self._drone.mode_name()}."
        if why == Why.TOO_LOW:
            if self._was_active:
                # Lower than it may be above the take-off point, or (the laser measured it) above the object.
                low = (f"Follow ended: the drone is less than {fg.END_HEIGHT_M:.0f} m above the object."
                       if drone.height_m >= fg.END_HEIGHT_M else f"Follow ended: the drone is lower than {fg.END_HEIGHT_M:.0f} m.")
                return f"{low} {hold}"
            below = g.below_m(now)
            if drone.height_m >= fg.MIN_HEIGHT_M and below >= 0.0:
                return (f"Follow cannot start: the drone is only {below:.0f} m above the object. "
                        f"It needs {fg.MIN_HEIGHT_M:.0f} m.")
            return f"Follow cannot start: the drone is {drone.height_m:.0f} m up. It needs {fg.MIN_HEIGHT_M:.0f} m."
        if why == Why.NO_FIX:
            return ("Follow cannot start yet: the object's position is not known well enough. Keep it locked for a "
                    "few seconds. If this stays, tilt the camera further down or fly nearer.")
        if why == Why.TOO_CLOSE:
            return (f"Follow cannot start: the object is {g.distance_m(now):.0f} m away. "
                    f"It needs {fg.START_DISTANCE_M:.0f} m or more.")
        if why == Why.NEAR:
            return f"The object is nearer than {fg.MIN_DISTANCE_M:.0f} m. The drone does not fly towards it."
        if why == Why.DRONE_LATE:
            return "Follow waits: the drone's data is late. The drone stands."
        if why == Why.NOT_SEEN:
            return "Follow waits: the object is not seen. The drone stands."
        if why == Why.FIX_OLD:
            return "Follow waits: no new position of the object. The drone stands."
        if why == Why.JUMPED:
            return "Follow waits: the object's position jumped. The drone stands until it is sure again."
        if why == Why.STOPPED:
            return f"Follow stopped. {hold}"
        if why == Why.LOCK_ENDED:
            return f"Follow ended: the lock ended. {hold}"
        if why == Why.NOT_GUIDED:
            return f"Follow ended: the drone did not change to Guided. It is in {self._drone.mode_name()}."
        if why == Why.PILOT_TOOK_OVER:
            return f"Follow ended: the flight mode was changed to {self._drone.mode_name()}."
        if why == Why.TOO_FAR:
            return f"Follow ended: the drone is {g.range_limit:.0f} m from where Follow started, the limit. {hold}"
        if why == Why.FIX_LOST:
            return f"Follow ended: the object was not found again. {hold}"
        if why == Why.NOT_OBEYED:
            return "Follow ended: the drone did not fly the speeds it was asked. Take control of the drone."
        if why == Why.DRONE_CHANGED:
            if not self._drone.has_drone():
                return "Follow ended: the link to the drone ended."
            return "Follow ended: another drone is the active one now."
        return ""

    def _refresh(self) -> None:
        now = self._now()
        is_active = self.active()
        if is_active:
            # What it does now: one line that changes with the state.
            self._message = self.text(self._guide.why)
            self._was_active = True
        elif self._was_active:
            # It has just ended: why. The line stays until it is read or Follow starts again.
            self._message = self.text(self._guide.why)
            self._was_active = False
        lock = self._lock.lock_active() and self._lock.lock_by_us()
        drone_age = self._guide.drone_age_s(now)
        self._height_m = self._guide.last_drone.height_m if 0.0 <= drone_age <= fg.DRONE_SILENT_S else -1.0
        self._distance_m = (self._guide.distance_m(now) if lock and (is_active or self._guide.object_known(now))
                            else -1.0)
        not_ready = ""
        if not is_active:
            not_ready = self._drone.why_not()
            if not not_ready and self._lock.lock_active() and not self._lock.lock_by_us():
                not_ready = self._lock.not_ours_text()
            if not not_ready:
                why = self._guide.would_start(lock and self._lock.lock_seen(), now)
                not_ready = "" if why == Why.NONE else self.text(why)
        self._ready = not is_active and not not_ready
        self._not_ready_text = not_ready
        # The screen is told when something it shows has changed: whole metres, tenths of a metre a second.
        told = "|".join(str(x) for x in (
            int(is_active), int(self.waiting()), self._message, int(self._ready), self._not_ready_text,
            _rounded(self._distance_m), _rounded(self.kept_distance_m()), _rounded(self._height_m),
            _rounded(self.asked_speed_mps() * 10.0), int(self._by_laser), _rounded(self.flies_on_s() * 10.0)))
        if told != self._told:
            self._told = told
            self.changed.emit()


def _number(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _rounded(value: float) -> int:
    """qRound: half away from zero. Not a number (a height the drone did not send, say): -1, not known."""
    if not math.isfinite(value):
        return -1
    return math.floor(value + 0.5) if value >= 0 else -math.floor(-value + 0.5)
