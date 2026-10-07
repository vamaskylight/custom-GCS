"""The speed the drone flies each leg of a mission at, kept to what the plan says.

Measured in the simulator on 2026-10-07 (ArduCopter 4.6.2 and 4.7.0, the same):

- Entering AUTO forgets the speed the mission had set. After Pause and Resume
  the drone flew 10 m/s, its own default, where the plan said 4 m/s. The speed
  item is not run again, because the drone goes on from the waypoint it was
  flying to. The mode list or an RC switch out of AUTO and back does the same.
- A jump to another waypoint skips the speed items in between. A jump back to
  a 4 m/s waypoint was flown at 9 m/s.
- A speed command (DO_CHANGE_SPEED) is accepted in AUTO and refused while the
  drone holds in BRAKE (result 4).
- A speed changed while a leg is already starting is overshot: with the speed
  set 0.2 s after Resume, a 2 m/s leg was still flown at up to 6.3 m/s for
  some seconds. A leg that starts after the speed was set has no overshoot.

So:

- For a jump, the target waypoint's speed is sent first, then the jump.
- After a return to AUTO, the speed of the waypoint the drone says it is flying
  to is sent, and then that same leg is started again, where that is safe.

"Started again" is MISSION_SET_CURRENT with the item the drone itself has just
reported. It is not sent:

- when a payload release or any other command sits between the waypoint before
  and this one: the drone drops what is left of such a sequence (the servo
  would stay open),
- when the drone is close to the waypoint, where it could send it back to a
  waypoint it has reached in the meantime,
- when the position is not known.

Nothing here is sent for a mission VGCS does not know. The plan is dropped when
the drone reports another number of mission items than the plan has, which is
what a mission sent from another ground station looks like.

Pure Python, no Qt and no pymavlink. The link thread feeds in what it hears
and calls tick() on every pass of its loop.
"""

from __future__ import annotations

import math
from typing import Callable

from vgcs.link.confirmations import CommandAck, MissionJump

DO_CHANGE_SPEED = 178

# A leg is only started again further than this from its waypoint: the larger
# of a distance and a flying time at the planned speed.
RESTART_MIN_DISTANCE_M = 20.0
RESTART_MIN_TIME_S = 4.0
# The drone's next MISSION_CURRENT after the speed was accepted must come
# within this time, or the leg is left alone.
RESTART_WAIT_S = 2.0
# Another item count than the plan's has to be reported for this long before
# the plan is dropped. One odd message is not a new mission.
MISMATCH_S = 1.0

# MISSION_CURRENT.mission_state values that mean the firmware filled in the
# count of items at all (0 is "unknown": an older firmware sends nothing).
_STATES_WITH_A_COUNT = (1, 2, 3, 4, 5)


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north = (lat2 - lat1) * 111_320.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


class LegSpeed:
    """Keeps the planned speed across jumps and returns to AUTO. See the module text."""

    def __init__(
        self,
        *,
        send_speed: Callable[[float], None],
        send_jump: Callable[[int], None],
        say: Callable[[str], None],
        tell: Callable[[bool, str], None],
        get_plan: Callable[[], object],
        is_armed: Callable[[], bool],
        plan_lost: Callable[[str], None] | None = None,
    ) -> None:
        self._send_speed = send_speed
        self._send_jump = send_jump
        self._say = say                    # a line for the log
        self._tell = tell                  # (ok, text) the operator must see
        self._get_plan = get_plan          # the mission VGCS knows the drone holds, or None
        self._is_armed = is_armed
        self._plan_lost = plan_lost
        self.mode = ""
        self.current: int | None = None
        self.position: tuple[float, float] | None = None
        self._auto_entered = False
        self._speed: CommandAck | None = None
        self._speed_mps = 0.0
        self._jump: MissionJump | None = None
        self._restart: tuple[int, float, float] | None = None    # (item, speed, asked at)
        self._mismatch_since: float | None = None
        self._now = 0.0                    # the time of the last thing heard

    # --- what the link thread tells it ---------------------------------------

    @property
    def plan(self):
        return self._get_plan()

    @property
    def armed(self) -> bool:
        return bool(self._is_armed())

    def mission_changed(self) -> None:
        """Another mission was uploaded or downloaded: nothing waits for the old one."""
        self._mismatch_since = None
        self._restart = None
        self._auto_entered = False

    def heard_mode(self, mode: str) -> None:
        """The mode in a heartbeat from the flight controller."""
        name = str(mode or "").strip()
        if not name:
            return
        before, self.mode = self.mode, name
        if name != "AUTO":
            self._auto_entered = False
            self._restart = None
        elif before and before != "AUTO":
            self._auto_entered = True

    def heard_position(self, lat: float, lon: float) -> None:
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return
        self.position = (float(lat), float(lon))

    def heard_current(self, seq: int, now: float, total: int | None = None, state: int | None = None) -> None:
        """A MISSION_CURRENT from the flight controller: the item it is flying to."""
        try:
            current = int(seq)
        except (TypeError, ValueError):
            return
        self._now = float(now)
        self.current = current
        self._check_the_plan_is_the_drones(now, total, state)
        if self._jump is not None:
            self._jump.heard_current(current)
            if self._jump is not None and self._jump.done:
                self._jump = None
            if self._jump is not None:
                return            # a jump is on its way: which item is current is not settled
        restart, self._restart = self._restart, None
        if restart is not None:
            self._start_leg_again(restart, current, now)
        if self._auto_entered:
            self._auto_entered = False
            self._back_in_auto(current, now)

    def heard_ack(self, command: int, result: int, now: float) -> None:
        """A COMMAND_ACK from the flight controller."""
        self._now = float(now)
        request = self._speed
        if request is None:
            return
        request.heard_ack(command, result)
        if request.done and self._speed is request:
            self._speed = None

    def tick(self, now: float) -> None:
        """Send again what got no answer, and settle what ran out of time."""
        self._now = float(now)
        request = self._speed
        if request is not None:
            if request.tick(now):
                self._try(self._send_speed, self._speed_mps)
                self._say("Mission speed: no answer yet, sent again")
            if request.done and self._speed is request:
                self._speed = None
        jump = self._jump
        if jump is not None:
            if jump.tick(now):
                self._try(self._send_jump, jump.seq)
                self._say(f"Mission jump: item {jump.seq} not shown by the drone yet, sent again")
            if jump.done and self._jump is jump:
                self._jump = None
        if self._restart is not None and now - self._restart[2] > RESTART_WAIT_S:
            self._restart = None

    def open(self) -> bool:
        """True while something waits for the drone's answer."""
        return self._speed is not None or self._jump is not None or self._restart is not None

    def cancel(self, reason: str) -> None:
        """The link is gone: whatever waits is over."""
        speed, self._speed = self._speed, None
        jump, self._jump = self._jump, None
        self._restart = None
        self._auto_entered = False
        for waiting in (speed, jump):
            if waiting is not None and not waiting.done:
                waiting.done = True
                waiting.on_done(False, reason)

    # --- the operator's jump --------------------------------------------------

    def jump(self, wp_index: int, now: float, on_done: Callable[[bool, str], None]) -> None:
        """Fly to a 0-based waypoint now. on_done(ok, text) runs once, when the drone shows it."""
        number = int(wp_index) + 1
        self._now = float(now)
        plan = self.plan
        if plan is None:
            on_done(False, "VGCS does not know the mission on this drone. Upload or download it first.")
            return
        seq = plan.seq_for_waypoint_index(int(wp_index))
        if seq is None:
            on_done(False, f"WP {number} is not in the mission")
            return
        # A newer request wins. The older one is not reported as failed, since
        # the operator moved on, not the drone.
        for old in (self._jump, self._speed):
            if old is not None:
                old.done = True
        self._jump = None
        self._speed = None
        self._restart = None
        speed = plan.speed_for_seq(seq)

        def shown(ok: bool, detail: str) -> None:
            if not ok:
                on_done(False, f"WP {number} not taken: {detail}")
                return
            self._say(f"Mission jump confirmed: WP {number} is mission item {seq}")
            # The texts start with what matters: the header shows only their beginning.
            if self._flying_the_mission():
                text = f"Flying to WP {number}"
                if speed is not None and sent_speed:
                    text += f" at {speed:.1f} m/s"
            else:
                text = f"WP {number} is next. The drone is in {self.mode or 'another mode'}: press Resume to fly there"
            on_done(True, text)

        def send_the_jump() -> None:
            jump = MissionJump(seq, now, shown)
            self._jump = jump
            try:
                self._send_jump(seq)
            except Exception as e:
                jump.done = True
                self._jump = None
                shown(False, f"could not send it: {e}")
                return
            self._say(f"Mission jump: WP {number} is mission item {seq}, waiting for the drone to show it")

        sent_speed = bool(self._flying_the_mission() and speed is not None)
        if not sent_speed:
            # Not in AUTO: the drone takes the jump while it holds but refuses
            # a speed there. The speed follows when it is back in AUTO.
            send_the_jump()
            return
        # In AUTO the jump itself is the fresh start of a leg, so nothing is
        # left to do after a return to AUTO that has not been dealt with yet.
        self._auto_entered = False

        def speed_answered(ok: bool, detail: str) -> None:
            if not ok:
                on_done(False, f"WP {number} not taken: the drone did not accept the speed for it ({detail})")
                return
            self._say(f"Mission speed {speed:.1f} m/s set for the jump to WP {number}")
            send_the_jump()

        self._ask_for_speed(float(speed), now, speed_answered)

    # --- the parts -------------------------------------------------------------

    def _flying_the_mission(self) -> bool:
        return self.plan is not None and self.armed and self.mode == "AUTO"

    @staticmethod
    def _try(send, value) -> None:
        try:
            send(value)
        except Exception:
            pass

    def _ask_for_speed(self, speed: float, now: float, on_done: Callable[[bool, str], None]) -> None:
        old = self._speed
        if old is not None:
            old.done = True
        request = CommandAck(DO_CHANGE_SPEED, now, on_done)
        self._speed = request
        self._speed_mps = float(speed)
        try:
            self._send_speed(float(speed))
        except Exception as e:
            request.done = True
            self._speed = None
            on_done(False, f"could not send it: {e}")

    def _back_in_auto(self, seq: int, now: float) -> None:
        """The drone is in AUTO again and has said which item it is flying to."""
        if not self._flying_the_mission():
            return
        plan = self.plan
        speed = plan.speed_for_seq(seq)
        if speed is None:
            return                # the take-off, the final return, or no speed in the plan
        index = plan.waypoint_index_for_seq(seq)
        name = f"WP {index + 1}" if index is not None else f"mission item {seq}"

        def answered(ok: bool, detail: str) -> None:
            if not ok:
                self._tell(
                    False,
                    f"{name}: planned {speed:.1f} m/s NOT set again ({detail}). "
                    "The drone may fly at its own default speed",
                )
                return
            self._tell(True, f"{name}: planned {speed:.1f} m/s set again after the return to AUTO")
            if plan.leg_can_start_again(seq):
                # Decided at the drone's next report, so the item is fresh.
                self._restart = (int(seq), float(speed), self._now)

        self._ask_for_speed(float(speed), now, answered)

    def _start_leg_again(self, restart: tuple[int, float, float], current: int, now: float) -> None:
        """Start the leg again, now that the speed is set: no overshoot (see the module text)."""
        seq, speed, _asked = restart
        plan = self.plan
        if current != seq or not self._flying_the_mission() or plan is None or self.position is None:
            return
        where = plan.position_for_seq(seq)
        if where is None:
            return
        left = _distance_m(self.position[0], self.position[1], where[0], where[1])
        if left < max(RESTART_MIN_DISTANCE_M, RESTART_MIN_TIME_S * speed):
            return
        try:
            self._send_jump(seq)
        except Exception:
            return
        self._say(f"Mission: the leg to mission item {seq} started again at {speed:.1f} m/s ({left:.0f} m to go)")

    def _check_the_plan_is_the_drones(self, now: float, total: int | None, state: int | None) -> None:
        plan = self.plan
        if plan is None or total is None or state is None or not plan.items:
            self._mismatch_since = None
            return
        try:
            total_i, state_i = int(total), int(state)
        except (TypeError, ValueError):
            return
        if state_i not in _STATES_WITH_A_COUNT or total_i == int(plan.items[-1].seq):
            self._mismatch_since = None
            return
        if self._mismatch_since is None:
            self._mismatch_since = float(now)
            return
        if now - self._mismatch_since < MISMATCH_S:
            return
        have = int(plan.items[-1].seq)
        self.mission_changed()
        if self._plan_lost is not None:
            self._plan_lost(
                f"The mission on the drone changed (it reports {total_i} items, VGCS knows {have}). "
                "Press Download in Plan Flight"
            )
