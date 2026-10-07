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

Is the plan still the drone's mission?

Nothing here is sent for a mission VGCS does not know. And VGCS cannot take
its plan for the drone's mission on trust. Measured on 2026-10-08:

- The drone tells only the ground station that sent a mission that it has a
  new one ("Flight plan received" goes to that link alone). A mission sent by
  another station over another link goes unnoticed here.
- MISSION_CURRENT reports the number of items, so a mission of another size is
  noticed, and the plan is dropped. A mission of the same size is not noticed.
- The drone answers a request for one mission item at any time, in a few ms.

So before anything is sent for a waypoint, the items it depends on are read
from the drone and compared with the plan (MissionPlan.items_to_check_for_seq
and same_item): the waypoint, the speed item that sets its speed, and what
sits between the waypoint before and it.

- Not the same: the plan is dropped, nothing is sent, and the operator is told.
- No answer: nothing is sent either, and the operator is told that.

Pure Python, no Qt and no pymavlink. The link thread feeds in what it hears
and calls tick() on every pass of its loop.
"""

from __future__ import annotations

import math
from typing import Callable

from vgcs.link.confirmations import CommandAck, ItemsRead, MissionJump

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

# Why something that waited for the drone is over, when the mission was replaced.
_MISSION_CHANGED = "the mission on the drone changed"
_NOT_THE_ONE = "mission on the drone is not the one VGCS knows"
_DOWNLOAD = "Press Download in Plan Flight"


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north = (lat2 - lat1) * 111_320.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


class _JumpRequest:
    """One "fly to this waypoint now", from the operator's click to its one result."""

    def __init__(self, number: int, on_done: Callable[[bool, str], None]) -> None:
        self.number = int(number)
        self.on_done = on_done
        self.speed_sent = False
        self.sent = False              # MISSION_SET_CURRENT has gone out
        self.over = False

    def result(self, ok: bool, text: str) -> None:
        if self.over:
            return
        self.over = True
        self.on_done(ok, text)

    def interrupted(self, reason: str) -> None:
        """It cannot be finished: the link is gone, or the mission is another one."""
        if self.sent:
            self.result(
                False,
                f"WP {self.number}: the jump was sent, then {reason}. VGCS cannot say whether the drone took it",
            )
        else:
            self.result(False, f"WP {self.number} not taken: {reason}")


class LegSpeed:
    """Keeps the planned speed across jumps and returns to AUTO. See the module text."""

    def __init__(
        self,
        *,
        send_speed: Callable[[float], None],
        send_jump: Callable[[int], None],
        ask_item: Callable[[int], None],
        say: Callable[[str], None],
        tell: Callable[[bool, str], None],
        get_plan: Callable[[], object],
        is_armed: Callable[[], bool],
        plan_lost: Callable[[str], None] | None = None,
    ) -> None:
        self._send_speed = send_speed
        self._send_jump = send_jump
        self._ask_item = ask_item          # ask the drone for one mission item
        self._say = say                    # a line for the log
        self._tell = tell                  # (ok, text) the operator must see
        self._get_plan = get_plan          # the mission VGCS knows the drone holds, or None
        self._is_armed = is_armed
        self._plan_lost = plan_lost
        self.mode = ""
        self.current: int | None = None
        self.position: tuple[float, float] | None = None
        self._auto_entered = False
        self._read: ItemsRead | None = None
        self._speed: CommandAck | None = None
        self._speed_mps = 0.0
        self._jump: MissionJump | None = None
        self._jumping: _JumpRequest | None = None                # the operator's jump, until its result
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
        """Another mission was uploaded or downloaded, or the plan was dropped.

        Nothing waits for the old one any more. A jump that was under way is
        reported as not done, since its waypoint number meant the old mission.
        """
        self._mismatch_since = None
        self._auto_entered = False
        self._end_what_waits(_MISSION_CHANGED)

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
        jump = self._jump
        if jump is not None:
            jump.heard_current(current)
            if jump.done and self._jump is jump:
                self._jump = None
        if self._jumping is not None:
            return            # a jump is under way: which item is current is not settled
        restart, self._restart = self._restart, None
        if restart is not None:
            self._after_the_speed(restart, current, now)
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

    def heard_item(self, seq: int, item: object, now: float) -> None:
        """One mission item from the flight controller, as a download gives it."""
        self._now = float(now)
        read = self._read
        if read is not None:
            read.heard_item(seq, item)

    def heard_item_refused(self, result: int, now: float) -> None:
        """A MISSION_ACK with an error: the drone does not give an item that was asked for."""
        self._now = float(now)
        read = self._read
        if read is not None:
            read.heard_refusal(result)

    def reading(self) -> bool:
        """True while mission items are asked for and have not all come."""
        return self._read is not None

    def tick(self, now: float) -> None:
        """Send again what got no answer, and settle what ran out of time."""
        self._now = float(now)
        read = self._read
        if read is not None:
            again = read.tick(now)
            for seq in again:
                self._try(self._ask_item, seq)
            if again:
                self._say("Mission check: no answer yet, asked again")
            if read.done and self._read is read:
                self._read = None
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
        return (
            self._read is not None
            or self._speed is not None
            or self._jump is not None
            or self._restart is not None
        )

    def cancel(self, reason: str) -> None:
        """The link is gone: whatever waits is over."""
        self._auto_entered = False
        self._end_what_waits(reason)

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
        self._drop_quietly()
        request = _JumpRequest(number, on_done)
        self._jumping = request
        speed = plan.speed_for_seq(seq)

        def over(ok: bool, text: str) -> None:
            if self._jumping is request:
                self._jumping = None
            request.result(ok, text)

        def shown(ok: bool, detail: str) -> None:
            if not ok:
                over(False, f"WP {number} not taken: {detail}")
                return
            self._say(f"Mission jump confirmed: WP {number} is mission item {seq}")
            # The texts start with what matters: the header shows only their beginning.
            if self._flying_the_mission():
                text = f"Flying to WP {number}"
                if speed is not None and request.speed_sent:
                    text += f" at {speed:.1f} m/s"
            else:
                text = f"WP {number} is next. The drone is in {self.mode or 'another mode'}: press Resume to fly there"
            over(True, text)

        def send_the_jump() -> None:
            jump = MissionJump(seq, self._now, shown)
            self._jump = jump
            try:
                self._send_jump(seq)
            except Exception as e:
                jump.done = True
                self._jump = None
                shown(False, f"could not send it: {e}")
                return
            request.sent = True
            self._say(f"Mission jump: WP {number} is mission item {seq}, waiting for the drone to show it")

        def speed_answered(ok: bool, detail: str) -> None:
            if not ok:
                over(False, f"WP {number} not taken: the drone did not accept the speed for it ({detail})")
                return
            self._say(f"Mission speed {speed:.1f} m/s set for the jump to WP {number}")
            send_the_jump()

        def checked(verdict: str, detail: str) -> None:
            if verdict == "changed":
                # The waypoint number meant another mission than the drone holds.
                if self._jumping is request:
                    self._jumping = None
                self._lose_the_plan(f"The {_NOT_THE_ONE} ({detail}). {_DOWNLOAD}")
                request.result(False, f"WP {number} not taken: the {_NOT_THE_ONE}. {_DOWNLOAD}")
                return
            if verdict == "silent":
                over(False, f"WP {number} not taken: VGCS could not check the waypoint with the drone ({detail})")
                return
            if verdict == "stopped":
                over(False, f"WP {number} not taken: {detail}")
                return
            if self.plan is not plan:
                over(False, f"WP {number} not taken: {_MISSION_CHANGED}")
                return
            if self._flying_the_mission() and speed is not None:
                # In AUTO the jump itself is the fresh start of a leg, so nothing
                # is left to do after a return to AUTO that has not been dealt
                # with yet.
                self._auto_entered = False
                request.speed_sent = True
                self._ask_for_speed(float(speed), self._now, speed_answered)
                return
            # Not in AUTO: the drone takes the jump while it holds but refuses
            # a speed there. The speed follows when it is back in AUTO.
            send_the_jump()

        self._check_with_the_drone(plan, seq, now, checked)

    # --- the parts -------------------------------------------------------------

    def _flying_the_mission(self) -> bool:
        return self.plan is not None and self.armed and self.mode == "AUTO"

    @staticmethod
    def _try(send, value) -> None:
        try:
            send(value)
        except Exception:
            pass

    def _drop_quietly(self) -> None:
        """A newer request takes the place of whatever waits. Nobody is told:
        the operator moved on, not the drone."""
        for stage in (self._read, self._speed, self._jump):
            if stage is not None:
                stage.done = True
        self._read = self._speed = self._jump = None
        self._restart = None
        old, self._jumping = self._jumping, None
        if old is not None:
            old.over = True

    def _end_what_waits(self, reason: str) -> None:
        """Whatever waits for the drone is over, and whoever asked is told why."""
        stages = [s for s in (self._read, self._speed, self._jump) if s is not None and not s.done]
        self._read = self._speed = self._jump = None
        self._restart = None
        request, self._jumping = self._jumping, None
        if request is not None:
            # The stages of a jump say nothing each: the jump says it in one piece.
            for stage in stages:
                stage.done = True
            request.interrupted(reason)
            return
        for stage in stages:
            stage.give_up(reason)

    def _lose_the_plan(self, text: str) -> None:
        """The mission on the drone is not the plan's any more: nothing of the plan is used."""
        self.mission_changed()
        if self._plan_lost is not None:
            self._plan_lost(text)

    def _check_with_the_drone(self, plan, seq: int, now: float, on_done: Callable[[str, str], None]) -> None:
        """Read the items the leg to ``seq`` depends on from the drone, and compare them with the plan.

        on_done(verdict, detail) runs once. The verdict is "same", "changed"
        (detail says what is different), "silent" (no answer came, detail says
        how that showed) or "stopped" (the link is gone or the mission was
        replaced, detail says which). See "Is the plan still the drone's
        mission?" in the module text.
        """
        wanted = plan.items_to_check_for_seq(seq)

        def answered(items: dict | None, refused: bool, detail: str) -> None:
            if self._read is read:
                self._read = None
            if items is None:
                on_done("stopped" if read.given_up else "changed" if refused else "silent", detail)
                return
            for number in wanted:
                if not plan.same_item(number, items.get(number)):
                    on_done("changed", f"mission item {number} is different")
                    return
            self._say(
                "Mission check: the drone still holds what VGCS knows "
                f"(mission items read: {', '.join(str(n) for n in wanted)})"
            )
            on_done("same", "")

        read = ItemsRead(wanted, now, answered)
        old, self._read = self._read, read
        if old is not None:
            old.done = True
        try:
            for number in wanted:
                self._ask_item(number)
        except Exception as e:
            read.done = True
            self._read = None
            on_done("silent", f"the request could not be sent: {e}")

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
        # A return that was still being dealt with is replaced by this one.
        for old in (self._read, self._speed):
            if old is not None:
                old.done = True
        self._read = self._speed = None
        self._restart = None

        def not_set(detail: str) -> None:
            self._tell(
                False,
                f"{name}: planned {speed:.1f} m/s NOT set again ({detail}). "
                "The drone may fly at its own default speed",
            )

        def answered(ok: bool, detail: str) -> None:
            if not ok:
                not_set(detail)
                return
            self._tell(True, f"{name}: planned {speed:.1f} m/s set again after the return to AUTO")
            # What comes next is decided at the drone's next report, so the item is fresh.
            self._restart = (int(seq), float(speed), self._now)

        def checked(verdict: str, detail: str) -> None:
            if verdict == "changed":
                self._lose_the_plan(f"The {_NOT_THE_ONE} ({detail}). {_DOWNLOAD}")
                # The plan's name for this item may not be the drone's any more.
                self._tell(False, f"Planned speed NOT set again: the {_NOT_THE_ONE}. {_DOWNLOAD}")
                return
            if verdict == "silent":
                not_set(f"VGCS could not check the waypoint with the drone, {detail}")
                return
            if verdict == "stopped":
                not_set(detail)
                return
            if not self._flying_the_mission() or self.plan is not plan:
                return            # paused again, or another mission, while the answer was on its way
            if self.current is not None and self.current != seq:
                # The drone reached that waypoint in the meantime: the leg it
                # flies now is another one.
                self._back_in_auto(self.current, self._now)
                return
            self._ask_for_speed(float(speed), self._now, answered)

        self._check_with_the_drone(plan, seq, now, checked)

    def _after_the_speed(self, restart: tuple[int, float, float], current: int, now: float) -> None:
        """The drone's first report after the speed was set: start the leg again, or follow the drone.

        Started again, the leg has no overshoot (see the module text).
        """
        seq, speed, _asked = restart
        plan = self.plan
        if not self._flying_the_mission() or plan is None:
            return
        if current != seq:
            # It reached that waypoint in the meantime and flies another leg.
            # Where that leg has another planned speed, the speed just sent is
            # the wrong one for it.
            other = plan.speed_for_seq(current)
            if other is not None and abs(other - speed) > 1e-6:
                self._back_in_auto(current, now)
            return
        if not plan.leg_can_start_again(seq) or self.position is None:
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
        self._lose_the_plan(
            f"The mission on the drone changed (it reports {total_i} items, VGCS knows {have}). "
            f"{_DOWNLOAD}"
        )
