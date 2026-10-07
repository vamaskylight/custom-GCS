"""Requests to the drone that are only "done" once the drone says so.

Found by the simulator test (M17, 2026-10-07). VGCS reported three things as
done that the drone had not done:

- A setting written with PARAM_SET was reported "OK" the moment it was sent.
  ArduPilot answers a write with PARAM_VALUE, and says nothing at all (4.6) or
  sends a PARAM_ERROR that pymavlink cannot read (4.7) for a name it does not
  have. So "WPNAV_SPEED OK" was shown on a 4.7 drone, which has no such setting.
- A mode change was reported as done when it was sent. ArduCopter refuses
  LOITER without a position, and says so in a STATUSTEXT
  ("Mode change to LOITER failed: requires position").
- Reading settings stopped the link for a second per name and threw away
  everything else the drone sent in that time, positions and messages too.

Each request here is answered by what the drone sends back, and gives up after
a few tries with a reason. Nothing here waits: the link thread feeds in what it
hears and calls tick() on every pass of its loop, so the position, the
heartbeat and the drone's messages keep flowing while a request is open.

Pure Python, no Qt and no pymavlink, so it is tested without a link.
"""

from __future__ import annotations

from typing import Callable

# One PARAM_SET is sent again if no echo came back in this time, up to
# PARAM_WRITE_TRIES sends. A radio link loses single packets, so one silence is
# not yet a refusal.
PARAM_WRITE_WAIT_S = 1.5
PARAM_WRITE_TRIES = 3

# Reading: every name is asked for, then the missing ones once more. A name
# still missing after that is not on this drone (or the link is very bad).
PARAM_READ_WAIT_S = 2.5
PARAM_READ_ROUNDS = 2

# ArduCopter sends its heartbeat once a second, so a mode it entered shows
# within about a second. One more SET_MODE goes out if neither a heartbeat in
# the new mode nor a refusal was heard by MODE_RESEND_S.
MODE_RESEND_S = 1.5
MODE_GIVE_UP_S = 4.0
# pymavlink sends a mode change as DO_SET_MODE, which ArduPilot answers at once
# with an ack. A refusal's reason follows in a STATUSTEXT a moment later, so a
# refusing ack waits this long for the words before it is reported bare.
MODE_REASON_WAIT_S = 1.0

# float32 on the wire: 0.1 comes back as 0.10000000149. Anything closer than
# this is the value that was sent.
_SAME_VALUE_REL = 1e-5
_SAME_VALUE_ABS = 1e-6


def same_value(sent: float, got: float) -> bool:
    return abs(float(got) - float(sent)) <= max(_SAME_VALUE_ABS, _SAME_VALUE_REL * abs(float(sent)))


def clean_name(name: str) -> str:
    """Parameter names are upper case and at most 16 characters on the wire."""
    return str(name or "").strip().strip("\x00").upper()[:16]


def show_value(value: float) -> str:
    """A setting's value as the operator would type it: 4.0 shows as 4, 0.1 as 0.1."""
    return f"{float(value):.6g}"


class _WriteGroup:
    def __init__(self, wanted: dict[str, float], on_done: Callable[[dict[str, tuple[bool, str]]], None]) -> None:
        self.wanted = dict(wanted)
        self.results: dict[str, tuple[bool, str]] = {}
        self.on_done = on_done

    def settle(self, name: str, ok: bool, detail: str) -> None:
        if name in self.results:
            return
        self.results[name] = (ok, detail)
        if len(self.results) == len(self.wanted):
            self.on_done(dict(self.results))


class ParamWrites:
    """Settings sent to the drone and not yet echoed back.

    A group is settings that belong together (a fence is five of them). Its
    callback gets {name: (ok, detail)} once every one of them is settled.
    detail is the value the drone now holds, written as a number, or the reason
    it is not written.
    """

    def __init__(self, wait_s: float = PARAM_WRITE_WAIT_S, tries: int = PARAM_WRITE_TRIES) -> None:
        self.wait_s = float(wait_s)
        self.tries = max(1, int(tries))
        # name -> [value, last sent, sends so far, group]
        self._open: dict[str, list] = {}

    def start(self, values: dict[str, float], now: float,
              on_done: Callable[[dict[str, tuple[bool, str]]], None]) -> list[tuple[str, float]]:
        """Open a group. Returns the (name, value) pairs to send now."""
        wanted = {clean_name(k): float(v) for k, v in values.items() if clean_name(k)}
        group = _WriteGroup(wanted, on_done)
        if not wanted:
            on_done({})
            return []
        for name, value in wanted.items():
            old = self._open.get(name)
            if old is not None:
                # A newer write of the same setting replaces the old one. The
                # old request is answered now, so its caller is not left hanging.
                old[3].settle(name, False, f"replaced by a newer write of {show_value(value)}")
            self._open[name] = [value, now, 1, group]
        return list(wanted.items())

    def heard(self, name: str, value: float) -> bool:
        """The drone sent PARAM_VALUE. True when it answered an open write."""
        name = clean_name(name)
        entry = self._open.pop(name, None)
        if entry is None:
            return False
        wanted, _sent, _tries, group = entry
        if same_value(wanted, value):
            group.settle(name, True, show_value(value))
        else:
            # Read-only settings, and whole numbers given a fraction, come back
            # with the value the drone kept. That is a refusal, not a write.
            group.settle(name, False, f"the drone kept {show_value(value)} (sent {show_value(wanted)})")
        return True

    def tick(self, now: float) -> list[tuple[str, float]]:
        """Settle what has run out of tries. Returns the (name, value) pairs to send again."""
        again: list[tuple[str, float]] = []
        for name in list(self._open):
            entry = self._open[name]
            value, sent, tries, group = entry
            if now - sent < self.wait_s:
                continue
            if tries < self.tries:
                entry[1] = now
                entry[2] = tries + 1
                again.append((name, value))
                continue
            del self._open[name]
            group.settle(
                name, False,
                f"the drone did not confirm it after {tries} tries. This firmware may not have "
                f"{name}, or the link is losing messages",
            )
        return again

    def pending(self) -> list[str]:
        return list(self._open)

    def cancel_all(self, reason: str) -> None:
        for name in list(self._open):
            entry = self._open.pop(name)
            entry[3].settle(name, False, reason)


class ParamReads:
    """Settings asked for and not yet answered. Several reads may be open at once."""

    def __init__(self, wait_s: float = PARAM_READ_WAIT_S, rounds: int = PARAM_READ_ROUNDS) -> None:
        self.wait_s = float(wait_s)
        self.rounds = max(1, int(rounds))
        # each: [wanted names (ordered), got {name: value}, round started, round number, on_done]
        self._open: list[list] = []

    def start(self, names: list[str], now: float,
              on_done: Callable[[dict[str, float], list[str]], None]) -> list[str]:
        """Open a read. Returns the names to ask for now.

        on_done(values, missing) runs once: values the drone sent, and the
        names it never answered.
        """
        wanted: list[str] = []
        for raw in names:
            name = clean_name(raw)
            if name and name not in wanted:
                wanted.append(name)
        if not wanted:
            on_done({}, [])
            return []
        self._open.append([wanted, {}, now, 1, on_done])
        return list(wanted)

    def heard(self, name: str, value: float) -> bool:
        """The drone sent PARAM_VALUE. True when an open read wanted it."""
        name = clean_name(name)
        used = False
        for entry in list(self._open):
            wanted, got = entry[0], entry[1]
            if name in wanted:
                got[name] = float(value)
                used = True
                if len(got) == len(wanted):
                    self._open.remove(entry)
                    entry[4](dict(got), [])
        return used

    def tick(self, now: float) -> list[str]:
        """Finish the reads that ran out of time. Returns the names to ask for again."""
        again: list[str] = []
        for entry in list(self._open):
            wanted, got, started, round_no, on_done = entry
            if now - started < self.wait_s:
                continue
            missing = [n for n in wanted if n not in got]
            if round_no < self.rounds:
                entry[2] = now
                entry[3] = round_no + 1
                again.extend(n for n in missing if n not in again)
                continue
            self._open.remove(entry)
            on_done(dict(got), missing)
        return again

    def pending(self) -> list[str]:
        out: list[str] = []
        for entry in self._open:
            out.extend(n for n in entry[0] if n not in entry[1] and n not in out)
        return out

    def cancel_all(self) -> None:
        for entry in list(self._open):
            self._open.remove(entry)
            entry[4](dict(entry[1]), [n for n in entry[0] if n not in entry[1]])


def _mode_key(name: str) -> str:
    """ArduPilot says SMARTRTL and "AUTO RTL" where pymavlink says SMART_RTL and AUTO_RTL."""
    return "".join(ch for ch in str(name or "").upper() if ch.isalnum())


def mode_refusal(text: str) -> tuple[str, str] | None:
    """(mode, reason) when ArduCopter refuses a mode, or None for any other text.

    "Mode change to LOITER failed: requires position" gives
    ("LOITER", "requires position").
    """
    line = str(text or "").strip()
    low = line.lower()
    prefix = "mode change to "
    if not (low.startswith(prefix) and " failed" in low):
        return None
    cut = low.index(" failed")
    mode = line[len(prefix):cut].strip()
    reason = line[cut + len(" failed"):].lstrip(": ").strip()
    return mode, reason or "the drone refused it"


class ModeChange:
    """One mode change waiting for the drone's heartbeat to show it.

    on_done(ok, detail) runs exactly once: when a heartbeat shows the mode,
    when the drone refuses it, or when it gives up.
    """

    def __init__(self, mode: str, custom_mode: int | None, now: float,
                 on_done: Callable[[bool, str], None],
                 resend_s: float = MODE_RESEND_S, give_up_s: float = MODE_GIVE_UP_S) -> None:
        self.mode = str(mode)
        self.custom_mode = None if custom_mode is None else int(custom_mode)
        self.started = float(now)
        self.resent = False
        self.done = False
        self.on_done = on_done
        self.resend_s = float(resend_s)
        self.give_up_s = float(give_up_s)
        self._answered = False
        self._refused_at: float | None = None
        self._refused_result = 0

    def _finish(self, ok: bool, detail: str) -> None:
        if self.done:
            return
        self.done = True
        self.on_done(ok, detail)

    def heard_mode(self, custom_mode: int) -> None:
        """A heartbeat from the flight controller."""
        if self.custom_mode is not None and int(custom_mode) == self.custom_mode:
            self._finish(True, f"the drone is in {self.mode}")

    def heard_text(self, text: str) -> None:
        refusal = mode_refusal(text)
        if refusal is None:
            return
        mode, reason = refusal
        # Only a refusal of this mode. The text can come after the next
        # request has gone out: the simulator refused LOITER, VGCS asked for
        # LAND straight after, and the late LOITER text was blamed on LAND.
        if _mode_key(mode) != _mode_key(self.mode):
            return
        self._finish(False, f"the drone refused {self.mode}: {reason}")

    def heard_ack(self, result: int, now: float) -> None:
        """The drone's answer to DO_SET_MODE (pymavlink sends the mode change as that command).

        Accepted still waits for the heartbeat. Refused waits a moment for the
        STATUSTEXT that says why (simulator, 4.6.2: the ack came first and the
        reason was lost).
        """
        self._answered = True
        if int(result) in (0, 5):         # accepted, in progress
            return
        if self._refused_at is None:
            self._refused_at = float(now)
            self._refused_result = int(result)

    def tick(self, now: float) -> bool:
        """True when the request should be sent once more."""
        if self.done:
            return False
        if self._refused_at is not None and now - self._refused_at >= MODE_REASON_WAIT_S:
            self._finish(False, f"the drone refused {self.mode} (result {self._refused_result}, no reason given)")
            return False
        waited = now - self.started
        if waited >= self.give_up_s:
            self._finish(False, f"no heartbeat in {self.mode} within {self.give_up_s:.0f} s")
            return False
        if not self.resent and not self._answered and waited >= self.resend_s:
            self.resent = True
            return True
        return False
