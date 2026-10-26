"""A fence on its way to the drone, and a fence read from it.

The drone keeps a fence as a list of items and hands it over with the mission
protocol, mission type "fence" (MAVLink 2 only: MAVLink 1 has no mission type).

    sending:  MISSION_COUNT ->        <- MISSION_REQUEST (one for each item)
              MISSION_ITEM_INT ->     <- MISSION_ACK (after the last item)
    reading:  MISSION_REQUEST_LIST -> <- MISSION_COUNT
              MISSION_REQUEST_INT ->  <- MISSION_ITEM_INT (one for each item)
              MISSION_ACK ->

Measured in the ArduCopter simulator, 4.6.2 and 4.7.0, on 2026-10-14:

- The drone asks for the items with MISSION_REQUEST, not MISSION_REQUEST_INT,
  and takes MISSION_ITEM_INT as the answer.
- A fence it does not take is answered with a MISSION_ACK that carries an
  error (1: not valid, 4: no room). Its reason follows in a STATUSTEXT a
  moment later ("Invalid vertex count (2)", "Only 70 items are supported"),
  so a refusal waits that moment for the words. The drone keeps the fence it
  had.
- A read right after an upload can still give the old count for a moment: the
  drone makes its list of the stored items again in its own time. So the read
  that checks an upload is tried again before it is called another fence.

Nothing here waits. The link thread feeds in what it hears and calls tick() on
every pass of its loop, so the position, the heartbeat and the drone's
messages keep flowing while a fence is sent.

Pure Python, no Qt and no pymavlink, so it is tested without a link.
"""

from __future__ import annotations

from typing import Callable

# MISSION_COUNT is sent again when the drone has not asked for an item yet.
UPLOAD_COUNT_WAIT_S = 1.5
UPLOAD_COUNT_TRIES = 3
# The drone asks again by itself for an item it did not get, and gives the
# whole upload up after 8 seconds. Past that nothing more can come.
UPLOAD_QUIET_S = 9.0
# The reason for a refusal comes in a STATUSTEXT after the MISSION_ACK.
REFUSAL_WORDS_WAIT_S = 0.7

# One request, sent again when no answer came.
READ_WAIT_S = 1.5
READ_TRIES = 3

# One message asked for with MAV_CMD_REQUEST_MESSAGE.
ASK_WAIT_S = 1.0
ASK_TRIES = 3

# The read that checks an upload: tried again this often, this far apart.
CHECK_TRIES = 5
CHECK_WAIT_S = 0.6

# MAV_MISSION_RESULT in the operator's words.
_REFUSALS = {
    1: "it found the fence not valid",
    2: "it does not take positions given this way",
    3: "this firmware has no fence of this kind",
    4: "it has no room for so many corners",
    5: "it found the fence not valid",
    13: "the corners arrived out of order",
    14: "it refused the fence",
    15: "the upload was broken off",
}

# Texts of the drone that explain why a fence was not taken.
_FENCE_WORDS = ("fence", "vertex", "lat or lon", "items are supported", "item count", "radius",
                "return point", "incorrect type")


def refusal_words(result: int) -> str:
    result = int(result)
    return f"{_REFUSALS.get(result, 'it answered with an error')} (result {result})"


def is_fence_text(text: str) -> bool:
    low = str(text or "").lower()
    return any(word in low for word in _FENCE_WORDS)


class FenceUpload:
    """Fence items being sent to the drone.

    on_done(ok, detail) runs exactly once. ok means the drone acknowledged
    every item. That is not yet proof of what it stored: read it back.
    """

    def __init__(self, items: list, now: float, on_done: Callable[[bool, str], None],
                 count_wait_s: float = UPLOAD_COUNT_WAIT_S, count_tries: int = UPLOAD_COUNT_TRIES,
                 quiet_s: float = UPLOAD_QUIET_S, words_wait_s: float = REFUSAL_WORDS_WAIT_S) -> None:
        self.items = list(items)
        self.on_done = on_done
        self.done = False
        self.count_wait_s = float(count_wait_s)
        self.count_tries = max(1, int(count_tries))
        self.quiet_s = float(quiet_s)
        self.words_wait_s = float(words_wait_s)
        self.texts: list[str] = []
        self._asked: set[int] = set()
        self._counts_sent = 1
        self._last_count = float(now)
        self._last_heard = float(now)
        self._refused: tuple[int, float] | None = None

    def _finish(self, ok: bool, detail: str) -> None:
        if self.done:
            return
        self.done = True
        self.on_done(ok, detail)

    def heard_request(self, seq: int, now: float):
        """The drone asks for an item. Returns the item to send, or None."""
        if self.done or self._refused is not None:
            return None
        try:
            number = int(seq)
        except (TypeError, ValueError):
            return None
        if not 0 <= number < len(self.items):
            return None
        self._asked.add(number)
        self._last_heard = float(now)
        return self.items[number]

    def heard_ack(self, result: int, now: float) -> None:
        """The drone's MISSION_ACK for the fence."""
        if self.done:
            return
        if int(result) == 0:
            # Accepted before it asked for every item is not the answer to
            # this upload (a late answer to something sent before).
            if len(self._asked) >= len(self.items):
                self._finish(True, "")
            return
        if self._refused is None:
            self._refused = (int(result), float(now))

    def heard_text(self, text: str) -> None:
        """A STATUSTEXT of the drone while the upload runs."""
        if not self.done and is_fence_text(text):
            line = str(text).strip()
            if line and line not in self.texts:
                self.texts.append(line)

    def tick(self, now: float) -> bool:
        """True when MISSION_COUNT should be sent once more."""
        if self.done:
            return False
        if self._refused is not None:
            result, when = self._refused
            if now - when >= self.words_wait_s:
                said = f' The drone says: "{"; ".join(self.texts)}".' if self.texts else ""
                self._finish(False, f"the drone did not take the fence: {refusal_words(result)}.{said}")
            return False
        if not self._asked:
            if now - self._last_count < self.count_wait_s:
                return False
            if self._counts_sent >= self.count_tries:
                self._finish(False, f"the drone did not ask for the fence ({self._counts_sent} tries). "
                                    "This firmware may have no fence, or the link is losing messages")
                return False
            self._counts_sent += 1
            self._last_count = float(now)
            return True
        if now - self._last_heard >= self.quiet_s:
            if len(self._asked) >= len(self.items):
                self._finish(False, "the drone did not confirm the fence after its last corner")
            else:
                self._finish(False, f"the drone stopped asking after {len(self._asked)} of "
                                    f"{len(self.items)} corners")
        return False

    def give_up(self, reason: str) -> None:
        """Nothing more can come (the link is gone)."""
        self._finish(False, reason)


class FenceRead:
    """The fence items the drone holds, read one by one.

    on_done(items, detail) runs exactly once: items is the list (may be
    empty), or None when the drone did not give it, and detail says why.
    """

    def __init__(self, now: float, on_done: Callable[[list | None, str], None],
                 wait_s: float = READ_WAIT_S, tries: int = READ_TRIES) -> None:
        self.on_done = on_done
        self.done = False
        self.count: int | None = None
        self.wait_s = float(wait_s)
        self.tries = max(1, int(tries))
        self._items: dict[int, object] = {}
        self._want = 0
        self._sent = 1
        self._last_send = float(now)

    def _finish(self, items: list | None, detail: str) -> None:
        if self.done:
            return
        self.done = True
        self.on_done(items, detail)

    def _ask_next(self, seq: int, now: float) -> int:
        self._want = int(seq)
        self._sent = 1
        self._last_send = float(now)
        return self._want

    def heard_count(self, count: int, now: float) -> int | None:
        """MISSION_COUNT of the fence. Returns the item to ask for, or None (an empty fence: done)."""
        if self.done or self.count is not None:
            return None
        try:
            self.count = max(0, int(count))
        except (TypeError, ValueError):
            self.count = None
            return None
        if self.count == 0:
            self._finish([], "")
            return None
        return self._ask_next(0, now)

    def heard_item(self, seq: int, item: object, now: float) -> int | None:
        """A fence item. Returns the next item to ask for, or None."""
        if self.done or self.count is None:
            return None
        try:
            number = int(seq)
        except (TypeError, ValueError):
            return None
        if number != self._want:
            return None            # a late answer to a request sent twice
        self._items[number] = item
        if number + 1 >= self.count:
            self._finish([self._items[i] for i in range(self.count)], "")
            return None
        return self._ask_next(number + 1, now)

    def heard_ack(self, result: int, now: float) -> None:
        """A MISSION_ACK instead of an answer: the drone does not give the fence."""
        if self.done or int(result) == 0:
            return
        self._finish(None, f"the drone answered a request for its fence with an error (result {int(result)})")

    def tick(self, now: float):
        """What to send again: "list", an item number, or None."""
        if self.done or now - self._last_send < self.wait_s:
            return None
        if self._sent >= self.tries:
            what = "its fence" if self.count is None else f"corner {self._want + 1} of its fence"
            self._finish(None, f"the drone did not give {what} ({self._sent} tries)")
            return None
        self._sent += 1
        self._last_send = float(now)
        return "list" if self.count is None else self._want

    def give_up(self, reason: str) -> None:
        self._finish(None, reason)


class MessageAsk:
    """One message asked for, and asked for again when it does not come.

    on_done(payload) runs exactly once: what came, or None when nothing did.
    """

    def __init__(self, now: float, on_done: Callable[[object], None],
                 wait_s: float = ASK_WAIT_S, tries: int = ASK_TRIES) -> None:
        self.on_done = on_done
        self.done = False
        self.wait_s = float(wait_s)
        self.tries = max(1, int(tries))
        self._sent = 1
        self._last_send = float(now)

    def _finish(self, payload: object) -> None:
        if self.done:
            return
        self.done = True
        self.on_done(payload)

    def heard(self, payload: object) -> None:
        self._finish(payload)

    def tick(self, now: float) -> bool:
        """True when the request should be sent once more."""
        if self.done or now - self._last_send < self.wait_s:
            return False
        if self._sent >= self.tries:
            self._finish(None)
            return False
        self._sent += 1
        self._last_send = float(now)
        return True

    def give_up(self) -> None:
        self._finish(None)


class FenceJob:
    """One fence request of the operator, from the first check to the drone's last answer.

    The link thread owns it and moves it on, one step at a time. Only one is
    open at a time: two exchanges of the mission protocol on one link would
    answer each other's questions.
    """

    def __init__(self, kind: str, cfg: dict, now: float) -> None:
        self.kind = str(kind)              # "polygon", "circle", "circle_items" (with areas to stay out of),
                                           # "nofly" (those areas alone), "read"
        self.cfg = dict(cfg)
        self.started = float(now)
        self.corners: list = []
        self.nofly: list = []              # the areas to stay out of (vgcs.mission.fence.NoFly)
        self.items: list = []
        self.radius_m = 0.0
        self.alt_max_m = 0.0
        self.action = 1.0
        self.notes: list[str] = []         # said with the result: what could not be checked
        self.checked_armed = False         # the checks for an armed drone were made (position, height, home)
        # The step that waits for the drone. At most one of the three is open.
        self.home: MessageAsk | None = None
        self.read: FenceRead | None = None
        self.upload: FenceUpload | None = None
        self.read_purpose = ""             # "before", "check", "show"
        self.check_tries = 0
        self.check_again_at: float | None = None
        self.send_at: float | None = None  # the items go out then, once FENCE_TOTAL was set to 0 (fence_link.py)
        self.on_drone = None               # what the read found (vgcs.mission.fence.FenceOnDrone)
        self.settings_open = False         # the fence settings are being written or read
