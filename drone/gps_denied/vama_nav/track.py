"""Which matched positions become fixes for the flight controller, over time.

One picture can pass every check of matcher.py and still be wrong (a field like another field). So
a position is sent only when it agrees with the track: the last sent position, moved on by the
flight controller's own movement since (over a few seconds its position drifts, its movement does
not). A position that disagrees is held. When two in a row disagree with the track but agree with
each other, they start a new track: the old one was wrong, or the drone was lost for a while. The
first position of a flight waits for a second one in the same way.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from .geo import north_east, offset

# moved(from_ns, to_ns): the drone's movement (north m, east m) between two moments, or None.
Moved = Callable[[int, int], tuple[float, float] | None]


@dataclass(frozen=True)
class Point:
    local_ns: int
    lat: float
    lon: float


class Track:
    def __init__(self, gate_m: float = 25.0, grow_mps: float = 3.0, keep_s: float = 10.0, pair_s: float = 3.0) -> None:
        """``gate_m``: how far a position may be from the track (plus ``grow_mps`` for each second
        since the track's last position). After ``keep_s`` without a sent position the track is
        too old to judge by. Two positions start a new track when at most ``pair_s`` apart."""
        self.gate_m = float(gate_m)
        self.grow_mps = float(grow_mps)
        self.keep_ns = int(keep_s * 1e9)
        self.pair_ns = int(pair_s * 1e9)
        self.last: Point | None = None
        self.candidate: Point | None = None
        self.new_tracks = 0

    def _distance(self, a: Point, b: Point, moved: Moved) -> float | None:
        """How far b is from where a would be at b's moment, or None when the movement is not known."""
        step = moved(a.local_ns, b.local_ns)
        if step is None:
            return None
        lat, lon = offset(a.lat, a.lon, *step)
        return math.hypot(*north_east(lat, lon, b.lat, b.lon))

    def judge(self, point: Point, moved: Moved) -> tuple[bool, str]:
        """(send it, why not or what happened)."""
        why = "no track yet"
        last = self.last
        if last is not None and point.local_ns - last.local_ns <= self.keep_ns:
            d = self._distance(last, point, moved)
            gate = self.gate_m + self.grow_mps * (point.local_ns - last.local_ns) / 1e9
            if d is not None and d <= gate:
                self.last, self.candidate = point, None
                return True, ""
            why = "the movement since the track's last position is not known" if d is None \
                else f"{d:.0f} m from the track (gate {gate:.0f} m)"
        elif last is not None:
            why = f"the track is {(point.local_ns - last.local_ns) / 1e9:.0f} s old"
        cand = self.candidate
        if cand is not None and 0 < point.local_ns - cand.local_ns <= self.pair_ns:
            d = self._distance(cand, point, moved)
            if d is not None and d <= self.gate_m:
                self.last, self.candidate = point, None
                self.new_tracks += 1
                return True, f"a new track: two positions in a row agree ({d:.0f} m apart), {why}"
        self.candidate = point
        return False, f"{why}; held until a second position agrees"

    def reset(self) -> None:
        self.last = self.candidate = None
