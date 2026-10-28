"""The flight controller's clock on this computer's clock.

The flight controller stamps ATTITUDE, GLOBAL_POSITION_INT and SYSTEM_TIME with its own time since
boot (time_boot_ms). Each one reaches the Pi a few milliseconds later. For one message,
arrival - time_boot_ms is the difference of the two clocks plus that delay. The fastest message of
each quarter second carries the least delay, so those (the last three seconds) mark the clocks'
difference, free of the delay's jitter. A straight line through them follows the two clocks as they
drift apart (a few ms a minute between a Cube and a Pi; the simulator's clock runs a few percent
slow, and unevenly). A picture's row is written a second after the picture (flight.py), so the line
runs through the picture's moment instead of ending before it. So a camera frame, stamped on the
Pi's clock, is placed on the flight controller's clock to within a few milliseconds, and its
position and angles are taken from the flight controller's own samples at that moment.
"""

from __future__ import annotations

from collections import deque

NS_PER_MS = 1_000_000


def _fit(points: list[tuple[int, int]]) -> tuple[float, float, float]:
    """A straight line through (time, offset) points: (mean time, mean offset, slope)."""
    n = len(points)
    mean_t = sum(t for t, _o in points) / n
    mean_o = sum(o for _t, o in points) / n
    var = sum((t - mean_t) ** 2 for t, _o in points)
    slope = sum((t - mean_t) * (o - mean_o) for t, o in points) / var if var > 0 else 0.0
    return mean_t, mean_o, slope


def _at(line: tuple[float, float, float], t: float) -> float:
    return line[1] + line[2] * (t - line[0])


class AutopilotClock:
    BUCKET_NS = 250_000_000         # the fastest message of each quarter second
    BUCKETS = 12                    # the last three seconds
    RESTART_MS = 2000               # time_boot_ms this much smaller than before: the controller restarted

    def __init__(self) -> None:
        # [bucket, arrival_ns, offset_ns]: the smallest offset heard in that quarter second, oldest first.
        self._buckets: deque[list[int]] = deque()
        self._last_boot_ms: int | None = None
        self.restarts = 0
        self.samples = 0

    def hear(self, boot_ms: int, arrival_ns: int) -> None:
        """One message with the controller's time_boot_ms, and when it arrived here (ns)."""
        boot_ms = int(boot_ms)
        arrival_ns = int(arrival_ns)
        if self._last_boot_ms is not None and boot_ms < self._last_boot_ms - self.RESTART_MS:
            self._buckets.clear()
            self.restarts += 1
        self._last_boot_ms = boot_ms
        offset = arrival_ns - boot_ms * NS_PER_MS
        second = arrival_ns // self.BUCKET_NS
        last = self._buckets[-1] if self._buckets else None
        if last is not None and second <= last[0]:
            if offset < last[2]:
                last[1], last[2] = arrival_ns, offset
        else:
            self._buckets.append([second, arrival_ns, offset])
            while len(self._buckets) > self.BUCKETS:
                self._buckets.popleft()
        self.samples += 1

    def offset_at(self, local_ns: int) -> int | None:
        """This computer's clock minus the controller's at ``local_ns``, or None before any message."""
        if not self._buckets:
            return None
        if len(self._buckets) < 4:
            return min(x[2] for x in self._buckets)
        # The quarters that are over: the newest one has had only part of its messages, and its fastest
        # one may still be a slow one.
        points = [(x[1], x[2]) for x in list(self._buckets)[:-1]]
        line = _fit(points)
        if len(points) >= 6:
            # A quarter in which no message came fast lies above the line. Keep the lower half, again.
            lower = sorted(points, key=lambda p: p[1] - _at(line, p[0]))[: len(points) // 2]
            line = _fit(lower)
        return int(round(_at(line, int(local_ns))))

    @property
    def offset_ns(self) -> int | None:
        """The offset now (at the newest message)."""
        return self.offset_at(self._buckets[-1][1]) if self._buckets else None

    def boot_ms(self, local_ns: int) -> float | None:
        """The controller's time_boot_ms at this computer's ``local_ns``."""
        offset = self.offset_at(local_ns)
        return None if offset is None else (int(local_ns) - offset) / NS_PER_MS

    def local_ns(self, boot_ms: float) -> int | None:
        """This computer's clock at the controller's ``boot_ms`` (near the newest messages)."""
        offset = self.offset_ns
        return None if offset is None else int(round(float(boot_ms) * NS_PER_MS)) + offset
