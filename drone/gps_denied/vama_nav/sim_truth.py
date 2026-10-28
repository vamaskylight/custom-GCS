"""A stand-in for the picture matching: the ArduCopter simulator's true position, with noise
(--sim-truth). Only for the simulator: a real flight controller does not send SIM_STATE.

Like the camera will, it gives one position a second, only above ``match_above_m`` (a camera low
over the ground matches nothing with a satellite map), each with its own error, and hands it over
``delay_s`` after its moment (the matching takes time), so the sender carries it forward.

A test can steer it with a small JSON file, read again when it changes:
    {"offset_north_m": 150, "offset_east_m": 0, "pause": false}
an offset is a picture matched to the wrong place, a pause is ground that matches nothing.
"""

from __future__ import annotations

import json
import logging
import math
import random
from collections.abc import Callable
from pathlib import Path

from .gps_input import M_PER_DEG, Fix

log = logging.getLogger("vama_nav.sim_truth")

TRUTH_MAX_AGE_NS = 500_000_000      # a SIM_STATE older than this: the simulator is not talking, no fix
HEIGHT_MAX_AGE_NS = 2_000_000_000   # the same for the height above home
CONTROL_EVERY_NS = 1_000_000_000
# Message id: rate in Hz. SIM_STATE is the simulator's own true state, GLOBAL_POSITION_INT gives the
# height above home (as the camera would know it from the flight controller).
SIM_STREAMS_HZ = {108: 10.0, 33: 5.0}


class SimTruth:
    STREAMS_HZ = SIM_STREAMS_HZ         # asked for by the runner

    def __init__(self, offer: Callable[[Fix], object], noise_m: float = 2.0, speed_noise_mps: float = 0.2,
                 every_s: float = 1.0, delay_s: float = 0.4, match_above_m: float = 15.0,
                 control: Path | None = None, rng: random.Random | None = None) -> None:
        """``noise_m``: the error's standard deviation north, east and up; ``speed_noise_mps`` the
        same for the speed. The fixes report these as their accuracy."""
        self.offer = offer
        self.noise_m = float(noise_m)
        self.speed_noise_mps = float(speed_noise_mps)
        self.every_ns = int(every_s * 1e9)
        self.delay_ns = int(delay_s * 1e9)
        self.match_above_m = float(match_above_m)
        self.control = control
        self.rng = rng or random.Random()
        self.offset_north_m = self.offset_east_m = 0.0
        self.paused = False
        self.made = self.handed = 0
        self._truth: tuple | None = None        # (arrival_ns, lat, lon, alt_msl, vn, ve)
        self._height: tuple | None = None       # (arrival_ns, metres above home)
        self._pending: list[Fix] = []           # made, not handed over yet
        self._next_ns: int | None = None
        self._control_read_ns: int | None = None
        self._control_stamp: float | None = None

    def hear(self, msg, arrival_ns: int) -> None:
        kind = msg.get_type()
        if kind == "SIM_STATE":
            lat_int, lon_int = getattr(msg, "lat_int", None), getattr(msg, "lon_int", None)
            if lat_int is not None and lon_int is not None and (lat_int or lon_int):
                lat, lon = lat_int / 1e7, lon_int / 1e7
            else:                                # MAVLink 1: the float fields only
                lat, lon = float(msg.lat), float(msg.lon)
            self._truth = (int(arrival_ns), lat, lon, float(msg.alt), float(msg.vn), float(msg.ve))
        elif kind == "GLOBAL_POSITION_INT":
            self._height = (int(arrival_ns), msg.relative_alt / 1000.0)

    def tick(self, now_ns: int) -> None:
        """Hand over the fixes that are due, and make a new one each ``every_s``."""
        self._read_control(now_ns)
        while self._pending and self._pending[0].local_ns + self.delay_ns <= now_ns:
            self.offer(self._pending.pop(0))
            self.handed += 1
        if self._next_ns is None or now_ns >= self._next_ns:
            self._next_ns = max(now_ns, (self._next_ns or now_ns) + self.every_ns)
            fix = self._make(now_ns)
            if fix is not None:
                self._pending.append(fix)
                self.made += 1

    def why_not(self, now_ns: int) -> str | None:
        """Why no fix would be made now, or None."""
        if self.paused:
            return "paused by the control file"
        if self._truth is None or now_ns - self._truth[0] > TRUTH_MAX_AGE_NS:
            return "no SIM_STATE from the simulator"
        if self._height is None or now_ns - self._height[0] > HEIGHT_MAX_AGE_NS:
            return "height above home not known"
        if self._height[1] < self.match_above_m:
            return f"below {self.match_above_m:g} m"
        return None

    def _make(self, now_ns: int) -> Fix | None:
        if self.why_not(now_ns) is not None:
            return None
        t, lat, lon, alt, vn, ve = self._truth
        north = self.offset_north_m + self.rng.gauss(0.0, self.noise_m)
        east = self.offset_east_m + self.rng.gauss(0.0, self.noise_m)
        return Fix(local_ns=t, lat=lat + north / M_PER_DEG,
                   lon=lon + east / (M_PER_DEG * math.cos(math.radians(lat))),
                   alt_msl_m=alt + self.rng.gauss(0.0, self.noise_m),
                   vn=vn + self.rng.gauss(0.0, self.speed_noise_mps), ve=ve + self.rng.gauss(0.0, self.speed_noise_mps),
                   horiz_acc_m=max(self.noise_m * math.sqrt(2.0), 0.1), vert_acc_m=max(self.noise_m, 0.1),
                   speed_acc_mps=max(self.speed_noise_mps * math.sqrt(2.0), 0.05))

    def _read_control(self, now_ns: int) -> None:
        if self.control is None:
            return
        if self._control_read_ns is not None and now_ns - self._control_read_ns < CONTROL_EVERY_NS:
            return
        self._control_read_ns = now_ns
        try:
            stamp = self.control.stat().st_mtime
            if stamp == self._control_stamp:
                return
            values = json.loads(self.control.read_text(encoding="utf-8"))
            north = float(values.get("offset_north_m", 0.0))
            east = float(values.get("offset_east_m", 0.0))
            paused = bool(values.get("pause", False))
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            log.warning("control file %s not read: %s", self.control, exc)
            return
        self._control_stamp = stamp
        if (north, east, paused) != (self.offset_north_m, self.offset_east_m, self.paused):
            log.info("control: offset %.0f m north, %.0f m east, %s", north, east, "paused" if paused else "running")
        self.offset_north_m, self.offset_east_m, self.paused = north, east, paused
