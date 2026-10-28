"""A stand-in for the down camera, for the ArduCopter simulator (--sim-camera): what the camera would
see at the simulator's true position and angles (SIM_STATE), made from a map of the ground.

Give it a map of another year than the drone's own map, so the two differ as a map and the real
ground do. On top it blurs the picture a little, darkens its corners (as a lens does), changes its
brightness and adds the sensor's noise. Not simulated: the rolling shutter, motion blur, clouds and
shadows of the moment, hills (the simulator's ground is flat).

Each picture's moment is the flight controller's own time of its truth, not the moment the truth
reached this program (that can be tens of milliseconds late, and at 300 m a drone that tips at 20
degrees a second moves its picture by metres in that time). The simulator sends SIM_STATE right
after the ATTITUDE of the same moment when both have the same rate: the ATTITUDE's time_boot_ms,
on this computer's clock by the recorder's clock fit, is the picture's moment. A SIM_STATE without
such an ATTITUDE just before it keeps its arrival.

A test can steer it with a small JSON file, read again when it changes:
    {"blank": false, "pause": false}
blank is ground that shows nothing (fog, water: a grey picture), pause is a camera that gives no
pictures.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from vama_recorder.camera import CameraError, Shot
from vama_recorder.clock import AutopilotClock

from .geometry import Lens, Pose, render

log = logging.getLogger("vama_nav.sim_camera")

TRUTH_MAX_AGE_NS = 500_000_000
CONTROL_EVERY_NS = 1_000_000_000
SAME_BURST_NS = 5_000_000           # an ATTITUDE this close before a SIM_STATE was sent with it


class SimCamera:
    STREAMS_HZ = {108: 20.0, 30: 20.0}  # SIM_STATE, and ATTITUDE at the same rate: asked for by the runner

    def __init__(self, world, lens: Lens, size: tuple[int, int] = (2028, 1520), ground_alt_m: float | None = None,
                 noise: float = 2.0, blur: float = 0.8, control: Path | None = None,
                 rng: np.random.Generator | None = None, clock=time.monotonic_ns) -> None:
        """``world``: the ground as a TileMap. ``ground_alt_m``: the ground's height above sea level;
        None takes the simulator's first height (start the program before take-off)."""
        self.world = world
        self.lens = lens
        self.size = (int(size[0]), int(size[1]))
        self.ground_alt_m = ground_alt_m
        self.noise = float(noise)
        self.blur = float(blur)
        self.control = control
        self.rng = rng or np.random.default_rng()
        self.clock = clock
        self.blank = self.paused = False
        self.taken = 0
        self._lock = threading.Lock()
        self._truth: deque = deque(maxlen=40)    # (moment_ns, lat, lon, alt, roll, pitch, yaw)
        self._clock = AutopilotClock()
        self._attitude: tuple[int, int] | None = None     # (time_boot_ms, arrival_ns) of the last ATTITUDE
        self.paired = self.unpaired = 0
        self._shade: np.ndarray | None = None
        self._control_read_ns: int | None = None
        self._control_stamp: float | None = None

    def hear(self, msg, arrival_ns: int) -> None:
        kind = msg.get_type()
        if kind == "ATTITUDE" and msg.get_srcComponent() == 1:
            with self._lock:
                self._clock.hear(int(msg.time_boot_ms), int(arrival_ns))
                self._attitude = (int(msg.time_boot_ms), int(arrival_ns))
            return
        if kind != "SIM_STATE":
            return
        lat_int, lon_int = getattr(msg, "lat_int", None), getattr(msg, "lon_int", None)
        if lat_int is not None and lon_int is not None and (lat_int or lon_int):
            lat, lon = lat_int / 1e7, lon_int / 1e7
        else:
            lat, lon = float(msg.lat), float(msg.lon)
        with self._lock:
            if self.ground_alt_m is None:
                self.ground_alt_m = float(msg.alt)
                log.info("the ground is at %.1f m above sea level (the simulator's first height)", self.ground_alt_m)
            moment = int(arrival_ns)
            att = self._attitude
            if att is not None and 0 <= arrival_ns - att[1] <= SAME_BURST_NS:
                local = self._clock.local_ns(att[0])
                if local is not None:
                    moment = min(local, int(arrival_ns))
                    self.paired += 1
            else:
                self.unpaired += 1
            self._truth.append((moment, lat, lon, float(msg.alt), float(msg.roll), float(msg.pitch),
                                float(msg.yaw)))

    def truth_at(self, local_ns: int) -> tuple[float, float] | None:
        """The simulator's true position at ``local_ns`` (for the results file), or None."""
        with self._lock:
            samples = list(self._truth)
        for a, b in zip(samples, samples[1:], strict=False):
            if a[0] <= local_ns <= b[0] and b[0] > a[0]:
                f = (local_ns - a[0]) / (b[0] - a[0])
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])
        return None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def describe(self) -> dict:
        return {"kind": "stand-in", "model": f"simulator camera on {self.world.describe()}",
                "picture_size": list(self.size), "hfov_deg": self.lens.hfov_deg}

    def capture(self) -> Shot:
        """A picture at the newest true state: its moment is when that state arrived."""
        self._read_control()
        if self.paused:
            raise CameraError("paused by the control file")
        with self._lock:
            truth = self._truth[-1] if self._truth else None
            ground = self.ground_alt_m
        if truth is None or self.clock() - truth[0] > TRUTH_MAX_AGE_NS:
            raise CameraError("no SIM_STATE from the simulator")
        t, lat, lon, alt, roll, pitch, yaw = truth
        width, height = self.size
        picture = None
        if not self.blank:
            picture = render(self.world, self.lens, Pose(roll, pitch, yaw, max(alt - ground, 1.0)), lat, lon, width, height)
        if picture is None:
            picture = np.full((height, width), 128, np.uint8)
        picture = self._as_camera(picture)
        self.taken += 1
        return Shot(sensor_ns=t, extension="jpg", exposure_us=1000, gain=1.0,
                    _save=lambda path: self._save(path, picture), _pixels=lambda: picture)

    def _as_camera(self, picture: np.ndarray) -> np.ndarray:
        h, w = picture.shape
        if self._shade is None or self._shade.shape != (h, w):
            y, x = np.mgrid[0:h, 0:w].astype(np.float32)
            r2 = ((x - w / 2) ** 2 + (y - h / 2) ** 2) / ((w / 2) ** 2 + (h / 2) ** 2)
            self._shade = 1.0 - 0.3 * r2
        out = picture.astype(np.float32)
        if self.blur > 0:
            out = cv2.GaussianBlur(out, (0, 0), self.blur)
        gain = 1.0 + 0.05 * float(self.rng.standard_normal())
        out = out * self._shade * gain + 4.0 * float(self.rng.standard_normal())
        if self.noise > 0:
            out += self.rng.standard_normal(out.shape, dtype=np.float32) * self.noise
        return np.clip(out, 0, 255).astype(np.uint8)

    @staticmethod
    def _save(path: Path, picture: np.ndarray) -> None:
        if not cv2.imwrite(str(path), picture, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise OSError(f"the picture could not be written to {path}")

    def _read_control(self) -> None:
        if self.control is None:
            return
        now = self.clock()
        if self._control_read_ns is not None and now - self._control_read_ns < CONTROL_EVERY_NS:
            return
        self._control_read_ns = now
        try:
            stamp = self.control.stat().st_mtime
            if stamp == self._control_stamp:
                return
            values = json.loads(self.control.read_text(encoding="utf-8"))
            blank, paused = bool(values.get("blank", False)), bool(values.get("pause", False))
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            log.warning("control file %s not read: %s", self.control, exc)
            return
        self._control_stamp = stamp
        if (blank, paused) != (self.blank, self.paused):
            log.info("control: %s", "paused" if paused else "blank ground" if blank else "the ground as it is")
        self.blank, self.paused = blank, paused
