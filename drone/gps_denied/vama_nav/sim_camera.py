"""A stand-in for the down camera, for the ArduCopter simulator (--sim-camera): what the camera would
see at the simulator's true position and angles (SIM_STATE), made from a map of the ground.

Give it a map of another year than the drone's own map, so the two differ as a map and the real
ground do. On top it blurs the picture a little, darkens its corners (as a lens does), changes its
brightness and adds the sensor's noise. Not simulated: the rolling shutter, motion blur, clouds and
shadows of the moment. The simulator's ground is flat; ``hills`` (a made-up ground, see Hills) lays the
photos over rolling hills for the camera alone.

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

from .geo import TILE, latlon, m_per_px, north_east, world_px
from .geometry import Lens, Pose, render
from .heights import ENCODING, terrarium_png
from .map_tiles import TileWriter

log = logging.getLogger("vama_nav.sim_camera")

TRUTH_MAX_AGE_NS = 500_000_000
CONTROL_EVERY_NS = 1_000_000_000
SAME_BURST_NS = 5_000_000           # an ATTITUDE this close before a SIM_STATE was sent with it
HEIGHTS_OFF_WAVE_M = 700.0          # Hills' ``off_m``: an elevation model's error has hills of its own


class Hills:
    """A made-up ground: rolling hills within plus or minus ``amp_m``, about ``wave_m`` from one top to the
    next, level with ``origin`` at that place. Asked as a heights.Heights is, with heights above the
    origin. ``off_m``: its heights are off by up to that much, smoothly, as an elevation model's are
    (SRTM's about 5 to 10 m): for what the drone knows of the ground, not for what the camera sees."""

    def __init__(self, amp_m: float, origin: tuple[float, float], wave_m: float = 3000.0, phase: float = 0.0,
                 off_m: float = 0.0) -> None:
        self.amp_m, self.origin, self.wave_m, self.phase, self.off_m = float(amp_m), origin, float(wave_m), float(phase), float(off_m)

    @staticmethod
    def _wavy(amp: float, wave: float, phase: float, north, east):
        k = 2.0 * np.pi / wave
        return amp * (0.6 * np.sin(k * north + 0.7 + phase) * np.cos(0.8 * k * east + 0.3 + phase)
                      + 0.4 * np.sin(1.7 * k * east + 1.1 + phase) * np.cos(1.3 * k * north + 2.1 + phase))

    def _z(self, north, east):
        """The height at metres north and east of the origin, level with the origin there."""
        z = self._wavy(self.amp_m, self.wave_m, self.phase, north, east) - self._wavy(self.amp_m, self.wave_m, self.phase, 0.0, 0.0)
        if self.off_m:
            z = z + self._wavy(self.off_m, HEIGHTS_OFF_WAVE_M, self.phase + 1.3, north, east)
        return z

    def at(self, lat: float, lon: float) -> float:
        return float(self._z(*north_east(self.origin[0], self.origin[1], lat, lon)))

    def around(self, lat: float, lon: float):
        n0, e0 = north_east(self.origin[0], self.origin[1], lat, lon)
        below = float(self._z(n0, e0))
        return lambda north, east: self._z(n0 + np.asarray(north), e0 + np.asarray(east)) - below

    def write_file(self, path: Path, ground_alt_m: float, south: float, west: float, north: float, east: float,
                   zoom: int = 13) -> None:
        """A heights file of the box (heights.Heights reads it): for a test, as an elevation model's would be."""
        writer = TileWriter(path, "made-up hills", "png", "made up for a test", f"plus or minus {self.amp_m:g} m")
        writer.meta["encoding"] = ENCODING
        x0, y0 = world_px(north, west, zoom)
        x1, y1 = world_px(south, east, zoom)
        s = m_per_px(0.5 * (south + north), zoom)
        for ty in range(int(y0 // TILE), int(y1 // TILE) + 1):
            for tx in range(int(x0 // TILE), int(x1 // TILE) + 1):
                top, left = latlon(tx * TILE + 0.5, ty * TILE + 0.5, zoom)
                n0, e0 = north_east(self.origin[0], self.origin[1], top, left)
                jj, ii = np.mgrid[0:TILE, 0:TILE]
                z = ground_alt_m + self._z(n0 - jj * s, e0 + ii * s)
                if z.min() < 0.0:
                    raise ValueError(f"the made-up valleys go {-z.min():.0f} m below sea level, where a heights file "
                                     "means the sea: put the ground higher")
                writer.put(zoom, tx, ty, terrarium_png(z))
        writer.close()


class SimCamera:
    # SIM_STATE, and ATTITUDE at the same rate (each true state right after the ATTITUDE of its moment):
    # asked for by the runner. 100 Hz, as match_source asks ATTITUDE for the gyros' turn between pictures.
    STREAMS_HZ = {108: 100.0, 30: 100.0}

    def __init__(self, world, lens: Lens, size: tuple[int, int] = (2028, 1520), ground_alt_m: float | None = None,
                 noise: float = 2.0, blur: float = 0.8, control: Path | None = None,
                 rng: np.random.Generator | None = None, clock=time.monotonic_ns, hills: Hills | None = None) -> None:
        """``world``: the ground as a TileMap. ``ground_alt_m``: the ground's height above sea level at
        take-off; None takes the simulator's first height (start the program before take-off). ``hills``:
        the ground's rolling hills above that, for the pictures only."""
        self.world = world
        self.hills = hills
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
        self._truth: deque = deque(maxlen=400)   # (moment_ns, lat, lon, alt, roll, pitch, yaw): 4 s at 100 Hz
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
        hill = self.hills.at(lat, lon) if self.hills is not None else 0.0
        picture = self.draw(Pose(roll, pitch, yaw, max(alt - ground - hill, 1.0)), lat, lon, blank=self.blank)
        self.taken += 1
        return Shot(sensor_ns=t, extension="jpg", exposure_us=1000, gain=1.0,
                    _save=lambda path: self._save(path, picture), _pixels=lambda: picture)

    def draw(self, pose: Pose, lat: float, lon: float, blank: bool = False) -> np.ndarray:
        """What the camera would see from ``pose`` above (lat, lon): the world drawn, a little blur,
        uneven light and the sensor's noise. Grey where the world has no picture (or ``blank``)."""
        width, height = self.size
        rise = self.hills.around(lat, lon) if self.hills is not None else None
        picture = None if blank else render(self.world, self.lens, pose, lat, lon, width, height, rise)
        if picture is None:
            picture = np.full((height, width), 128, np.uint8)
        return self._as_camera(picture)

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
