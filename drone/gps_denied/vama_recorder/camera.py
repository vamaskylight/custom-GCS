"""The down camera: the Raspberry Pi's camera through picamera2, or a stand-in without one.

A shot carries the camera's own time of the frame on this computer's clock (time.monotonic_ns). On
the Pi that is libcamera's SensorTimestamp, which counts from boot like CLOCK_MONOTONIC (the two
only part when the computer sleeps, and the Pi on a drone does not), so the recorder can place each
picture on the flight controller's clock without the delay of taking and saving it.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from .settings import Settings


class CameraError(RuntimeError):
    """The camera cannot be used, in plain words."""


@dataclass
class Shot:
    sensor_ns: int                         # the frame's time, this computer's clock
    extension: str                         # "jpg", or "pgm" from the stand-in
    exposure_us: int | None = None
    gain: float | None = None
    lens_position: float | None = None
    _save: Callable[[Path], None] | None = field(default=None, repr=False)
    _release: Callable[[], None] | None = field(default=None, repr=False)
    _pixels: Callable[[], object] | None = field(default=None, repr=False)

    def pixels(self):
        """The picture as a numpy array (for the matching), or None. Before save or discard."""
        return self._pixels() if self._pixels is not None else None

    def save(self, path: Path) -> None:
        """Write the picture. The shot is finished after this, saved or not."""
        try:
            if self._save is not None:
                self._save(Path(path))
        finally:
            self.discard()

    def discard(self) -> None:
        release, self._release = self._release, None
        if release is not None:
            release()


class PiCamera:
    """The Raspberry Pi camera (Camera Module 3, HQ, Global Shutter, ...) through picamera2."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._cam = None
        self._size: tuple[int, int] | None = None
        self._controls: dict = {}

    def start(self) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError:
            raise CameraError("picamera2 is not installed (sudo apt install python3-picamera2)") from None
        try:
            cam = Picamera2(self.settings.camera_index)
        except (IndexError, RuntimeError) as exc:
            raise CameraError(f"no camera found on camera port {self.settings.camera_index} ({exc}). "
                              "Check the ribbon cable and: rpicam-hello --list-cameras") from None
        self._cam = cam
        self._size = tuple(self.settings.size or cam.sensor_resolution)
        config = cam.create_still_configuration(main={"size": self._size}, buffer_count=2)
        cam.configure(config)
        cam.options["quality"] = int(self.settings.jpeg_quality)
        controls: dict = {}
        if "AfMode" in cam.camera_controls and self.settings.focus != "none":
            from libcamera import controls as lc
            if self.settings.focus == "far":
                controls.update(AfMode=lc.AfModeEnum.Manual, LensPosition=0.0)   # 0 dioptres: infinity
            else:
                controls.update(AfMode=lc.AfModeEnum.Continuous)
        if self.settings.exposure_us > 0:
            controls.update(AeEnable=False, ExposureTime=int(self.settings.exposure_us),
                            AnalogueGain=float(self.settings.gain))
        if controls:
            cam.set_controls(controls)
        self._controls = controls
        cam.start()
        time.sleep(1.0)                    # the automatic exposure settles

    def describe(self) -> dict:
        cam = self._cam
        props = dict(getattr(cam, "camera_properties", {}) or {}) if cam is not None else {}
        return {"kind": "raspberry pi camera", "model": str(props.get("Model", "")),
                "sensor_size": list(getattr(cam, "sensor_resolution", ()) or ()),
                "picture_size": list(self._size or ()),
                "focus": self.settings.focus, "exposure_us": self.settings.exposure_us,
                "controls": {k: str(v) for k, v in self._controls.items()}}

    def capture(self) -> Shot:
        if self._cam is None:
            raise CameraError("the camera is not started")
        request = self._cam.capture_request()
        md = request.get_metadata() or {}
        sensor_ns = md.get("SensorTimestamp")
        return Shot(sensor_ns=int(sensor_ns) if sensor_ns else time.monotonic_ns(), extension="jpg",
                    exposure_us=md.get("ExposureTime"), gain=md.get("AnalogueGain"),
                    lens_position=md.get("LensPosition"),
                    _save=lambda path: request.save("main", str(path)), _release=request.release,
                    _pixels=lambda: request.make_array("main"))

    def stop(self) -> None:
        cam, self._cam = self._cam, None
        if cam is not None:
            try:
                cam.stop()
            finally:
                cam.close()


class FakeCamera:
    """No camera: a small grey picture (PGM) whose shade is the frame number. For tests, and for a
    bench run of the recorder before the camera is fitted (``--fake-camera``)."""

    WIDTH, HEIGHT = 64, 48

    def __init__(self, settings: Settings | None = None, clock: Callable[[], int] = time.monotonic_ns) -> None:
        self.settings = settings
        self._clock = clock
        self.frames = 0
        self.started = False

    def start(self) -> None:
        self.started = True

    def describe(self) -> dict:
        return {"kind": "stand-in", "model": "fake", "picture_size": [self.WIDTH, self.HEIGHT]}

    def capture(self) -> Shot:
        if not self.started:
            raise CameraError("the camera is not started")
        self.frames += 1
        shade = self.frames % 256
        data = b"P5 %d %d 255\n" % (self.WIDTH, self.HEIGHT) + bytes([shade]) * (self.WIDTH * self.HEIGHT)
        return Shot(sensor_ns=int(self._clock()), extension="pgm", exposure_us=1000, gain=1.0,
                    _save=lambda path: Path(path).write_bytes(data))

    def stop(self) -> None:
        self.started = False
