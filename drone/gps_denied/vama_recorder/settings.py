"""The recorder's settings: one INI file, every value with a default (see recorder.ini)."""

from __future__ import annotations

import configparser
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_FILE = Path("/etc/vama-recorder.ini")

BAUDS = (9600, 19200, 38400, 57600, 111100, 115200, 230400, 256000, 460800, 500000, 921600, 1500000)
FOCUS = ("far", "auto", "none")


class SettingsError(ValueError):
    """A value in the settings file that cannot be used, with what is wrong in plain words."""


@dataclass(frozen=True)
class Settings:
    # [link]
    device: str = "/dev/serial0"
    baud: int = 115200
    # The recorder's own MAVLink address: the drone's system and the onboard computer component.
    source_system: int = 1
    source_component: int = 191
    # [camera]
    camera_index: int = 0
    interval_s: float = 1.0
    size: tuple[int, int] | None = None          # None: the camera's full picture
    jpeg_quality: int = 92
    focus: str = "far"
    exposure_us: int = 0                          # 0: the camera chooses
    gain: float = 1.0                             # with a fixed exposure only
    mount_yaw_deg: float = 0.0
    # [recording]
    folder: str = "~/vama-flights"
    start_height_m: float = 30.0
    min_free_mb: int = 2000
    # Where the values came from, for the flight's own record.
    source: str = field(default="built-in defaults", compare=False)

    @property
    def flights_dir(self) -> Path:
        return Path(self.folder).expanduser()

    def as_dict(self) -> dict:
        out = {k: getattr(self, k) for k in self.__dataclass_fields__ if k != "source"}
        out["size"] = "full" if self.size is None else f"{self.size[0]}x{self.size[1]}"
        return out


def _number(raw: str, name: str, low: float, high: float, kind=float):
    try:
        value = kind(raw)
    except ValueError:
        raise SettingsError(f"{name} must be a number from {low:g} to {high:g} (it is '{raw}')") from None
    if not low <= value <= high:
        raise SettingsError(f"{name} must be from {low:g} to {high:g} (it is {value:g})")
    return value


def _size(raw: str) -> tuple[int, int] | None:
    text = raw.strip().lower().replace(" ", "")
    if text in ("", "full"):
        return None
    parts = text.split("x")
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        raise SettingsError(f"camera size must be full or width x height, for example 2304x1296 (it is '{raw}')")
    w, h = int(parts[0]), int(parts[1])
    if not (64 <= w <= 10000 and 64 <= h <= 10000):
        raise SettingsError(f"camera size {w}x{h} is not a picture size")
    return w, h


def load(path: Path | None = None) -> Settings:
    """The settings in ``path``, or the defaults where the file has no value. No file: the defaults."""
    if path is None:
        if not DEFAULT_FILE.is_file():
            return Settings()
        path = DEFAULT_FILE
    path = Path(path)
    if not path.is_file():
        raise SettingsError(f"no settings file at {path}")
    ini = configparser.ConfigParser(inline_comment_prefixes=("#", ";"))
    try:
        ini.read(path, encoding="utf-8")
    except configparser.Error as exc:
        raise SettingsError(f"{path} cannot be read: {exc}") from None
    known = {"link": {"device", "baud", "source_system", "source_component"},
             "camera": {"camera_index", "interval_s", "size", "jpeg_quality", "focus", "exposure_us", "gain",
                        "mount_yaw_deg"},
             "recording": {"folder", "start_height_m", "min_free_mb"}}
    for section in ini.sections():
        if section not in known:
            raise SettingsError(f"{path}: unknown section [{section}]")
        for key in ini[section]:
            if key not in known[section]:
                raise SettingsError(f"{path}: unknown setting '{key}' in [{section}]")

    def get(section: str, key: str) -> str | None:
        return ini.get(section, key, fallback=None)

    d = Settings()
    values: dict = {}
    if (v := get("link", "device")) is not None:
        if not v.strip():
            raise SettingsError("link device is empty")
        values["device"] = v.strip()
    if (v := get("link", "baud")) is not None:
        baud = _number(v, "link baud", 1200, 3000000, int)
        if baud not in BAUDS:
            raise SettingsError(f"link baud {baud} is not a speed the flight controller uses "
                                f"({', '.join(str(b) for b in BAUDS)})")
        values["baud"] = baud
    if (v := get("link", "source_system")) is not None:
        values["source_system"] = _number(v, "link source_system", 1, 255, int)
    if (v := get("link", "source_component")) is not None:
        values["source_component"] = _number(v, "link source_component", 1, 255, int)
    if (v := get("camera", "camera_index")) is not None:
        values["camera_index"] = _number(v, "camera camera_index", 0, 1, int)
    if (v := get("camera", "interval_s")) is not None:
        values["interval_s"] = _number(v, "camera interval_s", 0.2, 60)
    if (v := get("camera", "size")) is not None:
        values["size"] = _size(v)
    if (v := get("camera", "jpeg_quality")) is not None:
        values["jpeg_quality"] = _number(v, "camera jpeg_quality", 50, 100, int)
    if (v := get("camera", "focus")) is not None:
        focus = v.strip().lower()
        if focus not in FOCUS:
            raise SettingsError(f"camera focus must be far, auto or none (it is '{v}')")
        values["focus"] = focus
    if (v := get("camera", "exposure_us")) is not None:
        values["exposure_us"] = _number(v, "camera exposure_us", 0, 200000, int)
    if (v := get("camera", "gain")) is not None:
        values["gain"] = _number(v, "camera gain", 1.0, 16.0)
    if (v := get("camera", "mount_yaw_deg")) is not None:
        values["mount_yaw_deg"] = _number(v, "camera mount_yaw_deg", -360, 360)
    if (v := get("recording", "folder")) is not None:
        if not v.strip():
            raise SettingsError("recording folder is empty")
        values["folder"] = v.strip()
    if (v := get("recording", "start_height_m")) is not None:
        values["start_height_m"] = _number(v, "recording start_height_m", 0, 5000)
    if (v := get("recording", "min_free_mb")) is not None:
        values["min_free_mb"] = _number(v, "recording min_free_mb", 100, 1_000_000, int)
    fields = {k: getattr(d, k) for k in d.__dataclass_fields__}
    fields.update(values)
    fields["source"] = str(path)
    return Settings(**fields)
