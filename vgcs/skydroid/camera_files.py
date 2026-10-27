"""Cameras the admin adds, as files: one camera per file (task AA, client 2026-10-16).

The client asked for an admin panel to add a camera to the exe and the APK without new
code. The APK came first (apk/custom/src/Skydroid/CameraProfile.h/.cc). These are the same
files, read and written by the same rules: format "vama-camera-1", the same checks with the
same sentences, and the same file names. So a Skydroid camera made in either app works in
the other once its file is copied over (the APK keeps them in
Android/data/com.vama.gcs/files/Cameras, VGCS in its own folder, default_camera_folder()).

A file names its protocol: Skydroid's TOP, Viewpro or SIYI (the APK speaks all three since
task AK). VGCS uses the Skydroid cameras of these files. A Viewpro or SIYI camera file is for
the APK: VGCS reads it, says so in the notes, and leaves it out of its list (VGCS sets up those
cameras in Application Settings, Video).

The Skydroid cameras both apps were written for are built in, with the APK's numbers (V12,
V13, V14 Pro, V20). They cannot be changed: the admin copies one as the start of a new
camera. In VGCS the built-in cameras keep their own profiles (command_map.py); an added
camera becomes a profile through command_profile().
"""

from __future__ import annotations

import ipaddress
import json
import math
import os
import tempfile
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from vgcs.skydroid.command_map import (
    SKYDROID_PROFILES,
    ZOOM_MODEL_COMMANDED_OPTICAL,
    ZOOM_MODEL_DZM_STEP,
    ZOOM_MODEL_LENS_PULSE,
    SkydroidCommandProfile,
    SkydroidLens,
)

FORMAT = "vama-camera-1"
# The protocols the APK speaks. VGCS uses the Skydroid TOP cameras only.
PROTOCOL_TOP = "skydroid-top"
PROTOCOL_VIEWPRO = "viewpro"
PROTOCOL_SIYI = "siyi"
PROTOCOLS = (PROTOCOL_TOP, PROTOCOL_VIEWPRO, PROTOCOL_SIYI)
PROTOCOL = PROTOCOL_TOP  # the one VGCS uses, and the one of every file before task AK
MAX_FILE_BYTES = 64 * 1024
MAX_LENSES = 4
MAX_NAME_LENGTH = 40
# A counted zoom step, as both apps count it.
COUNTED_STEP_ZOOM = 0.1

ZOOM_NONE = "none"
ZOOM_COUNTED = "counted"
ZOOM_REPORTED = "reported"
ZOOM_LIVE = "live"  # the camera says its zoom and its view (Viewpro)
ZOOM_PULSE = "pulse"  # the camera zooms while it is told to, and does not say how far (SIYI, the C20's lens)
ZOOMS = (ZOOM_NONE, ZOOM_COUNTED, ZOOM_REPORTED, ZOOM_LIVE, ZOOM_PULSE)
PROTOCOL_ZOOMS = {
    PROTOCOL_TOP: (ZOOM_NONE, ZOOM_COUNTED, ZOOM_REPORTED, ZOOM_PULSE),
    PROTOCOL_VIEWPRO: (ZOOM_NONE, ZOOM_LIVE),
    PROTOCOL_SIYI: (ZOOM_NONE, ZOOM_PULSE),
}

# How TOP reaches the camera. A file from before the C20 has no "transport": UDP.
TRANSPORT_UDP = "udp"
TRANSPORT_TCP = "tcp"
TRANSPORTS = (TRANSPORT_UDP, TRANSPORT_TCP)
# Only a TOP camera has the choice: Viewpro takes its commands over TCP, SIYI over UDP.
PROTOCOL_TRANSPORTS = {PROTOCOL_VIEWPRO: TRANSPORT_TCP, PROTOCOL_SIYI: TRANSPORT_UDP}

# The addresses of Skydroid's cameras: day picture on RTSP port 554, thermal on 555.
SKYDROID_HOST = "192.168.144.108"
SKYDROID_PORT = 5000
SKYDROID_DAY_URL = "rtsp://192.168.144.108:554/stream=1"
SKYDROID_THERMAL_URL = "rtsp://192.168.144.108:555/stream=2"

# The addresses of the APK's Viewpro and SIYI cameras, as VGCS has them (vgcs/video/camera_control.py).
VIEWPRO_HOST = "192.168.2.119"  # the factory address printed on the unit
VIEWPRO_PORT = 2000
SIYI_HOST = "192.168.144.25"
SIYI_PORT = 37260

# The profile id an added camera has in VGCS's settings (camera/skydroid_profile).
FILE_PROFILE_PREFIX = "file:"


@dataclass(frozen=True)
class Lens:
    """One lens of a camera that reports its zoom step: its view at its first step, and its steps."""

    fov_h_deg: float
    fov_v_deg: float
    first_step: int
    last_step: int


@dataclass(frozen=True)
class Camera:
    """A camera. The values without a note are the C13's (as in the APK)."""

    id: str = ""  # built in: "C12", "C13", "C14 Pro", "C20"; added: its file name without ".json"
    name: str = ""  # on screen
    built_in: bool = False
    protocol: str = PROTOCOL_TOP  # VGCS uses the TOP cameras; Viewpro and SIYI ones are the APK's
    # Addresses. Choosing the camera sets them.
    host: str = ""
    port: int = SKYDROID_PORT
    # TOP over TCP, not UDP: the C20 takes it on TCP port 5000 only (TOP document V1.1.6,
    # section 2.1.1). Its file says "transport": "tcp" (else "udp").
    transport: str = TRANSPORT_UDP
    day_video_url: str = ""
    thermal_video_url: str = ""  # empty: no thermal picture (unless thermal_by_switch)
    # The thermal picture comes in the day video after a sensor switch (a Viewpro): no address of its own.
    thermal_by_switch: bool = False
    # The day picture
    fov_h_deg: float = 83.4  # at the widest zoom (none and counted)
    fov_v_deg: float = 46.9
    zoom: str = ZOOM_COUNTED  # none, counted (0.1x a step), reported (the camera's step picks lens and crop)
    max_zoom: float = 30.0  # counted: the highest zoom
    lenses: tuple[Lens, ...] = ()  # reported: in step order
    sensor_w: int = 3840  # reported: the picture is a crop of this many sensor points,
    sensor_h: int = 2160
    crop_step_w: int = 36  # and each step crops this many more (Skydroid, 2026-09-19)
    crop_step_h: int = 20
    # The thermal picture
    thermal_fov_h_deg: float = 48.7
    thermal_fov_v_deg: float = 38.6
    thermal_fov_sure: bool = False  # False: known from a data sheet only
    # How far the gimbal turns
    pitch_min_deg: float = -90.0
    pitch_max_deg: float = 10.0
    yaw_limit_deg: float = 90.0  # either way from the middle
    # The laser
    laser: bool = True
    laser_max_m: float = 1000.0  # longer readings are not taken as real
    # Other features
    photo: bool = True
    record: bool = True
    own_tracker: bool = True  # the camera's own tracker (else VGCS follows in software)
    # A detail of the protocol: every gimbal frame with the upper-case "#TP" header (C14 Pro).
    upper_gimbal_header: bool = False

    @property
    def has_thermal(self) -> bool:
        return bool(self.thermal_video_url) or self.thermal_by_switch

    @property
    def control_transport(self) -> str:
        """How the commands reach the camera: a TOP camera's own choice, else its protocol's."""
        own = PROTOCOL_TRANSPORTS.get(self.protocol)
        if own:
            return own
        return self.transport if self.transport in TRANSPORTS else TRANSPORT_UDP


def _skydroid(camera_id: str, name: str, **values: Any) -> Camera:
    return Camera(
        id=camera_id,
        name=name,
        built_in=True,
        host=SKYDROID_HOST,
        port=SKYDROID_PORT,
        day_video_url=SKYDROID_DAY_URL,
        thermal_video_url=SKYDROID_THERMAL_URL,
        **values,
    )


def built_in_cameras() -> list[Camera]:
    """The built-in cameras, in the order the list shows them: the APK's numbers.

    The C12 has the C13's calibrated view (VGCS aims the C12 with it too), 4x zoom and up to
    +30 degrees, its thermal lens from the sensor. The C13 is the default. The C14 Pro reports
    its zoom step: short lens steps 0 to 85, long lens 86 to 184, the maker's thermal lens,
    up to +60 degrees and 120 either way, a laser to 1500 m, "#TP" on every gimbal frame.
    """
    return [
        _skydroid("C12", "V12", max_zoom=4.0, thermal_fov_h_deg=36.4, thermal_fov_v_deg=27.7, pitch_max_deg=30.0),
        _skydroid("C13", "V13"),
        _skydroid(
            "C14 Pro",
            "V14 Pro",
            zoom=ZOOM_REPORTED,
            fov_h_deg=61.4,
            fov_v_deg=47.9,
            lenses=(Lens(61.4, 47.9, 0, 85), Lens(14.7, 11.1, 86, 184)),
            thermal_fov_h_deg=32.84,
            thermal_fov_v_deg=26.35,
            thermal_fov_sure=True,
            pitch_max_deg=60.0,
            yaw_limit_deg=120.0,
            laser_max_m=1500.0,
            upper_gimbal_header=True,
        ),
        # The C20: TOP over TCP port 5000 only, "#TP" on every gimbal frame, 22x optical zoom,
        # 58.3 x 34.9 at 1x (1080p), -90 to +45 degrees, 150 either way, no laser, no thermal
        # picture, its day video as the client's own camera file had it (2026-10-22). Its lens
        # zooms while told to and does not say how far (task AN, client 2026-10-27): "pulse".
        replace(
            _skydroid("C20", "V20"),
            transport=TRANSPORT_TCP,
            zoom=ZOOM_PULSE,
            day_video_url="rtsp://192.168.144.108:554/main",
            thermal_video_url="",
            fov_h_deg=58.3,
            fov_v_deg=34.9,
            max_zoom=22.0,
            pitch_max_deg=45.0,
            yaw_limit_deg=150.0,
            laser=False,
            own_tracker=False,
            upper_gimbal_header=True,
        ),
    ]


def apk_built_in_cameras() -> list[Camera]:
    """The APK's built-in Viewpro and SIYI cameras (task AK). VGCS does not use them: they are
    here for the cross-app files, so that both apps hold the same numbers.

    The A20KTR: Viewpro over TCP 2000, its main stream, the view its own screen showed at 1x
    (70.2 x 43.1, used until the camera says its own), its zoom said by the camera. Its tilt and
    turn are estimates, not from a data sheet. Its thermal picture comes in the same video after
    a sensor switch; its thermal lens is not known (so a point there is placed at the cross only).
    No laser in the APK yet, and no tracker of its own.

    The ZR10: SIYI over UDP 37260, 71.5 x 45.2 at 1x (the ZR10 manual), a zoom that does not say
    how far it went, -90 to +25 degrees and 45 either way (the range it can be told to go).
    """
    return [
        Camera(
            id="A20KTR",
            name="Viewpro A20KTR",
            built_in=True,
            protocol=PROTOCOL_VIEWPRO,
            host=VIEWPRO_HOST,
            port=VIEWPRO_PORT,
            transport=TRANSPORT_TCP,
            day_video_url="rtsp://192.168.2.119:554/stream0",
            thermal_by_switch=True,
            fov_h_deg=70.2,
            fov_v_deg=43.1,
            zoom=ZOOM_LIVE,
            pitch_max_deg=30.0,
            yaw_limit_deg=150.0,
            laser=False,
            own_tracker=False,
        ),
        Camera(
            id="ZR10",
            name="SIYI ZR10",
            built_in=True,
            protocol=PROTOCOL_SIYI,
            host=SIYI_HOST,
            port=SIYI_PORT,
            day_video_url="rtsp://192.168.144.25:8554/main.264",
            fov_h_deg=71.5,
            fov_v_deg=45.2,
            zoom=ZOOM_PULSE,
            pitch_max_deg=25.0,
            yaw_limit_deg=45.0,
            laser=False,
            own_tracker=False,
        ),
    ]


# VGCS's built-in profile of each built-in camera (for a copy made from what VGCS uses).
BUILT_IN_PROFILE_IDS = {
    "C12": "c12_default",
    "C13": "c13_default",
    "C14 Pro": "c14pro_default",
    "C20": "c20_default",
}


def _is_built_in_id(camera_id: str) -> bool:
    # The APK's own built-in ids too: a file with one of those names is refused there.
    return any(c.id.casefold() == str(camera_id).casefold() for c in built_in_cameras() + apk_built_in_cameras())


def _in_range(value: Any, low: float, high: float) -> bool:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(v) and low <= v <= high


def _is_rtsp(url: str) -> bool:
    return url[:7].lower() == "rtsp://" and len(url) > 7


def _is_ip(host: str) -> bool:
    try:
        ipaddress.ip_address(host)
    except ValueError:
        return False
    return True


def problem(cam: Camera) -> str:
    """What is wrong with a camera's values, as a sentence for the editor, or ""."""
    name = cam.name.strip()
    if not name:
        return "Give the camera a name."
    if len(name) > MAX_NAME_LENGTH:
        return f"The name is too long: {MAX_NAME_LENGTH} signs at the most."
    if not _is_ip(cam.host.strip()):
        return "The camera IP address is not an IP address (for example 192.168.144.108)."
    if not isinstance(cam.port, int) or not 1 <= cam.port <= 65535:
        return "The control port must be 1 to 65535."
    if not _is_rtsp(cam.day_video_url.strip()):
        return "The day video must be an RTSP address (rtsp://...)."
    if cam.thermal_video_url.strip() and not _is_rtsp(cam.thermal_video_url.strip()):
        return "The thermal video must be an RTSP address (rtsp://...), or empty for a camera without one."
    zooms = PROTOCOL_ZOOMS.get(cam.protocol)
    if zooms is None:
        return 'The protocol must be "skydroid-top", "viewpro" or "siyi".'
    if cam.zoom not in zooms:
        return {
            PROTOCOL_TOP: "A Skydroid TOP camera's zoom is none, counted, reported or pulse.",
            PROTOCOL_VIEWPRO: "A Viewpro camera's zoom is none or live.",
            PROTOCOL_SIYI: "A SIYI camera's zoom is none or pulse.",
        }[cam.protocol]
    if cam.protocol != PROTOCOL_TOP:
        if cam.laser:
            return "The app has no laser for a Viewpro or SIYI camera yet. Untick Laser."
        if cam.own_tracker:
            return 'The app uses no tracker of a Viewpro or SIYI camera. Untick "Its own object tracker".'
    if cam.thermal_by_switch:
        if cam.protocol != PROTOCOL_VIEWPRO:
            return "Only a Viewpro camera switches its day video to the thermal picture."
        if cam.thermal_video_url.strip():
            return "A camera that switches to its thermal picture has no thermal video of its own: leave it empty."
    if cam.zoom == ZOOM_REPORTED:
        if not cam.lenses or len(cam.lenses) > MAX_LENSES:
            return f"A camera that reports its zoom step needs 1 to {MAX_LENSES} lenses."
        if (
            not _in_range(cam.sensor_w, 16, 100000)
            or not _in_range(cam.sensor_h, 16, 100000)
            or cam.crop_step_w < 0
            or cam.crop_step_h < 0
        ):
            return "The sensor size must be 16 to 100000 points, and the crop of a step 0 or more."
        for i, lens in enumerate(cam.lenses):
            n = i + 1
            if not _in_range(lens.fov_h_deg, 0.5, 179.0) or not _in_range(lens.fov_v_deg, 0.5, 179.0):
                return f"Lens {n}: its view must be 0.5 to 179 degrees."
            if lens.first_step < 0 or lens.last_step < lens.first_step or lens.last_step > 9999:
                return f"Lens {n}: its steps must run from a first to a last step, 0 to 9999."
            if i > 0 and lens.first_step <= cam.lenses[i - 1].last_step:
                return f"Lens {n} must start after the last step of lens {n - 1}."
            m = lens.last_step - lens.first_step
            if cam.sensor_w - cam.crop_step_w * m < 1 or cam.sensor_h - cam.crop_step_h * m < 1:
                return f"Lens {n}: its last step crops more than the whole sensor."
    else:
        if not _in_range(cam.fov_h_deg, 1.0, 179.0) or not _in_range(cam.fov_v_deg, 1.0, 179.0):
            return "The day lens's view must be 1 to 179 degrees."
        if cam.zoom == ZOOM_COUNTED and not _in_range(cam.max_zoom, 1.1, 100.0):
            return "The highest zoom must be 1.1x to 100x."
    if (cam.thermal_video_url.strip() or cam.thermal_by_switch) and (
        not _in_range(cam.thermal_fov_h_deg, 1.0, 179.0) or not _in_range(cam.thermal_fov_v_deg, 1.0, 179.0)
    ):
        return "The thermal lens's view must be 1 to 179 degrees."
    if (
        not _in_range(cam.pitch_min_deg, -180.0, 0.0)
        or not _in_range(cam.pitch_max_deg, 0.0, 180.0)
        or not cam.pitch_max_deg > cam.pitch_min_deg
    ):
        return "The tilt must go from -180 to 0 degrees (down) up to 0 to 180 degrees."
    if not _in_range(cam.yaw_limit_deg, 1.0, 360.0):
        return "The turn either way must be 1 to 360 degrees."
    # The laser's reading has 4 hex digits of decimetres.
    if cam.laser and not _in_range(cam.laser_max_m, 5.0, 6553.0):
        return "The laser's longest reading must be 5 to 6553 m."
    return ""


def _finite(value: Any) -> Any:
    """A number for the file. Not a number, or endless: null, as Qt writes it (JSON has no NaN)."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def to_json(cam: Camera) -> dict[str, Any]:
    """The camera as its file holds it."""
    return {
        "format": FORMAT,
        "name": cam.name,
        "protocol": cam.protocol if cam.protocol in PROTOCOLS else PROTOCOL_TOP,
        "host": cam.host,
        "port": cam.port,
        "transport": cam.control_transport,
        "dayVideoUrl": cam.day_video_url,
        "thermalVideoUrl": cam.thermal_video_url,
        "thermalBySwitch": cam.thermal_by_switch,
        "zoom": cam.zoom if cam.zoom in ZOOMS else ZOOM_COUNTED,
        "dayFovH": _finite(cam.fov_h_deg),
        "dayFovV": _finite(cam.fov_v_deg),
        "maxZoom": _finite(cam.max_zoom),
        "lenses": [
            {
                "fovH": _finite(lens.fov_h_deg),
                "fovV": _finite(lens.fov_v_deg),
                "firstStep": lens.first_step,
                "lastStep": lens.last_step,
            }
            for lens in cam.lenses
        ],
        "sensorW": cam.sensor_w,
        "sensorH": cam.sensor_h,
        "cropStepW": cam.crop_step_w,
        "cropStepH": cam.crop_step_h,
        "thermalFovH": _finite(cam.thermal_fov_h_deg),
        "thermalFovV": _finite(cam.thermal_fov_v_deg),
        "thermalFovSure": cam.thermal_fov_sure,
        "pitchMin": _finite(cam.pitch_min_deg),
        "pitchMax": _finite(cam.pitch_max_deg),
        "yawLimit": _finite(cam.yaw_limit_deg),
        "laser": cam.laser,
        "laserMaxM": _finite(cam.laser_max_m),
        "photo": cam.photo,
        "record": cam.record,
        "ownTracker": cam.own_tracker,
        "upperGimbalHeader": cam.upper_gimbal_header,
    }


def _is_number(value: Any) -> bool:
    # A JSON true is a Python int too: not a number here, as for Qt.
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _whole(value: Any) -> int:
    """A whole number, or -1 (as the APK's reader takes them)."""
    if _is_number(value) and math.isfinite(float(value)) and float(value) == math.floor(float(value)) and abs(value) < 1e9:
        return int(value)
    return -1


def from_json(data: Any, camera_id: str) -> tuple[Camera | None, str]:
    """A camera from its file: (camera, "") or (None, why it is not one)."""
    if not isinstance(data, dict):
        return None, "it is not a camera file of this app."
    missing: list[str] = []

    def value(key: str) -> Any:
        if key not in data and not missing:
            missing.append(key)
        return data.get(key)

    def number(key: str) -> float:
        v = value(key)
        return float(v) if _is_number(v) else math.nan

    def text(key: str) -> str:
        v = value(key)
        return v.strip() if isinstance(v, str) else ""

    def flag(key: str, otherwise: bool) -> bool:
        v = data.get(key)
        return v if isinstance(v, bool) else otherwise

    if data.get("format") != FORMAT:
        return None, f'it is not a camera file of this app (no "format": "{FORMAT}").'
    protocol = data.get("protocol")
    if protocol not in PROTOCOLS:
        return None, 'its protocol is not "skydroid-top", "viewpro" or "siyi".'
    # Files from before the C20 have no transport: UDP, as every camera had then.
    transport = data.get("transport", TRANSPORT_UDP)
    if transport not in TRANSPORTS:
        return None, 'its transport is not "udp" or "tcp".'
    transport = PROTOCOL_TRANSPORTS.get(protocol, transport)  # Viewpro and SIYI have their own
    name = text("name")
    host = text("host")
    port = _whole(value("port"))
    day = text("dayVideoUrl")
    thermal = data.get("thermalVideoUrl")
    thermal = thermal.strip() if isinstance(thermal, str) else ""
    by_switch = flag("thermalBySwitch", False)  # files from before task AK have none: no switch
    zoom = value("zoom")
    if zoom not in ZOOMS:
        if not missing:
            return None, "its zoom is not one of none, counted, reported, live or pulse."
        zoom = ZOOM_COUNTED
    cam = Camera(
        id=camera_id,
        name=name,
        protocol=protocol,
        host=host,
        port=port,
        transport=transport,
        day_video_url=day,
        thermal_video_url=thermal,
        thermal_by_switch=by_switch,
        zoom=zoom,
    )
    if zoom == ZOOM_REPORTED:
        lenses = []
        raw = value("lenses")
        for item in raw if isinstance(raw, list) else []:
            item = item if isinstance(item, dict) else {}
            fov_h = item.get("fovH")
            fov_v = item.get("fovV")
            lenses.append(
                Lens(
                    float(fov_h) if _is_number(fov_h) else math.nan,
                    float(fov_v) if _is_number(fov_v) else math.nan,
                    _whole(item.get("firstStep")),
                    _whole(item.get("lastStep")),
                )
            )
        cam = replace(
            cam,
            lenses=tuple(lenses),
            sensor_w=_whole(value("sensorW")),
            sensor_h=_whole(value("sensorH")),
            crop_step_w=_whole(value("cropStepW")),
            crop_step_h=_whole(value("cropStepH")),
        )
        # The widest view, for the editor.
        if lenses:
            cam = replace(cam, fov_h_deg=lenses[0].fov_h_deg, fov_v_deg=lenses[0].fov_v_deg)
    else:
        cam = replace(cam, fov_h_deg=number("dayFovH"), fov_v_deg=number("dayFovV"))
        # A pulse zoom keeps its lens's range too, when its file has one (the V20's 22x), though
        # nothing counts it.
        if zoom == ZOOM_COUNTED or (zoom == ZOOM_PULSE and "maxZoom" in data):
            cam = replace(cam, max_zoom=number("maxZoom"))
    if thermal or by_switch:
        cam = replace(cam, thermal_fov_h_deg=number("thermalFovH"), thermal_fov_v_deg=number("thermalFovV"))
    laser = flag("laser", True)
    cam = replace(
        cam,
        thermal_fov_sure=flag("thermalFovSure", False),
        pitch_min_deg=number("pitchMin"),
        pitch_max_deg=number("pitchMax"),
        yaw_limit_deg=number("yawLimit"),
        laser=laser,
        laser_max_m=number("laserMaxM") if laser else 1000.0,
        photo=flag("photo", True),
        record=flag("record", True),
        own_tracker=flag("ownTracker", True),
        upper_gimbal_header=flag("upperGimbalHeader", False),
    )
    if missing:
        return None, f'it has no "{missing[0]}".'
    wrong = problem(cam)
    if wrong:
        return None, wrong
    return cam, ""


def id_for_name(name: str) -> str:
    """A file name (without ".json") for a camera's name: small letters, digits and "-"."""
    out = ""
    for c in str(name).lower():
        if c.isascii() and c.isalnum():
            out += c
        elif out and not out.endswith("-"):
            out += "-"
    out = out.rstrip("-")
    return out or "camera"


def default_camera_folder() -> Path:
    """VGCS's own folder of added cameras (VGCS_CAMERA_DIR, else .vgcs\\Cameras in the user's folder)."""
    override = str(os.environ.get("VGCS_CAMERA_DIR", "") or "").strip()
    if override:
        return Path(override)
    return Path.home() / ".vgcs" / "Cameras"


class CameraStore:
    """The cameras: the built-in ones, then the added ones (files in a folder), by name."""

    def __init__(self, folder: Path | str | None = None) -> None:
        self.folder = Path(folder) if folder is not None else default_camera_folder()
        self._cameras: list[Camera] = []
        self._app_only: list[Camera] = []
        self._notes: list[str] = []

    @property
    def cameras(self) -> list[Camera]:
        """The cameras VGCS can use: the built-in ones, then the added Skydroid ones."""
        return list(self._cameras)

    @property
    def app_only(self) -> list[Camera]:
        """The Viewpro and SIYI cameras of the folder: good files, for the APK only."""
        return list(self._app_only)

    @property
    def notes(self) -> list[str]:
        """The files that were not read, each with why."""
        return list(self._notes)

    def load(self) -> None:
        """Reads the folder again (and makes it, so that files can be copied into it)."""
        self._cameras = built_in_cameras()
        self._app_only = []
        self._notes = []
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            self._notes.append(f"The folder for cameras cannot be made: {self.folder}")
            return
        added: list[Camera] = []
        app_only: list[Camera] = []
        for path in sorted(self.folder.glob("*.json"), key=lambda p: p.name):
            if not path.is_file():
                continue
            camera_id = path.stem
            why = ""
            try:
                size = path.stat().st_size
            except OSError:
                continue  # removed meanwhile
            if _is_built_in_id(camera_id):
                why = "it has the file name of a built-in camera."
            elif size > MAX_FILE_BYTES:
                why = "it is too big for a camera file."
            else:
                try:
                    raw = path.read_bytes()
                except OSError:
                    why = "it cannot be read."
                else:
                    try:
                        data = json.loads(raw.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                        why = f"it is not a camera file ({exc})."
                    else:
                        cam, why = from_json(data, camera_id)
                        if cam is not None:
                            # Names are checked over every file, as the APK reads them all.
                            for other in self._cameras + added + app_only:
                                if other.name.casefold() == cam.name.casefold():
                                    why = f'a camera named "{other.name}" is there already.'
                                    break
                            if not why and cam.protocol != PROTOCOL_TOP:
                                app_only.append(cam)
                                kind = "Viewpro" if cam.protocol == PROTOCOL_VIEWPRO else "SIYI"
                                why = (
                                    f"a {kind} camera, for the APK only. VGCS sets up Viewpro and SIYI "
                                    "cameras in Application Settings, Video, Camera / gimbal control."
                                )
                            elif not why:
                                added.append(cam)
            if why:
                self._notes.append(f"{path.name}: {why}")
        added.sort(key=lambda c: c.name.casefold())
        app_only.sort(key=lambda c: c.name.casefold())
        self._cameras += added
        self._app_only = app_only

    def find(self, camera_id: str) -> Camera | None:
        for cam in self._cameras:
            if cam.id == camera_id:
                return cam
        return None

    def _name_problem(self, cam: Camera) -> str:
        for other in self._cameras + self._app_only:
            if other.id != cam.id and other.name.casefold() == cam.name.strip().casefold():
                return f'A camera named "{other.name}" is there already. Give this one another name.'
        return ""

    def save(self, cam: Camera) -> tuple[str, str]:
        """Saves an added camera, a new one when its id is empty or a built-in's: (problem, saved id)."""
        cam = replace(
            cam,
            built_in=False,
            name=cam.name.strip(),
            host=cam.host.strip(),
            day_video_url=cam.day_video_url.strip(),
            thermal_video_url=cam.thermal_video_url.strip(),
        )
        # An added camera keeps its file. Anything else (a copy of a built-in one, a new one) gets
        # a new file, named after the camera.
        known = self.find(cam.id)
        if known is None or known.built_in:
            base = id_for_name(cam.name)
            new_id = base
            n = 2
            while _is_built_in_id(new_id) or self.find(new_id) is not None or (self.folder / f"{new_id}.json").exists():
                new_id = f"{base}-{n}"
                n += 1
            cam = replace(cam, id=new_id)
        why = problem(cam) or self._name_problem(cam)
        if why:
            return why, ""
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            return f"The folder for cameras cannot be made: {self.folder}", ""
        path = self.folder / f"{cam.id}.json"
        try:
            # Written whole, then put in place: a camera file is never half written. Lines end
            # in "\n" alone, as the APK writes them, on every system.
            fd, tmp = tempfile.mkstemp(prefix=f".{cam.id}.", suffix=".tmp", dir=self.folder)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                json.dump(to_json(cam), f, indent=4, sort_keys=True)
                f.write("\n")
            os.replace(tmp, path)
        except OSError:
            return f"The camera's file cannot be written: {path}", ""
        self.load()
        return "", cam.id

    def remove(self, camera_id: str) -> str:
        """Removes an added camera's file: the problem, or ""."""
        cam = self.find(camera_id)
        if cam is None:
            return "There is no such camera."
        if cam.built_in:
            return "A built-in camera cannot be deleted."
        path = self.folder / f"{camera_id}.json"
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        except OSError:
            if path.exists():
                return f"The camera's file cannot be deleted: {path}"
        self.load()
        return ""


def profile_id_for(cam: Camera) -> str:
    """The id VGCS saves for a camera (camera/skydroid_profile)."""
    if cam.built_in:
        return BUILT_IN_PROFILE_IDS.get(cam.id, "c13_default")
    return FILE_PROFILE_PREFIX + cam.id


def command_profile(cam: Camera) -> SkydroidCommandProfile:
    """VGCS's profile of an added camera: the C13's commands, with this camera's lens, zoom and features.

    Its own lens is used for aiming (aim_fov_from_profile). No probe falls back to another
    camera's lens. Only the colour checks of the video gate run: its picture is not the
    C13's, which the full checks were tuned on.
    """
    base = SKYDROID_PROFILES["c13_default"]
    reported = cam.zoom == ZOOM_REPORTED and bool(cam.lenses)
    lenses = tuple(
        SkydroidLens(
            f"lens {i + 1}",
            float(lens.fov_h_deg),
            float(lens.fov_v_deg),
            int(lens.first_step),
            int(lens.last_step),
            sensor_w=int(cam.sensor_w),
            sensor_h=int(cam.sensor_h),
            crop_step_w=int(cam.crop_step_w),
            crop_step_h=int(cam.crop_step_h),
        )
        for i, lens in enumerate(cam.lenses)
    ) if reported else ()
    fov_h = float(lenses[0].fov_h_deg) if reported else float(cam.fov_h_deg)
    fov_v = float(lenses[0].fov_v_deg) if reported else float(cam.fov_v_deg)
    return replace(
        base,
        profile_id=profile_id_for(cam),
        display_name=cam.name,
        fov_h_deg=fov_h,
        fov_v_deg=fov_v,
        aim_fov_from_profile=True,
        zoom_model=(ZOOM_MODEL_DZM_STEP if reported
                    else ZOOM_MODEL_LENS_PULSE if cam.zoom == ZOOM_PULSE else ZOOM_MODEL_COMMANDED_OPTICAL),
        lenses=lenses,
        zoom_fov_confirmed=True,
        preview_carries_zoom=reported or cam.zoom == ZOOM_PULSE,  # a lens's zoom is in the picture
        thermal_fov_h_deg=float(cam.thermal_fov_h_deg) if cam.has_thermal else None,
        thermal_fov_v_deg=float(cam.thermal_fov_v_deg) if cam.has_thermal else None,
        thermal_fov_estimated=not cam.thermal_fov_sure,
        laser_max_m=float(cam.laser_max_m) if cam.laser else 1000.0,
        g_frames_upper_header=bool(cam.upper_gimbal_header),
        video_qc_mode="colour",
        probe_fallback_profile_id="",
        max_zoom=1.0 if cam.zoom == ZOOM_NONE else (float(cam.max_zoom) if cam.zoom == ZOOM_COUNTED else 30.0),
        photo=bool(cam.photo),
        record=bool(cam.record),
        transport=cam.transport if cam.transport in TRANSPORTS else TRANSPORT_UDP,
    )


def load_camera(camera_id: str, folder: Path | str | None = None) -> Camera | None:
    """One added camera, read from its file, or None (no such file, or not a camera)."""
    store = CameraStore(folder)
    store.load()
    cam = store.find(camera_id)
    return cam if cam is not None and not cam.built_in else None


__all__ = [
    "BUILT_IN_PROFILE_IDS",
    "COUNTED_STEP_ZOOM",
    "Camera",
    "CameraStore",
    "FILE_PROFILE_PREFIX",
    "FORMAT",
    "Lens",
    "MAX_FILE_BYTES",
    "MAX_LENSES",
    "MAX_NAME_LENGTH",
    "PROTOCOL",
    "PROTOCOLS",
    "PROTOCOL_SIYI",
    "PROTOCOL_TOP",
    "PROTOCOL_TRANSPORTS",
    "PROTOCOL_VIEWPRO",
    "PROTOCOL_ZOOMS",
    "TRANSPORTS",
    "TRANSPORT_TCP",
    "TRANSPORT_UDP",
    "ZOOMS",
    "ZOOM_COUNTED",
    "ZOOM_LIVE",
    "ZOOM_NONE",
    "ZOOM_PULSE",
    "ZOOM_REPORTED",
    "SIYI_HOST",
    "SIYI_PORT",
    "VIEWPRO_HOST",
    "VIEWPRO_PORT",
    "apk_built_in_cameras",
    "built_in_cameras",
    "command_profile",
    "default_camera_folder",
    "from_json",
    "id_for_name",
    "load_camera",
    "problem",
    "profile_id_for",
    "to_json",
]
