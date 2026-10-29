"""Where the drone's position comes from: GPS 1, or the down camera as GPS 2 (item A8, step 4).

The GPS-denied navigation (``drone/gps_denied/vama_nav``) gives ArduPilot the camera's
position as its second GPS: ``GPS2_TYPE`` 14 (a MAVLink GPS), ``GPS_AUTO_SWITCH`` 4, so
ArduPilot takes GPS 2 only while GPS 1 has no 3D fix. ArduPilot reports GPS 1 in
GPS_RAW_INT and GPS 2 in GPS2_RAW. So:

- GPS 1 with a 3D fix: the position is GPS 1's, whatever GPS 2 says.
- GPS 1 without one, GPS 2 with one: the camera's (or a second real GPS, when GPS2_TYPE
  is not 14: then it is called "GPS 2").
- Neither: no position source. The EKF holds on its own sensors for a few seconds, then
  its failsafe acts.

While the drone is disarmed vama_nav passes GPS 1's own fix on as GPS 2 (ArduPilot arms
only when every GPS has a 3D fix), so a GPS 2 fix then is not a camera position: the line
says so. A second real receiver counts on the ground as in the air.

A drone without GPS2_RAW shows nothing new. Every reading has a lifetime: a GPS that stopped
reporting is not shown as having a fix. A change of source is said once it has lasted
``EVENT_STABLE_S`` (a fix that flickers at the edge is not news every second).

Pure: no Qt and no link code.
"""

from __future__ import annotations

import math
import time

STALE_AFTER_S = 5.0
EVENT_STABLE_S = 2.0
MAVLINK_GPS_TYPE = 14

GPS1, CAMERA, SECOND_GPS, NONE, UNKNOWN = "gps1", "camera", "gps2", "none", "unknown"

LEVEL_NA, LEVEL_OK, LEVEL_WARN, LEVEL_BAD = "na", "ok", "warn", "bad"
NO_DATA = "N/A"


def _metres(value: object) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) and number > 0.0 else None


class PositionSource:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """A new link: nothing known."""
        self._gps1: tuple[int, float] | None = None              # (fix type, when)
        self._gps2: tuple[int, float | None, int, float] | None = None  # (fix, accuracy m, satellites, when)
        self._seen_gps2 = False
        self.armed = False
        self.gps2_type: int | None = None
        self.auto_switch: int | None = None
        self._said: str | None = None
        self._candidate: tuple[str, float] | None = None

    # --- what arrives -----------------------------------------------------------------------

    def update_gps1(self, fix_type: object, now: float | None = None) -> None:
        try:
            fix = int(fix_type)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return
        self._gps1 = (fix, time.monotonic() if now is None else float(now))

    def update_gps2(self, fix_type: object, h_acc_m: object = None, satellites: object = 0,
                    now: float | None = None) -> None:
        try:
            fix = int(fix_type)  # type: ignore[arg-type]
            sats = int(satellites or 0)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return
        self._gps2 = (fix, _metres(h_acc_m), sats, time.monotonic() if now is None else float(now))
        self._seen_gps2 = True

    def update_settings(self, params: dict) -> None:
        """GPS2_TYPE (GPS_TYPE2 before ArduPilot 4.6) and GPS_AUTO_SWITCH, as read from the drone."""
        for name in ("GPS2_TYPE", "GPS_TYPE2"):
            if name in params:
                try:
                    self.gps2_type = int(float(params[name]))
                except (TypeError, ValueError):
                    pass
        if "GPS_AUTO_SWITCH" in params:
            try:
                self.auto_switch = int(float(params["GPS_AUTO_SWITCH"]))
            except (TypeError, ValueError):
                pass

    # --- what it means ----------------------------------------------------------------------

    def has_gps2(self) -> bool:
        return self._seen_gps2

    def is_camera(self) -> bool:
        return self.gps2_type == MAVLINK_GPS_TYPE

    def _fresh(self, now: float) -> tuple[int | None, tuple | None]:
        g1 = self._gps1[0] if self._gps1 is not None and now - self._gps1[1] <= STALE_AFTER_S else None
        g2 = self._gps2 if self._gps2 is not None and now - self._gps2[3] <= STALE_AFTER_S else None
        return g1, g2

    def source(self, now: float | None = None) -> str:
        now = time.monotonic() if now is None else float(now)
        g1, g2 = self._fresh(now)
        if g1 is None and g2 is None:
            return UNKNOWN
        if g1 is not None and g1 >= 3:
            return GPS1
        if g2 is not None and g2[0] >= 3:
            return CAMERA if self.is_camera() else SECOND_GPS
        return NONE

    def _gps2_name(self) -> str:
        return "the camera (GPS 2)" if self.is_camera() else "GPS 2"

    def _gps2_words(self, g2: tuple | None) -> str:
        if g2 is None:
            return "no report"
        fix, acc, _sats, _t = g2
        if fix < 3:
            return "no fix"
        return f"3D, about {acc:.1f} m" if acc is not None else "3D"

    def field_text(self, now: float | None = None) -> str:
        now = time.monotonic() if now is None else float(now)
        if not self._seen_gps2:
            return NO_DATA
        g1, g2 = self._fresh(now)
        src = self.source(now)
        if src == UNKNOWN:
            return NO_DATA
        if src == GPS1:
            if self.is_camera() and not self.armed:
                return "GPS 1 (GPS 2 repeats GPS 1 until the drone is armed)"
            return f"GPS 1 ({self._gps2_name()}: {self._gps2_words(g2)})"
        if src in (CAMERA, SECOND_GPS):
            name = self._gps2_name()
            return f"{name[0].upper()}{name[1:]}, {self._gps2_words(g2)} (GPS 1 has no 3D fix)"
        if not self.armed and g2 is not None and g2[0] < 3:
            return "None on the ground (GPS 1 has no 3D fix)"
        return "None: GPS 1 and " + self._gps2_name() + " have no 3D fix"

    def level(self, now: float | None = None) -> str:
        src = self.source(now)
        if not self._seen_gps2 or src == UNKNOWN:
            return LEVEL_NA
        return {GPS1: LEVEL_OK, CAMERA: LEVEL_WARN, SECOND_GPS: LEVEL_WARN}.get(src, LEVEL_BAD)

    def tooltip(self, now: float | None = None) -> str:
        lines = ["Where the drone's position comes from: GPS 1 while it has a 3D fix, else GPS 2."]
        if self.is_camera():
            lines.append("GPS 2 is the down camera's position (GPS-denied navigation, vama_nav on the Pi).")
        elif self.gps2_type is not None:
            lines.append(f"GPS2_TYPE is {self.gps2_type}: GPS 2 is a receiver, not the camera (that is 14).")
        if self.auto_switch is not None and self.is_camera() and self.auto_switch != 4:
            lines.append(f"GPS_AUTO_SWITCH is {self.auto_switch}: vama_nav holds the camera's positions back "
                         "until it is 4.")
        return "\n".join(lines)

    def header_tag(self, now: float | None = None) -> str:
        """Beside the satellite count in the header: "CAM" while the camera gives the position."""
        src = self.source(now)
        return "CAM" if src == CAMERA else ("GPS2" if src == SECOND_GPS else "")

    def take_event(self, now: float | None = None) -> tuple[str, bool] | None:
        """(what to say, a warning) once a new source has lasted EVENT_STABLE_S, else None.
        Only in the air and only for a drone with GPS 2: GPS 1's own loss is said elsewhere."""
        now = time.monotonic() if now is None else float(now)
        if not self._seen_gps2 or not self.armed:
            self._candidate = None
            return None
        src = self.source(now)
        if src == UNKNOWN:
            return None
        if self._candidate is None or self._candidate[0] != src:
            self._candidate = (src, now)
        if src == self._said or now - self._candidate[1] < EVENT_STABLE_S:
            return None
        first, self._said = self._said is None, src
        if src == GPS1:
            return (None if first else ("The position comes from GPS 1 again", False))
        if src in (CAMERA, SECOND_GPS):
            return (f"GPS 1 has no 3D fix: the position comes from {self._gps2_name()}", True)
        return (f"No position: GPS 1 and {self._gps2_name()} have no 3D fix", True)
