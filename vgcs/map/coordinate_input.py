"""Read a position the way an operator would write one down.

Requested 2026-09-08: "add one feature that if I put the lat long then I can
see that particular location on the map". The inverse of the map-click
read-out, which turns a click into coordinates and a grid reference.

Whatever this crew is handed will arrive in one of three shapes, so all three
are accepted rather than making them convert first:

* decimal degrees, ``20.4101472, 72.8798915``
* degrees/minutes/seconds, ``20°24'36.5"N 72°52'47.6"E``
* a military grid reference, ``43QBC7707662276`` or ``43Q BC 77076 62276``

The one rule worth stating: **this never guesses.** A string it does not
recognise returns ``None`` and the caller says so. Silently flying to a
misparsed coordinate is worse than refusing to parse it.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

__all__ = ["ParsedLocation", "parse_location", "describe_formats"]

_DEG = "°"
_MIN = "′"
_SEC = "″"


@dataclass(frozen=True)
class ParsedLocation:
    lat: float
    lon: float
    kind: str          # "decimal" | "dms" | "grid"


def describe_formats() -> str:
    """One line for a dialog, in the formats they actually use."""
    return (
        "20.4101472, 72.8798915\n"
        "20°24'36.5\"N 72°52'47.6\"E\n"
        "43QBC7707662276"
    )


def parse_location(text: str | None) -> ParsedLocation | None:
    """Best-effort read of a typed position. ``None`` when unrecognised."""
    raw = str(text or "").strip()
    if not raw:
        return None
    for parse in (_parse_grid, _parse_dms, _parse_decimal):
        got = parse(raw)
        if got is not None:
            return got
    return None


# --------------------------------------------------------------------------- #


def _valid(lat: float, lon: float) -> bool:
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


def _parse_grid(raw: str) -> ParsedLocation | None:
    from vgcs.observe.grid_reference import mgrs_to_latlon

    got = mgrs_to_latlon(raw)
    if got is None:
        return None
    return ParsedLocation(got[0], got[1], "grid")


_DECIMAL = re.compile(
    r"""^\s*
    (?P<lat_sign>[NS])?\s*
    (?P<lat>[+-]?\d{1,3}(?:\.\d+)?)\s*(?:[%s])?\s*(?P<lat_hemi>[NS])?
    \s*[,;/\s]\s*
    (?P<lon_sign>[EW])?\s*
    (?P<lon>[+-]?\d{1,3}(?:\.\d+)?)\s*(?:[%s])?\s*(?P<lon_hemi>[EW])?
    \s*$""" % (_DEG, _DEG),
    re.VERBOSE | re.IGNORECASE,
)


def _apply_hemi(value: float, *prefixes: str | None) -> float | None:
    """Apply an N/S/E/W marker. Two markers on one number is a typo, not a
    position, so it is refused rather than resolved."""
    marks = [p.upper() for p in prefixes if p]
    if len(marks) > 1:
        return None
    if not marks:
        return value
    if value < 0:
        return None            # "S -20" says south twice, in two ways
    return -value if marks[0] in ("S", "W") else value


def _parse_decimal(raw: str) -> ParsedLocation | None:
    m = _DECIMAL.match(raw.replace("–", "-").replace("—", "-"))
    if not m:
        return None
    try:
        lat = float(m.group("lat"))
        lon = float(m.group("lon"))
    except (TypeError, ValueError):
        return None
    lat = _apply_hemi(lat, m.group("lat_sign"), m.group("lat_hemi"))
    lon = _apply_hemi(lon, m.group("lon_sign"), m.group("lon_hemi"))
    if lat is None or lon is None or not _valid(lat, lon):
        return None
    return ParsedLocation(lat, lon, "decimal")


_DMS_PART = (
    r"(?P<{p}_sign>[{ns}])?\s*"
    r"(?P<{p}_d>[+-]?\d{{1,3}})\s*(?:[{deg}d]|\s)\s*"
    r"(?:(?P<{p}_m>\d{{1,2}}(?:\.\d+)?)\s*(?:['{minute}m]|\s)\s*)?"
    r"(?:(?P<{p}_s>\d{{1,2}}(?:\.\d+)?)\s*(?:[\"{second}s]|'')?\s*)?"
    r"(?P<{p}_hemi>[{ns}])?"
)
_DMS = re.compile(
    r"^\s*"
    + _DMS_PART.format(p="lat", ns="NS", deg=_DEG, minute=_MIN, second=_SEC)
    + r"\s*(?P<sep>[,;/])?\s*"
    + _DMS_PART.format(p="lon", ns="EW", deg=_DEG, minute=_MIN, second=_SEC)
    + r"\s*$",
    re.IGNORECASE,
)


def _dms_value(d: str, m: str | None, s: str | None) -> float | None:
    try:
        deg = float(d)
        minutes = float(m) if m else 0.0
        seconds = float(s) if s else 0.0
    except (TypeError, ValueError):
        return None
    if not (0.0 <= minutes < 60.0 and 0.0 <= seconds < 60.0):
        return None
    magnitude = abs(deg) + minutes / 60.0 + seconds / 3600.0
    return -magnitude if deg < 0 else magnitude


def _parse_dms(raw: str) -> ParsedLocation | None:
    m = _DMS.match(raw)
    if not m:
        return None
    # Degrees alone is decimal notation, not DMS; let the decimal parser own it
    # so "20 72" is not read as two whole-degree positions by accident.
    if not any(m.group(g) for g in ("lat_m", "lat_s", "lon_m", "lon_s")):
        return None
    # Where latitude ends and longitude begins has to be unambiguous. With
    # neither a hemisphere letter nor punctuation between them, this pattern
    # will happily swallow the longitude's digits as the latitude's minutes and
    # seconds: "20° 72°" came out as 20.0019, 2.0 - a position 7000 km from the
    # one that was typed, reported as if it were read correctly. A real DMS
    # position always carries one or the other.
    if not (m.group("sep") or m.group("lat_hemi") or m.group("lat_sign")):
        return None
    lat = _dms_value(m.group("lat_d"), m.group("lat_m"), m.group("lat_s"))
    lon = _dms_value(m.group("lon_d"), m.group("lon_m"), m.group("lon_s"))
    if lat is None or lon is None:
        return None
    lat = _apply_hemi(lat, m.group("lat_sign"), m.group("lat_hemi"))
    lon = _apply_hemi(lon, m.group("lon_sign"), m.group("lon_hemi"))
    if lat is None or lon is None or not _valid(lat, lon):
        return None
    return ParsedLocation(lat, lon, "dms")
