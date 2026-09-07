"""Lat/lon → military grid reference (MGRS) for observation / DOOAF reports."""

from __future__ import annotations

import math
import re
from typing import Any

_MGRS: Any | None = None
_MGRS_IMPORT_ERROR: str | None = None

try:
    import mgrs as _mgrs_mod

    _MGRS = _mgrs_mod.MGRS()
except Exception as exc:  # pragma: no cover - optional at import on some builds
    _MGRS_IMPORT_ERROR = str(exc)


def _float_pair(lat: object, lon: object) -> tuple[float, float] | None:
    try:
        la = float(lat)
        lo = float(lon)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(la) and math.isfinite(lo)):
        return None
    if abs(la) < 1e-9 and abs(lo) < 1e-9:
        return None
    return la, lo


def latlon_to_mgrs(
    lat: float | None,
    lon: float | None,
    *,
    precision: int = 5,
) -> str | None:
    """
  Convert WGS84 decimal degrees to MGRS (NATO grid reference).

  ``precision`` 5 ≈ 1 m (five easting + five northing digits in the 100 km square).
    """
    pair = _float_pair(lat, lon)
    if pair is None:
        return None
    if _MGRS is None:
        return None
    la, lo = pair
    prec = max(0, min(5, int(precision)))
    try:
        raw = str(_MGRS.toMGRS(la, lo, MGRSPrecision=prec))
    except Exception:
        return None
    return raw.strip() or None


def format_mgrs_display(mgrs: str | None) -> str:
    """Human-readable GR, e.g. ``43Q BC 77080 62277``."""
    raw = str(mgrs or "").strip().replace(" ", "").upper()
    if not raw:
        return ""
    m = re.match(r"^(\d{1,2})([C-HJ-NP-X])([A-HJ-NP-Z]{2})(\d+)$", raw)
    if not m:
        return raw
    zone, band, square, digits = m.groups()
    half = len(digits) // 2
    if half <= 0:
        return f"{zone}{band} {square}"
    east = digits[:half]
    north = digits[half:]
    return f"{zone}{band} {square} {east} {north}"


def format_grid_reference(
    lat: float | None,
    lon: float | None,
    *,
    precision: int = 5,
) -> str:
    """Grid reference for CSV/HTML export; empty string when unavailable."""
    mgrs = latlon_to_mgrs(lat, lon, precision=precision)
    if mgrs is None:
        return ""
    return format_mgrs_display(mgrs)


def mgrs_to_latlon(grid: str | None) -> tuple[float, float] | None:
    """A grid reference back to WGS84 decimal degrees.

    The inverse of :func:`latlon_to_mgrs`, for typing in a GR that came over the
    radio and seeing where it is (requested 2026-09-08). Spaces are optional, so
    both ``43QBC7707662276`` and ``43Q BC 77076 62276`` work.
    """
    raw = str(grid or "").strip().replace(" ", "").upper()
    if not raw or _MGRS is None:
        return None
    # Reject anything that is not a grid reference before handing it over: the
    # library raises on some inputs and silently misreads others.
    #
    # Digits are required. "43QBC" is a legal MGRS *square*, but it names a
    # 100 km area, and the library resolves it to that square's corner - up to
    # 70 km from wherever the operator meant. A position needs at least one
    # easting and one northing digit.
    if not re.match(r"^\d{1,2}[C-HJ-NP-X][A-HJ-NP-Z]{2}\d{2,10}$", raw):
        return None
    digits = re.sub(r"^\d{1,2}[C-HJ-NP-X][A-HJ-NP-Z]{2}", "", raw)
    if len(digits) % 2 != 0:
        return None            # easting and northing must be the same length
    try:
        lat, lon = _MGRS.toLatLon(raw)
    except Exception:
        return None
    try:
        la, lo = float(lat), float(lon)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(la) and math.isfinite(lo)):
        return None
    if not (-90.0 <= la <= 90.0 and -180.0 <= lo <= 180.0):
        return None
    return la, lo


def grid_reference_available() -> bool:
    return _MGRS is not None
