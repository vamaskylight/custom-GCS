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


def grid_reference_available() -> bool:
    return _MGRS is not None
