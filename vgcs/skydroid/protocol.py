from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


def _xor_checksum(data: str) -> int:
    value = 0
    for ch in data.encode("ascii", errors="ignore"):
        value ^= int(ch)
    return value & 0xFF


def build_top_frame(command: str, params: Mapping[str, object] | None = None) -> bytes:
    """
    Build a lightweight TOP-style ASCII UDP frame with checksum.

    Format:
      $TOP,<COMMAND>[,<key>=<value>...]*<XOR2>\r\n
    """
    cmd = str(command or "").strip().upper()
    if not cmd:
        raise ValueError("command is required")
    parts = ["TOP", cmd]
    for key, value in (params or {}).items():
        k = str(key or "").strip().lower()
        if not k:
            continue
        parts.append(f"{k}={value}")
    body = ",".join(parts)
    checksum = _xor_checksum(body)
    return f"${body}*{checksum:02X}\r\n".encode("ascii", errors="ignore")


@dataclass(frozen=True)
class DecodedTopFrame:
    command: str
    params: dict[str, str]
    raw: str


def parse_top_frame(raw: bytes) -> DecodedTopFrame | None:
    text = (raw or b"").decode("ascii", errors="ignore").strip()
    if not text:
        return None
    if text.startswith("$"):
        text = text[1:]
    if "*" in text:
        text = text.split("*", 1)[0]
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) < 2:
        return None
    command = parts[1].upper()
    params: dict[str, str] = {}
    bare_nums: list[str] = []
    for part in parts[2:]:
        if "=" in part:
            k, v = part.split("=", 1)
            params[k.strip().lower()] = v.strip()
        else:
            bare_nums.append(part)
    if bare_nums:
        if "yaw" not in params and len(bare_nums) >= 1:
            params["yaw"] = bare_nums[0]
        if "pitch" not in params and len(bare_nums) >= 2:
            params["pitch"] = bare_nums[1]
        if "roll" not in params and len(bare_nums) >= 3:
            params["roll"] = bare_nums[2]
    return DecodedTopFrame(command=command, params=params, raw=(raw or b"").decode("ascii", errors="ignore"))


_ATTITUDE_KEYS: tuple[tuple[str, str], ...] = (
    ("yaw", "yaw"),
    ("yaw_deg", "yaw"),
    ("pan", "yaw"),
    ("y", "yaw"),
    ("pitch", "pitch"),
    ("pitch_deg", "pitch"),
    ("tilt", "pitch"),
    ("p", "pitch"),
)


def extract_attitude_deg(dec: DecodedTopFrame | None) -> tuple[float | None, float | None]:
    """Parse yaw/pitch from a TOP frame (GAA/GAC/GAY replies and async telemetry)."""
    if dec is None:
        return None, None
    yaw_v: float | None = None
    pitch_v: float | None = None
    for src, dst in _ATTITUDE_KEYS:
        if dst == "yaw" and yaw_v is not None:
            continue
        if dst == "pitch" and pitch_v is not None:
            continue
        raw = dec.params.get(src)
        if raw is None:
            continue
        val = _to_float(raw)
        if val is None:
            continue
        if dst == "yaw":
            yaw_v = val
        else:
            pitch_v = val
    return yaw_v, pitch_v


def _to_float(v: object) -> float | None:
    try:
        if v is None:
            return None
        s = str(v).strip().replace("°", "")
        if not s:
            return None
        return float(s)
    except Exception:
        return None

