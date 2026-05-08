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
    for part in parts[2:]:
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        params[k.strip().lower()] = v.strip()
    return DecodedTopFrame(command=command, params=params, raw=(raw or b"").decode("ascii", errors="ignore"))

