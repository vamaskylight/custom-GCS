from __future__ import annotations

import re
import subprocess
import sys
from urllib.parse import urlparse


def _wifi_ipv4_gateway() -> str | None:
    """Best-effort default gateway for the active Wi-Fi adapter (RC hotspot path)."""
    if sys.platform != "win32":
        return None
    try:
        out = subprocess.check_output(
            ["ipconfig"],
            text=True,
            errors="ignore",
            timeout=4,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except Exception:
        return None
    blocks = re.split(r"\r?\n\r?\n", out)
    for block in blocks:
        low = block.lower()
        if "wireless lan adapter wi-fi" not in low and "wireless lan adapter wlan" not in low:
            if "wi-fi" not in low and "wlan" not in low:
                continue
        m = re.search(
            r"Default Gateway[^:\r\n]*:\s*(\d{1,3}(?:\.\d{1,3}){3})",
            block,
            re.IGNORECASE,
        )
        if not m:
            continue
        gw = m.group(1).strip()
        if gw and gw != "0.0.0.0":
            return gw
    return None


def resolve_skydroid_control_hosts(settings, *, default: str = "192.168.144.108") -> list[str]:
    """
    Hosts to try for Skydroid TOP UDP (attitude poll).

    Order: explicit setting, RTSP hostname, Wi-Fi gateway (RC forwards TOP when PC is on
    192.168.43.x hotspot), then C13 default IP.
    """
    out: list[str] = []

    def _add(h: str) -> None:
        h = str(h or "").strip()
        if h and h not in out:
            out.append(h)

    _add(str(settings.value("camera/skydroid_host", "") or "").strip())
    for key in ("video/rtsp_day", "video/rtsp_thermal"):
        url = str(settings.value(key, "") or "").strip()
        if url.lower().startswith("rtsp://"):
            parsed = urlparse(url)
            if parsed.hostname:
                _add(str(parsed.hostname))
    _add(_wifi_ipv4_gateway() or "")
    _add(str(default))
    return out
