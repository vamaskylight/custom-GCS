from __future__ import annotations

import re
import socket
import subprocess
import sys
from urllib.parse import urlparse


def local_ipv4_for_target(host: str) -> str | None:
    """Pick the local IPv4 the OS would use to reach ``host`` (multi-NIC laptops)."""
    h = str(host or "").strip()
    if not h:
        return None
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((h, 9))
        ip = str(probe.getsockname()[0] or "").strip()
        if ip and not ip.startswith("127."):
            return ip
    except Exception:
        return None
    finally:
        try:
            probe.close()
        except Exception:
            pass
    return None


def _ipv4_gateways_from_ipconfig() -> list[str]:
    if sys.platform != "win32":
        return []
    try:
        out = subprocess.check_output(
            ["ipconfig"],
            text=True,
            errors="ignore",
            timeout=4,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except Exception:
        return []
    found: list[str] = []
    for m in re.finditer(
        r"Default Gateway[^:\r\n]*:\s*(\d{1,3}(?:\.\d{1,3}){3})",
        out,
        re.IGNORECASE,
    ):
        gw = m.group(1).strip()
        if gw and gw != "0.0.0.0" and gw not in found:
            found.append(gw)
    return found


def _pick_rc_gateway(gateways: list[str]) -> str | None:
    """Prefer typical RC hotspot gateways (192.168.43.1, etc.)."""
    for gw in gateways:
        if gw.startswith("192.168.43."):
            return gw
    for gw in gateways:
        parts = gw.split(".")
        if len(parts) == 4 and gw.startswith("192.168.") and gw != "192.168.144.1":
            return gw
    return gateways[0] if gateways else None


def _wifi_ipv4_gateway() -> str | None:
    return _pick_rc_gateway(_ipv4_gateways_from_ipconfig())


def resolve_skydroid_control_hosts(settings, *, default: str = "192.168.144.108") -> list[str]:
    """
    Hosts to try for Skydroid TOP UDP (attitude poll).

    Order: explicit setting, RTSP hostname, default gateway (RC hotspot), then C13 IP.
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
    _add("192.168.144.12")  # legacy field note; PROTOCAL uses camera IP (.108) on UDP 5000
    return out
