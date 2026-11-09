"""The PC's clock at the start of every line VGCS prints (row B33, 2026-11-09).

The client's logs are VGCS's console text, and its lines had no time. In the V20 log of 2026-11-07
the two ends of the camera's stream could not be put beside the gimbal commands and the camera
port's questions: those are in `skydroid_top_udp.log` with times, the video lines had none. Every
line VGCS prints now starts with the time, "14:36:18.742 ", so the next log shows what came when.

Only lines VGCS writes through `sys.stdout` and `sys.stderr` get it. Qt's own warnings and the
crash trace (`faulthandler`) write to the process's handles directly, as before.

VGCS_LOG_CLOCK=0 leaves the lines as before.
"""

from __future__ import annotations

import datetime
import os
import sys
import threading
from typing import TextIO


def log_clock_wanted() -> bool:
    """False when VGCS_LOG_CLOCK=0 asks for lines without the time."""
    return str(os.environ.get("VGCS_LOG_CLOCK", "1") or "1").strip().lower() not in ("0", "false", "no", "off")


def _clock() -> str:
    # The PC's own clock, as in skydroid_top_udp.log.
    now = datetime.datetime.now(datetime.timezone.utc).astimezone()
    return now.strftime("%H:%M:%S.") + f"{now.microsecond // 1000:03d} "


class ClockLines:
    """A text stream that puts the time before each line it passes on."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream
        self._at_line_start = True
        self._lock = threading.Lock()

    def write(self, text: str) -> int:
        if not text:
            return 0
        with self._lock:
            out: list[str] = []
            pieces = text.split("\n")
            for i, piece in enumerate(pieces):
                if piece and self._at_line_start:
                    out.append(_clock())
                    self._at_line_start = False
                out.append(piece)
                if i < len(pieces) - 1:
                    out.append("\n")
                    self._at_line_start = True
            self._stream.write("".join(out))
        return len(text)

    def flush(self) -> None:
        self._stream.flush()

    def __getattr__(self, name: str):
        # encoding, errors, buffer, fileno, isatty, reconfigure... are the stream's own.
        return getattr(self._stream, name)


def install_log_clock() -> None:
    """Put the time before every line of `sys.stdout` and `sys.stderr`, once. No stream (a
    windowed start) is left alone."""
    if not log_clock_wanted():
        return
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name, None)
        if stream is None or isinstance(stream, ClockLines):
            continue
        setattr(sys, name, ClockLines(stream))
