"""Entry point for the M14 tracker's isolated worker process: `python -m
vgcs.observe.tracker_worker_main`.

Launched via ``subprocess.Popen`` (stdin/stdout pipes), NOT
``multiprocessing.Process``. This distinction matters: a field machine was
found where the parent process itself hard-crashed (Windows access
violation) inside multiprocessing's own spawn bootstrap — the internal pipe
handshake uses ``_winapi.DuplicateHandle``, which failed with
``PermissionError: [WinError 5] Access is denied`` on that machine (almost
certainly antivirus/EDR software blocking cross-process handle duplication,
a classic process-injection heuristic). That failure happened deep inside
CPython's own C implementation, not in cv2, so it wasn't a bug in our code —
but it meant multiprocessing.Process was simply not usable there.

subprocess.Popen with plain PIPE handles is a completely different, simpler
OS mechanism (anonymous pipes inherited at CreateProcess time, no
DuplicateHandle dance) — and it's already proven reliable on that exact
machine: this codebase has used it for the FFmpeg video decode subprocess
throughout this entire investigation without a single crash. Reusing that
same proven mechanism for the tracker worker, instead of inventing a new
one, is the point.
"""

from __future__ import annotations

import sys


def _run() -> None:
    raw_out = sys.stdout.buffer
    raw_in = sys.stdin.buffer
    # Redirect print()'s default target to stderr so ordinary diagnostic
    # output (e.g. _InProcessTracker's own prints on a caught cv2.error)
    # can never interleave with and corrupt the binary framed protocol on
    # stdout. stderr is inherited from the parent's console unmodified, so
    # those messages still show up exactly as before.
    sys.stdout = sys.stderr

    from vgcs.observe.visual_object_tracker import (
        _InProcessTracker,
        _recv_framed,
        _send_framed,
    )

    engine = _InProcessTracker()
    try:
        while True:
            msg = _recv_framed(raw_in)
            if not msg:
                break
            cmd = msg[0]
            if cmd == "start":
                _, frame_bgr, bbox = msg
                ok = engine.start(frame_bgr, bbox)
                if not _send_framed(raw_out, ("started", ok, engine.algo_used)):
                    break
            elif cmd == "update":
                _, frame_bgr = msg
                ok, box = engine.update(frame_bgr)
                if not _send_framed(raw_out, ("updated", ok, box, engine.lost_streak())):
                    break
            elif cmd == "stop":
                engine.stop()
                break
    except Exception:
        pass


if __name__ == "__main__":
    _run()
