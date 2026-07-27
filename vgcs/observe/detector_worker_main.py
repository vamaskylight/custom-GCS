"""Entry point for the M14 detector's isolated worker process: `python -m
vgcs.observe.detector_worker_main`.

Same proven mechanism as ``tracker_worker_main.py`` (``subprocess.Popen``
with stdin/stdout pipes, deliberately NOT ``multiprocessing.Process`` — see
that module's docstring for why). Unlike the tracker worker, this one runs a
single request then exits — on-demand detection has no persistent per-tick
state to keep alive between calls.
"""

from __future__ import annotations

import sys


def _run() -> None:
    raw_out = sys.stdout.buffer
    raw_in = sys.stdin.buffer
    # Redirect print()'s default target to stderr so ordinary diagnostic
    # output can never interleave with and corrupt the binary framed
    # protocol on stdout — same convention as tracker_worker_main.py.
    sys.stdout = sys.stderr

    from vgcs.observe.object_detector import _InProcessDetector
    from vgcs.observe.worker_ipc import decode_frame, recv_framed, send_framed

    engine = _InProcessDetector()
    try:
        while True:
            msg = recv_framed(raw_in)
            if not msg:
                break
            cmd = msg[0]
            if cmd == "detect":
                _, encoded_frame = msg
                dets = engine.detect(decode_frame(encoded_frame))
                if not send_framed(raw_out, ("detected", dets)):
                    break
            elif cmd == "stop":
                break
    except Exception:
        pass


if __name__ == "__main__":
    _run()
