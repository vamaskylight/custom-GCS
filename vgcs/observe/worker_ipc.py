"""Shared subprocess-IPC plumbing for GCS worker processes.

Extracted from ``visual_object_tracker.py`` (the M14 CSRT tracker's process
isolation, added after a field machine repeatedly crashed the whole GCS
inside native cv2 code — see [[m13-track-state]] memory / DOCS). Nothing here
is tracker-specific: it's the generic length-prefixed pickle framing plus the
reader/writer-thread pattern needed to talk to any ``subprocess.Popen`` child
over stdin/stdout without ever blocking the calling thread. The M14 object
detector (``object_detector.py``) reuses this instead of re-deriving it.
"""

from __future__ import annotations

import pickle
import queue
import threading

import numpy as np


def encode_frame(frame_bgr: np.ndarray) -> tuple:
    """(shape, dtype-name, raw-bytes) instead of the ndarray itself.

    Field-confirmed crash point: pickling a raw numpy.ndarray (inside
    numpy's own C-level __reduce__/pickling code) hard-crashed the PARENT
    process on one client machine — an access violation, not a catchable
    exception — even though the array itself was a perfectly ordinary,
    freshly-built uint8 frame. Pickling plain ``bytes`` avoids numpy's
    array-pickling code path entirely.
    """
    arr = np.ascontiguousarray(frame_bgr)
    return (arr.shape, str(arr.dtype), arr.tobytes())


def decode_frame(encoded: tuple) -> np.ndarray:
    shape, dtype_name, raw = encoded
    return np.frombuffer(raw, dtype=np.dtype(dtype_name)).reshape(shape)


def read_exactly(stream, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None  # EOF — other end closed/died
        buf.extend(chunk)
    return bytes(buf)


def send_framed(stream, obj) -> bool:
    """Length-prefixed pickle frame. Returns False (never raises) on failure —
    a broken pipe here just means the worker is gone, not a caller-facing error."""
    try:
        data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
        stream.write(len(data).to_bytes(4, "little"))
        stream.write(data)
        stream.flush()
        return True
    except Exception:
        return False


def recv_framed(stream):
    """Inverse of ``send_framed``. Returns None on EOF/any failure."""
    header = read_exactly(stream, 4)
    if header is None:
        return None
    n = int.from_bytes(header, "little")
    data = read_exactly(stream, n)
    if data is None:
        return None
    try:
        return pickle.loads(data)
    except Exception:
        return None


def start_reader(proc) -> "queue.Queue":
    """Background thread continuously reading framed responses off the
    worker's stdout into a queue — mirrors the existing `_drain_stderr`
    pattern already used for the FFmpeg subprocess in pipeline.py. A
    sentinel `None` is pushed when the stream ends (worker died/closed), so
    the caller's `queue.get()` never blocks forever on a dead worker beyond
    its explicit timeout.
    """
    q: "queue.Queue" = queue.Queue()

    def _reader() -> None:
        try:
            while True:
                resp = recv_framed(proc.stdout)
                if resp is None:
                    break
                q.put(resp)
        except Exception:
            pass
        finally:
            try:
                q.put(None)
            except Exception:
                pass

    threading.Thread(target=_reader, daemon=True).start()
    return q


def start_writer(proc) -> "queue.Queue":
    """Background thread that owns every write to the worker's stdin —
    decouples a slow/stuck pipe from the calling thread.

    Field-confirmed bug this fixes (see [[m13-track-state]]): writing
    directly on the calling thread with no timeout on the write itself could
    block indefinitely once the worker fell behind draining its stdin,
    silently wedging the caller forever. Routing writes through a queue here
    means a stuck pipe can only ever delay this thread, never block the
    caller past its own read-side timeout.
    """
    q: "queue.Queue" = queue.Queue()

    def _writer() -> None:
        try:
            while True:
                msg = q.get()
                if msg is None:
                    break
                if not send_framed(proc.stdin, msg):
                    break
        except Exception:
            pass

    threading.Thread(target=_writer, daemon=True).start()
    return q


def request(
    proc, resp_q: "queue.Queue | None", send_q: "queue.Queue | None", msg: tuple, timeout_s: float
) -> tuple | None:
    """Enqueue ``msg`` for the writer thread and wait up to ``timeout_s`` for
    a response. Returns None on ANY failure to get a well-formed reply — dead
    worker, broken pipe, or timeout are all treated identically by the caller.
    """
    if proc is None or resp_q is None or send_q is None:
        return None
    try:
        send_q.put(msg)
    except Exception:
        return None
    try:
        return resp_q.get(timeout=timeout_s)
    except queue.Empty:
        return None
