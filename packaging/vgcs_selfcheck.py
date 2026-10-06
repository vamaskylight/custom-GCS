"""Self-check of a VGCS build: run "VGCS.exe --selfcheck".

It runs, inside the packaged app, the parts that break when packaging goes
wrong: bundled files, native libraries (Qt WebEngine, OpenCV, GDAL and PROJ,
MGRS), the MAVLink dialects, the tracker and detector worker processes, and
FFmpeg. It prints one line per check and returns the number of failed checks,
so 0 means the build is good. packaging/build_exe.py runs it after each build.

FAIL means the build itself is broken.
WARN means something is missing on this machine (for example FFmpeg), which
the build cannot fix.

From source, "py packaging/vgcs_selfcheck.py" runs the same checks. That helps
to tell a packaging problem from a code problem.
"""

from __future__ import annotations

import gc
import importlib
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
import traceback
from pathlib import Path
from typing import Callable

import numpy as np


class CheckWarning(Exception):
    """A problem with this machine, not with the build. Reported as WARN."""


def _frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def _is_bundled(path: Path) -> bool:
    bundle = getattr(sys, "_MEIPASS", None)
    return bundle is not None and Path(bundle).resolve() in path.resolve().parents


def _test_frame() -> np.ndarray:
    """A textured 640x480 BGR frame that a tracker can lock on to."""
    rng = np.random.default_rng(7)
    return rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8)


def _wait_for(app, done: Callable[[], bool], timeout_s: float) -> bool:
    """Run the Qt event loop until done() is true or the timeout passes."""
    from PySide6.QtCore import QEventLoop

    deadline = time.monotonic() + timeout_s
    while not done() and time.monotonic() < deadline:
        app.processEvents(QEventLoop.ProcessEventsFlag.AllEvents, 50)
        time.sleep(0.01)
    return done()


def check_data_dir(data_dir: Path) -> str:
    probe = data_dir / ".vgcs-selfcheck"
    probe.write_text("ok", encoding="utf-8")
    probe.unlink()
    return f"{data_dir} is writable"


def check_bundled_files() -> str:
    from vgcs.observe.object_detector import _LPD_MODEL_PATH, _YOLOX_MODEL_PATH
    from vgcs.paths import vgcs_assets_dir

    root = vgcs_assets_dir()
    required = [
        root / "Vama Logo.png",
        root / "header_icons" / "gps.svg",
        root / "menu_icons" / "plan_flight.svg",
        root / "vendor" / "leaflet" / "leaflet.js",
        # The 3D view: the library, one of its workers, its styles, and the
        # small world picture it draws with no internet.
        root / "vendor" / "cesium" / "Cesium.js",
        root / "vendor" / "cesium" / "Workers" / "createVerticesFromHeightmap.js",
        root / "vendor" / "cesium" / "Widgets" / "widgets.css",
        root / "vendor" / "cesium" / "Assets" / "Textures" / "NaturalEarthII" / "tilemapresource.xml",
        _YOLOX_MODEL_PATH,
        _LPD_MODEL_PATH,
    ]
    missing = [str(p) for p in required if not p.is_file()]
    if missing:
        raise RuntimeError("missing: " + ", ".join(missing))
    seed_tiles = sum(1 for _ in (root / "companion_tile_seed").rglob("*.png"))
    if seed_tiles == 0:
        raise RuntimeError("the offline map tile seed is empty")
    return f"{root} ({seed_tiles} offline seed tiles)"


def check_license() -> str:
    """The license window is bundled, this computer's machine code can be read,
    and whether a valid key is saved (a warning, not a failure, when none is)."""
    from vgcs.app import license_dialog  # noqa: F401  (bundled with its window)
    from vgcs.app import license_key

    fingerprint = license_key.machine_fingerprint()
    if fingerprint is None:
        raise RuntimeError("this computer's ID cannot be read, so no license key can be made for it")
    code = license_key.machine_code(fingerprint)
    result = license_key.check_stored_license(fingerprint=fingerprint)
    if result.ok:
        return f"machine code {code}, key OK ({license_key.describe(result.info)})"
    raise CheckWarning(f"machine code {code}, no valid key saved ({result.status.value})")


def check_mavlink() -> str:
    import pymavlink
    from pymavlink import mavutil  # noqa: F401  (loads the default dialect, like the app does)

    for wire in ("v10", "v20"):
        dialect = importlib.import_module(f"pymavlink.dialects.{wire}.ardupilotmega")
        sender = dialect.MAVLink(None, srcSystem=255, srcComponent=190)
        packet = sender.heartbeat_encode(6, 8, 0, 0, 4).pack(sender)
        received = dialect.MAVLink(None).parse_buffer(packet)
        if not received or received[0].get_type() != "HEARTBEAT":
            raise RuntimeError(f"a {wire} HEARTBEAT did not survive pack and parse")
    return f"pymavlink {pymavlink.__version__}, ardupilotmega MAVLink 1 and 2 pack and parse"


def check_serial_ports() -> str:
    from serial.tools import list_ports

    ports = [p.device for p in list_ports.comports()]
    return f"{len(ports)} port(s)" + (f": {', '.join(ports)}" if ports else "")


def check_grid_reference() -> str:
    from vgcs.observe.grid_reference import grid_reference_available, latlon_to_mgrs, mgrs_to_latlon

    if not grid_reference_available():
        raise RuntimeError("the MGRS native library did not load")
    grid = latlon_to_mgrs(20.445, 72.864, precision=5)
    back = mgrs_to_latlon(grid)
    if not grid or back is None or abs(back[0] - 20.445) > 1e-4 or abs(back[1] - 72.864) > 1e-4:
        raise RuntimeError(f"round trip failed: {grid!r} -> {back!r}")
    return f"20.445 N, 72.864 E -> {grid} -> back within 1e-4 deg"


def check_geotiff_dem() -> str:
    import rasterio
    from rasterio.transform import from_origin

    from vgcs.observe.dem import load_dem_model

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        path = Path(tmp) / "selfcheck_dem.tif"
        heights = np.full((4, 4), 123.0, dtype="float32")
        # Writing with an EPSG code needs the PROJ database (proj.db).
        with rasterio.open(
            path, "w", driver="GTiff", width=4, height=4, count=1, dtype="float32",
            crs="EPSG:4326", transform=from_origin(72.0, 21.0, 0.25, 0.25),
        ) as ds:
            ds.write(heights, 1)
        with rasterio.open(path) as ds:
            epsg = ds.crs.to_epsg() if ds.crs else None
        if epsg != 4326:
            raise RuntimeError(f"the CRS read back as {epsg!r}, not 4326 (PROJ data missing?)")
        model = load_dem_model(path)
        elevation = model.elevation_m(20.5, 72.5) if model is not None else None
        del model  # the DEM keeps the file open; release it before the folder is removed
        gc.collect()
    if elevation != 123.0:
        raise RuntimeError(f"the VGCS DEM loader read {elevation!r}, expected 123.0")
    return f"rasterio {rasterio.__version__}, GDAL {rasterio.__gdal_version__}: GeoTIFF, CRS and DEM sample"


def check_opencv() -> str:
    import cv2

    from vgcs.observe.object_detector import _LPD_MODEL_PATH, _YOLOX_MODEL_PATH

    if not hasattr(cv2, "TrackerCSRT_create"):
        raise RuntimeError("no CSRT tracker: this is not the opencv-contrib build")
    for model in (_YOLOX_MODEL_PATH, _LPD_MODEL_PATH):
        cv2.dnn.readNet(str(model))
    return f"OpenCV {cv2.__version__} (contrib), CSRT present, both detector models load"


def check_tracker_worker() -> str:
    from vgcs.observe.visual_object_tracker import TrackBox, VisualObjectTracker

    frame = _test_frame()
    tracker = VisualObjectTracker()
    try:
        if not tracker.start(frame, TrackBox(280.0, 200.0, 80.0, 80.0)):
            raise RuntimeError("the worker did not start, or the tracker did not initialise (see above)")
        ok, _box = tracker.update(frame)
        if not ok:
            raise RuntimeError("the worker started, but the first update failed")
        return f"{tracker.algo_used} tracking in its own process"
    finally:
        tracker.stop()


def check_detector_worker() -> str:
    from vgcs.observe import worker_ipc

    # Same launch as VisualObjectDetector.detect(). detect() itself returns []
    # both on failure and when nothing is found, so talk to the worker directly.
    proc = subprocess.Popen(
        [sys.executable, "-m", "vgcs.observe.detector_worker_main"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )
    send_q = None
    try:
        resp_q = worker_ipc.start_reader(proc)
        send_q = worker_ipc.start_writer(proc)
        request = ("detect", worker_ipc.encode_frame(_test_frame()))
        reply = worker_ipc.request(proc, resp_q, send_q, request, 30.0)
        if reply is None:
            code = proc.poll()
            raise RuntimeError("no reply in 30 s" + (f", worker exit code {code}" if code is not None else ""))
        return f"worker replied ({len(reply[1] or [])} detection(s) on a noise frame)"
    finally:
        if send_q is not None:
            send_q.put(("stop",))
            send_q.put(None)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def _run_tool(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    result = subprocess.run(cmd, capture_output=True, timeout=60, **kwargs)
    if result.returncode != 0:
        err = result.stderr if isinstance(result.stderr, str) else result.stderr.decode("utf-8", "replace")
        raise RuntimeError(f"{Path(cmd[0]).name} failed: {err.strip()[-300:] or result.returncode}")
    return result


def _ffmpeg_round_trip(ffmpeg: str, ffprobe: str) -> None:
    """Record, probe and decode a short clip the way VGCS does.

    Record: raw RGB frames on stdin to a libx264 MP4 (the arguments of
    pipeline.py's recording). Probe: ffprobe reads the size back (as pipeline.py
    sizes UDP video). Decode: MP4 to raw RGB with VGCS's own scale and pad filter
    (as live video does).
    """
    from vgcs.video.pipeline import _ffmpeg_vf_rgb_fixed_size

    w, h, frames = 320, 240, 10
    with tempfile.TemporaryDirectory() as tmp:
        clip = str(Path(tmp) / "clip.mp4")
        raw = np.ascontiguousarray(_test_frame()[:h, :w]).tobytes() * frames
        _run_tool([
            ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-video_size", f"{w}x{h}", "-framerate", "30",
            "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", clip,
        ], input=raw)
        size = _run_tool([
            ffprobe, "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=width,height", "-of", "csv=p=0", clip,
        ], text=True).stdout.strip()
        if size != f"{w},{h}":
            raise RuntimeError(f"ffprobe read the size as {size!r}, expected {w},{h}")
        out_w, out_h = 640, 480
        decoded = _run_tool([
            ffmpeg, "-hide_banner", "-loglevel", "error", "-i", clip,
            "-vf", _ffmpeg_vf_rgb_fixed_size(out_w, out_h), "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1",
        ]).stdout
        frame_bytes = out_w * out_h * 3
        if len(decoded) < frame_bytes or len(decoded) % frame_bytes:
            raise RuntimeError(f"decoding returned {len(decoded)} bytes, not whole {out_w}x{out_h} frames")


def check_ffmpeg() -> str:
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if ffmpeg is None:
        raise CheckWarning("not bundled in this build and not on PATH. Live video and recording need FFmpeg.")
    bundled = _is_bundled(Path(ffmpeg))
    where = "bundled" if bundled else "from PATH"
    version_line = _run_tool([ffmpeg, "-hide_banner", "-version"], text=True).stdout.splitlines()[0]
    version = version_line.split()[2] if len(version_line.split()) > 2 else version_line
    try:
        if ffprobe is None:
            raise RuntimeError("no ffprobe, so UDP video falls back to a 1280x720 canvas")
        _ffmpeg_round_trip(ffmpeg, ffprobe)
    except Exception as error:
        if bundled:
            raise
        raise CheckWarning(f"{where} FFmpeg {version} at {ffmpeg}: {error}") from error
    return f"{where} FFmpeg {version}: recorded (libx264 MP4), probed and decoded a test clip"


def check_qt_and_map_page() -> str:
    # Import order as in the app: Qt WebEngine before the QApplication exists.
    from PySide6 import QtMultimedia  # noqa: F401  (used by the video widgets)
    from PySide6.QtCore import qVersion
    from PySide6.QtGui import QImageReader
    from PySide6.QtWidgets import QApplication

    from vgcs.main import _apply_webengine_chromium_flags_from_env
    from vgcs.map.legacy_leaflet_build import build_leaflet_html
    from vgcs.map.map_web_3d import HAS_WEBENGINE, assets_base_url

    if not HAS_WEBENGINE:
        raise RuntimeError("Qt WebEngine did not load")
    from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile, QWebEngineUrlRequestInterceptor

    class OfflineOnly(QWebEngineUrlRequestInterceptor):
        """Block the network. The page falls back to a CDN copy of Leaflet and
        of Cesium, so without this the check would pass online even if the
        bundled copies are missing. Blocked, it proves the map works offline,
        as in the field."""

        def interceptRequest(self, info) -> None:
            if info.requestUrl().scheme() in ("http", "https"):
                info.block(True)

    _apply_webengine_chromium_flags_from_env()
    app = QApplication.instance() or QApplication([sys.argv[0]])
    formats = {bytes(f).decode() for f in QImageReader.supportedImageFormats()}
    missing = sorted({"png", "jpg", "svg"} - formats)
    if missing:
        raise RuntimeError(f"Qt image formats missing: {missing} (map tiles or icons would not show)")

    result: dict[str, object] = {}
    profile = QWebEngineProfile()  # off the record: no cache or cookies on disk
    interceptor = OfflineOnly(profile)
    profile.setUrlRequestInterceptor(interceptor)
    page = QWebEnginePage(profile)
    try:
        page.loadFinished.connect(lambda ok: result.setdefault("loaded", ok))
        page.setHtml(build_leaflet_html(), assets_base_url())
        if not _wait_for(app, lambda: "loaded" in result, 60.0):
            raise RuntimeError("the map page did not finish loading in 60 s")
        if not result["loaded"]:
            raise RuntimeError("the map page failed to load")
        page.runJavaScript("typeof L", 0, lambda value: result.setdefault("leaflet", value))
        if not _wait_for(app, lambda: "leaflet" in result, 20.0):
            raise RuntimeError("the map page's JavaScript did not answer in 20 s")
        if result["leaflet"] != "object":
            raise RuntimeError(f"Leaflet did not load from the bundled files (typeof L = {result['leaflet']!r})")
        page.runJavaScript("typeof Cesium", 0, lambda value: result.setdefault("cesium", value))
        if not _wait_for(app, lambda: "cesium" in result, 20.0):
            raise RuntimeError("the map page's JavaScript did not answer in 20 s")
        if result["cesium"] != "object":
            raise RuntimeError(
                f"Cesium (the 3D view) did not load from the bundled files (typeof Cesium = {result['cesium']!r})"
            )
    finally:
        # The page must go before its profile, or Qt WebEngine complains at exit.
        page.deleteLater()
        _wait_for(app, lambda: False, 0.3)
        profile.deleteLater()
        _wait_for(app, lambda: False, 0.3)
    return (
        f"Qt {qVersion()}, png/jpg/svg images, map page loads the bundled Leaflet and Cesium (3D) "
        "with the network blocked"
    )


def run_selfcheck(build: str = "VGCS from source", data_dir: Path | None = None) -> int:
    folder = data_dir or Path.cwd()
    checks: list[tuple[str, Callable[[], str]]] = [
        ("data folder", lambda: check_data_dir(folder)),
        ("bundled files", check_bundled_files),
        ("license", check_license),
        ("MAVLink", check_mavlink),
        ("serial ports", check_serial_ports),
        ("MGRS grid reference", check_grid_reference),
        ("GeoTIFF DEM", check_geotiff_dem),
        ("OpenCV", check_opencv),
        ("tracker worker", check_tracker_worker),
        ("detector worker", check_detector_worker),
        ("FFmpeg", check_ffmpeg),
        # Last: a native crash in Qt WebEngine would end the run.
        ("Qt and map page", check_qt_and_map_page),
    ]
    kind = "source"
    if _frozen():
        # The single VGCS.exe unpacks to %LOCALAPPDATA%\VGCS\app\<version>.
        app_root = Path(sys.executable).resolve().parents[1]
        single = app_root.name == "app" and app_root.parent.name == "VGCS"
        kind = "single-file exe, unpacked copy" if single else "folder exe"
    print(f"VGCS self-check: {build}")
    print(f"Python {sys.version.split()[0]}, {kind}: {sys.executable}", flush=True)
    failed = warned = 0
    for name, check in checks:
        started = time.monotonic()
        trace = ""
        try:
            status, detail = "OK  ", check()
        except CheckWarning as warning:
            status, detail = "WARN", str(warning)
            warned += 1
        except Exception as error:
            status, detail = "FAIL", f"{type(error).__name__}: {error}"
            trace = traceback.format_exc()
            failed += 1
        print(f"[{status}] {name}: {detail} ({time.monotonic() - started:.1f} s)", flush=True)
        if trace:
            print(textwrap.indent(trace.rstrip(), "       "), flush=True)
    print(f"Self-check finished: {failed} failed, {warned} warning(s).", flush=True)
    return failed


if __name__ == "__main__":
    # From source: make "vgcs" importable here and in the worker processes.
    repo = str(Path(__file__).resolve().parents[1])
    sys.path.insert(0, repo)
    os.environ["PYTHONPATH"] = os.pathsep.join(p for p in (repo, os.environ.get("PYTHONPATH", "")) if p)
    raise SystemExit(run_selfcheck())
