"""The position source of step 3b: the down camera's pictures matched with the map on board.

Its own threads, so that GPS_INPUT keeps going out ten times a second however long a picture takes:

- the camera thread takes a picture every ``interval_s`` while the drone is armed and high enough
  (the Pi camera waits for its next frame, the stand-in draws one);
- the runner's thread (``tick``) waits until the flight controller's samples cover each picture's
  moment, hands it to the matching with the drone's angles, height and position then, and turns
  each answer into a fix for the sender;
- the matching thread matches one picture at a time (matcher.py). When it is busy, a newer picture
  takes the place of the one waiting;
- with --record, the recording's own thread writes the flight folder (recording.py): the card can
  take a second to save, and the runner's thread must not wait for it.

Each fix is moved from its picture's moment to the flight controller's newest sample by the
controller's own movement in between (a few tenths of a second, which its sensors measure well),
so the matching's delay does not show as a position or a speed late by that much. The speed is the
flight controller's own, corrected by how much the ground's shift between two pictures (up to 3 s
apart) differs from the controller's own movement over the same time: right whatever the drone did
in between, a turn too. The shift is first freed of the EKF's own roll and pitch corrections in
between, by the gyros' turn (ATTITUDE's rates at 100 Hz, ``_tilt_shift``). Only a fix that agrees
with the track (track.py) goes to the sender.
"""

from __future__ import annotations

import bisect
import csv
import logging
import math
import queue
import re
import statistics
import threading
import time
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vama_recorder.camera import CameraError
from vama_recorder.telemetry import Telemetry

from .geo import north_east, offset
from .geometry import Pose, body_in_ned, rotation, rotation_vector, tilt_shift_m, turn_of
from .gps_input import Fix
from .matcher import Job, Matcher, Result, motion_m
from .track import Point, Track

log = logging.getLogger("vama_nav.match")

# ATTITUDE (a picture's angles matter most: one degree is about 9 m on the ground from 500 m up; at
# 100 Hz its turn rates summed between two pictures keep the EKF's own angle corrections out of the
# speed, see _tilt_shift), GLOBAL_POSITION_INT (position, height, speed), SYSTEM_TIME (the clock fit,
# the recorded rows' time).
STREAMS_HZ = {30: 100.0, 33: 10.0, 2: 1.0}
COVER_WAIT_NS = 600_000_000         # a picture waits this long at most for the samples after its moment
STATUS_EVERY_NS = 10_000_000_000
WHY_EVERY_NS = 5_000_000_000        # "no position: ..." at most this often, unless the reason changes
SPEED_KEEP_NS = 3_000_000_000       # the last measured speed, brought forward, stands in this long
ALT_ACC_M = 5.0                     # the height sent is the flight controller's own (the EKF takes its
                                    # height from the barometer, not from this GPS)
EKF_SPEED_ACC = 1.0                 # no speed measured: the controller's own speed, weakly
SPEED_TIPPING_MAX = 0.8             # m/s a speed may be off from the drone's tipping (TIMING_S): over it, unused
SPEED_GATE_MPS = 4.0                # a measured speed this far from the controller's own: unused
RATE_SPAN_NS = 100_000_000          # the drone's turning rates over this time before a picture
MAX_RATE_GAP_MS = 30.0              # ATTITUDE's turn rates further apart: the gyros' turn is not summed
RATES_KEEP_MS = 10_000              # ATTITUDE's turn rates kept this long behind the newest
GYRO_BIAS_TAU_S = 40.0              # the gyros' bias left in ATTITUDE's rates is learnt over this long
GYRO_BIAS_MAX_RAD_S = math.radians(0.5)   # gyros and EKF differ more (a heading reset): not learnt from
# ATTITUDE's rates come through the gyros' low-pass filter (INS_GYRO_FILTER, two poles), its angles do
# not: the rates are late by about 0.225 s / the filter's Hz. ArduCopter's default 20 Hz: 11 ms. Not
# taken out, entering and leaving a turn (10 to 20 degrees a second of roll) the gyros' turn between two
# pictures was 0.2 degrees off, 1 m from 300 m (the simulator, 2026-10-29).
GYRO_FILTER_PARAM = "INS_GYRO_FILTER"
GYRO_FILTER_DEFAULT_HZ = 20.0
GYRO_LAG_S_HZ = math.sqrt(2.0) / (2.0 * math.pi)
HEADING_NOTE_DEG = 5.0              # the controller's heading found this far off (or changed by this): said
# One row per picture (--results): what was found, what the flight controller said, what was sent.
RESULT_FIELDS = ["local_ns", "ok", "why", "lat", "lon", "horiz_acc_m", "inliers", "pairs", "turn_deg", "scale",
                 "rechecked", "heading_error_deg", "height_factor", "motion_m", "off_prior_m", "moved_n_m",
                 "moved_e_m", "moved_s", "tilt_shift_n_m", "tilt_shift_e_m", "speed_used", "fc_lat", "fc_lon", "fc_vn", "fc_ve", "fc_yaw_deg",
                 "fc_rel_alt_m", "sent", "fix_local_ns", "fix_lat", "fix_lon", "fix_vn", "fix_ve", "fix_speed_acc",
                 "seconds", "true_lat", "true_lon", "true_fix_lat", "true_fix_lon"]
# true_*: only with the simulator's stand-in camera, its truth at the picture's and at the fix's moment.


@dataclass(frozen=True)
class SourceSettings:
    interval_s: float = 1.0
    match_above_m: float = 50.0     # no pictures below: from low the camera sees too little of the map
    search_m: float = 60.0          # how far from the controller's position the map is searched
    search_grow_mps: float = 5.0    # and how fast that grows without a sent position (and GPS 1)
    max_search_m: float = 400.0
    speed_acc_mps: float = 0.3      # the measured speed's error (one sigma), plus 0.1 % of the height
    track_gate_m: float = 25.0

    def problem(self) -> str | None:
        if not 0.2 <= self.interval_s <= 10:
            return "interval_s must be from 0.2 to 10"
        if self.match_above_m < 0 or self.search_m <= 0 or self.max_search_m < self.search_m:
            return "the heights and search distances must be positive, max_search_m at least search_m"
        return None


@dataclass
class _Picture:
    local_ns: int
    image: np.ndarray


class MatchSource:
    PARAMS = (GYRO_FILTER_PARAM,)       # the settings it reads, asked for by the runner

    def __init__(self, offer: Callable[[Fix], object], camera, matcher: Matcher, settings: SourceSettings | None = None,
                 telemetry: Telemetry | None = None, clock: Callable[[], int] = time.monotonic_ns,
                 threaded: bool = True, results: Path | None = None, recording=None) -> None:
        """``results``: a CSV file that gets one row per picture (RESULT_FIELDS). ``recording``: a
        recording.Recording that keeps the pictures in flight folders, or None."""
        self.offer = offer
        self.camera = camera
        self.matcher = matcher
        self.settings = settings or SourceSettings()
        problem = self.settings.problem()
        if problem:
            raise ValueError(problem)
        self.telemetry = telemetry or Telemetry()
        self.clock = clock
        self.threaded = threaded
        self.STREAMS_HZ = dict(STREAMS_HZ)
        self.STREAMS_HZ.update(getattr(camera, "STREAMS_HZ", {}))
        self.track = Track(gate_m=self.settings.track_gate_m)
        self.counts: Counter = Counter()
        self.no_position: Counter = Counter()      # reasons, for the status line
        self.match_seconds: deque = deque(maxlen=60)
        self.errors_m: deque = deque(maxlen=600)   # (only with a working GPS 1) sent fix vs GPS 1
        self.why = ""
        self._why_logged_ns = 0
        self._wanted = threading.Event()
        self._forget = threading.Event()
        self._stop = threading.Event()
        self._pictures: queue.Queue = queue.Queue()
        self._jobs: queue.Queue = queue.Queue(maxsize=1)
        self._results: queue.Queue = queue.Queue()
        self._waiting: list[_Picture] = []
        self._threads: list[threading.Thread] = []
        self._next_shot: int | None = None
        self._camera_why = ""
        self._last_sent_ns: int | None = None
        self._gps_good_ns: int | None = None
        self._speed: tuple[int, float, float, float] | None = None  # (moment, vn, ve, its extra error) last measured
        self._motion: dict[int, float] = {}         # picture moment: how far its tipping may move it (m)
        self._shift: tuple[int, tuple | None] = (0, None)   # (picture moment, its tilt shift (north, east) or None)
        self._turn_rates: list[tuple] = []          # ATTITUDE's (boot_ms, roll, pitch, yaw rate) rad/s, the body's
        self._turn_rates_lock = threading.Lock()    # the link's thread hands the messages in
        self._gyro_bias = np.zeros(3)               # rad/s left in ATTITUDE's rates, learnt (_tilt_shift)
        self._gyro_bias_boot = 0.0                  # the controller's time it was last learnt at
        self.gyro_lag_ms = 1000.0 * GYRO_LAG_S_HZ / GYRO_FILTER_DEFAULT_HZ   # until INS_GYRO_FILTER is read
        self.gyro_filter_hz: float | None = None
        self._speed_note = ""
        self._heading_said = 0.0
        self._next_status: int | None = None
        self._status_counts: Counter = Counter()
        self.results_path = results
        self._results_file = None
        self._rows = None
        self.recording = recording
        self.matching = False               # pictures go to the matching (armed, high enough)

    # --- the runner's calls ---------------------------------------------------------------

    def start(self) -> None:
        if self.results_path is not None:
            self._results_file = open(self.results_path, "w", newline="", encoding="utf-8")  # noqa: SIM115
            self._rows = csv.writer(self._results_file)
            self._rows.writerow(RESULT_FIELDS)
        self.camera.start()
        if self.threaded:
            if self.recording is not None:
                self.recording.start()
            for name, target in (("camera", self._camera_loop), ("matching", self._match_loop)):
                th = threading.Thread(target=target, name=name, daemon=True)
                th.start()
                self._threads.append(th)

    def close(self) -> None:
        self._stop.set()
        self._wanted.clear()
        for th in self._threads:
            th.join(timeout=5.0)
        self.camera.stop()
        if self.recording is not None:
            self.recording.close(self.clock(), "vama_nav stopped")
        if self._results_file is not None:
            self._results_file.close()
            self._results_file = self._rows = None

    def hear(self, msg, arrival_ns: int) -> None:
        fc = self.telemetry.hear(msg, arrival_ns)
        if fc and msg.get_type() == "PARAM_VALUE" and str(msg.param_id).rstrip("\x00") == GYRO_FILTER_PARAM:
            hz = float(msg.param_value)
            if hz != self.gyro_filter_hz:
                self.gyro_filter_hz = hz
                self.gyro_lag_ms = 1000.0 * GYRO_LAG_S_HZ / hz if hz > 0 else 0.0     # 0: no filter
                self.gyro_lag_ms = min(self.gyro_lag_ms, 50.0)
                log.info("the gyros' filter is %g Hz: ATTITUDE's turn rates are about %.1f ms late", hz,
                         self.gyro_lag_ms)
        if fc and msg.get_type() == "ATTITUDE":
            sample = (int(msg.time_boot_ms), float(msg.rollspeed), float(msg.pitchspeed), float(msg.yawspeed))
            with self._turn_rates_lock:
                if self._turn_rates and sample[0] < self._turn_rates[-1][0] - 1000:
                    self._turn_rates.clear()                    # the controller restarted: its clock began again
                bisect.insort(self._turn_rates, sample, key=lambda s: s[0])
                cut = bisect.bisect_left(self._turn_rates, self._turn_rates[-1][0] - RATES_KEEP_MS, key=lambda s: s[0])
                del self._turn_rates[:cut]
        hear = getattr(self.camera, "hear", None)
        if hear is not None:
            hear(msg, arrival_ns)
        if self.recording is not None:
            self.recording.hear(msg, arrival_ns)

    def tick(self, now_ns: int) -> None:
        t = self.telemetry
        gps = t.gps_now(now_ns)
        if gps is not None and gps.get("fix", 0) >= 3:
            self._gps_good_ns = now_ns
        height = t.rel_alt_now(now_ns)
        match = t.link_alive(now_ns) and t.armed and height is not None and height >= self.settings.match_above_m
        if match and not self.matching:
            self.matching = True
            log.info("matching from now on (armed, %.0f m above home)", height)
        elif not match and self.matching:
            self.matching = False
            self.matcher_forget()
            log.info("no matching: %s", "disarmed" if not t.armed else "no flight controller" if not t.link_alive(now_ns)
                     else f"below {self.settings.match_above_m:g} m")
        record = False
        if self.recording is not None:     # its own thread writes; without threads it writes here
            record = self.recording.active if self.threaded else self.recording.poll(now_ns)
        if match or record:
            self._wanted.set()
        else:
            self._wanted.clear()
        if not self.threaded:
            self._camera_step(now_ns)
        while True:
            try:
                self._waiting.append(self._pictures.get_nowait())
            except queue.Empty:
                break
        self._hand_over(now_ns)
        if not self.threaded:
            try:
                job = self._jobs.get_nowait()
            except queue.Empty:
                job = None
            if job is not None:
                self._results.put(self._match(job))
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                break
            self._use(result, now_ns)
        self._status(now_ns)

    def matcher_forget(self) -> None:
        """Pictures stopped (landed, below the height): the next one starts afresh."""
        self._next_shot = None
        if not self.threaded:
            self.matcher.forget()
        else:
            self._forget.set()               # the matching thread forgets before its next picture

    # --- the camera ---------------------------------------------------------------------------

    def _camera_loop(self) -> None:
        while not self._stop.is_set():
            if not self._wanted.wait(0.2):
                continue
            now = self.clock()
            if self._next_shot is not None and now < self._next_shot:
                self._stop.wait(min((self._next_shot - now) / 1e9, 0.05))
                continue
            self._camera_step(now)

    def _camera_step(self, now_ns: int) -> None:
        if not self._wanted.is_set():
            return
        if self._next_shot is not None and now_ns < self._next_shot:
            return
        every = int(self.settings.interval_s * 1e9)
        self._next_shot = now_ns + every if self._next_shot is None or self._next_shot + every <= now_ns \
            else self._next_shot + every
        try:
            shot = self.camera.capture()
            try:
                image = shot.pixels()
                if self.recording is not None:
                    self.recording.keep(shot, now_ns)          # saved with its row, or not recording
            finally:
                shot.discard()
        except CameraError as exc:
            self._camera_failed(str(exc))
            return
        except Exception as exc:
            log.exception("camera")
            self._camera_failed(f"{type(exc).__name__}: {exc}")
            return
        if image is None:
            self._camera_failed("the camera gives no picture to match")
            return
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self._camera_why:
            log.info("the camera gives pictures again")
            self._camera_why = ""
        self.counts["pictures"] += 1
        if self.matching:
            self._pictures.put(_Picture(int(shot.sensor_ns), image))

    def _camera_failed(self, why: str) -> None:
        self.counts["camera errors"] += 1
        if why != self._camera_why:
            log.warning("camera: %s", why)
            self._camera_why = why

    # --- the matching ---------------------------------------------------------------------------

    def _match_loop(self) -> None:
        while not self._stop.is_set():
            try:
                job = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            if self._forget.is_set():
                self._forget.clear()
                self.matcher.forget()
            self._results.put(self._match(job))

    def _match(self, job: Job) -> Result:
        try:
            return self.matcher.process(job)
        except Exception as exc:
            log.exception("the matching failed")
            return Result(job.local_ns, False, f"the matching failed: {type(exc).__name__}: {exc}")

    def _search_m(self, now_ns: int) -> float:
        s = self.settings
        good = [t for t in (self._last_sent_ns, self._gps_good_ns) if t is not None]
        if not good:
            return s.max_search_m
        age = max(0.0, (now_ns - max(good)) / 1e9)
        return min(s.search_m + s.search_grow_mps * age, s.max_search_m)

    def _state(self, local_ns: int) -> dict | None:
        """The flight controller's state at this computer's ``local_ns``, or None."""
        boot = self.telemetry.boot_ms_at(local_ns)
        if boot is None:
            return None
        state = self.telemetry.state_at(boot)
        if state.get("position_how") == "none" or state.get("attitude_how") == "none":
            return None
        return state

    def _hand_over(self, now_ns: int) -> None:
        keep = []
        for pic in self._waiting:
            boot = self.telemetry.boot_ms_at(pic.local_ns)
            covered = boot is not None and self.telemetry.covered(boot)
            if not covered and now_ns - pic.local_ns <= COVER_WAIT_NS:
                keep.append(pic)
                continue
            state = self._state(pic.local_ns) if covered else None
            if state is None or state.get("rel_alt_m") is None:
                self._no_position(now_ns, "no flight controller samples at the picture's moment")
                continue
            pose = Pose(math.radians(state["roll_deg"]), math.radians(state["pitch_deg"]),
                        math.radians(state["yaw_deg"]), float(state["rel_alt_m"]))
            rates = self._rates(pic.local_ns, state)
            self._motion[pic.local_ns] = motion_m(pose, rates, pose.height_m)
            while len(self._motion) > 20:
                del self._motion[min(self._motion)]
            job = Job(pic.local_ns, pic.image, state["lat"], state["lon"], pose, self._search_m(pic.local_ns), rates)
            try:
                dropped = self._jobs.get_nowait()             # the matching is busy: the newer picture goes first
                if dropped is not None:
                    self.counts["skipped (matching busy)"] += 1
            except queue.Empty:
                pass
            self._jobs.put(job)
        self._waiting = keep

    def _rates(self, local_ns: int, state: dict) -> tuple[float, float, float]:
        """The drone's roll, pitch and yaw rates (rad/s) over the RATE_SPAN_NS before ``local_ns``."""
        before = self._state(local_ns - RATE_SPAN_NS)
        if before is None:
            return 0.0, 0.0, 0.0
        span = RATE_SPAN_NS / 1e9
        return tuple(math.radians(((state[k] - before[k] + 180.0) % 360.0) - 180.0) / span
                     for k in ("roll_deg", "pitch_deg", "yaw_deg"))

    # --- the answers --------------------------------------------------------------------------

    def _moved(self, from_ns: int, to_ns: int) -> tuple[float, float] | None:
        """The flight controller's own movement (north, east) between two moments."""
        a, b = self._state(from_ns), self._state(to_ns)
        if a is None or b is None:
            return None
        return north_east(a["lat"], a["lon"], b["lat"], b["lon"])

    def _tilt_shift(self, from_ns: int, to_ns: int, boot_a: float | None, boot_b: float | None,
                    at: dict) -> tuple[float, float] | None:
        """(north, east) metres the EKF's own roll and pitch corrections between two pictures put into
        the ground's measured shift, by the gyros' turn in between; None without enough turn rates."""
        before = self._state(from_ns)
        if boot_a is None or boot_b is None or before is None or at.get("rel_alt_m") is None:
            self.counts["speeds without the gyros' turn"] += 1
            return None
        # The rates measured the turn gyro_lag_ms before they were stamped (the gyros' filter).
        lag_a, lag_b = boot_a + self.gyro_lag_ms, boot_b + self.gyro_lag_ms
        with self._turn_rates_lock:
            i = bisect.bisect_right(self._turn_rates, lag_a, key=lambda s: s[0]) - 1
            j = bisect.bisect_left(self._turn_rates, lag_b, key=lambda s: s[0])
            rates = self._turn_rates[i:j + 1] if i >= 0 else []
        if rates and rates[-1][0] < lag_b <= rates[-1][0] + MAX_RATE_GAP_MS:
            rates.append((lag_b, *rates[-1][1:]))      # the newest rate held for the last few ms
        turn = turn_of(rates, lag_a, lag_b, MAX_RATE_GAP_MS)
        if turn is None:
            self.counts["speeds without the gyros' turn"] += 1
            return None
        height = float(at["rel_alt_m"]) * getattr(self.matcher, "height_factor", 1.0)

        def pose(state: dict) -> Pose:
            return Pose(math.radians(state["roll_deg"]), math.radians(state["pitch_deg"]),
                        math.radians(state["yaw_deg"]), height)

        a, b = pose(before), pose(at)
        dt = (boot_b - boot_a) / 1000.0
        # ATTITUDE's rates are the gyros corrected by the EKF's own bias estimate, which lags (the
        # simulator drifts its gyros on purpose, 0.02 to 0.04 degrees a second were left). What is left
        # is learnt slowly from how much more the gyros turned than the EKF's angles, which are right in
        # the long run; over the seconds between two pictures the gyros are the better.
        used = turn @ rotation(-self._gyro_bias * dt)
        seen = rotation_vector((body_in_ned(a).T @ body_in_ned(b)).T @ turn) / max(dt, 1e-3)
        if np.linalg.norm(seen) < GYRO_BIAS_MAX_RAD_S:
            gain = min(1.0, max(0.0, (boot_b - self._gyro_bias_boot) / 1000.0) / GYRO_BIAS_TAU_S)
            self._gyro_bias = self._gyro_bias + (seen - self._gyro_bias) * gain
        self._gyro_bias_boot = boot_b
        return tilt_shift_m(a, b, used)

    def _use(self, result: Result, now_ns: int) -> None:
        at = self._state(result.local_ns)
        fix = self._judge(result, at, now_ns)
        if self._rows is not None:
            self._write(result, at, fix)

    def _write(self, result: Result, at: dict | None, fix: Fix | None) -> None:
        mv = result.movement
        at = at or {}
        row = {"local_ns": result.local_ns, "ok": int(result.ok), "why": result.why, "inliers": result.inliers,
               "pairs": result.pairs, "sent": int(fix is not None), "seconds": f"{result.seconds:.3f}"}
        row.update(rechecked=int(result.rechecked), heading_error_deg=f"{result.heading_error_deg:.2f}",
                   height_factor=f"{result.height_factor:.4f}",
                   motion_m=f"{self._motion.get(result.local_ns, 0.0):.2f}",
                   speed_used=int(self._speed is not None and self._speed[0] == result.local_ns))
        if result.ok:
            row.update(lat=f"{result.lat:.7f}", lon=f"{result.lon:.7f}", horiz_acc_m=f"{result.horiz_acc_m:.2f}",
                       turn_deg=f"{result.turn_deg:.2f}", scale=f"{result.scale:.4f}",
                       off_prior_m=f"{result.off_prior_m:.1f}")
        if mv is not None:
            row.update(moved_n_m=f"{mv.north_m:.3f}", moved_e_m=f"{mv.east_m:.3f}",
                       moved_s=f"{(result.local_ns - mv.since_ns) / 1e9:.3f}")
            if self._shift[0] == result.local_ns and self._shift[1] is not None:
                row.update(tilt_shift_n_m=f"{self._shift[1][0]:.3f}", tilt_shift_e_m=f"{self._shift[1][1]:.3f}")
        if at:
            row.update(fc_lat=f"{at['lat']:.7f}", fc_lon=f"{at['lon']:.7f}", fc_vn=f"{at['vn_mps']:.2f}",
                       fc_ve=f"{at['ve_mps']:.2f}", fc_yaw_deg=f"{at['yaw_deg']:.1f}",
                       fc_rel_alt_m=f"{at['rel_alt_m']:.1f}")
        if fix is not None:
            row.update(fix_local_ns=fix.local_ns, fix_lat=f"{fix.lat:.7f}", fix_lon=f"{fix.lon:.7f}",
                       fix_vn=f"{fix.vn:.2f}", fix_ve=f"{fix.ve:.2f}", fix_speed_acc=f"{fix.speed_acc_mps:.2f}")
        truth_at = getattr(self.camera, "truth_at", None)
        if truth_at is not None:
            for prefix, moment in (("true_", result.local_ns), ("true_fix_", fix.local_ns if fix else None)):
                where = truth_at(moment) if moment is not None else None
                if where is not None:
                    row.update({prefix + "lat": f"{where[0]:.7f}", prefix + "lon": f"{where[1]:.7f}"})
        self._rows.writerow([row.get(k, "") for k in RESULT_FIELDS])     # flushed with each status line

    def _judge(self, result: Result, at: dict | None, now_ns: int) -> Fix | None:
        """The fix sent for this result, or None."""
        self.counts["matched" if result.ok else "no position"] += 1
        self.match_seconds.append(result.seconds)
        if result.movement is not None and at is not None:
            mv = result.movement
            # The time between the pictures on the flight controller's clock, the clock its speeds count in
            # (the simulator's runs a few percent slow against this computer's).
            b1, b2 = self.telemetry.boot_ms_at(mv.since_ns), self.telemetry.boot_ms_at(result.local_ns)
            dt = (b2 - b1) / 1000.0 if b1 is not None and b2 is not None else (result.local_ns - mv.since_ns) / 1e9
            fc_moved = self._moved(mv.since_ns, result.local_ns)
            # The EKF's own corrections of its roll and pitch between the two pictures, taken out by the
            # gyros' turn (geometry.tilt_shift_m). Without the rates the shift stays as measured.
            shift = self._tilt_shift(mv.since_ns, result.local_ns, b1, b2, at)
            self._shift = (result.local_ns, shift)
            north, east = (mv.north_m + shift[0], mv.east_m + shift[1]) if shift else (mv.north_m, mv.east_m)
            if fc_moved is not None and dt > 0:
                # The controller's speed, plus what the measured movement says it got wrong meanwhile.
                # (The shift's mean speed brought forward by the controller's speed change was nearer
                # the truth, but it tells the EKF its speed error of a second or more ago as if it were
                # now: in the simulator the drone then tipped three times as much, 2026-10-29.)
                vn = at["vn_mps"] + (north - fc_moved[0]) / dt
                ve = at["ve_mps"] + (east - fc_moved[1]) / dt
                tipping = (self._motion.get(mv.since_ns, 0.0) + self._motion.get(result.local_ns, 0.0)) / dt
                off = math.hypot(vn - at["vn_mps"], ve - at["ve_mps"])
                if tipping > SPEED_TIPPING_MAX:
                    note = "the drone tipped too fast for the speed between two pictures"
                elif off > SPEED_GATE_MPS:
                    note = f"the speed between two pictures is {off:.1f} m/s from the flight controller's"
                else:
                    note = ""
                    self._speed = (result.local_ns, vn, ve, tipping)
                    self.counts["speeds measured"] += 1
                if note:
                    self.counts["speeds not used"] += 1
                    if note != self._speed_note:
                        log.info("speed not used: %s", note)
                self._speed_note = note
        said = self._heading_said
        if result.ok and (abs(result.heading_error_deg - said) >= HEADING_NOTE_DEG
                          or (abs(result.heading_error_deg) >= HEADING_NOTE_DEG) != (abs(said) >= HEADING_NOTE_DEG)):
            log.info("the flight controller's heading seems %.1f degrees off (the pictures are turned by that)",
                     result.heading_error_deg)
            self._heading_said = result.heading_error_deg
        if not result.ok:
            self._no_position(now_ns, result.why)
            return None
        ok, why = self.track.judge(Point(result.local_ns, result.lat, result.lon), self._moved)
        if not ok:
            self.counts["held (track)"] += 1
            self._no_position(now_ns, why)
            return None
        if why:
            log.info("%s", why)
        fix = self._fix(result, at)
        if fix is None:
            self._no_position(now_ns, "the flight controller's samples since the picture are missing")
            return None
        if not self.offer(fix):
            return None
        self.counts["sent"] += 1
        self._last_sent_ns = result.local_ns
        if self.why:
            log.info("positions again (%d features agree, about %.1f m)", result.inliers, result.horiz_acc_m)
            self.why = ""
        gps = self.telemetry.gps_now(now_ns)
        if gps is not None and gps.get("fix", 0) >= 3 and at is not None:
            # GPS 1 works: how far the camera's position is from the controller's, at the picture's moment.
            self.errors_m.append(math.hypot(*north_east(at["lat"], at["lon"], result.lat, result.lon)))
        return fix

    def _fix(self, result: Result, at: dict | None) -> Fix | None:
        """The matched position and the speed, moved to the controller's newest sample."""
        newest_ms = self.telemetry.newest_position_ms()
        newest_ns = self.telemetry.local_ns_at(newest_ms) if newest_ms is not None else None
        if at is None or newest_ns is None:
            return None
        now = self._state(newest_ns) if newest_ns > result.local_ns else at
        if now is None:
            return None
        step_n, step_e = north_east(at["lat"], at["lon"], now["lat"], now["lon"])
        lat, lon = offset(result.lat, result.lon, step_n, step_e)
        moment = max(newest_ns, result.local_ns)
        height = float(at.get("rel_alt_m") or 0.0)
        speed_acc = self.settings.speed_acc_mps + 0.001 * height
        sp = self._speed
        if sp is not None and moment - sp[0] <= SPEED_KEEP_NS:
            then = self._state(sp[0])
            if then is None:
                return None
            vn, ve = sp[1] + now["vn_mps"] - then["vn_mps"], sp[2] + now["ve_mps"] - then["ve_mps"]
            speed_acc += sp[3] + (0.2 * (moment - sp[0]) / 1e9 if sp[0] != result.local_ns else 0.0)
        else:
            vn, ve, speed_acc = now["vn_mps"], now["ve_mps"], EKF_SPEED_ACC
        # More than one fix a second: each weighs as much less as there are more of them, so that a
        # second of fixes weighs what one fix a second did. Pictures that close see nearly the same
        # ground, and their errors are nearly the same (a second apart they were 55 to 80 % alike
        # without GPS 1, in the simulator): the EKF must not take them as new knowledge each time. By
        # the time since the last fix sent: a picture skipped (a slow matching, no position) leaves
        # the next one its full weight.
        since = 1.0 if self._last_sent_ns is None else (result.local_ns - self._last_sent_ns) / 1e9
        weight = math.sqrt(1.0 / min(1.0, max(since, self.settings.interval_s)))
        return Fix(local_ns=moment, lat=lat, lon=lon, alt_msl_m=float(now["alt_msl_m"]), vn=vn, ve=ve,
                   horiz_acc_m=result.horiz_acc_m * weight, vert_acc_m=ALT_ACC_M,
                   speed_acc_mps=min(speed_acc, EKF_SPEED_ACC) * weight)

    def _no_position(self, now_ns: int, why: str) -> None:
        kind = re.sub(r"\d+(\.\d+)?", "N", why.split(" (")[0].split(";")[0])   # "only N features pair with the map"
        self.no_position[kind] += 1
        if why != self.why and (now_ns - self._why_logged_ns >= WHY_EVERY_NS or not self.why):
            log.info("no position: %s", why)
            self._why_logged_ns = now_ns
        self.why = why

    def _status(self, now_ns: int) -> None:
        if self._next_status is None:
            self._next_status = now_ns + STATUS_EVERY_NS
            return
        if now_ns < self._next_status:
            return
        self._next_status = now_ns + STATUS_EVERY_NS
        if self._results_file is not None:
            self._results_file.flush()
        done = self.counts - self._status_counts
        self._status_counts = Counter(self.counts)
        if not done:
            return
        took = f", matching {statistics.median(self.match_seconds):.2f} s a picture" if self.match_seconds else ""
        gps1 = (f", camera vs GPS 1 middle {statistics.median(self.errors_m):.1f} m" if self.errors_m else "")
        log.info("pictures %d, positions sent %d, held by the track %d, no position %d%s%s", done["pictures"],
                 done["sent"], done["held (track)"], done["no position"], took, gps1)
