"""The picture matching: where on the map the drone is, from one picture of the down camera.

Each picture is turned north up at the map's scale with the drone's angles and height at its
moment (geometry.rectify). It then lies on the map by a shift alone, so features found in both
(ORB by default, AKAZE or SIFT on request, all in OpenCV) are paired, and the shift is fitted to
the pairs that agree (RANSAC, a shift with a small turn and scale, estimateAffinePartial2D). The
point below the drone, carried by that fit onto the map, is its position.

Only near where the drone should be is searched (``search_m`` around the flight controller's own
position), and a fit is a position only when it passes every check below. A position that is not
sure is no position: the flight controller then holds on its own sensors for a few seconds.

- Enough pairs agree (``min_inliers``), and they spread over the picture, not one corner of it.
- No map point is chosen by two picture points (a lone map feature that "matches" everything).
- The fit's turn and scale fit the drone's heading and height (see below).
- No second place fits nearly as well (rows of houses or of trees repeat).
- The place is within ``search_m`` of where the drone should be.

The fits also tell how far off the flight controller's heading and height are, and the matcher
follows that: each picture is turned with the heading corrected by the error found so far, and
scaled with the height corrected. A picture that still turns more than ``recheck_turn_deg`` is
tried again with its own turn taken out, so a heading up to ``max_heading_error_deg`` off (an EKF
that reset its heading, a compass near iron) still gives positions, and the movement between two
pictures is in true north and east. In the simulator an EKF heading reset of 20 degrees otherwise
stopped every position after it (2026-10-29).

The same features give the drone's movement over the last few seconds (two pictures north up at the
same scale: the shift between them), from which match_source.py takes the speed. It is measured
against the oldest picture of up to ``vo_max_gap_s`` before: each picture is turned with the flight
controller's angles, and an error in those that changes between the two pictures shows as a false
movement (0.15 degrees is 0.8 m from 300 m). Over 3 s instead of 1 that is a third of the speed
error. In the simulator with GPS 1 off, the speed from one second was up to 4 m/s off.
"""

from __future__ import annotations

import math
import time
from collections import Counter, deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from .geometry import Lens, Pose, Rectified, rectify

FEATURES = ("orb", "akaze", "sift")
# How far a picture's moment may be off the moment of the drone's angles it is turned with (the
# camera's own time stamp, its rolling shutter). While the drone tips, the picture moves by its
# tipping rate times this: at 300 m and 20 degrees a second, 2 m.
TIMING_S = 0.02


@dataclass(frozen=True)
class MatchSettings:
    features: str = "orb"
    max_features: int = 3000            # per picture, and per map search area
    ratio: float = 0.8                  # a pair counts when its best map feature is clearly better than the next
    ransac_px: float = 3.0              # a pair agrees with a fit within this many map pixels
    min_inliers: int = 15
    min_spread: float = 0.06            # the agreeing points' spread, as a share of the picture's size
    max_turn_deg: float = 8.0           # the final fit may turn the picture this much
    recheck_turn_deg: float = 2.0       # a first fit turned more than this: tried again with the turn taken out
    max_heading_error_deg: float = 45.0  # the flight controller's heading may be this far off
    min_scale: float = 0.8              # the fit may scale the picture this much (height error)
    max_scale: float = 1.25
    follow: float = 0.3                 # how much of each fit's heading and height error is followed
    ambiguous: float = 0.6              # a second place with this share of the best's agreeing pairs: no fix
    map_error_m: float = 3.0            # the map's own placement error (one sigma), in every position
    attitude_error_deg: float = 0.3     # the drone's angles' error, in every position
    min_accuracy_m: float = 1.5
    vo_min_inliers: int = 20
    vo_max_turn_deg: float = 5.0
    vo_max_scale_error: float = 0.08
    vo_max_gap_s: float = 3.0           # the movement is measured against the oldest picture this far back

    def problem(self) -> str | None:
        if self.features not in FEATURES:
            return f"features must be one of {', '.join(FEATURES)} (it is '{self.features}')"
        if self.min_inliers < 6:
            return "min_inliers must be 6 or more"
        if not 0 < self.ratio < 1:
            return "ratio must be between 0 and 1"
        if not self.min_scale < 1 < self.max_scale:
            return "the scale range must hold 1"
        if not 0 <= self.follow <= 1:
            return "follow must be from 0 to 1"
        return None


@dataclass
class Job:
    """One picture, with what the flight controller said at its moment."""

    local_ns: int
    image: np.ndarray                   # grey
    prior_lat: float                    # the flight controller's own position
    prior_lon: float
    pose: Pose                          # its angles and its height above the ground
    search_m: float
    rates: tuple[float, float, float] = (0.0, 0.0, 0.0)     # roll, pitch, yaw rates then (rad/s)


@dataclass
class Movement:
    """The drone's movement over the ground from an earlier picture (``since_ns``) to this one."""

    since_ns: int
    north_m: float
    east_m: float
    inliers: int


@dataclass
class Result:
    local_ns: int
    ok: bool
    why: str = ""                       # why there is no position, in plain words
    lat: float = 0.0
    lon: float = 0.0
    horiz_acc_m: float = 0.0
    inliers: int = 0
    pairs: int = 0
    rms_m: float = 0.0
    turn_deg: float = 0.0               # the flight controller's heading error this picture shows
    scale: float = 1.0
    off_prior_m: float = 0.0
    rechecked: bool = False             # tried a second time with the heading corrected
    motion_m: float = 0.0               # how far the drone's tipping may move the picture (TIMING_S)
    heading_error_deg: float = 0.0      # what the matcher follows after this picture
    height_factor: float = 1.0
    movement: Movement | None = None
    seconds: float = 0.0
    timings: dict = field(default_factory=dict)


def motion_m(pose: Pose, rates: tuple[float, float, float], height_m: float) -> float:
    """How far on the ground the picture may move in TIMING_S while the drone tips and turns: tipping
    moves all of it, turning moves it around the point below the drone (the picture's middle is that
    far from it when the drone leans)."""
    roll_rate, pitch_rate, yaw_rate = rates
    tilt = math.acos(max(-1.0, min(1.0, math.cos(pose.roll) * math.cos(pose.pitch))))
    reach = height_m * math.tan(min(tilt, math.radians(80)))
    return TIMING_S * (height_m * math.hypot(roll_rate, pitch_rate) + reach * abs(yaw_rate))


class Features:
    def __init__(self, kind: str, max_features: int) -> None:
        self.kind = kind
        self.max_features = int(max_features)
        if kind == "orb":
            self._det = cv2.ORB_create(nfeatures=self.max_features, fastThreshold=10)
            self.norm = cv2.NORM_HAMMING
        elif kind == "akaze":
            self._det = cv2.AKAZE_create(threshold=0.0005)
            self.norm = cv2.NORM_HAMMING
        elif kind == "sift":
            self._det = cv2.SIFT_create(nfeatures=self.max_features)
            self.norm = cv2.NORM_L2
        else:
            raise ValueError(f"unknown features '{kind}'")

    def find(self, image: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
        """(points as an N x 2 array, descriptors or None)."""
        points = self._det.detect(image, mask)
        if len(points) > self.max_features:               # AKAZE has no limit of its own
            points = sorted(points, key=lambda p: -p.response)[: self.max_features]
        points, desc = self._det.compute(image, points)
        if desc is None or not points:
            return np.zeros((0, 2), np.float32), None
        return np.float32([p.pt for p in points]), desc

    def pair(self, d1: np.ndarray, d2: np.ndarray, ratio: float) -> list[tuple[int, int]]:
        """Pairs (index in the first, index in the second) that pass the ratio test, each feature
        of the second used once at most."""
        if d1 is None or d2 is None or len(d1) < 1 or len(d2) < 2:      # the ratio test needs two map features
            return []
        if self.norm == cv2.NORM_L2:
            matcher = cv2.FlannBasedMatcher({"algorithm": 1, "trees": 4}, {"checks": 48})
        else:
            matcher = cv2.BFMatcher(self.norm)
        good = [m[0] for m in matcher.knnMatch(d1, d2, k=2) if len(m) == 2 and m[0].distance < ratio * m[1].distance]
        used = Counter(m.trainIdx for m in good)
        return [(m.queryIdx, m.trainIdx) for m in good if used[m.trainIdx] == 1]


@dataclass
class _Fit:
    m: np.ndarray                       # 2 x 3, picture pixel to target pixel
    inliers: np.ndarray                 # bool per pair
    turn_deg: float
    scale: float
    rms_px: float


def _fit(src: np.ndarray, dst: np.ndarray, ransac_px: float) -> _Fit | None:
    if len(src) < 3:
        return None
    m, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=ransac_px,
                                         maxIters=4000, confidence=0.999)
    if m is None or inl is None:
        return None
    inliers = inl[:, 0].astype(bool)
    n = int(inliers.sum())
    if n < 3:
        return None
    moved = (m[:, :2] @ src[inliers].T).T + m[:, 2]
    rms = float(np.sqrt(((moved - dst[inliers]) ** 2).sum(axis=1).mean()))
    return _Fit(m, inliers, math.degrees(math.atan2(m[1, 0], m[0, 0])), math.hypot(m[0, 0], m[1, 0]), rms)


def _apply(m: np.ndarray, x: float, y: float) -> tuple[float, float]:
    return float(m[0, 0] * x + m[0, 1] * y + m[0, 2]), float(m[1, 0] * x + m[1, 1] * y + m[1, 2])


def second_place(src: np.ndarray, dst: np.ndarray, fit: _Fit, s: MatchSettings, nadir: tuple[float, float],
                 m_per_px: float) -> tuple[int, float] | None:
    """Another place, nearly as good, that the pairs left out of ``fit`` agree on: (its agreeing
    pairs, metres from the first place), or None when there is none."""
    rest = ~fit.inliers
    if rest.sum() < s.min_inliers:
        return None
    other = _fit(src[rest], dst[rest], s.ransac_px)
    if other is None:
        return None
    n1, n2 = int(fit.inliers.sum()), int(other.inliers.sum())
    x1, y1 = _apply(fit.m, *nadir)
    x2, y2 = _apply(other.m, *nadir)
    apart = math.hypot(x2 - x1, y2 - y1) * m_per_px
    if n2 >= max(s.min_inliers, s.ambiguous * n1) and apart > 3 * s.ransac_px * m_per_px:
        return n2, apart
    return None


class _Try:
    """One try at placing a picture: the picture turned north up with one heading, its features, and
    its best fit with the map (if any)."""

    def __init__(self, rect: Rectified | None, yaw: float) -> None:
        self.rect = rect
        self.yaw = yaw
        self.pts = np.zeros((0, 2), np.float32)
        self.desc = None
        self.pairs = 0
        self.src = self.dst = None
        self.fit: _Fit | None = None
        self.x0 = self.y0 = 0
        self.prior_px = (0.0, 0.0)          # the flight controller's position, in the crop's pixels
        self.why = ""


class Matcher:
    """Matches pictures with the map, one at a time (one thread)."""

    def __init__(self, tile_map, lens: Lens, settings: MatchSettings | None = None) -> None:
        self.map = tile_map
        self.lens = lens
        self.settings = settings or MatchSettings()
        problem = self.settings.problem()
        if problem:
            raise ValueError(problem)
        self.features = Features(self.settings.features, self.settings.max_features)
        self.heading_error_deg = 0.0        # added to the flight controller's heading
        self.height_factor = 1.0            # times the flight controller's height above home
        # The last pictures, for the movement: (moment, turned north up, features, descriptors).
        self._history: deque[tuple[int, Rectified, np.ndarray, np.ndarray | None]] = deque(maxlen=8)

    def _try(self, job: Job, yaw: float, m_per_px: float, timings: dict) -> _Try:
        s = self.settings

        def timed(name: str, since: float) -> None:
            timings[name] = timings.get(name, 0.0) + time.perf_counter() - since

        t = time.perf_counter()
        pose = Pose(job.pose.roll, job.pose.pitch, yaw, job.pose.height_m * self.height_factor)
        attempt = _Try(rectify(job.image, self.lens, pose, m_per_px), yaw)
        timed("rectify", t)
        rect = attempt.rect
        if rect is None:
            attempt.why = "the drone leans too far, or is too high, for a picture of the ground"
            return attempt
        t = time.perf_counter()
        attempt.pts, attempt.desc = self.features.find(rect.image, rect.mask)
        timed("features", t)
        if len(attempt.pts) < s.min_inliers:
            attempt.why = f"the picture shows too little ({len(attempt.pts)} features)"
            return attempt
        # The map around where the drone should be, placed so that the picture's point below the
        # drone falls on the flight controller's position when the shift is 0.
        px, py = self.map.px(job.prior_lat, job.prior_lon)
        margin = job.search_m / m_per_px + 8
        x0, y0 = math.floor(px - rect.ox - margin), math.floor(py - rect.oy - margin)
        w, h = rect.image.shape[1] + int(2 * margin) + 1, rect.image.shape[0] + int(2 * margin) + 1
        t = time.perf_counter()
        crop, cmask = self.map.crop(x0, y0, w, h)
        timed("map", t)
        attempt.x0, attempt.y0 = x0, y0
        # A crop's OpenCV pixel k is map pixel x0 + k, whose middle is x0 + k + 0.5 in map coordinates.
        attempt.prior_px = (px - x0 - 0.5, py - y0 - 0.5)
        if cmask.mean() < 0.5 * 255:
            attempt.why = "outside the map, or the map has no picture there"
            return attempt
        t = time.perf_counter()
        mpts, mdesc = self.features.find(crop, cmask)
        timed("map_features", t)
        t = time.perf_counter()
        pairs = self.features.pair(attempt.desc, mdesc, s.ratio)
        timed("pair", t)
        attempt.pairs = len(pairs)
        if len(pairs) < s.min_inliers:
            attempt.why = f"only {len(pairs)} features pair with the map"
            return attempt
        attempt.src = attempt.pts[[i for i, _ in pairs]]
        attempt.dst = mpts[[j for _, j in pairs]]
        t = time.perf_counter()
        fit = _fit(attempt.src, attempt.dst, s.ransac_px)
        timed("fit", t)
        n = 0 if fit is None else int(fit.inliers.sum())
        if n < s.min_inliers:
            attempt.why = f"only {n} of {len(pairs)} pairs agree on one place"
        else:
            attempt.fit = fit
        return attempt

    def process(self, job: Job) -> Result:
        began = time.perf_counter()
        s = self.settings
        out = Result(job.local_ns, False)
        m_per_px = self.map.m_per_px(job.prior_lat)
        yaw = job.pose.yaw + math.radians(self.heading_error_deg)
        attempt = self._try(job, yaw, m_per_px, out.timings)
        if attempt.fit is not None and s.recheck_turn_deg < abs(attempt.fit.turn_deg) <= s.max_heading_error_deg:
            # Turned more than a little: the heading is off. Again, with this picture's own turn taken out.
            again = self._try(job, yaw + math.radians(attempt.fit.turn_deg), m_per_px, out.timings)
            if again.rect is not None:
                attempt, out.rechecked = again, True
        rect = attempt.rect
        if rect is None:
            self._history.clear()
            out.why = attempt.why
            return self._done(out, began)
        out.movement = self._movement(job.local_ns, rect, attempt.pts, attempt.desc)
        self._history.append((job.local_ns, rect, attempt.pts, attempt.desc))
        out.pairs = attempt.pairs
        fit = attempt.fit
        if fit is None:
            out.why = attempt.why
            return self._done(out, began)
        n = int(fit.inliers.sum())
        out.inliers, out.scale = n, fit.scale
        out.turn_deg = math.degrees(attempt.yaw - job.pose.yaw) + fit.turn_deg
        out.rms_m = fit.rms_px * m_per_px
        if abs(fit.turn_deg) > s.max_turn_deg or abs(out.turn_deg) > s.max_heading_error_deg \
                or not s.min_scale <= fit.scale <= s.max_scale:
            out.why = (f"the place found does not fit the drone's heading and height (turned {out.turn_deg:.1f} "
                       f"degrees, scaled {fit.scale:.2f})")
            return self._done(out, began)
        src, dst = attempt.src, attempt.dst
        agree = src[fit.inliers]
        spread = float(np.sqrt(((agree - agree.mean(axis=0)) ** 2).sum(axis=1).mean()))
        size = math.hypot(*rect.image.shape[:2])
        if spread < s.min_spread * size:
            out.why = f"the {n} agreeing features are bunched in one spot of the picture"
            return self._done(out, began)
        # The point below the drone, on the map.
        nx, ny = _apply(fit.m, rect.ox, rect.oy)
        lat, lon = self.map.latlon(attempt.x0 + nx + 0.5, attempt.y0 + ny + 0.5)
        out.off_prior_m = math.hypot(nx - attempt.prior_px[0], ny - attempt.prior_px[1]) * m_per_px
        if out.off_prior_m > job.search_m:
            out.why = (f"the place found is {out.off_prior_m:.0f} m from where the drone should be "
                       f"(searched {job.search_m:.0f} m)")
            return self._done(out, began)
        # A second place that fits nearly as well: the ground repeats, the position is not sure.
        second = second_place(src, dst, fit, s, (rect.ox, rect.oy), m_per_px)
        if second is not None:
            n2, apart = second
            out.why = f"two places fit ({n} and {n2} features agree, {apart:.0f} m apart)"
            return self._done(out, began)
        # Accuracy (one sigma): the map's own error, the angles' error seen from this height, the
        # fit's (its shift, and its turn and scale carried to the point below the drone), and the
        # drone's tipping within the picture's time error.
        centre = agree.mean(axis=0)
        reach = math.hypot(rect.ox - centre[0], rect.oy - centre[1])
        fit_m = 2.0 * out.rms_m / math.sqrt(n) * math.sqrt(1.0 + 2.0 * (reach / max(spread, 1.0)) ** 2)
        height = job.pose.height_m * self.height_factor
        att_m = height * math.tan(math.radians(s.attitude_error_deg))
        out.motion_m = motion_m(job.pose, job.rates, height)
        out.horiz_acc_m = max(math.sqrt(s.map_error_m ** 2 + att_m ** 2 + fit_m ** 2 + out.motion_m ** 2),
                              s.min_accuracy_m)
        out.lat, out.lon, out.ok = lat, lon, True
        # What this fit says of the flight controller's heading and height, for the next pictures. A
        # second try means a new heading error (a reset): taken at once.
        gain = 1.0 if out.rechecked else s.follow
        self.heading_error_deg = max(-60.0, min(60.0, self.heading_error_deg
                                                + (out.turn_deg - self.heading_error_deg) * gain))
        self.height_factor = max(0.7, min(1.4, self.height_factor * (1.0 + (fit.scale - 1.0) * s.follow)))
        return self._done(out, began)

    def _movement(self, local_ns: int, rect: Rectified, pts: np.ndarray, desc) -> Movement | None:
        """The ground's shift from the oldest picture of up to ``vo_max_gap_s`` before to this one: the
        drone's movement. A newer picture is tried when the oldest does not pair."""
        s = self.settings
        if desc is None:
            return None
        for since_ns, prect, ppts, pdesc in self._history:
            dt = (local_ns - since_ns) / 1e9
            if not 0 < dt <= s.vo_max_gap_s or abs(prect.m_per_px / rect.m_per_px - 1.0) > 1e-3:
                continue
            pairs = self.features.pair(pdesc, desc, s.ratio)
            if len(pairs) < s.vo_min_inliers:
                continue
            fit = _fit(ppts[[i for i, _ in pairs]], pts[[j for _, j in pairs]], 1.5)
            if (fit is None or fit.inliers.sum() < s.vo_min_inliers or abs(fit.turn_deg) > s.vo_max_turn_deg
                    or abs(fit.scale - 1.0) > s.vo_max_scale_error):
                continue
            # The earlier picture's point below the drone, carried into this picture, against this
            # picture's own: the ground moved by the difference, the drone by the opposite.
            x, y = _apply(fit.m, prect.ox, prect.oy)
            return Movement(since_ns, -(rect.oy - y) * rect.m_per_px, (rect.ox - x) * rect.m_per_px,
                            int(fit.inliers.sum()))
        return None

    def forget(self) -> None:
        """The next picture is not compared with the earlier ones (a gap, a restart)."""
        self._history.clear()

    def _done(self, out: Result, began: float) -> Result:
        out.heading_error_deg, out.height_factor = self.heading_error_deg, self.height_factor
        out.seconds = time.perf_counter() - began
        return out
