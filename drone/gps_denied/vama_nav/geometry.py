"""The down camera's geometry: where each pixel looks on flat ground, a picture turned north up at
the map's scale (``rectify``), and a picture made from a map (``render``, the stand-in camera).

Axes. The drone's angles are ArduPilot's (ATTITUDE): roll right wing down, pitch nose up, yaw
clockwise from north, applied yaw, pitch, roll. The camera is fixed under the drone, looking
straight down, the top of its picture toward the nose (``mount_yaw_deg`` 90: toward the right
side). Ground points are metres north and east of the point straight below the camera, which is
the drone's own position. A picture's pixels count x to the right and y down, as in OpenCV.

A tilted drone does not look at the point below it: at 12 m/s a quadcopter flies about 10 degrees
nose down, and from 500 m its camera's centre is then about 90 m behind it. The drone's angles
at the picture's moment take that out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

# The camera's axes in the drone's (x forward, y right, z down): picture right is the drone's
# right, picture down is its tail, the camera looks down.
CAMERA_IN_BODY = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
MAX_RAY_DEG = 75.0          # a picture corner looking further from straight down: no picture
MAX_OUT_PX = 2400           # the longest side of a picture turned north up


@dataclass(frozen=True)
class Lens:
    """The camera: its field of view across the picture, and how it sits on the drone (degrees).
    The pixels are square, so the height of the field follows from the picture's shape."""

    hfov_deg: float
    mount_yaw_deg: float = 0.0
    mount_pitch_deg: float = 0.0        # the camera looks ahead of straight down
    mount_roll_deg: float = 0.0         # the camera looks right of straight down

    def focal_px(self, width: int) -> float:
        return (width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def matrix(self, width: int, height: int) -> np.ndarray:
        f = self.focal_px(width)
        return np.array([[f, 0.0, (width - 1) / 2.0], [0.0, f, (height - 1) / 2.0], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class Pose:
    """The drone at a picture's moment: angles in radians, height above the ground in metres."""

    roll: float
    pitch: float
    yaw: float
    height_m: float


@dataclass
class Rectified:
    """A picture turned north up at ``m_per_px``: x east, y south. (ox, oy) is the pixel of the
    point straight below the drone."""

    image: np.ndarray
    mask: np.ndarray            # 255 where the picture has ground, 0 outside it
    ox: float
    oy: float
    m_per_px: float


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def body_in_ned(pose: Pose) -> np.ndarray:
    """Turns a direction in the drone's axes (x forward, y right, z down) into north, east, down."""
    return rot_z(pose.yaw) @ rot_y(pose.pitch) @ rot_x(pose.roll)


def rotation(w: np.ndarray) -> np.ndarray:
    """The rotation by the vector ``w`` (its length is the angle in radians)."""
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    skew = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * skew @ skew


def rotation_vector(m: np.ndarray) -> np.ndarray:
    """The vector of a small rotation (radians about x, y, z): the inverse of ``rotation`` near no turn."""
    return 0.5 * np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])


def turn_of(rates: list[tuple], boot_a: float, boot_b: float, max_gap_ms: float) -> np.ndarray | None:
    """The drone's own turn from ``boot_a`` to ``boot_b`` (ms) by its gyros: ATTITUDE's turn rates
    (boot_ms, roll, pitch and yaw rate in rad/s, the body's) summed step by step, interpolated at both
    ends, as a rotation in the drone's axes. None when the samples do not reach both ends or leave a
    gap over ``max_gap_ms`` (summed from too few samples, a tipping drone's turn comes out wrong)."""
    if not rates or rates[0][0] > boot_a or rates[-1][0] < boot_b or boot_b < boot_a:
        return None
    if any(b[0] - a[0] > max_gap_ms for a, b in zip(rates, rates[1:])):
        return None

    def rate_at(t: float) -> np.ndarray:
        for a, b in zip(rates, rates[1:]):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                return np.array(a[1:4]) + f * (np.array(b[1:4]) - np.array(a[1:4]))
        return np.array(rates[-1][1:4])

    points = [(boot_a, rate_at(boot_a))] + [(s[0], np.array(s[1:4])) for s in rates if boot_a < s[0] < boot_b] \
        + [(boot_b, rate_at(boot_b))]
    turn = np.eye(3)
    for (ta, wa), (tb, wb) in zip(points, points[1:]):
        turn = turn @ rotation(0.5 * (wa + wb) * (tb - ta) / 1000.0)
    return turn


def tilt_shift_m(pose_a: Pose, pose_b: Pose, turn: np.ndarray) -> tuple[float, float]:
    """(north, east) metres to add to the ground's shift measured between two pictures turned north up
    with ``pose_a`` and ``pose_b`` (the flight controller's angles), when ``turn`` is the drone's own
    turn in between (its gyros, ``turn_of``).

    An EKF corrects its roll and pitch now and then. A change of its angles that the drone did not
    make shifts the second picture's ground by the height (``pose_b``'s) times that change, and the
    measured shift with it: from 300 m, 0.1 degree is 0.5 m. Without GPS the EKF's corrections follow
    the camera's own positions and speeds, so this error would go round and round."""
    wx, wy, _wz = rotation_vector(body_in_ned(pose_b) @ turn.T @ body_in_ned(pose_a).T)   # what the drone did not turn
    return pose_b.height_m * float(wy), -pose_b.height_m * float(wx)


def ned_from_camera(lens: Lens, pose: Pose) -> np.ndarray:
    """Turns a direction in the camera's axes into north, east, down."""
    body = body_in_ned(pose)
    # A positive turn about the drone's x axis tips the camera to look left (as rolling right does),
    # so the right-looking mount roll goes in with its sign turned.
    mount = rot_z(math.radians(lens.mount_yaw_deg)) @ rot_y(math.radians(lens.mount_pitch_deg)) \
        @ rot_x(-math.radians(lens.mount_roll_deg))
    return body @ mount @ CAMERA_IN_BODY


def image_from_ground(lens: Lens, pose: Pose, width: int, height: int) -> np.ndarray:
    """The homography from ground points (north m, east m, 1) to picture pixels."""
    r = ned_from_camera(lens, pose).T              # camera from north, east, down
    return lens.matrix(width, height) @ np.column_stack([r[:, 0], r[:, 1], pose.height_m * r[:, 2]])


def ground_of(lens: Lens, pose: Pose, width: int, height: int, pixels) -> np.ndarray | None:
    """Ground points (north, east) seen at ``pixels`` [(x, y), ...], or None when one of them looks
    further than MAX_RAY_DEG from straight down (or at the sky)."""
    rays = ned_from_camera(lens, pose) @ np.linalg.inv(lens.matrix(width, height)) \
        @ np.vstack([np.asarray(pixels, float).T, np.ones(len(pixels))])
    down = rays[2] / np.linalg.norm(rays, axis=0)
    if np.any(down < math.cos(math.radians(MAX_RAY_DEG))):
        return None
    t = pose.height_m / rays[2]
    return np.column_stack([rays[0] * t, rays[1] * t])


def footprint(lens: Lens, pose: Pose, width: int, height: int) -> np.ndarray | None:
    """The picture's four corners on the ground (north, east), or None (see ``ground_of``)."""
    return ground_of(lens, pose, width, height, [(0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1)])


def rectify(image: np.ndarray, lens: Lens, pose: Pose, m_per_px: float) -> Rectified | None:
    """The picture turned north up at ``m_per_px`` metres a pixel, around the point below the drone.
    None when the drone leans too far, or the picture would be too large."""
    h, w = image.shape[:2]
    if pose.height_m <= 0 or m_per_px <= 0:
        return None
    # Shrink first by a whole factor while the picture stays finer than the result: the warp alone
    # would skip pixels and alias.
    k = int(m_per_px / (pose.height_m / lens.focal_px(w)))
    if k >= 2:
        image = cv2.resize(image, (w // k, h // k), interpolation=cv2.INTER_AREA)
        h, w = image.shape[:2]
    corners = footprint(lens, pose, w, h)
    if corners is None:
        return None
    n_max, e_max = corners.max(axis=0)
    n_min, e_min = corners.min(axis=0)
    out_w = math.ceil((e_max - e_min) / m_per_px) + 1
    out_h = math.ceil((n_max - n_min) / m_per_px) + 1
    if max(out_w, out_h) > MAX_OUT_PX:
        return None
    ox, oy = -e_min / m_per_px, n_max / m_per_px
    s = m_per_px
    ground_from_out = np.array([[0.0, -s, oy * s], [s, 0.0, -ox * s], [0.0, 0.0, 1.0]])
    m = image_from_ground(lens, pose, w, h) @ ground_from_out
    flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
    out = cv2.warpPerspective(image, m, (out_w, out_h), flags=flags, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), m, (out_w, out_h),
                               flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP, borderValue=0)
    # No features on the picture's edge: also where it meets the result's border (erode pads the
    # border as inside unless told otherwise).
    mask = cv2.erode(mask, np.ones((7, 7), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0)
    return Rectified(out, mask, ox, oy, s)


def render(tile_map, lens: Lens, pose: Pose, lat: float, lon: float, width: int, height: int) -> np.ndarray | None:
    """What the camera sees at (lat, lon) with ``pose`` over flat ground, made from ``tile_map``
    (a ``TileMap``): grey, ``width`` by ``height``. None when it would look too far (see
    ``ground_of``). Ground the map does not have is black."""
    corners = footprint(lens, pose, width, height)
    if corners is None:
        return None
    s = tile_map.m_per_px(lat)
    cx, cy = tile_map.px(lat, lon)
    xs = cx + corners[:, 1] / s
    ys = cy - corners[:, 0] / s
    x0, y0 = math.floor(xs.min()) - 2, math.floor(ys.min()) - 2
    cw, ch = math.ceil(xs.max()) + 3 - x0, math.ceil(ys.max()) + 3 - y0
    if max(cw, ch) > 4 * MAX_OUT_PX:
        return None
    crop, _mask = tile_map.crop(x0, y0, cw, ch)
    # Map pixel k covers k to k + 1, OpenCV's pixel k is centred on k: half a pixel between them.
    crop_from_ground = np.array([[0.0, 1.0 / s, cx - x0 - 0.5], [-1.0 / s, 0.0, cy - y0 - 0.5], [0.0, 0.0, 1.0]])
    m = crop_from_ground @ np.linalg.inv(image_from_ground(lens, pose, width, height))
    return cv2.warpPerspective(crop, m, (width, height), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
