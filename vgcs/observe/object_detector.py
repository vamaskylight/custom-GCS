"""M14 — on-demand object + license-plate-region detection.

Runs cv2.dnn against a bundled ONNX model, on a single captured frame, only
when the operator explicitly requests it (not continuously — unlike the M14
CSRT follow tracker in ``visual_object_tracker.py``, this is click-to-detect,
so the same per-tick performance/crash risk that dominated the tracker's
field history does not apply here in the same way). Still runs in an
isolated worker subprocess anyway, for the same reason as the tracker: cv2's
native code has a documented history of crashing the whole GCS on at least
one client machine, and a detection call is exactly the kind of native-heavy
work that triggered it before (see [[m13-track-state]] memory).

Detection pre/post-processing is ported from the OpenCV Zoo's own reference
implementations (``yolox.py``, ``lpd_yunet.py`` in
https://github.com/opencv/opencv_zoo, Apache-2.0, same models bundled in
``vgcs/assets/models/`` — see MODEL_SOURCE.md there for full provenance),
not reinvented, to minimize the chance of a subtle decode bug.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np

from vgcs.observe.worker_ipc import (
    decode_frame as _decode_frame,
    encode_frame as _encode_frame,
)
from vgcs.observe import worker_ipc
from vgcs.paths import vgcs_assets_dir

# COCO class names, in the exact index order the bundled YOLOX model outputs
# (verbatim from the OpenCV Zoo's own demo.py — see MODEL_SOURCE.md).
COCO_CLASSES: tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus",
    "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
)
# Classes a vehicle-plate check is meaningful for.
_VEHICLE_CLASSES = {"car", "bus", "truck", "motorcycle"}

_YOLOX_MODEL_PATH = vgcs_assets_dir() / "models" / "object_detection_yolox_2022nov.onnx"
_LPD_MODEL_PATH = vgcs_assets_dir() / "models" / "license_plate_detection_lpd_yunet_2023mar.onnx"
_YOLOX_INPUT_SIZE = (640, 640)  # (w, h)
_YOLOX_CONF_THRESHOLD = 0.5
_YOLOX_NMS_THRESHOLD = 0.5
_LPD_INPUT_SIZE = (320, 240)  # (w, h)
_LPD_CONF_THRESHOLD = 0.8
_LPD_NMS_THRESHOLD = 0.3


@dataclass(frozen=True)
class Detection:
    """One detected object, in SOURCE-FRAME pixel space (top-left origin)."""

    label: str
    confidence: float
    x: float
    y: float
    w: float
    h: float
    plate_box: tuple[float, float, float, float] | None = None  # (x, y, w, h) or None


def _letterbox(img: np.ndarray, target_wh: tuple[int, int]) -> tuple[np.ndarray, float]:
    """Resize keeping aspect ratio, pad with 114 to a square — verbatim from
    the OpenCV Zoo YOLOX demo's own ``letterbox()``."""
    import cv2

    target_w, target_h = target_wh
    padded = np.full((target_h, target_w, 3), 114.0, dtype=np.float32)
    ratio = min(target_h / img.shape[0], target_w / img.shape[1])
    new_w, new_h = int(img.shape[1] * ratio), int(img.shape[0] * ratio)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    padded[:new_h, :new_w] = resized
    return padded, ratio


class _YoloXEngine:
    """Generic object detector — thin port of the OpenCV Zoo's ``yolox.py``."""

    def __init__(self, model_path: str) -> None:
        import cv2

        self._net = cv2.dnn.readNet(model_path)
        self._strides = (8, 16, 32)
        self._grids, self._expanded_strides = self._generate_anchors()

    def _generate_anchors(self):
        target_w, target_h = _YOLOX_INPUT_SIZE
        grids, expanded_strides = [], []
        for stride in self._strides:
            hsize, wsize = target_h // stride, target_w // stride
            xv, yv = np.meshgrid(np.arange(hsize), np.arange(wsize))
            grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
            grids.append(grid)
            expanded_strides.append(np.full((*grid.shape[:2], 1), stride))
        return np.concatenate(grids, 1), np.concatenate(expanded_strides, 1)

    def detect(self, frame_rgb: np.ndarray) -> list[tuple[float, float, float, float, float, int]]:
        """Returns (x, y, w, h, score, class_idx) tuples in letterboxed-frame pixel space."""
        import cv2

        blob = np.transpose(frame_rgb, (2, 0, 1))[np.newaxis, :, :, :]
        self._net.setInput(blob)
        outs = self._net.forward(self._net.getUnconnectedOutLayersNames())
        dets = outs[0][0]

        dets[:, :2] = (dets[:, :2] + self._grids) * self._expanded_strides
        dets[:, 2:4] = np.exp(dets[:, 2:4]) * self._expanded_strides

        boxes = dets[:, :4]
        boxes_xywh = np.ones_like(boxes)
        boxes_xywh[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
        boxes_xywh[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
        boxes_xywh[:, 2] = boxes[:, 2]
        boxes_xywh[:, 3] = boxes[:, 3]

        scores = dets[:, 4:5] * dets[:, 5:]
        max_scores = np.amax(scores, axis=1)
        max_scores_idx = np.argmax(scores, axis=1)

        keep = cv2.dnn.NMSBoxesBatched(
            boxes_xywh.tolist(), max_scores.tolist(), max_scores_idx.tolist(),
            _YOLOX_CONF_THRESHOLD, _YOLOX_NMS_THRESHOLD,
        )
        out: list[tuple[float, float, float, float, float, int]] = []
        for i in keep:
            i = int(i)
            x, y, w, h = boxes_xywh[i]
            out.append((float(x), float(y), float(w), float(h), float(max_scores[i]), int(max_scores_idx[i])))
        return out


class _LpdYuNetEngine:
    """License-plate REGION detector (box only, no text) — thin port of the
    OpenCV Zoo's ``lpd_yunet.py``."""

    def __init__(self, model_path: str) -> None:
        import cv2

        self._net = cv2.dnn.readNet(model_path)
        self._min_sizes = ([10, 16, 24], [32, 48], [64, 96], [128, 192, 256])
        self._steps = (8, 16, 32, 64)
        self._variance = (0.1, 0.2)
        self._priors = self._prior_gen(_LPD_INPUT_SIZE)

    def _prior_gen(self, input_wh: tuple[int, int]) -> np.ndarray:
        w, h = input_wh
        fm2 = (int((h + 1) / 2) // 2, int((w + 1) / 2) // 2)
        fm3 = (fm2[0] // 2, fm2[1] // 2)
        fm4 = (fm3[0] // 2, fm3[1] // 2)
        fm5 = (fm4[0] // 2, fm4[1] // 2)
        feature_maps = (fm3, fm4, fm5, (fm5[0] // 2, fm5[1] // 2))
        priors = []
        for k, f in enumerate(feature_maps):
            for i, j in product(range(f[0]), range(f[1])):
                for min_size in self._min_sizes[k]:
                    s_kx, s_ky = min_size / w, min_size / h
                    cx = (j + 0.5) * self._steps[k] / w
                    cy = (i + 0.5) * self._steps[k] / h
                    priors.append([cx, cy, s_kx, s_ky])
        return np.array(priors, dtype=np.float32)

    def detect(self, crop_bgr: np.ndarray) -> tuple[float, float, float, float] | None:
        """``crop_bgr`` must already be resized to ``_LPD_INPUT_SIZE``. Returns the
        best plate box as (x, y, w, h) in ``crop_bgr`` pixel space, or None."""
        import cv2

        blob = cv2.dnn.blobFromImage(crop_bgr)
        self._net.setInput(blob)
        loc, conf, iou = self._net.forward(["loc", "conf", "iou"])

        cls_scores = conf[:, 1]
        iou_scores = np.clip(iou[:, 0], 0.0, 1.0)
        scores = np.sqrt(cls_scores * iou_scores)

        scale = np.array(_LPD_INPUT_SIZE, dtype=np.float32)
        # Four corner points (x0,y0,x1,y1,x2,y2,x3,y3) — take an axis-aligned
        # bounding box over them since this feature only needs a region, not
        # the plate's true (possibly rotated) quadrilateral.
        corners = np.hstack((
            (self._priors[:, 0:2] + loc[:, 4:6] * self._variance[0] * self._priors[:, 2:4]) * scale,
            (self._priors[:, 0:2] + loc[:, 6:8] * self._variance[0] * self._priors[:, 2:4]) * scale,
            (self._priors[:, 0:2] + loc[:, 10:12] * self._variance[0] * self._priors[:, 2:4]) * scale,
            (self._priors[:, 0:2] + loc[:, 12:14] * self._variance[0] * self._priors[:, 2:4]) * scale,
        ))
        # NMS convention (first two corner points as the "box") ported as-is
        # from the OpenCV Zoo reference — an approximation, not a true
        # axis-aligned rect, but it's what the vendor's own tested code uses.
        keep = cv2.dnn.NMSBoxes(
            bboxes=corners[:, 0:4].tolist(),
            scores=scores.tolist(),
            score_threshold=_LPD_CONF_THRESHOLD,
            nms_threshold=_LPD_NMS_THRESHOLD,
        )
        if len(keep) == 0:
            return None
        keep_idx = [int(i) for i in np.array(keep).reshape(-1)]
        best = max(keep_idx, key=lambda i: scores[i])
        pts = corners[best].reshape(4, 2)
        x0, y0 = pts[:, 0].min(), pts[:, 1].min()
        x1, y1 = pts[:, 0].max(), pts[:, 1].max()
        return (float(x0), float(y0), float(x1 - x0), float(y1 - y0))


class _InProcessDetector:
    """Runs the actual cv2.dnn calls in-process. NEVER instantiate directly
    outside a worker process (see ``VisualObjectDetector`` below) — a native
    crash inside any of its methods takes down whatever process it's in."""

    def __init__(self) -> None:
        self._yolox: _YoloXEngine | None = None
        self._lpd: _LpdYuNetEngine | None = None

    def _ensure_loaded(self) -> bool:
        if self._yolox is not None:
            return True
        try:
            if not _YOLOX_MODEL_PATH.exists():
                print(f"[VGCS:m14detect] model file missing: {_YOLOX_MODEL_PATH}")
                return False
            self._yolox = _YoloXEngine(str(_YOLOX_MODEL_PATH))
            if _LPD_MODEL_PATH.exists():
                self._lpd = _LpdYuNetEngine(str(_LPD_MODEL_PATH))
            else:
                print(f"[VGCS:m14detect] plate model missing (plate boxes disabled): {_LPD_MODEL_PATH}")
        except Exception as ex:
            print(f"[VGCS:m14detect] model load failed: {ex!r}")
            self._yolox = None
            self._lpd = None
            return False
        return True

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        import cv2

        if not self._ensure_loaded() or self._yolox is None:
            return []
        try:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            letterboxed, ratio = _letterbox(frame_rgb, _YOLOX_INPUT_SIZE)
            raw = self._yolox.detect(letterboxed)
        except Exception as ex:
            print(f"[VGCS:m14detect] detection failed: {ex!r}")
            return []
        fh, fw = frame_bgr.shape[0], frame_bgr.shape[1]
        out: list[Detection] = []
        has_vehicle = False
        for x, y, w, h, score, cls_idx in raw:
            # Undo the letterbox scale, then clamp to the real frame.
            x, y, w, h = x / ratio, y / ratio, w / ratio, h / ratio
            x = max(0.0, min(float(fw), x))
            y = max(0.0, min(float(fh), y))
            w = max(0.0, min(float(fw) - x, w))
            h = max(0.0, min(float(fh) - y, h))
            label = COCO_CLASSES[cls_idx] if 0 <= cls_idx < len(COCO_CLASSES) else f"class_{cls_idx}"
            has_vehicle = has_vehicle or label in _VEHICLE_CLASSES
            out.append(Detection(label, float(score), x, y, w, h, None))
        if self._lpd is not None and has_vehicle:
            out = self._attach_plate_box(frame_bgr, out)
        return out

    def _attach_plate_box(self, frame_bgr: np.ndarray, dets: list[Detection]) -> list[Detection]:
        """Runs LPD on the FULL frame (its priors/anchors are tuned for a
        whole-scene view, not a cropped sub-image — empirically, feeding it
        a tight vehicle-only crop misses plates that whole-frame detection
        finds), then attaches the result to whichever vehicle box contains
        its center, if any."""
        import cv2

        if self._lpd is None:
            return dets
        try:
            fh, fw = frame_bgr.shape[0], frame_bgr.shape[1]
            resized = cv2.resize(frame_bgr, _LPD_INPUT_SIZE, interpolation=cv2.INTER_LINEAR)
            box = self._lpd.detect(resized)
        except Exception as ex:
            print(f"[VGCS:m14detect] plate-region check failed: {ex!r}")
            return dets
        if box is None:
            return dets
        px, py, pw, ph = box
        sx, sy = fw / _LPD_INPUT_SIZE[0], fh / _LPD_INPUT_SIZE[1]
        plate_box = (float(px * sx), float(py * sy), float(pw * sx), float(ph * sy))
        pcx, pcy = plate_box[0] + plate_box[2] / 2.0, plate_box[1] + plate_box[3] / 2.0
        out: list[Detection] = []
        attached = False
        for det in dets:
            if (
                not attached
                and det.label in _VEHICLE_CLASSES
                and det.x <= pcx <= det.x + det.w
                and det.y <= pcy <= det.y + det.h
            ):
                out.append(
                    Detection(det.label, det.confidence, det.x, det.y, det.w, det.h, plate_box)
                )
                attached = True
            else:
                out.append(det)
        return out


_M14DETECT_WORKER_TIMEOUT_S = 8.0


class VisualObjectDetector:
    """Process-isolated, one-shot detector: spawns a fresh worker process
    (`python -m vgcs.observe.detector_worker_main`) per call via
    ``subprocess.Popen`` — same proven mechanism as ``VisualObjectTracker``
    (see visual_object_tracker.py) — so a native cv2 crash during detection
    can only ever kill a disposable worker, never the GCS itself. On-demand
    only: no persistent process, no per-tick state, unlike the tracker."""

    def detect(self, frame_bgr: np.ndarray, timeout_s: float = _M14DETECT_WORKER_TIMEOUT_S) -> list[Detection]:
        proc = None
        try:
            import subprocess
            import sys

            proc = subprocess.Popen(
                [sys.executable, "-m", "vgcs.observe.detector_worker_main"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,
            )
        except Exception as ex:
            print(f"[VGCS:m14detect] worker process failed to start: {ex!r}")
            return []
        try:
            resp_q = worker_ipc.start_reader(proc)
            send_q = worker_ipc.start_writer(proc)
            resp = worker_ipc.request(proc, resp_q, send_q, ("detect", _encode_frame(frame_bgr)), timeout_s)
            if resp is None:
                if proc.poll() is not None:
                    print("[VGCS:m14detect] worker crashed during detection (contained - GCS unaffected)")
                else:
                    print("[VGCS:m14detect] worker did not respond in time")
                return []
            _, dets = resp
            return list(dets) if dets else []
        finally:
            try:
                send_q.put(("stop",))
                send_q.put(None)
            except Exception:
                pass
            try:
                proc.wait(timeout=1.5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
