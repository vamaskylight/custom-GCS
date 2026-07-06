"""Analyze mark overlay stability vs scene in a screen recording."""
from __future__ import annotations

import sys

import cv2
import numpy as np

MARKS = {
    "gun": (0.322, 0.811),
    "target": (0.472, 0.495),
    "impact": (0.536, 0.683),
}


def find_marks(frame: np.ndarray, w: int, h: int) -> list[tuple[float, float, float]]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (35, 70, 70), (95, 255, 255))
    for lo, hi in ((80, 70, 70), (100, 255, 255)), ((0, 100, 100), (10, 255, 255)), (
        (160, 100, 100),
        (180, 255, 255),
    ):
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts: list[tuple[float, float, float]] = []
    for c in cnts:
        area = cv2.contourArea(c)
        if 12 < area < 2000:
            m = cv2.moments(c)
            if m["m00"]:
                pts.append((m["m10"] / m["m00"] / w, m["m01"] / m["m00"] / h, area))
    return pts


def analyze(path: str) -> None:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print("FAIL open", path)
        sys.exit(1)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"video: {w}x{h} fps={fps:.1f} frames={n} dur={n/fps:.1f}s")

    for name, (mu, mv) in MARKS.items():
        px, py = int(mu * w), int(mv * h)
        r = 22
        ref_patch = None
        ref_fi = 0
        rows: list[tuple] = []
        for fi in range(0, n, 4):
            cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
            ret, frame = cap.read()
            if not ret:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if ref_patch is None:
                patch = gray[max(0, py - r) : py + r, max(0, px - r) : px + r]
                if patch.shape[0] >= 16:
                    ref_patch = patch.copy()
                    ref_fi = fi
            ncc_f = float("nan")
            best_ncc, best_uv = -1.0, (mu, mv)
            if ref_patch is not None:
                patch = gray[max(0, py - r) : py + r, max(0, px - r) : px + r]
                if patch.shape == ref_patch.shape:
                    ncc_f = float(
                        cv2.matchTemplate(patch, ref_patch, cv2.TM_CCOEFF_NORMED)[0, 0]
                    )
                for dy in range(-70, 71, 5):
                    for dx in range(-70, 71, 5):
                        y1, y2 = max(0, py - r + dy), min(h, py + r + dy)
                        x1, x2 = max(0, px - r + dx), min(w, px + r + dx)
                        cand = gray[y1:y2, x1:x2]
                        if cand.shape != ref_patch.shape:
                            continue
                        sc = float(
                            cv2.matchTemplate(cand, ref_patch, cv2.TM_CCOEFF_NORMED)[0, 0]
                        )
                        if sc > best_ncc:
                            best_ncc, best_uv = sc, ((x1 + x2) / 2 / w, (y1 + y2) / 2 / h)
            scene_d = math_hypot(best_uv[0] - mu, best_uv[1] - mv)
            near = sorted(
                [(u, v) for u, v, _ in find_marks(frame, w, h) if abs(u - mu) < 0.18 and abs(v - mv) < 0.18],
                key=lambda p: (p[0] - mu) ** 2 + (p[1] - mv) ** 2,
            )
            ov = near[0] if near else None
            ov_err = math_hypot(ov[0] - best_uv[0], ov[1] - best_uv[1]) if ov else None
            rows.append((fi, fi / fps, ncc_f, scene_d, best_uv, ov, ov_err))

        print(f"=== {name} pick={MARKS[name]} ref_frame={ref_fi} ===")
        errs = [r[6] for r in rows if r[6] is not None]
        ovs = [r[5] for r in rows if r[5] is not None]
        if ovs:
            du = max(u for u, v in ovs) - min(u for u, v in ovs)
            dv = max(v for u, v in ovs) - min(v for u, v in ovs)
            print(f"  overlay span du={du:.3f} dv={dv:.3f}")
        if errs:
            good = sum(1 for e in errs if e < 0.03)
            print(
                f"  overlay vs scene err mean={np.mean(errs):.3f} max={np.max(errs):.3f} "
                f"within3%={good}/{len(errs)}"
            )
        for fi, t, ncc, sd, buv, ov, oe in rows:
            if sd > 0.025 or (oe is not None and oe > 0.04) or (ncc < 0.5 and t > 5):
                os_ = f"({ov[0]:.3f},{ov[1]:.3f})" if ov else "none"
                oe_s = f"{oe:.3f}" if oe is not None else "n/a"
                print(
                    f"  t={t:5.1f}s sd={sd:.3f} ncc={ncc:5.2f} "
                    f"scene=({buv[0]:.3f},{buv[1]:.3f}) ov={os_} err={oe_s}"
                )
    cap.release()


def math_hypot(x: float, y: float) -> float:
    return float((x * x + y * y) ** 0.5)


if __name__ == "__main__":
    analyze(sys.argv[1] if len(sys.argv) > 1 else "")
