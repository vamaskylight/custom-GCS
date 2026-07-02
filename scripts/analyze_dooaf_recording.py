#!/usr/bin/env python3
"""Analyze DOOAF field screen recordings — timeline, motion, overlay stability hints."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np


def _frame_motion(prev_gray: np.ndarray, gray: np.ndarray) -> dict[str, float]:
    flow = cv2.calcOpticalFlowFarneback(
        prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
    )
    mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    return {
        "mean_flow_px": float(np.mean(mag)),
        "p95_flow_px": float(np.percentile(mag, 95)),
        "max_flow_px": float(np.max(mag)),
    }


def _bright_spots(frame_bgr: np.ndarray, *, top_n: int = 8) -> list[dict[str, float]]:
    """Heuristic: saturated / bright UI overlay pixels (crosshairs, labels)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    # bright or highly saturated (common for HUD marks)
    mask = cv2.inRange(hsv, (0, 80, 180), (179, 255, 255))
    ys, xs = np.where(mask > 0)
    if len(xs) < 20:
        return []
    pts = np.column_stack([xs, ys]).astype(np.float32)
    h, w = frame_bgr.shape[:2]
    # grid clusters
    cell = max(24, min(w, h) // 20)
    buckets: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for x, y in zip(xs, ys, strict=True):
        buckets.setdefault((int(x) // cell, int(y) // cell), []).append((int(x), int(y)))
    ranked = sorted(buckets.items(), key=lambda kv: len(kv[1]), reverse=True)[:top_n]
    out: list[dict[str, float]] = []
    for (_cx, _cy), members in ranked:
        mx = float(np.mean([p[0] for p in members]))
        my = float(np.mean([p[1] for p in members]))
        out.append(
            {
                "u": round(mx / w, 4),
                "v": round(my / h, 4),
                "pixels": len(members),
            }
        )
    return out


def analyze_video(path: Path, *, sample_every_s: float = 1.0) -> dict:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"Cannot open video: {path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    duration_s = total / fps if fps > 0 else 0.0

    step = max(1, int(round(fps * sample_every_s)))
    samples: list[dict] = []
    prev_gray: np.ndarray | None = None
    idx = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % step != 0:
            idx += 1
            continue
        t_s = idx / fps
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        entry: dict = {"t_s": round(t_s, 2), "frame": idx}
        if prev_gray is not None:
            entry.update(_frame_motion(prev_gray, gray))
        else:
            entry.update({"mean_flow_px": 0.0, "p95_flow_px": 0.0, "max_flow_px": 0.0})
        entry["overlay_hints"] = _bright_spots(frame)
        samples.append(entry)
        prev_gray = gray
        idx += 1

    cap.release()

    # Motion segments (likely gimbal pan / LRF slew)
    motion_thresh = 1.2  # px mean flow on downscaled-equivalent frame
    pan_segments: list[dict] = []
    in_seg = False
    seg_start = 0.0
    for s in samples:
        moving = float(s.get("mean_flow_px", 0)) >= motion_thresh
        if moving and not in_seg:
            in_seg = True
            seg_start = float(s["t_s"])
        elif not moving and in_seg:
            in_seg = False
            pan_segments.append({"start_s": seg_start, "end_s": float(s["t_s"])})
    if in_seg and samples:
        pan_segments.append({"start_s": seg_start, "end_s": float(samples[-1]["t_s"])})

    # Overlay hint drift between consecutive samples (proxy for mark movement)
    drifts: list[dict] = []
    for a, b in zip(samples, samples[1:], strict=False):
        if not a.get("overlay_hints") or not b.get("overlay_hints"):
            continue
        # match nearest hint by UV
        best = None
        for ha in a["overlay_hints"]:
            for hb in b["overlay_hints"]:
                du = float(hb["u"]) - float(ha["u"])
                dv = float(hb["v"]) - float(ha["v"])
                d = (du * du + dv * dv) ** 0.5
                if best is None or d < best["drift_uv"]:
                    best = {
                        "t0_s": a["t_s"],
                        "t1_s": b["t_s"],
                        "drift_uv": round(d, 4),
                        "from_uv": [ha["u"], ha["v"]],
                        "to_uv": [hb["u"], hb["v"]],
                    }
        if best and best["drift_uv"] > 0.02:
            drifts.append(best)

    return {
        "path": str(path),
        "fps": round(fps, 2),
        "frames": total,
        "width": width,
        "height": height,
        "duration_s": round(duration_s, 1),
        "sample_every_s": sample_every_s,
        "pan_segments": pan_segments,
        "overlay_drifts": drifts[:20],
        "samples": samples,
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("video", type=Path)
    p.add_argument("--sample-every", type=float, default=1.0)
    p.add_argument("--out", type=Path, default=None, help="Write JSON report")
    p.add_argument("--thumb-dir", type=Path, default=None, help="Save thumbs every 30s")
    args = p.parse_args(argv)

    report = analyze_video(args.video, sample_every_s=args.sample_every)

    if args.thumb_dir:
        args.thumb_dir.mkdir(parents=True, exist_ok=True)
        cap = cv2.VideoCapture(str(args.video))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        every = int(30 * fps)
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % every == 0:
                cv2.imwrite(str(args.thumb_dir / f"t_{i // every:03d}.jpg"), frame)
            i += 1
        cap.release()

    summary = {
        k: report[k]
        for k in (
            "path",
            "duration_s",
            "fps",
            "width",
            "height",
            "pan_segments",
            "overlay_drifts",
        )
    }
    print(json.dumps(summary, indent=2))

    if args.out:
        args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Wrote full report: {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
