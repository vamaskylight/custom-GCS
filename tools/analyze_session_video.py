"""Quick field-session video analyzer — frame timestamps + motion heuristics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np


def analyze_video(path: Path, *, sample_every_s: float = 1.0) -> dict:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {path}")

    fps = float(cap.get(cv2.CAP_PROP_FPS) or 25.0)
    frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    duration_s = frames / fps if fps > 0 and frames > 0 else 0.0

    step = max(1, int(round(fps * sample_every_s)))
    prev_gray: np.ndarray | None = None
    motion_samples: list[dict] = []
    scene_changes: list[dict] = []

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
        gray = cv2.GaussianBlur(gray, (9, 9), 0)
        if prev_gray is not None and prev_gray.shape == gray.shape:
            diff = cv2.absdiff(prev_gray, gray)
            mean_diff = float(np.mean(diff))
            motion_samples.append({"t_s": round(t_s, 2), "mean_diff": round(mean_diff, 2)})
            if mean_diff >= 18.0:
                scene_changes.append(
                    {"t_s": round(t_s, 2), "mean_diff": round(mean_diff, 2)}
                )
        prev_gray = gray
        idx += 1

    cap.release()

    # Cluster high-motion windows (likely gimbal slew or UI chaos).
    bursts: list[dict] = []
    if motion_samples:
        active: list[dict] = []
        for s in motion_samples:
            if float(s["mean_diff"]) >= 12.0:
                active.append(s)
            elif active:
                bursts.append(
                    {
                        "start_s": active[0]["t_s"],
                        "end_s": active[-1]["t_s"],
                        "peak_diff": max(float(x["mean_diff"]) for x in active),
                    }
                )
                active = []
        if active:
            bursts.append(
                {
                    "start_s": active[0]["t_s"],
                    "end_s": active[-1]["t_s"],
                    "peak_diff": max(float(x["mean_diff"]) for x in active),
                }
            )

    return {
        "path": str(path),
        "fps": round(fps, 2),
        "frames": frames,
        "size": [w, h],
        "duration_s": round(duration_s, 2),
        "sample_every_s": sample_every_s,
        "motion_bursts": bursts,
        "scene_changes": scene_changes[:20],
        "avg_motion": round(
            float(np.mean([m["mean_diff"] for m in motion_samples])) if motion_samples else 0.0,
            2,
        ),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Analyze VGCS field session video")
    p.add_argument("video", type=Path)
    p.add_argument("--interval", type=float, default=1.0, help="Sample interval seconds")
    args = p.parse_args(argv)
    report = analyze_video(args.video, sample_every_s=float(args.interval))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
