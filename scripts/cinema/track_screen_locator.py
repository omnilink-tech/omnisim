#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Add a restrained screen-space locator to fixed-camera simulator footage.

The source remains the authority: this utility follows the only moving visual
subject in a locked shot.  It does not synthesize motion or alter simulation
state.  It is intended for very wide evidence shots where a real robot is too
small to read at delivery size.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def _window_sum(values: np.ndarray, radius: int) -> np.ndarray:
    padded = np.pad(values, ((radius, radius), (radius, radius)), mode="constant")
    integral = np.pad(padded, ((1, 0), (1, 0)), mode="constant").cumsum(0).cumsum(1)
    size = radius * 2 + 1
    return (
        integral[size:, size:]
        - integral[:-size, size:]
        - integral[size:, :-size]
        + integral[:-size, :-size]
    )


def _candidate_peaks(score: np.ndarray, count: int = 24, exclusion: int = 8) -> list[tuple[float, int, int]]:
    work = score.copy()
    peaks: list[tuple[float, int, int]] = []
    for _ in range(count):
        flat = int(np.argmax(work))
        value = float(work.flat[flat])
        if value <= 0:
            break
        y, x = np.unravel_index(flat, work.shape)
        peaks.append((value, int(x), int(y)))
        y0, y1 = max(0, y - exclusion), min(work.shape[0], y + exclusion + 1)
        x0, x1 = max(0, x - exclusion), min(work.shape[1], x + exclusion + 1)
        work[y0:y1, x0:x1] = 0
    return peaks


def _track(frames: list[Path], scale: int) -> tuple[list[tuple[float, float, float]], tuple[int, int]]:
    with Image.open(frames[0]) as first:
        full_size = first.size
    small_size = (max(1, full_size[0] // scale), max(1, full_size[1] // scale))

    sample_indices = np.linspace(0, len(frames) - 1, min(41, len(frames)), dtype=int)
    samples = []
    for index in sample_indices:
        with Image.open(frames[int(index)]) as image:
            samples.append(np.asarray(image.convert("RGB").resize(small_size, Image.Resampling.BILINEAR), dtype=np.float32))
    background = np.median(np.stack(samples, axis=0), axis=0)

    observations: list[list[tuple[float, int, int]]] = []
    for frame in frames:
        with Image.open(frame) as image:
            rgb = np.asarray(image.convert("RGB").resize(small_size, Image.Resampling.BILINEAR), dtype=np.float32)
        difference = np.mean(np.abs(rgb - background), axis=2)
        warmth = np.clip(rgb[:, :, 0] - rgb[:, :, 2] - 3.0, 0.0, 48.0) / 48.0
        brightness = np.mean(rgb, axis=2) / 255.0
        # Motion is primary. Warm, bright pixels favour the Husky's cream body
        # over the cool maze walls without inventing a target position.
        pixel_score = np.maximum(difference - 1.4, 0.0) * (0.65 + 1.35 * warmth) * (0.45 + brightness)
        margin_x = max(3, small_size[0] // 45)
        margin_y = max(3, small_size[1] // 45)
        pixel_score[:margin_y] = 0
        pixel_score[-margin_y:] = 0
        pixel_score[:, :margin_x] = 0
        pixel_score[:, -margin_x:] = 0
        observations.append(_candidate_peaks(_window_sum(pixel_score, radius=3)))

    # Dynamic programming chooses a temporally coherent peak sequence. The
    # transition cost permits real accelerated movement but rejects unrelated
    # compression shimmer elsewhere in the static shot.
    costs: list[np.ndarray] = []
    parents: list[np.ndarray] = []
    first_peaks = observations[0]
    if not first_peaks:
        raise RuntimeError("no moving subject candidates found")
    first_values = np.array([p[0] for p in first_peaks], dtype=np.float64)
    costs.append(-first_values / max(1.0, float(first_values.max())))
    parents.append(np.full(len(first_peaks), -1, dtype=np.int32))

    for frame_index in range(1, len(observations)):
        current = observations[frame_index]
        previous = observations[frame_index - 1]
        if not current:
            current = [(0.0, previous[0][1], previous[0][2])]
            observations[frame_index] = current
        peak_values = np.array([p[0] for p in current], dtype=np.float64)
        peak_values /= max(1.0, float(peak_values.max()))
        frame_cost = np.empty(len(current), dtype=np.float64)
        frame_parent = np.empty(len(current), dtype=np.int32)
        for j, (_, x, y) in enumerate(current):
            transition = []
            for i, (_, px, py) in enumerate(previous):
                distance = math.hypot(x - px, y - py)
                transition.append(costs[-1][i] + 0.075 * min(distance, 40.0))
            best = int(np.argmin(transition))
            frame_cost[j] = transition[best] - peak_values[j]
            frame_parent[j] = best
        costs.append(frame_cost)
        parents.append(frame_parent)

    selected = [0] * len(observations)
    selected[-1] = int(np.argmin(costs[-1]))
    for frame_index in range(len(observations) - 1, 0, -1):
        selected[frame_index - 1] = int(parents[frame_index][selected[frame_index]])

    raw = []
    for frame_index, candidate_index in enumerate(selected):
        value, x, y = observations[frame_index][candidate_index]
        raw.append((float(x * scale), float(y * scale), float(value)))

    # A short symmetric smoothing window removes sub-pixel detector jitter.
    tracked = []
    for index in range(len(raw)):
        window = raw[max(0, index - 2): min(len(raw), index + 3)]
        tracked.append((
            float(np.median([p[0] for p in window])),
            float(np.median([p[1] for p in window])),
            raw[index][2],
        ))
    return tracked, full_size


def _draw_locator(frame: Path, x: float, y: float, radius: int) -> Image.Image:
    with Image.open(frame) as image:
        canvas = image.convert("RGB")
    draw = ImageDraw.Draw(canvas, "RGBA")
    box = (x - radius, y - radius, x + radius, y + radius)
    shadow = (x - radius - 2, y - radius - 2, x + radius + 2, y + radius + 2)
    draw.ellipse(shadow, outline=(0, 0, 0, 175), width=7)
    draw.ellipse(box, outline=(34, 220, 255, 245), width=4)
    gap = max(5, radius // 4)
    draw.arc(box, 38, 142, fill=(220, 252, 255, 255), width=5)
    draw.arc(box, 218, 322, fill=(220, 252, 255, 255), width=5)
    draw.ellipse((x - gap / 2, y - gap / 2, x + gap / 2, y + gap / 2), fill=(34, 220, 255, 220))
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output-frames", type=Path)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--scale", type=int, default=4)
    parser.add_argument("--radius", type=int, default=24)
    args = parser.parse_args()

    frames = sorted(args.frames.glob("frame_*.png"))
    if not frames:
        raise SystemExit(f"no frames in {args.frames}")
    tracked, full_size = _track(frames, args.scale)
    max_score = max(point[2] for point in tracked) or 1.0
    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen([
        args.ffmpeg, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-video_size", f"{full_size[0]}x{full_size[1]}", "-framerate", str(args.fps),
        "-i", "-",
        "-c:v", "libx264", "-preset", "slow", "-crf", "12",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output_video),
    ], stdin=subprocess.PIPE)
    assert encoder.stdin is not None
    try:
        if args.output_frames is not None:
            args.output_frames.mkdir(parents=True, exist_ok=True)
        for index, (frame, point) in enumerate(zip(frames, tracked), start=1):
            canvas = _draw_locator(frame, point[0], point[1], args.radius)
            if args.output_frames is not None:
                canvas.save(args.output_frames / f"frame_{index:04d}.png")
            encoder.stdin.write(canvas.tobytes())
    finally:
        encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError(f"ffmpeg failed with exit code {encoder.returncode}")
    receipt = {
        "source_frames": str(args.frames),
        "output_video": str(args.output_video),
        "frame_count": len(frames),
        "frame_size": list(full_size),
        "fps": args.fps,
        "locator": {
            "kind": "screen_space_motion_tracker",
            "simulation_effect": "none",
            "radius_px": args.radius,
            "positions": [
                {"frame": i + 1, "x": round(p[0], 3), "y": round(p[1], 3), "confidence": round(p[2] / max_score, 6)}
                for i, p in enumerate(tracked)
            ],
        },
    }
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
