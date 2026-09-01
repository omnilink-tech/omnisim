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

"""Deterministic proxy critique for Agent Build films.

This is deliberately local and reproducible.  It catches the expensive classes
of editorial failure before the 1080p encode: inert hero footage, blank or
clipped frames, disorienting full-frame motion in a declared wide, missing
frames, unsafe overlay combinations, and damaged cut boundaries.  It also
emits overview and exact cut-pair sheets for the semantic human/agent review
that pixels alone cannot replace.
"""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from .agent_build import (
    INK,
    MUTED,
    AgentBuildSpec,
    PROXY_PROFILE,
    _ffmpeg,
    _font,
    probe_media,
)


SAMPLE_W, SAMPLE_H = 320, 180
# A bird's-eye robot can occupy well under one percent of the frame.  The
# benchmark's verified moving locator scores 0.00056 while a held outcome frame
# scores 0.00024, so the threshold sits between those measured cases.
MIN_MOTION_SCORE = 0.00035
MAX_WIDE_CHANGED_FRACTION = 0.72
MIN_FRAME_STD = 7.0
MIN_SHARPNESS = 2.2


@dataclass(frozen=True)
class FrameMetrics:
    mean_luma: float
    std_luma: float
    sharpness: float


def _frame_metrics(frame: np.ndarray) -> FrameMetrics:
    rgb = frame.astype(np.float32)
    gray = 0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1] + 0.0722 * rgb[:, :, 2]
    gx = np.abs(np.diff(gray, axis=1)).mean()
    gy = np.abs(np.diff(gray, axis=0)).mean()
    return FrameMetrics(float(gray.mean()), float(gray.std()), float((gx + gy) / 2.0))


def _sample_indices(frame_count: int) -> list[int]:
    last = max(0, frame_count - 1)
    return sorted(set((0, round(last * 0.25), round(last * 0.5),
                       round(last * 0.75), last)))


def _sample_frames(path: Path) -> list[np.ndarray]:
    facts = probe_media(path)
    frame_count = int(facts["frames"] or round(facts["duration_s"] * facts["fps"]))
    indices = _sample_indices(max(1, frame_count))
    expression = "+".join(f"eq(n\\,{index})" for index in indices)
    process = subprocess.run([
        _ffmpeg(), "-v", "error", "-i", str(path),
        "-vf", f"select='{expression}',scale={SAMPLE_W}:{SAMPLE_H}:flags=bilinear",
        "-vsync", "0", "-frames:v", str(len(indices)),
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-",
    ], capture_output=True)
    if process.returncode:
        raise RuntimeError(f"could not sample {path}: {process.stderr.decode(errors='replace')[-1000:]}")
    stride = SAMPLE_W * SAMPLE_H * 3
    if len(process.stdout) < stride:
        raise RuntimeError(f"no readable frames in {path}")
    return [
        np.frombuffer(process.stdout[offset:offset + stride], dtype=np.uint8)
        .reshape((SAMPLE_H, SAMPLE_W, 3)).copy()
        for offset in range(0, len(process.stdout) - stride + 1, stride)
    ]


def _motion(frames: list[np.ndarray]) -> tuple[float, float]:
    if len(frames) < 2:
        return 0.0, 0.0
    scores: list[float] = []
    fractions: list[float] = []
    for left, right in zip(frames, frames[1:]):
        diff = np.abs(right.astype(np.int16) - left.astype(np.int16)).mean(axis=2)
        scores.append(float(diff.mean() / 255.0))
        fractions.append(float(np.mean(diff > 8.0)))
    return float(np.mean(scores)), float(np.mean(fractions))


def _labelled_frame(frame: np.ndarray, label: str, sublabel: str = "") -> Image.Image:
    image = Image.fromarray(frame).convert("RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rectangle((0, SAMPLE_H - 42, SAMPLE_W, SAMPLE_H), fill=(5, 8, 12, 220))
    draw.text((10, SAMPLE_H - 35), label, font=_font(12, "semi"), fill=INK)
    if sublabel:
        draw.text((10, SAMPLE_H - 18), sublabel, font=_font(9), fill=MUTED)
    return image


def _sheet(items: list[tuple[np.ndarray, str, str]], output: Path,
           columns: int = 4) -> Path:
    rows = max(1, (len(items) + columns - 1) // columns)
    canvas = Image.new("RGB", (columns * SAMPLE_W, rows * SAMPLE_H), (8, 11, 15))
    for index, (frame, label, sublabel) in enumerate(items):
        canvas.paste(_labelled_frame(frame, label, sublabel),
                     ((index % columns) * SAMPLE_W, (index // columns) * SAMPLE_H))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=94)
    return output


def review_proxy(spec: AgentBuildSpec, base_out: Path | None = None) -> dict[str, Any]:
    """Review the exact proxy edit and write a fail-closed approval receipt."""
    started = time.perf_counter()
    base = (base_out or spec.source_path.parent / "build" / spec.slug).resolve()
    proxy_dir = base / PROXY_PROFILE.name
    parts = proxy_dir / "parts"
    master = proxy_dir / f"{spec.slug}_proxy.mp4"
    if not master.exists():
        raise FileNotFoundError(f"proxy master missing; render it first: {master}")

    blockers: list[str] = []
    warnings: list[str] = []
    shots: list[dict[str, Any]] = []
    sampled: list[tuple[str, list[np.ndarray]]] = []
    required_motion = {spec.segments[0].id, spec.climax_segment}
    required_motion.update(
        segment_id for segment_id in spec.wide_reference_segments
        if next((segment for segment in spec.segments
                 if segment.id == segment_id and segment.act == 3), None)
    )
    master_facts = probe_media(master)
    if ((master_facts["width"], master_facts["height"])
            != (PROXY_PROFILE.width, PROXY_PROFILE.height)):
        blockers.append("proxy delivery geometry drifted from 1280x720")
    if abs(master_facts["fps"] - PROXY_PROFILE.fps) > 0.01:
        blockers.append("proxy delivery rate drifted from CFR 15 fps")
    expected_master_frames = round(spec.duration_s * PROXY_PROFILE.fps)
    if master_facts["frames"] and master_facts["frames"] != expected_master_frames:
        blockers.append(
            f"proxy master has {master_facts['frames']} frames, expected {expected_master_frames}"
        )

    for index, segment in enumerate(spec.segments, 1):
        part = parts / f"{index:03d}_{segment.id}.mp4"
        if not part.exists():
            blockers.append(f"{segment.id}: proxy part is missing")
            continue
        facts = probe_media(part)
        expected_frames = round(segment.duration_s * PROXY_PROFILE.fps)
        if facts["frames"] and int(facts["frames"]) != expected_frames:
            blockers.append(
                f"{segment.id}: {facts['frames']} frames, expected {expected_frames}"
            )
        try:
            frames = _sample_frames(part)
        except RuntimeError as exc:
            blockers.append(f"{segment.id}: {exc}")
            continue
        sampled.append((segment.id, frames))
        per_frame = [_frame_metrics(frame) for frame in frames]
        motion_score, changed_fraction = _motion(frames)
        minimum_std = min(item.std_luma for item in per_frame)
        minimum_sharpness = min(item.sharpness for item in per_frame)
        means = [item.mean_luma for item in per_frame]
        if min(means) < 8.0 or max(means) > 248.0 or minimum_std < MIN_FRAME_STD:
            blockers.append(f"{segment.id}: blank or clipped representative frame")
        if minimum_sharpness < MIN_SHARPNESS:
            warnings.append(f"{segment.id}: one sampled frame may be soft")
        if segment.id in required_motion and motion_score < MIN_MOTION_SCORE:
            blockers.append(
                f"{segment.id}: declared story-critical footage appears inert "
                f"(motion {motion_score:.5f})"
            )
        if (segment.coverage == "wide" and changed_fraction > MAX_WIDE_CHANGED_FRACTION
                and motion_score > 0.035):
            blockers.append(
                f"{segment.id}: declared wide has excessive full-frame motion; "
                "use a locked or deliberately restrained camera"
            )
        if (segment.kind == "clip" and changed_fraction > 0.85
                and motion_score > 0.12):
            blockers.append(
                f"{segment.id}: most of the frame changes at once; the camera move is "
                "likely disorienting rather than explanatory"
            )
        if segment.overlay and segment.claim_boundary:
            blockers.append(f"{segment.id}: overlay and claim boundary compete in one frame")
        shots.append({
            "segment": segment.id, "kind": segment.kind, "coverage": segment.coverage,
            "frames": facts["frames"], "motion_score": round(motion_score, 6),
            "changed_fraction": round(changed_fraction, 6),
            "minimum_frame_std": round(minimum_std, 3),
            "minimum_sharpness": round(minimum_sharpness, 3),
            "overlay_safe": not (segment.overlay and segment.claim_boundary),
        })

    overview_items = [
        (frames[len(frames) // 2], segment_id, "MIDPOINT")
        for segment_id, frames in sampled
    ]
    overview = _sheet(overview_items, proxy_dir / "proxy_overview.jpg")
    cut_items: list[tuple[np.ndarray, str, str]] = []
    for (left_id, left), (right_id, right) in zip(sampled, sampled[1:]):
        cut_items.extend(((left[-1], left_id, "CUT OUT"),
                          (right[0], right_id, "CUT IN")))
        left_metrics = _frame_metrics(left[-1])
        right_metrics = _frame_metrics(right[0])
        if (left_metrics.std_luma < MIN_FRAME_STD
                or right_metrics.std_luma < MIN_FRAME_STD):
            blockers.append(f"cut {left_id} -> {right_id}: blank boundary frame")
    cut_sheet = _sheet(cut_items, proxy_dir / "proxy_cut_pairs.jpg", columns=4)

    if sampled:
        last_id, last_frames = sampled[-1]
        final_metrics = _frame_metrics(last_frames[-1])
        if final_metrics.std_luma < MIN_FRAME_STD or not 10.0 < final_metrics.mean_luma < 245.0:
            blockers.append(f"{last_id}: last build frame does not preserve a readable outcome")
    else:
        blockers.append("proxy contains no reviewable story footage")

    # Pixel analysis proves integrity and motion, not story semantics.  The
    # generated exact cut sheets make the remaining monitored review fast.
    warnings.append(
        "semantic outcome/claim correctness still requires the independent monitored review"
    )
    report = {
        "version": 1, "style": "agent_build_proxy_review_v1",
        "approved": not blockers, "master": str(master),
        "master_facts": master_facts,
        "elapsed_s": round(time.perf_counter() - started, 3),
        "thresholds": {
            "min_motion_score": MIN_MOTION_SCORE,
            "max_wide_changed_fraction": MAX_WIDE_CHANGED_FRACTION,
            "min_frame_std": MIN_FRAME_STD, "min_sharpness": MIN_SHARPNESS,
        },
        "blockers": blockers, "warnings": warnings, "shots": shots,
        "overview_sheet": str(overview), "cut_pair_sheet": str(cut_sheet),
        "outcome_frame_present": not any("last build frame" in item for item in blockers),
    }
    receipt = proxy_dir / "proxy_review.json"
    receipt.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if blockers:
        raise RuntimeError("proxy review failed:\n- " + "\n- ".join(blockers))
    return report
