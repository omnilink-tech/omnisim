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

"""Vision-based shot critique — the agentic feedback loop.

For each rendered shot, extract a representative frame and ask a vision
model: is the subject visible? composition? exposure? motion blur?
The critique returns a structured verdict + suggested camera tweaks
the director can apply on a re-shoot.

Provider: Anthropic Claude (uses ANTHROPIC_API_KEY). Skipped silently
when the key is missing — the storyboard still produces output, just
without the self-correction loop. Per the user's local network memory,
TLS interception is enabled, so we inject `truststore` before any HTTPS.

Critique is *optional*. If you skip it, the director ships whatever
the first pass produced.
"""

from __future__ import annotations

import base64
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


CRITIQUE_PROMPT = """You are an experienced cinematographer reviewing a frame
from an in-progress robotics simulation video.

The frame is from a {shot_kind} shot of a {subject} robot. The cinematic
intent is "{beat_intent}".

Critique this frame on 1-5 scales (5 = excellent):
- subject_visibility: is the subject clearly visible and the focal point?
- composition: rule of thirds, headroom, leading lines, balance
- exposure: not too dark, not blown out, dynamic range used
- framing_intent: does the frame deliver on the cinematic intent of a {beat_intent} beat?

Then suggest ONE concrete camera adjustment if any score is below 4:
- adjust: "move_in" | "move_out" | "raise" | "lower" | "rotate_left" |
  "rotate_right" | "none"
- magnitude: small | medium | large

Reply ONLY as JSON, no commentary, no markdown fence:
{{"subject_visibility": int, "composition": int, "exposure": int,
  "framing_intent": int, "adjust": str, "magnitude": str,
  "reasoning": "one sentence"}}
"""


@dataclass
class Critique:
    shot_name: str
    scores: dict
    adjust: str = "none"
    magnitude: str = "none"
    reasoning: str = ""
    raw_response: str = ""
    skipped: bool = False
    error: str | None = None

    @property
    def overall(self) -> float:
        if not self.scores:
            return 0.0
        return sum(self.scores.values()) / len(self.scores)

    @property
    def needs_reshoot(self) -> bool:
        # Any score below 3 OR overall below 3.5 = reshoot worth trying.
        if not self.scores:
            return False
        return any(v < 3 for v in self.scores.values()) or self.overall < 3.5


def _ffmpeg() -> str:
    p = shutil.which("ffmpeg")
    if p:
        return p
    fallback = "C:/ffmpeg/bin/ffmpeg.exe"
    if Path(fallback).exists():
        return fallback
    raise RuntimeError("ffmpeg not found")


def extract_preview_frame(
    video_path: Path, out_path: Path, at_fraction: float = 0.5,
) -> Path:
    """Pull a single PNG frame from `at_fraction` (0..1) of the clip's duration.
    Default 0.5 = midpoint, where composition is usually most representative."""
    # ffprobe duration
    probe = subprocess.run(
        [_ffmpeg().replace("ffmpeg", "ffprobe"), "-v", "error",
         "-show_entries", "format=duration", "-of", "default=nw=1:nk=1",
         str(video_path)],
        capture_output=True, text=True, timeout=30,
    )
    try:
        duration = float(probe.stdout.strip())
    except ValueError:
        duration = 5.0
    t = max(0.1, min(duration - 0.1, duration * at_fraction))
    cmd = [
        _ffmpeg(), "-y", "-ss", f"{t:.2f}", "-i", str(video_path),
        "-frames:v", "1", str(out_path),
    ]
    subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=True)
    return out_path


def critique_shot(
    video_path: Path, shot_name: str, beat_intent: str, subject_name: str,
    shot_kind: str,
) -> Critique:
    """Run a single shot through vision critique. Returns Critique(skipped=True)
    if no API key — caller can branch on .skipped."""
    api_key = os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("OMNI_KEY")
    if not api_key:
        return Critique(shot_name=shot_name, scores={}, skipped=True,
                        error="no ANTHROPIC_API_KEY in env")

    # AVG/corporate TLS interception workaround per the user's memory.
    try:
        import truststore  # type: ignore
        truststore.inject_into_ssl()
    except ImportError:
        pass

    try:
        from anthropic import Anthropic  # type: ignore
    except ImportError:
        return Critique(shot_name=shot_name, scores={}, skipped=True,
                        error="anthropic SDK not installed; pip install anthropic")

    # Extract a preview frame for the model.
    preview = video_path.with_suffix(".preview.png")
    try:
        extract_preview_frame(video_path, preview)
    except Exception as exc:
        return Critique(shot_name=shot_name, scores={}, error=f"frame extract: {exc}")

    with open(preview, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")

    client = Anthropic(api_key=api_key)
    prompt = CRITIQUE_PROMPT.format(
        shot_kind=shot_kind, subject=subject_name, beat_intent=beat_intent,
    )
    try:
        resp = client.messages.create(
            model="claude-haiku-4-5-20251001",
            max_tokens=400,
            messages=[{
                "role": "user",
                "content": [
                    {"type": "image", "source": {
                        "type": "base64", "media_type": "image/png", "data": b64,
                    }},
                    {"type": "text", "text": prompt},
                ],
            }],
        )
    except Exception as exc:
        return Critique(shot_name=shot_name, scores={}, error=f"API call: {exc}")

    text = resp.content[0].text if resp.content else ""
    try:
        # Strip possible markdown fences just in case.
        cleaned = text.strip()
        if cleaned.startswith("```"):
            cleaned = cleaned.strip("`").strip()
            if cleaned.startswith("json"):
                cleaned = cleaned[4:].strip()
        data = json.loads(cleaned)
    except json.JSONDecodeError:
        return Critique(shot_name=shot_name, scores={}, raw_response=text,
                        error="model returned non-JSON")

    scores = {
        k: int(data.get(k, 3))
        for k in ("subject_visibility", "composition", "exposure", "framing_intent")
    }
    return Critique(
        shot_name=shot_name, scores=scores,
        adjust=str(data.get("adjust", "none")),
        magnitude=str(data.get("magnitude", "none")),
        reasoning=str(data.get("reasoning", "")),
        raw_response=text,
    )


def adjustment_to_param_delta(adjust: str, magnitude: str) -> dict:
    """Translate a critique's adjustment into camera-primitive parameter
    deltas. Used by the director when retrying a flagged shot."""
    mag = {"small": 0.5, "medium": 1.0, "large": 2.0}.get(magnitude, 1.0)
    delta: dict = {}
    if adjust == "move_in":
        delta["distance_chardim_mult"] = 1.0 - 0.15 * mag
    elif adjust == "move_out":
        delta["distance_chardim_mult"] = 1.0 + 0.25 * mag
    elif adjust == "raise":
        delta["elevation_deg_offset"] = +6.0 * mag
    elif adjust == "lower":
        delta["elevation_deg_offset"] = -6.0 * mag
    elif adjust == "rotate_left":
        delta["azimuth_deg_offset"] = -15.0 * mag
    elif adjust == "rotate_right":
        delta["azimuth_deg_offset"] = +15.0 * mag
    return delta
