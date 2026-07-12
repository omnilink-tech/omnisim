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

"""Single-shot renderer — converts a ShotSpec into an mp4 via the capture service.

Splits cleanly so the director can render shots independently (useful
for resuming, re-rendering a single beat after critique, etc.).
"""

from __future__ import annotations

import json
import math
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from . import camera
from .lenses import Lens
from .looks import Look
from .storyboard import ShotSpec
from .subjects import Subject


@dataclass
class ShotResult:
    name: str
    output_path: Path
    duration_s: float
    fov_rad: float
    keyframes: list[dict]
    load_ms: int | None = None


def _post(svc: str, endpoint: str, payload: dict, timeout_s: float = 300.0) -> dict:
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{svc}{endpoint}", data=body,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def render_shot(
    svc_base_url: str,
    subject: Subject,
    spec: ShotSpec,
    look: Look,
    lens: Lens,
    output_dir: Path,
    shot_index: int,
    fps: int = 30,
) -> ShotResult:
    """Render one shot. Assumes the world is already loaded in the capture
    service (the director handles world loading once per storyboard).

    Returns metadata about the result so the director can chain with edit/grade.
    """
    duration_s = spec.resolve_duration_s()
    primitive_name = spec.resolve_shot_primitive()

    keyframes = camera.render_keyframes(
        primitive_name, subject, duration_s, lens.focal_mm, spec.params,
    )

    name = f"s{shot_index:02d}_{spec.beat}_{primitive_name}"
    rel_output = f"{output_dir.name}/{name}.mp4"
    abs_output = output_dir / f"{name}.mp4"
    abs_output.parent.mkdir(parents=True, exist_ok=True)

    # Set viewpoint FoV to match the lens (so the rendered viewport
    # actually has the lens's framing, not OmniSim's default).
    # The capture supervisor's set_camera_pose mirrors the viewpoint
    # orientation; the FoV is a viewpoint field we set separately via the
    # initial /world/load. For mid-storyboard lens changes we'd need a
    # field-set RPC; for now most shots in a storyboard share a focal
    # length range and the world-load FoV is close enough.

    result = _post(svc_base_url, "/capture/sequence", {
        "path_keyframes": keyframes,
        "duration_s": duration_s,
        "fps": int(fps),
        "output": str(abs_output).replace("\\", "/"),
        "codec": "h264",
        "crf": 16,
        "preset": "medium",
        "ease": "smoothstep",
        "warmup_steps": 8,
        "playback_speed": 1.0,
    }, timeout_s=duration_s * 30 + 120)

    return ShotResult(
        name=name,
        output_path=Path(result.get("output", str(abs_output))),
        duration_s=duration_s,
        fov_rad=lens.horizontal_fov_rad,
        keyframes=keyframes,
        load_ms=None,
    )
