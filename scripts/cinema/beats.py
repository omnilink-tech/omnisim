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

"""Story beats — the structural roles a shot plays in a video.

A beat is a higher-level intent than a camera primitive. "Establish"
maps to a wide static; "hero" maps to a beauty close-up; "resolve"
maps to a pull-out. When a storyboard specifies a beat without an
explicit shot primitive, the director uses these defaults — so an
agent can write `{"beat": "hero"}` and get sensible cinematography
out of the box.

Beats also drive hold-length recommendations (a hero close-up needs
3-4 s to land; an establish can hold 5-6) and color-grade weighting.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Beat:
    """A named story role with default shot primitive + duration hint."""
    name: str
    description: str
    default_shot: str
    """Default camera primitive (see `camera.PRIMITIVES`)."""
    default_duration_s: float
    """Reasonable starting length for this beat; storyboard can override."""
    feels_like: str


BEATS: dict[str, Beat] = {
    "establish": Beat(
        name="establish",
        description="Set the scene. Subject in environment, wide angle, holds a moment.",
        default_shot="establish",
        default_duration_s=5.0,
        feels_like="Where are we? What are we looking at?",
    ),
    "reveal": Beat(
        name="reveal",
        description="Show the subject for the first time after letting environment register.",
        default_shot="reveal_tilt",
        default_duration_s=4.0,
        feels_like="Here is the thing this video is about.",
    ),
    "build": Beat(
        name="build",
        description="Move toward subject — invite the viewer in.",
        default_shot="push_in",
        default_duration_s=5.0,
        feels_like="Getting closer to the action.",
    ),
    "action": Beat(
        name="action",
        description="Subject is doing the thing. Camera tracks alongside.",
        default_shot="tracking_side",
        default_duration_s=6.0,
        feels_like="Watch this work in motion.",
    ),
    "perspective": Beat(
        name="perspective",
        description="Show a different angle on the same action. Orbit or chase.",
        default_shot="orbit",
        default_duration_s=6.0,
        feels_like="And from over here, it looks like this.",
    ),
    "hero": Beat(
        name="hero",
        description="The signature frame. Tight, lit, slight low angle.",
        default_shot="beauty",
        default_duration_s=4.0,
        feels_like="This is what to remember.",
    ),
    "detail": Beat(
        name="detail",
        description="A specific element gets a close-up (gripper, sensor, leg joint).",
        default_shot="push_in",
        default_duration_s=3.0,
        feels_like="And here's the precise mechanism.",
    ),
    "scale": Beat(
        name="scale",
        description="Reveal context — how big the environment is around the subject.",
        default_shot="crane_up",
        default_duration_s=6.0,
        feels_like="And the whole arena looks like this.",
    ),
    "resolve": Beat(
        name="resolve",
        description="Pull back to leave the viewer with a final wide. Closing image.",
        default_shot="pull_out",
        default_duration_s=6.0,
        feels_like="And we leave them here.",
    ),
}


def get(name: str) -> Beat:
    needle = name.lower().strip()
    if needle not in BEATS:
        raise ValueError(f"unknown beat {name!r}; known: {sorted(BEATS.keys())}")
    return BEATS[needle]
