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

"""Subject registry — what the camera is looking at.

A "subject" is a robot in the world that the cinema pipeline locks the
camera onto. Subjects carry the metadata camera primitives need to frame
them well: a characteristic dimension (so a "medium close-up" of Spot
isn't the same as one of the Husky fleet), eye-line height (where to put
`target` for a head-and-shoulders shot), and a "personality" tag that
informs default look + lens.

Use `resolve(svc_base_url, "spot")` to get a Subject with a live world
pose — the supervisor's `/world/subject` endpoint backs the pose lookup
so this works regardless of where the robot was spawned.
"""

from __future__ import annotations

import json
import urllib.request
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class SubjectProfile:
    """Static metadata about a subject — sized for its body, not its world pose."""
    name: str
    aliases: tuple[str, ...]
    char_dim_m: float
    """Characteristic body dimension in meters. Used by camera primitives
    to scale distances (e.g. a tracking shot at `dist=3` char_dims away)."""
    eye_height_m: float
    """Where the camera target should sit for head-and-shoulders framing."""
    floor_height_m: float
    """The floor target (for low-angle hero shots)."""
    personality: str
    """One of: 'hero', 'industrial', 'agile', 'swarm', 'object'. Steers
    default look + lens choice when the storyboard doesn't override."""


PROFILES: dict[str, SubjectProfile] = {
    "spot": SubjectProfile(
        name="spot", aliases=("boston_dynamics_spot", "quadruped"),
        char_dim_m=0.9, eye_height_m=0.55, floor_height_m=0.05,
        personality="hero",
    ),
    "husky": SubjectProfile(
        name="husky", aliases=("clearpath_husky",),
        char_dim_m=1.0, eye_height_m=0.45, floor_height_m=0.1,
        personality="industrial",
    ),
    "jackal": SubjectProfile(
        name="jackal", aliases=("clearpath_jackal",),
        char_dim_m=0.5, eye_height_m=0.25, floor_height_m=0.05,
        personality="agile",
    ),
    "turtlebot3": SubjectProfile(
        name="TurtleBot3Burger", aliases=("turtlebot3", "tb3_burger", "burger"),
        char_dim_m=0.3, eye_height_m=0.2, floor_height_m=0.0,
        personality="agile",
    ),
    "rosbot": SubjectProfile(
        name="rosbot", aliases=("husarion_rosbot",),
        char_dim_m=0.4, eye_height_m=0.2, floor_height_m=0.0,
        personality="agile",
    ),
    "mavic": SubjectProfile(
        name="Mavic_2_PRO", aliases=("drone", "mavic_2_pro", "dji_mavic"),
        char_dim_m=0.35, eye_height_m=2.5, floor_height_m=2.5,
        personality="agile",
    ),
}


def find_profile(name: str) -> Optional[SubjectProfile]:
    """Look up a subject profile by canonical name or alias (case-insensitive)."""
    needle = name.lower()
    for prof in PROFILES.values():
        if prof.name.lower() == needle or needle in (a.lower() for a in prof.aliases):
            return prof
    return None


@dataclass
class Subject:
    """A subject with a live world pose. Camera primitives consume these."""
    profile: SubjectProfile
    position: tuple[float, float, float]
    rotation_axis_angle: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.0)
    """[ax, ay, az, angle] — OmniSim axis-angle. Used to derive subject heading
    so a tracking shot knows where 'in front of' the subject is."""

    @property
    def heading_yaw_rad(self) -> float:
        """Yaw extracted from axis-angle (heading on the XY plane).
        Most ground robots rotate around Z; we treat non-Z rotations as ~0 yaw."""
        ax, ay, az, ang = self.rotation_axis_angle
        if abs(az) < 1e-3 and (abs(ax) > 1e-3 or abs(ay) > 1e-3):
            return 0.0
        return float(ang) * (1.0 if az >= 0 else -1.0)


def resolve(svc_base_url: str, name: str, timeout_s: float = 10.0) -> Subject:
    """Query the capture service for the named robot's world pose, return a
    Subject ready to feed into camera primitives. Raises if the supervisor
    can't find a robot by that name (alias names are resolved here).
    """
    profile = find_profile(name)
    if profile is None:
        raise ValueError(
            f"unknown subject {name!r}; known: {sorted(PROFILES.keys())}"
        )
    # Try the profile's canonical name first, then each alias. Worlds use
    # inconsistent naming (e.g. "spot" vs "Spot" vs "TurtleBot3Burger").
    candidates = [profile.name, *profile.aliases, name]
    last_exc: Exception | None = None
    for candidate in candidates:
        body = json.dumps({"name": candidate}).encode("utf-8")
        req = urllib.request.Request(
            f"{svc_base_url}/world/subject", data=body,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout_s) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except Exception as exc:
            last_exc = exc
            continue
        pos = tuple(float(x) for x in data.get("translation", [0, 0, 0]))
        rot_list = data.get("rotation") or [0.0, 0.0, 1.0, 0.0]
        rot = tuple(float(x) for x in rot_list)
        return Subject(profile=profile, position=pos, rotation_axis_angle=rot)
    raise RuntimeError(
        f"subject {name!r} not found in current world "
        f"(tried {candidates}; last error: {last_exc})"
    )


def list_robots_in_world(svc_base_url: str, timeout_s: float = 10.0) -> list[dict]:
    """Return every Robot/URDFRobot in the loaded world (name, def, translation).
    Useful for the agent when it doesn't know what's available."""
    req = urllib.request.Request(f"{svc_base_url}/world/robots", method="GET")
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    return data.get("robots", [])
