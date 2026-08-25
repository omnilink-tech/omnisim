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

"""Robot-independent motion intent representation.

MotionIR intentionally stops above joint space.  A walk is expressed in base motion,
contact, payload, and constraint semantics; a :class:`MotionBinding` then points to a
robot-specific ghost/checkpoint/compiler.  This avoids pretending that copying G1
joint columns onto an H1 or quadruped is retargeting.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


_DOMAINS = {"locomotion", "balance", "manipulation", "transport", "terrain"}
_TIME = {"cyclic", "finite", "static"}
_AXES = {"forward", "lateral", "vertical", "yaw", "pitch", "roll"}


@dataclass(frozen=True)
class MotionIR:
    name: str
    domain: str
    time_model: str
    control_axes: tuple[str, ...]
    contact_model: str
    payload: str = "none"
    constraints: tuple[str, ...] = ()
    parameters: dict[str, Any] = field(default_factory=dict)
    schema: int = 1

    def validate(self) -> list[str]:
        issues: list[str] = []
        if self.schema != 1:
            issues.append(f"motion IR schema must be 1, got {self.schema!r}")
        if not self.name:
            issues.append("motion IR name is required")
        if self.domain not in _DOMAINS:
            issues.append(f"unknown motion domain {self.domain!r}")
        if self.time_model not in _TIME:
            issues.append(f"unknown time model {self.time_model!r}")
        unknown = sorted(set(self.control_axes) - _AXES)
        if unknown:
            issues.append(f"unknown task-space control axes: {unknown}")
        if not self.contact_model:
            issues.append("contact model is required")
        return issues

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["control_axes"] = list(self.control_axes)
        data["constraints"] = list(self.constraints)
        return data

    @classmethod
    def from_skill(cls, skill: dict[str, Any]) -> "MotionIR":
        """Adapt a v2 skill manifest to task-space intent without reading anatomy."""
        explicit = skill.get("motion_ir")
        if isinstance(explicit, dict):
            return cls(
                name=str(explicit.get("name", skill.get("name", ""))),
                domain=str(explicit.get("domain", "locomotion")),
                time_model=str(explicit.get("time_model", "finite")),
                control_axes=tuple(explicit.get("control_axes", ())),
                contact_model=str(explicit.get("contact_model", "declared-by-binding")),
                payload=str(explicit.get("payload", "none")),
                constraints=tuple(explicit.get("constraints", ())),
                parameters=dict(explicit.get("parameters", {})),
                schema=int(explicit.get("schema", 1)),
            )
        name = str(skill.get("name", ""))
        text = f"{name} {skill.get('title', '')}".lower()
        motion_class = str(skill.get("motion_class", "sequence"))
        time_model = {"cyclic": "cyclic", "static": "static"}.get(motion_class, "finite")
        if "carry" in text:
            domain, axes, payload = "transport", ("forward", "lateral", "yaw"), "held-object"
        elif "turn" in text:
            domain, axes, payload = "locomotion", ("yaw",), "optional"
        elif "walk" in text:
            domain, axes, payload = "locomotion", ("forward", "lateral", "yaw"), "optional"
        elif "climb" in text or "stair" in text:
            domain, axes, payload = "terrain", ("forward", "vertical", "pitch"), "none"
        elif "arm" in text:
            domain, axes, payload = "manipulation", (), "optional"
        else:
            domain, axes, payload = "balance", ("roll", "pitch", "yaw"), "none"
        return cls(
            name=name,
            domain=domain,
            time_model=time_model,
            control_axes=axes,
            contact_model="periodic-support" if time_model == "cyclic" else "binding-declared",
            payload=payload,
            constraints=("support-safe-handover",) if domain in {"locomotion", "transport"} else (),
        )

@dataclass(frozen=True)
class MotionBinding:
    """A robot-specific implementation of a MotionIR contract."""

    motion: str
    robot: str
    implementation: str
    reference: str = ""
    checkpoint: str = ""
    compiler: str = "manifest-v2"
    schema: int = 1

    def validate(self) -> list[str]:
        issues: list[str] = []
        for field_name in ("motion", "robot", "implementation", "compiler"):
            if not getattr(self, field_name):
                issues.append(f"motion binding {field_name} is required")
        return issues

    @classmethod
    def from_skill(cls, skill: dict[str, Any], motion: MotionIR) -> "MotionBinding":
        ghost = skill.get("ghost") or {}
        policy = skill.get("policy") or {}
        robots = skill.get("robots") or []
        return cls(
            motion=motion.name,
            robot=str(robots[0]) if robots else "",
            implementation=str(skill.get("method", "")),
            reference=str(ghost.get("lut", "")),
            checkpoint=str(policy.get("checkpoint") or ""),
        )
