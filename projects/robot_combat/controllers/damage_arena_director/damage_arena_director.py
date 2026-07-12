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

"""damage_arena_director — supervisor that drops boxes onto the Husky.

Phase 0 of the physics-based damage system. See
docs/developer/damage-system-plan.md.

Reads a JSON drop schedule from the supervisor robot's customData field
and spawns boxes via Supervisor.importMFNodeFromString as their scheduled
times come due. No damage tracking here; that lands in later phases on
the harness_supervisor side.

customData schema
-----------------
{
  "boxes": [
    {"t_ms": <int>,                    # required: ms since (re)start
     "pos": [x, y, z],                 # default [0, 0, 5]
     "size": <float>,                  # cube edge length, default 0.3
     "mass": <float>,                  # kg, default 5.0
     "color": [r, g, b]},              # default [0.7, 0.5, 0.3]
    ...
  ],
  "loop": <bool>,                      # default false
  "loop_period_ms": <int>,             # default 30000
  "cleanup_after_ms": <int>            # remove boxes older than this; default 60000
}

Empty/missing customData uses DEFAULT_SCHEDULE below.
"""

from __future__ import annotations

import json
import math
import sys

from omnisim import Supervisor

GRAVITY = 9.81  # m/s^2, for lead-the-target fall-time math


DEFAULT_SCHEDULE = {
    "loop": True,
    "loop_period_ms": 30000,
    "cleanup_after_ms": 60000,
    "boxes": [
        {"t_ms": 2000, "pos": [0.0, 0.0, 4.0], "size": 0.3, "mass": 5.0},
        {"t_ms": 5000, "pos": [0.4, 0.0, 5.0], "size": 0.4, "mass": 10.0,
         "color": [0.9, 0.4, 0.2]},
        {"t_ms": 9000, "pos": [-0.3, 0.2, 6.0], "size": 0.35, "mass": 7.0,
         "color": [0.2, 0.6, 0.9]},
    ],
}

DEFAULT_BOX_POS = [0.0, 0.0, 5.0]
DEFAULT_BOX_SIZE = 0.3
DEFAULT_BOX_MASS = 5.0
DEFAULT_BOX_COLOR = [0.7, 0.5, 0.3]


def parse_schedule(raw: str) -> dict:
    if not raw:
        return DEFAULT_SCHEDULE
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        sys.stderr.write(
            f"[damage_arena_director] customData JSON invalid ({exc}); using default schedule\n"
        )
        return DEFAULT_SCHEDULE
    if not isinstance(parsed, dict) or "boxes" not in parsed:
        sys.stderr.write(
            "[damage_arena_director] customData missing 'boxes'; using default schedule\n"
        )
        return DEFAULT_SCHEDULE
    return parsed


def box_node_string(name: str, pos: list[float], size: float, mass: float,
                    color: list[float]) -> str:
    """Build a VRML stanza for a dropped box. Kept on one logical node so
    importMFNodeFromString can append it directly to the root children
    field. The DEF name doubles as the node name so it shows up in the
    scene tree under a predictable label.
    """
    return (
        f'DEF {name} Solid {{'
        f'  translation {pos[0]} {pos[1]} {pos[2]}'
        f'  name "{name}"'
        f'  children ['
        f'    Shape {{'
        f'      appearance PBRAppearance {{'
        f'        baseColor {color[0]} {color[1]} {color[2]}'
        f'        roughness 0.7'
        f'        metalness 0.0'
        f'      }}'
        f'      geometry Box {{ size {size} {size} {size} }}'
        f'    }}'
        f'  ]'
        f'  boundingObject Box {{ size {size} {size} {size} }}'
        f'  physics Physics {{ density -1 mass {mass} }}'
        f'}}'
    )


def tracked_drop_pos(track_node, spec: dict) -> list[float]:
    """Compute a drop position that LEADS a moving target so the box
    actually lands on it.

    Fixed-position schedules can't hit a driving robot: by the time a box
    falls from several metres the robot has moved on. Here we read the
    target's live position + velocity and aim the drop at where it WILL
    be after the box's fall time (t = sqrt(2h/g)). `spec["z"]` (or the z
    of `spec["pos"]`) is the release height; optional `spec["offset"]`
    [dx, dy] nudges the aim point (e.g. to bias toward a bumper).
    """
    height = float(spec.get("z", spec.get("pos", [0.0, 0.0, 6.0])[2]))
    offset = spec.get("offset", [0.0, 0.0])
    pos = track_node.getPosition()           # [x, y, z]
    vel = track_node.getVelocity()           # [vx, vy, vz, wx, wy, wz]
    fall_h = max(0.1, height - pos[2])
    t_fall = math.sqrt(2.0 * fall_h / GRAVITY)
    return [
        pos[0] + vel[0] * t_fall + float(offset[0]),
        pos[1] + vel[1] * t_fall + float(offset[1]),
        height,
    ]


def spawn_box(supervisor: Supervisor, root_children, name: str, spec: dict) -> None:
    pos = spec.get("pos", DEFAULT_BOX_POS)
    size = float(spec.get("size", DEFAULT_BOX_SIZE))
    mass = float(spec.get("mass", DEFAULT_BOX_MASS))
    color = spec.get("color", DEFAULT_BOX_COLOR)
    if not isinstance(pos, list) or len(pos) != 3:
        sys.stderr.write(f"[damage_arena_director] box {name}: bad pos {pos}; skipping\n")
        return
    stanza = box_node_string(name, [float(x) for x in pos], size, mass,
                             [float(c) for c in color])
    root_children.importMFNodeFromString(-1, stanza)
    sys.stderr.write(
        f"[damage_arena_director] spawned {name} at {pos} size={size} mass={mass}\n"
    )


def cleanup_old_boxes(supervisor: Supervisor, spawned: dict, sim_time_ms: int,
                      cleanup_after_ms: int) -> None:
    """Remove boxes whose age exceeds cleanup_after_ms.

    spawned: {name: spawn_sim_time_ms}. We mutate in place.
    """
    if cleanup_after_ms <= 0:
        return
    expired = [name for name, t in spawned.items() if sim_time_ms - t >= cleanup_after_ms]
    for name in expired:
        node = supervisor.getFromDef(name)
        if node is not None:
            try:
                node.remove()
                sys.stderr.write(f"[damage_arena_director] cleaned up {name}\n")
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(f"[damage_arena_director] cleanup {name} failed: {exc}\n")
        spawned.pop(name, None)


def main() -> int:
    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())

    self_node = supervisor.getSelf()
    raw_custom_data = ""
    if self_node is not None:
        f = self_node.getField("customData")
        if f is not None:
            try:
                raw_custom_data = f.getSFString()
            except Exception:
                raw_custom_data = ""

    schedule = parse_schedule(raw_custom_data)
    boxes = schedule.get("boxes") or []
    loop = bool(schedule.get("loop", False))
    loop_period_ms = int(schedule.get("loop_period_ms", 30000))
    cleanup_after_ms = int(schedule.get("cleanup_after_ms", 60000))

    # Optional target tracking: when "track" names a DEF'd node, each box
    # is dropped to lead that node's live position so it lands on a moving
    # target instead of a precomputed fixed spot. Resolved once; if the
    # DEF isn't found we log and fall back to fixed positions.
    track_def = schedule.get("track")
    track_node = supervisor.getFromDef(track_def) if track_def else None
    if track_def and track_node is None:
        sys.stderr.write(
            f"[damage_arena_director] track target DEF {track_def!r} not found; "
            "using fixed box positions\n"
        )

    # Sort by t_ms so we can walk through the cursor monotonically.
    boxes = sorted(boxes, key=lambda b: int(b.get("t_ms", 0)))

    sys.stderr.write(
        f"[damage_arena_director] loaded {len(boxes)} drops, loop={loop}, "
        f"period={loop_period_ms}ms, cleanup_after={cleanup_after_ms}ms\n"
    )
    sys.stderr.flush()

    root = supervisor.getRoot()
    root_children = root.getField("children") if root is not None else None
    if root_children is None:
        sys.stderr.write("[damage_arena_director] cannot access root children; aborting\n")
        return 1

    spawned: dict[str, int] = {}  # def_name -> spawn time ms (absolute sim_time)
    spawn_counter = 0
    cursor = 0          # index into boxes for the current loop iteration
    loop_start_ms = 0   # absolute sim_time when current loop iteration started
    sim_time_ms = 0

    while supervisor.step(basic_step_ms) != -1:
        sim_time_ms += basic_step_ms
        elapsed = sim_time_ms - loop_start_ms

        # Spawn any boxes whose t_ms has come due in the current loop pass.
        while cursor < len(boxes) and elapsed >= int(boxes[cursor].get("t_ms", 0)):
            spec = boxes[cursor]
            if track_node is not None:
                # Aim this box at the moving target's predicted position.
                spec = dict(spec)
                try:
                    spec["pos"] = tracked_drop_pos(track_node, spec)
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[damage_arena_director] track aim failed: {exc}; "
                        "using spec pos\n"
                    )
            name = f"dropped_box_{spawn_counter:03d}"
            spawn_counter += 1
            spawn_box(supervisor, root_children, name, spec)
            spawned[name] = sim_time_ms
            cursor += 1

        # Loop bookkeeping: when the loop period elapses, restart the cursor.
        if loop and elapsed >= loop_period_ms:
            loop_start_ms = sim_time_ms
            cursor = 0

        cleanup_old_boxes(supervisor, spawned, sim_time_ms, cleanup_after_ms)

    return 0


if __name__ == "__main__":
    sys.exit(main())
