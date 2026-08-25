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

"""The T1 scene on the OmniSim column: the container's Husky on a floor.

The world is written next to the container's own description so the
``URDFRobot url`` stays a short relative path, and the robot is turned a
quarter turn about z so that its own forward axis points at **+y, north** --
the direction the task's offset is expressed in. Nothing here knows the
distance; the offset reaches the controller as an argument from the oracle,
which reads it from ``meta.json``.

The floor is a plain ``Solid`` with a ``boundingObject`` (a floor without one
is a hologram -- see AGENTS.md), sized so a 5 m run and its overshoot stay on
it with room to spare.
"""

from __future__ import annotations

import os
from pathlib import Path

WORLD_TEMPLATE = """#VRML_SIM R2025a utf8

# T1 arrive: the container's Husky drives north and stops. A SCRIPTED control
# (ladder/controllers/ladder_t1_drive), not an agent's work.

EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"

WorldInfo {{
  basicTimeStep 16
  title "ladder T1: arrive"
  defaultPhysicsBackend "{backend}"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonCompoundColliders TRUE
  contactProperties [
    ContactProperties {{ material1 "default" material2 "default"
                        coulombFriction [ 1.2 ] bounce 0 }}
  ]
  # The SAME friction, declared again for the Newton path, which does not read
  # contactProperties. Measured 2026-08-02: without this the Husky ran at the
  # Newton default mu of 1.0, slipped, and stopped 4.6374 m along a 5 m run --
  # T1.1 red on Newton while ODE passed 5/5 from the identical file. The
  # engine now warns when the two disagree; this is the warning's own remedy.
  newtonGroundMu 1.2
}}
Viewpoint {{
  orientation -0.42 0.38 0.82 1.85
  position 9.5 -9.0 7.5
}}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}

DEF FLOOR Solid {{
  translation 0 6 0
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 0.42 0.44 0.46 roughness 1 metalness 0 }}
      geometry Box {{ size 40 40 0.1 }}
    }}
  ]
  name "floor"
  boundingObject Box {{ size 40 40 0.1 }}
}}

DEF HUSKY URDFRobot {{
  url "{url}"
  translation 0 0 0.18
  rotation 0 0 1 1.5708
  name "husky"
  controller "ladder_t1_drive"
  controllerArgs [ {args} ]
  supervisor TRUE
}}
"""


def build_world(out_dir, *, description_dir, offset_m=(0.0, 5.0),
                settle_s=0.5, duration_s=60.0, backend="ode",
                extra_args=()):
    """Write the T1 world and return its path.

    ``backend`` is explicit rather than "auto" because a row that does not name
    the backend it ran under cannot be compared with one that does.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    urdf = Path(description_dir) / "urdf" / "husky.urdf"
    if not urdf.is_file():
        raise FileNotFoundError(urdf)
    world = out_dir / "t1_arrive.wbt"
    try:
        rel = os.path.relpath(str(urdf.resolve()), str(world.parent.resolve()))
    except ValueError:
        # different drives (a run dir under %TEMP% on C:, the container on O:)
        # -- an absolute url is correct, just less tidy
        rel = str(urdf.resolve())
    args = " ".join('"%s"' % a for a in (
        "--settle-s", "%.3f" % float(settle_s),
        "--offset-x", "%.4f" % float(offset_m[0]),
        "--offset-y", "%.4f" % float(offset_m[1]),
        "--duration-s", "%.1f" % float(duration_s)) + tuple(extra_args))
    world.write_text(
        WORLD_TEMPLATE.format(url=rel.replace("\\", "/"), args=args,
                              backend=backend),
        encoding="utf-8")
    return world
