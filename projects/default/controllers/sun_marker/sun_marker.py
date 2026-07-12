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

"""Unreal-style sun-position marker driver.

Watches a `DEF SUN_MARKER` Solid node and uses its translation as the
sun direction.  The user drags the marker in the 3D viewport (or
edits its translation in the scene tree) and the atmospheric sky +
DirectionalLight update live thanks to WbBackground's auto-rebake
on directionChanged.

Keys
----
  V  : toggle marker visibility (parks it far above the scene when
       hidden; the last visible position is remembered so the sun
       direction doesn't jump when you toggle back on).

The on-screen HUD shows current azimuth/elevation derived from the
marker's position.
"""

from __future__ import annotations

import math
import os

from omnisim import Keyboard, Supervisor


HIDDEN_TRANSLATION = (0.0, 0.0, 100000.0)
LABEL_COLOUR = 0xFFCC55
LABEL_DIM = 0x666666


def find_first_node_by_type(supervisor: Supervisor, type_names):
    # Accept a single name or a tuple of names. Matches against both the
    # node's runtime typeName (e.g. "DirectionalLight") and its PROTO
    # typeName when wrapped (e.g. "OmniSimSun"), so the fallback works
    # for both raw nodes and the canonical OmniSim recipe PROTOs.
    if isinstance(type_names, str):
        type_names = (type_names,)
    root = supervisor.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        n = children.getMFNode(i)
        if n is not None and n.getTypeName() in type_names:
            return n
    return None


def main() -> None:
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    # SUN_MARKER may be: (a) a stand-alone Solid (legacy spot.wbt layout)
    # or (b) the supervisor robot itself when worlds use the OmniSimSunMarker
    # PROTO that collapses the marker visual and the driver robot into one
    # Robot node. getSelf() covers (b) when getFromDef cannot reach it.
    marker = supervisor.getFromDef("SUN_MARKER") or supervisor.getSelf()
    if marker is None:
        print("[sun_marker] no DEF SUN_MARKER node found; controller idle", flush=True)
        while supervisor.step(time_step) != -1:
            pass
        return

    sun = supervisor.getFromDef("SUN") or find_first_node_by_type(
        supervisor, ("DirectionalLight", "OmniSimSun")
    )
    if sun is None:
        print("[sun_marker] no DirectionalLight in scene; controller idle", flush=True)
        return

    marker_t = marker.getField("translation")
    sun_dir = sun.getField("direction")
    if marker_t is None or sun_dir is None:
        print("[sun_marker] missing translation or direction field", flush=True)
        return

    # The marker starts parked off-screen by default so demo worlds
    # don't get a yellow sphere floating above the robot. The sun
    # direction is still driven from the marker's *last visible*
    # position, so the lighting is unaffected. User can press V to
    # bring the marker into the viewport. Set OMNISIM_SUN_MARKER_HIDDEN=0
    # to override and start with the marker visible.
    start_hidden = os.environ.get("OMNISIM_SUN_MARKER_HIDDEN", "1").strip() != "0"
    last_visible_pos = list(marker_t.getSFVec3f())
    visible = not start_hidden
    if start_hidden:
        marker_t.setSFVec3f(list(HIDDEN_TRANSLATION))

    keyboard = supervisor.getKeyboard()
    keyboard.enable(time_step)
    v_was_down = False

    _ready_msg = ("ready — marker HIDDEN, press V to show"
                  if start_hidden else
                  "ready — drag the glowing marker, press V to hide")
    print(f"[sun_marker] {_ready_msg}", flush=True)

    while supervisor.step(time_step) != -1:
        keys = []
        k = keyboard.getKey()
        while k != -1:
            keys.append(k & ~Keyboard.SHIFT & ~Keyboard.CONTROL & ~Keyboard.ALT)
            k = keyboard.getKey()
        v_down = ord("V") in keys

        if v_down and not v_was_down:
            if visible:
                last_visible_pos = list(marker_t.getSFVec3f())
                marker_t.setSFVec3f(list(HIDDEN_TRANSLATION))
                visible = False
            else:
                marker_t.setSFVec3f(last_visible_pos)
                visible = True
        v_was_down = v_down

        # Source the sun direction from whichever position represents
        # "where the user currently wants the sun" — the live marker
        # when visible, the last stored position when hidden.
        active_pos = marker_t.getSFVec3f() if visible else last_visible_pos
        mag = math.sqrt(active_pos[0] ** 2 + active_pos[1] ** 2 + active_pos[2] ** 2)
        if mag > 1e-3:
            inv = 1.0 / mag
            # DirectionalLight.direction is the direction light *travels*
            # (away from the sun), so negate the toward-sun vector.
            sun_dir.setSFVec3f([-active_pos[0] * inv,
                                -active_pos[1] * inv,
                                -active_pos[2] * inv])


if __name__ == "__main__":
    main()
