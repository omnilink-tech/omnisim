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

"""Mars-world observer.

A Webots Supervisor that walks the scene tree, finds every Robot
whose ``name`` starts with ``husky_``, and logs each husky's world
position every 2 s of sim time to a per-run file. Used to verify
offline whether the husky controllers are actually causing the
robots to move (versus just commanding wheels that spin in place).

The supervisor is added to the Mars world only when
``husky_count > 0`` AND ``husky_observer=True``, which the recipe
defaults to True so the test machinery is always present.
"""

from __future__ import annotations

import math
import os
import time

from omnisim import Supervisor


_TRACE_DIRS = (
    r"C:\tmp\husky_trace",
    "/tmp/husky_trace",
)
_LOG_INTERVAL_MS = 2000


def _open_log():
    for d in _TRACE_DIRS:
        try:
            os.makedirs(d, exist_ok=True)
            return open(
                os.path.join(d, "_observer.log"),
                "w", encoding="utf-8", buffering=1,
            )
        except Exception:
            continue
    return None


def _find_huskies(sup):
    """Walk the top-level scene-tree children and collect every Robot
    node whose name starts with ``husky_``. Returns a list of
    ``(name, node)`` pairs."""
    out = []
    root = sup.getRoot()
    children = root.getField("children")
    if children is None:
        return out
    for i in range(children.getCount()):
        try:
            node = children.getMFNode(i)
        except Exception:
            continue
        type_name = node.getTypeName()
        # The OmniSim URDFRobot expands to a Robot node at the top of
        # the imported tree; the type name is "Robot".
        if type_name != "Robot":
            continue
        name_field = node.getField("name")
        if name_field is None:
            continue
        name = name_field.getSFString()
        if name.startswith("husky_"):
            out.append((name, node))
    return out


def main() -> None:
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    log = _open_log()
    if log is not None:
        log.write(f"observer_boot wall={time.time():.3f}\n")

    huskies = _find_huskies(sup)
    if log is not None:
        log.write(f"found_huskies count={len(huskies)} "
                  f"names={[n for n, _ in huskies]}\n")
    if not huskies:
        # Nothing to observe; just spin.
        while sup.step(ts) != -1:
            pass
        return

    # Capture initial positions for displacement tracking.
    initial = {}
    # Per-husky max linear speed seen since last log emission. Reset on
    # each log so we get peak-per-window not peak-since-start.
    max_speed_window = {name: 0.0 for name, _ in huskies}
    for name, node in huskies:
        try:
            pos = node.getPosition()
            initial[name] = (pos[0], pos[1])
        except Exception:
            initial[name] = (0.0, 0.0)

    next_log_ms = 0
    sim_ms = 0
    while sup.step(ts) != -1:
        sim_ms += ts
        # Sample linear-speed every step so the per-window max captures
        # transient peaks (acceleration spikes, hard collisions, etc.).
        for name, node in huskies:
            try:
                vel = node.getVelocity()  # [vx, vy, vz, wx, wy, wz]
            except Exception:
                continue
            speed = math.sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2])
            if speed > max_speed_window[name]:
                max_speed_window[name] = speed

        if log is None or sim_ms < next_log_ms:
            continue
        next_log_ms = sim_ms + _LOG_INTERVAL_MS
        for name, node in huskies:
            try:
                pos = node.getPosition()
                vel = node.getVelocity()
            except Exception:
                continue
            x, y, z = pos[0], pos[1], pos[2]
            speed = math.sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2])
            ang_speed = math.sqrt(
                vel[3] * vel[3] + vel[4] * vel[4] + vel[5] * vel[5]
            )
            ix, iy = initial[name]
            d = math.hypot(x - ix, y - iy)
            peak = max_speed_window[name]
            max_speed_window[name] = 0.0
            log.write(
                f"t_ms={sim_ms} {name} pos=({x:.2f},{y:.2f},{z:.2f}) "
                f"disp_from_start_m={d:.2f} "
                f"speed_m_s={speed:.3f} ang_speed_rad_s={ang_speed:.3f} "
                f"peak_speed_m_s={peak:.3f}\n"
            )


if __name__ == "__main__":
    main()
