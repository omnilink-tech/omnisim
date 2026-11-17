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

"""Connector weld probe for tests/test_newton_weld_parity.py.

Runs on the bodiless ANCHOR supervisor robot whose active Connector hangs
0.2 m above the passive Connector on the free 1 kg PAYLOAD box. Timeline:

    t=3 ticks   lock()      -> the weld must catch the payload mid-air
    +0.25 s     pose A      -> reference pose once the engage transient dies
    +2.0 s      pose B      -> hold window (drift = |B - A|)
    unlock()
    +1.2 s      pose C      -> the payload must have fallen away

On the rupture world (finite tensileStrength < m*g) the payload rips free
during the hold window instead, so B is already near the floor. The poses ARE
the verdict; assertions live in the test. Output: OMNISIM_WELD_PROBE_OUT.
"""
import json
import os
import sys

from omnisim import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
conn = robot.getDevice("conn")
conn.enablePresence(dt)
payload = robot.getFromDef("PAYLOAD")


def pos():
    p = payload.getPosition()
    return [float(p[0]), float(p[1]), float(p[2])]


def advance(ms):
    n = max(1, int(round(ms / dt)))
    for _ in range(n):
        if robot.step(dt) == -1:
            return False
    return True


out = {}
advance(3 * dt)                       # let Newton finalize; payload falls ~0.3 mm
out["p_spawn"] = pos()
conn.lock()                           # autoLock TRUE retries every tick if this races finalize
advance(250)
out["p_lock"] = pos()                 # pose A
out["presence_locked"] = int(conn.getPresence())
advance(2000)
out["p_hold_end"] = pos()             # pose B
out["hold_drift"] = max(abs(a - b) for a, b in zip(out["p_lock"], out["p_hold_end"]))
conn.unlock()
advance(1200)
out["p_after_unlock"] = pos()         # pose C

path = os.environ.get("OMNISIM_WELD_PROBE_OUT", "weld_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("weld_parity_probe: wrote %s\n" % path)
sys.stdout.flush()
