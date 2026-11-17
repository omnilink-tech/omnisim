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

"""A fixed (staticBase) robot root must COLLIDE under Newton.

The defect this pins (found 2026-09-29 on a user's 5-DOF arm): the staticBase
branch of the Newton flush registered the robot's root as a welded static body
and `continue`d without attaching its `boundingObject`. The compiled MuJoCo
model had ZERO geoms on the base, so nothing -- the robot's own links with
self-collision on, a part dropped on the base plate, another robot -- could
touch it. `/sim/contacts` was honestly empty (`mjData.ncon` was 0) and the log
said nothing.

The probe is a two-link arm on a 0.5 x 0.5 x 0.1 m base box (top at z = 0.10).
`j2` swings a 0.26 m bar down; its tip reaches the base top at j2 ~= 0.66 rad.
The bar is commanded to 1.22 rad through `POST /robot/<def>/joints/set`:

  case                                   j2 after the ramp   /sim/contacts
  default (fixed)                        ~0.664              <robot name>+arm
  OMNISIM_NEWTON_STATIC_BASE_COLLIDERS=0 ~1.222 (89 mm in)   empty

`arm` vs the base is a GRANDPARENT pair, so newton's parent/child filter does
not apply; `OMNISIM_NEWTON_SELF_COLLISION=1` switches the default intra-robot
filter off (without it this pair is filtered BY DESIGN). The root's contacts
report under the ROBOT's name, not `base_link`, because the URDF root link is
the Robot node itself.

The same run pins the harness half of the fix: the ramp's motion is produced
inside `joints/set`'s `settle_steps` (`_advance`), which until 2026-09-30 never
polled the event producers -- so the contact was in `/sim/contacts` and never
on `/sim/events`. It must now fire `contact.began`.

    python -m pytest tests/test_newton_static_base_colliders.py -v

Honours OMNISIM_BINARY (the harness does), so a scratch build can be pinned
without touching the live engine. Two harness launches, ~1 minute each. CPU
`newtonSolver "mujoco"`. Ports default to 16993/16994
(OMNISIM_TEST_STATIC_BASE_PORT overrides the first; the second is +1).
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
# OMNISIM_TEST_STATIC_BASE_PORT: harness port this engine test binds (default 16993).
PORT = int(os.environ.get("OMNISIM_TEST_STATIC_BASE_PORT", "16993"))
BASE = f"http://127.0.0.1:{PORT}"
ROBOT_NAME = "fixed_rootbox"

#: Where the bar's tip meets the base top (measured 0.6643 on the fixed build).
J2_CONTACT = 0.664
#: Where the ramp ends with nothing in the way (measured 1.2220 on the old build).
J2_COMMANDED = 1.22

URDF = """<?xml version="1.0"?>
<robot name="rootbox">
  <link name="base_link">
    <inertial><origin xyz="0 0 0.05"/><mass value="2.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.08"/></inertial>
    <visual><origin xyz="0 0 0.05"/><geometry><box size="0.5 0.5 0.1"/></geometry></visual>
    <collision><origin xyz="0 0 0.05"/><geometry><box size="0.5 0.5 0.1"/></geometry></collision>
  </link>
  <link name="post">
    <inertial><origin xyz="0 0 0.10"/><mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0001"/></inertial>
    <visual><origin xyz="0 0 0.10"/><geometry><box size="0.04 0.04 0.20"/></geometry></visual>
    <collision><origin xyz="0 0 0.10"/><geometry><box size="0.04 0.04 0.20"/></geometry></collision>
  </link>
  <link name="arm">
    <inertial><origin xyz="0.15 0 0"/><mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.0015"/></inertial>
    <visual><origin xyz="0.17 0 0"/><geometry><box size="0.26 0.04 0.04"/></geometry></visual>
    <collision><origin xyz="0.17 0 0"/><geometry><box size="0.26 0.04 0.04"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="post"/>
    <origin xyz="0 0 0.10"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="5" velocity="3"/></joint>
  <joint name="j2" type="revolute"><parent link="post"/><child link="arm"/>
    <origin xyz="0 0 0.20"/><axis xyz="0 1 0"/>
    <limit lower="-1.6" upper="1.6" effort="5" velocity="3"/></joint>
</robot>
"""

WORLD = """#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
WorldInfo {
  basicTimeStep 4
  newtonSolver "mujoco"
  newtonCompoundColliders TRUE
}
Viewpoint { orientation -0.2 0.12 0.97 2.0 position 0.5 -1.2 0.7 }
RectangleArena { floorSize 2 2 }
DEF ARM URDFRobot {
  url "root_box.urdf"
  name "%s"
  staticBase TRUE
  controller "<none>"
}
""" % ROBOT_NAME


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _call(method, path, body=None, timeout=600):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
        return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as e:
        return {"_http_error": e.code, "_body": e.read().decode("utf-8", "replace")[:2000]}


def _healthy():
    try:
        return urllib.request.urlopen(BASE + "/healthz", timeout=3).status == 200
    except Exception:  # noqa: BLE001
        return False


def _reap(proc):
    """`python -m omnisim harness` spawns the real harness as a child, and the
    engine survives its parent -- kill the whole tree, plus anything still
    listening on our two ports."""
    try:
        import psutil
    except ImportError:
        proc.kill()
        return
    procs = []
    try:
        root = psutil.Process(proc.pid)
        procs = root.children(recursive=True) + [root]
    except psutil.NoSuchProcess:
        pass
    for c in psutil.net_connections("tcp"):
        if c.laddr and c.laddr.port in (PORT, PORT + 1) and c.status == "LISTEN" and c.pid:
            try:
                p = psutil.Process(c.pid)
                procs += p.children(recursive=True) + [p]
            except psutil.NoSuchProcess:
                pass
    for p in procs:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass
    psutil.wait_procs(procs, timeout=15)


def _ramp(tmp_path, tag, hatch=None):
    """Launch a private harness, ramp j2 to 1.22 rad through joints/set, and
    return (final j2, contact pair names, contact.began events)."""
    if _healthy():
        pytest.skip(f"port {PORT} is already serving a harness; set OMNISIM_TEST_STATIC_BASE_PORT")
    work = tmp_path / tag
    work.mkdir(parents=True, exist_ok=True)
    (work / "root_box.urdf").write_text(URDF, encoding="utf-8")
    world = work / "w_root.omniworld"
    world.write_text(WORLD, encoding="utf-8")
    env = dict(os.environ, OMNISIM_NEWTON_SELF_COLLISION="1",
               OMNISIM_LOG_PATH=str(work / "engine.log"))
    env.pop("OMNISIM_NEWTON_STATIC_BASE_COLLIDERS", None)
    if hatch is not None:
        env["OMNISIM_NEWTON_STATIC_BASE_COLLIDERS"] = hatch
    out = open(work / "harness.txt", "w")
    proc = subprocess.Popen([sys.executable, "-m", "omnisim", "harness", "--port", str(PORT),
                             "--supervisor-port", str(PORT + 1)],
                            cwd=str(REPO), env=env, stdout=out, stderr=subprocess.STDOUT)
    try:
        t0 = time.time()
        while not _healthy():
            if proc.poll() is not None or time.time() - t0 > 180:
                pytest.fail("harness did not come up; see " + str(work / "harness.txt"))
            time.sleep(1)
        load = _call("POST", "/world/load", {"path": str(world), "light": False}, timeout=900)
        assert load.get("ok", True) and "_http_error" not in load, load
        _call("POST", "/sim/pause", {"lease_ms": 300000})
        names = [j["name"] for j in _call("GET", "/robot/ARM/joints").get("joints", [])]
        j2 = sorted((n for n in names if n.startswith("j2")), key=len)[-1]
        hold = {n: 0.0 for n in names if n != j2}
        ev = _call("GET", "/sim/events?since=0&log_since=0&limit=1")
        cursor = (ev.get("next_since", 0), ev.get("next_log_since", 0))
        n = 20
        achieved = None
        for i in range(n + 2):
            q = J2_COMMANDED * min(i, n) / n
            r = _call("POST", "/robot/ARM/joints/set",
                      {"joints": dict(hold, **{j2: q}), "settle_steps": 125 if i == n + 1 else 10})
            achieved = ((r.get("joints") or {}).get(j2) or {}).get("achieved")
        contacts = _call("GET", "/sim/contacts").get("contacts", [])
        pairs = {frozenset((c.get("a_name"), c.get("b_name"))) for c in contacts}
        events = _call("GET", "/sim/events?since=%s&log_since=%s&limit=5000&types=contact.began"
                       % cursor).get("events", [])
        _call("POST", "/sim/resume", {})
        return achieved, pairs, events
    finally:
        _reap(proc)
        out.close()


def test_fixed_root_stops_the_arm_and_reports_the_contact(tmp_path):
    achieved, pairs, events = _ramp(tmp_path, "fixed")
    assert achieved is not None
    assert abs(achieved - J2_CONTACT) < 0.05, (
        f"j2 ended at {achieved:.4f} rad: the bar passed into the fixed base "
        f"(it stops at ~{J2_CONTACT} when the base collides)")
    assert frozenset((ROBOT_NAME, "arm")) in pairs, pairs
    # The motion came from joints/set settle steps, which now poll the producers.
    assert events, "contact.began never fired for a contact /sim/contacts reports"


def test_hatch_zero_restores_the_shapeless_root(tmp_path):
    """The revert must be exact, or a regression from the fix is not bisectable
    with one variable."""
    achieved, pairs, _ = _ramp(tmp_path, "hatch0", hatch="0")
    assert achieved is not None and achieved > J2_COMMANDED - 0.05, achieved
    assert frozenset((ROBOT_NAME, "arm")) not in pairs, pairs
