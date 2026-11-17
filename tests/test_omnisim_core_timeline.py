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

"""Time control on the in-process core (omnisim_core/timeline.py): record, replay, seek, branch, diff.

What is pinned is the promise each verb makes:

* a recording replays EXACTLY (every tick's fingerprint), and a tampered one is caught at the tick tampered with;
* a run opened at any tick is the run that was there (keyframe + stepping lands on the recorded state);
* a branch is the parent, bit for bit, up to the change, and parts from it AT the change;
* a diff names the first tick two runs differ and says whether the commands or the physics parted first;
* a change scheduled into a live run gives the same run as the branch when the commands are the same.

Runs in a child interpreter for the reason tests/test_omnisim_core_stepping.py gives (the engine's own
MuJoCo build goes first on the import path), which is why it sits in the engine lane though it starts no simulator.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "packages" / "omnisim-core" / "src"

CHECK = r'''
import json, math, sys, shutil
from pathlib import Path
from omnisim_core.runtime import ensure_runtime
ok, why = ensure_runtime()
if not ok:
    print("SKIP:", why)
    raise SystemExit(0)
import numpy as np, mujoco
from omnisim_core import core as C, timeline as T
from omnisim_core.errors import CoreUnsupported

tmp = Path(sys.argv[1])
XML = """<mujoco><option timestep="0.002"/><worldbody>
 <geom name="floor" type="plane" size="5 5 0.1" friction="1 0.005 0.0001"/>
 <body name="ball" pos="0 0 0.3"><freejoint/><geom type="sphere" size="0.05" mass="0.5" friction="1 0.005 0.0001"/></body>
 <body name="car" pos="1 0 0.2"><joint name="s" type="slide" axis="1 0 0" range="-0.5 0.5" limited="true"/>
  <geom type="box" size="0.05 0.05 0.05" mass="1"/></body>
</worldbody><actuator><position joint="s" kp="100"/><velocity joint="s" kv="10"/></actuator></mujoco>"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
s0 = np.zeros(mujoco.mj_stateSize(m, C.P_SPEC)); mujoco.mj_getState(m, d, s0, C.P_SPEC)
jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "s")
ball, car = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball"), mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "car")
names = {"dt_ms": 2.0, "defs": {"BALL": {"pos": [0, 0, 0.3], "dynamic": True, "body": ball}}, "solid_body": {}, "milling": [],
         "robots": [{"name": "car", "def": "CAR", "root_body": car, "bodies": [car], "controller": "<none>",
                     "motors": [{"name": "s_motor", "sensor": "s_sensor", "jid": jid, "a_pos": 0, "a_vel": 1, "vmax": 1.0, "fmax": 10.0,
                                 "servo": True, "child_link": "car_link", "body": car}]}]}
comp = tmp / "compiled"; comp.mkdir()
mujoco.mj_saveModel(m, str(comp / C.MODEL), None)
np.save(comp / C.STATE0, s0)
(comp / C.ENGINE).write_text(json.dumps({"ok": True, "substeps": 1, "slack": [0.05, 0.005], "has_clamp": False}))
(comp / C.META).write_text(json.dumps({"world": "test", "names": names}))
N = 400

def command(t):
    return 0.3 * math.sin(t * 0.02)

def drive(core, n, before=None):
    while core.ticks < n:
        core.ctrl[0] = float(np.float32(command(core.ticks)))
        if core.ticks == 50:                                       # a supervisor pose write, as the API queues it
            j = int(core.m.body_jntadr[ball]); qa = int(core.m.jnt_qposadr[j]); da = np.arange(core.m.jnt_dofadr[j], core.m.jnt_dofadr[j] + 6)
            core.pending_pose.append((qa, da, [0.2, 0.0, 0.4], None))
        if before:
            before(core)
        core.step()

core = C.Core.from_compiled(comp)
rec = T.Recorder(core, keyframe_every=64); core.recorder = rec
drive(core, N)
core.recorder = None
final = T.state_hash(core)
T.Recorder.save(rec, tmp / "run", comp, {"kind": "run", "world": "test"})
R = T.Recording(tmp / "run")
assert R.ticks == N and len(R.key_tick) == 1 + N // 64 and int(R.hash[-1]) == final

# 1. replay is exact, keyframes restore the recorded state, and a seek from the middle arrives the same
v = R.verify()
assert v["exact"] and v["first_tick_that_differs"] is None and not v["keyframes_that_differ"] and v["seek_from_the_middle_arrives_the_same"], v
# 2. the run opened at any tick is the run that was there (between keyframes, at one, just after the pose write, at the end)
for t in (1, 51, 63, 64, 65, 137, 399, 400):
    assert T.state_hash(R.at(t)) == int(R.hash[t - 1]), t
c = R.at(137); R.advance(c, N); assert T.state_hash(c) == final
# 3. a tampered recording is caught at the tick tampered with
shutil.copytree(tmp / "run", tmp / "tampered")
z = dict(np.load(tmp / "tampered" / T.DATA)); z["ctrl"] = z["ctrl"].copy(); z["ctrl"][222, 0] += 1e-9
np.savez_compressed(tmp / "tampered" / T.DATA, **z)
bad = T.Recording(tmp / "tampered").verify()
assert not bad["exact"] and bad["first_tick_that_differs"] == 222, bad

# 4. a branch is the parent up to the change and parts from it AT the change
B = T.Recording(T.branch(R, 200, [T.parse_edit("vel:BALL=1,0,0")], tmp / "shove"))
assert B.ticks == N and B.edit_tick == 200 and np.array_equal(B.hash[:200], R.hash[:200]) and B.hash[200] != R.hash[200]
assert np.array_equal(B.ctrl, R.ctrl) and B.verify()["exact"]
for t in (199, 200, 201, 300):                                      # seeking across the change, and from keyframes after it
    assert T.state_hash(B.at(t)) == int(B.hash[t - 1]), t
dv = T.diff(R, B)
assert not dv["identical"] and dv["first_tick_state_differs"] == 200 and dv["first_tick_commands_differ"] is None
assert "physics parted first" in dv["verdict"] and dv["at_the_end"]["bodies"][0]["body"] == "BALL" and dv["growth"]["body"] == "BALL"
assert T.diff(R, R)["identical"]
try:
    T.branch(B, 300, [T.parse_edit("vel:BALL=0,1,0")], tmp / "nested"); raise SystemExit("a branch of a branch must be refused")
except CoreUnsupported:
    pass
# 5. a command override shows up as the commands parting first
Cc = T.Recording(T.branch(R, 200, [T.parse_edit("ctrl:s_motor=0.4")], tmp / "held"))
dc = T.diff(R, Cc)
assert dc["first_tick_commands_differ"] == 200 and "commands parted first" in dc["verdict"] and Cc.verify()["exact"]
assert abs(float(Cc.at(N).d.qpos[m.jnt_qposadr[jid]]) - 0.4) < 0.1      # 0.4 s on: the servo is closing on the held target

# 6. the same change scheduled into a live run, with the same commands, IS the branch
core2 = C.Core.from_compiled(comp)
rec2 = T.Recorder(core2, keyframe_every=64); core2.recorder = rec2
iv = T.Intervention(200, [T.parse_edit("vel:BALL=1,0,0")], names)
drive(core2, N, before=iv.before_step)
core2.recorder = None
rec2.edit_tick, rec2.edits = iv.tick, iv.edits
L = T.Recording(rec2.save(tmp / "live", comp, {"kind": "run"}))
assert T.diff(B, L)["identical"] and L.verify()["exact"]

# 7. the changes do what they say
F = T.Recording(T.branch(R, 300, [T.parse_edit("friction:BALL=0.05"), T.parse_edit("mass:BALL=2.0"), T.parse_edit("gain:s_motor=0.5")], tmp / "edits"))
fc = F.at(N)
g = int(np.nonzero(fc.m.geom_bodyid == ball)[0][0])
assert fc.m.geom_friction[g, 0] == 0.05 and fc.m.geom_priority[g] > 0 and abs(fc.m.body_mass[ball] - 2.0) < 1e-12
assert abs(fc.m.actuator_gainprm[0, 0] - 50.0) < 1e-9 and F.verify()["exact"]
snap = T.snapshot(F, fc)
touch = [c for c in snap["contacts"] if {c["a"], c["b"]} == {"BALL", "world"}]
assert touch and abs(touch[0]["friction"] - 0.05) < 1e-12                          # the body's own value wins the contact
assert abs(sum(c["normal_force"] for c in touch) - 2.0 * 9.81) < 0.5              # and it rests with its new weight
row = [j for j in snap["joints"] if j["motor"] == "s_motor"][0]
assert row["min"] == -0.5 and row["max"] == 0.5 and row["at_limit"] is False and "actuator_force" in row
# 8. events found by replay: the ball lands on the floor
ev = T.events(R)["events"]
landings = [e for e in ev if e["type"] == "contact.began" and {e["a"], e["b"]} == {"BALL", "world"}]
assert landings and landings[0]["tick"] > 10, ev[:4]
# 9. edits are refused by name, not guessed
for bad_edit in ("friction:NOPE=1", "ctrl:nope=1", "pos:CAR=0,0,0"):
    try:
        T.branch(R, 10, [T.parse_edit(bad_edit)], tmp / "bad"); raise SystemExit("accepted " + bad_edit)
    except CoreUnsupported:
        pass
for text in ("friction=1", "spin:BALL=1", "pos:BALL=1,2"):
    try:
        T.parse_edit(text); raise SystemExit("parsed " + text)
    except ValueError:
        pass
print("OK")
'''


def test_timeline_record_replay_seek_branch_diff(tmp_path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    r = subprocess.run([sys.executable, "-c", CHECK, str(tmp_path)], cwd=ROOT, env=env, capture_output=True, text=True, timeout=600)
    out = (r.stdout or "") + (r.stderr or "")
    if "SKIP:" in r.stdout:
        pytest.skip("the engine's physics runtime bundle is not staged here: " + r.stdout.strip()[-200:])
    assert r.returncode == 0 and r.stdout.strip().endswith("OK"), out[-3000:]
