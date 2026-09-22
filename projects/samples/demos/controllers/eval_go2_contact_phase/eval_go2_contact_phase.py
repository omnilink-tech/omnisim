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

"""When does each foot ACTUALLY touch down, against when the gait intended it to?

The reference is the foot-space trot model in
projects/policies/control/gait/go2_trot_gait.py, which states its intent
explicitly rather than leaving it implicit in a policy: diagonal pairs
(FL+RR at phase 0, FR+RL at phase 0.5) and a duty fraction of stance.

Only the model is run -- no learned residual -- so what is measured is the
gait against the ground, not a policy's correction of it. Per tick this
records, for each of the four feet:

    intended  stance or swing, from the model's own phase and duty
    actual    whether that foot reports a contact this tick

and then reports, per contact interval, the touchdown and lift-off phase
against the intended ones. Contact presence only: this engine reports no
contact force, so nothing here says how much load went through a foot.
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

# Walk up to the repo root and APPEND -- inserting at position 0 shadows the
# runtime `omnisim` controller module and the controller dies at import.
# This is the pattern go2_walk_deploy.py already uses.
_REPO = next(_p for _p in Path(__file__).resolve().parents
             if (_p / "projects" / "policies").is_dir()
             or (_p / "AGENTS.md").exists())
sys.path.append(str(_REPO))

from omnisim import Supervisor                                # noqa: E402
from projects.policies.control.gait import go2_trot_gait as stg   # noqa: E402

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260915",
                   "guangsen")

LEGS = ["FL", "FR", "RL", "RR"]
PARTS = ["hip", "thigh", "calf"]
JOINT_NAMES = ["%s_%s_joint" % (leg, part) for leg in LEGS for part in PARTS]

SETTLE_S = 1.0
RUN_S = 6.0


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0

    me = robot.getFromDef("GO2") or robot.getSelf()
    motors, sensors = [], []
    for jn in JOINT_NAMES:
        m = robot.getDevice(jn + "_motor")
        s = robot.getDevice(jn + "_sensor")
        if m is None:
            print("[go2] FATAL missing motor %s_motor" % jn, flush=True)
            sys.exit(1)
        if s is not None:
            s.enable(ms)
        motors.append(m)
        sensors.append(s)

    # Foot bodies for contact readback. URDF-imported links carry a NAME,
    # not a DEF, so getFromDef finds nothing -- walk the tree and match the
    # name field instead. The links that actually touch the floor are the
    # calves (the foot is the calf tip), which is what /sim/contacts names.
    def walk(node, out, depth=0):
        if node is None or depth > 12:
            return
        try:
            f = node.getField("name")
            nm = f.getSFString() if f is not None else None
        except Exception:
            nm = None
        if nm:
            out[nm] = node
        for fld in ("children", "endPoint"):
            try:
                ff = node.getField(fld)
            except Exception:
                ff = None
            if ff is None:
                continue
            try:
                if fld == "endPoint":
                    walk(ff.getSFNode(), out, depth + 1)
                else:
                    for i in range(ff.getCount()):
                        walk(ff.getMFNode(i), out, depth + 1)
            except Exception:
                pass

    found = {}
    walk(robot.getRoot(), found)
    feet = {}
    for leg in LEGS:
        feet[leg] = found.get(leg + "_calf") or found.get(leg + "_foot")
    resolved = {k: (v is not None) for k, v in feet.items()}
    probe = {}
    for leg, n in feet.items():
        if n is None:
            probe[leg] = "node not found"
            continue
        try:
            pts = n.getContactPoints(True)
            probe[leg] = "getContactPoints ok, %d pts at t0" % len(pts or [])
        except TypeError:
            try:
                pts = n.getContactPoints()
                probe[leg] = "getContactPoints(no-arg) ok, %d pts" % len(pts or [])
            except Exception as e:
                probe[leg] = "raised: %r" % (e,)
        except Exception as e:
            probe[leg] = "raised: %r" % (e,)
    print("[go2] foot nodes resolved: %s" % resolved, flush=True)

    gp = stg.GaitParams()
    omega = 2.0 * math.pi * gp.freq

    def foot_touching(leg):
        n = feet.get(leg)
        if n is None:
            return None
        try:
            pts = n.getContactPoints(True)
        except TypeError:
            pts = n.getContactPoints()
        except Exception:
            return None
        return len(pts or []) > 0

    rec = {"gait": {"freq_hz": gp.freq, "duty": gp.duty, "vx": gp.vx,
                    "body_height": gp.body_height,
                    "step_height": gp.step_height,
                    "cycle_s": 1.0 / gp.freq,
                    "intended_stance_s": gp.duty / gp.freq,
                    "intended_swing_s": (1.0 - gp.duty) / gp.freq},
           "basic_time_step_ms": ms,
           "foot_node_names_searched": [l + "_calf" for l in LEGS],
           "samples": []}

    # hold the model's standing pose while the robot settles
    t = 0.0
    q0, _ = stg.targets_np(stg.QS_PHASE, gp, t_since_start=0.0)
    while robot.step(ms) != -1 and t < SETTLE_S:
        for m, q in zip(motors, q0):
            m.setPosition(float(q))
        t += dt

    gait_t = 0.0
    t = 0.0
    while robot.step(ms) != -1 and t < RUN_S:
        phase = stg.QS_PHASE + omega * gait_t
        q, sw = stg.targets_np(phase, gp, t_since_start=gait_t)
        for m, qq in zip(motors, q):
            m.setPosition(float(qq))

        # the model's own intent for each leg this tick
        # leg phase: FL and RR at 0, FR and RL at 0.5
        intended = {}
        for i, leg in enumerate(LEGS):
            off = 0.0 if leg in ("FL", "RR") else 0.5
            u = ((phase / (2.0 * math.pi)) + off) % 1.0
            intended[leg] = "stance" if u < gp.duty else "swing"
        actual = {leg: foot_touching(leg) for leg in LEGS}

        p = me.getPosition() if me is not None else [0, 0, 0]
        rec["samples"].append({
            "t": round(t, 4), "gait_t": round(gait_t, 4),
            "phase_frac": round((phase / (2 * math.pi)) % 1.0, 5),
            "intended": intended, "contact": actual,
            "base_xyz": [round(float(v), 5) for v in p],
        })
        gait_t += dt
        t += dt

    rec["foot_nodes_resolved"] = resolved
    rec["contact_probe_at_start"] = probe
    rec["node_names_seen"] = sorted(k for k in found
                                    if any(t in k for t in
                                           ("calf", "foot", "thigh", "base")))
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "contact_phase.json"), "w") as fh:
        json.dump(rec, fh)
    n = len(rec["samples"])
    got = sum(1 for s in rec["samples"] if any(v for v in s["contact"].values()
                                               if v))
    print("[go2] %d samples, %d with at least one foot contact" % (n, got),
          flush=True)
    print("[go2] wrote contact_phase.json", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "tb.txt"), "a") as fh:
            fh.write(tb)
        print("[go2] FAILED " + tb, flush=True)
        raise
