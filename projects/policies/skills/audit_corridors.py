#!/usr/bin/env python3
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

"""CORRIDOR-ADEQUACY AUDIT of the whole skill library (2026-07-13).

    python projects/policies/skills/audit_corridors.py

Grades every skill's shipped GHOST_RESIDUAL against the CORRIDOR-ADEQUACY LAW
(docs/developer/ghost-design-rules.md):

    dq_cop = m * g * (sole_length / 2) / kp        G1: 34.1 kg, 0.17 m sole, kp=200 -> 0.142 rad

To balance on the ankle a robot must drive its centre of pressure to the edge of the sole; that costs
tau = m*g*(sole/2), which through the PD plant costs dq = tau/kp of joint deviation. A corridor
narrower than dq_cop structurally forbids the policy from reaching the edge of its own foot -- and
whatever the corridor denies, the CRANE supplies.

⚠️ THIS IS A STATIC AUDIT, ON PURPOSE. The obvious alternative -- run each skill and watch for action
saturation -- DOES NOT WORK on the shipped library, and the failure is instructive: saturation only
appears in a policy that is TRYING to self-support. Every shipped G1 skill was trained at lam=0.9 with
HARNESS_GRAD_SURV=2.0 (the crane never weans), so none of them ever learned to reach for the corridor.
Measured: the flagship at lam=0.2 does not strain against its corridor -- it face-plants (base z=0.07 m,
roll 1.7 rad, within seconds). It does not fight and lose; it does not fight. So runtime saturation is a
TRAINING-time instrument, and this -- geometry, not behaviour -- is the audit-time one.
"""
import json
import os
import sys

_RT = os.environ.get("OMNISIM_HOME", os.getcwd())
if _RT not in sys.path:
    sys.path.insert(0, _RT)

from projects.policies.common import robot_registry as RR  # noqa: E402

SKILLS_DIR = os.path.join(_RT, "projects", "policies", "skills")
KP = float(os.environ.get("AUDIT_KP", "200"))


def _corridor_of(m):
    """The GHOST_RESIDUAL this skill actually ships with (deploy env wins; else the train recipe).

    ⛔ Only g1_walk declares one. The other four Shadowing skills carry NO GHOST_RESIDUAL in their
    manifest -- their corridor lives in the BATON sequence env (which `skill_lib verify-demos` proves
    matches the demo scripts key-for-key) and in the demo scripts themselves. So the SHIPPED corridors
    must be read from the demos; see audit_demos(). Related bug found the same day: `skill_lib run
    <skill>` renders the SOLO launch with no GHOST_LUT_JSON and no GHOST_RESIDUAL for those four --
    it silently starts a Shadowing checkpoint with its corridor switched OFF. verify-demos never
    caught it because it only verifies SEQUENCES, never solo runs.
    """
    for block, key in (("deploy", "primary_env"), ("train", "recipe_env")):
        env = (m.get(block) or {}).get(key) or {}
        if "GHOST_RESIDUAL" in env:
            return float(env["GHOST_RESIDUAL"]), block
    return None, None


def audit_demos(dq, rec):
    """Grade the SHIPPED demo scripts -- the verified ground truth for what each skill actually runs."""
    import glob
    import re
    rows = []
    pats = [os.path.join(_RT, "projects", "policies", "demos", "*.sh"),
            os.path.join(_RT, "projects", "policies", "worlds", "run_*.sh")]
    for f in sorted(sum((glob.glob(p) for p in pats), [])):
        txt = open(f, encoding="utf-8", errors="replace").read()
        m = re.search(r"\bGHOST_RESIDUAL=([0-9.]+)", txt)
        if m:
            c = float(m.group(1))
            rows.append((os.path.basename(f), c, 100.0 * c / dq))
    print("\nSHIPPED DEMOS (the ground truth -- verify-demos proves these == the manifests)\n")
    print("%-34s %9s %8s  %s" % ("demo", "CORRIDOR", "% of CoP", "VERDICT"))
    print("-" * 90)
    nf = 0
    for name, c, pct in sorted(rows, key=lambda r: r[2]):
        if pct < 100:
            v = "FAIL  capped below its own ankle authority -- the crane pays the rest"
            nf += 1
        elif pct < 150:
            v = "WARN  reachable, no margin (recommend >= %.3f)" % rec
        else:
            v = "ok"
        print("%-34s %9.3f %7.0f%%  %s" % (name, c, pct, v))
    print("\n%d of %d shipped G1 Shadowing demos are BELOW full ankle-CoP authority." % (nf, len(rows)))
    return nf


def main():
    rows, skipped = [], []
    for root, _d, files in os.walk(SKILLS_DIR):
        if "skill.json" not in files:
            continue
        m = json.load(open(os.path.join(root, "skill.json")))
        name = m.get("name", os.path.basename(root))
        if m.get("method") != "shadowing":
            skipped.append((name, m.get("method", "?"), "not Shadowing -- no ghost corridor"))
            continue
        robot = (m.get("robots") or ["?"])[0]
        corr, src = _corridor_of(m)
        if corr is None:
            skipped.append((name, robot, "no GHOST_RESIDUAL in its manifest"))
            continue
        try:
            if RR.robot_class(robot) != RR.HUMANOID:
                skipped.append((name, robot, "quadruped -- point feet, no ankle CoP lever"))
                continue
            law = RR.corridor_min_rad(robot, kp=KP)
        except (RR.UnknownRobot, RR.ModelUnavailable) as e:
            skipped.append((name, robot, f"cannot measure foot: {str(e)[:44]}"))
            continue
        dq = law["dq_cop_pitch_rad"]
        rows.append((name, robot, corr, src, 100.0 * corr / dq, dq, law["recommended_rad"]))

    print("CORRIDOR-ADEQUACY AUDIT -- shipped GHOST_RESIDUAL vs the robot's own ankle-CoP authority")
    print("(kp=%.0f. dq_cop = m*g*(sole/2)/kp: below 100%% the policy cannot reach the edge of its foot)\n" % KP)
    print("%-18s %-5s %9s %8s   %s" % ("SKILL", "ROBOT", "CORRIDOR", "% of CoP", "VERDICT"))
    print("-" * 92)
    n_fail = 0
    for name, robot, corr, src, pct, dq, rec in sorted(rows, key=lambda r: r[4]):
        if pct < 100:
            v = "FAIL  structurally capped -- the crane pays the rest (need >=%.3f)" % dq
            n_fail += 1
        elif pct < 150:
            v = "WARN  reachable, no margin (recommend >=%.3f)" % rec
        else:
            v = "ok"
        print("%-18s %-5s %9.3f %7.0f%%   %s" % (name, robot, corr, pct, v))
    if skipped:
        print("\nnot graded:")
        for name, who, why in skipped:
            print("  %-18s %-5s  %s" % (name, who, why))
    print("\n%d of %d graded Shadowing skills are BELOW full ankle-CoP authority." % (n_fail, len(rows)))
    if n_fail:
        print("Every one of them is crane-dependent BY CONSTRUCTION, independent of how well it trained.")

    # The manifests are incomplete (only g1_walk declares a corridor), so grade the DEMOS too --
    # verify-demos proves those are what the manifests assemble.
    law_g1 = RR.corridor_min_rad("g1", kp=KP)
    n_fail += audit_demos(law_g1["dq_cop_pitch_rad"], law_g1["recommended_rad"])
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
