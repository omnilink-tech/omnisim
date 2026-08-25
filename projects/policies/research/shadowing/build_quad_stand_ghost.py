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

"""Build a QUADRUPED STAND ghost -- the second specialist a quad BATON sequence needs.

ROBOT-GENERAL (--robot go2|omniquad|b2), generalized from build_go2_stand_ghost.py for the
"Shadowing is robot-general" campaign. The robot table is IMPORTED from
build_quad_shadow_ghost.py -- there is exactly ONE place a robot-specific constant lives,
and both builders read it, so the walk ghost and the stand ghost can never disagree about
a robot's joint names, kp or gait model.

A ghost is "the achievable reference a policy tracks". For a STAND on a statically stable body
that reference is simply THE NOMINAL STANCE, held: a constant, phase-independent leg track, zero
forward speed, zero feedforward. It is achievable by construction -- it is the pose the robot
settles into under its own PD before the gait clock ever starts, and the robot's OWN gait model
(<robot>_trot_gait.standing_pose) is where that pose is defined. No training, no recording.

⭐ AND THAT IS THE POINT. A quadruped standing needs NO LEARNED POLICY: four feet on the ground
is statically stable, so the specialist that holds it is a DETERMINISTIC HOLD -- a BATON
specialist with `policy=None`, contributing a zero residual and tracking this ghost exactly.
A biped cannot do that (the G1's `stand` is a trained specialist riding a weight-bearing crane),
which is exactly the kind of difference a policy-switching LIBRARY must not care about.

Shape parity with the walk ghost is REQUIRED, because BATON blends the two element-wise:
same robot, same joint order, same nb. Only the content differs (constant vs cyclic). This
builder therefore READS the walk ghost and copies nb / freq / joints / ffdq_kp from it --
it never re-derives them, so parity is structural, not a convention someone has to remember.

    python projects/policies/research/shadowing/build_quad_stand_ghost.py --robot go2
    python projects/policies/research/shadowing/build_quad_stand_ghost.py --robot omniquad \
        --walk-lut _scratch/omniquad_shadow_ghost_lut.json --out _scratch/omniquad_stand_ghost_lut.json
    python projects/policies/training/ghost_validator.py \
        projects/policies/ghosts/go2/go2_stand_ghost_lut.json --stamp
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = os.environ.get("OMNISIM_HOME") or str(
    next(p for p in Path(__file__).resolve().parents
         if (p / "AGENTS.md").exists() or (p / ".git").exists()))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

# THE robot table -- imported, never re-declared (the single-source-of-truth rule).
from projects.policies.research.shadowing.build_quad_shadow_ghost import (  # noqa: E402
    ROBOTS, Spec, repo_rel)

# Where each robot's ghosts live (the ghosts/<robot>/ store).
def _default_walk(robot: str) -> Path:
    return Path(REPO) / f"projects/policies/ghosts/{robot}/{robot}_shadow_ghost_lut.json"


def _default_out(robot: str) -> Path:
    return Path(REPO) / f"projects/policies/ghosts/{robot}/{robot}_stand_ghost_lut.json"


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Build a quadruped STAND ghost (a held standing_pose) with shape "
                    "parity to that robot's walk ghost.")
    ap.add_argument("--robot", choices=tuple(ROBOTS), default="go2")
    ap.add_argument("--walk-lut", default=None,
                    help="the walk ghost to take nb/freq/joints from "
                         "(default: projects/policies/ghosts/<robot>/<robot>_shadow_ghost_lut.json)")
    ap.add_argument("--out", default=None,
                    help="output lut json (default: "
                         "projects/policies/ghosts/<robot>/<robot>_stand_ghost_lut.json)")
    args = ap.parse_args()

    sp = Spec(args.robot)
    stg = sp.stg
    walk_lut = Path(args.walk_lut) if args.walk_lut else _default_walk(sp.robot)
    out_path = Path(args.out) if args.out else _default_out(sp.robot)
    if not walk_lut.exists():
        raise SystemExit(
            f"walk ghost not found: {walk_lut}\n"
            f"Build it first:  python projects/policies/research/shadowing/"
            f"build_quad_shadow_ghost.py --robot {sp.robot}\n"
            "(The stand ghost takes nb/freq/joints from the walk ghost -- BATON blends them "
            "element-wise, so shape parity is REQUIRED, not optional.)")

    walk = json.loads(walk_lut.read_text())
    got_robot = walk.get("robot")
    if got_robot and got_robot != sp.robot:
        raise SystemExit(f"walk lut {walk_lut} is for robot {got_robot!r}, not {sp.robot!r}")
    nb = int(walk["nb"])                       # SAME bin count -- BATON blends element-wise
    joints = list(walk["joints"])              # SAME real joint names, SAME order
    if joints != sp.joints:
        raise SystemExit(
            f"walk lut joint names disagree with the {sp.robot} table:\n"
            f"  lut   {joints}\n  table {sp.joints}\n"
            "(The WBMATCH name-trap rule: never fall back to positional matching.)")
    body_h = float(walk.get("gait", {}).get("body_height",
                                            stg.GaitParams().body_height))

    gp = stg.GaitParams(vx=0.0, freq=float(walk["freq"]), duty=0.6,
                        step_height=stg.GaitParams().step_height,
                        body_height=body_h, x0=0.0, ramp_s=1.0)
    stance = np.asarray(stg.standing_pose(gp), dtype=np.float64)
    if stance.shape != (len(joints),):
        raise SystemExit(f"standing_pose is {stance.shape}, expected ({len(joints)},)")
    over_lo = float((sp.lim_lo - stance).max())
    over_hi = float((stance - sp.lim_hi).max())
    if over_lo > 1e-6 or over_hi > 1e-6:
        raise SystemExit(
            f"[{sp.robot}] standing_pose violates the deploy joint limits "
            f"(under-lo {over_lo:+.4f}, over-hi {over_hi:+.4f} rad) -- a ghost that starts "
            "out of range is not achievable by construction.")

    leg = np.tile(stance[None, :], (nb, 1))            # constant: the held pose
    ff = np.zeros((nb, len(joints)), dtype=np.float64)  # a hold has NO feedforward
    # (zeros, not "absent": BATON blends channels element-wise, so the walk ghost's
    #  feedforward must FADE OUT across the handover, not survive it as a stale table.)

    out = {
        "robot": sp.robot,
        "joints": joints,
        "joint_order": walk.get("joint_order"),
        "nb": nb,
        "freq": float(walk["freq"]),
        "vx": 0.0,
        "leg_lut": [[round(float(v), 6) for v in row] for row in leg],
        "ffdq_lut": [[round(float(v), 6) for v in row] for row in ff],
        "ffdq_kp": walk.get("ffdq_kp", sp.kp),
        "source": ("constructed-feasible / achievable by construction: the %s's own gait model "
                   "standing_pose(body_height=%.2f) held constant over %d bins, vx=0, zero "
                   "feedforward. This is the pose the robot settles into under its deploy PD "
                   "before the gait clock starts -- it is not designed, it is READ from the model "
                   "the deploy already uses. A quadruped is statically stable on it, so the "
                   "specialist that tracks it needs no learned policy (BATON policy=None, zero "
                   "residual)." % (sp.display, body_h, nb)),
        "gait": {"vx": 0.0, "freq": float(walk["freq"]), "body_height": body_h},
        "provenance": {
            "kind": "constructed",
            "robot": sp.robot,
            "from": f"{sp.gait_module.replace('.', '/')}.py::standing_pose",
            "builder": "projects/policies/research/shadowing/build_quad_stand_ghost.py",
            "shape_parity_with": repo_rel(walk_lut),
            "body_height": body_h,
        },
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, separators=(", ", ": ")))
    print(f"wrote {repo_rel(out_path)}")
    print(f"  robot={sp.robot} nb={nb} joints={len(joints)} vx=0 freq={out['freq']} "
          f"kp={out['ffdq_kp']}")
    print("  stance pose (rad): " + " ".join(f"{v:+.3f}" for v in stance))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
