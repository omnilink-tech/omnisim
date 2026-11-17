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

"""Build the Go2 STAND ghost -- the second specialist a quadruped BATON sequence needs.

A ghost is "the achievable reference a policy tracks". For a STAND on a statically stable body
that reference is simply THE NOMINAL STANCE, held: a constant, phase-independent leg track, zero
forward speed, zero feedforward. It is achievable by construction -- it is the pose the robot
settles into under its own PD before the gait clock ever starts, and the Go2's own gait model
(go2_trot_gait.standing_pose) is where that pose is defined. No training, no recording.

⭐ AND THAT IS THE POINT. A quadruped standing needs NO LEARNED POLICY: four feet on the ground
is statically stable, so the specialist that holds it is a DETERMINISTIC HOLD -- a BATON
specialist with `policy=None`, contributing a zero residual and tracking this ghost exactly.
A biped cannot do that (the G1's `stand` is a trained specialist riding a weight-bearing crane),
which is exactly the kind of difference a policy-switching LIBRARY must not care about.

Shape parity with the walk ghost is REQUIRED, because BATON blends the two element-wise:
same robot, same joint order, same nb. Only the content differs (constant vs cyclic).

    python projects/policies/research/shadowing/build_go2_stand_ghost.py
    python projects/policies/training/ghost_validator.py projects/policies/ghosts/go2/go2_stand_ghost_lut.json --stamp
"""
from __future__ import annotations

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

from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402

WALK_LUT = Path(REPO) / "projects/policies/ghosts/go2/go2_shadow_ghost_lut.json"
OUT = Path(REPO) / "projects/policies/ghosts/go2/go2_stand_ghost_lut.json"


def main() -> int:
    walk = json.loads(WALK_LUT.read_text())
    nb = int(walk["nb"])                       # SAME bin count -- BATON blends element-wise
    joints = list(walk["joints"])              # SAME real joint names, SAME order
    body_h = float(walk.get("gait", {}).get("body_height", 0.30))

    gp = stg.GaitParams(vx=0.0, freq=float(walk["freq"]), duty=0.6,
                        step_height=0.05, body_height=body_h, x0=0.0, ramp_s=1.0)
    stance = np.asarray(stg.standing_pose(gp), dtype=np.float64)
    if stance.shape != (len(joints),):
        raise SystemExit(f"standing_pose is {stance.shape}, expected ({len(joints)},)")

    leg = np.tile(stance[None, :], (nb, 1))            # constant: the held pose
    ff = np.zeros((nb, len(joints)), dtype=np.float64)  # a hold has NO feedforward
    # (zeros, not "absent": BATON blends channels element-wise, so the walk ghost's
    #  feedforward must FADE OUT across the handover, not survive it as a stale table.)

    out = {
        "robot": "go2",
        "joints": joints,
        "joint_order": walk.get("joint_order"),
        "nb": nb,
        "freq": float(walk["freq"]),
        "vx": 0.0,
        "leg_lut": [[round(float(v), 6) for v in row] for row in leg],
        "ffdq_lut": [[round(float(v), 6) for v in row] for row in ff],
        "ffdq_kp": walk.get("ffdq_kp", 250.0),
        "source": ("constructed-feasible / achievable by construction: the Go2's own gait model "
                   "standing_pose(body_height=%.2f) held constant over %d bins, vx=0, zero "
                   "feedforward. This is the pose the robot settles into under its deploy PD "
                   "before the gait clock starts -- it is not designed, it is READ from the model "
                   "the deploy already uses. A quadruped is statically stable on it, so the "
                   "specialist that tracks it needs no learned policy (BATON policy=None, zero "
                   "residual)." % (body_h, nb)),
        "gait": {"vx": 0.0, "freq": float(walk["freq"]), "body_height": body_h},
        "provenance": {
            "kind": "constructed",
            "from": "projects/policies/control/gait/go2_trot_gait.py::standing_pose",
            "shape_parity_with": str(WALK_LUT.relative_to(REPO)).replace("\\", "/"),
            "body_height": body_h,
        },
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, separators=(", ", ": ")))
    print(f"wrote {OUT.relative_to(REPO)}")
    print(f"  nb={nb} joints={len(joints)} vx=0 freq={out['freq']}")
    print("  stance pose (rad): " + " ".join(f"{v:+.3f}" for v in stance))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
