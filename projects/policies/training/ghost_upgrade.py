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

"""ghost_upgrade.py -- lift a ghost lut to the SCHEMA every validator needs.

THE FORMAT IS THE BLOCKER, NOT THE TOOLING
Our luts vary in three independent ways: whole-body pose or leg-only, base trajectory present or absent,
and a contact schedule that is never declared at all. That is exactly why five builders grew five
incompatible feasibility notions -- no single validator could see enough to judge any two of them the
same way. Unify the format and one certificate covers walk, turn, stairs, crawl and carry.

SCHEMA v2 -- what a ghost must declare
    wb_lut / wb_joints    the WHOLE-BODY pose. Dynamics needs every link's mass, so a leg-only lut
                          cannot be judged; the arms move the COM.
    root_lut              the base trajectory (x, y, z, yaw). Not optional and not invented: for cyclic
                          ghosts it is RECOVERED from the joints + contact schedule (ghost_root.py),
                          because a planted contact is stationary in the world.
    contact_schedule      which contacts are planted, per frame. The one thing no lut ever said, and the
                          thing every feasibility question starts from.
    ghost_schema = 2

This upgrader never invents. It derives:
  legs      leg_lut -> wb slots 0..11 (same slot order the trainer reads: knees at 3 and 9)
  arms      arm_lut (shoulder pitch) + elbow_lut -> wb slots 13/16 and 18/21; shoulder roll from the
            `shroll` scalar, mirrored L/R as the builder's own CARRY_ARM convention does
  base      integrated from contact stationarity, then dropped so the lowest planted contact touches
  contacts  inferred from which contacts sit lowest (needs the base ATTITUDE only, never its position)

Usage:  python ghost_upgrade.py <in_lut.json> <out_lut.json> [--contacts feet|crawl|spec.json]
"""
import argparse
import json
import os
import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ghost_contacts as GC
import ghost_root as GR
from build_step_turn_ghost import WB_JOINTS   # one source of truth for slot order

RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")


def build_wb(lut):
    """leg_lut (+ arm/elbow/shroll) -> the 23-slot whole-body pose the dynamics needs."""
    if "wb_lut" in lut:
        return np.asarray(lut["wb_lut"], float), False
    leg = np.asarray(lut["leg_lut"], float)
    NB = leg.shape[0]
    wb = np.zeros((NB, len(WB_JOINTS)))
    wb[:, 0:12] = leg                                   # slots 0-11: L leg then R leg
    wb[:, 12] = 0.0                                     # waist_yaw
    arm = np.asarray(lut["arm_lut"], float) if "arm_lut" in lut else np.zeros((NB, 2))
    elb = np.asarray(lut["elbow_lut"], float) if "elbow_lut" in lut else np.zeros((NB, 2))
    shr = float(lut.get("shroll", 0.0))
    wb[:, 13] = arm[:, 0]; wb[:, 14] = shr;  wb[:, 16] = elb[:, 0]     # left shoulder pitch/roll, elbow
    wb[:, 18] = arm[:, 1]; wb[:, 19] = -shr; wb[:, 21] = elb[:, 1]     # right (roll mirrored)
    return wb, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--contacts", default="feet")
    ap.add_argument("--yaw", type=float, default=0.0, help="constant base yaw (straight-line ghosts)")
    a = ap.parse_args()

    lut = json.load(open(a.src))
    spec = GC.load_spec(a.contacts)
    wb, synthesized_wb = build_wb(lut)
    WBJ = lut.get("wb_joints", WB_JOINTS)

    m = mj.MjModel.from_xml_path(MJCF)
    m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT) | int(mj.mjtDisableBit.mjDSBL_CONSTRAINT)
    d = mj.MjData(m)

    had_root = "root_lut" in lut
    if had_root:
        root = np.asarray(lut["root_lut"], float)
        p_bc = GR.base_frame_contacts(m, d, wb, WBJ, spec)
        R_wb = [GR._R(0, 0, float(root[i, 3])) for i in range(wb.shape[0])]
        sched = GR.infer_schedule(p_bc, R_wb, [c["name"] for c in spec["contacts"]],
                                  float(spec.get("swing_tol", 0.012)))
    else:
        root, sched = GR.reconstruct(lut, m, d, spec, wb, WBJ, a.yaw)

    out = dict(lut)
    out["wb_lut"] = [[float(v) for v in r] for r in wb]
    out["wb_joints"] = WBJ
    out["root_lut"] = [[float(v) for v in r] for r in root]
    out["contact_schedule"] = sched
    out["contact_spec"] = a.contacts
    out["ghost_schema"] = 2
    out["source"] = lut.get("source", "") + " | ghost_upgrade.py -> schema 2"
    json.dump(out, open(a.dst, "w"))

    from collections import Counter
    hist = Counter("+".join(s) or "flight" for s in sched)
    disp = float(np.hypot(root[-1, 0] - root[0, 0], root[-1, 1] - root[0, 1]))
    vx_lut = float(lut.get("vx", 0.0))
    cyc = float(lut.get("cycle_s", 1.0))
    print("GHOST UPGRADE: %s -> %s (schema 2)" % (os.path.basename(a.src), os.path.basename(a.dst)))
    print("  wb_lut     %s (%d joints)" % ("SYNTHESIZED from leg/arm/elbow" if synthesized_wb else "already present", wb.shape[1]))
    print("  root_lut   %s" % ("already present" if had_root else "RECOVERED from contact stationarity"))
    if not had_root:
        print("             base travel %.3f m over %.2f s -> %.3f m/s   (lut declares vx=%.3f)"
              % (disp, cyc, disp / cyc if cyc else 0.0, vx_lut))
        print("             ^ an INDEPENDENT check: the recovered base speed should match the lut's own vx,")
        print("               because nothing in the reconstruction ever looked at vx.")
    print("  base z     %.3f .. %.3f m" % (root[:, 2].min(), root[:, 2].max()))
    print("  contacts   %s" % dict(hist.most_common()))


if __name__ == "__main__":
    main()
