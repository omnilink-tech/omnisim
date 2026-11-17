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

"""GHOST SCREEN -- rank candidate motions (any BVH) for dynamic trainability on any robot.

The choreography-selection stage of the Shadowing pipeline (owner directive 2026-07-04:
"generalize for any ghost, any motion, any robot"). For each candidate BVH it runs the full
retarget (seq_ghost_retarget.py --audit: all gates REPORT, none refuse) with the standard
achievability alterations, then the GATE-1d balance audit, and ranks candidates by a
trainability score. The scoring is anchored to measured campaign outcomes on the G1:

  - salsa 60_03 (zmp-viol 62-76%, airborne 12%): core segments train to only 0.23-0.51
  - the walking regime (near-zero violations): trains to WBMATCH 0.91 at survival 1.0
  - segment-isolation survival rank-matched the per-16th COM margins (2026-07-04)

score = 100 - 0.6*zmp_viol% - 0.2*static_viol% - 1.0*airborne% - 40*max(0, vmax - envelope)
(higher = more of the motion lies inside the robot's dynamic envelope; style fidelity
[skelmatch] is reported separately -- a motion can be beautiful and untrainable).

Usage:
  python ghost_screen.py <out_dir> <a.bvh> [b.bvh ...] [--robot-spec specs/retarget_g1.json]
                         [--nb 512] [--stance-min 0.10] [--balance-min 0.02]
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.environ.get("OMNISIM_HOME", os.getcwd())


def run_one(bvh, out_dir, a):
    tag = os.path.splitext(os.path.basename(bvh))[0]
    lut = os.path.join(out_dir, "ghost_%s.json" % tag)
    cmd = [sys.executable, os.path.join(HERE, "seq_ghost_retarget.py"), bvh, lut,
           "--nb", str(a.nb), "--stance-min", str(a.stance_min),
           "--balance-min", str(a.balance_min), "--audit",
           "--robot-spec", a.robot_spec]
    r1 = subprocess.run(cmd, capture_output=True, text=True, cwd=RT)
    txt = r1.stdout + r1.stderr
    row = {"tag": tag, "emitted": os.path.exists(lut)}
    if not row["emitted"] and "GATE" not in txt:
        # candidate died before any gate: surface the real error, never a silent -999
        tail = " | ".join((r1.stderr or r1.stdout).strip().splitlines()[-3:])
        print("    !! %s crashed before the gates: %s" % (tag, tail), flush=True)
    for key, pat in (("skelmatch", r'"skelmatch_pct": ([0-9.]+)'),
                     ("foot_rms", r'"foot_rms_L": ([0-9.]+)'),
                     ("leg_viol", r'"leg_violation_pct": ([0-9.]+)'),
                     ("vmax", r'"vmax": ([0-9.]+)')):
        mm = re.search(pat, txt)
        row[key] = float(mm.group(1)) if mm else None
    row["gate_fails"] = len(re.findall(r"FAIL", txt))
    if row["emitted"]:
        r2 = subprocess.run([sys.executable, os.path.join(HERE, "ghost_balance_gate.py"), lut,
                             "--robot-spec", a.robot_spec],
                            capture_output=True, text=True, cwd=RT)
        for key, pat in (("static_viol", r"static margin:.*violated (\d+)%"),
                         ("zmp_viol", r"zmp margin:.*violated (\d+)%"),
                         ("airborne", r"no-contact: (\d+)%")):
            mm = re.search(pat, r2.stdout)
            row[key] = float(mm.group(1)) if mm else None
    spec = json.load(open(a.robot_spec))
    env = float(spec.get("root_speed_envelope", 0.75))
    if row.get("zmp_viol") is not None:
        row["score"] = round(100.0 - 0.6 * row["zmp_viol"] - 0.2 * (row.get("static_viol") or 0)
                             - 1.0 * (row.get("airborne") or 0)
                             - 40.0 * max(0.0, (row.get("vmax") or 0) - env), 1)
    else:
        row["score"] = -999.0
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir"); ap.add_argument("bvhs", nargs="+")
    ap.add_argument("--robot-spec", default=os.path.join(HERE, "specs", "retarget_g1.json"))
    ap.add_argument("--nb", type=int, default=512)
    ap.add_argument("--stance-min", type=float, default=0.10)
    ap.add_argument("--balance-min", type=float, default=0.02)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    rows = []
    for bvh in a.bvhs:
        print(">>> screening %s ..." % os.path.basename(bvh), flush=True)
        try:
            rows.append(run_one(bvh, a.out_dir, a))
        except Exception as e:
            rows.append({"tag": os.path.basename(bvh), "score": -999.0, "err": str(e)[:80]})
        print("    %s" % json.dumps(rows[-1]), flush=True)
    rows.sort(key=lambda r: -(r.get("score") or -999))
    print("\n=== GHOST SCREEN RANKING (robot: %s) ===" % os.path.basename(a.robot_spec))
    print("%-22s %6s %8s %8s %8s %6s %9s %5s" % ("candidate", "score", "zmpV%", "statV%", "airb%", "vmax", "skelmatch", "fails"))
    for r in rows:
        print("%-22s %6s %8s %8s %8s %6s %9s %5s" % (
            r["tag"], r.get("score"), r.get("zmp_viol"), r.get("static_viol"),
            r.get("airborne"), r.get("vmax"), r.get("skelmatch"), r.get("gate_fails")))


if __name__ == "__main__":
    main()
