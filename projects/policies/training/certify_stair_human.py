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

"""HUMAN-CLIMB CERTIFICATE for the G1 stair demos -- the per-run verifier the owner asked for
("verify that the robot is climbing the stairs as someone actually would do it", 2026-07-10).

Reads one deploy log (the <tag>_mpc.txt written by run_walk_rl.sh deploy with FOOT_LOG=1) and
grades four gates; ALL must pass:

  CHEST-FORWARD  max |base yaw| < 15 deg while upright (z > 0.5 -- post-fall thrash excluded).
  PLACEMENT      mean planted-foot sink on treads > -8 mm, measured against the COLLIDER sole
                 (ankle frame - 36 mm; the 2026-07-10 foot-contact audit -- the visual mesh
                 overhangs the collider 22 mm at the toe, so "the foot looks sunk" is not this).
                 Worse than -8 mm mean = EDGE-RIDING (feet land over tread edges, mesh in riser).
  LATERAL        max |base y| < 0.35 m (no crabbing off the staircase line).
  SUMMIT+STAND   final 8 s: both feet within the landing span and base motionless (x-std < 5 mm,
                 z > 0.80).

Usage:
    python projects/policies/training/certify_stair_human.py <tag>_mpc.txt [x_start]
x_start = staircase front edge (demo3 world = 1.2 [default], walk3 = 0.6, runway = 4.0).
Geometry assumed: 5 steps, riser 0.03, going 0.26 (the shipped 3 cm demo family).
Exit code 0 = certificate PASS.

STATUS 2026-07-10: every current champion FAILS CHEST-FORWARD/LATERAL live. Root cause is
measured and policy-independent: the live plant applies a deterministic yaw moment (a pure
ghost-PD puppet with corridor ~0 veers -0.76 rad by x~1.0, bit-identical run-to-run) that the
batched trainer plant does not reproduce (batched full-episode drift 0.17). Policy-side
mitigation attempts and their outcomes are logged in docs/developer/train-deploy-gap.md;
the fix is live-vs-batched contact-generation parity (engine work).
"""
import re, numpy as np, sys

log = sys.argv[1]
X0 = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2   # staircase x_start
SOLE = 0.036   # ankle-frame -> collider sole (foot-contact audit; z_stance 0.047 is a target, not geometry)
rows = []; yaws = []
for ln in open(log):
    m = re.search(r"FOOTLOG t=(\d+) bx=([-\d.]+) bz=([-\d.]+) fLx=([-\d.]+) fLz=([-\d.]+) fRx=([-\d.]+) fRz=([-\d.]+)", ln)
    if m: rows.append([float(v) for v in m.groups()])
    m2 = re.search(r"walk-recipe deploy t=\d+ x=([-\d.]+) y=([-\d.]+) z=([-\d.]+) .* yaw=([-\d.]+)", ln)
    if m2:
        _x, _y, _z, _w = (float(m2.group(i)) for i in (1, 2, 3, 4))
        if _z > 0.5:   # upright only -- post-fall thrash is not gait yaw
            yaws.append((_y, _w))
if not rows:
    print("no FOOTLOG rows in %s (deploy with FOOT_LOG=1 FOOT_LOG_EVERY=6)" % log); sys.exit(2)
a = np.array(rows); t = a[:, 0] * 0.016

def surf(x):
    if x < X0: return 0.0
    return min(int((x - X0) / 0.26) + 1, 5) * 0.03

pens = []
for xi, zi in ((3, 4), (5, 6)):
    z = a[:, zi]; x = a[:, xi]; dz = np.abs(np.gradient(z)); pl = dz < 0.0008   # planted = z not moving
    pens += [z[i] - (surf(x[i]) + SOLE) for i in range(len(z)) if pl[i] and x[i] > X0]
pens = np.array(pens) if pens else np.array([0.0])
ymax = max(abs(y) for y, _ in yaws) if yaws else -1
yawmax = np.degrees(max(abs(w) for _, w in yaws)) if yaws else -1
last = a[t > t[-1] - 8.0]
feet_on = (X0 + 1.30 < last[:, 3].mean() < X0 + 2.80) and (X0 + 1.30 < last[:, 5].mean() < X0 + 2.80)
stand = last[:, 1].std() < 0.005 and last[:, 2].mean() > 0.80
print("CHEST-FORWARD: max |yaw| %.1f deg  %s" % (yawmax, "PASS" if 0 <= yawmax < 15 else "FAIL"))
print("PLACEMENT: tread sink mean %+.1f mm  p10 %+.1f mm  %s" % (1000 * pens.mean(), 1000 * np.percentile(pens, 10),
      "PASS" if pens.mean() > -0.008 else "FAIL (edge-riding)"))
print("LATERAL: max |y| %.2f  %s" % (ymax, "PASS" if 0 <= ymax < 0.35 else "FAIL"))
print("SUMMIT+STAND: feet %.2f/%.2f x-std %.1fmm z %.2f  %s" % (last[:, 3].mean(), last[:, 5].mean(),
      1000 * last[:, 1].std(), last[:, 2].mean(), "PASS" if (feet_on and stand) else "FAIL"))
allok = (0 <= yawmax < 15) and pens.mean() > -0.008 and (0 <= ymax < 0.35) and feet_on and stand
print("HUMAN-CLIMB CERTIFICATE:", "PASS" if allok else "FAIL")
sys.exit(0 if allok else 1)
