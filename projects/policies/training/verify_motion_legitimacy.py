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

"""MOTION LEGITIMACY VERIFIER -- does the robot achieve the motion HONESTLY?

Owner directive (2026-07-11): the chest-forward stair champion passed every kinematic gate
(heading, foot placement, summit stand) while climbing WRONG -- levering itself up on its
knees and hanging on the crane's attitude springs. Kinematic rulers cannot see that. This
verifier grades the DYNAMICS of a run and names, in numbers, how the motion is achieved:

  GATE L1  CRANE SUPPORT   The harness must not do the work. Reports per-phase mean/max of
                           |fy| (lateral catch), |fz| (vertical rope), |tx|,|ty| (attitude
                           springs -- 350 N*m caps can hold a leaning robot like a hand on
                           the torso), and the sustained-torque fraction: % of ticks with
                           |tx| or |ty| above TAU_SUS (default 40 N*m ~ what balancing
                           transients need). FAIL if sustained-fraction > 15% or mean
                           |fy| > 40 N.
  GATE L2  CONTACT PURITY  Only FEET may touch the world during locomotion. Census of every
                           robot-body contact (CONTACT_LOG_ALL=1 + CONTACTMAP names): knees,
                           shins, hands, pelvis touching terrain = illegitimate support.
                           FAIL if non-foot contact on > 2% of contact-logged ticks.
  GATE L3  FEET SUPPORT    The feet must be under load continuously: % of logged ticks with
                           at least one foot contact (penetration < 0), and the longest
                           airborne gap (both feet off) outside genuine flight phases.
                           FAIL if airborne > 25% of ticks or a gap > 0.5 s.
  GATE L4  KINEMATIC       The original human-climb gates (heading window, tread placement,
                           step sequence) -- run certify_stair_human.py separately.

Usage:
    python projects/policies/training/verify_motion_legitimacy.py <log> [x0] [x1]
where <log> is a deploy _mpc.txt or eval _rl.txt containing CRANELOG + CONTACTLOG(+ALL) +
CONTACTMAP lines, and [x0,x1] bound the locomotion phase in base-x (default 0.2..3.2:
launch-settle excluded, stand-catch excluded). Exit 0 = all legitimacy gates PASS.

Generalization: nothing here is stairs-specific. L1-L3 apply verbatim to walks, turns,
squats, crawls (with the allowed-contact set swapped, e.g. crawl allows knees+hands).
"""
import re
import sys

import numpy as np

TAU_SUS = 40.0     # N*m of attitude torque considered "supporting", not transient
FY_MEAN_MAX = 40.0  # N sustained lateral catch = the rope is steering/holding
SUS_FRAC_MAX = 0.15
NONFOOT_FRAC_MAX = 0.02
AIRBORNE_FRAC_MAX = 0.25
FOOT_BODIES = ("ankle_roll", "foot", "ankle")   # substrings that mark legitimate contact bodies
FOOT_GEOMS = None   # set via argv[4] "6,12" -- Newton renames bodies to body_N, names are useless
CAP_SAT = 340.0     # attitude torque within 10 N*m of the 350 cap = the crane at full strength
CAP_FRAC_MAX = 0.005

log = sys.argv[1]
X0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
X1 = float(sys.argv[3]) if len(sys.argv) > 3 else 3.2
if len(sys.argv) > 4:
    FOOT_GEOMS = {int(v) for v in sys.argv[4].split(",")}

gmap = {}
crane = []          # t, fx, fy, fz, tx, ty, tz, bx, bz
contacts = []       # t, ga, gb, dist, bx
for ln in open(log, encoding="utf-8", errors="replace"):
    m = re.search(r"CONTACTMAP g=(\d+) body=(\S+)", ln)
    if m:
        gmap[int(m.group(1))] = m.group(2)
        continue
    m = re.search(r"CRANELOG (?:side=\S+ )?t=(\d+) fx=([-\d.]+) fy=([-\d.]+) fz=([-\d.]+) "
                  r"tx=([-\d.]+) ty=([-\d.]+) tz=([-\d.]+) bx=([-\d.]+) bz=([-\d.]+)", ln)
    if m:
        crane.append([float(v) for v in m.groups()])
        continue
    m = re.search(r"CONTACTLOG side=\S+ t=(\d+) g=\((\d+),(\d+)\) d=([-\d.]+) p=\([^)]*\) "
                  r"n=\([^)]*\) bx=([-\d.]+)", ln)
    if m:
        contacts.append([float(v) for v in m.groups()])

if not crane and not contacts:
    print("no CRANELOG/CONTACTLOG lines -- run the deploy/eval with CRANE_LOG=1 CONTACT_LOG=1 CONTACT_LOG_ALL=1")
    sys.exit(2)

verdicts = []

# ---- GATE L0: did the run even stay up? (a fallen run floods every census) ------------
_flog = [ln for ln in open(log, encoding="utf-8", errors="replace") if "FOOTLOG t=" in ln]
if _flog:
    m = re.search(r"bz=([-\d.]+)", _flog[-1])
    if m and float(m.group(1)) < 0.5:
        print("L0 UPRIGHT: FAIL -- the run ENDED FALLEN (final base z %.2f). The gates below "
              "describe a fallen robot; fix survival before reading them as gait verdicts." % float(m.group(1)))
        verdicts.append(False)

# ---- GATE L1: crane support ----------------------------------------------------------
if crane:
    c = np.array(crane)
    win = c[(c[:, 7] >= X0) & (c[:, 7] <= X1)]
    if len(win):
        fy, fz, tx, ty = np.abs(win[:, 2]), np.abs(win[:, 3]), np.abs(win[:, 4]), np.abs(win[:, 5])
        sus = float(np.mean((tx > TAU_SUS) | (ty > TAU_SUS)))
        capf = float(np.mean((tx > CAP_SAT) | (ty > CAP_SAT)))
        ok = sus <= SUS_FRAC_MAX and float(fy.mean()) <= FY_MEAN_MAX and capf <= CAP_FRAC_MAX
        verdicts.append(ok)
        print("L1 CRANE SUPPORT: |fy| mean %5.1f N (max %6.1f)   |fz| mean %5.1f N   "
              "|tx| mean %5.1f N*m (max %6.1f)   |ty| mean %5.1f (max %6.1f)"
              % (fy.mean(), fy.max(), fz.mean(), tx.mean(), tx.max(), ty.mean(), ty.max()))
        print("   sustained attitude torque (>%g N*m): %4.1f%% of ticks   AT-CAP (>%g): %4.2f%% -> %s"
              % (TAU_SUS, 100 * sus, CAP_SAT, 100 * capf,
                 "PASS" if ok else "FAIL (the crane is holding the robot)"))
    else:
        print("L1 CRANE SUPPORT: no crane samples in x window"); verdicts.append(False)
else:
    print("L1 CRANE SUPPORT: no CRANELOG lines (add CRANE_LOG=1)"); verdicts.append(False)

# ---- GATE L2: contact purity ---------------------------------------------------------
def body_of(g):
    return gmap.get(int(g), "?")

def is_foot_geom(g):
    if FOOT_GEOMS is not None:
        return int(g) in FOOT_GEOMS
    return any(k in body_of(g).lower() for k in FOOT_BODIES)

def is_robot(name):
    # Newton renames: robot links = body_N; terrain/statics = static_body_N / world / floor.
    n = name.lower()
    return not (n.startswith("static_body") or n.startswith("stair")
                or n in ("floor", "world", "ground", "?") or "arena" in n)

if contacts and gmap:
    k = np.array(contacts)
    win = k[(k[:, 4] >= X0) & (k[:, 4] <= X1)]
    ticks = sorted(set(win[:, 0].astype(int))) if len(win) else []
    nonfoot_ticks = set()
    census = {}
    for row in win:
        for g in (row[1], row[2]):
            nm = body_of(g)
            if is_robot(nm) and not is_foot_geom(g):
                census[nm] = census.get(nm, 0) + 1
                nonfoot_ticks.add(int(row[0]))
    frac = len(nonfoot_ticks) / max(1, len(ticks))
    ok = frac <= NONFOOT_FRAC_MAX
    verdicts.append(ok)
    top = sorted(census.items(), key=lambda x: -x[1])[:6]
    print("L2 CONTACT PURITY: non-foot robot contact on %4.1f%% of ticks   offenders: %s   -> %s"
          % (100 * frac, ", ".join("%s x%d" % kv for kv in top) or "none",
             "PASS" if ok else "FAIL (climbing on non-foot bodies)"))
else:
    print("L2 CONTACT PURITY: need CONTACT_LOG_ALL=1 + CONTACTMAP lines"); verdicts.append(False)

# ---- GATE L3: feet under load --------------------------------------------------------
if contacts and gmap:
    k = np.array(contacts)
    win = k[(k[:, 4] >= X0) & (k[:, 4] <= X1)]
    ticks = sorted(set(win[:, 0].astype(int)))
    foot_ticks = {int(r[0]) for r in win if (is_foot_geom(r[1]) or is_foot_geom(r[2])) and r[3] < 0}
    if ticks:
        airborne = [t for t in ticks if t not in foot_ticks]
        frac_air = len(airborne) / len(ticks)
        # longest consecutive airborne gap in ticks (log cadence-spaced)
        gap = best = 0
        prev_air = False
        for t in ticks:
            if t in foot_ticks:
                prev_air = False; gap = 0
            else:
                gap = gap + 1 if prev_air else 1
                prev_air = True; best = max(best, gap)
        ok = frac_air <= AIRBORNE_FRAC_MAX
        verdicts.append(ok)
        print("L3 FEET SUPPORT: foot-loaded on %4.1f%% of ticks (airborne %4.1f%%, longest gap %d samples) -> %s"
              % (100 * (1 - frac_air), 100 * frac_air, best, "PASS" if ok else "FAIL (not self-supporting on feet)"))
    else:
        print("L3 FEET SUPPORT: no contact ticks in window"); verdicts.append(False)
else:
    verdicts.append(False)

allok = all(verdicts)
print("MOTION-LEGITIMACY:", "PASS" if allok else "FAIL")
sys.exit(0 if allok else 1)
