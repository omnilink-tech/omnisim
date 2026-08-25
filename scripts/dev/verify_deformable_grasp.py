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
"""Join a deformable-grasp controller log against the engine's particle
telemetry and decide, numerically, whether the deformable was actually GRIPPED.

WHY THIS EXISTS
---------------
Under `WorldInfo.newtonSolver "vbd"` there is no mj_model, so there is no
contact readback at all -- `getContactPoints`, `GET /sim/contacts` and
`GET /sim/grips` are empty. And a `Cloth` / `SoftBody` is not a scene node with
a readable pose: its `translation` is never written back, so a supervisor
polling it sees the authored value for ever whether the particles are
simulating perfectly or the node is completely inert.

That leaves exactly two observations, written by two different processes to two
different files:

  * the CONTROLLER log -- measured rigid pad poses, from `body_q`; and
  * OMNISIM_CLOTH_TELEMETRY -- the engine's own per-grid particle centroid,
    bbox, `soft_contacts` and `nonfinite` count.

Neither alone can witness a grasp. The pad log shows a gripper moving whether
or not it is holding anything; the telemetry shows a deformable moving whether
it is gripped or merely falling. **The grasp is the CORRELATION between them**,
and joining the two is the only proof available on this solver path.

No such join existed in this tree. The shipped t-shirt grasp demo says so in
its own docstring, and its published tracking numbers are consequently not
reproducible from committed tooling. This script closes that gap and is
deliberately written to serve any deformable-grasp demo, not just one.

WHAT IT CHECKS
--------------
  1. TRACKING   -- during the carry phases, does the deformable's centroid move
                   WITH the pad centre? Reported as the peak drift of
                   (centroid - pad_centre) from its value at the start of the
                   lift. A held object holds that offset; a dropped one does
                   not.
  2. LIFT       -- did the deformable actually leave its start height, by more
                   than a threshold? A grasp that never lifts is a touch.
  3. DEFORMATION-- did the pinch axis measurably compress? A rigid-looking
                   deformable means the elasticity is not being exercised.
  4. INTEGRITY  -- did the particle cloud stay a body? A collapse to a point
                   (bbox -> 0) or any `nonfinite` particle is a solver failure,
                   and it is easy to miss because the run still exits 0 and the
                   centroid stays finite and plausible.

⚠ THE JOIN IS BY SEQUENCE, NOT BY TIME. The telemetry records `step`, the
controller records sim `t`; they are written by different processes at
different cadences and there is no shared clock in either file. This script
therefore resamples both onto a common normalised progress axis. That is good
enough to answer "did these two move together", which is a shape question, and
it is NOT good enough for anything phase-exact. Do not use it to attribute an
event to a specific step.

USAGE
    python scripts/dev/verify_deformable_grasp.py \
        --pad-log .build_tmp/sponge_pads.jsonl \
        --telemetry .build_tmp/sponge.jsonl

Exit code 0 if the grasp is confirmed, 1 if not. `--json` for machine output.
"""

import argparse
import json
import sys


def read_jsonl(path):
    rows = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def resample(seq, n):
    """Nearest-neighbour resample of `seq` onto n evenly spaced positions."""
    if not seq:
        return []
    if len(seq) == 1:
        return [seq[0]] * n
    out = []
    for i in range(n):
        f = i / float(n - 1) if n > 1 else 0.0
        out.append(seq[min(len(seq) - 1, int(round(f * (len(seq) - 1))))])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pad-log", required=True,
                    help="controller JSONL with per-sample pad_c and phase")
    ap.add_argument("--telemetry", required=True,
                    help="OMNISIM_CLOTH_TELEMETRY JSONL")
    ap.add_argument("--grid", type=int, default=0,
                    help="which soft/cloth grid, when a world has several")
    ap.add_argument("--carry-phases", default="lift,traverse,press",
                    help="comma-separated controller phases that constitute the carry")
    ap.add_argument("--min-lift", type=float, default=0.03,
                    help="metres the deformable must rise to count as lifted")
    ap.add_argument("--max-drift", type=float, default=0.05,
                    help="metres of pad-relative drift allowed during the carry")
    ap.add_argument("--min-compression", type=float, default=0.002,
                    help="metres the pinch axis must compress to count as deformed")
    ap.add_argument("--pinch-axis", type=int, default=1,
                    help="0=x 1=y 2=z, the axis the jaws close along")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args(argv)

    pad_rows = [r for r in read_jsonl(args.pad_log) if r.get("kind") == "s"]
    tel_rows = read_jsonl(args.telemetry)
    if args.grid:
        tel_rows = [r for r in tel_rows if r.get("grid", 0) == args.grid]
    if not pad_rows:
        print("verify: no sample rows in %s" % args.pad_log, file=sys.stderr)
        return 2
    if not tel_rows:
        print("verify: no telemetry rows in %s -- was OMNISIM_CLOTH_TELEMETRY set?"
              % args.telemetry, file=sys.stderr)
        return 2

    carry = set(p.strip() for p in args.carry_phases.split(",") if p.strip())
    ax = args.pinch_axis

    # --- integrity, read straight off the telemetry ------------------------
    widths, nonfinite = [], 0
    for r in tel_rows:
        bmin, bmax = r.get("bbox_min"), r.get("bbox_max")
        if bmin and bmax:
            widths.append(max(bmax[i] - bmin[i] for i in range(3)))
        nonfinite += int(r.get("nonfinite", 0) or 0)
    rest_extent = widths[0] if widths else 0.0
    min_extent = min(widths) if widths else 0.0
    collapsed = bool(widths) and min_extent < 0.2 * rest_extent

    # --- deformation along the pinch axis ----------------------------------
    pinch = [r["bbox_max"][ax] - r["bbox_min"][ax]
             for r in tel_rows if r.get("bbox_min") and r.get("bbox_max")]
    pinch_rest = pinch[0] if pinch else 0.0
    pinch_min = min(pinch) if pinch else 0.0
    compression = pinch_rest - pinch_min

    # --- lift ---------------------------------------------------------------
    cz = [r["centroid"][2] for r in tel_rows if r.get("centroid")]
    lift = (max(cz) - cz[0]) if cz else 0.0

    # --- tracking: the correlation that is the actual proof ----------------
    # Resample the deformable centroid onto the pad samples, then look only at
    # the carry phases and ask whether the pad->centroid offset held constant.
    tel_c = [r["centroid"] for r in tel_rows if r.get("centroid")]
    tel_on_pad = resample(tel_c, len(pad_rows))

    drift_max, offset0, n_carry = 0.0, None, 0
    for row, cen in zip(pad_rows, tel_on_pad):
        if row.get("phase") not in carry:
            continue
        pad = row.get("pad_c")
        if not pad:
            continue
        off = [cen[i] - pad[i] for i in range(3)]
        if offset0 is None:
            offset0 = off
            continue
        n_carry += 1
        d = max(abs(off[i] - offset0[i]) for i in range(3))
        drift_max = max(drift_max, d)

    tracked = n_carry > 0 and drift_max <= args.max_drift
    lifted = lift >= args.min_lift
    deformed = compression >= args.min_compression
    intact = (not collapsed) and nonfinite == 0
    grasped = tracked and lifted and deformed and intact

    result = {
        "grasped": grasped,
        "tracked": tracked, "lifted": lifted,
        "deformed": deformed, "intact": intact,
        "carry_drift_max_m": round(drift_max, 6),
        "carry_samples": n_carry,
        "lift_m": round(lift, 6),
        "pinch_rest_m": round(pinch_rest, 6),
        "pinch_min_m": round(pinch_min, 6),
        "compression_m": round(compression, 6),
        "extent_rest_m": round(rest_extent, 6),
        "extent_min_m": round(min_extent, 6),
        "collapsed": collapsed,
        "nonfinite_particles": nonfinite,
        "pad_samples": len(pad_rows),
        "telemetry_samples": len(tel_rows),
    }

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        def mark(b):
            return "PASS" if b else "FAIL"
        print("deformable-grasp verification")
        print("  pad samples %d   telemetry samples %d"
              % (len(pad_rows), len(tel_rows)))
        print("  [%s] tracked    carry drift %.4f m over %d samples (limit %.3f)"
              % (mark(tracked), drift_max, n_carry, args.max_drift))
        print("  [%s] lifted     centroid rose %.4f m (min %.3f)"
              % (mark(lifted), lift, args.min_lift))
        print("  [%s] deformed   pinch axis %.4f -> %.4f m, compression %.4f (min %.3f)"
              % (mark(deformed), pinch_rest, pinch_min, compression,
                 args.min_compression))
        print("  [%s] intact     max extent %.4f -> %.4f m, %d nonfinite%s"
              % (mark(intact), rest_extent, min_extent, nonfinite,
                 "  <-- COLLAPSED TO A POINT" if collapsed else ""))
        print("  VERDICT: %s" % ("GRASPED" if grasped else "NOT GRASPED"))
    return 0 if grasped else 1


if __name__ == "__main__":
    sys.exit(main())
