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

"""Does a body-to-body contact between two aerial vehicles get reported,
and does it name both bodies?

Two phases, so the answer is not one lucky event:
  drop      drone_b released above drone_a; they must collide
  near_pass drone_b flown past drone_a at a commanded lateral offset,
            swept, to find the separation at which contact starts and stops
            being reported

Both drones are driven kinematically by the supervisor, which is what a
trajectory replay does -- the question is what the contact instrument says,
not whether a controller can fly.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260915\drones"
SEPARATIONS = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.14, 0.20]
BODY_HALF = 0.045          # the 90 mm frame
PASS_SPEED = 1.2           # m/s along x


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    a = robot.getFromDef("DRONE_A")
    b = robot.getFromDef("DRONE_B")
    if a is None or b is None:
        print("[drones] FATAL: DEFs not found", flush=True)
        sys.exit(1)
    fa = a.getField("translation")
    fb = b.getField("translation")

    rec = {"basic_time_step_ms": dt_ms, "body_half_m": BODY_HALF,
           "phases": []}

    def contacts_now():
        """Contact pairs the supervisor can see this step, as names."""
        out = []
        for node, label in ((a, "DRONE_A"), (b, "DRONE_B")):
            try:
                pts = node.getContactPoints(True)
            except TypeError:
                pts = node.getContactPoints()
            except Exception:
                pts = []
            for p in (pts or []):
                nm = None
                for attr in ("node_name", "name"):
                    nm = getattr(p, attr, None)
                    if nm:
                        break
                out.append({"on": label, "other": nm,
                            "point": list(getattr(p, "point", []) or [])})
        return out

    # ---- phase 1: an unambiguous collision -------------------------------
    fa.setSFVec3f([0.0, 0.0, 0.30])
    fb.setSFVec3f([0.0, 0.0, 0.60])
    robot.step(dt_ms)
    seen = []
    t = 0.0
    while robot.step(dt_ms) != -1 and t < 3.0:
        t += dt
        c = contacts_now()
        if c:
            seen.append({"t_s": round(t, 3), "pairs": c,
                         "a_z": round(a.getPosition()[2], 5),
                         "b_z": round(b.getPosition()[2], 5)})
            if len(seen) >= 4:
                break
    rec["phases"].append({
        "phase": "drop", "contact_reported": bool(seen),
        "first_events": seen[:4],
        "a_final": [round(v, 5) for v in a.getPosition()],
        "b_final": [round(v, 5) for v in b.getPosition()],
    })
    print(f"[drones] drop: contact_reported={bool(seen)} "
          f"n={len(seen)}", flush=True)

    # ---- phase 2: lateral near-pass sweep --------------------------------
    for sep in SEPARATIONS:
        fa.setSFVec3f([0.0, 0.0, 0.60])
        fb.setSFVec3f([-0.80, sep, 0.60])
        for _ in range(5):
            robot.step(dt_ms)
        hits = []
        min_gap = 9.9
        x = -0.80
        t = 0.0
        while robot.step(dt_ms) != -1 and x < 0.80:
            t += dt
            x += PASS_SPEED * dt
            # both held at altitude and driven kinematically
            fa.setSFVec3f([0.0, 0.0, 0.60])
            fb.setSFVec3f([x, sep, 0.60])
            pa, pb = a.getPosition(), b.getPosition()
            gap = math.dist(pa, pb)
            min_gap = min(min_gap, gap)
            c = contacts_now()
            if c:
                hits.append({"t_s": round(t, 3), "x": round(x, 4),
                             "gap_m": round(gap, 5), "pairs": c[:2]})
        # surface-to-surface separation along y, for the two 90 mm frames
        surf = sep - 2 * BODY_HALF
        rec["phases"].append({
            "phase": f"near_pass_sep{sep:.2f}",
            "commanded_centre_offset_m": sep,
            "surface_separation_m": round(surf, 5),
            "min_centre_distance_m": round(min_gap, 5),
            "contact_reported": bool(hits),
            "n_contact_steps": len(hits),
            "first_hit": hits[0] if hits else None,
        })
        print(f"[drones] near_pass sep={sep:.2f} surface={surf:+.3f} "
              f"min_gap={min_gap:.4f} contact={bool(hits)} "
              f"steps={len(hits)}", flush=True)

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "two_drone_contacts.json"), "w") as f:
        json.dump(rec, f, indent=1)
    print("[drones] wrote two_drone_contacts.json", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "tb.txt"), "a") as f:
            f.write(tb)
        print("[drones] FAILED " + tb, flush=True)
        raise
