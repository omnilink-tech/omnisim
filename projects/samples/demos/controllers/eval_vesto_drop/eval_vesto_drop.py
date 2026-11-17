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

"""Drop the wheel-legged biped with its legs held, and see where it rests.

The question is not whether it falls over -- with the legs pinned straight
it cannot -- but WHERE THE WHEEL ENDS UP. The wheel's rest height above the
floor is a direct readout of the collision geometry the solver actually
received, and it can be predicted from the URDF three different ways:

    0.081 m   the wheel's own radius, and what the README's 0.55 m
              standing height implies (base_link sits 0.4651 above the
              wheel axis; 0.4651 + 0.081 = 0.5461)
    0.020 m   half the collision cylinder's LENGTH -- what you get if that
              cylinder is lying flat, i.e. if its axis is not the spin axis
    0.101 m   the same flat cylinder after this engine's cylinder-to-capsule
              conversion, which applies whenever radius >= half-length

So the measurement distinguishes an authoring question from an engine
question, which a single number could not.

Leg joints are commanded to zero throughout; the wheels are left free.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260919",
                   "vesto_runs")

LEG_JOINTS = [
    "%s_%s_joint" % (side, stem)
    for side in ("left", "right")
    for stem in ("hip_pitch", "hip_roll", "thigh_rotate", "knee_pitch")
]
WHEEL_JOINTS = ["left_wheel_drive_joint", "right_wheel_drive_joint"]
TRACK_LINKS = ["base_link", "left_wheel_link", "right_wheel_link",
               "left_shank_link", "right_shank_link"]

SETTLE_TOL_M = 2e-5      # per-step motion of base_link counted as settled
SETTLE_STEPS = 60        # consecutive quiet steps required
RUN_S = 8.0


def arg(flag, default):
    a = sys.argv
    return a[a.index(flag) + 1] if flag in a else default


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0
    tag = arg("--arm", "unknown")
    mu = float(arg("--mu", "0"))

    # URDF links carry a `name`, not a DEF, so getFromDef finds nothing --
    # walk the tree and match the name field instead.
    def walk(node, out, depth=0):
        if node is None or depth > 14:
            return
        try:
            f = node.getField("name")
            nm = f.getSFString() if f is not None else None
        except Exception:
            nm = None
        if nm and nm not in out:
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
    nodes = {n: found.get(n) for n in TRACK_LINKS}
    # The URDF root link is the robot node itself and carries the robot's
    # `name`, not the link's, so the tree walk never finds "base_link".
    if nodes.get("base_link") is None:
        nodes["base_link"] = robot.getSelf()
    missing = [n for n, v in nodes.items() if v is None]

    motors, sensors = {}, {}
    for jn in LEG_JOINTS:
        m = robot.getDevice(jn + "_motor")
        s = robot.getDevice(jn + "_sensor")
        if m is None:
            print("[vesto] FATAL missing motor %s_motor" % jn, flush=True)
            sys.exit(1)
        if s is not None:
            s.enable(ms)
        motors[jn] = m
        sensors[jn] = s
    for jn in WHEEL_JOINTS:
        s = robot.getDevice(jn + "_sensor")
        if s is not None:
            s.enable(ms)
            sensors[jn] = s

    rec = {"arm": tag, "ground_mu": mu, "basic_time_step_ms": ms,
           "links_not_found": missing,
           "predictions_m": {"wheel_radius": 0.081,
                             "half_collision_length": 0.020,
                             "capsule_from_flat_cylinder": 0.101},
           "samples": []}

    def zof(name):
        n = nodes.get(name)
        return None if n is None else float(n.getPosition()[2])

    def pitch_deg():
        n = nodes.get("base_link")
        if n is None:
            return None
        o = n.getOrientation()
        # rotation of body +x out of the horizontal plane
        return math.degrees(math.asin(max(-1.0, min(1.0, -o[6]))))

    # A two-wheeled machine standing on two round wheels is an inverted
    # pendulum: with the legs pinned and no balance controller it MUST topple,
    # so there is no settled standing pose to measure. What is measurable is
    # the pose during the first contact, before the topple develops, and how
    # long that takes -- which is where ground friction shows up.
    t = 0.0
    prev_wz = None
    touchdown = None
    topple = None
    while robot.step(ms) != -1 and t < RUN_S:
        for jn, m in motors.items():
            m.setPosition(0.0)
        wz = zof("left_wheel_link")
        pd = pitch_deg()
        if touchdown is None and wz is not None and prev_wz is not None:
            if wz >= prev_wz - 1e-6:          # stopped descending
                touchdown = round(t, 4)
        prev_wz = wz
        if topple is None and pd is not None and abs(pd) > 20.0:
            topple = round(t, 4)
        if len(rec["samples"]) < 4000:
            rec["samples"].append({
                "t": round(t, 4),
                "z": {k: (None if zof(k) is None else round(zof(k), 6))
                      for k in TRACK_LINKS},
                "pitch_deg": None if pd is None else round(pd, 4),
            })
        t += dt

    # Wheel axis height during the stable window that follows touchdown and
    # precedes the topple -- this is the collision-geometry readout.
    win = []
    if touchdown is not None:
        lo, hi = touchdown + 0.10, touchdown + 0.50
        if topple is not None:
            hi = min(hi, topple - 0.02)
        win = [s for s in rec["samples"]
               if lo <= s["t"] <= hi and s["z"]["left_wheel_link"] is not None]
    rec["touchdown_s"] = touchdown
    rec["topple_s"] = topple
    rec["stable_window_s"] = [round(win[0]["t"], 4),
                              round(win[-1]["t"], 4)] if win else None
    if win:
        zs = [s["z"]["left_wheel_link"] for s in win]
        mean = sum(zs) / len(zs)
        rec["wheel_axis_height_stable_m"] = {
            "mean": round(mean, 6), "min": round(min(zs), 6),
            "max": round(max(zs), 6), "n": len(zs)}
        best = min(rec["predictions_m"].items(), key=lambda kv: abs(kv[1] - mean))
        rec["matches_prediction"] = {"name": best[0], "value": best[1],
                                     "residual_m": round(mean - best[1], 6)}
    settled_at = touchdown

    final = {k: zof(k) for k in TRACK_LINKS}
    joints_final = {}
    for jn in LEG_JOINTS + WHEEL_JOINTS:
        s = sensors.get(jn)
        if s is None:
            continue
        try:
            joints_final[jn] = round(float(s.getValue()), 6)
        except Exception:
            pass

    rec["settled_at_s"] = settled_at
    rec["final_z_m"] = {k: (None if v is None else round(v, 6))
                        for k, v in final.items()}
    rec["final_pitch_deg"] = (None if pitch_deg() is None
                              else round(pitch_deg(), 4))
    rec["final_joint_rad"] = joints_final
    wl, wr = final.get("left_wheel_link"), final.get("right_wheel_link")
    rec["wheel_axis_height_final_m"] = {
        "left": None if wl is None else round(wl, 6),
        "right": None if wr is None else round(wr, 6),
    }

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "%s.json" % tag), "w") as fh:
        json.dump(rec, fh, indent=1)

    print("[vesto] arm %s  mu %.2f" % (tag, mu), flush=True)
    print("[vesto]   settled at %s s" % settled_at, flush=True)
    print("[vesto]   base_link z %.6f   pitch %.4f deg"
          % (final.get("base_link") or -1, rec["final_pitch_deg"] or 0),
          flush=True)
    print("[vesto]   wheel axis height  L %.6f  R %.6f"
          % (wl or -1, wr or -1), flush=True)
    if "matches_prediction" in rec:
        print("[vesto]   nearest prediction: %s = %.3f  (residual %+.6f m)"
              % (rec["matches_prediction"]["name"],
                 rec["matches_prediction"]["value"],
                 rec["matches_prediction"]["residual_m"]), flush=True)
    worst = max((abs(v) for k, v in joints_final.items()
                 if k in LEG_JOINTS), default=0.0)
    print("[vesto]   worst held-joint deviation from 0: %.6f rad" % worst,
          flush=True)
    if missing:
        print("[vesto]   LINKS NOT FOUND: %s" % missing, flush=True)
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
        print("[vesto] FAILED " + tb, flush=True)
        raise
