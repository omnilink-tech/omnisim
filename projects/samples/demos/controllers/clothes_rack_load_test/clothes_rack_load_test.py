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

"""Pushes the clothes rack at its bar until it starts to tip, and says at what force.

Settle; then a horizontal force at the bar rising at RAMP_N_S, toward the back (+Y) and
then sideways (+X). The moment the feet on the far side rise LIFT_MM off the floor, or
the rack slides SLIDE_MM, the push stops and the force is recorded against the hand
calculation (load_test.py writes it into --test). Repeats, for anyone watching.

CLOTHES_RACK_REPORT=<path> writes the first round's result as JSON; CLOTHES_RACK_QUIT=1
then ends the simulation.
"""
import argparse
import json
import math
import os

from omnisim import Supervisor

RAMP_N_S = {"front": 2.0, "side": 4.0}
LIFT_MM = 1.0
SLIDE_MM = 5.0
SETTLE_S = 3.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", required=True)
    cfg = json.loads(ap.parse_args().test)
    sup = Supervisor()
    step = int(sup.getBasicTimeStep())
    dt = step / 1000.0
    rack = sup.getFromDef("RACK")
    arrow = sup.getFromDef("PUSH")
    a_tr, a_rot = arrow.getField("translation"), arrow.getField("rotation")

    def feet_z():
        p, R = rack.getPosition(), rack.getOrientation()
        return [p[2] + R[6] * fx + R[7] * fy for fx, fy in cfg["feet"]]

    def wait(seconds):
        end = sup.getTime() + seconds
        while sup.getTime() < end:
            if sup.step(step) == -1:
                raise SystemExit(0)

    def label(text):
        sup.setLabel(0, "Clothes rack on Newton: "
                     f"{cfg['mass_kg']:.1f} kg with {cfg['load_kg']:.0f} kg of clothes", 0.02, 0.02, 0.07, 0xFFFFFF, 0.0, "Arial")
        sup.setLabel(1, text, 0.02, 0.92, 0.08, 0xFFE9A8, 0.0, "Arial")

    def push(name, direction):
        """Ramp a push at the bar along `direction` until the far feet lift or it slides."""
        ux, uy = direction
        z0, p0 = feet_z(), rack.getPosition()
        far = [i for i, (fx, fy) in enumerate(cfg["feet"]) if fx * ux + fy * uy < 0]
        expect = cfg["expect"][f"{name}_n"]
        a_rot.setSFRotation([-uy, ux, 0.0, math.pi / 2])            # the arrow's +z along the push
        a_tr.setSFVec3f([-0.4 * ux, -0.4 * uy, cfg["push_z"]])
        force, result = 0.0, None
        while force < 3.0 * expect:
            force += RAMP_N_S[name] * dt
            rack.addForceWithOffset([force * ux, force * uy, 0.0], [0.0, 0.0, cfg["push_z"]], False)
            if sup.step(step) == -1:
                raise SystemExit(0)
            z = feet_z()
            lift = max((z[i] - z0[i]) * 1000.0 for i in far)
            p = rack.getPosition()
            slide = math.hypot(p[0] - p0[0], p[1] - p0[1]) * 1000.0
            if int(sup.getTime() / dt) % 25 == 0:
                label(f"Pushing the bar {'back' if name == 'front' else 'sideways'}: {force:5.1f} N "
                      f"(the hand calculation says it tips at {expect:.1f} N)")
            if lift >= LIFT_MM or slide >= SLIDE_MM:
                result = {"force_n": round(force, 2), "expected_n": expect,
                          "error_pct": round(100.0 * (force - expect) / expect, 1),
                          "mode": "tips" if lift >= LIFT_MM else "slides",
                          "far_feet_lift_mm": round(lift, 2), "slide_mm": round(slide, 2)}
                break
        a_tr.setSFVec3f([0.0, 0.0, -5.0])
        if result is None:
            result = {"force_n": None, "expected_n": expect, "mode": "held", "note": f"held {force:.0f} N"}
        label(f"{'Back' if name == 'front' else 'Sideways'}: {result['mode']} at {result['force_n']} N; "
              f"calculated {expect:.1f} N. Released.")
        print(f"[clothes_rack_load_test] {name}: {result}", flush=True)
        wait(4.0)
        back = max(abs(a - b) for a, b in zip(feet_z(), z0)) * 1000.0
        result["after_release_mm"] = round(back, 2)
        return result

    first = True
    while True:
        label("Settling on the four levelling feet")
        p0 = rack.getPosition()
        wait(SETTLE_S)
        p1 = rack.getPosition()
        z = feet_z()
        report = {"mass_kg": cfg["mass_kg"], "load_kg": cfg["load_kg"],
                  "settle": {"sink_mm": round((p0[2] - p1[2]) * 1000.0, 3),
                             "level_mm": round((max(z) - min(z)) * 1000.0, 3)},
                  "slide_n_calculated": cfg["expect"]["slide_n"]}
        report["front"] = push("front", (0.0, 1.0))
        report["side"] = push("side", (1.0, 0.0))
        print("[clothes_rack_load_test] RESULT " + json.dumps(report), flush=True)
        if first and os.environ.get("CLOTHES_RACK_REPORT"):
            with open(os.environ["CLOTHES_RACK_REPORT"], "w", encoding="utf-8") as fh:
                json.dump(report, fh, indent=2)
        if first and os.environ.get("CLOTHES_RACK_QUIT", "0") not in ("", "0"):
            sup.simulationQuit(0)
            return
        first = False
        label(f"Back: tips at {report['front']['force_n']} N (calc. {cfg['expect']['front_n']}). "
              f"Sideways: {report['side']['mode']} at {report['side']['force_n']} N (calc. {cfg['expect']['side_n']}).")
        wait(8.0)


if __name__ == "__main__":
    main()
