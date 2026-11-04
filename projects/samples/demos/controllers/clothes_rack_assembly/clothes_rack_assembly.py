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

"""Plays a clothes-rack assembly plan (projects/manufacturing/products/clothes_rack/assembly.py).

Each part's Pose follows its keyframes: position and orientation eased between keys, an
arc of `lift` metres over each move, and screw turns about the part's own z. The camera
follows the plan's keys and then orbits the finished rack; between camera moves it is
left alone, so you can look around. The whole build loops after `hold_s`.
"""
import argparse
import json
import math
from pathlib import Path

from omnisim import Supervisor

ROOT = Path(__file__).resolve().parents[5]


def ease(s):
    return s * s * (3 - 2 * s)


def slerp(a, b, s):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = [-x for x in b], -dot
    if dot > 0.9995:
        q = [x + s * (y - x) for x, y in zip(a, b)]
    else:
        th = math.acos(dot)
        wa, wb = math.sin((1 - s) * th) / math.sin(th), math.sin(s * th) / math.sin(th)
        q = [wa * x + wb * y for x, y in zip(a, b)]
    n = math.sqrt(sum(x * x for x in q))
    return [x / n for x in q]


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def axis_angle(q):
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    return [x / s, y / s, z / s, 2 * math.acos(min(1.0, w))]


def pose_at(keys, t):
    """Position and quaternion of a part at time t."""
    if t <= keys[0][0]:
        k = keys[0]
        return k[1:4], k[4:8]
    for a, b in zip(keys, keys[1:]):
        if t < b[0]:
            s = ease((t - a[0]) / (b[0] - a[0]))
            p = [x + s * (y - x) for x, y in zip(a[1:4], b[1:4])]
            p[2] += b[8] * math.sin(math.pi * s)
            q = slerp(a[4:8], b[4:8], s)
            turns = a[9] + s * (b[9] - a[9])
            break
    else:
        b = keys[-1]
        p, q, turns = list(b[1:4]), b[4:8], b[9]
    if turns:
        h = math.pi * turns           # half the screw angle
        q = qmul(q, [math.cos(h), 0.0, 0.0, math.sin(h)])
    return p, q


def look(pos, target):
    """Viewpoint axis-angle: the camera looks along its +x with +z up."""
    f = [t - p for t, p in zip(target, pos)]
    n = math.sqrt(sum(v * v for v in f))
    f = [v / n for v in f]
    up = [0.0, 0.0, 1.0]
    left = [up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]]
    n = math.sqrt(sum(v * v for v in left))
    left = [v / n for v in left]
    u = [f[1] * left[2] - f[2] * left[1], f[2] * left[0] - f[0] * left[2], f[0] * left[1] - f[1] * left[0]]
    m = [[f[0], left[0], u[0]], [f[1], left[1], u[1]], [f[2], left[2], u[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    if ang < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    ax = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    n = math.sqrt(sum(v * v for v in ax))
    if n < 1e-9:              # 180 degrees: the axis from the diagonal
        ax = [math.sqrt(max(0.0, (m[i][i] + 1) / 2)) for i in range(3)]
        n = math.sqrt(sum(v * v for v in ax))
    return [v / n for v in ax] + [ang]


def camera_at(plan, t):
    keys = plan["camera"]
    if t >= keys[-1][0]:
        o, c = plan["orbit"], plan["orbit"]["centre"]
        last = keys[-1]                       # start where the last camera key left off
        a = math.atan2(last[2] - c[1], last[1] - c[0]) + 2 * math.pi * (t - last[0]) / o["period_s"]
        return [c[0] + o["radius"] * math.cos(a), c[1] + o["radius"] * math.sin(a), o["height"]], c, True
    for a, b in zip(keys, keys[1:]):
        if t < b[0]:
            if a[1:] == b[1:]:
                return None, None, False          # holding: leave the camera to the viewer
            s = ease((t - a[0]) / (b[0] - a[0]))
            mix = [x + s * (y - x) for x, y in zip(a[1:], b[1:])]
            return mix[:3], mix[3:], True
    return None, None, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    args = ap.parse_args()
    plan = json.loads((ROOT / args.plan).read_text(encoding="utf-8"))
    sup = Supervisor()
    step = int(sup.getBasicTimeStep())
    parts = {}
    for name, keys in plan["parts"].items():
        node = sup.getFromDef(f"P_{name.upper()}")
        if node is None:
            print(f"[clothes_rack_assembly] no P_{name.upper()} in the world", flush=True)
            continue
        parts[name] = (node.getField("translation"), node.getField("rotation"), keys, [None])
    view = sup.getFromDef("VIEW")
    vpos, vori = view.getField("position"), view.getField("orientation")
    total = plan["duration"] + plan["hold_s"]
    t0, caption = sup.getTime(), None
    print(f"[clothes_rack_assembly] {len(parts)} parts, {plan['duration']} s build, loops every {total} s", flush=True)
    while sup.step(step) != -1:
        t = sup.getTime() - t0
        if t > total:
            t0, t = sup.getTime(), 0.0
        for tr, rot, keys, last in parts.values():
            p, q = pose_at(keys, t)
            key = (round(p[0], 6), round(p[1], 6), round(p[2], 6), round(q[0], 6), round(q[1], 6), round(q[3], 6))
            if key != last[0]:                 # a still part sends nothing, so the window can hold its frames
                tr.setSFVec3f(p)
                rot.setSFRotation(axis_angle(q))
                last[0] = key
        pos, target, moving = camera_at(plan, t)
        if moving:
            vpos.setSFVec3f(pos)
            vori.setSFRotation(look(pos, target))
        text = next((c[2] for c in plan["captions"] if c[0] <= t < c[1]), plan["kit"])
        if text != caption:
            caption = text
            sup.setLabel(0, plan["title"], 0.02, 0.02, 0.07, 0xFFFFFF, 0.0, "Arial")
            sup.setLabel(1, text, 0.02, 0.92, 0.08, 0xFFE9A8, 0.0, "Arial")
            print(f"[clothes_rack_assembly] t={t:5.1f} s  {text}", flush=True)


if __name__ == "__main__":
    main()
