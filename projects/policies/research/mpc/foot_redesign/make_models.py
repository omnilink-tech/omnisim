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

"""Foot-redesign experiment — generate MODIFIED COPIES of the G1/H1 deploy MJCFs.

The ORIGINAL models under projects/robots/** are NEVER touched (other sessions and
demos depend on them, and we keep them as the A/B control). This script reads the
originals read-only and writes self-contained variants into ./models/.

The hypothesis under test (from the offline-MPC morphology finding): the G1 deterministic
stand/walk wall is the small foot + weak ankle (forward-pitch CoP moment ~Fz*toe_reach
is too small to arrest a single-support forward fall); H1 clears the SAGITTAL wall with a
long box foot but is lateral-marginal because its foot is NARROW (0.03 m) with no
ankle_roll DOF. So: give G1 a longer/wider foot (+ optionally a stronger ankle), and give
H1 a wider foot, then re-run the exact offline harness and measure.

MuJoCo box `size` is HALF-extents. Foot reach forward of the ankle = pos_x + size_x.

Originals:
  G1 foot  size="0.085 0.03 0.006" pos="0.035 0 -0.03"  -> 0.17 x 0.06 m, toe @ +0.120, heel @ -0.050
           ankle torque +-35 N*m, HAS ankle_roll
  H1 foot  size="0.14 0.015 0.012" pos="0.05 0 -0.05"    -> 0.28 x 0.03 m, toe @ +0.190, heel @ -0.090
           ankle torque +-40 N*m, NO ankle_roll (single pitch ankle)
"""
from __future__ import annotations
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "models"
OUT.mkdir(exist_ok=True)
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())

G1_LEGS = REPO / "projects/robots/unitree/g1/urdf/g1_legs_kp100.mjcf.xml"
G1_FULL = REPO / "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml"
H1_LEGS = REPO / "projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"

# --- original foot/ankle signatures (unique within each file) -------------------
G1_FOOT = 'size="0.085 0.03 0.006" pos="0.035 0 -0.03"'   # x2 (L/R)
G1_ANKTQ = 'actuatorfrcrange="-35 35"'                      # x4 (pitch+roll, L/R)
H1_FOOT = 'size="0.14 0.015 0.012" pos="0.05 0 -0.05"'     # x2 (L/R)


def g1_foot(half_x, half_y, pos_x, thick=0.006):
    return f'size="{half_x} {half_y} {thick}" pos="{pos_x} 0 -0.03"'


def h1_foot(half_x, half_y, pos_x, thick=0.012):
    return f'size="{half_x} {half_y} {thick}" pos="{pos_x} 0 -0.05"'


def write(name, src: Path, repls: list[tuple[str, str, int]]):
    """repls = [(old, new, expected_count)]; assert each count to catch silent misses."""
    txt = src.read_text(encoding="utf-8")
    for old, new, n in repls:
        c = txt.count(old)
        assert c == n, f"{name}: expected {n}x {old!r}, found {c}"
        txt = txt.replace(old, new)
    (OUT / name).write_text(txt, encoding="utf-8")
    print(f"  wrote {name}")


print("G1 variants (base: g1_legs_kp100):")
# control copy
write("g1_orig_legs.mjcf.xml", G1_LEGS, [])
# LONGFOOT: extend the TOE forward to +0.18 (= H1 reach), keep width 0.06, keep ankle +-35.
#   heel stays -0.05; center=(-.05+.18)/2=0.065, half=(.18+.05)/2=0.115
write("g1_longfoot_legs.mjcf.xml", G1_LEGS,
      [(G1_FOOT, g1_foot(0.115, 0.03, 0.065), 2)])
# LONGFOOT + STRONG ankle (+-70) so the bigger CoP geometry isn't torque-capped.
write("g1_longfoot_strong_legs.mjcf.xml", G1_LEGS,
      [(G1_FOOT, g1_foot(0.115, 0.03, 0.065), 2),
       (G1_ANKTQ, 'actuatorfrcrange="-70 70"', 4)])
# BIGFOOT: long (toe +0.19, heel -0.07) + WIDE (0.09) + strong ankle (+-88 = knee level).
#   center=(-.07+.19)/2=0.06, half=(.19+.07)/2=0.13
write("g1_bigfoot_legs.mjcf.xml", G1_LEGS,
      [(G1_FOOT, g1_foot(0.13, 0.045, 0.06, thick=0.008), 2),
       (G1_ANKTQ, 'actuatorfrcrange="-88 88"', 4)])
# full-body twins (for --full / arm-CAM runs)
write("g1_orig_full.mjcf.xml", G1_FULL, [])
write("g1_bigfoot_full.mjcf.xml", G1_FULL,
      [(G1_FOOT, g1_foot(0.13, 0.045, 0.06, thick=0.008), 2),
       (G1_ANKTQ, 'actuatorfrcrange="-88 88"', 4)])

print("H1 variants (base: h1_legs_newton):")
write("h1_orig.mjcf.xml", H1_LEGS, [])
# WIDEFOOT: widen 0.03 -> 0.12 m (half 0.015 -> 0.06), keep the long 0.28 m sole.
write("h1_widefoot.mjcf.xml", H1_LEGS,
      [(H1_FOOT, h1_foot(0.14, 0.06, 0.05), 2)])
# WIDEFOOT-XL: widen to 0.16 m (half 0.08) for a bigger lateral base.
write("h1_widefoot_xl.mjcf.xml", H1_LEGS,
      [(H1_FOOT, h1_foot(0.14, 0.08, 0.05), 2)])

print("done.")
