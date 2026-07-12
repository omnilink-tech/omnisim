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

"""Quad foot-redesign experiment -- generate MODIFIED COPIES of the quad deploy MJCFs.

The ORIGINAL models under projects/policies/research/training/mjcf/** are NEVER
touched (the GPU trainers, the deploy controllers and the A/B control all depend on
them). This reads the originals read-only and writes self-contained variants into
./models/.

THE HYPOTHESIS (the quad analog of the G1/H1 foot-redesign finding):
  every quad foot is a single POINT-CONTACT SPHERE (spot r=0.035, go2 0.022,
  b2 0.032). A point foot has ZERO contact patch, so it gives NO per-foot
  yaw/roll resistance -- on rough terrain the body cannot arrest the lateral/yaw
  kick from an asymmetric bump strike and DRIFTS SIDEWAYS (the exact terrain-demo
  failure; the residual-RL terrain policy drifts ~40 deg, the bare/MPC trot
  marches in place). Give each foot a real CONTACT PATCH (a flat box sole,
  wide in the lateral axis) and the deterministic MPC gains the lateral/yaw
  authority to keep going FORWARD -- mirroring the humanoid foot whose width
  bought the lateral CoP margin.

  We keep the BOTTOM of the new foot at exactly the same world z as the sphere's
  bottom (sphere bottom = pos_z - radius), so the standing settle, the leg IK
  (which targets the kinematic tip at -L2, independent of the geom) and the gait
  are byte-for-byte unchanged -- the ONLY thing that changes is the contact geom.
  Clean foot-isolation, same as the G1/H1 experiment.

MuJoCo box `size` is HALF-extents; the foot box is (half_x fore/aft, half_y lateral,
half_z thick). A SHORT fore/aft extent avoids edge-catch in the travel direction;
a WIDE lateral extent is the sideways-drift lever.
"""
from __future__ import annotations

from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "models"
OUT.mkdir(exist_ok=True)
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())
MJCF = REPO / "projects/policies/research/training/mjcf"

# Per-robot original point-foot signature (appears 4x, one per leg) and its
# sphere bottom = pos_z - radius. We size every box so its bottom matches.
ROBOTS = {
    # robot: (flat_src, rough_src_or_None, sphere_r, sphere_z)
    "spot": ("spot_newton_fixed2.xml", "spot_newton_fixed2_rough.xml", 0.035, -0.32),
    "go2":  ("go2_newton.xml", None, 0.022, -0.213),
    "b2":   ("b2_newton.xml", None, 0.032, -0.35),
}


def sphere_sig(r, z):
    return f'size="{r}" pos="0 0 {z}"'


def box_geom(half_x, half_y, half_z, bottom):
    """A flat box sole whose bottom face sits at world-z `bottom` (= sphere bottom)."""
    cz = round(bottom + half_z, 5)
    return f'type="box" size="{half_x} {half_y} {half_z}" pos="0 0 {cz}"'


def bigsphere(r_new, bottom):
    cz = round(bottom + r_new, 5)
    return f'size="{r_new}" pos="0 0 {cz}"'


def write(name, src: Path, old: str, new: str, n_expected: int):
    txt = src.read_text(encoding="utf-8")
    c = txt.count(old)
    assert c == n_expected, f"{name}: expected {n_expected}x {old!r}, found {c}"
    (OUT / name).write_text(txt.replace(old, new), encoding="utf-8")
    print(f"  wrote {name}  ({old!r} -> box/sphere)")


def gen(robot):
    flat_src, rough_src, r, z = ROBOTS[robot]
    bottom = round(z - r, 5)
    sig = sphere_sig(r, z)
    # Variants, scaled loosely to robot size. boxwide = the primary candidate
    # (short fore/aft, WIDE lateral -> the sideways-drift fix); box = square
    # patch; bigsphere = a control (bigger but still ~point contact).
    s = r / 0.035  # size scale relative to spot
    variants = {
        "boxwide":   box_geom(round(0.045 * s, 4), round(0.075 * s, 4), round(0.013 * s, 4), bottom),
        "box":       box_geom(round(0.060 * s, 4), round(0.060 * s, 4), round(0.013 * s, 4), bottom),
        "bigsphere": bigsphere(round(r * 1.7, 4), bottom),
    }
    print(f"{robot}: sphere r={r} bottom={bottom}")
    for vname, geom in variants.items():
        write(f"{robot}_{vname}_flat.xml", MJCF / flat_src, sig, geom, 4)
        if rough_src:
            write(f"{robot}_{vname}_rough.xml", MJCF / rough_src, sig, geom, 4)
    # an unmodified copy of the rough world too, so the A/B control reads from ./models/
    if rough_src:
        (OUT / f"{robot}_orig_rough.xml").write_text(
            (MJCF / rough_src).read_text(encoding="utf-8"), encoding="utf-8")
        print(f"  wrote {robot}_orig_rough.xml  (control)")


if __name__ == "__main__":
    import sys
    for rb in (sys.argv[1:] or ["spot"]):
        gen(rb)
    print("done.")
