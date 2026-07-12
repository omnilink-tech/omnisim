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

"""Hill terrain for Shadowing hill-walking -- a SINGLE source of truth for the
up-over-down hill, shared by the ghost generator, the RL trainer, and the
OmniSim deploy/preview world.

The hill is a 1-D height profile h(x) (constant across y): a flat approach, a
constant-grade UP ramp, a short flat crest, a constant-grade DOWN ramp, then a
flat runout -- with ROUNDED transitions (filleted corners) so there is no sharp
concave kink at the ramp base (which trips a stepping foot) and the base-pitch
command the ghost tracks is continuous (a body cannot snap-pitch 15 deg in one
step). The rounded h(x) is realized as many short static BOX segments so the
geometry lives entirely in the XML -- every loader that does from_xml_path()
gets the hill with no post-load data step -- and mirrors exactly to OmniSim Box
solids for the preview/deploy world.

Because the body-frame trot foot targets rotate WITH the base, pitching the base
to the local surface angle makes the flat-ground gait land its feet on the
inclined surface -- so the ghost intent is just: advance along the surface at
vx, set base z = h(x) + ride*cos(pitch), base pitch = atan(h'(x)). See
generate_hill_walk.py.

Convention: walking in +x. Up ramp rises toward +x.

  python projects/policies/research/shadowing/hill_terrain.py     # self-test (box tops == h(x))
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _smoothstep(a):
    a = min(1.0, max(0.0, a))
    return a * a * (3.0 - 2.0 * a)


@dataclass
class HillProfile:
    """Up-over-down hill with filleted corners. x layout:
        [0, x_up)         flat approach (z=0)
        ~x_up             rounded base, ramp begins (+grade)
        ~x_crest          rounded crest begins (flat top)
        ~x_down           rounded shoulder, ramp ends (-grade)
        ~x_end            rounded toe, back to flat (z=0)
    The grade is reached in the MIDDLE of each ramp; corners are blended over
    +-`fillet` meters.
    """
    grade_deg: float = 15.0    # peak ramp inclination
    approach: float = 1.5      # flat run before the hill (reach gait speed)
    ramp_run: float = 1.8      # HORIZONTAL length of each ramp
    crest: float = 1.0         # crest length (rounded shoulders eat into it)
    fillet: float = 0.4        # corner-rounding half-width (m)
    width: float = 5.0         # cross-slope (y) extent of the terrain
    thickness: float = 0.40    # box thickness (sunk below the surface)
    box_dx: float = 0.08       # horizontal box-segment length

    def __post_init__(self):
        self._build()

    # ---- breakpoints ----
    @property
    def x_up(self):
        return self.approach

    @property
    def x_crest(self):
        return self.x_up + self.ramp_run

    @property
    def x_down(self):
        return self.x_crest + self.crest

    @property
    def x_end(self):
        return self.x_down + self.ramp_run

    @property
    def rise(self):
        return self.ramp_run * math.tan(self.grade_rad)

    @property
    def grade_rad(self):
        return math.radians(self.grade_deg)

    # ---- build the smoothed h(x) on a fine grid ----
    def _slope_raw_breaks(self):
        """(x_break, delta_slope) -- slope steps that build the trapezoid."""
        t = math.tan(self.grade_rad)
        return [(self.x_up, +t), (self.x_crest, -t), (self.x_down, -t), (self.x_end, +t)]

    def _slope(self, x):
        """Smoothed dh/dx = sum of smoothstep'd slope jumps at each corner."""
        s = 0.0
        for xb, ds in self._slope_raw_breaks():
            s += ds * _smoothstep((x - (xb - self.fillet)) / (2.0 * self.fillet))
        return s

    def _build(self):
        x0, x1 = -0.5, self.x_end + 1.0
        self._gx = np.arange(x0, x1, 0.01)
        sl = np.array([self._slope(x) for x in self._gx])
        h = np.concatenate([[0.0], np.cumsum(0.5 * (sl[1:] + sl[:-1]) * np.diff(self._gx))])
        self._gh = np.clip(h, 0.0, None)
        self._gs = sl

    def height(self, x: float) -> float:
        return float(np.interp(x, self._gx, self._gh))

    def slope(self, x: float) -> float:
        return float(np.interp(x, self._gx, self._gs))

    def pitch(self, x: float) -> float:
        """Desired base pitch (rad) = surface inclination = atan(h'(x))."""
        return math.atan(self.slope(x))

    # ---- per-segment boxes (top face on the surface) ----
    def _segments(self):
        """Fine piecewise-linear surface segments over the elevated region."""
        lo = self.x_up - 2.0 * self.fillet
        hi = self.x_end + 2.0 * self.fillet
        xs = np.arange(lo, hi + self.box_dx, self.box_dx)
        segs = []
        for i in range(len(xs) - 1):
            x0, x1 = float(xs[i]), float(xs[i + 1])
            z0, z1 = self.height(x0), self.height(x1)
            if max(z0, z1) > 5e-4:           # skip flat sections (the plane covers them)
                segs.append((x0, x1, z0, z1))
        return segs

    def _boxes(self):
        hy, hz = self.width / 2.0, self.thickness / 2.0
        out = []
        for (x0, x1, z0, z1) in self._segments():
            dx, dz = x1 - x0, z1 - z0
            theta = math.atan2(dz, dx)
            sx, sz = 0.5 * (x0 + x1), 0.5 * (z0 + z1)
            cx = sx + hz * math.sin(theta)
            cz = sz - hz * math.cos(theta)
            out.append(dict(cx=cx, cz=cz, theta=theta,
                            hx=math.hypot(dx, dz) / 2.0, hy=hy, hz=hz))
        return out

    # --- MuJoCo: static box geoms (string injected after <worldbody>) --------
    def mjcf_geoms(self, contype=2, conaffinity=2147483645,
                   friction="2 0.005 0.0001", solref="0.02",
                   rgba="0.35 0.30 0.25 1") -> str:
        lines = []
        for i, b in enumerate(self._boxes()):
            lines.append(
                f'    <geom name="hill_{i}" type="box" '
                f'size="{b["hx"]:.5f} {b["hy"]:.5f} {b["hz"]:.5f}" '
                f'pos="{b["cx"]:.5f} 0 {b["cz"]:.5f}" euler="0 {-b["theta"]:.6f} 0" '
                f'contype="{contype}" conaffinity="{conaffinity}" '
                f'friction="{friction}" solref="{solref}" rgba="{rgba}"/>')
        return "\n".join(lines)

    def inject_mjcf(self, src_path: str, out_path: str) -> str:
        with open(src_path, "r", encoding="utf-8") as f:
            s = f.read()
        marker = "<worldbody>"
        i = s.index(marker) + len(marker)
        s = s[:i] + "\n" + self.mjcf_geoms() + s[i:]
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(s)
        return out_path

    # --- OmniSim .wbt: matching Box solids (full sizes, axis-angle rotation) --
    def wbt_nodes(self, name="hill", color="0.35 0.30 0.25",
                  collidable=False) -> str:
        nodes = []
        for i, b in enumerate(self._boxes()):
            sz = f'{2*b["hx"]:.5f} {2*b["hy"]:.5f} {2*b["hz"]:.5f}'
            bo = f'\n  boundingObject Box {{ size {sz} }}' if collidable else ''
            nodes.append(
                f'Solid {{\n'
                f'  translation {b["cx"]:.5f} 0 {b["cz"]:.5f}\n'
                f'  rotation 0 1 0 {-b["theta"]:.6f}\n'
                f'  children [ Shape {{ appearance PBRAppearance {{ '
                f'baseColor {color} roughness 0.95 metalness 0 }} '
                f'geometry Box {{ size {sz} }} }} ]\n'
                f'  name "{name}_{i}"{bo}\n'
                f'}}')
        return "\n".join(nodes)


@dataclass
class RoughProfile:
    """Randomized rough-terrain course for BLIND terrain-curriculum RL — a field
    of full-width BUMP BARS of random height in [amp_min, amp] along +x, with a
    flat approach so the gait reaches speed first. Same injection interface as
    HillProfile (mjcf_geoms / inject_mjcf / wbt_nodes / height) so the trainer
    and deploy share one source of truth. Difficulty is the single knob `amp`
    (max bump height) — ramp it across curriculum stages.

    Bars are axis-aligned boxes resting on the floor (top face at h), constant
    across y (both feet meet the same bar — matches the rough-track benchmark).
    """
    amp: float = 0.06          # max bump height (m) — the curriculum difficulty knob
    amp_min_frac: float = 0.35  # bumps are U(amp_min_frac*amp, amp)
    x_start: float = 2.0       # flat approach before the first bump
    x_end: float = 40.0        # bump field extent (+x)
    gap_min: float = 0.45      # spacing between bar centres (m)
    gap_max: float = 0.95
    w_min: float = 0.15        # bar x-thickness (m)
    w_max: float = 0.30
    width: float = 6.0         # full cross-track (y) extent
    seed: int = 0

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        bars = []
        x = self.x_start
        while x < self.x_end:
            w = float(rng.uniform(self.w_min, self.w_max))
            h = float(rng.uniform(self.amp_min_frac * self.amp, self.amp))
            bars.append((x, h, w))
            x += w + float(rng.uniform(self.gap_min, self.gap_max))
        self.bars = bars
        # dense surface grid h(x) for ground_h lookups (step function)
        gx = np.arange(self.x_start - 1.0, self.x_end + 1.0, 0.01, dtype=np.float32)
        gh = np.zeros_like(gx)
        for (cx, h, w) in bars:
            gh[(gx >= cx - w / 2.0) & (gx <= cx + w / 2.0)] = h
        self._gx, self._gh = gx, gh

    def height(self, x: float) -> float:
        return float(np.interp(x, self._gx, self._gh))

    def _boxes(self):
        hy = self.width / 2.0
        return [dict(cx=cx, cz=h / 2.0, theta=0.0, hx=w / 2.0, hy=hy, hz=h / 2.0)
                for (cx, h, w) in self.bars]

    def mjcf_geoms(self, contype=2, conaffinity=2147483645,
                   friction="2 0.005 0.0001", solref="0.02",
                   rgba="0.40 0.32 0.20 1") -> str:
        lines = []
        for i, b in enumerate(self._boxes()):
            lines.append(
                f'    <geom name="rough_{i}" type="box" '
                f'size="{b["hx"]:.5f} {b["hy"]:.5f} {b["hz"]:.5f}" '
                f'pos="{b["cx"]:.5f} 0 {b["cz"]:.5f}" '
                f'contype="{contype}" conaffinity="{conaffinity}" '
                f'friction="{friction}" solref="{solref}" rgba="{rgba}"/>')
        return "\n".join(lines)

    def inject_mjcf(self, src_path: str, out_path: str) -> str:
        with open(src_path, "r", encoding="utf-8") as f:
            s = f.read()
        marker = "<worldbody>"
        i = s.index(marker) + len(marker)
        s = s[:i] + "\n" + self.mjcf_geoms() + s[i:]
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(s)
        return out_path


@dataclass
class StairProfile:
    """A discrete STAIRCASE for G1 stair-climb Shadowing -- a flat approach, N
    treads of equal `riser` height and `going` depth rising toward +x, then a
    flat top landing to stand on. Same injection interface as HillProfile /
    RoughProfile (height / mjcf_geoms / inject_mjcf / wbt_nodes) so the ghost
    generator, the in-engine RL trainer, and the OmniSim deploy/preview world
    all share ONE terrain. Because the recipe trainer compiles its batched model
    from the loaded .wbt (put_model of the live MjModel), the collidable Box
    solids emitted by wbt_nodes() ARE the training terrain -- train == deploy by
    construction.

    Each tread k (k = 1..n_steps) is realized as a FULL-HEIGHT box (bottom on the
    floor, top face at k*riser) spanning x in [x0+(k-1)*going, x0+k*going]; the
    exposed vertical face between adjacent blocks is the riser, the exposed top
    strip is the tread. A final landing box at n_steps*riser gives the climber a
    place to arrive and stand. Convention: climbing in +x.

        h(x) = 0                                  x <  x_start
             = k*riser   for the k-th tread       x in the tread's x-span
             = n_steps*riser                      x >= top of the last tread (landing)

      python projects/policies/research/shadowing/hill_terrain.py --stairs   # self-test
    """
    riser: float = 0.10       # step height (m) -- clearable with knee lift
    going: float = 0.26       # tread depth (m) == the per-step forward stride
    n_steps: int = 5          # number of treads (climb = n_steps*riser)
    width: float = 3.0        # cross-track (y) extent of the staircase (m)
    x_start: float = 1.5      # flat approach before the first riser (reach gait speed)
    landing: float = 1.5      # flat top-landing depth in +x past the last tread (m)

    def __post_init__(self):
        # dense surface grid h(x) for ground_h lookups + a rising harness target
        x_lo, x_hi = self.x_start - 1.0, self.x_top + self.landing + 1.0
        gx = np.arange(x_lo, x_hi, 0.01, dtype=np.float32)
        gh = np.zeros_like(gx)
        for k in range(1, self.n_steps + 1):
            xa = self.x_start + (k - 1) * self.going
            xb = self.x_start + k * self.going
            gh[(gx >= xa) & (gx < xb)] = k * self.riser
        gh[gx >= self.x_top] = self.n_steps * self.riser
        self._gx, self._gh = gx, gh

    # ---- breakpoints ----
    @property
    def x_top(self):
        """x at which the last tread ends and the landing begins."""
        return self.x_start + self.n_steps * self.going

    @property
    def rise(self):
        return self.n_steps * self.riser

    def height(self, x: float) -> float:
        return float(np.interp(x, self._gx, self._gh))

    # ---- per-step full-height boxes (top face on the tread) ----
    def _boxes(self):
        hy = self.width / 2.0
        out = []
        for k in range(1, self.n_steps + 1):
            top = k * self.riser
            cx = self.x_start + (k - 0.5) * self.going
            out.append(dict(cx=cx, cz=top / 2.0, hx=self.going / 2.0, hy=hy, hz=top / 2.0))
        # top landing (full height of the last tread)
        top = self.n_steps * self.riser
        out.append(dict(cx=self.x_top + self.landing / 2.0, cz=top / 2.0,
                        hx=self.landing / 2.0, hy=hy, hz=top / 2.0))
        return out

    # --- MuJoCo: static box geoms (string injected after <worldbody>) --------
    def mjcf_geoms(self, contype=2, conaffinity=2147483645,
                   friction="1.5 0.005 0.0001", solref="0.02",
                   rgba="0.38 0.34 0.30 1") -> str:
        lines = []
        for i, b in enumerate(self._boxes()):
            lines.append(
                f'    <geom name="stair_{i}" type="box" '
                f'size="{b["hx"]:.5f} {b["hy"]:.5f} {b["hz"]:.5f}" '
                f'pos="{b["cx"]:.5f} 0 {b["cz"]:.5f}" '
                f'contype="{contype}" conaffinity="{conaffinity}" '
                f'friction="{friction}" solref="{solref}" rgba="{rgba}"/>')
        return "\n".join(lines)

    def inject_mjcf(self, src_path: str, out_path: str) -> str:
        with open(src_path, "r", encoding="utf-8") as f:
            s = f.read()
        marker = "<worldbody>"
        i = s.index(marker) + len(marker)
        s = s[:i] + "\n" + self.mjcf_geoms() + s[i:]
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(s)
        return out_path

    # --- OmniSim .wbt: matching Box solids (full sizes, collidable) -----------
    def wbt_nodes(self, name="stair", color="0.38 0.34 0.30",
                  collidable=True) -> str:
        nodes = []
        for i, b in enumerate(self._boxes()):
            sz = f'{2*b["hx"]:.5f} {2*b["hy"]:.5f} {2*b["hz"]:.5f}'
            bo = f'\n  boundingObject Box {{ size {sz} }}' if collidable else ''
            nodes.append(
                f'DEF {name.upper()}_{i} Solid {{\n'
                f'  translation {b["cx"]:.5f} 0 {b["cz"]:.5f}\n'
                f'  children [ Shape {{ appearance PBRAppearance {{ '
                f'baseColor {color} roughness 0.9 metalness 0 }} '
                f'geometry Box {{ size {sz} }} }} ]\n'
                f'  name "{name}_{i}"{bo}\n'
                f'}}')
        return "\n".join(nodes)


def _self_test():
    """Build a B2 model with the hill injected and confirm each box's TOP face
    height (probed by a downward ray) equals the smoothed h(x)."""
    import os
    import tempfile
    import mujoco

    REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    src = os.path.join(REPO, "projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml")
    hp = HillProfile()
    print(f"HillProfile: grade={hp.grade_deg}deg rise={hp.rise:.3f}m fillet={hp.fillet}  "
          f"x_up={hp.x_up} x_crest={hp.x_crest} x_down={hp.x_down} x_end={hp.x_end}  "
          f"({len(hp._boxes())} boxes)")
    print(f"  peak slope reached mid-ramp: pitch({hp.x_up+hp.ramp_run/2:.2f})="
          f"{math.degrees(hp.pitch(hp.x_up+hp.ramp_run/2)):.1f}deg (target {hp.grade_deg})")

    out = os.path.join(tempfile.gettempdir(), "b2_hill_selftest.xml")
    hp.inject_mjcf(src, out)
    m = mujoco.MjModel.from_xml_path(out)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    xs = np.linspace(hp.x_up - 0.2, hp.x_end + 0.2, 80)
    worst = 0.0
    for x in xs:
        gid = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(m, d, np.array([x, 0.0, 3.0]),
                             np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
        if dist < 0:
            continue
        worst = max(worst, abs((3.0 - dist) - hp.height(float(x))))
    print(f"box-top vs smoothed h(x): worst error over {len(xs)} probes = {worst*1000:.2f} mm")
    assert worst < 8e-3, "hill box tops do not match the smoothed profile"
    # continuity: slope is C0 (no jumps) thanks to the fillets
    grid = np.linspace(0, hp.x_end + 0.5, 2000)
    dpitch = np.abs(np.diff([hp.pitch(x) for x in grid]))
    print(f"max pitch step between adjacent x (2mm apart) = {math.degrees(dpitch.max()):.3f} deg")
    assert dpitch.max() < math.radians(1.0)
    print("SELF-TEST OK")


def _stair_self_test():
    """Confirm the StairProfile box tops == the step function h(x), the treads
    are contiguous, and the geometry lists cleanly for both back-ends."""
    sp = StairProfile()
    print(f"StairProfile: riser={sp.riser} going={sp.going} n_steps={sp.n_steps} "
          f"rise={sp.rise:.3f}m  x_start={sp.x_start} x_top={sp.x_top:.3f} "
          f"landing={sp.landing}  ({len(sp._boxes())} boxes)")
    # each tread's centre samples to its own top height; the box top face matches
    worst = 0.0
    for k in range(1, sp.n_steps + 1):
        xc = sp.x_start + (k - 0.5) * sp.going
        worst = max(worst, abs(sp.height(xc) - k * sp.riser))
    # landing centre
    worst = max(worst, abs(sp.height(sp.x_top + sp.landing / 2.0) - sp.rise))
    print(f"tread-centre h(x) vs k*riser: worst error = {worst*1000:.3f} mm")
    assert worst < 1e-6, "stair height table disagrees with the tread boxes"
    # contiguity: adjacent tread boxes share an x face (no gap, no overlap)
    bx = sp._boxes()
    for k in range(sp.n_steps - 1):
        right = bx[k]["cx"] + bx[k]["hx"]
        left = bx[k + 1]["cx"] - bx[k + 1]["hx"]
        assert abs(right - left) < 1e-6, f"tread {k}->{k+1} not contiguous"
    for k in range(sp.n_steps):
        for tag, s in (("wbt", sp.wbt_nodes()), ("mjcf", sp.mjcf_geoms())):
            assert f"stair_{k}" in s or f"STAIR_{k}" in s, f"{tag} missing stair_{k}"
    print(f"riser height {sp.riser*100:.0f} cm, going {sp.going*100:.0f} cm, "
          f"total climb {sp.rise*100:.0f} cm over {sp.n_steps*sp.going:.2f} m")
    print("STAIR SELF-TEST OK")


if __name__ == "__main__":
    import sys
    if "--stairs" in sys.argv:
        _stair_self_test()
    else:
        _self_test()
