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

"""Metazoa organism control: gait CPG, body plan, steering, lone-cell flip,
docking geometry.

PURE Python on purpose -- no `omnisim` import, no engine, no I/O -- so every
number the supervisor writes into a hinge can be unit-tested in milliseconds
(`tests/test_organism.py`). The supervisor owns the engine round trips: it
reads poses, calls this module, and batches the hinge targets / lock writes it
hands back. Nothing here knows what a DEF is.

Contract: `projects/metazoa/DESIGN.md` ("The organism (B)"). The constants
below are the numbers written there; change them there first.

Frames and units
----------------
World is ENU, z up. A cell's chain axis is its local +x (tail block at the
origin, hinge in the seam at x = +0.03 with axis local y, nose block centred at
x = +0.06). A 2-D pose is `(x, y, yaw)`: yaw is the world angle of the cell's
+x axis. A face pose is the same triple where yaw is the OUTWARD docking
normal of the face (the Connector's +x).

Chain indexing
--------------
Spine cell i sits at i * (CELL_LEN + gap) along the chain's +x from cell 0, so
**index 0 is the TAIL cell (its tail face is free) and index n-1 is the
HEAD (its nose face is free)**. The travelling wave `sin(omega*t + i*dphi)`
therefore runs head -> tail, which is the backward-travelling body wave that
propels a lateral undulator forward (toward the head's +x). Which way a real
chain goes is measured in P1, not assumed here.

Dock rotations
--------------
`dock_rotation_pattern[i]` (cycled) is the roll of spine cell i about the
chain's own x axis in quarter turns, RELATIVE TO THE CHAIN FRAME (cell 0's
frame). It is not composed with cell i-1's roll: DESIGN.md's docking geometry
places cell j "yawed like i ... rolled 90 deg about its own x", i.e. each cell
takes the chain's yaw and then its own roll, and the P1 gate calls
[0, 1, 0, 1] "the alternating chain", which is only true under this reading.
The relative rotation a worldgen composing orientations incrementally needs is
`relative_dock_rotation(bodyplan, i)` = (roll_i - roll_{i-1}) mod 4.
"""
import copy
import math

TAU = 2.0 * math.pi

# --- cell geometry (DESIGN.md "The cell") ----------------------------------
BLOCK = 0.06                 # one block edge, m
CELL_LEN = 2.0 * BLOCK       # tail block + nose block, folded flat
NOSE_X = 0.06                # nose block centre along the cell's +x
# Face poses in the CELL frame at hinge angle 0: (x, y, outward-normal yaw).
FACE_LOCAL = {
    "f_tail": (-0.03, 0.0, math.pi),
    "f_nose": (0.09, 0.0, 0.0),
    "f_left": (NOSE_X, 0.03, math.pi / 2.0),
    "f_right": (NOSE_X, -0.03, -math.pi / 2.0),
}
BRANCH_SIDE_FACE = {"L": "f_left", "R": "f_right"}

# --- motor envelope --------------------------------------------------------
MOTOR_LIMIT = 2.6            # RotationalMotor min/maxPosition authored in the cell
TARGET_CEIL = 2.5            # every target is clamped inside +-2.5 rad: a target
                             # past the motor limit logs one WARNING per joint per
                             # tick (the alife log-storm trap)
FLIP_FOLD = 2.4              # lone-cell somersault fold angle, rad

# --- genome ranges (DESIGN.md "The organism (B)") --------------------------
# Ranges centred on the P1c-measured gait (A 0.9 / omega 4.5 / dphi 1.2 was
# the best of the sweep; A*omega must stay under the motor's maxVelocity 5 or
# the chain tears at its welds -- measured 38 m separations at A 1.0 / omega 8).
GENOME_RANGES = {
    "A": (0.5, 1.0),
    "omega": (3.0, 5.0),
    "dphi": (0.9, 1.6),
    "bias_pitch": (-0.3, 0.3),
    "bias_yaw": (-0.3, 0.3),
    "branch_phase": (-math.pi, math.pi),   # an angle: wrapped, not clamped
    "branch_scale": (0.0, 1.0),
    "steer_gain": (0.0, 0.8),     # beyond ~0.7 rad the rudder stalls the body (measured)
}
GENOME_KEYS = tuple(GENOME_RANGES)
# per-key gaussian creep sigma at rate 1.0 (about 1/10 of the range)
GENOME_SIGMA = {
    "A": 0.09, "omega": 0.6, "dphi": 0.18, "bias_pitch": 0.12, "bias_yaw": 0.12,
    "branch_phase": 0.5, "branch_scale": 0.1, "steer_gain": 0.06,
}

# --- body plan ranges ------------------------------------------------------
TARGET_LENGTH = (4, 12)
PATTERN_LEN = (1, 4)         # entries in dock_rotation_pattern (cycled)
ROTATIONS = (0, 1, 2, 3)     # quarter turns about the chain x
SIDES = ("L", "R")

# --- steering --------------------------------------------------------------
STEER_ERR_FULL = 1.2       # rad of heading error at full steer (0.6 over-steered: +-2 rad oscillation, measured)
STEER_DEADBAND = 0.1         # rad; inside it the organism goes straight


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    w = a - TAU * math.floor((a + math.pi) / TAU)
    return w + TAU if w <= -math.pi else w


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _creep(rng, v, sigma, lo, hi):
    return clamp(v + rng.gauss(0.0, sigma), lo, hi)


# ================================================================== genome
# Seeding draws from a NARROW window around the P1c-measured gait (A 0.9 /
# omega 4.5 / dphi 1.2, biases ~0): the first live reef seeded uniformly over
# GENOME_RANGES and every organism crawled at <0.02 m/s with a -0.26 pitch
# bias arching it onto its side. Mutation still explores the full ranges.
SEED_WINDOW = {
    # MEASURED (probe_wave, 6 cells, gain 0.5, 2026-08-29): the yaw rate is
    # genome-sensitive -- A .9/w 4.5/dphi 1.2 turns 0.62 rad per 15 s, the
    # reef's own draws A .85/w 4.55/dphi 1.02 0.36 and A .86/w 4.15/dphi 1.26
    # 0.17 (all walk 0.08-0.12 m/s straight). Seeds sit on the measured
    # walker; mutation (GENOME_RANGES) explores from there.
    "A": (0.88, 0.92), "omega": (4.4, 4.6), "dphi": (1.15, 1.25),
    "bias_pitch": (-0.04, 0.04), "bias_yaw": (-0.04, 0.04),
    "branch_phase": (-math.pi, math.pi), "branch_scale": (0.3, 0.7),
    "steer_gain": (0.45, 0.55),   # rudder angle (rad at full lock). MEASURED on the 6-cell probe
                                  # (probe_wave, 2026-08-29): 0.5 -> +0.62/-0.43 rad per 15 s at 0.08 m/s;
                                  # 1.0 halves the speed AND the yaw rate (a 57 deg head is an anchor);
                                  # the 0.9-1.1 window this used to hold was never measured
}


def random_genome(rng, seed_window=True):
    """Draw a genome. `rng` is a random.Random -- the ONLY source of
    randomness in this module. seed_window=True (default) draws inside
    SEED_WINDOW; False draws uniformly over GENOME_RANGES."""
    g = {}
    src = SEED_WINDOW if seed_window else GENOME_RANGES
    for k, (lo, hi) in src.items():
        g[k] = rng.uniform(lo, hi)
    return g


def mutate(genome, rng, rate=1.0):
    """Gaussian creep on every parameter, clamped into range (branch_phase is
    an angle and wraps instead). The gait STRUCTURE -- one travelling wave down
    the spine, per-axis biases, mirrored branches -- is fixed by construction
    (the alife `lock_gait` lesson); only its parameters evolve. Returns a new
    dict; the parent is untouched. Always passes validate() when the parent
    does."""
    m = copy.deepcopy(genome)
    for k, (lo, hi) in GENOME_RANGES.items():
        sigma = GENOME_SIGMA[k] * rate
        if k == "branch_phase":
            m[k] = wrap_angle(m[k] + rng.gauss(0.0, sigma))
        else:
            m[k] = _creep(rng, m[k], sigma, lo, hi)
    return m


def validate(genome):
    """Returns a list of problems (empty = ok), the alife convention: a missing
    key, a non-finite value, or a value outside its range."""
    bad = []
    if not isinstance(genome, dict):
        return ["genome is not a dict: %r" % (type(genome).__name__,)]
    for k, (lo, hi) in GENOME_RANGES.items():
        if k not in genome:
            bad.append("missing %s" % k)
            continue
        v = genome[k]
        if not _finite(v):
            bad.append("%s non-finite: %r" % (k, v))
        elif not (lo - 1e-9 <= v <= hi + 1e-9):
            bad.append("%s = %.4f outside [%.3f, %.3f]" % (k, v, lo, hi))
    for k in genome:
        if k not in GENOME_RANGES:
            bad.append("unknown key %s" % k)
    return bad


# ================================================================ body plan
def random_bodyplan(rng):
    """target_length 2-8, a 1-4 entry rotation pattern, and a branch pair on
    30 % of draws."""
    n = rng.randint(8, TARGET_LENGTH[1])
    # rotations {0,1} only: a roll of 2/3 puts every roller in the air (cell v3);
    # seed the two patterns P1c measured -- pitch chain (fast) or alternating (steers)
    # always alternating: a pitch-only chain has no steering channel and cannot
    # reach a recruit (measured: 6 failed approaches in 240 s)
    pattern = [0, 1]
    bp = {"target_length": n, "dock_rotation_pattern": pattern, "branch_rule": "none"}
    if rng.random() < 0.3:
        bp["branch_rule"] = _random_branch(rng, n)
    return bp


def _random_branch(rng, n):
    sides = list(rng.choice((("L",), ("R",), ("L", "R"))))
    return {"at": rng.randint(0, n - 1), "sides": sides}


def mutate_bodyplan(bodyplan, rng, rate=1.0):
    """Developmental-program creep: target_length +-1 (p 0.3), one rotation
    entry re-rolled (p 0.15 each), the pattern grows or shrinks by one entry
    (p 0.1), the branch rule toggles / moves / changes side (p 0.1). Returns a
    new dict that passes validate_bodyplan() when the parent does."""
    bp = copy.deepcopy(bodyplan)
    lo, hi = TARGET_LENGTH
    if rng.random() < 0.3 * rate:
        bp["target_length"] = clamp(bp["target_length"] + rng.choice((-1, 1)), lo, hi)
    pat = bp["dock_rotation_pattern"]
    for i in range(len(pat)):
        if rng.random() < 0.15 * rate:
            pat[i] = rng.choice(ROTATIONS)
    if rng.random() < 0.1 * rate:
        if len(pat) < PATTERN_LEN[1] and (len(pat) <= PATTERN_LEN[0] or rng.random() < 0.5):
            pat.append(rng.choice(ROTATIONS))
        elif len(pat) > PATTERN_LEN[0]:
            pat.pop()
    n = bp["target_length"]
    if rng.random() < 0.1 * rate:
        if bp["branch_rule"] == "none":
            bp["branch_rule"] = _random_branch(rng, n)
        else:
            r = rng.random()
            if r < 0.4:
                bp["branch_rule"] = "none"
            elif r < 0.7:
                bp["branch_rule"]["at"] = rng.randint(0, n - 1)
            else:
                bp["branch_rule"]["sides"] = list(rng.choice((("L",), ("R",), ("L", "R"))))
    br = bp["branch_rule"]
    if br != "none" and br["at"] >= n:      # a shrunk spine no longer has cell k
        br["at"] = n - 1
    return bp


def validate_bodyplan(bodyplan):
    """Returns a list of problems (empty = ok)."""
    bad = []
    if not isinstance(bodyplan, dict):
        return ["bodyplan is not a dict: %r" % (type(bodyplan).__name__,)]
    n = bodyplan.get("target_length")
    if not isinstance(n, int) or isinstance(n, bool):
        bad.append("target_length not an int: %r" % (n,))
        n = None
    elif not (TARGET_LENGTH[0] <= n <= TARGET_LENGTH[1]):
        bad.append("target_length = %d outside [%d, %d]" % (n, TARGET_LENGTH[0], TARGET_LENGTH[1]))
    pat = bodyplan.get("dock_rotation_pattern")
    if not isinstance(pat, list):
        bad.append("dock_rotation_pattern not a list: %r" % (pat,))
    else:
        if not (PATTERN_LEN[0] <= len(pat) <= PATTERN_LEN[1]):
            bad.append("dock_rotation_pattern has %d entries, allowed [%d, %d]"
                       % (len(pat), PATTERN_LEN[0], PATTERN_LEN[1]))
        for i, r in enumerate(pat):
            if not isinstance(r, int) or isinstance(r, bool) or r not in ROTATIONS:
                bad.append("dock_rotation_pattern[%d] = %r not in 0..3" % (i, r))
    br = bodyplan.get("branch_rule", None)
    if br == "none":
        pass
    elif isinstance(br, dict):
        k = br.get("at")
        if not isinstance(k, int) or isinstance(k, bool):
            bad.append("branch_rule.at not an int: %r" % (k,))
        elif n is not None and not (0 <= k < n):
            bad.append("branch_rule.at = %d outside [0, %d)" % (k, n))
        sides = br.get("sides")
        if not isinstance(sides, list) or not sides:
            bad.append("branch_rule.sides must be a non-empty list: %r" % (sides,))
        else:
            for s in sides:
                if s not in SIDES:
                    bad.append("branch_rule.sides entry %r not in L/R" % (s,))
            if len(set(sides)) != len(sides):
                bad.append("branch_rule.sides repeats a side: %r" % (sides,))
        for key in br:
            if key not in ("at", "sides"):
                bad.append("branch_rule unknown key %s" % key)
    else:
        bad.append("branch_rule must be 'none' or {at, sides}: %r" % (br,))
    for key in bodyplan:
        if key not in ("target_length", "dock_rotation_pattern", "branch_rule"):
            bad.append("unknown key %s" % key)
    return bad


def roll_of(bodyplan, i):
    """Roll of spine cell i about the chain x, in quarter turns (0..3), read
    from the cycled dock_rotation_pattern (see module docstring)."""
    pat = bodyplan["dock_rotation_pattern"]
    return pat[i % len(pat)] % 4


def relative_dock_rotation(bodyplan, i):
    """Quarter turns cell i is rolled relative to cell i-1 (0 for cell 0) --
    what a worldgen that composes each cell's orientation from its
    predecessor's needs."""
    if i <= 0:
        return 0
    return (roll_of(bodyplan, i) - roll_of(bodyplan, i - 1)) % 4


def axis_of(bodyplan, i):
    """'pitch' when spine cell i's hinge axis is horizontal-transverse (roll 0
    or 180 deg: the axis is the chain's +-y), 'yaw' when it is vertical (roll
    +-90 deg: the axis is +-z). A 180 deg roll keeps the class."""
    return "yaw" if roll_of(bodyplan, i) % 2 else "pitch"


def axes(bodyplan, n):
    return [axis_of(bodyplan, i) for i in range(n)]


# ====================================================================== gait
def _clip(v):
    return clamp(v, -TARGET_CEIL, TARGET_CEIL)


def chain_targets(genome, bodyplan, n, t, steer=0.0, branches=(), out=None):
    """Hinge targets for an n-cell spine at time t (rad, tail -> head order),
    followed by one target per entry of `branches` (side letters 'L'/'R' of
    the branch cells actually docked, in the order given).

    Spine cell i: `bias_axis + A*sin(omega*t + i*dphi)`, where a pitch cell
    gets `bias_pitch` and a yaw cell `bias_yaw + steer_gain*steer`. `steer` in
    [-1, 1] (clamped) is a uniform yaw-bias asymmetry that arcs the whole
    body -- the steering channel P1/P2 measure; its sign is calibrated there.

    Branch cell on side s of spine cell k (bodyplan.branch_rule.at):
    `branch_scale*A*sin(omega*t + k*dphi + sign_s*branch_phase)` with sign
    +1 for L and -1 for R, so a pair is mirrored like a limb pair. No bias
    (the DESIGN formula has none). With branch_rule "none" the branch targets
    are 0.0 (a hitch-hiker, not a limb).

    Every target is clamped inside +-TARGET_CEIL. Pass `out` to reuse a list
    across ticks."""
    A, omega, dphi = genome["A"], genome["omega"], genome["dphi"]
    steer = clamp(steer, -1.0, 1.0)
    yaw_bias = genome["bias_yaw"] + genome["steer_gain"] * steer
    pitch_bias = genome["bias_pitch"]
    wt = omega * t
    if out is None:
        out = []
    else:
        del out[:]
    for i in range(n):
        bias = yaw_bias if axis_of(bodyplan, i) == "yaw" else pitch_bias
        out.append(_clip(bias + A * math.sin(wt + i * dphi)))
    for side in branches:
        out.append(branch_target(genome, bodyplan, side, t))
    return out


def branch_target(genome, bodyplan, side, t):
    """One branch cell's hinge target (see chain_targets)."""
    br = bodyplan["branch_rule"]
    if br == "none":
        return 0.0
    sign = 1.0 if side == "L" else -1.0
    phase = genome["omega"] * t + br["at"] * genome["dphi"] + sign * genome["branch_phase"]
    return _clip(genome["branch_scale"] * genome["A"] * math.sin(phase))


def wave_speed_estimate(genome, cell_len=CELL_LEN):
    """Phase speed of the body wave along the spine, m/s: wavelength
    (cell_len * 2*pi/dphi) x frequency (omega / 2*pi) = cell_len*omega/dphi.
    It is the speed at which a crest travels down the body, i.e. an upper
    bound on ground speed for a non-slipping undulator; the driver logs it
    beside the measured centroid speed so the slip ratio is a number."""
    return cell_len * genome["omega"] / genome["dphi"]


# ============================================================== lone flip
def flip_sequence(t, period=2.0, fold=FLIP_FOLD):
    """Hinge target for a lone cell's somersault step (M-TRAN): a fast LINEAR
    fold to `fold` over the first 25 % of the period, a hold for 10 %, and a
    slow linear unfold to 0 over the remaining 65 %. Linear on purpose: 2.4
    rad over 0.5 s is 4.8 rad/s, just inside the motor's maxVelocity 5, where
    a smooth ramp would peak at twice the mean rate and saturate. Direction
    of travel is the cell's +x (the nose tips over the tail); a lone cell
    cannot turn."""
    if period <= 0.0:
        return 0.0
    u = (t % period) / period
    if u < 0.25:
        return fold * (u / 0.25)
    if u < 0.35:
        return fold
    return fold * (1.0 - (u - 0.35) / 0.65)


# ================================================================ geometry
def compose_pose(base, local):
    """World pose of `local` (x, y, yaw) expressed in the frame of `base`."""
    bx, by, byaw = base
    lx, ly, lyaw = local
    c, s = math.cos(byaw), math.sin(byaw)
    return (bx + c * lx - s * ly, by + s * lx + c * ly, wrap_angle(byaw + lyaw))


def face_pose(cell_pose, face):
    """World pose of a cell's face (x, y, outward-normal yaw) at hinge angle
    0. `face` is a face name ('f_nose' ...) or a local (x, y, yaw) triple. The
    nose-block faces move with the hinge, so the driver commands 0 on the
    hinges involved before it trusts this."""
    local = FACE_LOCAL[face] if isinstance(face, str) else tuple(face)
    return compose_pose(cell_pose, local)


def approach_pose(cell_pose, free_face_pose, gap, head_face="f_nose"):
    """The organism-HEAD root pose (x, y, yaw) that mates the head's
    `head_face` with a free cell's face.

    `cell_pose` is the free cell's root pose; `free_face_pose` is that face
    in the free cell's frame -- a name ('f_nose', 'f_tail', 'f_left',
    'f_right') or a local (x, y, yaw) triple -- or None when `cell_pose`
    already IS the world face pose. The result puts the two faces `gap` apart
    with normals opposed (the Connector's distance/axis tolerance both
    satisfied at gap <= 0.03), so the block centres are BLOCK + gap apart
    along the free face's normal and the head faces yaw_face + pi (for the
    nose face). Both cells flat on the floor at roll 0 also satisfies the
    4-fold rotation tolerance."""
    if free_face_pose is None:
        fx, fy, fyaw = cell_pose
    else:
        fx, fy, fyaw = face_pose(cell_pose, free_face_pose)
    nx, ny = math.cos(fyaw), math.sin(fyaw)
    # where the head's face centre must be, and the normal it must carry
    tx, ty = fx + gap * nx, fy + gap * ny
    tyaw = fyaw + math.pi
    lx, ly, lyaw = FACE_LOCAL[head_face] if isinstance(head_face, str) else tuple(head_face)
    hyaw = wrap_angle(tyaw - lyaw)
    c, s = math.cos(hyaw), math.sin(hyaw)
    return (tx - (c * lx - s * ly), ty - (s * lx + c * ly), hyaw)


def heading_error(head_pose, target_xy):
    """wrap(bearing_to_target - head yaw); positive = target is to the head's
    LEFT (counter-clockwise, ENU). Returns 0.0 when the target is on the
    head (no bearing)."""
    dx, dy = target_xy[0] - head_pose[0], target_xy[1] - head_pose[1]
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return wrap_angle(math.atan2(dy, dx) - head_pose[2])


def distance_xy(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def steer_from_error(err, err_full=STEER_ERR_FULL, deadband=STEER_DEADBAND, sign=1.0):
    """Proportional steer command in [-1, 1] from a heading error, with a
    deadband. `sign` is the polarity P1 measures for the steering channel
    (+1 until the probe says otherwise)."""
    if err is None or abs(err) <= deadband:
        return 0.0
    return sign * clamp(err / err_full, -1.0, 1.0)
