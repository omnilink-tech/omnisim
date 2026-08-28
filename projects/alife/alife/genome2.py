#!/usr/bin/env python3
"""Genome v2: bilateral capsule creatures with a CPG brain and ecology traits.

Supersedes genome.py (box torso + independent limbs). A v2 creature is a
capsule torso with 1-3 BILATERAL limb pairs; each limb is 1-2 capsule
segments (hip, optional knee), so morphology reads as an animal rather than a
box with sticks. The contract is projects/alife/DESIGN_v2.md.

Morphology is still a TREE, for the same two hard engine reasons as v1:

  * A closed kinematic loop makes `SolverMuJoCo` construction raise, and the
    world then gets NO physics at all -- the entire population freezes, not
    just the offending creature. A tree cannot close a loop by construction.
  * A `Cone` boundingObject is absent from newton's geom mapping and kills the
    world at solver construction. Every part here is a Capsule or a Sphere.

The brain is a per-joint central pattern generator, mirrored across the body:

    target(t) = bias + amp * sin(2*pi*freq*t + phase [+ mirror_phase on the R side])

plus the ecology traits the director reads (steer_gain, heading_offset,
sense_radius, wander). CPGs rather than evolved networks on purpose: this
repo's history is that from-scratch policy learning under Newton reliably
fails, and a CPG locomotes in generation 1, so there is a gradient to climb.

Two mutation operators, deliberately separate:
  * `mutate_body`  -- between epochs, when the world is regenerated (a body
                      plan can only change at load: runtime spawn has no physics)
  * `mutate_brain` -- at birth, when a pooled slot is revived mid-run

Measured constraint (see ../README.md): perfectly left-right symmetric gaits
produce EXACTLY zero net displacement. mirror_phase is what breaks that.
"""
import copy
import math
import random

# Motor limits authored into the world. bias+amp must stay inside these or the
# engine emits one WARNING per joint per tick, and the log I/O alone is enough
# to slow the simulation below its own measurement points.
JOINT_LIMIT = 1.8
GAIT_CEIL = 1.55          # hard cap on |bias| + |amp|
BIAS_MAX = 0.9

# THE WALKER ENVELOPE. Ranges below are centred on the designed archetype
# that measurably walks (probe_steer2: 0.73 m/s, straight, no flips, standing
# at full height): torso 0.30 x r0.05, two pairs at x +-0.7, two 0.12 m
# segments, splay 0.6, freq 1.2, hip amp 0.35, knee amp 0.35 with bias -amp
# and +pi/2 lag, trot. Random bodies outside this envelope produced chaotic,
# unsteerable, flipping locomotion; evolution explores WITHIN it.
TORSO_LEN = (0.24, 0.38)
TORSO_RAD = (0.04, 0.062)
HEAD_RATIO = (0.6, 1.0)   # head radius as a fraction of torso radius
NPAIR = (2, 2)             # 3 pairs = 13 bodies/creature; over the realtime budget
NSEG = (2, 2)             # knees are what lift the foot on the swing; no knee, no walk
SEG_LEN = (0.09, 0.16)
SEG_RAD = (0.015, 0.028)
SPLAY = (0.45, 0.8)       # SPRAWL: the first two ecosystem runs tipped every body onto
                          # its back within ~3 s (screenshot _life_t6.png, legs in the
                          # air). Lizards and insects are stable because they sprawl:
                          # wide stance, low centre of mass. Narrow stances are refused
                          # by validate()'s lateral margin, not merely discouraged.
LATERAL_MARGIN = 0.75     # min (half stance width) / rest height
PAIR_X = (-0.9, 0.9)      # fraction of torso half-length
PAIR_Z = (-0.03, 0.03)    # absolute offset of the hip from the torso axis
FREQ = (0.9, 1.5)
AMP = (0.1, 1.0)
# Per-joint gait envelopes. The first ecosystem run let hips swing +-1.4 rad
# (bias 0.6 + amp 0.8): legs reached horizontal, every torso dropped onto its
# belly (z = torso radius, screenshot _life_t12.png) and the population lay
# sprawled and stationary for the whole epoch. A leg must stay within ~45 deg
# of its rest direction to keep carrying the body; a knee bends one way.
HIP_BIAS = (-0.10, 0.10)
HIP_AMP = (0.25, 0.42)
KNEE_BIAS = (-0.45, -0.20)
KNEE_AMP = (0.20, 0.45)
STEER = (0.15, 0.32)       # amp asymmetry; >0.3 collapses the walker (measured)
SENSE = (1.5, 7.0)
WANDER = (0.0, 1.0)

STAND_MARGIN = 0.2        # min (COM-to-support distance) / rest height at rest

DENSITY = 250.0           # kg/m^3 -> torso ~0.9 kg at 0.30 x r0.06
SPAWN_CLEAR = 0.05        # drop height above the geometric rest height
JOINTS = ("hip", "knee")
SIDES = ("L", "R")


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _creep(rng, v, sigma, lo, hi):
    return _clamp(v + rng.gauss(0.0, sigma), lo, hi)


def _finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


# ------------------------------------------------------------------ geometry
def capsule_mass(length, radius):
    """Mass of a capsule (cylinder + two hemispherical caps) at DENSITY."""
    return max(0.01, DENSITY * math.pi * radius * radius
               * (length + 4.0 * radius / 3.0))


def capsule_inertia(m, length, radius):
    """Cylinder approximation, (axial, transverse, transverse). Required
    EXPLICITLY on the Robot root: the geometry-derived inertia path EXCLUDES
    OmRobot bodies and silently substitutes a Husky preset (OmSolid.cpp:3842)."""
    axial = 0.5 * m * radius * radius
    trans = m * (3.0 * radius * radius + length * length) / 12.0
    return axial, trans, trans


def torso_mass(body):
    return capsule_mass(body["torso"]["length"], body["torso"]["radius"])


def legdrop(body):
    """Vertical reach of the longest limb at joint angle 0, hip to the end of
    its last segment's cylinder (caps excluded)."""
    return max(sum(s["length"] for s in p["segments"]) * math.cos(p["splay"])
               for p in body["pairs"])


def support_points(body):
    """Torso-frame (x, depth, label) of everything the creature can rest on
    with every joint at 0: each pair's foot bottoms and the torso's two end
    caps. depth is measured DOWN from the torso axis."""
    L, R = body["torso"]["length"], body["torso"]["radius"]
    pts = []
    for k, p in enumerate(body["pairs"]):
        drop = sum(s["length"] for s in p["segments"]) * math.cos(p["splay"])
        pts.append((p["x"] * L / 2.0, drop + p["segments"][-1]["radius"] - p["z"], "pair%d" % k))
    pts.append((L / 2.0, R, "nose"))
    pts.append((-L / 2.0, R, "tail"))
    return pts


def rest_pose(body):
    """The static rest of a creature with every joint at 0, MEASURED to be a
    pitched pose, not a level one: pairs have different leg lengths, so the
    torso rotates about y until two supports touch the floor (probe_v2, 8
    bodies: predicted vs measured torso z within 2 mm, pitch within 1.5 deg).
    The resting supports are the lower-hull edge of `support_points` that
    brackets the centre of mass (taken at the torso origin). Returns
    {z, pitch, margin, support} or None when no such edge exists -- the body
    topples. pitch is the rotation about +y (positive = nose down); margin is
    the smaller horizontal distance from the COM to a support, over the rest
    height (a stability ratio: 0.2 tolerates an 11 deg tilt)."""
    pts = support_points(body)
    best = None
    for a in range(len(pts)):
        for b in range(a + 1, len(pts)):
            (x0, d0, n0), (x1, d1, n1) = pts[a], pts[b]
            if x0 < x1:
                (x0, d0, n0), (x1, d1, n1) = (x1, d1, n1), (x0, d0, n0)
            if x0 - x1 < 1e-6:
                continue
            th = math.atan2(d1 - d0, x0 - x1)
            s, c = math.sin(th), math.cos(th)
            zc = x0 * s + d0 * c
            # every other support must stay above the floor
            if any(zc - x * s - d * c < -1e-6 for x, d, _n in pts):
                continue
            xw0, xw1 = x0 * c - d0 * s, x1 * c - d1 * s
            if not (xw1 <= 0.0 <= xw0):
                continue
            margin = min(xw0, -xw1) / max(zc, 1e-6)
            if best is None or margin > best["margin"]:
                best = {"z": zc, "pitch": th, "margin": margin, "support": (n0, n1)}
    return best


def lateral_margin(body):
    """(half stance width) / (rest height): the tip-over ratio. Half width =
    torso radius + the widest pair's lateral reach at joint 0; rest height =
    the deepest support below the torso axis."""
    R = body["torso"]["radius"]
    reach = max(R + sum(sg["length"] for sg in p["segments"]) * math.sin(p["splay"])
                for p in body["pairs"])
    h = max(d for _x, d, _l in support_points(body))
    return reach / h if h > 1e-9 else 0.0


def rest_height(body):
    """Geometric torso-centre height at static rest (see rest_pose). A body
    with no stable rest falls back to the longest-pair estimate."""
    rp = rest_pose(body)
    if rp is not None:
        return rp["z"]
    return max(body["torso"]["radius"],
               max(d for _x, d, _n in support_points(body)))


def spawn_z(body):
    """Contract: legdrop + R + 0.05 -- a short drop onto the feet."""
    return legdrop(body) + body["torso"]["radius"] + SPAWN_CLEAR


# ------------------------------------------------------------------ gait
def _fix_gait(j, kind="hip"):
    """Clamp a joint's gait to its envelope (hip or knee) and the motor limit."""
    bias_r, amp_r = (KNEE_BIAS, KNEE_AMP) if kind == "knee" else (HIP_BIAS, HIP_AMP)
    j["amp"] = abs(j["amp"])
    j["bias"] = _clamp(j["bias"], *bias_r)
    j["amp"] = _clamp(j["amp"], amp_r[0], min(amp_r[1], GAIT_CEIL - abs(j["bias"])))
    j["phase"] %= 2.0 * math.pi


def _random_joint(rng, kind="hip"):
    bias_r, amp_r = (KNEE_BIAS, KNEE_AMP) if kind == "knee" else (HIP_BIAS, HIP_AMP)
    j = {"amp": rng.uniform(*amp_r),
         "bias": rng.uniform(*bias_r),
         "phase": rng.uniform(0.0, 2.0 * math.pi)}
    _fix_gait(j, kind)
    return j


def _random_brain_pair(rng):
    return {"hip": _random_joint(rng, "hip"), "knee": _random_joint(rng, "knee")}


# Gait symmetry. With every pair free to run at any phase relative to the
# others, the CPG produces CHIRAL sequences (LF -> LB -> RF -> RB, a rotary
# gallop) and the body corkscrews: measured intrinsic spin 1.6 rad/s against a
# steering response of 0.49 rad/s, so no steering law could point it at food.
# Real gaits are symmetric -- trot (pairs in antiphase), pace (in phase), bound
# (mirror 0) -- and go straight. So inter-pair phase and the left/right mirror
# are snapped to {0, pi} plus a small evolvable skew. Knee lag relative to the
# hip is free (it is the same on both sides, so it cannot add chirality).
GAIT_SKEW_MAX = 0.25


def lock_gait(brain):
    """Pin the brain to the walker's gait STRUCTURE, in place: pairs strictly
    in antiphase (trot), left/right mirrored by pi, knee bias = -knee amp and
    knee phase = hip phase + pi/2 -- flexed while the foot swings forward,
    extended while it pushes. Amplitudes, frequency, hip bias and every body
    proportion stay free to evolve."""
    for k, p in enumerate(brain["pairs"]):
        hip, knee = p["hip"], p["knee"]
        hip["phase"] = (k * math.pi) % (2.0 * math.pi)
        knee["phase"] = (hip["phase"] + math.pi / 2.0) % (2.0 * math.pi)
        knee["bias"] = -abs(knee["amp"])
    brain["mirror_phase"] = math.pi
    return brain


def symmetrize_gait(brain):
    """Snap the brain's phase structure to an achiral gait, in place."""
    pairs = brain["pairs"]
    if not pairs:
        return brain
    ref = pairs[0]["hip"]["phase"]
    for k, p in enumerate(pairs):
        hip = p["hip"]
        rel = (hip["phase"] - ref) % (2.0 * math.pi)
        base = 0.0 if (rel < math.pi / 2.0 or rel >= 1.5 * math.pi) else math.pi
        skew = _clamp(((rel - base + math.pi) % (2.0 * math.pi)) - math.pi,
                      -GAIT_SKEW_MAX, GAIT_SKEW_MAX)
        lag = (p["knee"]["phase"] - hip["phase"]) % (2.0 * math.pi)
        hip["phase"] = (base + skew) % (2.0 * math.pi) if k else 0.0
        p["knee"]["phase"] = (hip["phase"] + lag) % (2.0 * math.pi)
    m = brain.get("mirror_phase", math.pi) % (2.0 * math.pi)
    base = 0.0 if (m < math.pi / 2.0 or m >= 1.5 * math.pi) else math.pi
    skew = _clamp(((m - base + math.pi) % (2.0 * math.pi)) - math.pi,
                  -GAIT_SKEW_MAX, GAIT_SKEW_MAX)
    brain["mirror_phase"] = (base + skew) % (2.0 * math.pi)
    return brain


# ------------------------------------------------------------------ body
def _random_segment(rng):
    return {"length": rng.uniform(*SEG_LEN), "radius": rng.uniform(*SEG_RAD)}


def _random_pair(rng, x, nseg=None, template=None):
    """A limb pair. With `template` (another pair's segments) the new legs are
    a jittered copy: MEASURED, pairs of independently random length pitch the
    torso by atan(dD/dx) and beyond ~40 deg the body flips over at rest."""
    n = nseg if nseg is not None else rng.randint(*NSEG)
    if template is not None and len(template) == n:
        segs = [{"length": _clamp(s["length"] * rng.uniform(0.9, 1.1), *SEG_LEN),
                 "radius": _clamp(s["radius"] * rng.uniform(0.9, 1.1), *SEG_RAD)}
                for s in template]
    else:
        segs = [_random_segment(rng) for _ in range(n)]
    return {"x": x, "z": rng.uniform(*PAIR_Z), "segments": segs,
            "splay": rng.uniform(0.4, SPLAY[1])}


def _pair_slots(n):
    """Evenly spread hip x-fractions, front to back, with room for jitter."""
    if n == 1:
        return [0.0]
    return [PAIR_X[1] - (PAIR_X[1] - PAIR_X[0]) * i / (n - 1) for i in range(n)]


def random_genome(rng, species, gid, pairs=None, segments=None, hue=None):
    """A fresh creature. `pairs` / `segments` pin the structure (the probes use
    2 x 2); `hue` pins the species colour (seed_species spaces them)."""
    R = rng.uniform(*TORSO_RAD)
    L = rng.uniform(*TORSO_LEN)
    n = pairs if pairs is not None else rng.randint(*NPAIR)
    plist = []
    for x in _pair_slots(n):
        plist.append(_random_pair(rng, _clamp(x + rng.uniform(-0.08, 0.08), *PAIR_X),
                                  segments,
                                  template=plist[0]["segments"] if plist else None))
    body = {
        "torso": {"length": L, "radius": R},
        "head": {"radius": R * rng.uniform(*HEAD_RATIO)},
        "pairs": plist,
        "hue": hue if hue is not None else rng.random(),
    }
    brain = {
        "freq": rng.uniform(0.8, 2.4),
        "pairs": [_random_brain_pair(rng) for _ in range(n)],
        "mirror_phase": rng.choice([math.pi, math.pi, rng.uniform(0.0, 2.0 * math.pi)]),
        "steer_gain": rng.uniform(0.3, 0.8),
        "heading_offset": 0.0,
        "sense_radius": rng.uniform(2.5, 5.5),
        "wander": rng.uniform(0.2, 0.6),
    }
    lock_gait(brain)
    return {"id": gid, "species": species, "parent": None,
            "body": body, "brain": brain}


# ------------------------------------------------------------------ mutation
def mutate_body(g, rng, new_species_id, rate=1.0, attempts=8):
    """Between epochs: gaussian creep on every morphological parameter plus
    rare structural change (add/drop a pair, add/drop a knee). The brain is
    carried over unchanged except where the structure forces a new pair; the
    result is a NEW species (the body plan differs, so no slot can host both).

    Always returns a genome that passes validate() when the parent does: a
    body that cannot stand (about 8% of single draws) is redrawn, and after
    `attempts` failures the parent's body is returned under the new id."""
    for _ in range(attempts):
        m = _mutate_body_once(g, rng, new_species_id, rate)
        if not validate(m):
            return m
    m = copy.deepcopy(g)
    m["species"] = new_species_id
    m["id"] = "%s_g0_00" % new_species_id
    m["parent"] = g["id"]
    return m


def _mutate_body_once(g, rng, new_species_id, rate):
    m = copy.deepcopy(g)
    m["species"] = new_species_id
    m["id"] = "%s_g0_00" % new_species_id
    m["parent"] = g["id"]
    b, br = m["body"], m["brain"]

    t = b["torso"]
    t["length"] = _creep(rng, t["length"], 0.03 * rate, *TORSO_LEN)
    t["radius"] = _creep(rng, t["radius"], 0.006 * rate, *TORSO_RAD)
    b["head"]["radius"] = _clamp(
        b["head"]["radius"] * (1.0 + rng.gauss(0, 0.08 * rate)),
        t["radius"] * HEAD_RATIO[0], t["radius"] * HEAD_RATIO[1])
    b["hue"] = (b["hue"] + rng.gauss(0, 0.04 * rate)) % 1.0

    for p in b["pairs"]:
        p["x"] = _creep(rng, p["x"], 0.06 * rate, *PAIR_X)
        p["z"] = _creep(rng, p["z"], 0.006 * rate, *PAIR_Z)
        p["splay"] = _creep(rng, p["splay"], 0.08 * rate, *SPLAY)
        for s in p["segments"]:
            s["length"] = _creep(rng, s["length"], 0.02 * rate, *SEG_LEN)
            s["radius"] = _creep(rng, s["radius"], 0.003 * rate, *SEG_RAD)

    # structural: knees first, then pairs
    for p in b["pairs"]:
        if len(p["segments"]) < NSEG[1] and rng.random() < 0.06 * rate:
            p["segments"].append(_random_segment(rng))
        elif len(p["segments"]) > NSEG[0] and rng.random() < 0.04 * rate:
            p["segments"].pop()
    if len(b["pairs"]) < NPAIR[1] and rng.random() < 0.08 * rate:
        b["pairs"].append(_random_pair(rng, rng.uniform(*PAIR_X)))
        br["pairs"].append(_random_brain_pair(rng))
    elif len(b["pairs"]) > NPAIR[0] and rng.random() < 0.06 * rate:
        k = rng.randrange(len(b["pairs"]))
        b["pairs"].pop(k)
        br["pairs"].pop(k)

    lock_gait(br)
    # keep pairs ordered front->back so the world writer's DEF k is stable and
    # the spacing check below has a defined neighbour
    order = sorted(range(len(b["pairs"])), key=lambda k: -b["pairs"][k]["x"])
    b["pairs"] = [b["pairs"][k] for k in order]
    br["pairs"] = [br["pairs"][k] for k in order]
    # A short torso cannot always seat three pairs: shed the rearmost pair
    # rather than hand back a genome validate() rejects.
    while not _separate_pairs(b) and len(b["pairs"]) > NPAIR[0]:
        b["pairs"].pop()
        br["pairs"].pop()
    return m


def mutate_brain(g, rng, gid, rate=1.0):
    """At birth: gaussian creep on every brain parameter, rare phase flip. The
    body is untouched (the child occupies a pooled slot authored with the
    parent species' body)."""
    m = copy.deepcopy(g)
    m["id"] = gid
    m["parent"] = g["id"]
    br = m["brain"]
    br["freq"] = _creep(rng, br["freq"], 0.18 * rate, *FREQ)
    for p in br["pairs"]:
        for name in JOINTS:
            j = p[name]
            j["amp"] += rng.gauss(0, 0.10 * rate)
            j["bias"] += rng.gauss(0, 0.10 * rate)
            j["phase"] += rng.gauss(0, 0.40 * rate)
            if rng.random() < 0.03 * rate:
                j["phase"] += math.pi
            _fix_gait(j, name)
    br["mirror_phase"] = (br["mirror_phase"] + rng.gauss(0, 0.30 * rate)) % (2.0 * math.pi)
    br["steer_gain"] = _creep(rng, br["steer_gain"], 0.10 * rate, *STEER)
    br["heading_offset"] = _clamp(br["heading_offset"] + rng.gauss(0, 0.15 * rate),
                                  -math.pi, math.pi)
    br["sense_radius"] = _creep(rng, br["sense_radius"], 0.40 * rate, *SENSE)
    br["wander"] = _creep(rng, br["wander"], 0.10 * rate, *WANDER)
    lock_gait(br)
    return m


def _separate_pairs(body):
    """Sibling limbs are NOT contact-excluded (only parent-child pairs are), so
    two hips authored closer than their capsules' radii interpenetrate at spawn
    and the solver launches them. Push neighbours apart along x."""
    L = body["torso"]["length"]
    ps = body["pairs"]
    for _ in range(4):
        moved = False
        for a, b in zip(ps, ps[1:]):
            need = _min_gap(a, b) / (L / 2.0) + 1e-6
            gap = a["x"] - b["x"]
            if gap < need:
                shift = (need - gap) / 2.0
                a["x"] = _clamp(a["x"] + shift, *PAIR_X)
                b["x"] = _clamp(b["x"] - shift, *PAIR_X)
                moved = True
        if not moved:
            return True
    return all((a["x"] - b["x"]) * L / 2.0 >= _min_gap(a, b) for a, b in zip(ps, ps[1:]))


def _min_gap(a, b):
    """Required hip spacing (m) for two pairs' first segments not to overlap."""
    return a["segments"][0]["radius"] + b["segments"][0]["radius"] + 0.02


# ------------------------------------------------------------------ validate
def validate(g):
    """Reject anything that would produce a world the solver refuses, a
    creature whose parts interpenetrate at spawn, or a gait the motors cannot
    track. Returns a list of problems (empty = ok)."""
    bad = []
    try:
        body, brain = g["body"], g["brain"]
        torso, head, pairs = body["torso"], body["head"], body["pairs"]
        bpairs = brain["pairs"]
    except (KeyError, TypeError) as exc:
        return ["malformed genome: %r" % (exc,)]

    def rng_check(name, v, lo, hi):
        if not _finite(v):
            bad.append("%s non-finite: %r" % (name, v))
        elif not (lo - 1e-9 <= v <= hi + 1e-9):
            bad.append("%s = %.4f outside [%.3f, %.3f]" % (name, v, lo, hi))

    rng_check("torso.length", torso.get("length"), *TORSO_LEN)
    rng_check("torso.radius", torso.get("radius"), *TORSO_RAD)
    if _finite(torso.get("radius")) and _finite(head.get("radius")):
        rng_check("head.radius/torso.radius",
                  head["radius"] / torso["radius"], *HEAD_RATIO)
    if not (NPAIR[0] <= len(pairs) <= NPAIR[1]):
        bad.append("pair count %d outside [%d, %d]" % (len(pairs), NPAIR[0], NPAIR[1]))
    if len(bpairs) != len(pairs):
        bad.append("brain has %d pairs, body has %d" % (len(bpairs), len(pairs)))

    for k, p in enumerate(pairs):
        rng_check("pair%d.x" % k, p.get("x"), *PAIR_X)
        rng_check("pair%d.z" % k, p.get("z"), *PAIR_Z)
        rng_check("pair%d.splay" % k, p.get("splay"), *SPLAY)
        segs = p.get("segments", [])
        if not (NSEG[0] <= len(segs) <= NSEG[1]):
            bad.append("pair%d segment count %d outside [%d, %d]"
                       % (k, len(segs), NSEG[0], NSEG[1]))
        for s_i, s in enumerate(segs):
            rng_check("pair%d.seg%d.length" % (k, s_i), s.get("length"), *SEG_LEN)
            rng_check("pair%d.seg%d.radius" % (k, s_i), s.get("radius"), *SEG_RAD)
    for a_i, (a, b) in enumerate(zip(pairs, pairs[1:])):
        if all(_finite(v) for v in (a.get("x"), b.get("x"))) and a["segments"] and b["segments"]:
            gap = (a["x"] - b["x"]) * torso["length"] / 2.0
            if gap < _min_gap(a, b) - 1e-6:
                bad.append("pairs %d/%d hips %.3f m apart, need %.3f (overlap at spawn)"
                           % (a_i, a_i + 1, gap, _min_gap(a, b)))

    if not bad:
        # Static standability, on the same geometry the world writer authors.
        rp = rest_pose(body)
        if rp is None:
            bad.append("cannot stand: no support edge brackets the COM (topples at rest)")
        elif rp["margin"] < STAND_MARGIN:
            bad.append("cannot stand: stability margin %.2f < %.2f (support %s/%s, pitch %.0f deg)"
                       % (rp["margin"], STAND_MARGIN, rp["support"][0], rp["support"][1],
                          math.degrees(rp["pitch"])))
        lm = lateral_margin(body)
        if lm < LATERAL_MARGIN:
            bad.append("tips over: lateral margin %.2f < %.2f (stance too narrow for its height)"
                       % (lm, LATERAL_MARGIN))

    rng_check("brain.freq", brain.get("freq"), *FREQ)
    for k, p in enumerate(bpairs):
        for name in JOINTS:
            j = p.get(name)
            if not isinstance(j, dict):
                bad.append("brain pair%d missing %s" % (k, name))
                continue
            rng_check("brain.pair%d.%s.amp" % (k, name), j.get("amp"), *AMP)
            rng_check("brain.pair%d.%s.bias" % (k, name), j.get("bias"), -BIAS_MAX, BIAS_MAX)
            rng_check("brain.pair%d.%s.phase" % (k, name), j.get("phase"), -2 * math.pi, 4 * math.pi)
            if _finite(j.get("amp")) and _finite(j.get("bias")):
                tot = abs(j["bias"]) + abs(j["amp"])
                if tot > GAIT_CEIL + 1e-9:
                    bad.append("brain.pair%d.%s |bias|+|amp| = %.3f > %.2f (motor limit %.1f)"
                               % (k, name, tot, GAIT_CEIL, JOINT_LIMIT))
    rng_check("brain.mirror_phase", brain.get("mirror_phase"), -2 * math.pi, 4 * math.pi)
    rng_check("brain.steer_gain", brain.get("steer_gain"), *STEER)
    rng_check("brain.heading_offset", brain.get("heading_offset"), -math.pi, math.pi)
    rng_check("brain.sense_radius", brain.get("sense_radius"), *SENSE)
    rng_check("brain.wander", brain.get("wander"), *WANDER)
    rng_check("body.hue", body.get("hue"), 0.0, 1.0)
    return bad


# ------------------------------------------------------------------ misc
def joint_count(g):
    """Hinges the world writer will author: 2 sides x segments per pair."""
    return 2 * sum(len(p["segments"]) for p in g["body"]["pairs"])


def describe(g):
    b, br = g["body"], g["brain"]
    segs = ",".join(str(len(p["segments"])) for p in b["pairs"])
    return ("%s[%s] torso=%.2fx%.3f pairs=%d[%s] splay=%.2f f=%.2f mirror=%.2f "
            "steer=%.2f sense=%.1f wander=%.2f" % (
                g["id"], g["species"], b["torso"]["length"], b["torso"]["radius"],
                len(b["pairs"]), segs,
                sum(p["splay"] for p in b["pairs"]) / len(b["pairs"]),
                br["freq"], br["mirror_phase"], br["steer_gain"],
                br["sense_radius"], br["wander"]))


def seed_species(rng, k, pairs=None, segments=None, tag="sp"):
    """k founder genomes, one species each, hues evenly spaced so species are
    tellable apart on the frame. Every genome passes validate()."""
    out = []
    while len(out) < k:
        i = len(out)
        sp = "%s%d" % (tag, i)
        g = random_genome(rng, sp, "%s_g0_00" % sp, pairs=pairs, segments=segments,
                          hue=((i + 0.5) / k + rng.uniform(-0.03, 0.03)) % 1.0)
        _separate_pairs(g["body"])
        if not validate(g):
            out.append(g)
    return out


if __name__ == "__main__":
    rng = random.Random(3)
    for g in seed_species(rng, 4):
        print(describe(g))
        c = mutate_brain(g, rng, g["id"].replace("_00", "_01"))
        assert not validate(c), validate(c)
        s = mutate_body(g, rng, g["species"] + "b")
        assert not validate(s), validate(s)
        print("   rest %.3f spawn %.3f joints %d" % (rest_height(g["body"]),
                                                   spawn_z(g["body"]), joint_count(g)))
