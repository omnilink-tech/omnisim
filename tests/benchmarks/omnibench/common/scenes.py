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

"""OmniBench lane-1 scene constants — the single source of truth for T1-T7.

Every lane (raw MuJoCo, raw PyBullet, OmniSim) imports THIS module for scene
parameters, masses and inertias; score.py imports it for the analytic
references. Do not re-declare a number from SPEC.md anywhere else.

Shared geometric conventions (part of the cross-engine contract):

* Gravity is (0, 0, -GRAVITY). SI units. Seed = SEED where applicable.
* Inclines (T2/T3): the incline is a static box of half-extents
  ``INCLINE_HALF`` rotated about the world +y axis by +theta degrees
  (right-handed), so **downhill is +x** and the surface normal is
  ``(sin(theta), 0, cos(theta))``. Lane k sits at y = k * T2.lane_spacing.
  The sliding/rolling body starts at slope coordinate ``s0`` (negative =
  uphill of the incline centre) resting exactly on the surface — use
  :func:`incline_body_start`.
* Chain (T4/T5): 3 links along **local +x** (capsule long axis = local x),
  hinge axes local +y, built horizontal along world +x. Link COM x positions
  at t=0 are 0.25, 0.75, 1.25 m. Body local frames sit AT the link COM and
  are principal inertia frames with diagonal inertia ``T4.link_inertia_diag``
  (I_axial on x). Runners must set this inertia explicitly in the engine so
  the scorer's energy/momentum bookkeeping matches the simulated dynamics.
* Recorded body names per test are in ``BODIES[test]`` — all engines must
  use exactly these names in the recording .npz (see common/recording.py).
"""

import math

GRAVITY = 9.81          # m/s^2, applied as (0, 0, -GRAVITY)
SEED = 42
DT_SWEEP_MS = (1, 2, 4, 8, 16, 32)
DT_REFERENCE_MS = 0.25  # numeric ground-truth check of the analytic formulas

TESTS = ("T1", "T2", "T3", "T4", "T5", "T6", "T7")
TEST_NAMES = {
    "T1": "bounce",
    "T2": "incline",
    "T3": "roll",
    "T4": "pendulum_energy",
    "T5": "momentum",
    "T6": "stack",
    "T7": "spin",
}
NAME_TO_TEST = {v: k for k, v in TEST_NAMES.items()}


def normalize_test_id(s):
    """Accept 'T1', 't1' or 'bounce' -> 'T1'. Raises ValueError otherwise."""
    s = str(s).strip()
    up = s.upper()
    if up in TESTS:
        return up
    lo = s.lower()
    if lo in NAME_TO_TEST:
        return NAME_TO_TEST[lo]
    raise ValueError("unknown test id %r (want T1..T7 or one of %s)"
                     % (s, sorted(NAME_TO_TEST)))


# ---------------------------------------------------------------------------
# inertia helpers (uniform-density rigid bodies, about COM, principal axes)
# ---------------------------------------------------------------------------

def solid_sphere_inertia(m, r):
    """Scalar I = (2/5) m r^2 (same about every axis)."""
    return 0.4 * m * r * r


def box_inertia_diag(m, full_sizes):
    """Diagonal inertia (Ixx, Iyy, Izz) of a solid box with FULL side lengths."""
    lx, ly, lz = full_sizes
    return (m * (ly * ly + lz * lz) / 12.0,
            m * (lx * lx + lz * lz) / 12.0,
            m * (lx * lx + ly * ly) / 12.0)


def capsule_inertia(m, r, l):
    """(I_axial, I_perp) of a solid capsule: cylinder length l, cap radius r,
    total mass m, uniform density. I_axial is about the long axis."""
    v_cyl = math.pi * r * r * l
    v_hemi = (2.0 / 3.0) * math.pi * r ** 3          # one hemisphere
    v_tot = v_cyl + 2.0 * v_hemi
    m_cyl = m * v_cyl / v_tot
    m_hemi = m * v_hemi / v_tot
    i_axial = 0.5 * m_cyl * r * r + 2.0 * m_hemi * (2.0 / 5.0) * r * r
    i_perp = (m_cyl * (l * l / 12.0 + r * r / 4.0)
              + 2.0 * m_hemi * (2.0 * r * r / 5.0 + l * l / 4.0
                                + 3.0 * l * r / 8.0))
    return i_axial, i_perp


# ---------------------------------------------------------------------------
# incline geometry (T2/T3 shared)
# ---------------------------------------------------------------------------

INCLINE_HALF = (10.0, 1.0, 0.05)   # half-extents of the static incline box
INCLINE_CENTER_Z = 2.0             # world z of the incline box centre
INCLINE_S0 = -8.0                  # start slope coordinate (uphill of centre)


def incline_frame(theta_deg):
    """(downhill_unit, normal_unit) world vectors for an incline rotated
    about +y by +theta_deg (downhill = +x by convention)."""
    th = math.radians(theta_deg)
    downhill = (math.cos(th), 0.0, -math.sin(th))
    normal = (math.sin(th), 0.0, math.cos(th))
    return downhill, normal


def incline_body_start(theta_deg, lane_y, body_clearance, s0=INCLINE_S0):
    """World position of a body resting on the incline surface at slope
    coordinate s0. body_clearance = distance from surface to body centre
    (half-height of a box face-down, or sphere radius)."""
    d, n = incline_frame(theta_deg)
    off = INCLINE_HALF[2] + body_clearance   # surface is half-thickness above centre
    return (s0 * d[0] + off * n[0],
            lane_y + s0 * d[1] + off * n[1],
            INCLINE_CENTER_Z + s0 * d[2] + off * n[2])


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# FLOOR_TOP — why the contact scenes do not sit on z = 0
# ---------------------------------------------------------------------------
# `OmNewtonBackend::openWorld()` adds an implicit ground plane at z = 0
# UNCONDITIONALLY. A scene whose own floor's top face is also at z = 0 therefore
# resolves its contacts against TWO coincident manifolds, which is a confound in
# exactly the scenes that measure contact: T1 (restitution) and T6 (stacking).
# The lane-1 validity audit (docs/benchmarks/lane1-validity-2026-08-07.md §6)
# flagged it, and noted that T1's contact-damping calibration had been fitted
# against that doubled stiffness.
#
# Lifting the floor off z = 0 removes the second manifold. It is a pure vertical
# TRANSLATION of the whole scene -- drop height, gaps and every relative distance
# are unchanged -- so the analytic references are untouched; only absolute z
# moves, and the scorer subtracts FLOOR_TOP for the same reason.
FLOOR_TOP = 0.5
FLOOR_THICK = 0.2

# T1 bounce — restitution fidelity
# ---------------------------------------------------------------------------

class T1:
    name = "bounce"
    sphere_r = 0.1
    mass = 1.0
    start_z = FLOOR_TOP + 0.1 + 1.0   # rest is FLOOR_TOP + r; drop height h0 = 1.0
    drop_h = 1.0
    restitution = 0.8
    friction = 0.0
    n_peaks = 5
    duration = 4.0           # enough sim-time for >= 5 peaks (~2.9 s analytic)
    body = "ball"
    inertia = solid_sphere_inertia(mass, sphere_r)

    @staticmethod
    def analytic_peaks():
        """Peak heights h_n = h0 * e^(2n), n = 1..n_peaks (metres above rest)."""
        return [T1.drop_h * T1.restitution ** (2 * n)
                for n in range(1, T1.n_peaks + 1)]


# ---------------------------------------------------------------------------
# T2 incline — friction-cone stick/slip
# ---------------------------------------------------------------------------

class T2:
    name = "incline"
    box_full = (0.2, 0.2, 0.2)
    box_half = 0.1
    mass = 1.0
    mu = 0.5
    theta_c_deg = math.degrees(math.atan(mu))     # 26.565...
    angles_deg = (15.0, 20.0, 25.0, 26.0, 27.0, 30.0, 35.0)
    duration = 3.0
    lane_spacing = 3.0        # lane k at y = k * lane_spacing
    stick_thresh_m = 0.001    # < 1 mm displacement over 3 s = stick
    slip_classify_m = 0.01    # > 1 cm tangential displacement = slip (for
                              # the transition-angle bracket)
    settle_t = 0.3            # displacement measured from here (skips the
                              # soft-contact normal settle)
    fit_t0, fit_t1 = 0.1, 1.1  # tangential-speed linear fit window (1 s)
    inertia_diag = box_inertia_diag(mass, box_full)

    @staticmethod
    def bodies():
        return ["box%d" % int(a) for a in T2.angles_deg]

    @staticmethod
    def body_angle(body_name):
        return float(body_name.replace("box", ""))

    @staticmethod
    def slide_accel(theta_deg):
        """Analytic slide acceleration g(sin th - mu cos th) for th > th_c."""
        th = math.radians(theta_deg)
        return GRAVITY * (math.sin(th) - T2.mu * math.cos(th))


# ---------------------------------------------------------------------------
# T3 roll — rolling without slip
# ---------------------------------------------------------------------------

class T3:
    name = "roll"
    sphere_r = 0.1
    mass = 1.0
    mu = 0.8
    theta_deg = 20.0
    duration = 2.0
    fit_t0, fit_t1 = 0.1, 1.1
    slip_eval_t = 1.0
    inertia = solid_sphere_inertia(mass, sphere_r)
    body = "ball"
    accel_analytic = (5.0 / 7.0) * GRAVITY * math.sin(math.radians(theta_deg))


# ---------------------------------------------------------------------------
# T4 pendulum_energy — articulated energy conservation
# ---------------------------------------------------------------------------

class T4:
    name = "pendulum_energy"
    n_links = 3
    link_l = 0.5              # cylinder length; also the joint-to-joint spacing
    link_r = 0.02
    link_m = 1.0
    duration = 10.0
    bodies = ("link1", "link2", "link3")
    # capsule long axis along local +x  ->  diag = (I_axial, I_perp, I_perp)
    _iax, _iperp = capsule_inertia(link_m, link_r, link_l)
    link_inertia_diag = (_iax, _iperp, _iperp)
    # initial (horizontal, along +x) link COM x positions, pivot at origin
    com_x0 = (0.25, 0.75, 1.25)


# ---------------------------------------------------------------------------
# T5 momentum — floating-base momentum conservation (gravity OFF)
# ---------------------------------------------------------------------------

class T5:
    name = "momentum"
    # same chain as T4, free-floating, no pivot; 2 internal hinges.
    # "middle joint" is interpreted as the link1-link2 hinge (j12) — with 3
    # links there are only 2 internal joints; both engines use the same one.
    duration = 10.0
    drive_t = 2.0             # sinusoidal torque window
    tau_amp = 5.0             # N*m
    tau_freq = 1.0            # Hz  (tau = 5 sin(2 pi t))
    bodies = T4.bodies
    link_inertia_diag = T4.link_inertia_diag

    @staticmethod
    def tau(t):
        if t < T5.drive_t:
            return T5.tau_amp * math.sin(2.0 * math.pi * T5.tau_freq * t)
        return 0.0


# ---------------------------------------------------------------------------
# T6 stack — multi-contact stability
# ---------------------------------------------------------------------------

class T6:
    name = "stack"
    n_boxes = 10
    box_full = (0.1, 0.1, 0.1)
    box_half = 0.05
    mass = 0.5
    mu = 0.8
    restitution = 0.0
    gap = 0.001
    duration = 10.0
    survivor_xy_m = 0.02      # within 2 cm of start XY
    creep_window = 5.0        # top-box drift over the last 5 s
    inertia_diag = box_inertia_diag(mass, box_full)

    @staticmethod
    def bodies():
        return ["box%d" % k for k in range(T6.n_boxes)]  # box0 = bottom

    @staticmethod
    def start_z(k):
        """Centre z of box k (0 = bottom), 1 mm gaps, measured from FLOOR_TOP."""
        return FLOOR_TOP + T6.box_half + k * (2 * T6.box_half + T6.gap)


# ---------------------------------------------------------------------------
# T7 spin — Dzhanibekov / gyroscopic integration (gravity OFF)
# ---------------------------------------------------------------------------

class T7:
    name = "spin"
    box_full = (0.4, 0.2, 0.1)
    mass = 1.0
    omega0 = (0.01, 5.0, 0.01)   # rad/s, about the unstable intermediate axis
    duration = 10.0
    body = "box"
    inertia_diag = box_inertia_diag(mass, box_full)  # Ix<Iy<Iz, y intermediate


BODIES = {
    "T1": [T1.body],
    "T2": T2.bodies(),
    "T3": [T3.body],
    "T4": list(T4.bodies),
    "T5": list(T5.bodies),
    "T6": T6.bodies(),
    "T7": [T7.body],
}

DURATIONS = {
    "T1": T1.duration, "T2": T2.duration, "T3": T3.duration,
    "T4": T4.duration, "T5": T5.duration, "T6": T6.duration,
    "T7": T7.duration,
}
