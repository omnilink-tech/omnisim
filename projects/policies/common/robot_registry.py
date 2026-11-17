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

"""THE ROBOT REGISTRY -- the one place that knows what a robot IS.

Shadowing is a robot-general method, but the code that implemented it was not: every
model-aware tool resolved the *G1's* URDF, and every role/symmetry rule was keyed on the
*G1's* joint names. The failure mode this produces is not a crash -- it is a VACUOUS PASS:

  * (fixed 2026-07-12) the H1 ghost was limit-checked against the G1's URDF, and its
    unknown joints defaulted to no-limit -- so it "passed" a gate that never ran.
  * (fixed here) a ghost for a robot with NO registered URDF skipped the model-aware
    gates entirely and still printed VERDICT: PASS. The Go2 shadow ghost -- the one
    artifact that makes the quadruped `method: shadowing` claim true -- passed exactly
    this way. The earlier fix closed *wrong-model*; it did not close *no-model*.
  * to get a humanoid-name-keyed symmetry gate to engage at all, the Go2 ghost builder
    wrote FAKE humanoid joint names ("FL_hip_roll_joint") into the lut and hid the real
    ones ("FL_hip_joint") under a side key. A rule you have to lie to is the wrong rule.

So: a robot is DATA (`ROBOTS`), and a joint's role is DERIVED FROM ITS NAME by a law that
spans the naming families we actually ship -- not by a hardcoded index list.

    unitree humanoid : left_/right_  + hip_roll|hip_pitch|hip_yaw|knee|ankle_pitch|...
    unitree quadruped: FL_/FR_/RL_/RR_ + hip(=abduction/roll) |thigh(=pitch)|calf(=knee)
    boston dynamics  : front_left_/... + hip_x(=roll)|hip_y(=pitch)|knee

The two consumers that must never disagree:
  * `training/ghost_validator.py` -- fail-closed model gates + the mirror-symmetry gate.
  * `training/g1_walk_recipe.py::_leg_roles` -- the corridor roles (roll stabilizers are
    never gated; the pitch plane is). It asserts at import that this module reproduces the
    G1 literals exactly, so a change here cannot silently move the flagship.

UNKNOWN IS AN ERROR, NOT A DEFAULT. `joint_limits()` raises `UnknownRobot` rather than
returning {} -- callers must turn that into a FAILED gate, never a skipped one.
"""

from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET

_RT = os.environ.get("OMNISIM_HOME", ".")

HUMANOID = "humanoid"
QUADRUPED = "quadruped"

# ── the registry ─────────────────────────────────────────────────────────────
# `urdf` is the model the LIMIT gates read. For the G1 it is resolved through the physics
# SPEC (projects/policies/research/backends/g1_physics.json) so the validator and the
# trainer cannot drift apart -- see docs/developer/g1-single-source-of-truth.md.
ROBOTS: dict[str, dict] = {
    "g1":  {"class": HUMANOID,  "urdf": None,  # via _g1_spec_urdf()
            "spec": "projects/policies/research/backends/g1_physics.json",
            "mjcf": "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf"},
    "h1":  {"class": HUMANOID,  "urdf": "projects/robots/unitree/h1/urdf/h1.urdf"},
    "go2": {"class": QUADRUPED, "urdf": "projects/robots/unitree/go2/urdf/go2.urdf"},
    "b2":  {"class": QUADRUPED, "urdf": "projects/robots/unitree/b2/urdf/b2.urdf"},
    "omniquad": {"class": QUADRUPED, "urdf": "projects/robots/omnisim/omniquad/urdf/omniquad.urdf"},
}


class UnknownRobot(KeyError):
    """The lut names a robot the registry does not know. Fail closed: do NOT validate."""


class ModelUnavailable(RuntimeError):
    """The robot is registered but its model could not be read. Fail closed."""


def is_known(robot: str) -> bool:
    return str(robot or "").strip().lower() in ROBOTS


def robot_class(robot: str) -> str:
    r = str(robot or "").strip().lower()
    if r not in ROBOTS:
        raise UnknownRobot(r)
    return ROBOTS[r]["class"]


def _g1_spec_urdf() -> str:
    spec = json.load(open(os.path.join(_RT, ROBOTS["g1"]["spec"])))
    u = spec["urdf"]
    if isinstance(u, dict):
        u = u.get("prim") or u.get("full")
    return u


def robot_urdf(robot: str) -> str:
    """Absolute path to the robot's URDF. Raises UnknownRobot / ModelUnavailable."""
    r = str(robot or "").strip().lower()
    if r not in ROBOTS:
        raise UnknownRobot(
            f"robot {r!r} is not in the registry ({', '.join(sorted(ROBOTS))}). "
            f"Add it to projects/policies/common/robot_registry.py -- an unregistered "
            f"robot CANNOT be model-checked, and an unchecked ghost is not a valid ghost.")
    try:
        u = _g1_spec_urdf() if r == "g1" else ROBOTS[r]["urdf"]
        p = u if os.path.isabs(u) else os.path.join(_RT, u)
        if not os.path.exists(p):
            raise ModelUnavailable(f"{r}: URDF not found at {p}")
        return p
    except ModelUnavailable:
        raise
    except Exception as e:                                   # spec unreadable, bad json...
        raise ModelUnavailable(f"{r}: model unavailable ({e!r})") from e


def total_mass(robot: str) -> float:
    """Total mass (kg) summed from the URDF's link inertials. Fail-closed."""
    import xml.etree.ElementTree as ET
    urdf = robot_urdf(robot)
    try:
        root = ET.parse(urdf).getroot()
        m = sum(float(x.get("value")) for x in root.iter("mass"))
    except Exception as e:
        raise ModelUnavailable(f"{robot}: cannot sum link masses from {urdf} ({e!r})") from e
    if not (m > 0):
        raise ModelUnavailable(f"{robot}: URDF {urdf} declares no mass")
    return m


def foot_half_extent(robot: str) -> tuple[float, float]:
    """(half_length, half_width) in metres of the foot's ground-contact box -- the CoP lever arms.

    This is the geometry that bounds ANKLE BALANCE AUTHORITY: to hold yourself up on one foot you must
    drive the centre of pressure toward the edge of the sole, and the furthest it can go IS this lever.
    It is what makes the corridor-adequacy law computable before training (see corridor_min_rad).

    Fail-closed: a robot whose foot we cannot measure cannot be corridor-checked, and an unchecked
    corridor is exactly how every shipped G1 skill ended up capped at 70% of its own ankle authority.
    """
    import xml.etree.ElementTree as ET
    r = str(robot or "").strip().lower()
    if r not in ROBOTS:
        raise UnknownRobot(r)
    # 1. the physics SPEC wins when it declares a contact box (G1: model.foot_box) -- that IS the
    #    collider the engine simulates, which may not equal the visual mesh.
    spec_rel = ROBOTS[r].get("spec")
    if spec_rel:
        try:
            spec = json.load(open(os.path.join(_RT, spec_rel)))
            size = spec.get("model", {}).get("foot_box", {}).get("size")
            if size and len(size) >= 2:
                return float(size[0]) / 2.0, float(size[1]) / 2.0
        except Exception:
            pass
    # 2. otherwise measure the largest box collider on a foot/ankle link in the URDF.
    urdf = robot_urdf(r)
    try:
        root = ET.parse(urdf).getroot()
        best = None
        for link in root.iter("link"):
            nm = (link.get("name") or "").lower()
            if not any(k in nm for k in ("foot", "ankle_roll", "sole", "toe")):
                continue
            for col in link.iter("collision"):
                for box in col.iter("box"):
                    d = [float(v) for v in (box.get("size") or "").split()]
                    if len(d) == 3 and (best is None or d[0] * d[1] > best[0] * best[1]):
                        best = d
        if best:
            return best[0] / 2.0, best[1] / 2.0
    except Exception as e:
        raise ModelUnavailable(f"{r}: cannot read foot geometry from {urdf} ({e!r})") from e
    raise ModelUnavailable(
        f"{r}: no foot contact box found (no spec model.foot_box, no box collider on a foot/ankle "
        f"link in {urdf}). Declare one before corridor-checking a ghost for this robot.")


def corridor_min_rad(robot: str, kp: float = 200.0) -> dict:
    """THE CORRIDOR-ADEQUACY LAW (measured 2026-07-13).

    A Shadowing policy can only express torque as a position offset through the PD plant: dq = tau/kp.
    To balance on the ankle it must drive the centre of pressure to the edge of the sole, which costs

        tau_cop = m * g * (foot_length / 2)          G1: 34.1 kg * 9.81 * 0.085 m = 28.5 N*m
        dq_cop  = tau_cop / kp                       G1 at kp=200:  0.142 rad

    If GHOST_RESIDUAL < dq_cop the policy is STRUCTURALLY FORBIDDEN from commanding full ankle
    authority -- it cannot reach the edge of its own foot -- and whatever it is denied, the crane
    supplies. Every shipped G1 skill grants 0.100 rad = 70% of dq_cop.

    This is INDEPENDENT of, and additional to, GHOST_FF: the feedforward fixes the corridor's CENTRE
    (it must carry the reference's own stance torque, tau_ff/kp); this fixes its WIDTH (it must carry
    the balance corrections, and it must do so EARLY -- a clipped corrector cannot fix an error while
    it is small, so the error grows until the correction it needs is one the corridor refuses).

    Measured correlation on the free-standing campaign (identical ghost, identical iteration):
        0.120 rad =  84% of dq_cop -> 43.6% of action-dims SATURATED, crane lean 25%, stalled
        0.240 rad = 169% of dq_cop ->  0.0% saturated, crane lean 6.4%, gmatch BETTER (0.931->0.946)
    """
    m = total_mass(robot)
    half_len, half_wid = foot_half_extent(robot)
    g = 9.81
    tau_pitch = m * g * half_len
    tau_roll = m * g * half_wid
    return {
        "robot": robot, "mass_kg": m, "kp": float(kp),
        "foot_half_len_m": half_len, "foot_half_wid_m": half_wid,
        "tau_cop_pitch_nm": tau_pitch, "tau_cop_roll_nm": tau_roll,
        "dq_cop_pitch_rad": tau_pitch / float(kp),     # the MINIMUM defensible corridor
        "dq_cop_roll_rad": tau_roll / float(kp),
        "recommended_rad": 1.5 * tau_pitch / float(kp),  # margin for tracking error + dynamics
    }


def joint_limits(robot: str) -> dict[str, tuple[float, float, float]]:
    """name -> (lower, upper, velocity) from the robot's own URDF.

    Fail-closed: raises rather than returning {}. A caller that cannot get limits must
    FAIL the ghost, not skip the gate.
    """
    urdf = robot_urdf(robot)
    try:
        root = ET.parse(urdf).getroot()
    except Exception as e:
        raise ModelUnavailable(f"{robot}: URDF unparseable ({e!r})") from e
    lim = {}
    for j in root.iter("joint"):
        nm, li = j.get("name"), j.find("limit")
        if nm and li is not None:
            lim[nm] = (float(li.get("lower", -99)), float(li.get("upper", 99)),
                       float(li.get("velocity", 99)))
    if not lim:
        raise ModelUnavailable(f"{robot}: URDF {urdf} declares no limited joints")
    return lim


# ── joint-name law ───────────────────────────────────────────────────────────
# side: "L" | "R" (None if the joint is not lateral -- torso/waist).
# row : "front" | "rear" (quadrupeds only; None on a humanoid).
# axis: the ROLE in the corridor/symmetry rules. The two that matter everywhere:
#         "roll"  -- the LATERAL stabilizer. Never gated by the stance mask: lateral
#                    balance is applied by the PLANTED leg.
#         pitch-plane ("pitch"/"knee"/"ankle_pitch") -- gated during stance.
_SIDE_PREFIX = [
    ("left_", "L", None), ("right_", "R", None),
    ("front_left_", "L", "front"), ("front_right_", "R", "front"),
    ("rear_left_", "L", "rear"), ("rear_right_", "R", "rear"),
    ("FL_", "L", "front"), ("FR_", "R", "front"),
    ("RL_", "L", "rear"), ("RR_", "R", "rear"),
]
# longest prefix first: "front_left_" must beat "left_" would-be substring confusion.
_SIDE_PREFIX.sort(key=lambda t: -len(t[0]))


def side_row(name: str) -> tuple[str | None, str | None]:
    for pfx, side, row in _SIDE_PREFIX:
        if name.startswith(pfx):
            return side, row
    return None, None


def axis_of(name: str) -> str:
    """The joint's kinematic role, from its name. Spans the shipped naming families.

    ⛔ MATCH WHOLE TOKENS, NEVER SUBSTRINGS. `"hip_y" in "left_hip_yaw_joint"` is TRUE --
    a substring rule classifies the G1's hip-YAW as OmniQuad's hip-Y (pitch), silently drags it
    into the pitch-plane symmetry gate, and drops the flagship walk ghost from PASS to WARN.
    Caught by the old-vs-new verdict diff; do not "simplify" this back to `in`.
    """
    t = set(name.lower().split("_"))
    if "ankle" in t:
        return "ankle_roll" if "roll" in t else "ankle_pitch"
    if "knee" in t or "calf" in t:                  # unitree quad: calf == knee
        return "knee"
    if "thigh" in t:                                # unitree quad: thigh == hip flexion
        return "pitch"
    if "yaw" in t:
        return "yaw"
    if "roll" in t:
        return "roll"
    if "pitch" in t:
        return "pitch"
    if "hip" in t and "x" in t:                     # boston dynamics: hip_x = abduction
        return "roll"
    if "hip" in t and "y" in t:                     # boston dynamics: hip_y = flexion
        return "pitch"
    if "hip" in t:                                  # bare unitree-quad hip = abduction
        return "roll"
    return "unknown"


PITCH_PLANE = ("pitch", "knee", "ankle_pitch")


def is_pitch_plane(name: str) -> bool:
    return axis_of(name) in PITCH_PLANE


def is_roll(name: str) -> bool:
    return axis_of(name) in ("roll", "ankle_roll")


def mirror_joint(name: str) -> str | None:
    """The joint's counterpart across the sagittal plane, or None if it has none.

    left_knee_joint <-> right_knee_joint ;  FL_calf_joint <-> FR_calf_joint
    front_left_hip_x <-> front_right_hip_x  (row is PRESERVED: a quad mirrors L/R, not F/R)
    """
    swaps = [("left_", "right_"), ("front_left_", "front_right_"), ("rear_left_", "rear_right_"),
             ("FL_", "FR_"), ("RL_", "RR_")]
    swaps.sort(key=lambda t: -len(t[0]))
    for a, b in swaps:
        if name.startswith(a):
            return b + name[len(a):]
        if name.startswith(b):
            return a + name[len(b):]
    return None


def leg_roles(names):
    """-> (n, roll_idx, yaw_idx, pitch_mask, per_leg) -- the corridor roles, from NAMES.

    pitch_mask is 1.0 on the pitch-plane joints and 0.0 on the roll stabilizers. This is
    THE law the trainer's corridor and stance gate are built from; it reproduces the G1's
    hand-written literals exactly (asserted at import in g1_walk_recipe.py) and derives the
    same roles for the H1 and for a quadruped without a line of per-robot code.
    """
    import numpy as np
    n = len(names)
    roll_idx = [i for i, j in enumerate(names) if is_roll(j)]
    yaw_idx = [i for i, j in enumerate(names) if axis_of(j) == "yaw"]
    pitch_mask = np.array([0.0 if is_roll(j) else 1.0 for j in names], np.float32)
    if n % 2:
        raise RuntimeError(f"ghost declares an odd number of leg joints ({n}): {names}")
    n_legs = 4 if any(side_row(j)[1] for j in names) else 2
    return n, roll_idx, yaw_idx, pitch_mask, n // n_legs
