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

"""SINGLE SOURCE OF TRUTH loader for the G1 Newton walk train<->deploy model.

This module is the ONE place the trainer and the deploy derive their physics
model from. It combines two source-of-truth artifacts:

  1. ``g1_physics.json``  -- the tunable physics knobs (solver, gains, friction,
     clamp, contract scalars). Human/CI editable, language-neutral.
  2. the prim URDF        -- geometry, inertia, and the per-joint position /
     velocity / effort limits, read LIVE (never transcribed).

Consumers:
  * trainer  -> ``from projects.policies.research.backends import g1_physics_spec as SPEC``
                and read SPEC.KE / SPEC.SUBSTEPS / SPEC.leg_limits() etc.
  * deploy   -> ``scripts/dev/g1_deploy_launch.py`` calls ``SPEC.newton_env()``
                to emit the OMNISIM_NEWTON_* env block the backend reads.
  * conformance test -> asserts a built model matches SPEC.

Joint ORDER is imported from g1_robot_spec (declared "FIXED FOREVER" there) so
there is exactly one source for ordering too.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from projects.policies.research.backends.g1_robot_spec import JOINT_NAMES

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
_SPEC_JSON = Path(__file__).resolve().parent / "g1_physics.json"

# ----------------------------------------------------------------------------
# Load the JSON knobs.
# ----------------------------------------------------------------------------
with _SPEC_JSON.open("r", encoding="utf-8") as _fh:
    _SPEC: dict = json.load(_fh)

# ----------------------------------------------------------------------------
# Joint order (the ONE source: g1_robot_spec). Legs+waist first 13, arms last 10.
# ----------------------------------------------------------------------------
LEGS_JOINTS: Tuple[str, ...] = JOINT_NAMES[:13]
ARM_JOINTS: Tuple[str, ...] = JOINT_NAMES[13:]
assert len(LEGS_JOINTS) == 13 and len(ARM_JOINTS) == 10

# ----------------------------------------------------------------------------
# Flat physics constants (mirror the JSON; consumers read THESE, never literals).
# ----------------------------------------------------------------------------
SOLVER_KIND: str = _SPEC["solver"]["kind"]
SUBSTEPS: int = int(_SPEC["solver"]["substeps"])
DT: float = float(_SPEC["solver"]["dt"])
NJMAX: int = int(_SPEC["solver"]["njmax"])
NCONMAX: int = int(_SPEC["solver"]["nconmax"])

KE: float = float(_SPEC["gains"]["ke"])
KD: float = float(_SPEC["gains"]["kd"])
ARMATURE: float = float(_SPEC["gains"]["armature"])

GROUND_MU: float = float(_SPEC["contact"]["ground_mu"])
CONTACT_KE: float = float(_SPEC["contact"]["contact_ke"])
CONTACT_KD: float = float(_SPEC["contact"]["contact_kd"])

USE_INERTIA: bool = bool(_SPEC["model"]["use_inertia"])
SEED_POSE: bool = bool(_SPEC["model"]["seed_pose"])
SELF_COLLISION: bool = bool(_SPEC["model"]["self_collision"])
FOOT_BOX_SIZE: Tuple[float, float, float] = tuple(_SPEC["model"]["foot_box"]["size"])  # type: ignore[assignment]
FOOT_BOX_ORIGIN: Tuple[float, float, float] = tuple(_SPEC["model"]["foot_box"]["origin"])  # type: ignore[assignment]

CLAMP_ENABLED: bool = bool(_SPEC["clamp"]["enabled"])

SPAWN_Z: float = float(_SPEC["contract"]["spawn_z"])
ACT_SCALE: float = float(_SPEC["contract"]["act_scale"])
OBS_DIM: int = int(_SPEC["contract"]["obs_dim"])
NJ: int = int(_SPEC["contract"]["nj"])
NOMINAL_LEGS: Tuple[float, ...] = tuple(_SPEC["contract"]["nominal_legs"])
ARM_NOMINAL: Tuple[float, ...] = tuple(_SPEC["contract"]["arm_nominal"])

# ----------------------------------------------------------------------------
# Deterministic open-loop physics probe (Phase 1 of binary-level parity).
# The SAME scripted joint-target sequence is applied in the trainer Python
# solver and in the deploy binary; the per-tick trajectories are then diffed.
# These knobs (and probe_targets() below) are the ONE source both sides import
# -- the deterministic stand pose/gains were previously hand-synced literals
# (deploy KE400/deep-squat vs the build_g1_native probe KE20/shallow-squat),
# the exact class of trainer<->deploy mismatch this probe is meant to surface.
# ----------------------------------------------------------------------------
_PROBE: dict = _SPEC.get("probe", {})
PROBE_POSE_LEGS: Tuple[float, ...] = tuple(_PROBE.get("pose_legs") or NOMINAL_LEGS)
PROBE_ARM_POSE: Tuple[float, ...] = tuple(_PROBE.get("arm_pose") or ARM_NOMINAL)
PROBE_KE: float = float(_PROBE.get("ke", 400.0))
PROBE_KD: float = float(_PROBE.get("kd", 60.0))
PROBE_STATIC_BASE: bool = bool(_PROBE.get("static_base", True))
PROBE_SETTLE_TICKS: int = int(_PROBE.get("settle_ticks", 80))
PROBE_DURATION_TICKS: int = int(_PROBE.get("duration_ticks", 200))
PROBE_SEQUENCE: str = str(_PROBE.get("sequence", "sinusoid"))   # "hold" | "sinusoid"
PROBE_SINE_AMP: float = float(_PROBE.get("sine_amp", 0.15))
PROBE_SINE_PERIOD_TICKS: int = int(_PROBE.get("sine_period_ticks", 100))

# Absolute URDF paths.
URDF_PRIM: Path = REPO_ROOT / _SPEC["urdf"]["prim"]
URDF_FULL: Path = REPO_ROOT / _SPEC["urdf"]["full"]
URDF_LEGS_PRIM: Path = REPO_ROOT / _SPEC["urdf"]["legs_prim"]


# ----------------------------------------------------------------------------
# Live URDF limit extraction -- the SoT for position/velocity/effort limits.
# ----------------------------------------------------------------------------
class JointLimit:
    __slots__ = ("lower", "upper", "velocity", "effort")

    def __init__(self, lower: float, upper: float, velocity: float, effort: float):
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.effort = effort

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        return (f"JointLimit(lo={self.lower:.4f}, hi={self.upper:.4f}, "
                f"v={self.velocity:.3f}, e={self.effort:.1f})")


_LIMIT_CACHE: Dict[str, Dict[str, JointLimit]] = {}


def urdf_limits(urdf_path: Path | str | None = None) -> Dict[str, JointLimit]:
    """Parse <joint>/<limit> for every joint in the URDF, keyed by joint name.

    Cached per URDF path. This is the single source for joint limits -- the
    deploy reads them through the same chain; the trainer/clamp read them here
    instead of carrying hardcoded arrays.
    """
    path = Path(urdf_path) if urdf_path is not None else URDF_PRIM
    key = str(path)
    cached = _LIMIT_CACHE.get(key)
    if cached is not None:
        return cached

    tree = ET.parse(path)
    root = tree.getroot()
    out: Dict[str, JointLimit] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name is None:
            continue
        limit = joint.find("limit")
        if limit is None:
            # fixed / unlimited joint -> (0,0) sentinel means "no position limit"
            out[name] = JointLimit(0.0, 0.0, 0.0, 0.0)
            continue
        out[name] = JointLimit(
            lower=float(limit.get("lower", "0") or 0.0),
            upper=float(limit.get("upper", "0") or 0.0),
            velocity=float(limit.get("velocity", "0") or 0.0),
            effort=float(limit.get("effort", "0") or 0.0),
        )
    _LIMIT_CACHE[key] = out
    return out


def _lim_arrays(names: Sequence[str], urdf_path: Path | str | None = None):
    import numpy as np
    lim = urdf_limits(urdf_path)
    lo, hi, vel, eff = [], [], [], []
    for n in names:
        jl = lim[n]
        lo.append(jl.lower)
        hi.append(jl.upper)
        vel.append(jl.velocity)
        eff.append(jl.effort)
    f = np.float32
    return (np.array(lo, f), np.array(hi, f), np.array(vel, f), np.array(eff, f))


def leg_limits(urdf_path: Path | str | None = None):
    """(lo, hi, vel, effort) float32 arrays in LEGS_JOINTS order, from the URDF.

    Replaces the hardcoded LIM_LO/LIM_HI (deploy controller), JOINT_LIMITS_LO/HI
    (trainer constants), and vel_lim_t (trainer clamp).
    """
    return _lim_arrays(LEGS_JOINTS, urdf_path)


def joint_limits(names: Sequence[str], urdf_path: Path | str | None = None):
    """(lo, hi) float32 arrays for an arbitrary ordered list of joint names."""
    lo, hi, _vel, _eff = _lim_arrays(names, urdf_path)
    return lo, hi


def velocity_limits(names: Sequence[str], urdf_path: Path | str | None = None):
    _lo, _hi, vel, _eff = _lim_arrays(names, urdf_path)
    return vel


def effort_limits(names: Sequence[str], urdf_path: Path | str | None = None):
    _lo, _hi, _vel, eff = _lim_arrays(names, urdf_path)
    return eff


# ----------------------------------------------------------------------------
# Deterministic probe target generator -- the ONE definition both the trainer
# harness and the deploy controller call so the scripted motion is bit-identical
# on both sides. Pure-Python (no numpy), so it reproduces identically in the
# trainer process, the deploy controller, and any reimplementation.
# ----------------------------------------------------------------------------
def probe_targets(tick: int, *, sequence: str | None = None,
                  amp: float | None = None,
                  period_ticks: int | None = None) -> List[float]:
    """Per-joint LEG targets (length 13, LEGS_JOINTS order) at control-step ``tick``.

    Contract: at control step k (k=0,1,2,...) both sides apply ``probe_targets(k)``
    and THEN advance the physics one control step. ``sequence='hold'`` freezes
    every joint at PROBE_POSE_LEGS; ``'sinusoid'`` adds a deterministic per-joint
    sweep ``amp*sin(2*pi*k/period + j*pi/13)`` (a distinct phase per joint so the
    whole articulation is exercised, not just synchronised motion).
    """
    seq = PROBE_SEQUENCE if sequence is None else sequence
    a = PROBE_SINE_AMP if amp is None else amp
    period = PROBE_SINE_PERIOD_TICKS if period_ticks is None else period_ticks
    base = list(PROBE_POSE_LEGS)
    if seq == "hold" or a == 0.0:
        return base
    n = len(base)
    out: List[float] = []
    for j, b in enumerate(base):
        phase = (j * math.pi) / n
        out.append(b + a * math.sin(2.0 * math.pi * tick / period + phase))
    return out


def probe_settle_target(s: int, settle_ticks: int | None = None) -> List[float]:
    """Ramped LEG targets during the (unrecorded) SETTLE phase.

    Both sides spawn straight-legged (all-zero) -- the deploy's natural spawn.
    Linearly ramp the target from straight to PROBE_POSE_LEGS over the first 60%
    of the settle window, then HOLD the pose for the remaining 40% so both sides
    converge to the SAME position-PD equilibrium (gravity sag included) BEFORE the
    recorded probe begins. This removes the launch-IC asymmetry (trainer seeds the
    pose; the deploy spawns straight) and the stiff-PD slam, leaving a clean,
    gentle, identical approach -- so any post-settle divergence is pure stepping
    physics. A gentle ramp (not a step) keeps the high-ke PD from overshooting.
    """
    st = PROBE_SETTLE_TICKS if settle_ticks is None else settle_ticks
    ramp = max(1.0, 0.6 * st)
    frac = min(1.0, (s + 1) / ramp)
    return [frac * b for b in PROBE_POSE_LEGS]


# ----------------------------------------------------------------------------
# Deploy env emission -- the launcher turns the spec into OMNISIM_NEWTON_* env.
# ----------------------------------------------------------------------------
def newton_env(*, include_contact: bool = True) -> Dict[str, str]:
    """OMNISIM_NEWTON_* environment the deploy backend must run with.

    These are exactly the knobs in docs/developer/g1-deploy-walk.md's recipe,
    sourced from the spec instead of hand-transcribed.
    """
    mujoco_warp = SOLVER_KIND in ("mujoco_warp", "mjwarp")
    env: Dict[str, str] = {
        "OMNISIM_NEWTON_FORCE_MUJOCO": "1",
        "OMNISIM_NEWTON_MJWARP": "1" if mujoco_warp else "0",
        "OMNISIM_NEWTON_STATICS": "1",
        "OMNISIM_NEWTON_SUBSTEPS": str(SUBSTEPS),
        "OMNISIM_URDF_USE_INERTIA": "1" if USE_INERTIA else "0",
        "OMNISIM_NEWTON_SEED_POSE": "1" if SEED_POSE else "0",
        "OMNISIM_NEWTON_TARGET_KE": _num(KE),
        "OMNISIM_NEWTON_TARGET_KD": _num(KD),
        "OMNISIM_NEWTON_GROUND_MU": _num(GROUND_MU),
    }
    if ARMATURE:
        env["OMNISIM_NEWTON_JOINT_ARMATURE"] = _num(ARMATURE)
    if not CLAMP_ENABLED:
        env["OMNISIM_NEWTON_DISABLE_JOINT_CLAMP"] = "1"
    if SELF_COLLISION:
        env["OMNISIM_NEWTON_SELF_COLLISION"] = "1"
    if include_contact:
        env["OMNISIM_NEWTON_CONTACT_KE"] = _num(CONTACT_KE)
        env["OMNISIM_NEWTON_CONTACT_KD"] = _num(CONTACT_KD)
    return env


def _num(x: float) -> str:
    """Compact numeric string (no trailing .0 noise for ints)."""
    if float(x) == int(x):
        return str(int(x))
    return repr(float(x))


# ----------------------------------------------------------------------------
# Resolved snapshot -- persisted alongside a trained policy (Stage 0).
# ----------------------------------------------------------------------------
def resolved(extra: dict | None = None) -> dict:
    """Fully-resolved physics config for persisting next to policy.onnx.

    Includes the JSON knobs, the URDF-derived leg limits, and the spec file's
    own content so a run is reproducible from disk alone.
    """
    lim = urdf_limits(URDF_PRIM)
    leg_lim = {n: vars_of(lim[n]) for n in LEGS_JOINTS}
    snap = {
        "spec_file": str(_SPEC_JSON.relative_to(REPO_ROOT)).replace("\\", "/"),
        "knobs": _SPEC,
        "urdf_prim": _SPEC["urdf"]["prim"],
        "leg_limits_from_urdf": leg_lim,
        "legs_joints": list(LEGS_JOINTS),
    }
    if extra:
        snap["run"] = extra
    return snap


def vars_of(jl: JointLimit) -> dict:
    return {"lower": jl.lower, "upper": jl.upper,
            "velocity": jl.velocity, "effort": jl.effort}


__all__ = [
    "REPO_ROOT", "JOINT_NAMES", "LEGS_JOINTS", "ARM_JOINTS",
    "SOLVER_KIND", "SUBSTEPS", "DT", "NJMAX", "NCONMAX",
    "KE", "KD", "ARMATURE", "GROUND_MU", "CONTACT_KE", "CONTACT_KD",
    "USE_INERTIA", "SEED_POSE", "SELF_COLLISION",
    "FOOT_BOX_SIZE", "FOOT_BOX_ORIGIN", "CLAMP_ENABLED",
    "SPAWN_Z", "ACT_SCALE", "OBS_DIM", "NJ", "NOMINAL_LEGS", "ARM_NOMINAL",
    "PROBE_POSE_LEGS", "PROBE_ARM_POSE", "PROBE_KE", "PROBE_KD",
    "PROBE_STATIC_BASE", "PROBE_SETTLE_TICKS", "PROBE_DURATION_TICKS",
    "PROBE_SEQUENCE", "PROBE_SINE_AMP", "PROBE_SINE_PERIOD_TICKS",
    "probe_targets", "probe_settle_target",
    "URDF_PRIM", "URDF_FULL", "URDF_LEGS_PRIM",
    "JointLimit", "urdf_limits", "leg_limits", "joint_limits",
    "velocity_limits", "effort_limits", "newton_env", "resolved",
]
