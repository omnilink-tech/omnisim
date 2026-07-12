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

"""Robot-agnostic deterministic-probe spec — the ONE source both the trainer
harness (``robot_parity_probe.py``) and the deploy controller (``parity_probe``)
import so the scripted no-RL motion is bit-identical on both sides.

It is driven by the per-robot JSON the deterministic STAND already ships
(``projects/policies/controllers/humanoid_stand_deploy/specs/<robot>.json`` — pose,
deploy_ke/kd, ank_bias, joint_order, URDF, spawn_z), plus an optional
``projects/policies/controllers/parity_probe/specs/<robot>.json`` for robots that have
no humanoid stand spec (e.g. quadrupeds). No new physics literals: pose + gains
come from the stand spec, geometry/limits live in the URDF (read here).

Contract (identical to the G1 probe): both sides spawn straight, ramp+settle to
the pose (``settle_target``), then sweep the LEG joints with a deterministic
per-joint sinusoid while HOLDING the arm/torso joints at the pose
(``targets``), recording per-tick joint angles + base pose. ``g1_parity_compare``
diffs the two traces; the welded-base lane (chaos-free) is the machine-precision
parity certification.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
_SPEC_DIRS = [
    REPO / "projects/policies/controllers/parity_probe/specs",       # probe-specific (quadrupeds)
    REPO / "projects/policies/controllers/humanoid_stand_deploy/specs",  # reuse the stand specs
]

# Probe defaults (the deploy stand runs basicTimeStep 16 + OMNISIM_NEWTON_SUBSTEPS 4).
DEFAULT_SUBSTEPS = 4
DEFAULT_DT = 0.016
DEFAULT_SETTLE_TICKS = 80
DEFAULT_DURATION_TICKS = 120
DEFAULT_AMP = 0.15
DEFAULT_PERIOD = 100
DEFAULT_GROUND_MU = 2.0   # run_humanoid_stand_deploy.ps1 value (moot for the lifted welded lane)
DEFAULT_CONTACT_KE = 2500.0
DEFAULT_CONTACT_KD = 100.0


def _find_spec(robot: str) -> Path:
    for d in _SPEC_DIRS:
        p = d / f"{robot}.json"
        if p.is_file():
            return p
    raise FileNotFoundError(
        f"no parity-probe spec for '{robot}' in {[str(x) for x in _SPEC_DIRS]}")


class ProbeSpec:
    def __init__(self, robot: str):
        path = _find_spec(robot)
        d = json.loads(path.read_text(encoding="utf-8"))
        self.name = d["name"]
        self.spec_path = path
        self.urdf = (REPO / d["urdf"]).resolve()
        self.spawn_z = float(d["spawn_z"])
        self.joint_order: List[str] = list(d["joint_order"])
        self.ke = float(d.get("deploy_ke", d.get("ke", 400.0)))
        self.kd = float(d.get("deploy_kd", d.get("kd", 60.0)))
        self.substeps = int(d.get("probe_substeps", DEFAULT_SUBSTEPS))
        self.dt = float(d.get("probe_dt", DEFAULT_DT))
        self.settle_ticks = int(d.get("probe_settle_ticks", DEFAULT_SETTLE_TICKS))
        self.duration_ticks = int(d.get("probe_ticks", DEFAULT_DURATION_TICKS))
        self.amp = float(d.get("probe_amp", DEFAULT_AMP))
        self.period = int(d.get("probe_period", DEFAULT_PERIOD))
        self.ground_mu = float(d.get("ground_mu", DEFAULT_GROUND_MU))
        self.contact_ke = float(d.get("contact_ke", DEFAULT_CONTACT_KE))
        self.contact_kd = float(d.get("contact_kd", DEFAULT_CONTACT_KD))
        # pose = nominal (0 default) + ank_bias on the ankle-pitch joints.
        nominal = {j: 0.0 for j in self.joint_order}
        nominal.update({k: float(v) for k, v in d.get("nominal", {}).items()})
        ank_bias = float(d.get("ank_bias", 0.0))
        for j in d.get("ankle_pitch_joints", []):
            nominal[j] = nominal.get(j, 0.0) + ank_bias
        self.pose: List[float] = [nominal[j] for j in self.joint_order]
        # HOLD set = arm/torso/neck joints; SWEEP set = the rest (legs). The
        # legs are the gravity-loaded, locomotion-relevant joints to probe.
        self.hold = set(d.get("arm_hold", []))
        self.sweep = [j for j in self.joint_order if j not in self.hold]
        self._parse_urdf()

    def _parse_urdf(self) -> None:
        root = ET.parse(self.urdf).getroot()
        self.urdf_joint_order: List[str] = []   # add_urdf DOF order (URDF doc order)
        self.limits: Dict[str, Tuple[float, float]] = {}
        for j in root.iter("joint"):
            if j.get("type") in ("revolute", "continuous", "prismatic"):
                nm = j.get("name")
                self.urdf_joint_order.append(nm)
                lim = j.find("limit")
                if lim is not None and j.get("type") != "continuous":
                    self.limits[nm] = (float(lim.get("lower", "0") or 0.0),
                                       float(lim.get("upper", "0") or 0.0))
                else:
                    self.limits[nm] = (0.0, 0.0)   # (0,0) sentinel == unlimited
        missing = [j for j in self.joint_order if j not in self.urdf_joint_order]
        if missing:
            raise ValueError(f"{self.name}: joint_order names not actuated in URDF: {missing}")

    # --- deterministic scripted motion (shared by both sides) ----------------
    def targets(self, k: int, sequence: str = "sinusoid") -> List[float]:
        """Full joint_order target vector at control step k. Legs sweep; arms hold."""
        out = list(self.pose)
        if sequence == "hold" or self.amp == 0.0:
            return out
        n = len(self.joint_order)
        for idx, j in enumerate(self.joint_order):
            if j in self.hold:
                continue
            phase = (idx * math.pi) / n
            v = self.pose[idx] + self.amp * math.sin(2.0 * math.pi * k / self.period + phase)
            lo, hi = self.limits.get(j, (0.0, 0.0))
            if not (lo == 0.0 and hi == 0.0):
                v = max(lo, min(hi, v))
            out[idx] = v
        return out

    def settle_target(self, s: int) -> List[float]:
        """Ramp from straight (all-zero) to the pose over the first 60% of the
        settle window, then hold — drives both sides to the same PD equilibrium
        from the identical straight-leg IC before recording."""
        ramp = max(1.0, 0.6 * self.settle_ticks)
        frac = min(1.0, (s + 1) / ramp)
        return [frac * p for p in self.pose]

    # --- name -> newton DOF/qpos index (add_urdf adds DOFs in URDF doc order) -
    def dof_index(self, jn: str, free_offset: int) -> int:
        return free_offset + self.urdf_joint_order.index(jn)

    def qpos_index(self, jn: str, qpos_free: int) -> int:
        return qpos_free + self.urdf_joint_order.index(jn)

    def lifted_z(self, clearance: float = 0.5) -> float:
        """Welded-base spawn height that lifts the feet clear of the floor so the
        chaos-free lane has NO ground contact."""
        return self.spawn_z + clearance


def load(robot: str) -> ProbeSpec:
    return ProbeSpec(robot)
