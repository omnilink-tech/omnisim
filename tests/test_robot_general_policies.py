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

"""The policy stack is ROBOT-GENERAL. These tests are the guard rails that keep it that way.

Three claims, each of which was FALSE at some point and cost real time when it failed silently:

  1. BATON is a LIBRARY -- its executable code knows nothing about anatomy or runtimes.
  2. The Shadowing deploy corridor is derived from the ghost's joint NAMES, and on the G1 the
     derivation reproduces the hardcoded literals it replaced EXACTLY (so the flagship cannot
     move), while on a 10-DOF H1 it produces DIFFERENT, CORRECT columns.
  3. A ghost that cannot be model-checked FAILS. An unregistered robot is not a pass.

Pure stdlib + numpy: no engine, no torch, no GPU. Runs in bare CI.
"""
from __future__ import annotations

import ast
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
os.environ.setdefault("OMNISIM_HOME", str(REPO))
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from projects.policies.common import robot_registry as RR  # noqa: E402
from projects.policies.skills import manifest as SM  # noqa: E402
from projects.policies.skills import skill_lib as SKILL_LIB  # noqa: E402
from omnisim.policy import baton as BATON  # noqa: E402

G1_LEGS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
# the H1's leg is 10 DOF, in a DIFFERENT order (yaw,roll,pitch,knee,ankle -- no ankle roll)
H1_LEGS = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_joint",
]
GO2_LEGS = [f"{leg}_{p}_joint" for leg in ("FL", "FR", "RL", "RR")
            for p in ("hip", "thigh", "calf")]


def _knees(names):
    return [i for i, j in enumerate(names) if RR.axis_of(j) == "knee"]


def _hips(names):
    out = []
    for side in ("L", "R"):
        c = [i for i, j in enumerate(names)
             if RR.side_row(j)[0] == side and RR.axis_of(j) == "pitch"]
        out.append(c[0] if c else 0)
    return out


# ── 1. BATON is a library ────────────────────────────────────────────────────
def _baton_executable_code() -> str:
    """baton.py with every docstring stripped -- i.e. what actually RUNS."""
    tree = ast.parse((REPO / "omnisim/policy/baton.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Module)):
            if (node.body and isinstance(node.body[0], ast.Expr)
                    and isinstance(node.body[0].value, ast.Constant)
                    and isinstance(node.body[0].value.value, str)):
                node.body.pop(0)
    return ast.unparse(tree)


@pytest.mark.parametrize("token", ["torch", "world", "arm", "elb", "elbow", "crane", "harness",
                                   "leg_lut", "arm_lut", "pelvis"])
def test_baton_library_knows_no_anatomy_and_no_runtime(token):
    """BATON must not mention a body part, a crane, or a policy runtime -- ever.

    It began life inside the G1's deploy hook: it blended `arm`/`elb` channels, reached into
    `world._wr_*`, and ran torch nets. A quadruped has no elbow, and its deploy has no `world`
    and no torch at all (it is an ONNX controller in another process). If any of these names
    comes back, the library has quietly re-grown a robot.
    """
    code = _baton_executable_code()
    assert not re.search(r"\b" + re.escape(token) + r"\b", code), (
        f"baton.py's EXECUTABLE code mentions {token!r} -- the arbiter has re-acquired "
        f"knowledge of a specific body or runtime. Push it out into the host.")


def test_baton_imports_without_torch():
    """The quadruped host is an ONNX controller: torch is not installed in that interpreter."""
    src = (REPO / "omnisim/policy/baton.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    imported = {n.names[0].name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {n.module.split(".")[0] for n in ast.walk(tree)
                 if isinstance(n, ast.ImportFrom) and n.module}
    assert "torch" not in imported and "onnxruntime" not in imported
    assert imported <= {"__future__", "dataclasses", "json", "math", "os", "typing", "numpy"}


def test_support_gate_is_selected_by_morphology():
    """The biped default is calibrated; other legged gaits must inject their support model."""
    import math
    biped = BATON.gate_for("humanoid")
    ds = 0.65 * 2.0 * math.pi
    assert biped(ds) is True                      # double support: hand-over allowed
    assert biped(ds + math.pi / 2) is False       # mid-swing, on one foot: refused
    with pytest.raises(ValueError, match="gait-specific gate"):
        BATON.gate_for("quadruped")               # trot/pace/crawl have different windows


def test_a_policy_free_specialist_contributes_a_zero_residual():
    """A quadruped `stand` is a DETERMINISTIC HOLD (policy=None): it tracks its ghost exactly."""
    import numpy as np

    class _Host:
        channels = ("glut", "ref")

        def phase(self):
            return 0.0

        def log(self, m):
            pass

        def act(self, sp, obs):
            raise AssertionError("a policy-free specialist must never be evaluated")

        def reset_hidden(self, sp):
            pass

        def write_tables(self, eff):
            pass

    tables = {"glut": None, "ref": None, "vx": 0.0}
    st = BATON.BatonState(
        reg={"walk": BATON.Specialist("walk", tables, primary=True),
             "stand": BATON.Specialist("stand", tables, policy=None)},
        active="stand", target="stand", src=dict(tables), u=1.0)
    out = BATON.act(_Host(), st, np.zeros(4, np.float32), np.ones(12, np.float32))
    assert np.array_equal(out, np.zeros(12, np.float32))


# ── 2. the deploy corridor is role-derived ───────────────────────────────────
def test_g1_roles_reproduce_the_hardcoded_literals_exactly():
    """THE NO-REGRESSION PIN. The deploy corridor used to carry these literals:

        for col in range(12):  roll = (1,5,7,11)  yaw = (2,8)  knees = [3,9]  hips = [0,6]
        left leg = the first 6 columns

    They are now derived from the ghost's joint NAMES. On the G1 the derivation must reproduce
    them EXACTLY -- otherwise the flagship silently moves, and every G1 result in the repo is
    invalidated.
    """
    n, roll, yaw, mask, per = RR.leg_roles(G1_LEGS)
    assert n == 12
    assert roll == [1, 5, 7, 11]
    assert yaw == [2, 8]
    assert per == 6
    assert mask.tolist() == [1., 0., 1., 1., 1., 0.] * 2
    assert _knees(G1_LEGS) == [3, 9]
    assert _hips(G1_LEGS) == [0, 6]


def test_h1_roles_differ_from_the_g1_and_are_correct():
    """The whole point: a 10-DOF H1 is NOT a G1 with two columns missing.

    The old deploy read the knees at columns [3, 9]. On the H1, column 9 is a RIGHT ANKLE --
    it would have gated the stance corridor on the wrong joint, silently, forever.
    """
    n, roll, yaw, mask, per = RR.leg_roles(H1_LEGS)
    assert n == 10
    assert per == 5
    assert _knees(H1_LEGS) == [3, 8]            # NOT [3, 9]
    assert roll == [1, 6]
    assert yaw == [0, 5]
    assert H1_LEGS[9] == "right_ankle_joint"    # what the old code called a "knee"


def test_quadruped_roles_pair_mirrors_left_right_not_front_rear():
    """FL<->FR and RL<->RR. (A humanoid-shaped rule paired FL<->RL, which is why the Go2 ghost
    used to carry FAKE humanoid joint aliases just to make the gate engage.)"""
    assert RR.mirror_joint("FL_calf_joint") == "FR_calf_joint"
    assert RR.mirror_joint("RL_thigh_joint") == "RR_thigh_joint"
    assert RR.axis_of("FL_hip_joint") == "roll"      # a bare quad hip is the ABDUCTION joint
    assert RR.axis_of("FL_thigh_joint") == "pitch"
    assert RR.axis_of("FL_calf_joint") == "knee"
    assert _knees(GO2_LEGS) == [2, 5, 8, 11]


def test_hip_yaw_is_not_confused_with_omnisim_hip_y():
    """⛔ THE SUBSTRING TRAP. "hip_y" is a substring of "left_hip_YAW_joint", and OmniQuad's hip_y is
    a PITCH joint. Matching on substrings reclassified the G1's hip-yaw as pitch-plane, dragged
    it into the symmetry gate, and dropped the flagship walk ghost from PASS to WARN."""
    assert RR.axis_of("left_hip_yaw_joint") == "yaw"
    assert RR.axis_of("front_left_hip_y") == "pitch"
    assert RR.axis_of("front_left_hip_x") == "roll"


# ── 3. the validator fails closed ────────────────────────────────────────────
def test_ghost_validator_fails_closed_on_an_unregistered_robot(tmp_path):
    """A ghost that cannot be model-checked is a FAILED ghost, never a passed one.

    An unregistered robot used to skip EVERY model-aware gate and still print VERDICT: PASS --
    which is how the Go2 shadow ghost, the artifact the quadruped `method: shadowing` claim
    rests on, was passing.
    """
    src = REPO / "projects/policies/ghosts/g1/ghost_official_full_v3_lut.json"
    if not src.exists():
        pytest.skip("flagship ghost not present in this checkout")
    d = json.loads(src.read_text())
    d["robot"] = "valkyrie"                  # a real robot -- just not one we can model-check
    lut = tmp_path / "unregistered_lut.json"
    lut.write_text(json.dumps(d))

    r = subprocess.run([sys.executable,
                        str(REPO / "projects/policies/training/ghost_validator.py"), str(lut)],
                       capture_output=True, text=True, env={**os.environ, "OMNISIM_HOME": str(REPO)})
    assert r.returncode != 0, "an unmodel-checkable ghost must FAIL (non-zero exit)"
    assert "VERDICT: FAIL" in r.stdout
    assert "not in the registry" in r.stdout


def test_every_versioned_ghost_declares_its_robot_and_its_joints():
    """Identity is mandatory (schema 2). The positional fallback -- "assume it's a G1" -- is the
    guess that let an H1 ghost be limit-checked against a G1 URDF. Untracked experiments are
    deliberately outside the release contract; ``skill_lib.py ghost --all`` audits those."""
    store = REPO / "projects/policies/ghosts"
    if not store.exists():
        pytest.skip("ghost store not present")
    bad = []
    reg = SM.Registry.discover()
    for p in SKILL_LIB._contract_luts(reg):
        d = json.loads(p.read_text())
        if not d.get("robot") or not d.get("joints"):
            bad.append(p.name)
        elif len(d["leg_lut"][0]) != len(d["joints"]):
            bad.append(f"{p.name} (width != len(joints))")
    assert not bad, f"ghosts without a declared identity: {bad}"
