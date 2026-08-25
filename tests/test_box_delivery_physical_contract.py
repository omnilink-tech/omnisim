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

"""Regression contract for the physical-suction box-delivery demo."""

import json

from pathlib import Path
import pytest


ROOT = Path(__file__).resolve().parents[1]


def _text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_box_delivery_uses_real_newton_box_and_suction_hands() -> None:
    launcher = _text("projects/policies/demos/run_box_delivery.sh")
    world = _text("projects/policies/worlds/g1_box_grasp.omniworld")
    urdf = _text("projects/robots/unitree/g1/urdf/g1_23dof_grasp.urdf")

    assert "WALK_WORLD=projects/policies/worlds/g1_box_grasp.omniworld" in launcher
    assert "PHYS_GRASP=1" in launcher
    assert "GR_SUCTION=1" in launcher
    assert "GR_SUCTION_R=0.010" in launcher
    assert "GR_PAD_X=0.10 GR_PAD_Y=0.13" in launcher

    box_start = world.index("DEF CARRY_BOX Solid")
    box = world[box_start:world.index("DEF DELIVERY_CART_B Solid", box_start)]
    assert "boundingObject USE CARRY_BOX_GEO" in box
    assert "density -1" in box
    assert "mass 1.0" in box
    assert "g1_23dof_grasp.urdf" in world
    assert urdf.count('material name="cup_lip"') >= 2


def test_physical_mode_cannot_supervisor_move_the_box() -> None:
    rig = _text("projects/policies/controllers/harness_rig/harness_rig.py")
    physical_branch = rig.index('if os.environ.get("PHYS_GRASP", "") == "1":')
    branch_continue = rig.index("        continue", physical_branch)
    first_legacy_pose_write = rig.index("box_tr.setSFVec3f", physical_branch)

    assert branch_continue < first_legacy_pose_write


def test_suction_is_seal_gated_before_force_latch() -> None:
    recipe = _text("projects/policies/training/g1_walk_recipe.py")
    gate = recipe.index("_near9 = _SKe is not None")
    latch = recipe.index('_SKe["on"] = True', gate)

    assert gate < latch
    assert "surface-seal gap %.3f m" in recipe


def test_course_clears_each_table_before_resuming_the_corridor() -> None:
    launcher = _text("projects/policies/demos/run_box_delivery.sh")
    course_line = next(line for line in launcher.splitlines() if '"BATON_COURSE=' in line)
    course = course_line.split("BATON_COURSE=", 1)[1].split('"', 1)[0].split(";")

    # Pick: lift, then take the load-proven forward carrier arc around cart A.
    assert course[1:5] == ["stand,6", "carryto,5.3,-2.1", "carryto,7.8,-1.42", "carryto,8.55,-1.61"]
    assert not any(seg.startswith("carrybackto,") for seg in course)
    # Place: finish the press at segment 5, release/unstick, then clear cart B
    # south-first, then keep walking forward through an open-floor U-arc.
    assert "GR_PLACE_SEG=6" in launcher
    assert course[5:14] == [
        "carryto,9.3,-1.62",
        "stand,6",
        "walkto,8.70,-2.70",
        "walkto,11.0,-2.70",
        "walkto,12.0,-1.80",
        "walkto,12.0,0.0",
        "walkto,11.4,1.0",
        "walkto,10.7,1.9",
        "stand,0",
    ]
    assert not any(seg.startswith(("backto,", "carrybackto,")) for seg in course)
    assert not any(seg.startswith("turn,") for seg in course)


def test_box_delivery_routes_only_proven_forward_skills() -> None:
    launcher = _text("projects/policies/demos/run_box_delivery.sh")
    sequence = json.loads(_text("projects/policies/skills/sequences/box_delivery.json"))

    assert "OMNISIM_FOOT_TORSION=0" in launcher
    assert "OMNISIM_NEWTON_DISABLE_ISLAND=1" in launcher
    assert "back|" not in launcher
    assert "carryback|" not in launcher
    assert "g1_walk_backward" not in sequence["skills"]
    assert "g1_carry_backward" not in sequence["skills"]
    assert "g1_turn_in_place" not in sequence["skills"]
    assert not any(key.startswith("BATON_TURN") for key in sequence["env"])
    assert not any(key.startswith("TURN_") for key in sequence["env"])
    assert not any(key.startswith("BATON_BACK") for key in sequence["env"])
    assert not any(key.startswith("BATON_CARRYBACK") for key in sequence["env"])
    assert sequence["env"]["HSTAND_WARMUP_RELOAD"] == "0"


def test_box_delivery_eliminates_cold_box_contact_before_pickup() -> None:
    launcher = _text("projects/policies/demos/run_box_delivery.sh")
    sequence = json.loads(_text("projects/policies/skills/sequences/box_delivery.json"))
    recipe = _text("projects/policies/training/g1_walk_recipe.py")

    assert "GR_PREPICK_FIXTURE=1" in launcher
    assert "OMNISIM_NEWTON_NO_GRAPH" not in launcher
    assert "OMNISIM_NEWTON_BASE_GUARD=0" in launcher
    assert sequence["env"]["GR_PREPICK_FIXTURE"] == "1"
    assert "OMNISIM_NEWTON_NO_GRAPH" not in sequence["env"]
    assert sequence["env"]["OMNISIM_NEWTON_BASE_GUARD"] == "0"
    assert "PREPICK-FIXTURE armed" in recipe
    assert "PREPICK-FIXTURE released" in recipe
    assert '_SKf9["w"] = (_Ff9, _z3f9, _z3f9, _z3f9)' in recipe


def test_reverse_course_faces_away_from_travel_bearing() -> None:
    recipe = _text("projects/policies/training/g1_walk_recipe.py")

    assert '"backto", "carrybackto"' in recipe
    assert '_reverse9 = _seg[0] in ("backto", "carrybackto")' in recipe
    assert '_travel9 + (math.pi if _reverse9 else 0.0)' in recipe
    assert '"backto": "back", "carrybackto": "carryback"' in recipe
    assert '_f("BATON_BACK_ARRIVE_R", 0.20) if _reverse9' in recipe


def test_runpod_motion_campaign_matches_production_harness() -> None:
    _camp = ROOT / "cloud/runpod/campaigns/campaign_g1_motion_pro.sh"
    if not _camp.exists():
        pytest.skip("cloud/ ops tree is not part of the public snapshot")
    campaign = _camp.read_text(encoding="utf-8")

    for setting in (
        "HARNESS_LAM0=0.9", "HARNESS_KP=600", "HARNESS_KD=60",
        "HARNESS_FY=400", "HARNESS_KZ=2000", "HARNESS_DZ=150",
        "HARNESS_FY_HEADING=1", "HARNESS_ATT_HEADING=1",
    ):
        assert setting in campaign
    assert "PPO_DEVICE=cuda" in campaign
    assert "W_TRACK_LIN=15" in campaign
    assert 'W_SEQ_YAWRATE=${TURN_W_SEQ_YAWRATE:-3}' in campaign
    assert "SEQ_YAWRATE_SIG=0.30" in campaign
    assert "SEQ_YAWRATE_QVEL_SIGN=-1" in campaign
    assert "TURN_RESUME" in campaign
    assert 'W_TRACK_ANG=${TURN_W_TRACK_ANG:-5}' in campaign
    assert 'PPO_LR=${TURN_PPO_LR:-0.0002}' in campaign
    assert 'CKPT_EVERY=${TURN_CKPT_EVERY:-100}' in campaign
    assert 'PPO_ITERS=${TURN_ITERS:-500}' in campaign
    assert "W_SEQWZ=" not in campaign
    assert 'assert vals["fwd"] <= -0.35' in campaign
    assert 'assert vals["surv"] >= 0.95' in campaign
    assert 'assert 82.0 <= y["progress"] <= 98.0' in campaign
    assert 'y["err"] <= 8.0' in campaign


def test_walk_launcher_reaps_engine_when_campaign_is_interrupted() -> None:
    launcher = _text("projects/policies/training/run_walk_rl.sh")

    assert 'PY="${OMNISIM_RUNNER_PYTHON:-${OMNISIM_PYTHON:-python}}"' in launcher
    assert "cleanup_runner()" in launcher
    assert 'pkill -TERM -P "$RPID"' in launcher
    assert "terminate_runner()" in launcher
    assert "trap cleanup_runner EXIT" in launcher
    assert "trap terminate_runner INT TERM" in launcher
    assert 'kill -TERM "$RPID"' in launcher
    assert "trap - EXIT INT TERM" in launcher


def test_baton_supports_opt_in_per_specialist_handover_tuning() -> None:
    baton = _text("omnisim/policy/baton.py")

    assert 'geti("BATON_%s_MORPH_TICKS" % _env_stem(des)' in baton
    assert '"BATON_%s_ACTION_SCALE" % _env_stem(nm)' in baton
    assert '"1.0"' in baton


def test_box_demo_disables_mujoco_islands_before_first_physics_step() -> None:
    backend = _text("src/omnisim/physics/OmNewtonBackend.cpp")
    construct = backend.index("self.solver = newton.solvers.SolverMuJoCo")
    early_disable = backend.index('if _force_mujoco and _os.environ.get("OMNISIM_NEWTON_DISABLE_ISLAND")')
    rolling_friction = backend.index("# OPT-IN ROLLING FRICTION")

    assert construct < early_disable < rolling_friction
    assert "mjtDisableBit.mjDSBL_ISLAND" in backend[early_disable:rolling_friction]
    assert "islands=off" in backend[early_disable:rolling_friction]
