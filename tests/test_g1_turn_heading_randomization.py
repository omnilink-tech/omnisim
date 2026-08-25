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

from pathlib import Path
import pytest


ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "projects/policies/training/g1_walk_recipe.py"


def test_sequence_yaw_randomization_rotates_reset_and_target_frames() -> None:
    recipe = RECIPE.read_text(encoding="utf-8")

    assert '_seq_yaw0_rand = _f("SEQ_YAW0_RAND", 0.0) if seq_mode else 0.0' in recipe
    assert "seq_yaw0_t.copy_(torch.where(mask, _yo, seq_yaw0_t))" in recipe
    assert "_y0 = seq_yaw_t[b0] + seq_yaw0_t" in recipe
    # Both deploy-eval and PPO rollout must follow the same rotated routine frame.
    assert recipe.count("ytgt_t.copy_(seq_yaw_t[") == 2
    assert recipe.count("+ seq_yaw0_t)") == 2


def test_deploy_selects_payload_turner_only_while_suction_is_live() -> None:
    recipe = RECIPE.read_text(encoding="utf-8")

    assert '("BATON_CARRY_TURN", "_wr_carry_turnctx", "payload")' in recipe
    assert '(getattr(world, "_wr_suck", None) or {}).get("on", False)' in recipe
    assert 'ctx = getattr(world, "_wr_carry_turnctx", None) or ctx' in recipe
    assert 'world._wr_active_turnctx = ctx' in recipe
    assert '_ctx9 = getattr(world, "_wr_active_turnctx", None) or {}' in recipe
    assert 'ctx.get("label", "turn")' in recipe
    assert '"gres": float(gres)' in recipe
    assert '"greslat": float(greslat)' in recipe
    assert '"gresyaw": float(gresyaw)' in recipe
    assert 'world._wr_gres = ctx["gres"]' in recipe
    assert 'world._wr_gres = b["gres"]' in recipe
    assert "professional plateau stop" in recipe
    assert 'TURN_CARRY_PLATEAU_STOP_LEAD_DEG' in recipe
    assert 'TURN_TARGET_STOP' in recipe
    assert 'TURN-LOOP: physical target stop' in recipe
    assert 'W_SEQ_YAWRATE' in recipe
    assert 'SEQ-YAW-EVAL progress=' in recipe
    assert 'seqwz_qvel_sign = _f("SEQ_YAWRATE_QVEL_SIGN", 1.0)' in recipe
    assert '_wzmeas9 = seqwz_qvel_sign * qvel_t[:, 5]' in recipe
    assert '1.0 - (_wzmeas9 - _wzref9).abs() / seqwz_sig' in recipe
    assert 'torch.exp(-((qvel_t[:, 5] - _wzref9) ** 2)' not in recipe


def test_payload_hand_resolution_is_independent_and_fails_loudly() -> None:
    recipe = RECIPE.read_text(encoding="utf-8")
    launcher = (ROOT / "projects/policies/training/run_walk_rl.sh").read_text(encoding="utf-8")
    _camp = ROOT / "cloud/runpod/campaigns/campaign_g1_turn_pro.sh"
    if not _camp.exists():
        pytest.skip("cloud/ ops tree is not part of the public snapshot")
    campaign = _camp.read_text(encoding="utf-8")

    assert "if whole and (w_armg > 0 or payload_kg > 0):" in recipe
    assert "if len(arm_qadr) != 2 or len(elb_qadr) != 2:" in recipe
    assert 'raise RuntimeError("required CARRY_PAYLOAD_KG setup failed")' in recipe
    assert '_status_write("ERROR", 0, _i("PPO_ITERS", 0))' in recipe
    assert '\"state\": \"ERROR\"' in launcher
    assert "RC=5; break" in launcher
    assert "W_ARMGHOST=4 W_WB=6 ARM_RESIDUAL=0.15" in campaign
    assert "wr_turn_pro90_s31_it200.pt" in campaign
    assert "wr_turn_pro90_s47_it200.pt" in campaign
    pod_driver = (ROOT / "cloud/runpod/campaigns/launch_g1_turn_pro_pod.sh").read_text(encoding="utf-8")
    assert 'expected = [-0.90, 0.15, 0.0, 0.86, 0.0, -0.90, -0.15, 0.0, 0.86, 0.0]' in pod_driver
    assert 'assert d["elbow_lut"][0] == [0.86, 0.86]' in pod_driver


def test_turn_root_follower_tracks_reference_without_yaw_assistance() -> None:
    recipe = RECIPE.read_text(encoding="utf-8")

    assert '"seq_root": root[:, :2].astype(np.float32)' in recipe
    assert 'world._wr_turn_xy0 = (float(_qsxy[0]), float(_qsxy[1]))' in recipe
    assert '_f("TURN_CARRY_ROOT_TRACK_KP", _f("TURN_ROOT_TRACK_KP", 0.0))' in recipe
    assert 'else _f("TURN_ROOT_TRACK_KP", 0.0)' in recipe
    assert '_tdx5 = float(_troot5[_tb5][0] - _troot5[0][0])' in recipe
    assert '_tdy5 = float(_troot5[_tb5][1] - _troot5[0][1])' in recipe
    # Root tracking is horizontal force only; the active turn branch still zeros yaw torque.
    assert '_fx5 += _lam5 * (-_trk5 * (float(qp[0]) - _txref5)' in recipe
    assert '_fy5 += _lam5 * (-_trk5 * (float(qp[1]) - _tyref5)' in recipe
    assert 'if getattr(world, "_wr_turn_active", False):\n            _hky = 0.0' in recipe
    assert '_tz5 = 0.0   # steering toward the stale pre-turn heading would fight it' in recipe
