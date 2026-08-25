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

"""Canonical OmniQuad RobotSpec — the URDF + joint order + nominal pose
that every backend uses. Single source of truth so SB3, MJX, and Isaac
all train against identical contracts.
"""
from pathlib import Path

from .base import RobotSpec


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(
    f"{leg}_{j}" for leg in LEGS for j in ("hip_x", "hip_y", "knee")
)

# Standing pose matching omniquad_rl_agent / omniquad_rl_deploy.
_NOMINAL = {}
for leg in LEGS:
    _NOMINAL[(leg, "hip_x")] = +0.30 if "left" in leg else -0.30
    _NOMINAL[(leg, "hip_y")] = 0.30
    _NOMINAL[(leg, "knee")] = -0.60

NOMINAL_POSE = tuple(
    _NOMINAL[(leg, j)] for leg in LEGS for j in ("hip_x", "hip_y", "knee")
)

# Per-joint URDF limits (matches the in-repo URDF after our edits).
_LO = {}
_HI = {}
for leg in LEGS:
    if "left" in leg:
        _LO[(leg, "hip_x")] = +0.001; _HI[(leg, "hip_x")] = +0.60
    else:
        _LO[(leg, "hip_x")] = -0.60;  _HI[(leg, "hip_x")] = -0.001
    _LO[(leg, "hip_y")] = +0.001; _HI[(leg, "hip_y")] = +0.60
    _LO[(leg, "knee")]  = -1.20;  _HI[(leg, "knee")]  = -0.01

JOINT_LIMITS_LO = tuple(_LO[(leg, j)] for leg in LEGS for j in ("hip_x", "hip_y", "knee"))
JOINT_LIMITS_HI = tuple(_HI[(leg, j)] for leg in LEGS for j in ("hip_x", "hip_y", "knee"))


# Trot CPG phasing: diagonals in phase (FL+RR at 0.0, FR+RL at 0.5).
# Joint order is FL/FR/RL/RR × (hip_x, hip_y, knee), so 3 entries per leg.
_TROT_PHASES = {
    "front_left":  0.0,
    "front_right": 0.5,
    "rear_left":   0.5,
    "rear_right":  0.0,
}
CPG_LEG_PHASE_VEC = tuple(_TROT_PHASES[leg] for leg in LEGS for _j in range(3))


OMNIQUAD = RobotSpec(
    name="omniquad",
    urdf_path=REPO_ROOT / "projects" / "robots" / "omnisim" / "omniquad" / "urdf" / "omniquad.urdf",
    joint_names=JOINT_NAMES,
    nominal_pose=NOMINAL_POSE,
    joint_limits_lo=JOINT_LIMITS_LO,
    joint_limits_hi=JOINT_LIMITS_HI,
    action_scale=0.15,
    # 50 = legged_locomotion (9+3*12+4) + heading-deviation tail (obs[49]).
    # v12_200k policies are 49-dim; deploy controller auto-detects ONNX
    # input dim and feeds the matching obs slice. Trainer/env/sidecar must
    # match each other and the policy.onnx; sweeping to 50 lets the next
    # training and ONNX export agree (the prior obs50 run mis-traced ONNX
    # because this still said 49, producing a [s79,49]X[50,256] error).
    obs_dim=50,
    spawn_translation=(0.0, 0.0, 0.70),
    spawn_rotation=(0.0, 0.0, 1.0, 0.0),
    max_episode_steps=1024,
    # Motor gains: Webots ODE handles the auto-derived light-leg inertia
    # stably even at kp=20 (per the Newton commit history). kp=500 was
    # tried for MJX/MuJoCo deploy but caused PPO instability in early
    # CPU+Webots training. The CPU SB3 + ODE path's training and deploy
    # are both validated at kp=20 / action_scale=0.15.
    motor_pid=(20.0, 0.0, 0.3),
    # CPG defaults: hand-tuned via tools/cpg_sanity.py with the smoothed
    # (sin²) knee profile. 1.75 Hz trot with conservative amps (0.13,
    # 0.22) gives vx=+0.058 m/s open-loop at 100% upright with plenty of
    # stability margin for the policy to add residuals without tipping.
    # Larger amps walk faster open-loop (0.20/0.30 → 0.135 m/s) but PPO
    # consistently destabilizes within ~25k steps because the
    # combined CPG+policy motion has no headroom for noise.
    cpg_freq_hz=1.75,
    cpg_hip_y_amp=0.13,
    cpg_knee_amp=0.22,
    cpg_leg_phase=CPG_LEG_PHASE_VEC,
)
