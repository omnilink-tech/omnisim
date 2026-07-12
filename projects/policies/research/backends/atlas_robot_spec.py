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

"""Atlas (Boston Dynamics v5) RobotSpec.

⚠️ STALE — NOT the standing-deploy source of truth (2026-05-29). This is a
walking-oriented RobotSpec (CPG + walking action_scale) and carries an OLD
deep-squat NOMINAL pose the shipped standing work abandoned (it self-collides
— see `_scratch/atlas_pose_search.py` "BAD_OLD_NOMINAL"). The deployed Atlas
STANDING policy + pose live in `gpu_mjwarp_atlas_stand_trainer.py` and
`atlas_stand_deploy.py` (mild NOMINAL, RES_SCALE=0.05, no CPG), NOT here.
`eval_atlas_stand.py` / `eval_atlas_obs_modes.py` read this stale spec and are
effectively dead for the GPU-trained standing policy. Kept registered for the
generic --robot path; reconcile or retire when the migration no-rename freeze
lifts. See `docs/developer/rl-current-state.md`.

URDF source: Drake's `examples/atlas/urdf/atlas_convex_hull.urdf`
(pinned commit 73a8da32cd41ff7fd023c3680f8250860cbd0e6b), patched for RL
by `projects/policies/research/tools/patch_atlas_urdf_for_rl.py`:
  * 12 zero-radius contact-sphere collisions stripped (MuJoCo rejects
    them; Webots doesn't need them — foot chull is the real contact).
  * joint <dynamics> bumped from Drake's damping=0.1/friction=0 to
    damping=2.0/friction=0.1 (without this, PPO learns to pump joints
    at solver frequency to extract free energy from the integrator).

Joint layout (30 actuated DOFs, ordered to match the deploy
controller's motor lookup `f"{name}_motor"`):

    back  : bkz bky bkx           (3)
    neck  : ay                    (1)
    arms  : shz shx ely elx uwy mwx lwy   x2 = 14
    legs  : hpz hpx hpy kny aky akx       x2 = 12

The arm/back/neck joints are actuated but held at a relaxed nominal
pose; the policy can swing them for balance (important for bipedal
walking — locking the upper body makes it strictly harder for RL).
"""
from __future__ import annotations

from pathlib import Path

from .base import RobotSpec


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())


# ──────────────────────────────────────────────────────────────────────
# Joint order — matches Drake URDF naming + groups them by body region
# so debug logs are readable. The order is fixed forever: changing it
# breaks all trained policies on this spec.
# ──────────────────────────────────────────────────────────────────────

JOINT_NAMES = (
    # Torso (3)
    "back_bkz", "back_bky", "back_bkx",
    # Neck (1)
    "neck_ay",
    # Left arm (7), shoulder→hand
    "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx",
    "l_arm_uwy", "l_arm_mwx", "l_arm_lwy",
    # Right arm (7)
    "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx",
    "r_arm_uwy", "r_arm_mwx", "r_arm_lwy",
    # Left leg (6), hip→ankle
    "l_leg_hpz", "l_leg_hpx", "l_leg_hpy",
    "l_leg_kny", "l_leg_aky", "l_leg_akx",
    # Right leg (6)
    "r_leg_hpz", "r_leg_hpx", "r_leg_hpy",
    "r_leg_kny", "r_leg_aky", "r_leg_akx",
)
assert len(JOINT_NAMES) == 30


# ──────────────────────────────────────────────────────────────────────
# Nominal standing pose. Slight squat (hip pitch + knee + ankle pitch
# in a coordinated bend) so the CoM is well over the feet, arms hanging
# slightly out from the torso for balance. The robot can fall onto this
# pose from a small spawn drop without immediately tipping.
# ──────────────────────────────────────────────────────────────────────

_NOMINAL = {
    # Back stays upright.
    "back_bkz": 0.0, "back_bky": 0.0, "back_bkx": 0.0,
    # Neck level (limits clamp anyway).
    "neck_ay": 0.0,
    # Arms: shoulder yaw 0, shoulder roll slightly outward, elbow
    # bent ~70deg, forearm relaxed. Mirror left/right where the axis
    # is signed.
    "l_arm_shz": 0.0, "l_arm_shx": -1.2, "l_arm_ely": 1.2, "l_arm_elx": 0.5,
    "l_arm_uwy": 0.0, "l_arm_mwx": 0.0, "l_arm_lwy": 0.0,
    "r_arm_shz": 0.0, "r_arm_shx":  1.2, "r_arm_ely": 1.2, "r_arm_elx": -0.5,
    "r_arm_uwy": 0.0, "r_arm_mwx": 0.0, "r_arm_lwy": 0.0,
    # Legs: soft squat. hip_y negative = thigh forward; knee positive
    # bends; ankle_y negative compensates so the foot stays flat. These
    # values give pelvis ≈ 0.92 m off the ground.
    "l_leg_hpz": 0.0, "l_leg_hpx": 0.06, "l_leg_hpy": -0.55,
    "l_leg_kny": 1.10, "l_leg_aky": -0.55, "l_leg_akx": -0.06,
    "r_leg_hpz": 0.0, "r_leg_hpx": -0.06, "r_leg_hpy": -0.55,
    "r_leg_kny": 1.10, "r_leg_aky": -0.55, "r_leg_akx":  0.06,
}
NOMINAL_POSE = tuple(_NOMINAL[j] for j in JOINT_NAMES)


# ──────────────────────────────────────────────────────────────────────
# Per-joint limits — pulled from the URDF <limit lower upper> attributes
# verbatim. Action = clip(nominal + cpg + scale*policy_out, lo, hi).
# ──────────────────────────────────────────────────────────────────────

_LIM = {
    "back_bkz":  (-0.663225,  0.663225),
    "back_bky":  (-0.219388,  0.538783),
    "back_bkx":  (-0.523599,  0.523599),
    "neck_ay":   (-0.602139,  1.14319),
    "l_arm_shz": (-1.5708,    0.785398),
    "l_arm_shx": (-1.5708,    1.5708),
    "l_arm_ely": ( 0.0,       3.14159),
    "l_arm_elx": ( 0.0,       2.35619),
    "l_arm_uwy": (-3.011,     3.011),
    "l_arm_mwx": (-1.7628,    1.7628),
    "l_arm_lwy": (-2.9671,    2.9671),
    "r_arm_shz": (-0.785398,  1.5708),
    "r_arm_shx": (-1.5708,    1.5708),
    "r_arm_ely": ( 0.0,       3.14159),
    "r_arm_elx": (-2.35619,   0.0),
    "r_arm_uwy": (-3.011,     3.011),
    "r_arm_mwx": (-1.7628,    1.7628),
    "r_arm_lwy": (-2.9671,    2.9671),
    "l_leg_hpz": (-0.174358,  0.786794),
    "l_leg_hpx": (-0.523599,  0.523599),
    "l_leg_hpy": (-1.61234,   0.65764),
    "l_leg_kny": ( 0.0,       2.35637),
    "l_leg_aky": (-1.0,       0.7),
    "l_leg_akx": (-0.8,       0.8),
    "r_leg_hpz": (-0.786794,  0.174358),
    "r_leg_hpx": (-0.523599,  0.523599),
    "r_leg_hpy": (-1.61234,   0.65764),
    "r_leg_kny": ( 0.0,       2.35637),
    "r_leg_aky": (-1.0,       0.7),
    "r_leg_akx": (-0.8,       0.8),
}
JOINT_LIMITS_LO = tuple(_LIM[j][0] for j in JOINT_NAMES)
JOINT_LIMITS_HI = tuple(_LIM[j][1] for j in JOINT_NAMES)


# ──────────────────────────────────────────────────────────────────────
# Bipedal CPG prior: alternate-step gait with diagonal arm swing.
# Left leg phase 0.0, right leg phase 0.5 — opposite half-cycle. Arms
# swing opposite to their same-side leg (counter-rotating, like a human
# walk). The CPG is conservative: a small hip-pitch sweep + knee lift
# during swing, so the robot can keep its CoM over the stance foot
# without an aggressive heel-strike.
#
# Joint-name suffix conventions on Atlas:
#   *_hpy  hip pitch  → primary forward/backward leg swing
#   *_kny  knee       → bends during swing for ground clearance
#   *_aky  ankle pitch → couples with hpy for foot clearance
#   *_shx  shoulder roll → arm swings forward/back via shoulder roll
# ──────────────────────────────────────────────────────────────────────

# Per-joint phase in [0,1). Same length as JOINT_NAMES.
_PHASES = {j: 0.0 for j in JOINT_NAMES}
# Right leg is half-cycle offset from left.
for j in JOINT_NAMES:
    if j.startswith("r_leg_"):
        _PHASES[j] = 0.5
# Arm swing opposite to leg on the same side (counter-rotation).
for j in JOINT_NAMES:
    if j.startswith("l_arm_"):
        _PHASES[j] = 0.5
    elif j.startswith("r_arm_"):
        _PHASES[j] = 0.0
CPG_LEG_PHASE = tuple(_PHASES[j] for j in JOINT_NAMES)


# ──────────────────────────────────────────────────────────────────────
# Per-joint PD gains. Atlas is 175 kg and the arm/torso links carry
# a lot of mass with significant moment arms. Uniform kp=400 lets the
# arms droop because gravity torque on an extended arm (mass × g ×
# lever_arm ≈ 35 Nm) outpaces the kp at small joint errors. The fix
# is per-joint gains: clamp arms+back+neck at very stiff hold (kp=2500)
# so they barely move from nominal, while keeping legs moderate
# (kp=600) so the policy's residual still has authority.
# ──────────────────────────────────────────────────────────────────────

def _per_joint_pid():
    pid = {}
    for j in JOINT_NAMES:
        if j.startswith(("l_arm_", "r_arm_", "back_", "neck_")):
            # Stiff clamp — these joints are not RL-controlled in any
            # meaningful way; we want them held at nominal so the
            # policy can focus on legs.
            pid[j] = (2500.0, 0.0, 80.0)
        elif j.endswith("_kny"):
            # Knee carries body weight; needs high gain.
            pid[j] = (800.0, 0.0, 30.0)
        elif j.endswith("_hpy") or j.endswith("_aky"):
            # Hip pitch + ankle pitch: large effort joints (740-840 Nm).
            pid[j] = (600.0, 0.0, 25.0)
        elif j.endswith("_hpx") or j.endswith("_akx"):
            # Roll joints: moderate effort, moderate gain.
            pid[j] = (400.0, 0.0, 18.0)
        else:  # _hpz (hip yaw)
            pid[j] = (300.0, 0.0, 12.0)
    return tuple(pid[j] for j in JOINT_NAMES)


MOTOR_PID_PER_JOINT = _per_joint_pid()


ATLAS = RobotSpec(
    name="atlas",
    urdf_path=REPO_ROOT / "projects" / "robots" / "boston_dynamics" / "atlas"
              / "urdf" / "atlas_convex_hull.urdf",
    joint_names=JOINT_NAMES,
    nominal_pose=NOMINAL_POSE,
    joint_limits_lo=JOINT_LIMITS_LO,
    joint_limits_hi=JOINT_LIMITS_HI,
    # Bipedal RL is much more sensitive to action authority than
    # quadruped — 0.15 rad is plenty for legs, much more and the
    # policy can violate the CPG into a fall on a single tick.
    action_scale=0.15,
    # obs_dim = 9 (vel/grav) + 3*30 (q/qd/last_action) + 4 (cmd+clock)
    obs_dim=9 + 3 * 30 + 4,    # 103
    # MuJoCo forward-kinematics at the nominal squat pose puts the foot
    # bottom (sphere centers at -0.076 in foot frame, sphere radius 0.015)
    # at pelvis_z - 0.83 m. Spawn 5 cm above that so the first physics
    # step doesn't free-fall through the foot contacts — at z=1.05 we'd
    # see a +0.20 rad forward pitch within one tick from the impact.
    spawn_translation=(0.0, 0.0, 0.88),
    spawn_rotation=(0.0, 0.0, 1.0, 0.0),
    max_episode_steps=1024,
    obs_recipe="legged_locomotion",
    # Default uniform PD (used by Webots deploy controller via
    # setControlPID, and by MJX as a fallback when motor_pid_per_joint
    # isn't read).
    motor_pid=(400.0, 0.0, 15.0),
    motor_pid_per_joint=MOTOR_PID_PER_JOINT,
    # Bipedal trot prior: ~1.2 Hz step rate, modest hip swing + knee
    # lift. Start small — if RL drives the residual to add more swing,
    # we can raise these. Set freq_hz=0 to disable the CPG entirely
    # via env var (SPOT_CPG_FREQ_HZ=0 still works for atlas through
    # the spec sidecar; the field name is shared).
    cpg_freq_hz=1.2,
    cpg_hip_y_amp=0.20,
    cpg_knee_amp=0.30,
    cpg_leg_phase=CPG_LEG_PHASE,
)
