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

"""Spot RL agent — OmniSim-side controller for SpotEnv.

This controller binds a TCP server on `--port` and exchanges fixed-size
numpy float32 packets with the SpotEnv gym wrapper. The protocol:

    obs_packet  (controller -> trainer, on each OmniSim tick):
        float32[OBS_DIM]   -- observation vector
        float32            -- reward (single scalar)
        uint8              -- done flag (0 or 1)

    cmd_packet  (trainer -> controller, on each tick):
        uint8              -- command tag: 0=ACTION 1=RESET 2=QUIT
        float32[ACT_DIM]   -- action vector (joint position deltas, residual
                              on top of the nominal standing pose); ignored
                              when command is RESET or QUIT

Observation layout (OBS_DIM = 49):
   [0:3]    base linear velocity (world)
   [3:6]    base angular velocity (world)
   [6:9]    projected gravity in body frame (= R^T @ [0,0,-9.81], normalized)
   [9:21]   joint positions (12, ordered FL/FR/RL/RR x hip_x/hip_y/knee)
   [21:33]  joint velocities (12, finite difference from previous tick)
   [33:45]  last action (12, residuals as received from trainer)
   [45:48]  velocity command (vx_target, vy_target, wz_target)
   [48]     gait clock (in [0, 1), increments at 0.5 Hz so policy can use
            phase info if it wants)

Action layout (ACT_DIM = 12):
    residual joint position deltas in radians, scaled by ACTION_SCALE
    when applied. Final motor target = nominal_standing_pose + delta.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time

import numpy as np

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = []
for leg in LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))

# OBS_DIM=49 was the original layout (vel+ang+grav+q+qd+last_action+vel_cmd+
# clock). 50-dim variant appends obs[49] = wrapped heading deviation (yaw -
# yaw_at_episode_start), so the policy can actually know it's drifted (was
# blind to it). Required to train a true straight-walker; v12_200k (49-dim)
# stays deployable -- deploy controller detects via ONNX input shape.
OBS_DIM = int(os.environ.get("SPOT_OBS_DIM", "50"))
ACT_DIM = 12

# Nominal standing pose: residual policy adds deltas to these.
HIP_X_LEFT = 0.30
HIP_X_RIGHT = -0.30
HIP_Y_STAND = 0.30
KNEE_STAND = -0.60

NOMINAL_POSE = np.zeros(ACT_DIM, dtype=np.float32)
for i, (leg, joint) in enumerate(JOINT_ORDER):
    if joint == "hip_x":
        NOMINAL_POSE[i] = HIP_X_LEFT if "left" in leg else HIP_X_RIGHT
    elif joint == "hip_y":
        NOMINAL_POSE[i] = HIP_Y_STAND
    else:
        NOMINAL_POSE[i] = KNEE_STAND

ACTION_SCALE = 0.15  # max +/- rad of residual the policy can command.
                     # Kept at 0.15 (legacy value) — bumping to 0.20
                     # gave the policy enough authority that PPO's
                     # early random actions tipped the body within a
                     # handful of updates. 0.15 keeps training stable
                     # while still leaving the policy plenty of room
                     # to refine the CPG-driven motion (~9° per joint
                     # per tick of residual is more than enough).


# ── Central Pattern Generator (CPG) trotting prior ──
# An open-loop trot gait. The applied joint target is:
#     target = NOMINAL_POSE + CPG_BASE(t) + ACTION_SCALE * policy_action
# This turns the RL task from "discover walking" into "refine walking",
# which PPO solves in 100-500k steps on CPU instead of needing 50M+ steps
# on GPU. The amplitudes are tuned conservatively so the gait alone is
# at least stable in place (won't tip the body), and the policy can adjust
# both up and down via the residual.
#
# Trot phasing: diagonal pairs in phase.
#     FL (0.0) + RR (0.0)   "first diagonal"
#     FR (0.5) + RL (0.5)   "second diagonal, half-cycle offset"
CPG_LEG_PHASE = {
    "front_left":  0.0,
    "front_right": 0.5,
    "rear_left":   0.5,
    "rear_right":  0.0,
}


def cpg_base_offsets(t: float, freq_hz: float, hip_y_amp: float, knee_amp: float) -> np.ndarray:
    """Return ACT_DIM-vector of CPG joint deviations added on top of nominal
    pose. The hip_y oscillation sweeps the leg fore/aft; the knee
    oscillation lifts the foot during the swing half-cycle. If
    freq_hz<=0 or amplitudes are 0, returns all zeros (CPG disabled)."""
    base = np.zeros(ACT_DIM, dtype=np.float32)
    if freq_hz <= 0.0 or (hip_y_amp == 0.0 and knee_amp == 0.0):
        return base
    omega = 2.0 * math.pi * freq_hz
    for i, (leg, joint) in enumerate(JOINT_ORDER):
        phase = CPG_LEG_PHASE[leg]
        theta = omega * t + 2.0 * math.pi * phase
        if joint == "hip_y":
            # Fore/aft hip swing, using cos so the hip is at its
            # backmost position (+amp) at phase 0 (just-landed, start of
            # stance) and moves *forward* through stance (cos: +1 → 0 →
            # -1), which with the foot planted pulls the body FORWARD.
            # Then during swing (sin > 0, knee bent) hip recovers back.
            base[i] = hip_y_amp * math.cos(theta)
        elif joint == "knee":
            # Knee bends MORE NEGATIVE during the foot-lift half of the
            # cycle (sin(theta) > 0). Use *squared* clipped sine so the
            # bend has zero slope at the swing/stance transitions —
            # avoids the knee-velocity discontinuity (visible as a
            # jerky kick at heel-off / heel-strike) that the plain
            # max(0, sin) variant produced.
            s = max(0.0, math.sin(theta))
            base[i] = -knee_amp * (s * s)
        # hip_x stays at nominal.
    return base


# Joint limits (must match URDF). Action is clamped to these after adding nominal pose.
JOINT_LIMITS_LO = np.array([
    +0.001, +0.001, -1.20,   # FL: hip_x, hip_y, knee
    -0.60,  +0.001, -1.20,   # FR
    +0.001, +0.001, -1.20,   # RL
    -0.60,  +0.001, -1.20,   # RR
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +0.60,  +0.60,  -0.01,
    -0.001, +0.60,  -0.01,
    +0.60,  +0.60,  -0.01,
    -0.001, +0.60,  -0.01,
], dtype=np.float32)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--port", type=int, default=7100)
    p.add_argument("--host", default="127.0.0.1")
    args, _ = p.parse_known_args()
    # SPOT_RL_PORT env var overrides --port (so SpotEnv can pick a unique
    # port per worker without editing the world file).
    env_port = os.environ.get("SPOT_RL_PORT")
    if env_port:
        try:
            args.port = int(env_port)
        except ValueError:
            pass
    return args


def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Block-read exactly n bytes from sock; raise ConnectionError if peer closes."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf.extend(chunk)
    return bytes(buf)


def send_all(sock: socket.socket, data: bytes) -> None:
    sock.sendall(data)


def rotation_to_projected_gravity(ori9: list[float]) -> np.ndarray:
    """Webots returns body->world rotation as a 9-element row-major matrix.
    The world gravity vector is (0, 0, -1) (normalized). To get its
    projection in body frame, multiply by R^T = R_world->body.

    For a row-major R_body_to_world:
        R^T @ [0,0,-1] = (-R[2,0], -R[2,1], -R[2,2])
    With Webots's index layout (ori[i*3+j] = R[i][j]) this is:
        (-ori[2], -ori[5], -ori[8])
    Wait -- ori[6,7,8] is the THIRD ROW, which is R[2,:]. So R^T's third
    column is ori[6,7,8]. R^T @ [0,0,-1] = -third_column_of_R = (-ori[2],-ori[5],-ori[8])
    """
    return np.array([-ori9[2], -ori9[5], -ori9[8]], dtype=np.float32)


def main() -> int:
    args = parse_args()
    sys.stderr.write(f"[spot_rl_agent] resolved port = {args.port} "
                     f"(env SPOT_RL_PORT={os.environ.get('SPOT_RL_PORT','<unset>')})\n")
    sys.stderr.flush()

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    # ── Devices ────────────────────────────────────────────────────────
    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            sys.stderr.write(f"[spot_rl_agent] missing motor {name}\n")
            return 1
        motors.append(m)
        try:
            # Modest PD for RL; the policy outputs residual position targets.
            if hasattr(m, "setControlPID"):
                m.setControlPID(20.0, 0.0, 0.3)
        except Exception:
            pass
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None
    if self_node is None:
        sys.stderr.write("[spot_rl_agent] no supervisor handle\n")
        return 1
    try:
        translation_field = self_node.getField("translation")
        rotation_field = self_node.getField("rotation")
    except Exception:
        translation_field = None
        rotation_field = None

    # Initial spawn pose for resets. Body equilibrium at standing pose is
    # z=0.666 with foot spheres; spawn slightly above for clean landing.
    SPAWN_TRANS = [0.0, 0.0, 0.70]
    SPAWN_ROT = [0.0, 0.0, 1.0, 0.0]  # identity

    # Initial knee/hip already seeded at standing pose by URDF importer.
    # Just command the standing pose on tick 0.
    def apply_pose(target_pose: np.ndarray) -> None:
        for i, m in enumerate(motors):
            m.setPosition(float(target_pose[i]))

    apply_pose(NOMINAL_POSE)

    # ── TCP server ─────────────────────────────────────────────────────
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    sys.stderr.write(f"[spot_rl_agent] waiting for trainer on {args.host}:{args.port}\n")
    sys.stderr.flush()
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sys.stderr.write(f"[spot_rl_agent] trainer connected from {addr}\n")
    sys.stderr.flush()

    # Initial settle so the first obs reflects a stable standing pose,
    # not the mid-fall transient from world spawn.
    for _ in range(20):
        if robot.step(step_ms) == -1:
            break

    # ── State for observation building ─────────────────────────────────
    prev_joint_pos = NOMINAL_POSE.copy()
    prev_joint_vel = np.zeros(ACT_DIM, dtype=np.float32)
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    prev_action = np.zeros(ACT_DIM, dtype=np.float32)   # for action-rate reward
    vel_command = np.array([0.5, 0.0, 0.0], dtype=np.float32)  # default fwd 0.5 m/s
    episode_step = 0
    cum_reward = 0.0
    sim_time = 0.0
    episode_start_bx = 0.0   # body x at episode start (forward-distance witness)
    episode_start_by = 0.0   # body y at episode start (lateral-drift baseline)
    episode_start_yaw = 0.0  # body yaw at episode start (heading-drift baseline)

    # Reward shaping constants. Legged-gym style. Each can be overridden
    # via env vars at training time so we can iterate on reward shape
    # without redeploying the controller.
    def _env_float(key, default):
        v = os.environ.get(key)
        if v is None or v.strip() == "":
            return default
        try:
            return float(v)
        except ValueError:
            return default

    R_LIN_VEL_WT     = _env_float("SPOT_R_LIN_VEL_WT", 1.0)
    # Sharpness of the velocity-tracking Gaussian: r_lin = WT*exp(-err/SCALE).
    # The legacy 0.25 (sigma~0.35 m/s) is so broad that standing still
    # (vx=0 vs target 0.5) still pays 37% of max -- so the policy prefers
    # the safe stand over risking forward motion. A small SCALE (e.g. 0.05,
    # sigma~0.16) makes standing pay ~0 and only ~target speed pays well,
    # forcing the policy to actually hit the commanded velocity.
    R_LIN_VEL_SCALE  = _env_float("SPOT_R_LIN_VEL_SCALE", 0.25)
    R_ANG_VEL_WT     = _env_float("SPOT_R_ANG_VEL_WT", 0.5)
    R_VZ_WT          = _env_float("SPOT_R_VZ_WT", -1.0)
    R_ROLLPITCH_WT   = _env_float("SPOT_R_ROLLPITCH_WT", -0.1)
    R_ANG_XY_WT      = _env_float("SPOT_R_ANG_XY_WT", -0.02)
    R_ACTION_RATE_WT = _env_float("SPOT_R_ACTION_RATE_WT", -0.005)
    R_JOINT_ACC_WT   = _env_float("SPOT_R_JOINT_ACC_WT", -2.5e-7)
    R_ALIVE_BONUS    = _env_float("SPOT_R_ALIVE_BONUS", 0.05)
    # NEW: exponential UPRIGHTNESS reward. The flat alive bonus pays the
    # same whether the body is perfectly level or about to tip, so there is
    # almost no gradient teaching the FINE balance corrections that turn a
    # marginally-stable gait (survives ~150 steps open-loop) into a robust
    # one. The quadratic roll/pitch penalty has vanishing gradient near 0
    # too. This term pays R_UPRIGHT_WT * exp(-(roll^2+pitch^2)/(2 sigma^2)):
    # a sharp peak at level that falls off fast, so the policy is strongly
    # rewarded for actively staying centered and the gradient is steepest
    # exactly in the 0.1-0.4 rad band where the slow tip starts. 0 = off.
    R_UPRIGHT_WT     = _env_float("SPOT_R_UPRIGHT_WT", 0.0)
    R_UPRIGHT_SIGMA  = _env_float("SPOT_R_UPRIGHT_SIGMA", 0.15)
    # NEW: linear forward-velocity bonus. The exp-shaped tracking reward
    # saturates near 1.0 once vx is anywhere close to the target, so PPO
    # sees almost no gradient between "stand still" and "walk slowly".
    # This term adds an unbounded reward proportional to actual forward
    # velocity (clipped non-negative), which gives a clean monotonic
    # gradient toward walking faster.
    R_VX_BONUS_WT    = _env_float("SPOT_R_VX_BONUS_WT", 0.0)
    R_VX_DEADBAND    = _env_float("SPOT_R_VX_DEADBAND", 0.05)  # m/s
    # NEW: cap on the forward-velocity bonus (m/s). The uncapped bonus is
    # monotonic in vx, so the policy LUNGES (vx 2+ m/s) to farm it. The
    # exp tracking reward avoids lunging but pays 37% of max even at vx=0,
    # so standing-in-place stays the safe optimum and the policy never
    # walks forward. Capping the bonus at the TARGET speed gives the best
    # of both: standing earns 0 (strong push to move), reward rises
    # linearly to the target, then is FLAT above it (no lunge incentive).
    # 0 = uncapped (legacy).
    R_VX_BONUS_CAP   = _env_float("SPOT_R_VX_BONUS_CAP", 0.0)  # m/s
    # NEW: reward clip range (was hardcoded [-5,5]). Walking should be
    # able to push the per-step reward well past 5 to expose the gradient.
    R_CLIP_LO        = _env_float("SPOT_R_CLIP_LO", -10.0)
    R_CLIP_HI        = _env_float("SPOT_R_CLIP_HI", +10.0)
    # NEW: optional fixed velocity command override (instead of randomizing
    # vx in [0.2,0.8] each reset). Useful for stabilizing early training
    # before broadening the command distribution as a curriculum.
    SPOT_FIXED_VX    = _env_float("SPOT_FIXED_VX", float("nan"))
    SPOT_FIXED_WZ    = _env_float("SPOT_FIXED_WZ", float("nan"))
    # NEW: CPG trotting prior. Default freq 0 disables the CPG (recovers
    # the legacy "residual on nominal pose" behavior).
    CPG_FREQ_HZ      = _env_float("SPOT_CPG_FREQ_HZ", 0.0)
    CPG_HIP_Y_AMP    = _env_float("SPOT_CPG_HIP_Y_AMP", 0.20)
    CPG_KNEE_AMP     = _env_float("SPOT_CPG_KNEE_AMP", 0.30)
    # NEW: residual action scale (was the module constant ACTION_SCALE=0.15).
    # Env-tunable so we can (a) set 0.0 to observe the PURE CPG gait with the
    # policy disabled -- a diagnostic to separate gait-structural drift from
    # policy behavior -- or (b) give the policy more authority.
    ACTION_SCALE_RUN = _env_float("SPOT_ACTION_SCALE", float(ACTION_SCALE))
    # NEW: feedforward pitch trim (rad) added to every fore/aft hip joint
    # (the "hip_y" joint sweeps fore/aft; see cpg_base_offsets). Under
    # Newton at mu=2.0 the planted front feet act as a pivot and the CPG's
    # forward propulsion tips the body NOSE-DOWN (pitch dives to -1.0 rad ~=
    # PITCH_FAIL by ~step 100, even at vx=0). A uniform fore/aft hip bias
    # shifts the support fore/aft to re-level the gait equilibrium so the
    # policy only has to reject perturbations, not fight a constant moment.
    # Deploy MUST set the same value. 0.0 = legacy (no trim).
    PITCH_TRIM       = _env_float("SPOT_PITCH_TRIM", 0.0)
    _pitch_trim_vec = np.zeros(ACT_DIM, dtype=np.float32)
    if PITCH_TRIM != 0.0:
        for _i, (_leg, _joint) in enumerate(JOINT_ORDER):
            if _joint == "hip_y":
                _pitch_trim_vec[_i] = PITCH_TRIM
    # NEW: termination penalty for falling. The legacy controller hard-coded
    # -1.0 which is far too small relative to the per-tick walking reward
    # (often +6 to +10) — the policy could fall every few seconds and still
    # come out ahead. Setting this very negative (e.g. -50) makes any fall
    # catastrophically expensive, which is what we want when "never fall"
    # is the priority.
    R_TERM_PENALTY   = _env_float("SPOT_R_TERM_PENALTY", -1.0)
    # NEW: joint-limit-proximity penalty. Quadratic cost on how close
    # each joint sits to its URDF position limit, measured as a fraction
    # of the joint's range. Defeats the Newton "B-mode" failure where
    # the policy holds every joint pinned at its limit so the constraint
    # forces cancel and the body stays up while riding the alive bonus
    # -- a degenerate non-walking optimum that NaN'd on deploy. With a
    # negative weight here, sitting on the limits costs reward, so the
    # only route to high return is to actually move the legs through
    # their mid-range. 0.0 = disabled (legacy behaviour).
    R_JOINT_LIMIT_WT = _env_float("SPOT_R_JOINT_LIMIT_WT", 0.0)
    # ABSOLUTE-pose tracking: yaw rate goes to zero on average but the
    # heading still slowly drifts and the body wanders sideways. These
    # penalise the integrated drift directly so the policy actually holds
    # a straight line, not just a zero-mean yaw rate.
    R_HEADING_WT     = _env_float("SPOT_R_HEADING_WT", 0.0)   # rad^2  (try -2.0 .. -6.0)
    R_LATERAL_WT     = _env_float("SPOT_R_LATERAL_WT", 0.0)   # m^2    (try -1.0 .. -4.0)
    sys.stderr.write(
        f"[spot_rl_agent] rewards: lin={R_LIN_VEL_WT} ang={R_ANG_VEL_WT} "
        f"vz={R_VZ_WT} alive={R_ALIVE_BONUS} vx_bonus={R_VX_BONUS_WT} "
        f"heading={R_HEADING_WT} lateral={R_LATERAL_WT} "
        f"clip=[{R_CLIP_LO},{R_CLIP_HI}] fixed_vx={SPOT_FIXED_VX} fixed_wz={SPOT_FIXED_WZ} "
        f"cpg=({CPG_FREQ_HZ}Hz hip={CPG_HIP_Y_AMP} knee={CPG_KNEE_AMP})\n"
    )
    sys.stderr.flush()

    # Termination thresholds.
    BZ_FAIL = 0.30
    ROLL_FAIL = 1.0
    PITCH_FAIL = 1.0
    MAX_EPISODE_STEPS = 1024

    # Episode body-trace diagnostic (no rebuild): logs how episode 1 (the
    # seeded spawn) vs episodes 2+ (supervisor reset) behave step-by-step,
    # to see HOW post-reset episodes collapse. Opt-in via OMNISIM_AGENT_TRACE.
    _dbg = {"ep": 0}
    _dbg_path = os.environ.get("OMNISIM_AGENT_TRACE")

    def reset_episode() -> None:
        nonlocal prev_joint_pos, prev_joint_vel, last_action, prev_action, vel_command, episode_step, cum_reward, episode_start_bx, episode_start_by, episode_start_yaw, sim_time
        _dbg["ep"] += 1  # ep 0 = initial spawn; ep 1 = first reset; ...
        # Re-apply nominal pose BEFORE the supervisor reset so motors are
        # targeting standing as the body warps to spawn.
        apply_pose(NOMINAL_POSE)
        # Hard-reset body pose to spawn via supervisor fields.
        try:
            if translation_field is not None:
                translation_field.setSFVec3f(SPAWN_TRANS)
            if rotation_field is not None:
                rotation_field.setSFRotation(SPAWN_ROT)
            if hasattr(self_node, "resetPhysics"):
                self_node.resetPhysics()
        except Exception:
            pass
        # Step the simulator a few ticks so motors actually converge to
        # standing pose and the body settles, BEFORE returning control
        # to the trainer. Without this, the first 1-2 ticks of every
        # episode start with the body falling from a partially-cooked
        # pose -- the policy sees garbage obs and the episode terminates
        # immediately ("body fell" in <2 steps).
        for _ in range(20):
            if robot.step(step_ms) == -1:
                break
        prev_joint_pos = NOMINAL_POSE.copy()
        for i, s in enumerate(sensors):
            if s is not None:
                try:
                    prev_joint_pos[i] = float(s.getValue())
                except Exception:
                    pass
        prev_joint_vel = np.zeros(ACT_DIM, dtype=np.float32)
        last_action = np.zeros(ACT_DIM, dtype=np.float32)
        prev_action = np.zeros(ACT_DIM, dtype=np.float32)
        # Randomise command slightly: vx in [0.2, 0.8], yaw rate in [-0.3, 0.3].
        # If the env vars SPOT_FIXED_VX / SPOT_FIXED_WZ are finite, the
        # command is pinned to those instead (curriculum-friendly).
        if math.isfinite(SPOT_FIXED_VX):
            vel_command[0] = SPOT_FIXED_VX
        else:
            vel_command[0] = 0.2 + 0.6 * np.random.random()
        vel_command[1] = 0.0
        if math.isfinite(SPOT_FIXED_WZ):
            vel_command[2] = SPOT_FIXED_WZ
        else:
            vel_command[2] = (np.random.random() - 0.5) * 0.6
        episode_step = 0
        cum_reward = 0.0
        # CRITICAL: restart the gait clock each episode. sim_time drives both
        # the CPG offset (target = NOMINAL + cpg(sim_time) + action) and the
        # obs clock. It used to accumulate across episodes, so episode 1
        # started at gait phase 0 (works) but episodes 2+ started at a
        # RANDOM phase -- the seeded standing robot got a large CPG offset on
        # step 1 and lurched, collapsing the episode (the warm-start
        # "ep_len 710 -> 80 after first reset" bug, environmental / present
        # even with a frozen policy). Resetting to 0 makes every episode
        # start at the same phase as episode 1.
        sim_time = 0.0
        # Capture the post-settle body x so the episode-end log can
        # report net forward distance -- the witness that tells a real
        # walker apart from a B-mode "stand and farm alive bonus" policy.
        try:
            _p = self_node.getPosition() or [0.0, 0.0, 0.0]
            _o = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
            episode_start_bx = float(_p[0])
            episode_start_by = float(_p[1])
            episode_start_yaw = math.atan2(float(_o[3]), float(_o[0]))
        except Exception:
            episode_start_bx = 0.0
            episode_start_by = 0.0
            episode_start_yaw = 0.0

    def build_obs_reward_done() -> tuple[np.ndarray, float, bool, dict]:
        nonlocal prev_joint_pos, prev_joint_vel, sim_time
        try:
            pos = self_node.getPosition() or [0.0, 0.0, 0.0]
            ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
            vel = self_node.getVelocity() or [0,0,0,0,0,0]
        except Exception:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        # Body pose
        bz = float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        if _dbg_path and _dbg["ep"] <= 2 and episode_step <= 110 and episode_step % 5 == 0:
            try:
                _q = np.array([float(s.getValue()) if s is not None else 0.0
                               for s in sensors], dtype=np.float32)
                _qerr = float(np.max(np.abs(_q - NOMINAL_POSE)))
                with open(_dbg_path, "a") as _f:
                    _f.write(f"ep{_dbg['ep']} step{episode_step:3d} "
                             f"bz={bz:.3f} roll={roll:+.3f} pitch={pitch:+.3f} "
                             f"vx={float(vel[0]):+.2f} max|q-nom|={_qerr:.3f}\n")
            except Exception:
                pass
        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = np.array(vel[3:6], dtype=np.float32)
        proj_g = rotation_to_projected_gravity(ori)

        # Joint state
        q = np.zeros(ACT_DIM, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is not None:
                try:
                    q[i] = float(s.getValue())
                except Exception:
                    q[i] = 0.0
        qd = (q - prev_joint_pos) / max(step_dt, 1e-4)
        qdd = (qd - prev_joint_vel) / max(step_dt, 1e-4)
        prev_joint_pos = q
        prev_joint_vel = qd

        # Clock is phase-locked to the CPG frequency when CPG is active,
        # so the policy can correlate its residual with the gait cycle.
        # Falls back to 0.5 Hz when CPG is disabled (legacy behavior).
        _clock_freq = CPG_FREQ_HZ if CPG_FREQ_HZ > 0.0 else 0.5
        clock = (sim_time * _clock_freq) % 1.0
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = v_lin
        obs[3:6] = v_ang
        obs[6:9] = proj_g
        obs[9:21] = q
        obs[21:33] = qd
        obs[33:45] = last_action
        obs[45:48] = vel_command
        # obs[49] = wrapped heading deviation from spawn yaw (radians). Lets
        # the policy SEE its yaw drift instead of guessing from instantaneous
        # angular velocity. Required for true straight-walking; rate-only
        # obs proven insufficient (see project_spot_walker_standard).
        if OBS_DIM >= 50:
            cur_yaw = math.atan2(float(ori[3]), float(ori[0]))
            dyaw = cur_yaw - episode_start_yaw
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            obs[49] = float(dyaw)
        # Sanitize NaN/inf (physics can spit out NaN on solver divergence,
        # which then crashes PPO when it builds a Normal distribution).
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)
        obs[48] = clock

        # ── Reward ──
        # Linear velocity tracking (xy)
        v_xy = v_lin[:2]
        v_xy_target = vel_command[:2]
        lin_err = float(np.sum((v_xy - v_xy_target) ** 2))
        r_lin = R_LIN_VEL_WT * math.exp(-lin_err / R_LIN_VEL_SCALE)

        # Angular velocity tracking (yaw)
        wz_err = float((v_ang[2] - vel_command[2]) ** 2)
        r_ang = R_ANG_VEL_WT * math.exp(-wz_err / 0.25)

        # Penalties
        r_vz = R_VZ_WT * float(v_lin[2] ** 2)
        r_rp = R_ROLLPITCH_WT * (roll * roll + pitch * pitch)
        r_wxy = R_ANG_XY_WT * float(v_ang[0]**2 + v_ang[1]**2)
        # Exponential uprightness: sharp peak at level, fast fall-off.
        if R_UPRIGHT_WT != 0.0:
            _s2 = 2.0 * R_UPRIGHT_SIGMA * R_UPRIGHT_SIGMA
            r_upright = R_UPRIGHT_WT * math.exp(-(roll * roll + pitch * pitch) / _s2)
        else:
            r_upright = 0.0
        # Action rate: difference between consecutive applied actions.
        r_action_rate = R_ACTION_RATE_WT * float(np.sum((last_action - prev_action) ** 2))
        # Joint acceleration penalty (smoothness)
        r_joint_acc = R_JOINT_ACC_WT * float(np.sum(qdd * qdd))
        r_alive = R_ALIVE_BONUS

        # Linear forward-velocity bonus (unbounded, monotonic in vx).
        vx_eff = max(0.0, float(v_lin[0]) - R_VX_DEADBAND)
        if R_VX_BONUS_CAP > 0.0:
            vx_eff = min(vx_eff, R_VX_BONUS_CAP)
        r_vx_bonus = R_VX_BONUS_WT * vx_eff

        # Joint-limit-proximity penalty (anti-B-mode). For each joint,
        # how far it has travelled from the centre of its range, as a
        # fraction in [0, 1] (0 = mid-range, 1 = at a limit). Square it
        # and sum -- a joint pinned at its stop contributes ~1.0, the
        # whole-body sum approaches 12 when every joint is on a limit,
        # so a weight of e.g. -0.3 makes the B-mode pose cost ~3.6/step
        # which dwarfs the alive bonus it was trying to farm.
        if R_JOINT_LIMIT_WT != 0.0:
            jl_center = 0.5 * (JOINT_LIMITS_LO + JOINT_LIMITS_HI)
            jl_halfrange = 0.5 * (JOINT_LIMITS_HI - JOINT_LIMITS_LO) + 1e-6
            jl_frac = np.clip(np.abs(q - jl_center) / jl_halfrange, 0.0, 1.0)
            r_joint_limit = R_JOINT_LIMIT_WT * float(np.sum(jl_frac ** 2))
        else:
            r_joint_limit = 0.0

        # Heading deviation from spawn yaw (wrapped to [-pi, pi]) + lateral
        # drift from spawn y. Pinning yaw RATE to zero is insufficient — the
        # integrated drift still accumulates. These penalise the deviation
        # itself, so the policy holds a straight line.
        if R_HEADING_WT != 0.0:
            yaw_now = math.atan2(float(ori[3]), float(ori[0]))
            dyaw = yaw_now - episode_start_yaw
            # wrap to [-pi, pi]
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            r_heading = R_HEADING_WT * (dyaw * dyaw)
        else:
            r_heading = 0.0
        if R_LATERAL_WT != 0.0:
            r_lateral = R_LATERAL_WT * float((pos[1] - episode_start_by) ** 2)
        else:
            r_lateral = 0.0

        reward = (r_lin + r_ang + r_vz + r_rp + r_wxy
                  + r_action_rate + r_joint_acc + r_alive + r_vx_bonus
                  + r_joint_limit + r_upright + r_heading + r_lateral)

        # Termination
        done = False
        info = {"bz": bz, "roll": roll, "pitch": pitch, "bx": float(pos[0])}
        if bz < BZ_FAIL:
            done = True
            info["fail"] = "bz"
            reward += R_TERM_PENALTY
        elif abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL:
            done = True
            info["fail"] = "tilt"
            reward += R_TERM_PENALTY
        elif episode_step >= MAX_EPISODE_STEPS:
            done = True
            info["fail"] = "timeout"
        # Sanitize reward (NaN can leak in from joint_acc when finite-diff
        # produces inf, e.g. on physics divergence).
        if not math.isfinite(reward):
            reward = -1.0
            done = True
            info["fail"] = "nan_reward"
        # Bound reward magnitude so a single weird tick doesn't dominate
        # PPO's advantage estimates. Range is configurable via env vars
        # so walk-bonus training can let reward exceed the old [-5,5] cap.
        reward = max(R_CLIP_LO, min(R_CLIP_HI, reward))

        return obs, float(reward), done, info

    # ── Main loop ──────────────────────────────────────────────────────
    # Cmd packet: 1 byte tag + 12 * 4 bytes action
    CMD_BYTES = 1 + ACT_DIM * 4
    # Obs packet: OBS_DIM*4 + 4 (reward) + 1 (done)
    OBS_BYTES = OBS_DIM * 4 + 4 + 1

    last_obs_reward_done = None

    while robot.step(step_ms) != -1:
        sim_time += step_dt
        episode_step += 1

        # Build obs/reward/done from CURRENT state.
        obs, reward, done, info = build_obs_reward_done()
        cum_reward += reward

        # Send to trainer.
        pkt = obs.tobytes() + struct.pack("<f", reward) + struct.pack("<B", 1 if done else 0)
        try:
            send_all(conn, pkt)
        except (ConnectionError, OSError):
            sys.stderr.write("[spot_rl_agent] trainer disconnected; exiting\n")
            break

        # Read action command.
        try:
            cmd = recv_exact(conn, CMD_BYTES)
        except (ConnectionError, OSError):
            sys.stderr.write("[spot_rl_agent] trainer disconnected mid-cmd; exiting\n")
            break
        tag = cmd[0]
        if tag == 2:  # QUIT
            sys.stderr.write("[spot_rl_agent] QUIT received\n")
            break
        if tag == 1 or done:  # RESET (or auto-reset on done)
            # Episode-end witness: net forward distance this episode.
            # A real walker shows fwd growing with episode length; a
            # B-mode "stand and farm alive bonus" policy shows fwd ~0
            # however long it survives. Logged one-in-N to avoid spam.
            _net_fwd = float(info.get("bx", 0.0)) - episode_start_bx
            if episode_step > 0 and (episode_step >= MAX_EPISODE_STEPS
                                     or np.random.random() < 0.02):
                sys.stderr.write(
                    f"[spot_rl_agent] ep end: steps={episode_step} "
                    f"cum_r={cum_reward:.1f} net_fwd={_net_fwd:+.3f}m "
                    f"fail={info.get('fail','-')}\n")
                sys.stderr.flush()
            reset_episode()
            # First post-reset action is ignored (use nominal pose this tick).
            continue

        # tag == 0 (ACTION)
        action = np.frombuffer(cmd, dtype=np.float32, offset=1, count=ACT_DIM).copy()
        # Clamp to [-1, 1] then scale to ACTION_SCALE rad
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        prev_action = last_action.copy()
        last_action = action.copy()
        cpg = cpg_base_offsets(sim_time, CPG_FREQ_HZ, CPG_HIP_Y_AMP, CPG_KNEE_AMP)
        target = NOMINAL_POSE + _pitch_trim_vec + cpg + ACTION_SCALE_RUN * action
        # Clamp to joint limits
        target = np.clip(target, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        apply_pose(target)

    try:
        conn.close()
    except Exception:
        pass
    try:
        srv.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
