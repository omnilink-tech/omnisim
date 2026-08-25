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

"""OMNIARM6 grasp residual-RL — in-process trainer with a PRE-GRASP PUSH.

One headless run does BOTH the manipulation episodes and the policy updates.
Each episode dumps a packed cluster of cubes, picks the TOP-most one as the
target, and lets a tiny Gaussian policy choose a 6-D action:

    [push_x, push_y,  grasp_dx, grasp_dy, grasp_dz, grasp_dyaw]

The first two dims are a PRE-GRASP PUSH (the action-space expansion the rigid
single-grasp lacks): the closed gripper descends beside the target and sweeps,
shoving it toward open space so the wide 2F-85 jaws can then get around it. The
push is GATED on its own magnitude (small -> skip, just grasp), so the policy
learns to push only when the cube is packed. The last four dims are the grasp-
pose residual applied to the (re-read) target after the push.

This is OmniSim's residual-RL recipe extended to MULTI-STEP manipulation
(declutter-then-grasp). Two reward channels keep the high-variance REINFORCE
update learnable:
  * sparse  : +1 if the target is lifted clear, minus small action costs.
  * shaped  : if a push happened, +/- for how much it changed the target's
              local CLEARANCE (min of wall gaps and nearest-neighbour gap) --
              this gives the push dims a dense signal independent of the noisy
              grasp outcome.

Env knobs:
  GRASP_EPISODES (default 2200), GRASP_BATCH (32), GRASP_SAVE_EVERY (150),
  GRASP_OUT (policy.onnx path), GRASP_LOG (text log path).
"""

import os
import sys
import math
import random

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(
    _HERE, "..", "..", "..", "samples", "demos", "controllers", "omnilink_arm_bridge"))

import numpy as np  # noqa: E402
import torch  # noqa: E402
import torch.nn as nn  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import ArmBridge, dls_ik_pose, _mat_mul  # noqa: E402
from _arm_configs import get_config  # noqa: E402

# ── Task / policy dims ───────────────────────────────────────────────
N_CLUT = 10                      # clutter cubes (target + N_CLUT = the pile)
N_NEAR = 6                       # neighbours described in the observation
OBS_DIM = 3 + N_NEAR * 3 + 4     # up-vec(3) + 6 neighbours(18) + 4 wall-gaps
ACT_DIM = 6                      # push_x, push_y, dx, dy, dz, dyaw
GRASP_SCALE = np.array([0.025, 0.025, 0.015, 0.79], dtype=np.float32)  # m,m,m,rad
OZ = 0.25                        # finger-throat tool offset
CX, CY = 0.46, 0.0               # bin centre
LIFT_OK = 0.16                   # target z above this after lift = success
CARRY_Z = 0.30

# Small training bin interior (walls at x 0.345/0.575, y +/-0.135, 0.025 thick).
# Wall gaps are normalised by a FIXED 0.12 m and clipped so "0 == packed against
# this wall" reads identically here and in the (larger) deploy bin -- the policy
# transfers because the clearance features are local, not bin-size-specific.
BIN_X_IN = (0.358, 0.562)
BIN_Y_IN = (-0.122, 0.122)
GAP_NORM = 0.12

# Pre-grasp push primitive.
PUSH_GATE = 0.40                 # |(push_x,push_y)| below this -> no push
PUSH_Z = 0.05                    # blade height (low, catches the cube sides)
PUSH_BACK = 0.05                 # descend this far on the -dir side of the target
PUSH_LEN = 0.08                  # sweep this far past the target along +dir
CUBE = 0.05

EPISODES = int(os.environ.get("GRASP_EPISODES", "2200"))
BATCH = int(os.environ.get("GRASP_BATCH", "32"))
SAVE_EVERY = int(os.environ.get("GRASP_SAVE_EVERY", "150"))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
OUT = os.environ.get("GRASP_OUT", os.path.join(_REPO, "projects", "rl", "inference", "policies", "omniarm6_grasp", "policy.onnx"))
LOG = os.environ.get("GRASP_LOG", os.path.join(_REPO, "_grasp_train.log"))

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id="robotiq_2f85_phys")
IK = bridge.cfg["ik"]
target = robot.getFromDef("TARGET")
clut = [robot.getFromDef("CLUT_%d" % i) for i in range(1, N_CLUT + 1)]
clut = [c for c in clut if c is not None]
ALL = [target] + clut            # the whole mini-pile
ep_target = target               # the cube being grasped this episode (topmost)
fh = open(LOG, "w", encoding="utf-8", buffering=1)


def log(s):
    fh.write(s + "\n")
    print(s, flush=True)


def step_for(secs):
    for _ in range(int(secs * 1000 / dt)):
        if robot.step(dt) == -1:
            return False
        bridge.tick(robot.getTime())
    return True


# ── Policy: obs -> mean action (tanh), fixed/annealed sigma ───────────
class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(OBS_DIM, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, ACT_DIM), nn.Tanh())

    def forward(self, x):
        return self.net(x)


policy = Policy()
opt = torch.optim.Adam(policy.parameters(), lr=3e-4)


def export_onnx(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    policy.eval()
    dummy = torch.zeros(1, OBS_DIM)
    torch.onnx.export(policy, dummy, path, input_names=["obs"],
                      output_names=["action"],
                      dynamic_axes={"obs": {0: "b"}, "action": {0: "b"}},
                      opset_version=14)
    policy.train()


# ── Scene helpers ────────────────────────────────────────────────────
def rand_rot():
    ax = [random.uniform(-1, 1) for _ in range(3)]
    n = math.sqrt(sum(a * a for a in ax)) or 1.0
    return [ax[0] / n, ax[1] / n, ax[2] / n, random.uniform(0.0, 0.6)]


def place(node, x, y, z, rot):
    node.getField("translation").setSFVec3f([x, y, z])
    node.getField("rotation").setSFRotation(rot)
    node.resetPhysics()


def reset_episode():
    """Dump the whole mini-pile in randomly (a dense, partly wall-packed jumble
    matching the deploy distribution), let it settle, then this episode's TARGET
    is the TOP-most cube -- exactly what the deploy camera would pick."""
    global ep_target
    for k, c in enumerate(ALL):                      # jittered layers, tight spread
        place(c, CX + random.uniform(-0.085, 0.085),
              CY + random.uniform(-0.085, 0.085),
              0.05 + 0.062 * (k % 4), rand_rot())
    step_for(1.7)                                    # settle into a dense pile
    ep_target = max(ALL, key=lambda c: c.getPosition()[2])


def up_vec(node):
    o = node.getOrientation()                        # 3x3 row-major; col2 = local +Z
    return [o[2], o[5], o[8]]


def wall_gaps(p):
    """4 normalised+clipped gaps from xy to each bin wall (0 == packed)."""
    return [
        min(1.5, max(0.0, (p[0] - BIN_X_IN[0]) / GAP_NORM)),
        min(1.5, max(0.0, (BIN_X_IN[1] - p[0]) / GAP_NORM)),
        min(1.5, max(0.0, (p[1] - BIN_Y_IN[0]) / GAP_NORM)),
        min(1.5, max(0.0, (BIN_Y_IN[1] - p[1]) / GAP_NORM)),
    ]


def clearance(node):
    """How graspable the target is right now (metres): min of its wall gaps and
    its gap to the nearest neighbour's surface. Bigger -> more room for the jaws."""
    p = node.getPosition()
    wall = min((p[0] - BIN_X_IN[0]), (BIN_X_IN[1] - p[0]),
               (p[1] - BIN_Y_IN[0]), (BIN_Y_IN[1] - p[1]))
    nb = min((math.dist(c.getPosition()[:2], p[:2]) for c in ALL if c is not node),
             default=0.30) - CUBE
    return min(wall, nb)


def observe():
    tp = ep_target.getPosition()
    nb = sorted(((math.dist(c.getPosition(), tp), c) for c in ALL if c is not ep_target),
                key=lambda t: t[0])[:N_NEAR]
    obs = list(up_vec(ep_target))
    for _, c in nb:
        p = c.getPosition()
        obs += [(p[0] - tp[0]) / 0.07, (p[1] - tp[1]) / 0.07, (p[2] - tp[2]) / 0.07]
    while len(obs) < 3 + N_NEAR * 3:
        obs += [0.0]
    obs += wall_gaps(tp)
    return np.asarray(obs[:OBS_DIM], dtype=np.float32), tp


def move_to(x, y, z, yaw, dur=1.1):
    """Top-down pose rotated about world Z by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    r_target = _mat_mul(rz, top)
    q, _, _, _ = dls_ik_pose(IK["chain"], bridge._read_q(), [x, y, z], r_target,
                             (0.0, 0.0, OZ), IK, bridge.joint_limits)
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + 0.25)


def push_clear(tx, ty, px, py):
    """PRE-GRASP PUSH: if the action magnitude clears the gate, bring the closed
    gripper down on the -dir side of the target and sweep +dir, shoving the
    target toward open space. Returns True if a push was executed."""
    m = math.hypot(px, py)
    if m < PUSH_GATE:
        return False
    dx, dy = px / m, py / m
    sx, sy = tx - dx * PUSH_BACK, ty - dy * PUSH_BACK
    ex, ey = tx + dx * PUSH_LEN, ty + dy * PUSH_LEN
    bridge.act_grasp()                               # close -> a thin blade
    step_for(0.3)
    move_to(sx, sy, CARRY_Z, 0.0)                    # stage above start
    move_to(sx, sy, PUSH_Z, 0.0)                     # descend beside the target
    move_to(ex, ey, PUSH_Z, 0.0, dur=1.3)            # sweep, shoving the target
    move_to(ex, ey, CARRY_Z, 0.0)                    # lift clear
    bridge.act_open_gripper()
    step_for(0.4)
    return True


def attempt_grasp(act):
    """Optional pre-grasp push, then the residual-adjusted grasp of this
    episode's target. Returns (reward, success)."""
    tp0 = ep_target.getPosition()
    clr0 = clearance(ep_target)
    pushed = push_clear(tp0[0], tp0[1], float(act[0]), float(act[1]))
    clr1 = clearance(ep_target) if pushed else clr0

    res = act[2:6] * GRASP_SCALE                     # grasp residual -> m/rad
    tp = ep_target.getPosition()                     # re-read after the push
    gx, gy, gz = tp[0] + res[0], tp[1] + res[1], tp[2] + res[2]
    yaw = float(res[3])
    bridge.act_set_gripper_width(0.062)
    step_for(0.4)
    move_to(gx, gy, gz + 0.16, yaw)                  # stage above
    move_to(gx, gy, gz, yaw)                         # descend
    bridge.act_grasp()
    step_for(1.2)
    move_to(gx, gy, CARRY_Z, yaw, dur=1.3)           # lift
    tz = ep_target.getPosition()[2]
    success = tz > LIFT_OK
    bridge.act_open_gripper()

    reward = (1.0 if success else 0.0) - 0.04 * float(np.sum(act[2:6] ** 2))
    if pushed:                                       # shaped push signal + cost
        reward += 0.3 * float(np.clip((clr1 - clr0) / 0.05, -1.0, 1.0)) - 0.04
    return reward, success, pushed


# ── Train loop (REINFORCE) ───────────────────────────────────────────
step_for(1.0)
log("[grasp-rl] start  OBS=%d ACT=%d (push+grasp)  episodes=%d batch=%d cubes=%d"
    % (OBS_DIM, ACT_DIM, EPISODES, BATCH, len(ALL)))
baseline = 0.0
buf_obs, buf_act, buf_rew = [], [], []
succ_window, push_window = [], []
SIGMA0, SIGMA1 = 0.5, 0.18

for ep in range(1, EPISODES + 1):
    sigma = SIGMA0 + (SIGMA1 - SIGMA0) * min(1.0, ep / (0.7 * EPISODES))
    try:
        reset_episode()
        obs, _ = observe()
        with torch.no_grad():
            mu = policy(torch.from_numpy(obs)[None, :])[0].numpy()
        act = np.clip(mu + np.random.randn(ACT_DIM).astype(np.float32) * sigma, -1.5, 1.5)
        reward, success, pushed = attempt_grasp(act)
    except Exception as exc:                          # one bad IK/episode != abort
        log("[grasp-rl] ep %4d SKIP (%s)" % (ep, exc))
        continue
    buf_obs.append(obs); buf_act.append(act.astype(np.float32)); buf_rew.append(reward)
    succ_window.append(1.0 if success else 0.0)
    push_window.append(1.0 if pushed else 0.0)
    if len(succ_window) > 100:
        succ_window.pop(0); push_window.pop(0)

    if ep % BATCH == 0:                               # REINFORCE update
        O = torch.from_numpy(np.stack(buf_obs))
        A = torch.from_numpy(np.stack(buf_act))
        R = torch.tensor(buf_rew, dtype=torch.float32)
        baseline = 0.9 * baseline + 0.1 * float(R.mean())
        adv = R - baseline
        mu_b = policy(O)
        logp = (-0.5 * ((A - mu_b) / sigma) ** 2).sum(dim=1)   # Gaussian, const dropped
        loss = -(adv * logp).mean()
        opt.zero_grad(); loss.backward(); opt.step()
        buf_obs, buf_act, buf_rew = [], [], []
        log("[grasp-rl] ep %4d  sig=%.3f  succ(100)=%.0f%%  push(100)=%.0f%%  base=%.3f  loss=%.3f"
            % (ep, sigma, 100.0 * (sum(succ_window) / max(1, len(succ_window))),
               100.0 * (sum(push_window) / max(1, len(push_window))), baseline, float(loss)))

    if ep % SAVE_EVERY == 0:
        export_onnx(OUT)
        log("[grasp-rl] saved policy -> %s (ep %d)" % (OUT, ep))

export_onnx(OUT)
log("[grasp-rl] DONE  final succ(100)=%.0f%%  push(100)=%.0f%%  -> %s"
    % (100.0 * (sum(succ_window) / max(1, len(succ_window))),
       100.0 * (sum(push_window) / max(1, len(push_window))), OUT))
robot.simulationQuit(0)
