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

"""OMNIARM6 SEQUENTIAL declutter — in-process PPO trainer (multi-step manipulation).

This is the real fix for the dense-bin endgame the single-step residual could
not crack: a wall-locked cube needs a SEQUENCE of actions (push a blocker, maybe
push another, then grasp) where each action changes the scene for the next. That
is a multi-step credit-assignment problem, so we move from single-step REINFORCE
to PPO with a VALUE FUNCTION.

Each episode places a tight, wall-biased cluster of cubes and names one the
TARGET. At every macro-step the actor-critic observes the local scene and emits
a 7-D action:

    [mode,  push_x, push_y,  grasp_dx, grasp_dy, grasp_dz, grasp_dyaw]

mode>0 => GRASP the target with the residual (dx,dy,dz,dyaw); mode<=0 => a
NEIGHBOR-CLEARING PUSH in direction (push_x,push_y): the closed gripper descends
on the blocking neighbour and sweeps it away, opening a gap. The episode ends
when the target is lifted clear (reward +1) or the step budget is spent. A push
costs a little and is additionally shaped by how much it raises the target's
CLEARANCE, so the value head can learn "push now -> grasp later" fast.

The trained actor (mode + push + grasp heads) is exported to ONNX; the
bin-picking deploy controller runs it as a per-macro-step push/grasp LOOP.

Env knobs: DECL_ITERS (PPO iters, default 240), DECL_EPISODES_PER (16),
DECL_MAX_STEPS (4), DECL_SAVE_EVERY (20), DECL_OUT, DECL_LOG.
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
from torch.distributions import Normal  # noqa: E402

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import ArmBridge, dls_ik_pose, _mat_mul  # noqa: E402
from _arm_configs import get_config  # noqa: E402

# ── Task / policy dims ───────────────────────────────────────────────
N_CLUT = 10
N_NEAR = 6
OBS_DIM = 3 + N_NEAR * 3 + 4 + 1     # up-vec + 6 nb + 4 wall-gaps + clearance
ACT_DIM = 7                          # mode, push_x, push_y, dx, dy, dz, dyaw
GRASP_SCALE = np.array([0.025, 0.025, 0.015, 0.79], dtype=np.float32)
OZ = 0.25
CX, CY = 0.46, 0.0
LIFT_OK = 0.16
CARRY_Z = 0.30
CUBE = 0.05

BIN_X_IN = (0.358, 0.562)
BIN_Y_IN = (-0.122, 0.122)
GAP_NORM = 0.12

# Neighbour-clearing push.
PUSH_Z = 0.05
PUSH_CELL = 0.052                    # descend one cube over, on the +dir blocker
PUSH_LEN = 0.07                      # sweep the blocker this far away

# Fast training motions (the policy learns DECISIONS, not realistic speeds).
MV = 0.5

ITERS = int(os.environ.get("DECL_ITERS", "240"))
EP_PER = int(os.environ.get("DECL_EPISODES_PER", "16"))
MAX_STEPS = int(os.environ.get("DECL_MAX_STEPS", "4"))
SAVE_EVERY = int(os.environ.get("DECL_SAVE_EVERY", "20"))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
OUT = os.environ.get("DECL_OUT", os.path.join(_REPO, "projects", "rl", "inference", "policies", "omniarm6_declutter", "policy.onnx"))
LOG = os.environ.get("DECL_LOG", os.path.join(_REPO, "_declutter_train.log"))

GAMMA, LAM, CLIP, PPO_EPOCHS = 0.99, 0.95, 0.2, 4

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id="robotiq_2f85_phys")
IK = bridge.cfg["ik"]
target = robot.getFromDef("TARGET")
clut = [robot.getFromDef("CLUT_%d" % i) for i in range(1, N_CLUT + 1)]
clut = [c for c in clut if c is not None]
ALL = [target] + clut
active = list(ALL)
ep_target = target
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


# ── Actor-critic ─────────────────────────────────────────────────────
class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.trunk = nn.Sequential(
            nn.Linear(OBS_DIM, 128), nn.Tanh(), nn.Linear(128, 128), nn.Tanh())
        self.mu = nn.Linear(128, ACT_DIM)
        self.log_std = nn.Parameter(torch.full((ACT_DIM,), -0.4))
        self.val = nn.Linear(128, 1)

    def forward(self, x):
        h = self.trunk(x)
        return self.mu(h), self.log_std.exp(), self.val(h).squeeze(-1)

    def act(self, x):
        mu, std, v = self.forward(x)
        d = Normal(mu, std)
        a = d.sample()
        return a, d.log_prob(a).sum(-1), v

    def evaluate(self, x, a):
        mu, std, v = self.forward(x)
        d = Normal(mu, std)
        return d.log_prob(a).sum(-1), d.entropy().sum(-1), v


ac = ActorCritic()
opt = torch.optim.Adam(ac.parameters(), lr=3e-4)


class ActorMu(nn.Module):
    """Deterministic actor (mu only) for ONNX export / deploy."""
    def __init__(self, src):
        super().__init__()
        self.trunk, self.mu = src.trunk, src.mu

    def forward(self, x):
        return self.mu(self.trunk(x))


def export_onnx(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    m = ActorMu(ac).eval()
    torch.onnx.export(m, torch.zeros(1, OBS_DIM), path, input_names=["obs"],
                      output_names=["action"],
                      dynamic_axes={"obs": {0: "b"}, "action": {0: "b"}},
                      opset_version=14)


# ── Scene helpers ────────────────────────────────────────────────────
def rand_rot():
    ax = [random.uniform(-1, 1) for _ in range(3)]
    n = math.sqrt(sum(a * a for a in ax)) or 1.0
    return [ax[0] / n, ax[1] / n, ax[2] / n, random.uniform(0.0, 0.5)]


def place(node, x, y, z, rot):
    node.getField("translation").setSFVec3f([x, y, z])
    node.getField("rotation").setSFRotation(rot)
    node.resetPhysics()


def reset_scenario():
    """Tight, often wall-biased cluster of cubes; the TARGET is usually a
    blocked / wall-adjacent cube so a push is needed before the grasp."""
    global ep_target, active
    n = random.randint(4, len(ALL))
    pool = list(ALL); random.shuffle(pool)
    active, inactive = pool[:n], pool[n:]
    if random.random() < 0.6:                        # bias the cluster to a wall
        w = random.choice((0, 1, 2, 3))
        cx = {0: BIN_X_IN[0] + 0.03, 1: BIN_X_IN[1] - 0.03}.get(w, None)
        cy = {2: BIN_Y_IN[0] + 0.03, 3: BIN_Y_IN[1] - 0.03}.get(w, None)
        if cx is None:
            cx = random.uniform(BIN_X_IN[0] + 0.04, BIN_X_IN[1] - 0.04)
        if cy is None:
            cy = random.uniform(BIN_Y_IN[0] + 0.04, BIN_Y_IN[1] - 0.04)
    else:
        cx = random.uniform(BIN_X_IN[0] + 0.04, BIN_X_IN[1] - 0.04)
        cy = random.uniform(BIN_Y_IN[0] + 0.04, BIN_Y_IN[1] - 0.04)
    for k, c in enumerate(active):                   # tight cluster, jaws blocked
        place(c, cx + random.uniform(-0.04, 0.04), cy + random.uniform(-0.04, 0.04),
              0.05 + 0.06 * (k % 3), rand_rot())
    for i, c in enumerate(inactive):                 # park far outside the bin
        place(c, 1.5 + 0.08 * i, -0.6, 0.05, [0, 0, 1, 0])
    step_for(1.3)
    # target = a blocked cube most of the time (lowest clearance), else topmost
    if random.random() < 0.6:
        ep_target = min(active, key=clearance)
    else:
        ep_target = max(active, key=lambda c: c.getPosition()[2])


def up_vec(node):
    o = node.getOrientation()
    return [o[2], o[5], o[8]]


def wall_gaps(p):
    return [min(1.5, max(0.0, (p[0] - BIN_X_IN[0]) / GAP_NORM)),
            min(1.5, max(0.0, (BIN_X_IN[1] - p[0]) / GAP_NORM)),
            min(1.5, max(0.0, (p[1] - BIN_Y_IN[0]) / GAP_NORM)),
            min(1.5, max(0.0, (BIN_Y_IN[1] - p[1]) / GAP_NORM))]


def clearance(node):
    p = node.getPosition()
    wall = min((p[0] - BIN_X_IN[0]), (BIN_X_IN[1] - p[0]),
               (p[1] - BIN_Y_IN[0]), (BIN_Y_IN[1] - p[1]))
    nb = min((math.dist(c.getPosition()[:2], p[:2]) for c in active if c is not node),
             default=0.30) - CUBE
    return min(wall, nb)


def observe():
    tp = ep_target.getPosition()
    nb = sorted(((math.dist(c.getPosition(), tp), c) for c in active if c is not ep_target),
                key=lambda t: t[0])[:N_NEAR]
    obs = list(up_vec(ep_target))
    for _, c in nb:
        p = c.getPosition()
        obs += [(p[0] - tp[0]) / 0.07, (p[1] - tp[1]) / 0.07, (p[2] - tp[2]) / 0.07]
    while len(obs) < 3 + N_NEAR * 3:
        obs += [0.0]
    obs += wall_gaps(tp)
    obs += [min(1.5, max(-0.5, clearance(ep_target) / 0.05))]
    return np.asarray(obs[:OBS_DIM], dtype=np.float32)


def move_to(x, y, z, yaw, dur=MV):
    c, s = math.cos(yaw), math.sin(yaw)
    rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    top = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    q, _, _, _ = dls_ik_pose(IK["chain"], bridge._read_q(), [x, y, z],
                             _mat_mul(rz, top), (0.0, 0.0, OZ), IK, bridge.joint_limits)
    bridge.act_set_joint_positions(q, duration_s=dur)
    step_for(dur + 0.12)


def do_push(p):
    """NEIGHBOR-CLEARING push: descend the closed gripper on the +dir blocker
    and sweep it away, opening a gap on the target's +dir side."""
    px, py = float(p[0]), float(p[1])
    m = math.hypot(px, py) or 1.0
    dx, dy = px / m, py / m
    tp = ep_target.getPosition()
    sx, sy = tp[0] + dx * PUSH_CELL, tp[1] + dy * PUSH_CELL
    ex, ey = tp[0] + dx * (PUSH_CELL + PUSH_LEN), tp[1] + dy * (PUSH_CELL + PUSH_LEN)
    bridge.act_grasp()
    step_for(0.25)
    move_to(sx, sy, CARRY_Z, 0.0)
    move_to(sx, sy, PUSH_Z, 0.0)
    move_to(ex, ey, PUSH_Z, 0.0, dur=0.7)
    move_to(ex, ey, CARRY_Z, 0.0)
    bridge.act_open_gripper()
    step_for(0.3)


def do_grasp(g):
    """Residual-adjusted grasp of the target. Returns True if lifted clear."""
    res = np.asarray(g, dtype=np.float32) * GRASP_SCALE
    tp = ep_target.getPosition()
    gx, gy, gz = tp[0] + res[0], tp[1] + res[1], tp[2] + res[2]
    yaw = float(res[3])
    bridge.act_set_gripper_width(0.062)
    step_for(0.3)
    move_to(gx, gy, gz + 0.15, yaw)
    move_to(gx, gy, gz, yaw)
    bridge.act_grasp()
    step_for(0.9)
    move_to(gx, gy, CARRY_Z, yaw, dur=0.7)
    lifted = ep_target.getPosition()[2] > LIFT_OK
    bridge.act_open_gripper()
    return lifted


def in_bounds(node):
    p = node.getPosition()
    return (BIN_X_IN[0] - 0.06 <= p[0] <= BIN_X_IN[1] + 0.06 and
            BIN_Y_IN[0] - 0.06 <= p[1] <= BIN_Y_IN[1] + 0.06)


def do_action(a):
    """Execute one macro-step. Returns (reward, done, success)."""
    a = a.detach().numpy()
    if a[0] > 0.0:                                    # GRASP
        if do_grasp(a[3:7]):
            return 1.0, True, True
        if not in_bounds(ep_target):
            return -0.3, True, False
        return 0.0, False, False
    # PUSH (neighbour-clearing), shaped by clearance gain
    c0 = clearance(ep_target)
    do_push(a[1:3])
    if not in_bounds(ep_target):                      # pushed the target out
        return -0.3, True, False
    dc = clearance(ep_target) - c0
    return -0.05 + 0.3 * float(np.clip(dc / 0.05, -1.0, 1.0)), False, False


# ── PPO rollout + update ─────────────────────────────────────────────
def gae_episode(rews, vals, boot):
    """GAE for one episode. vals = V(obs_t); boot = V(obs_T) (0 if terminal)."""
    T = len(rews)
    adv = [0.0] * T
    last = 0.0
    for t in reversed(range(T)):
        nextv = vals[t + 1] if t + 1 < T else boot
        delta = rews[t] + GAMMA * nextv - vals[t]
        last = delta + GAMMA * LAM * last
        adv[t] = last
    ret = [adv[t] + vals[t] for t in range(T)]
    return adv, ret


def collect_batch():
    O, A, LP, ADV, RET = [], [], [], [], []
    succ, steps_used = 0, 0
    for _ in range(EP_PER):
        reset_scenario()
        eO, eA, eLP, eV, eR = [], [], [], [], []
        boot = 0.0
        for t in range(MAX_STEPS):
            obs = observe()
            with torch.no_grad():
                a, lp, v = ac.act(torch.from_numpy(obs)[None, :])
            rew, done, success = do_action(a[0])
            rew -= 0.02                               # per-step time cost
            eO.append(obs); eA.append(a[0].numpy()); eLP.append(float(lp))
            eV.append(float(v)); eR.append(rew)
            steps_used += 1
            if done:
                succ += 1 if success else 0
                break
            if t == MAX_STEPS - 1:                    # truncated -> bootstrap
                with torch.no_grad():
                    _, _, bv = ac.act(torch.from_numpy(observe())[None, :])
                boot = float(bv)
        adv, ret = gae_episode(eR, eV, boot)
        O += eO; A += eA; LP += eLP; ADV += adv; RET += ret
    return O, A, LP, ADV, RET, succ, steps_used


step_for(1.0)
log("[declutter] start  OBS=%d ACT=%d  iters=%d ep/iter=%d max_steps=%d"
    % (OBS_DIM, ACT_DIM, ITERS, EP_PER, MAX_STEPS))

for it in range(1, ITERS + 1):
    O, A, LP, adv, ret, succ, steps_used = collect_batch()
    O = torch.tensor(np.stack(O), dtype=torch.float32)
    A = torch.tensor(np.stack(A), dtype=torch.float32)
    LP = torch.tensor(LP, dtype=torch.float32)
    ADV = torch.tensor(adv, dtype=torch.float32)
    RET = torch.tensor(ret, dtype=torch.float32)
    ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-6)

    for _ in range(PPO_EPOCHS):
        lp, ent, v = ac.evaluate(O, A)
        ratio = (lp - LP).exp()
        s1 = ratio * ADV
        s2 = torch.clamp(ratio, 1 - CLIP, 1 + CLIP) * ADV
        pol_loss = -torch.min(s1, s2).mean()
        val_loss = ((v - RET) ** 2).mean()
        loss = pol_loss + 0.5 * val_loss - 0.01 * ent.mean()
        opt.zero_grad(); loss.backward()
        nn.utils.clip_grad_norm_(ac.parameters(), 1.0)
        opt.step()

    if it % 5 == 0 or it == 1:
        log("[declutter] it %3d  succ=%2d/%d  avg_steps=%.2f  ret=%.3f  vloss=%.3f  std=%.2f"
            % (it, succ, EP_PER, steps_used / EP_PER, float(RET.mean()),
               float(val_loss), float(ac.log_std.exp().mean())))
    if it % SAVE_EVERY == 0:
        export_onnx(OUT)
        log("[declutter] saved -> %s (it %d)" % (OUT, it))

export_onnx(OUT)
log("[declutter] DONE -> %s" % OUT)
robot.simulationQuit(0)
