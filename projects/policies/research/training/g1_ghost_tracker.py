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

"""Ghost-tracking RL for the Unitree G1, trained on the NEWTON deploy engine.

⚠️ STATUS (2026-06-26): this hand-rolled env is a DIAGNOSTIC RECORD, not the training
vehicle. The env faithfully reproduces the deploy (the BC clone walks 9.31 m no-fall in
it, n=1), BUT the newton `add_urdf` + `SolverMuJoCo` batched path CORRUPTS the shared
mujoco_warp solver under SYNCHRONISED MASS-RESET cascades (which from-scratch RL on a
marginally-stable operating point produces): once ~50% of worlds fall together, even a
freshly-reset world explodes to roll=pi / z<0 on the next step, permanently. Ruled out:
foot type, body/self collision (feet-only), njmax (2048), DR, CUDA graph, qacc_warmstart,
NaN, integrator (already implicitfast), duplicate ground planes, term threshold, reset
dynamics. The PROVEN `gpu_newton_g1_walk_trainer` (native prim builder) and the canonical
`gpu_humanoid_walk_trainer` (RAW mujoco_warp + MJCF) do NOT corrupt -- the difference is
the model-construction path, and the canonical trainer's STABLE deep-squat operating point
keeps falls sparse (never cascading).
THE PIVOT: ghost-imitation reward was added to the robust `gpu_humanoid_walk_trainer`
(raw-mjw + MJCF, same G1/H1), driven by a feasible deep-squat ghost
(`projects/policies/control/gait/g1_squat_ghost.py`). That is the live training path. Keep this file
for the deploy-faithful single-instance validation + the corruption diagnosis.

Original design notes follow.

The endgame of the Shadowing pipeline: a policy built ENTIRELY from a generated,
feasible ghost (no Unitree weights), that walks as well as the BC-from-Unitree clone.

Why Newton (not raw mujoco_warp): raw mujoco_warp can't hold the G1's point-contact
feet (the expert falls ~3s); Newton's SolverMuJoCo is the LITERAL deploy solver and
DOES hold the foot, GPU-batched. We build a batched Newton env on our Unitree
g1_12dof model (box feet for newton stability, knees-bent pose, Unitree per-joint
gains, act_scale 0.25, 47-d deploy obs) so train == deploy.

This module's __main__ VALIDATES the env first: roll the BC clone's actions in it
and confirm it walks (== the deploy). Only then is the ghost-tracking reward + PPO
worth running.
"""
import os
import sys
import math
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import torch

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

LEGS = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"]
NJ = 12
KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], np.float32)
KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], np.float32)
DEFAULT = np.array([-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0], np.float32)
ACT_SCALE = 0.25
SPAWN_Z = 0.80
GAIT_PERIOD = 0.8
FREE_DOF = 6
# 4-sphere Unitree URDF == the deploy model. newton.add_urdf loads all 8 foot
# spheres natively (verified shape_count=8), so train == g1_unitree_deploy.
URDF = str(REPO / "projects/robots/unitree/g1/g1_12dof/g1_12dof_clean.urdf")
SIM_DT = 0.002          # 500 Hz physics -- matches the BC clone's training/deploy rate
SUBSTEPS = 10           # -> 50 Hz policy (DT = 0.02)
SUB_DT = SIM_DT
DT = SIM_DT * SUBSTEPS
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")


def _build_builder():
    import warp as wp
    import newton
    tree = ET.parse(URDF)
    root = tree.getroot()
    # strip <visual> (render meshes blow up broad-phase when replicated). CRITICAL:
    # also strip <collision> from every link EXCEPT the feet. Body/self collisions in
    # degenerate fallen poses (which RL produces constantly) corrupt the shared batched
    # mujoco_warp solver -> a single cascade poisons even freshly-reset envs PERMANENTLY
    # (verified: post-cascade a clean reset + 1 step explodes to roll=pi, z<0). Primitive
    # feet only == what the proven gpu_newton trainer does.
    FOOT = "ankle_roll"
    for link in root.findall("link"):
        nm = link.get("name", "")
        for vis in list(link.findall("visual")):
            link.remove(vis)
        if FOOT not in nm:
            for col in list(link.findall("collision")):
                link.remove(col)
    urdf_xml = ET.tostring(root, encoding="unicode")
    mb = newton.ModelBuilder()
    mb.add_urdf(urdf_xml, xform=wp.transform((0.0, 0.0, SPAWN_Z), (0.0, 0.0, 0.0, 1.0)),
                floating=True)
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    # per-joint gains: actuated DOFs are 6..6+NJ in URDF (LEG) order.
    for i in range(NJ):
        d = FREE_DOF + i
        mb.joint_target_ke[d] = float(KP[i])
        mb.joint_target_kd[d] = float(KD[i])
        mb.joint_target_mode[d] = pv
    # NOTE: ground is added ONCE to the main builder after replicate (see __init__) --
    # adding it here would replicate N coincident infinite z=0 planes, so every foot
    # contacts N redundant ground planes -> degenerate stacked contacts that corrupt
    # the shared batched solver.
    return mb


def quat_to_grav(q):  # q (...,4) wxyz -> gravity in body frame
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return torch.stack([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)], -1)


class NewtonG1Env:
    def __init__(self, n, device="cuda:0"):
        import warp as wp
        import newton
        import mujoco
        self.wp, self.newton, self.mujoco = wp, newton, mujoco
        self.n = n
        self.device = wp.get_device(device)
        self.td = torch.device("cuda:0")

        g1 = _build_builder()
        main_b = newton.ModelBuilder()
        main_b.replicate(g1, world_count=n, spacing=(3.0, 3.0, 0.0))
        main_b.add_ground_plane()                  # ONE shared ground plane for all worlds
        self.model = main_b.finalize()
        self.solver = newton.solvers.SolverMuJoCo(
            self.model, use_mujoco_cpu=False,
            njmax=int(os.environ.get("G1_NJMAX", "96")),
            nconmax=int(os.environ.get("G1_NCONMAX", "96")))
        self.state_a = self.model.state(); self.state_b = self.model.state()
        self.control = self.model.control(); self.contacts = self.model.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_a)

        mjm = self.solver.mj_model
        self.nq, self.nv = int(mjm.nq), int(mjm.nv)
        self.dof_per_world = self.model.joint_dof_count // n
        name2qpos, name2dof = {}, {}
        for j in range(mjm.njnt):
            jn = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
            name2qpos[jn] = int(mjm.jnt_qposadr[j]); name2dof[jn] = int(mjm.jnt_dofadr[j])

        def find(s):
            if s in name2qpos: return s
            cand = [k for k in name2qpos if k.endswith(s)]
            if len(cand) != 1:
                raise RuntimeError(f"joint {s} not unique: {cand}")
            return cand[0]
        self.c2qpos = np.array([name2qpos[find(j)] for j in LEGS], np.int64)
        self.c2dof = np.array([name2dof[find(j)] for j in LEGS], np.int64)   # per-world-local
        # base free joint: the qpos adr 0..6 (pos+quat), qvel 0..5
        self.bq = 0; self.bv = 0
        # foot body ids (newton body index for foot xpos)
        self.fb = [mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, b) for b in FOOT_BODIES]

        self.qpos_t = wp.to_torch(self.solver.mjw_data.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.solver.mjw_data.qvel).view(n, self.nv)
        self.ctrl_pos_t = wp.to_torch(self.control.joint_target_q).view(-1)
        # flat write index for the NJ actuated joints across all worlds
        wi = np.concatenate([w * self.dof_per_world + FREE_DOF + (self.c2dof - FREE_DOF)
                             for w in range(n)])
        self.ctrl_write_idx = torch.tensor(wi, dtype=torch.long, device=self.td)
        self.qpos_idx_t = torch.tensor(self.c2qpos, dtype=torch.long, device=self.td)
        self.qvel_idx_t = torch.tensor(self.c2dof, dtype=torch.long, device=self.td)

        td = self.td
        self.default_t = torch.tensor(DEFAULT, device=td)
        # seed pose: base at spawn, identity quat, legs at default
        seed = np.zeros(self.nq, np.float32); seed[2] = SPAWN_Z; seed[3] = 1.0
        for i in range(NJ): seed[self.c2qpos[i]] = DEFAULT[i]
        self.seed_qpos_t = torch.tensor(seed, device=td)

        self.last_action = torch.zeros(n, NJ, device=td)
        self.phase = torch.zeros(n, device=td)
        self.cmd = torch.zeros(n, 3, device=td)
        self.ep = torch.zeros(n, dtype=torch.int32, device=td)
        self.max_ep = int(round(20.0 / DT))
        # the GENERATED+CALIBRATED ghost (q_ghost(phase)) -- the ONLY motion signal.
        gh = np.load(str(REPO / "_scratch/g1_ghost_calibrated.npz"))["q"]   # (256,12)
        self.ghost = torch.tensor(gh, dtype=torch.float32, device=td)
        self.W = self.ghost.shape[0]
        # ghost JOINT velocity (finite diff over one phase bin) -> reset states are
        # dynamically-consistent walking snapshots, not statically-unstable statues.
        gv = (np.roll(gh, -1, axis=0) - gh) / (GAIT_PERIOD / self.W)
        self.ghost_vel = torch.tensor(gv, dtype=torch.float32, device=td)
        self.walk_vx = 0.46                                   # the ghost's nominal forward speed
        self.res_scale = float(os.environ.get("G1_RES_SCALE", "0.15"))   # residual authority (rad)

        self._cuda_graph = None
        self._try_graph()
        self.reset(torch.arange(n, device=td))

    def ghost_at(self):
        b = (self.phase * self.W).long().clamp(0, self.W - 1)
        return self.ghost[b]                                  # (n,12)

    def _substeps(self):
        for _ in range(SUBSTEPS):
            self.state_a.clear_forces()
            self.model.collide(self.state_a, self.contacts)
            self.solver.step(self.state_a, self.state_b, self.control, self.contacts, SUB_DT)
            self.state_a, self.state_b = self.state_b, self.state_a

    def _try_graph(self):
        if os.environ.get("G1_NO_GRAPH"):
            return
        try:
            with self.wp.ScopedDevice(self.device):
                self._substeps(); self.wp.synchronize()
                self.wp.capture_begin(force_module_load=False)
                self._substeps()
                self._cuda_graph = self.wp.capture_end()
        except Exception as e:
            print(f"[newton-env] graph capture failed ({e}); direct", flush=True)
            self._cuda_graph = None

    def _physics(self):
        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                self._substeps()

    def reset(self, ids):
        if ids.numel() == 0:
            return
        m = ids.numel()
        self.qpos_t[ids] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[ids] = 0.0
        self.last_action[ids] = 0.0
        ph = torch.rand(m, device=self.td)                  # random start phase (RSI)
        self.phase[ids] = ph
        b = (ph * self.W).long().clamp(0, self.W - 1)
        # Seed a DYNAMICALLY-CONSISTENT walking snapshot at that phase: ghost leg pose,
        # ghost leg velocity, and the base moving forward at the walk speed. A frozen
        # (zero-velocity) mid-stride statue just tips -> ep_len 1; a moving one continues.
        self.qpos_t[ids.unsqueeze(1), self.qpos_idx_t.unsqueeze(0)] = self.ghost[b]
        self.qvel_t[ids.unsqueeze(1), self.qvel_idx_t.unsqueeze(0)] = self.ghost_vel[b]
        self.qvel_t[ids, 0] = self.walk_vx                  # base +x forward (identity quat)
        if hasattr(self, "ep"):
            self.ep[ids] = 0

    def _q(self): return self.qpos_t[:, self.qpos_idx_t]
    def _qd(self): return self.qvel_t[:, self.qvel_idx_t]

    def obs(self):
        quat = self.qpos_t[:, 3:7]
        ang_body = self.qvel_t[:, 3:6]
        ph = 2 * math.pi * self.phase
        return torch.cat([
            ang_body * 0.25, quat_to_grav(quat),
            self.cmd * torch.tensor([2.0, 2.0, 0.25], device=self.td),
            (self._q() - self.default_t), self._qd() * 0.05, self.last_action,
            torch.stack([torch.sin(ph), torch.cos(ph)], -1)], -1)

    def step(self, action, dr=True):
        action = action.clamp(-1.5, 1.5)
        # RESIDUAL-on-ghost: the GHOST is the feedforward (the gait), the policy learns a
        # BOUNDED balance correction. action=0 -> target=ghost (survives ~1s open-loop), so
        # a from-scratch policy starts on the walk instead of fighting it. "planning
        # describes (ghost), control solves (residual)."
        target = self.ghost_at() + self.res_scale * action
        self.ctrl_pos_t.index_copy_(0, self.ctrl_write_idx, target.reshape(-1))
        self._physics()
        self.ep += 1
        self.phase = (self.phase + DT / GAIT_PERIOD) % 1.0
        td = self.td
        # ── reward: track the GHOST (the only motion signal) + balance ──
        q = self._q(); quat = self.qpos_t[:, 3:7]
        grav = quat_to_grav(quat)
        z = self.base_z
        lin_world = self.qvel_t[:, 0:3]
        # body-frame forward velocity (ghost encodes ~0.46 m/s forward)
        w_, x_, y_, z_ = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
        fwd_x = 1 - 2 * (y_ * y_ + z_ * z_); fwd_y = 2 * (x_ * y_ + w_ * z_)
        vx_body = lin_world[:, 0] * fwd_x + lin_world[:, 1] * fwd_y
        terr = ((q - self.ghost_at()) ** 2).mean(1)
        r_track = 1.5 * torch.exp(-terr / 0.10)                       # PRIMARY: match the ghost pose
        r_alive = 0.10 * torch.ones(self.n, device=td)
        r_up = -1.0 * (grav[:, :2] ** 2).sum(1)
        r_h = -10.0 * (z - 0.77) ** 2
        r_vel = 0.5 * torch.exp(-((vx_body - 0.46) ** 2) / 0.10)      # ghost's own speed
        r_arate = -0.01 * ((action - self.last_action) ** 2).sum(1)
        reward = (r_track + r_alive + r_up + r_h + r_vel + r_arate)
        self.last_action = action
        roll = torch.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
        pitch = torch.asin((2 * (w_ * y_ - z_ * x_)).clamp(-1, 1))
        # terminate EARLY (shallow tilt), before the robot reaches a deep-penetration
        # fallen pose -- a deep fall feeds the shared batched mujoco_warp solver
        # degenerate contacts that corrupt it for ALL worlds. Catch the fall early.
        fell = (roll.abs() > 0.5) | (pitch.abs() > 0.5) | (z < 0.68) | (~torch.isfinite(z))
        timeout = self.ep >= self.max_ep
        done = fell | timeout
        if dr:                                                       # pushes (per-env)
            push = (torch.rand(self.n, device=td) < 0.01)
            if push.any():
                ids = push.nonzero(as_tuple=False).squeeze(-1)
                self.qvel_t[ids, 0:2] += (torch.rand(ids.numel(), 2, device=td) - 0.5) * 2 * 1.0
        self.reset(done.nonzero(as_tuple=False).squeeze(-1))
        return self.obs(), reward, fell, done

    @property
    def base_z(self): return self.qpos_t[:, 2]
    @property
    def base_x(self): return self.qpos_t[:, 0]


def _validate():
    """Roll the BC clone in the Newton env -> does it walk (== deploy)?"""
    env = NewtonG1Env(1)
    # index-map / state sanity right after reset
    q0 = env._q()[0].cpu().numpy(); bz = env.base_z.item()
    grav = quat_to_grav(env.qpos_t[:, 3:7])[0].cpu().numpy()
    print(f"[verify] base_z={bz:.3f} (want {SPAWN_Z})  q-default max|err|={np.abs(q0-DEFAULT).max():.3f} (want ~0)  "
          f"proj_grav={np.round(grav,2)} (want [0,0,-1])", flush=True)
    pol = torch.jit.load(str(REPO / "projects/policies/controllers/g1_unitree_deploy/g1_bc_walk.pt")).eval()
    env.cmd[:] = torch.tensor([0.5, 0.0, 0.0], device=env.td)
    obs = env.obs(); x0 = env.base_x.clone(); minz = 9.0; fell=None
    for t in range(int(20 / DT)):
        with torch.no_grad():
            a = pol(obs.cpu()).to(env.td)
        obs = env.step(a)
        z = env.base_z.item(); minz = min(minz, z)
        if z < 0.5 and fell is None: fell = t*DT
        if t % int(2.5/DT) == 0:
            print(f"  t={t*DT:5.1f}s x={env.base_x.item():+.2f} z={z:.3f}", flush=True)
    dx = (env.base_x - x0).item()
    print(f">>> BC clone in NEWTON env (n=1): dx={dx:+.2f}m  min_z={minz:.3f}  "
          f"{'FELL@%.1fs'%fell if fell else 'NEVER FELL (newton holds the foot!)'}", flush=True)


import torch.nn as nn


class AC(nn.Module):
    def __init__(self, obs=47, act=NJ, hidden=(256, 128), init_std=1.0):
        super().__init__()
        def mlp(o):
            L, d = [], obs
            for h in hidden:
                L += [nn.Linear(d, h), nn.ELU()]; d = h
            return nn.Sequential(*L, nn.Linear(d, o))
        self.actor = mlp(act); self.critic = mlp(1)
        self.log_std = nn.Parameter(np.log(init_std) * torch.ones(act))

    def act(self, o):
        mean = self.actor(o); std = self.log_std.exp()
        dist = torch.distributions.Normal(mean, std); a = dist.sample()
        return a, dist.log_prob(a).sum(-1), self.critic(o).squeeze(-1)

    def evaluate(self, o, a):
        mean = self.actor(o); dist = torch.distributions.Normal(mean, self.log_std.exp())
        return dist.log_prob(a).sum(-1), dist.entropy().sum(-1), self.critic(o).squeeze(-1)


def _save(ac, path):
    import copy
    a = copy.deepcopy(ac.actor).cpu().eval()
    class Pol(nn.Module):
        def __init__(s, net): super().__init__(); s.net = net
        def forward(s, o): return s.net(o).clamp(-1.5, 1.5)   # matches env action clamp
    ts = torch.jit.trace(Pol(a), torch.zeros(1, 47))
    torch.jit.save(ts, path)


def _pretrain_from_ghost(env, ac, dev, steps=4000, epochs=300):
    """BC-pretrain the actor on the OPEN-LOOP GHOST action (the only motion source,
    NOT Unitree) so RL starts already doing the gait instead of from a face-plant."""
    O, A = [], []
    env.reset(torch.arange(env.n, device=env.td)); obs = env.obs()
    for t in range(steps):
        a_ghost = ((env.ghost_at() - env.default_t) / ACT_SCALE)   # action that targets the ghost
        O.append(obs.detach()); A.append(a_ghost.detach())
        obs, _, _, _ = env.step(a_ghost, dr=False)
    Ot = torch.cat(O, 0); At = torch.cat(A, 0); N = Ot.shape[0]
    opt = torch.optim.Adam(ac.actor.parameters(), lr=1e-3)
    for e in range(epochs):
        idx = torch.randperm(N, device=dev)
        for s in range(0, N, 8192):
            j = idx[s:s + 8192]
            loss = (ac.actor(Ot[j]) - At[j]).pow(2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
    print(f"[ghost-tracker] BC-pretrain on open-loop ghost done (N={N}, loss={loss.item():.4f})", flush=True)


def train(args):
    dev = "cuda:0"
    torch.manual_seed(args.seed); np.random.seed(args.seed)
    env = NewtonG1Env(args.envs)
    env.cmd[:] = torch.tensor([0.5, 0.0, 0.0], device=env.td)
    ac = AC(init_std=args.init_std).to(dev)
    print(f"[ghost-tracker] FROM SCRATCH from the GHOST only (no Unitree): envs={args.envs}", flush=True)
    if not args.no_pretrain:
        _pretrain_from_ghost(env, ac, dev)
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    lr = args.lr; obs = env.obs(); T, B = args.steps, args.envs
    import time
    for it in range(args.iters):
        t0 = time.time()
        ob = torch.zeros(T, B, 47, device=dev); ac_b = torch.zeros(T, B, NJ, device=dev)
        lp = torch.zeros(T, B, device=dev); vv = torch.zeros(T, B, device=dev)
        rr = torch.zeros(T, B, device=dev); dd = torch.zeros(T, B, device=dev); to = torch.zeros(T, B, device=dev)
        with torch.no_grad():
            for t in range(T):
                a, l, v = ac.act(obs); ob[t] = obs; ac_b[t] = a; lp[t] = l; vv[t] = v
                obs, rew, fell, done = env.step(a)
                rr[t] = rew; dd[t] = done.float(); to[t] = (done & ~fell).float()
            _, _, lastv = ac.act(obs)
        adv = torch.zeros(T, B, device=dev); gae = torch.zeros(B, device=dev)
        for t in reversed(range(T)):
            nv = lastv if t == T - 1 else vv[t + 1]
            rt = rr[t] + args.gamma * nv * to[t]; nt = 1 - dd[t]
            delta = rt + args.gamma * nv * nt - vv[t]; gae = delta + args.gamma * args.lam * nt * gae
            adv[t] = gae
        ret = adv + vv; adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        of = ob.reshape(-1, 47); af = ac_b.reshape(-1, NJ); lpf = lp.reshape(-1); advf = adv.reshape(-1); retf = ret.reshape(-1)
        N = of.shape[0]; mb = N // args.minibatches; kl = 0.0
        warm = it < args.critic_warmup       # train ONLY the critic first so advantages are
        for _ in range(args.epochs):         # calibrated before the actor steps off the knife-edge
            idx = torch.randperm(N, device=dev)
            for s in range(0, N, mb):
                j = idx[s:s + mb]
                nlp, ent, v = ac.evaluate(of[j], af[j]); ratio = (nlp - lpf[j]).exp()
                pl = -torch.min(ratio * advf[j], ratio.clamp(1 - args.clip, 1 + args.clip) * advf[j]).mean()
                vl = 0.5 * (v - retf[j]).pow(2).mean()
                loss = vl if warm else (pl + vl - args.ent * ent.mean())
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(ac.parameters(), 1.0); opt.step()
                with torch.no_grad(): kl = ((ratio - 1) - (nlp - lpf[j])).mean().item()
        if kl > 2 * args.kl: lr = max(1e-5, lr / 1.5)
        elif 0 < kl < 0.5 * args.kl: lr = min(args.lr, lr * 1.5)   # never grow ABOVE the start lr
        for g in opt.param_groups: g["lr"] = lr
        if it % args.log_every == 0 or it == args.iters - 1:
            fps = (T * B) / (time.time() - t0)
            print(f"it {it:5d} | rew/step {rr.mean():.3f} | ep_len {(1/(dd.mean()+1e-6)):.0f} "
                  f"| kl {kl:.3f} | {fps:,.0f} st/s", flush=True)
        if args.save and (it % args.save_every == 0 or it == args.iters - 1):
            _save(ac, args.save)
    if args.save:
        _save(ac, args.save); print(f"saved {args.save}", flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--validate", action="store_true")
    p.add_argument("--envs", type=int, default=1024); p.add_argument("--steps", type=int, default=24)
    p.add_argument("--iters", type=int, default=2000); p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99); p.add_argument("--lam", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2); p.add_argument("--ent", type=float, default=0.01)
    p.add_argument("--epochs", type=int, default=5); p.add_argument("--minibatches", type=int, default=4)
    p.add_argument("--kl", type=float, default=0.01); p.add_argument("--init-std", type=float, default=1.0)
    p.add_argument("--seed", type=int, default=1); p.add_argument("--save", type=str, default="")
    p.add_argument("--save-every", type=int, default=200); p.add_argument("--log-every", type=int, default=20)
    p.add_argument("--no-pretrain", action="store_true")
    p.add_argument("--critic-warmup", type=int, default=25)
    a = p.parse_args()
    if a.validate:
        _validate()
    else:
        train(a)
