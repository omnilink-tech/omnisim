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

"""DAgger / MPC-distillation for the G1 sit-stand LAUNCH (Shadowing -- the principled fix).

The dead-seated push-off is the ONE piece RL-on-reference never learned (21 runs): it is
open-loop unstable AND the Ghost Generator's open-loop controls don't transfer across engines,
so feedforward/replay/hybrid-assist all drift. DAgger (Ross et al. 2011) fixes this the
principled way -- the policy is trained on the EXPERT's action at the states the POLICY
actually visits, so it learns the stabilizing CLOSED-LOOP feedback, not a brittle open-loop
replay.

  EXPERT = the Ghost Generator's receding-horizon MPPI (ghost_generator.GhostGenerator), which
           DOES execute the launch closed-loop in MuJoCo (it discovered the feasible rise).
           Here we expose it as act(state, t) -> position-target action (one MPPI re-plan from
           an ARBITRARY state -- exactly the corrective label DAgger needs).
  POLICY = a small MLP (proprio + base + phase clock) -> position targets. Deployable obs.
  LOOP   = roll out the current policy in MuJoCo; query the expert at each visited state;
           aggregate (obs, expert_action); supervised-train; repeat with a beta-annealed
           expert/policy mix. Eval = a PURE-policy rollout that stands up from dead-seated.

Run a smoke first:  python -m projects.policies.research.shadowing.dagger_launch --smoke
Then the real run:  python -m projects.policies.research.shadowing.dagger_launch --iters 8 --rollouts 3
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent, _interp_keys
import projects.policies.control.gait.g1_sitstand as GS

MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml")
OUTDIR = os.path.join(REPO, "projects/policies/research/shadowing/runs")


# ----------------------------------------------------------------------------- intent (chair)
def build_intent():
    """The feasible chair sit->lean->rise->step->stand->HOLD intent the MPPI solves (the same
    one generate_g1_sitstand.py uses -- copied so DAgger and the ghost share the expert)."""
    seated = GS.SEATED_POSE
    seated_full = dict(seated)
    stand_full = dict(seated)
    stand_full.update({
        "left_hip_pitch_joint": -0.33, "right_hip_pitch_joint": -0.33,
        "left_hip_roll_joint": 0.0, "right_hip_roll_joint": 0.0,
        "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
        "left_knee_joint": 0.55, "right_knee_joint": 0.55,
        "left_ankle_pitch_joint": -0.20, "right_ankle_pitch_joint": -0.20,
        "left_ankle_roll_joint": 0.0, "right_ankle_roll_joint": 0.0, "waist_yaw_joint": 0.0,
        "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.15,
        "left_shoulder_yaw_joint": 0.0, "left_elbow_joint": 0.0, "left_wrist_roll_joint": 0.0,
        "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.15,
        "right_shoulder_yaw_joint": 0.0, "right_elbow_joint": 0.0, "right_wrist_roll_joint": 0.0,
    })
    return Intent(
        total_time=6.0,
        joint_keys=[(0.0, seated_full), (1.2, seated_full), (3.0, stand_full), (6.0, stand_full)],
        base_keys=[(0.0, {"x": 0.15, "z": 0.56, "pitch": 0.0}),
                   (1.5, {"x": 0.28, "z": 0.60, "pitch": 0.30}),
                   (3.0, {"x": 0.40, "z": 0.76, "pitch": 0.0}),
                   (6.0, {"x": 0.40, "z": 0.76, "pitch": 0.0})],
        weights=dict(joint=3.0, base_xy=3.0, base_z=10.0, pitch=4.0,
                     upright=3.0, balance=5.0, effort=0.02, smooth=0.05, alive=0.5),
    )


# --------------------------------------------------------------------------------- the expert
class ExpertMPC:
    """One MPPI re-plan from an arbitrary state = the DAgger expert. Wraps GhostGenerator's
    per-step MPPI (sample -> rollout-cost -> softmax -> first action) but plans from whatever
    state it's handed, keeping its own warm-start so it stays a proper receding-horizon MPC."""

    def __init__(self, gen: GhostGenerator, intent: Intent, n_samples=48, horizon=12,
                 noise=0.18, temperature=0.5, seed=0):
        self.gen = gen
        self.intent = intent
        self.n_samples = n_samples
        self.horizon = horizon
        self.noise = noise
        self.temperature = temperature
        self.rng = np.random.default_rng(seed)
        self.np_pos = len(gen.pos_act)
        self.scratch = mujoco.MjData(gen.m)
        self.nominal = None

    def reset(self, d):
        # warm-start = hold the current joint targets across the horizon
        self.nominal = np.tile(d.qpos[self.gen.act_jqpos], (self.horizon, 1))

    def act(self, d, t, warm=None):
        """Return the position-target action the MPPI would apply from state `d` at time `t`.

        warm: optional (horizon, n_pos) warm-start nominal (e.g. the reference horizon) so a
        low-noise expert TRACKS a known-good trajectory and only deviates to correct drift --
        clean, near-deterministic labels for distilling an otherwise stochastic launch."""
        gen, intent = self.gen, self.intent
        if warm is not None:
            self.nominal = np.clip(warm, gen.ctrl_lo, gen.ctrl_hi)
        if self.nominal is None:
            self.reset(d)
        dU = self.rng.normal(0.0, self.noise, size=(self.n_samples, self.horizon, self.np_pos))
        for h in range(1, self.horizon):
            dU[:, h] = 0.6 * dU[:, h] + 0.4 * dU[:, h - 1]
        samples = self.nominal[None] + dU
        samples[0] = self.nominal
        samples = np.clip(samples, gen.ctrl_lo, gen.ctrl_hi)
        costs = np.empty(self.n_samples)
        for s in range(self.n_samples):
            self.scratch.qpos[:] = d.qpos
            self.scratch.qvel[:] = d.qvel
            self.scratch.time = t
            mujoco.mj_forward(gen.m, self.scratch)
            costs[s] = gen._rollout_cost(self.scratch, samples[s], intent, t)
        beta = costs.min()
        wgt = np.exp(-(costs - beta) / max(1e-6, self.temperature))
        wgt /= wgt.sum()
        nominal = np.einsum("s,shc->hc", wgt, samples)
        nominal = np.clip(nominal, gen.ctrl_lo, gen.ctrl_hi)
        action = nominal[0].copy()
        # receding-horizon warm-start shift for the NEXT query
        self.nominal = np.vstack([nominal[1:], nominal[-1:]])
        return action


# --------------------------------------------------------------------------------- env helpers
def seated_init(gen: GhostGenerator, rng=None, dr=0.0):
    """Reset to the dead-seated perch on the chair, settle into contact, return qpos/qvel."""
    m, d = gen.m, gen.d
    mujoco.mj_resetData(m, d)
    d.qpos[0:3] = [0.15, 0.0, 0.56]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, a in zip([n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos):
        if nm in GS.SEATED_POSE:
            d.qpos[a] = GS.SEATED_POSE[nm]
    if dr > 0.0 and rng is not None:
        d.qpos[gen.act_jqpos] += rng.normal(0.0, dr, size=len(gen.act_jqpos))
        d.qpos[2] += rng.normal(0.0, dr * 0.3)
    full = np.zeros(m.nu)
    full[gen.pos_act] = d.qpos[gen.act_jqpos]
    d.ctrl[:] = full
    for _ in range(int(0.4 / gen.sim_dt)):
        mujoco.mj_step(m, d)
    return d.qpos.copy(), d.qvel.copy()


def make_obs(gen: GhostGenerator, d, t, total_time):
    """Deployable proprio + base + phase-clock observation."""
    qa = d.qpos[gen.act_jqpos]
    qda = d.qvel[gen.act_jdof]
    z = np.array([d.qpos[2]])
    quat = d.qpos[3:7]
    lin = np.zeros(6)
    mujoco.mj_objectVelocity(gen.m, d, mujoco.mjtObj.mjOBJ_BODY, 1, lin, 0)  # pelvis = body 1
    angv, linv = lin[0:3], lin[3:6]
    ph = t / total_time
    phase = np.array([np.sin(2 * np.pi * ph), np.cos(2 * np.pi * ph), ph])
    return np.concatenate([qa, qda, z, quat, linv, angv, phase]).astype(np.float32)


def step_env(gen: GhostGenerator, action):
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = np.clip(action, gen.ctrl_lo, gen.ctrl_hi)
    gen.d.ctrl[:] = full
    for _ in range(gen.n_sub):
        mujoco.mj_step(gen.m, gen.d)


# --------------------------------------------------------------------------------- the policy
def build_policy(obs_dim, act_dim, hidden=256):
    import torch.nn as nn
    return nn.Sequential(
        nn.Linear(obs_dim, hidden), nn.ELU(),
        nn.Linear(hidden, hidden), nn.ELU(),
        nn.Linear(hidden, act_dim), nn.Tanh(),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=8, help="DAgger iterations")
    ap.add_argument("--rollouts", type=int, default=3, help="policy rollouts per iter (label collection)")
    ap.add_argument("--rise-time", type=float, default=3.5, help="seconds of motion to distill (the rise)")
    ap.add_argument("--n-samples", type=int, default=48)
    ap.add_argument("--horizon", type=int, default=12)
    ap.add_argument("--noise", type=float, default=0.06, help="IN-LOOP MPPI noise (low = clean tracking labels; warm-started on the reference)")
    ap.add_argument("--ref-noise", type=float, default=0.18, help="REFERENCE-discovery MPPI noise (high = explores to FIND the unstable launch)")
    ap.add_argument("--temp", type=float, default=0.5, help="MPPI softmax temperature (lower = sharper)")
    ap.add_argument("--epochs", type=int, default=120, help="SGD epochs per iter")
    ap.add_argument("--dr", type=float, default=0.0, help="seated-init joint noise (rad)")
    ap.add_argument("--res-scale", type=float, default=0.8,
                    help="residual authority (rad) around the expert reference ctrl -- fine "
                         "resolution; a ZERO residual replays the reference (deterministic ~stand)")
    ap.add_argument("--name", type=str, default="dagger_g1_launch")
    ap.add_argument("--smoke", action="store_true", help="tiny config to validate the pipeline")
    args = ap.parse_args()
    if args.smoke:
        args.iters, args.rollouts, args.rise_time = 2, 1, 1.2
        args.n_samples, args.horizon, args.epochs = 24, 8, 40

    import torch
    import torch.nn as nn
    torch.manual_seed(0)

    gen = GhostGenerator(MJCF, sim_dt=0.004, control_dt=0.016)
    intent = build_intent()
    # in-loop expert: LOW noise, warm-started on the reference (clean tracking/correction labels)
    expert = ExpertMPC(gen, intent, n_samples=args.n_samples, horizon=args.horizon,
                       noise=args.noise, temperature=args.temp)
    # reference-discovery expert: HIGH noise (explores to find the unstable launch)
    ref_expert = ExpertMPC(gen, intent, n_samples=args.n_samples, horizon=args.horizon,
                           noise=args.ref_noise, temperature=0.5, seed=0)
    n_steps = int(round(args.rise_time / gen.control_dt))
    rng = np.random.default_rng(0)

    # warm the obs dim
    q0, v0 = seated_init(gen)
    obs_dim = make_obs(gen, gen.d, 0.0, intent.total_time).shape[0]
    act_dim = len(gen.pos_act)
    print(f"[dagger] obs_dim={obs_dim} act_dim={act_dim} n_steps={n_steps} "
          f"expert(N={args.n_samples},H={args.horizon})")

    policy = build_policy(obs_dim, act_dim)
    opt = torch.optim.Adam(policy.parameters(), lr=1e-3)
    scale = float(args.res_scale)

    # ---- REFERENCE ctrl: one expert rollout from the nominal seated start. Replaying these
    #      ctrls open-loop in the SAME MuJoCo is DETERMINISTIC -> reproduces the stand. The
    #      policy only learns a SMALL residual on top (fine resolution, drift recovery). ----
    seated_init(gen)
    ref_expert.reset(gen.d)
    REF_CTRL = np.zeros((n_steps, act_dim), np.float32)   # commanded targets (expert warm-start)
    REF_Q = np.zeros((n_steps, act_dim), np.float32)      # ACHIEVED joint angles (FEEDFORWARD anchor)
    ref_peak = gen.d.qpos[2]
    for k in range(n_steps):
        a = ref_expert.act(gen.d, k * gen.control_dt)
        REF_CTRL[k] = a
        step_env(gen, a)
        REF_Q[k] = gen.d.qpos[gen.act_jqpos]
        ref_peak = max(ref_peak, gen.d.qpos[2])
    print(f"[dagger] reference recorded; closed-loop ref peak_z={ref_peak:.3f} res_scale={scale}")

    def ref_horizon(k):
        """The reference ctrl over the next `horizon` steps (padded) -- the in-loop expert's
        warm-start, so its low-noise re-plan TRACKS the reference + corrects drift."""
        H = args.horizon
        idx = np.clip(np.arange(k, k + H), 0, n_steps - 1)
        return REF_CTRL[idx]

    obs_buf, res_buf = [], []   # res = normalized residual label in [-1,1]
    os.makedirs(OUTDIR, exist_ok=True)
    obs_mean = np.zeros(obs_dim, np.float32)
    obs_std = np.ones(obs_dim, np.float32)

    def policy_residual(o_np):
        o = (torch.tensor(o_np) - torch.tensor(obs_mean)) / torch.tensor(obs_std)
        with torch.no_grad():
            return policy(o.unsqueeze(0))[0].numpy()   # in (-1,1)

    def policy_action(o_np, step):
        # FEEDFORWARD anchor = the ACHIEVED joint trajectory (position-control tracks it stably);
        # the residual provides the leading correction the MPC applied + drift recovery.
        ref = REF_Q[min(step, n_steps - 1)]
        return np.clip(ref + scale * policy_residual(o_np), gen.ctrl_lo, gen.ctrl_hi)

    best_score, best_state, best_norm, best_it = -1e9, None, None, -1
    t_start = time.time()
    for it in range(args.iters):
        beta = max(0.0, 1.0 - it / max(1, args.iters - 1))  # expert mix: 1 -> 0
        peaks = []
        for r in range(args.rollouts):
            seated_init(gen, rng=rng, dr=args.dr)
            expert.reset(gen.d)
            peak = gen.d.qpos[2]
            for k in range(n_steps):
                t = k * gen.control_dt
                o = make_obs(gen, gen.d, t, intent.total_time)
                a_exp = expert.act(gen.d, t, warm=ref_horizon(k))
                res_label = np.clip((a_exp - REF_Q[k]) / scale, -1.0, 1.0)
                obs_buf.append(o)
                res_buf.append(res_label.astype(np.float32))
                if it == 0 or np.random.rand() < beta:
                    a_exec = a_exp
                else:
                    a_exec = policy_action(o, k)
                step_env(gen, a_exec)
                peak = max(peak, gen.d.qpos[2])
            peaks.append(peak)
        # ---- supervised train on the aggregated dataset (normalized residual labels) ----
        O = np.asarray(obs_buf, np.float32)
        R = np.asarray(res_buf, np.float32)
        obs_mean = O.mean(0)
        obs_std = O.std(0) + 1e-4
        On = torch.tensor((O - obs_mean) / obs_std)
        Rt = torch.tensor(R)
        n = On.shape[0]
        for ep in range(args.epochs):
            idx = torch.randperm(n)
            for b in range(0, n, 512):
                j = idx[b:b + 512]
                pred = policy(On[j])
                loss = nn.functional.mse_loss(pred, Rt[j])
                opt.zero_grad(); loss.backward(); opt.step()
        # ---- eval: PURE-policy rollout from the nominal seated start, then a short HOLD
        #      (phase frozen at the rise end) to check it doesn't immediately collapse ----
        seated_init(gen)
        ev_peak = gen.d.qpos[2]
        for k in range(n_steps + 45):
            t = min(k, n_steps - 1) * gen.control_dt   # freeze phase during the hold tail
            o = make_obs(gen, gen.d, t, intent.total_time)
            step_env(gen, policy_action(o, k))
            ev_peak = max(ev_peak, gen.d.qpos[2])
        ev_final = gen.d.qpos[2]
        ev_tilt = np.degrees(np.arccos(max(-1.0, min(1.0,
                  1 - 2 * (gen.d.qpos[4] ** 2 + gen.d.qpos[5] ** 2)))))
        stood = ev_final > 0.70 and ev_tilt < 20.0
        # stand score: reward a high, upright FINAL pose (stable stand), not just a peak
        score = ev_final - 0.003 * ev_tilt
        if score > best_score:
            best_score = score
            best_state = {k: v.detach().clone() for k, v in policy.state_dict().items()}
            best_norm = (obs_mean.copy(), obs_std.copy())
            best_it = it
        dt = time.time() - t_start
        print(f"[dagger] it={it} beta={beta:.2f} D={n} loss={loss.item():.4f} "
              f"label_peak={np.mean(peaks):.3f} EVAL peak={ev_peak:.3f} final={ev_final:.3f} "
              f"tilt={ev_tilt:.1f} {'STOOD' if stood else ''}  ({dt:.0f}s)")

    out = os.path.join(OUTDIR, args.name + ".pt")
    save_state = best_state if best_state is not None else policy.state_dict()
    save_mean, save_std = best_norm if best_norm is not None else (obs_mean, obs_std)
    print(f"[dagger] saving BEST checkpoint from it={best_it} (score={best_score:.3f})")
    torch.save({"state_dict": save_state, "obs_mean": save_mean, "obs_std": save_std,
                "ctrl_lo": gen.ctrl_lo, "ctrl_hi": gen.ctrl_hi, "obs_dim": obs_dim,
                "act_dim": act_dim, "act_names": [n for n in gen.act_name],
                "ref_ff": REF_Q, "res_scale": scale}, out)
    print(f"[dagger] saved {out}")


if __name__ == "__main__":
    main()
