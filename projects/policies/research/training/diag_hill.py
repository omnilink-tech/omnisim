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

"""Single-env diagnostic: replay a hill policy from x=0 and log the traverse
(x, base height vs ground, roll, pitch vs TARGET pitch, height error) up to the
fall -- to see HOW the hill-walk fails (nose-in? roll? sink?)."""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))
import projects.policies.research.training.gpu_mjwarp_b2_walk_trainer as T  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ghost", default=str(REPO / "projects/policies/research/shadowing/ghosts/b2_hill8_ghost.npz"))
    ap.add_argument("--policy", default=str(REPO / "projects/policies/research/training/runs/gpu_b2_hill8/policy.pt"))
    ap.add_argument("--mjcf", default=r"C:\tmp\b2_newton.xml")
    ap.add_argument("--vx", type=float, default=0.35)
    ap.add_argument("--steps", type=int, default=900)
    ap.add_argument("--bare", action="store_true")
    ap.add_argument("--start-x", type=float, default=0.0,
                    help="spawn mid-hill at this x (pitched to the ghost) to test "
                         "STEADY-SLOPE climbing vs the flat->ramp transition")
    ap.add_argument("--step-h", type=float, default=0.08)
    ap.add_argument("--res", type=float, default=0.15)
    ap.add_argument("--duty", type=float, default=0.6)
    ap.add_argument("--freq", type=float, default=1.3)
    ap.add_argument("--gait-module", default=None,
                    help="e.g. projects.policies.control.gait.b2_crawl_gait (rebinds the trainer's gait)")
    a = ap.parse_args()
    if a.gait_module:
        import importlib
        T.stg = importlib.import_module(a.gait_module)
        print(f"[diag] gait = {a.gait_module}")

    rcfg = dict(gait_params=dict(vx=a.vx, freq=a.freq, duty=a.duty, step_height=a.step_h,
                                 body_height=0.50, ramp_s=1.0),
                res_scale=a.res, max_ep=a.steps + 10, seed_gait=True,
                hill_flat_frac=1.0, rw_sched=-5, rw_slip=-0.5)
    env = T.BatchedSpotWalkEnv(1, a.mjcf, reward_cfg=rcfg, dr_cfg={}, hill_ghost=a.ghost)
    obs_in = env.obs_dim

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.pi = nn.Sequential(nn.Linear(obs_in, 256), nn.Tanh(),
                                 nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, T.NJ))
        def forward(s, o):
            return s.pi(o)

    ac = AC().to(env.tdev)
    if not a.bare:
        sd = torch.load(a.policy, map_location=env.tdev)
        ac.load_state_dict({k: v for k, v in sd.items() if k.startswith("pi.")}, strict=False)
    ac.eval()

    obs = env.reset()
    if a.start_x > 0:                                  # spawn mid-hill, pitched to the ghost
        import math
        xt = torch.tensor([a.start_x], device=env.tdev)
        tp = float(env.hill.target_pitch(xt)[0]); tz = float(env.hill.target_z(xt)[0])
        env.qpos_t[:, 0] = a.start_x
        env.qpos_t[:, 2] = tz
        env.qpos_t[:, 3] = math.cos(tp / 2); env.qpos_t[:, 4] = 0.0
        env.qpos_t[:, 5] = math.sin(tp / 2); env.qpos_t[:, 6] = 0.0
        env.qvel_t[:, 0:6] = 0.0
        with env.wp.ScopedDevice(env.device):
            env.mjw.forward(env.mw_m, env.mw_d)
        obs = env._build_obs_t()
        print(f"[spawn] mid-hill x={a.start_x} pitched tgt={tp:+.3f} z={tz:.3f}")
    print(f"{'t':>5} {'x':>6} {'bz':>6} {'grnd':>6} {'roll':>6} {'pitch':>7} {'tgt_p':>7} {'hz_err':>7}")
    for k in range(a.steps):
        with torch.no_grad():
            act = torch.zeros(1, T.NJ, device=env.tdev) if a.bare else torch.clamp(ac(obs), -1, 1)
        obs, rew, done, info = env.step(act)
        if k % 10 == 0 or done.item():
            x = float(env.qpos_t[0, 0]); bz = float(info["bz"][0])
            roll = float(info["roll"][0]); pitch = float(info["pitch"][0])
            tp = float(env.hill.target_pitch(env.qpos_t[:, 0])[0])
            grnd = float(env.hill.ground_h(env.qpos_t[:, 0])[0])
            print(f"{k*T.DT:5.1f} {x:6.2f} {bz:6.3f} {grnd:6.3f} {roll:+6.2f} "
                  f"{pitch:+7.3f} {tp:+7.3f} {bz-env.hill.target_z(env.qpos_t[:,0])[0].item():+7.3f}")
        if done.item():
            print(f"--- FELL at t={k*T.DT:.1f}s x={float(env.qpos_t[0,0]):.2f} "
                  f"(roll={roll:+.2f} pitch={pitch:+.3f} tgt={tp:+.3f} bz-grnd={bz-grnd:.3f}) ---")
            break
    else:
        print(f"--- SURVIVED {a.steps} steps, final x={float(env.qpos_t[0,0]):.2f} ---")


if __name__ == "__main__":
    main()
