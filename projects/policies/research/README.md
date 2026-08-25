# RL research archive — non-working / superseded robot artifacts

Everything in this folder was **moved out of the live RL tree on 2026-06-26**
because it does **not** produce a working robot in the real OmniSim Newton deploy
(verified by live headless runs — see
[docs/developer/rl-current-state.md](../../../docs/developer/rl-current-state.md),
"Empirical re-verification"). Archiving here is a reversible `git mv`, so the
experiments and their history are preserved.

**Exception — Atlas was removed outright (2026-07-17).** The OmniLink Atlas
robot, its specs, controllers, worlds and checkpoints were deleted from the tree,
not archived: the stand was a confirmed negative result (PPO ≈ zero-action
baseline, never deployed to Newton) and the upstream chain of title for the v5
geometry was flagged unclear / do-not-ship by the licensing audit. The negative
result is retained as documentation in
[docs/developer/atlas-stand-rl-journey.md](../../../docs/developer/atlas-stand-rl-journey.md);
recover the code itself from git history before commit `HEAD` of this change.

## What's here (and why it was archived)

| Group | Artifacts | Why |
|---|---|---|
| **G1 from-scratch walk** | `controllers/g1_walk_deploy`, `g1_walk_canon_deploy`, `g1_model_walk`, `g1_ghost_walk` + worlds + runners | OmniSim-trained G1 walk face-plants ~1.3 s; ghost-built v6 falls ~1.44 s. Only the re-hosted Unitree `motion.pt` / BC clone walk durably (those stay live). |
| **G1 deterministic stand** | `controllers/g1_stand_deploy`, `humanoid_static_walk` + worlds + runners | Tips forward, FALL@~1.38 s (static-walk FALL@1.42 s). |
| **G1 demos** | standwave / braceguard / sit / sitstand / pose-sweep / torque-balance / shot controllers + worlds + runners | Experimental G1 motions; none deploy durably. |
| **Legacy OmniQuad RL stack** | `controllers/omniquad_rl_*`, `omniquad_residual_*`, `omniquad_model_walk`, `omniquad_raibert_walk`, `omniquad_gpu_residual_deploy`, `omniquad_recovery_agent` + worlds + `policies/omniquad_{mjx,newton,residual,straight,v12,...}*` | Superseded by the working `gpu_omniquad_walk_main` / `gpu_omniquad_walk_vc_main` (which stay live). |

## What is still LIVE (NOT here)

Durable, verified-working deploys remain in `projects/policies/controllers/`,
`projects/policies/worlds/`, `scripts/dev/run_*.ps1`, and `projects/policies/research/inference/policies/gpu_*`:
- **G1 walk** — `g1_unitree_deploy` (Unitree `motion.pt` + `g1_bc_walk.pt`), `run_g1_unitree_walk.ps1`
- **H1 walk** — `h1_unitree_deploy` (`motion.pt` + `h1_bc_walk.pt`), `run_h1_unitree_walk.ps1`
- **OmniQuad / Go2 / B2 walk** — `*_walk_deploy` + `gpu_*_walk_main`, `run_{omniquad,go2,b2}_walk_deploy.ps1`
- **H1 / Valkyrie stand** — shared `humanoid_stand_deploy`, `run_humanoid_stand_deploy.ps1`
- Shared infra stayed in place: `backends/`, `control/`, `gait/`, the trainers, and `training/runs/`.

## Reviving an archived experiment

The moved run scripts compute their repo root relative to `scripts/dev/`, so they
will not run as-is from `research/runners/` — fix the `$root`/world paths (or move the
pieces back) before use. These are kept for reference and reproducibility, not as
turnkey demos.
