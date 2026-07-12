# Ghost-built G1 walk — Shadowing, no Unitree weights

A G1 walking policy trained **entirely from a generated, feasible ghost**
([`projects/policies/control/gait/g1_squat_ghost.py`](../../../control/gait/g1_squat_ghost.py)) via a
full-authority RL policy with the ghost as a dense imitation reward — the Shadowing
method. **No behavior-cloning, no Unitree weights.**

Trainer: [`gpu_humanoid_walk_trainer.py`](../../training/gpu_humanoid_walk_trainer.py)
`--ghost <npz> --ghost-w` (default off → identical to the canonical pure-RL trainer).
Run the demo: `powershell -File scripts/dev/run_g1_ghost_walk_gui.ps1`.

## Honest status (banked 2026-06-26)

| | result |
|---|---|
| **Trainer durability** | ✅ **78–87 % no-fall over ~10 s, up to 12 m** — past the documented ~2 m from-scratch wall. The key was removing the analytic balance-assist crutch (`--bal-kp 0`) + DR + the ghost. |
| **Deploy (real Newton engine)** | ⚠️ **~1.0–1.4 s upright + stepping, then falls.** `v6` is the best transfer (1.44 s, genuinely balancing at 1 s). |

**Why deploy is capped — and why it's NOT a tuning bug:** the trainer↔deploy gap is
**world-level**. The deploy builds a *mesh-collision* model (`g1_legs_omnisim.urdf`)
through `SolverMuJoCo`; the trainer used a *box-foot* approximation (`g1_legs_kp100.mjcf`)
in raw mujoco_warp at a different operating point. The **canonical pure-RL policy hits the
SAME ~1.3 s ceiling** — so it sinks every from-scratch policy, not just the ghost. The BC
clone reaches 89 m only because it *copies* Unitree's already-deploy-proven policy.

**To close it (open, multi-day):** train in the deploy solver itself — the two-stage
`gpu_mjwarp_g1_walk_trainer` → `gpu_newton_g1_walk_trainer` (SolverMuJoCo) pipeline, on the
deploy model/gains/foot, carrying the crutch-free + DR + ghost recipe. The project's
documented attempt at this reached +5.9 m after a ~1B-step run (still not durable). Full
journal: memory `project_ghost_built_walk_pipeline`.

## Files
- `g1_ghost_walk_v6.onnx[.data]` / `.pt` — the banked policy (best deploy transfer).
- v1–v7 (`_scratch/`) were the exploration sweep (ghost-w, forward-gating, DR, crutch,
  zero-COM, qd-fd); v6 is the keeper.
