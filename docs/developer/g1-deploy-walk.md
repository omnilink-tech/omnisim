# G1 Humanoid — Walking in the OmniSim (Newton) Deploy

> **⚠️ Superseded (2026-07-01).** The finite ~34 s walk bout documented below was
> later superseded by a durable forward G1 walk. Canonical status:
> [rl-current-state.md](rl-current-state.md).

**Status (2026-06-17): a FINITE walk bout — real progress, not yet durable.** The
Unitree G1 walks **+5.9 m in 33.8 s then falls** (`FALL@33.82s`), upright and stable
while up (base height ~0.72, roll/pitch within ±0.3), in the OmniSim Newton/mjwarp
deploy. For months it had collapsed at ~1 s, so this is a real milestone — but it is
**a ~34 s bout, NOT an indefinite walk**, and the 33.8 s leaned on an *incorrect*
deploy COM (the corrected-COM `USE_LINK_COM=1` retrain falls ~11.7 s). Canonical
status: [rl-current-state.md](rl-current-state.md).

> **Ghost-fidelity (2026-06-18): ≥80 % shape-match achieved OVER the walk window — NOT durable.**
> A *feasible* ghost lifts the numerical shape match from the human ghost's **~67 % physical wall**
> (a balancing biped must deviate from a kinematic reference to stay up) to **84 % FAIR all-13 /
> 88 % moving — but measured over a 3 s window.** An 18 s eval shows the policy **topples ~6–8 s**,
> and the OmniSim deploy falls sooner. So the *shape* goal is met over the walk bout; a **durable
> ≥80 % deploy walk is NOT achieved** — it's entangled with the unsolved G1 trainer↔deploy
> durability gap. Full honest journal: [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md);
> details in [the last section](#walking-like-the-ghost--feasible-ghost-2026-06-18).

Winning policy: `projects/policies/research/training/runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.onnx`.

> **Reproducibility note.** Every `runs/<name>/policy.onnx` cited below
> (`gpu_newton_g1_walk_ft_pdoff_clamp`, the warm-start base `gpu_g1_walk_aggdr_c1`,
> the feasible-ghost `gpu_newton_g1_walk_ACH2_pdoff`) is a **GPU-trained
> output that is NOT in the repo** — `projects/policies/research/training/runs/` is gitignored
> (`.gitignore` line 123), so these directories exist only on the training machine,
> not in a fresh clone. To get a deployable policy, **reproduce it with
> the `gpu_newton_g1_walk_trainer.py` command in the
> Train section** (the run name is the directory name), then deploy the resulting
> `runs/<run-name>/policy.onnx`. The recipe scripts, worlds, and the
> `g1_achieved_gait.npz` dataset are all in-tree; only the trained policy weights are not.

> **Update (2026-06-18): the train ↔ deploy physics is no longer hand-matched.** The
> "byte-match the physics" parity this recipe depends on is now **single-source** — both
> the trainer and the deploy derive their model from `g1_physics.json` + `g1_physics_spec`
> + the prim URDF (never re-declare a constant on either side) — and it is **verified** three
> ways (structural model diff = 0 real-physics gaps, an 8.5 mm GPU golden trajectory, and a
> live H100 run that byte-matches the deploy spec 11/11). Same physics modulo the opt-in
> `OMNISIM_NEWTON_USE_LINK_COM` flag (default off) + documented residual diffs. Details:
> [g1-single-source-of-truth.md](g1-single-source-of-truth.md).

---

## Why it failed for months, and the fixes (in order)

The G1 is a 13-DOF biped — an inverted pendulum at the stability edge, so *any*
trainer↔deploy mismatch topples it (a quadruped like Spot absorbs the same
mismatches and walked fine all along). The gap was closed one mismatch at a time:

| Fix | What was wrong | Deploy result |
|---|---|---|
| **Silent XPBD fallback** (`cbe5e6f0`) | A deploy world's `RectangleArena` floor `<Plane>` sits on a non-static body; newton's `SolverMuJoCo` rejected it ("Planes can only be attached to static bodies"), the mjwarp build failed, and the backend **silently fell back to XPBD** — a different solver the mjwarp-trained policy can't survive. | engine now mjwarp; 1.07→1.46s |
| model + friction parity | deploy prim URDF had 4 extra shoulder colliders; ground mu 2.0 vs trainer 1.0 | 1.46s |
| more training | 700 iters vs 300 | 3.12s |
| **Faithful joint-clamp parity** (`9b6df709`) | The deploy applies a post-step joint clamp (clamp qpos to URDF limits, qvel to ±velocity_limit, **zero velocity driving into a stop**; `WbNewtonBackend.cpp`) that the trainer never had. Disabling it in deploy made the walk *worse* → it's **load-bearing**. Train *with* it. | **33.82s / 5.9 m** ✅ |

The decisive lever was the **joint-clamp parity**, not more iterations
(clamp-400iters crushed no-clamp-700iters). It was found by a **24-agent parallel
diagnosis** (`Workflow g1-gap-diagnose`) that surfaced 15 verified trainer↔deploy
mismatches at once.

**Key fixes in the backend (`cbe5e6f0`):** `add_shape_plane` now defers *all*
ground planes; `finalize()` drops them whenever the solver will be MuJoCo (by
`WorldInfo.newtonSolver` **or** `OMNISIM_NEWTON_FORCE_MUJOCO`); and a silent
XPBD fallback now emits a **loud `WbLog::warning`**.

---

## The recipe

### Deploy (watch it walk)
World: `projects/policies/worlds/g1_walk_arms_deploy_mjwarp.wbt` (arena-LESS — relies on
the backend's default static ground plane; the flagship arena world cold-load
crashes more, and its arena plane was the original XPBD trigger).

```
policy   = runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.onnx
launch   = scripts/dev/headless_runner.py <world> --gui --realtime --cold --port <p> --duration 200
env      = G1_GAIT_MODEL=human G1_GAIT_STYLE=winter G1_GAIT_VX=0.4 G1_GAIT_FREQ=1.3
           G1_GAIT_RAMP_S=2.0 G1_OBS_STACK=4 G1_OBS_LOOKAHEAD=0.1,0.4
           G1_GAIT_A_LAT=0.05 G1_GAIT_A_ANKLE=0.0 G1_GAIT_A_ARM=0.25 G1_GAIT_HIP_SCALE=0.9
           G1_ACT_SCALE=0.3 G1_BAL_KP_P=0 G1_BAL_KD_P=0 G1_BAL_KP_R=0 G1_BAL_KD_R=0   (balance PD OFF)
           OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_NEWTON_FORCE_MUJOCO=1
           OMNISIM_NEWTON_MJWARP=1 OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_SEED_POSE=1
           OMNISIM_NEWTON_TARGET_KE=100 OMNISIM_NEWTON_TARGET_KD=5 OMNISIM_NEWTON_GROUND_MU=1.0
```
`G1_ACT_SCALE` MUST equal the policy's training `--res-scale`. `G1_BAL_*=0` because
the policy was trained PD-off (it learns all balance itself). `--cold` (skip the
warmup reload) is markedly more stable against the intermittent G1 cold-load crash;
expect a ~60–90 s warp JIT compile before it steps, then retry on the rare crash.

### Train (reproduce the walker)
Local (CUDA GPU): run `gpu_newton_g1_walk_trainer.py` directly — a PPO fine-tune
stepped through the EXACT deploy solver (`SolverMuJoCo` mjwarp on the
primitive-collision 23-DOF G1), warm-started from `gpu_g1_walk_aggdr_c1`.

```
python projects/policies/research/training/gpu_newton_g1_walk_trainer.py \
  --save projects/policies/research/training/runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.pt \
  --envs 2048 --iters 400 --obs-stack 4 --obs-lookahead 0.1,0.4 \
          --hidden-dims 512,512,512 --hold-arms --gait-model human --gait-style winter \
          --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 \
          --gait-ramp-s 2.0 --gait-hip-scale 0.9 --rest-start-frac 0.4 --res-scale 0.3 \
          --train-joint-clamp --dr-push-vmax 1.2 --dr-push-prob 0.03 \
          --dr-action-latency-max 3 --dr-act-gain 0.2 --dr-init-tilt-band 0.06 --dr-init-vel-band 0.15
```
`"pdoff"` in the name sets `G1_TRAIN_BAL_*=0` (balance PD off — the analytic ankle PD
destabilises the deploy; the policy learns all balance instead). `--train-joint-clamp`
is the decisive parity flag. `--init <path>` warm-starts from any policy.pt in the
uploaded repo (continue from the original base `aggdr_c1`, NOT a finished walker —
warm-from-walker continuations destabilise).

---

## The method, generalised

This is the Ghost Method (residual policy on a kinematic gait reference) deployed
faithfully: **train in the deploy solver, and make the trainer's per-tick physics
byte-match the deploy's** — same engine (mjwarp `SolverMuJoCo`), same model
(primitive collision), same gains (ke=100/kd=5), same friction (mu=1.0), and the
same post-step joint clamp. The residual `target = gait_baseline + res_scale·action`
(clamped to joint limits) supplies the balance the open-loop gait reference lacks.

---

## Making it walk *like the ghost* — cracked via curriculum + partial-body imitation

The walker balances but its gait is functional-scrappy, not graceful — because the
**imitation reward was never on**. The Newton trainer's reward code reads `rw_track`
from `reward_cfg` but `main()` never set it → it defaulted to 0.0, so every fine-tune
optimised survival + forward progress only, never tracking the ghost. Wiring fixed in
`17129b77` (`--rw-track` / `--track-sigma` / `--rw-track-vel`).

Constant imitation from step 0 **regressed the walk at every weight tried** (rw-track 0.5→1.71s,
1.0→1.66s, 1.5→1.33s, vs 33.8s off): the open-loop ghost gait does not itself stay upright, so
paying the policy to track it from the start pulls it toward the falling motion — the residual's
*deviation* from the ghost is what keeps the biped up.

**Cracked (2026-06-18)** with two changes, trained + deployed on the verified single-source
parity physics:
1. **Imitation curriculum** (`--rw-track-warmup-frac 0.5`): ramp `rw_track` 0→target over the
   first half of training, so balance is learned first and ghost-tracking is added gradually.
2. **Partial-body imitation** (`--track-ankle-w 0.0`): free the balancing ankles from the
   open-loop-ghost pull; track only the hips/knees (the visible swing).

Result (`gpu_newton_g1_walk_ghost_pdoff`, 400 iters, H100): trained reward **2.56** (vs 2.13 with
imitation off) with **fall/step staying low (~0.003)** — no walk regression, and the higher reward
*is* the earned imitation bonus (it tracks the ghost in-sim). Deployed in OmniSim with
`OMNISIM_NEWTON_USE_LINK_COM=1`: **+9.8 m, upright ~31 s** before its eventual fall — farther than
the original no-ghost milestone (5.9 m / 33.8 s) and far better than the old policy on the
corrected deploy (3.9 m / 11.7 s). Recipe: the standard walk recipe (above) + `--rw-track 1.0
--rw-track-warmup-frac 0.5 --track-sigma 0.1 --track-ankle-w 0.0`.

Still open *(against the human ghost)*: it walks ~31 s then falls (like the original), and the
ghost-fidelity plateaued ~67 %. See the next section for the fix.

---

## Walking *like the ghost* — feasible ghost (2026-06-18)

> **Status: ≥80 % shape-match achieved OVER the ~7 s walk window; NOT a durable walk.** Read
> [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md) for the full honest journal incl.
> the durability correction below.

The ghost-fidelity was measured numerically (no eye-witness) with a per-joint metric
(amplitude `max(0,1-RMSE/PTP_fair)` + Pearson shape `r`; `--eval-ghost-similarity`).
Against the **ambitious human/winter ghost** the climb stalled at a *physical wall*:

| run | FAIR all-13 | moving-joints | upright |
|---|---|---|---|
| best human-ghost (sagittal-lever) | 63–67 % | 67–69 % | 54–65 / 128 |

**Why it stalls:** `similarity = 1 − RMSE/PTP`, and a balancing biped *must* deviate ~0.17 rad
from any **kinematic** reference to stay up (that deviation is the residual that keeps it
upright). So RMSE is floored → similarity caps ~70 % and ~half the robots fall. Stiffer gains,
reward re-tuning, and a left/right **mirror-symmetry loss** were all tried and **did not break
it** (the mirror loss collapsed the gait; `--mirror-loss`, kept but off by default). Shrinking
the ghost's amplitude makes it *worse* (same RMSE ÷ smaller PTP).

**The fix — make the ghost FEASIBLE (the robot's own gait, cleaned).** Use the `(B) achieved`
shadow ([g1-improved-shadow.md](g1-improved-shadow.md)) but extract it **in the Newton deploy
solver** (the mjwarp extraction isn't Newton-feasible — the whole train↔deploy gap):

```
# 1. extract the feasible ghost FROM a stable Newton walker, IN Newton:
python projects/policies/research/training/gpu_newton_g1_walk_trainer.py \
  --init-from runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.pt \
  --build-achieved projects/policies/control/gait/datasets/g1_achieved_gait.npz \
  --envs 512 --no-dr --train-joint-clamp --gait-model human --gait-style winter \
  --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 \
  --gait-hip-scale 0.9 --res-scale 0.3 --hold-arms --obs-stack 4 --obs-lookahead 0.1,0.4
#   (G1_NEWTON_KE=100 G1_NEWTON_KD=5 G1_TRAIN_BAL_*=0)
# -> phase-binned, L/R-symmetrised, smoothed (W_N,13)+stand -> datasets/g1_achieved_gait.npz

# 2. train a fresh policy to TRACK the feasible ghost (strong tracking is safe now):
python projects/policies/research/training/gpu_newton_g1_walk_trainer.py \
  --save projects/policies/research/training/runs/gpu_newton_g1_walk_ACH2_pdoff/policy.pt \
  ... --gait-style achieved --rw-track 8.0 --track-sigma 0.03 --track-ankle-w 1.0 \
  --track-waist-w 1.0 --rw-track-vel 0.5 --rw-track-warmup-frac 0.2 --iters 4000 \
  --res-scale 0.3 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --hold-arms \
  --obs-stack 4 --obs-lookahead 0.1,0.4 --train-joint-clamp --eval-every 1000
```

**Result (`gpu_newton_g1_walk_ACH2_pdoff`), measured vs the ghost:**

| metric (3 s window) | value |
|---|---|
| robots upright (3 s) | 128 / 128 |
| FAIR all-13 | **84.2 %** |
| hips + knees + waist (9) | **86.1 %** |
| sagittal swing (hp/kn/ap) | **87.0 %** |
| **moving joints** | **87.9 %** |

Per-joint fidelity 0.72–0.90; shapes (Pearson r) knee 0.89–0.94, hip_pitch 0.88, ankle_pitch
0.84. The tracking error collapsed (RMSE ~0.04–0.10 rad vs ~0.17 against the human ghost) — the
feasible-reference idea is correct, and *within the window* stability improved (128/128 over 3 s
vs ~half on the human ghost).

> **⚠️ Durability correction (do not skip).** The table above is over a **3 s** window. An **18 s**
> eval (`--eval-steps 1500 --eval-window-s 18`) returns *"no env stayed upright for ≥60% of the
> window — max upright-window steps 375"* ≈ **6 s**. So **ACH2 topples ~6–8 s** — the 128/128 was a
> short-window artifact. **Root cause:** the achieved ghost was extracted from the champion's gait
> *in the trainer env, where the champion survives only ~7.3 s* (it walks 33.8 s in the
> `omnisim-bin` deploy) — the byte-matched model's ~8.5 mm/10-tick drift **compounds** on the
> unstable biped, so trainer durability ≠ deploy durability. The ≥80 % is a real *shape* match
> over the walk bout; a **durable** ≥80 % walk needs the trainer↔deploy durability gap closed
> first (extract the feasible ghost where the gait is actually durable, add a durability reward).

**Deploy it** (single source of truth — train + deploy read the same `g1_achieved_gait.npz`).
**⚠️ As of 2026-06-18 this deploy is NOT durable:** it intermittently hits the G1 **cold-load
crash** (exit 1, retry) and, when it loads, the robot **falls quickly** (the ~7 s durability +
the trainer→deploy gap + the wide-splay achieved stand spawning in the arena world). Documented
as a known gap, not a working demo:

```
G1_GAIT_STYLE=achieved python scripts/dev/g1_deploy_launch.py \
  --world projects/policies/worlds/g1_walk_arms_deploy_mjwarp_floor.wbt \
  --policy runs/gpu_newton_g1_walk_ACH2_pdoff/policy.onnx --duration 200
```

**Honest caveat:** the feasible ghost was extracted from the *stable* champion, whose gait is
**gentle** (hip_pitch ±7°), so most joints have PTP < the 0.35-rad FAIR floor and are scored
against it — i.e. this is a faithful *"tracks its feasible reference to ~0.05 rad, stably,"* a
calm walk rather than an athletic stride. For a bigger gait at ≥80 %, rebuild the feasible ghost
from a larger-swing source (the `--track-sagittal-w` policy) at some stability cost. New levers
added for this: `--build-achieved`, `--gait-style achieved`, `--track-sagittal-w`, `--eval-every`
(live in-training ghost-similarity %).
