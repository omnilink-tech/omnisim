# MJX training — handoff for a real GPU box

This is a snapshot of the OmniSim RL training plumbing as of 2026-05-17,
written so anyone with a real GPU (A100/H100/etc.) can pick up where the
laptop run left off and produce a walking Spot policy.

The laptop GPU (RTX 3060 Mobile, 6 GB) ran out of VRAM at N=64 envs and
delivered ~476 physics env-steps/s — not enough to train Spot inside a
reasonable wall-clock. Everything else (code, env, ONNX export, deploy
controller) is in place. Move the run to a beefier box and it should
work end-to-end.

## Resuming from this snapshot

This document was authored at commit `ae84be57`; before running the steps below, confirm the current tree still matches the assumptions in §1–§2 (the trainer file paths and CLI in particular).

On the target machine:
```bash
git clone <this-repo>
cd omnisim
git checkout main
```

Then jump to §1 (deps) and run the steps in order. Total time from
clone to a deployable `policy.onnx` on an A100-class GPU is roughly:
~5 min deps + ~3 min bench + ~3 min smoke train + ~20 min full training
+ ~2 min deploy in OmniSim.

Two things that were **not** verified on the laptop and are the first
things to check on the new box:
1. The `lax.scan` rollout refactor (commit ae84be57) compiles, but was
   stopped before the first rollout finished. §1 → §2 will validate
   it in ~5 min total.
2. The MJX → ONNX export path has never been run against a real
   trained checkpoint. §2 ends by writing one; just verify the file
   exists and is non-trivial size.

If either fails, the trainer (`projects/policies/research/backends/mjx_gpu.py`) is
~500 LOC and the failure point will be obvious from the stack trace.

## What's in the repo

```
projects/policies/
  backends/
    base.py                 # RobotSpec + TrainingBackend protocol
    sb3_cpu.py              # SB3 CPU backend (existing)
    mjx_gpu.py              # MJX JAX/GPU backend (this work)
    isaac_remote.py         # NVIDIA Isaac Lab adapter (write-only, untested)
    robot_registry.py       # --robot <name> lookup
    spot_robot_spec.py      # Spot's canonical spec
    reward_recipes.py       # legged_locomotion / manipulation / balance
    _urdf_to_mjcf.py        # URDF -> MuJoCo XML loader (handles package://)
  training/
    train_robot.py          # generic --robot --backend CLI (canonical entry)
    train_spot.py           # thin shim -> train_robot --robot spot
  controllers/rl_deploy/
    rl_deploy.py            # robot-agnostic deploy controller (reads sidecar)
  worlds/rl_deploy.wbt      # generic deploy world (Spot URDF as default)
  inference/policies/
    spot_ppo_main/          # existing CPU-trained policy + sidecar (stand-stable)
  tools/
    wsl_train.sh            # WSL2 + GPU training launcher

docs/developer/
  rl-accelerated-training.md   # how the platform works
  rl-training-handoff.md       # this file
```

## Status

| Piece | State |
|---|---|
| MJX env (URDF -> JAX-vmapped physics) | ✅ Verified — Spot loads as nq=19 nv=18 nu=12, single-env probe gives clean rewards +1.27 → +1.57 |
| Pluggable RobotSpec + reward recipes | ✅ Documented in `rl-accelerated-training.md` §"Adding a new robot" |
| Custom JAX PPO + lax.scan rollout | ✅ Verified end-to-end — 5M-step run on RTX 5070 reached ema_return ~96 at ~9k fps in ~9 min (commit `28d614ee`) |
| Defensive NaN/Inf masking + grad-finite check | ✅ Exercised in the 5M-step run; no NaN events (commit `28d614ee`) |
| ONNX export from MJX checkpoint | ✅ Verified — export produces a walking policy; sim-to-OmniSim gap noted (commit `28d614ee`) |
| Sidecar `robot_spec.json` | ✅ Written by all backends on export |
| `rl_deploy` controller + world | ✅ Working (`spot_ppo_main` ONNX deploys + stands stable) |
| WSL2 + CUDA 13.2 + JAX 0.6.2 + cuDNN | ✅ Working on this laptop; replicable on any Linux box |

## Why the laptop wasn't enough

| Symptom | Cause |
|---|---|
| MJX OOMs at N>64 | 6 GB VRAM cap; each Spot env with foot contacts uses ~50 MB |
| Bench plateaus at ~476 env-steps/s (N=64) | Memory-bound, not compute-bound — small batch can't saturate the GPU |
| 5M-step training would take ~3–13 hours | Even before any policy-side overhead |
| Earlier runs showed `ema_return=+nan` after a few hundred steps | Combination of bad batches + no defensive masking; see "Defensive fixes" below |

A box with **≥24 GB VRAM** (A100 40 GB, H100, 3090, 4090) should run
1024–4096 envs and hit ~30k–100k env-steps/s, finishing 10M steps in
15–30 minutes.

## Running on a real GPU box

### 0. Clone + deps

```bash
git clone <this-repo> omnisim
cd omnisim
pip install --upgrade "jax[cuda12]==0.6.2" \
            mujoco mujoco-mjx \
            "flax>=0.10" "optax>=0.2.5" \
            onnx onnxruntime torch numpy
```

Verify GPU:
```bash
python -c "import jax; print(jax.devices())"
# Expected: [CudaDevice(id=0)]
```

If you see `[CpuDevice(id=0)]`, JAX didn't pick up CUDA. Set
`LD_LIBRARY_PATH` to include the CUDA runtime + cuDNN you installed,
e.g.:
```bash
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$(python -c "import nvidia.cudnn; import os; print(os.path.dirname(nvidia.cudnn.__file__) + '/lib')")
```

### 1. Bench raw physics first (~30s)

```bash
python projects/policies/research/tools/mjx_gpu_bench.py
```

Picks an `--envs` ceiling — find the largest N before OOM, then use
half of that for training (PPO update buffers need headroom).

### 2. Smoke train (~2 minutes once JIT compiles)

```bash
ENVS=256 STEPS=50000 bash projects/policies/research/tools/smoke_mjx_train.sh
```

Expected log lines:
```
[mjx] devices: [CudaDevice(id=0)]
[mjx] mujoco model: nq=19 nv=18 nu=12
[mjx] step   ...   ema_return=+0.95  upd=...
```

`ema_return` should be a finite number that **trends up** over the run.
If you see `+nan`, the defensive fixes have a regression — check
`mjx_gpu.py` rollout/PPO update. (The masking is conservative; NaN
should now be impossible unless a grad blows up in a way the
finite-check misses.)

### 3. Full training

Recommended config for an A100/H100 class GPU:
```bash
ENVS=4096 \
STEPS=20000000 \
N_STEPS=24 \
BATCH=24576 \
RUN_NAME=spot_mjx_v1 \
SPOT_R_LIN_VEL_WT=2.0 \
SPOT_R_ALIVE_BONUS=0.02 \
bash projects/policies/research/tools/wsl_train.sh
```

`wsl_train.sh` is named for its origin but the script is plain bash —
runs fine on any Linux GPU box. Or call `train_robot.py` directly if
you want to skip the shell wrapper.

### 4. Outputs

```
projects/policies/research/inference/policies/<RUN_NAME>/
  policy.onnx          # the actor MLP, ONNX schema (obs -> action)
  robot_spec.json      # sidecar for the deploy controller
  checkpoints/         # intermediate flax msgpack snapshots
```

Copy these two files back to a Windows OmniSim install for deployment.

### 5. Deploy back in OmniSim (Windows or Linux)

```bash
# point the deploy world at the new ONNX
export OMNISIM_POLICY_ONNX=projects/policies/research/inference/policies/spot_mjx_v1/policy.onnx

# launch the deploy world
omnisim-bin projects/policies/research/worlds/rl_deploy.wbt
```

The `rl_deploy` controller reads `robot_spec.json` next to the ONNX to
know obs layout, joint order, nominal pose, action scale, motor PIDs.
**No code edits required** to deploy a new policy or a new robot.

## Defensive fixes in `mjx_gpu.py` (this work, untested at scale)

The trainer was producing `ema_return=+nan` after a few hundred steps
on the laptop. Added (commits to follow this doc):

1. **`lax.scan` rollout + GAE** — old code was a Python `for t in
   range(n_steps)` that dispatched one device kernel per step (24 round
   trips per update). Now the whole rollout fuses into one XLA op.
   Should also be much faster (was ~106 env-steps/s on laptop with the
   Python loop — most of that was overhead, not physics).
2. **Reward sanitisation** — `jnp.nan_to_num` after each `reward_fn` call
   so a single bad-contact NaN can't poison the batch return.
3. **`qd_acc` cap** — clipped to ±500 rad/s² before going into the
   joint-acceleration reward penalty (square of qd_acc would otherwise
   overflow to inf if a contact briefly explodes).
4. **Auto-reset on bad physics** — `done | jnp.any(jnp.isnan(d.qpos))`.
5. **PPO loss safety**:
   - `log_std` clipped to `[-5, 2]` (std ∈ [6.7e-3, 7.4])
   - `logp - old_logp` clipped to `[-20, 20]` before `exp`
   - advantage clipped to `[-10, 10]` after normalisation
   - returns clipped to `[-100, 100]`
6. **Grad-finite check** — if `jax.tree_util.tree_leaves(grads)` has any
   non-finite element, the params for that update are reverted to the
   previous step.

These are conservative — most are no-ops on a healthy run, kick in only
when something would otherwise blow up. They should **not** mask a real
bug (the run would still show flat `ema_return` if the policy isn't
learning); they just keep one bad batch from poisoning the whole run.

## What I'd verify first on the new box

In this order — if any one fails, stop and debug before continuing:

1. `python -c "import jax; print(jax.devices())"` → `CudaDevice`.
2. Run the 50k-step smoke test from §1. Check:
   - JIT compile completes (~30–90s for the first update).
   - `[mjx] step` lines start appearing within ~2 minutes.
   - `ema_return` is finite (not `+nan` or `-inf`).
   - `fps=` line shows ≥5000 env-steps/s on a real GPU.
3. Look at `projects/policies/research/inference/policies/spot_smoke/policy.onnx` — file
   should exist and be ~50–100 KB.
4. Deploy `spot_smoke` in OmniSim (`OMNISIM_POLICY_ONNX=...
   omnisim-bin rl_deploy.wbt`). Spot should at least not blow up
   immediately — a 50k-step policy won't walk, but it should be valid
   ONNX in the right schema.

After those pass, run the real 10M–20M-step training (§2).

## Known gotchas

- **JAX cache miss after every package upgrade.** First training launch
  after installing JAX/MJX will spend ~60–90s on JIT compile before the
  first `[mjx] step` log line. This is normal.
- **Spot URDF expects ~50 GB+ JIT memory at very large batch sizes.** If
  you see OOM on first compile at N>4096, lower N or set
  `XLA_PYTHON_CLIENT_MEM_FRACTION=0.9`.
- **The `isaac_remote.py` backend is write-only** — it was scaffolded
  but never verified against a real Isaac Lab install. Treat as a
  starting point, not a working backend.
- **The sb3 backend is Spot-specific.** The MJX backend is robot-agnostic
  via `RobotSpec`. For a non-Spot robot, use `--backend mjx`.

## Contact / where to look if it breaks

- Architecture overview: `docs/developer/rl-accelerated-training.md`
- The trainer itself: `projects/policies/research/backends/mjx_gpu.py` — well-commented,
  ~500 LOC, all the moving pieces in one file.
- The reward formula: `projects/policies/research/backends/reward_recipes.py`
  (`_legged_locomotion`).
- The deploy controller: `projects/policies/controllers/rl_deploy/rl_deploy.py`
  — note the inline `_RobotSpecLite` class; do **not** import the full
  `base.py` from this controller (the `_autoregister()` at module load
  imports jax/torch, which deadlocks the OmniSim `--batch` subprocess).
