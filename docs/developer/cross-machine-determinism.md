# Cross-machine determinism — what is guaranteed, what is not, and how to check

**The question this answers:** *"the robot behaves a bit differently on my other
PC — same outcome, slightly different gait. Bug, or am I imagining it?"*
(Asked live 2026-07-16.) You are not imagining it, and it is usually not a bug.
This doc defines the guarantee tiers, names the variance sources found in this
repo, and gives the one-command checks that separate BUG from expected drift.

## The three guarantee tiers

| Tier | Setup | Guarantee | Gate |
|---|---|---|---|
| **A** | same machine, same binary, same seed | load-settle is bit-identical (cold-first-load writeup); a live GPU-physics trajectory carries the solver's intrinsic run-to-run band — **measured 2026-07-17: max \|dq\| = 8.5e-6 over 400 ticks** (two decent-walker runs, warp atomics) | `golden_compare.py` → IDENTICAL / SOLVER-BAND |
| **B** | same GPU model + driver + stack, different machine | in-band over a short horizon | `golden_compare.py` → SOLVER-BAND |
| **C** | different GPU / OS / CPU | **statistical equivalence only**: same outcomes, metrics inside tolerance bands. Micro-gait WILL differ — a balancing biped is chaotic and amplifies ulp-level float differences into visible sway/foot-placement variation. | `machine_conformance.py` → EQUIVALENT |

Bitwise identity across different GPUs is **not achievable** (reduction order,
atomics, kernel scheduling differ per GPU generation) and no amount of pinning
buys it. Tier C is the honest cross-device contract: hold both machines to the
same *outcomes*, not the same bits.

> **⚠️ Tier A is weaker than its number suggests — read two corrections into it.**
> (1) **Same-machine bitwise is not a GPU property, it is a SOLVER property.** Newton with
> `newtonSolver "mujoco"` (CPU `mj_step`) *does* reproduce bitwise across cold launches,
> verified at 336 contacts with ten live controllers. ⚠ **2026-08-08: ODE used to be the
> second half of this sentence and it is no longer verifiable** — `bdc02139` deleted the
> backend. The ODE bitwise result was real; it is now historical, and the CPU `mj_step`
> row has lost the independent serial backend that used to corroborate it. The GPU
> `newtonSolver "mujoco_warp"` path does **not**: **0 bitwise of 24** same-config cold pairs
> across six scenes. So "the solver's intrinsic run-to-run band" above is specifically the
> **GPU** solver's band, and it exists because mujoco_warp claims contact slots with
> `wp.atomic_add` (buffer order = thread-arrival order).
> (2) **The band grows with simulated time, so a 400-tick measurement understates it by
> orders of magnitude.** The `max |dq| = 8.5e-6 over 400 ticks` figure is real for that
> horizon and that scene; on a contact-rich scene the same measurement reaches ~5e-5 m at
> 120 steps and **9.152 m at 1000 steps**. Do not quote 8.5e-6 as "the" GPU band, and do not
> conclude from a short run that a long run will stay close. Scheduling is directly
> implicated: saturating the GPU with an unrelated CUDA process widened the spread ~3× with
> no range overlap.
>
> Full per-configuration scope: [../benchmarks/determinism-scope.md](../benchmarks/determinism-scope.md).

## Where cross-machine variance actually comes from (found live, 2026-07-16)

1. **The torch that runs deploy inference is NOT bundled.** On Windows the
   engine's `_pth` resolves the physics stack from the vendored
   `newton-runtime/site-packages` but **falls through to the system
   site-packages for torch/onnxruntime** (deliberately — they are not in
   `DEPLOY_STACK`). Two PCs with different torch builds compute slightly
   different LSTM actions every tick. `env_fingerprint` now tags such
   packages `[sys]` — **match the `[sys]` versions across machines first.**
2. **Stale dist-info metadata lied about versions.** `pip install --target`
   does not remove the previous version's dist-info on upgrade; the shipped
   bundle carried TWO dist-infos each for newton, mujoco, mujoco_warp and
   warp_lang, making `importlib.metadata.version()` (and everything built on
   it) ambiguous. Fixed; guarded in the bundler; fingerprint warns if it
   recurs. Repair with
   `python scripts/packaging/audit_dist_info.py <site-packages> --repair`.
3. **A stale engine binary.** The tree's commit is not provenance — this repo
   shipped runs where `omnisim-bin.exe` trailed the source by 8 days. The
   fingerprint now records the **binary's sha256**; diff it across machines.
4. **Multi-threaded CPU inference.** Multi-threaded reductions sum in
   nondeterministic order. Deploy inference is now pinned single-threaded +
   deterministic-kernels + TF32-off (`projects/policies/common/numerics.py`,
   logged as `numerics: deploy inference pinned (...)` in every deploy RL
   log; ONNX deploy controllers pin `intra_op_num_threads=1`). Escape hatch
   for A/B triage: `OMNISIM_LOOSE_NUMERICS=1`.
5. **Seeded IC noise.** Launchers perturb the initial pose (`DEPLOY_IC_SEED`,
   `DEPLOY_IC_NOISE` ≈ 0.02 rad) so runs are seed-reproducible but
   seed-different. Same seed on both machines or you are not comparing runs.
6. **Chaos.** After all of the above, two different GPUs still diverge —
   agreeing for the first N ticks and drifting apart after. That is physics,
   not a defect: see
   [closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md).

## The toolchain (one command each)

**Is the stack the same?** — every deploy run logs an `[envfp]` block with the
stack line (`newton=… torch=…[sys]`) and `binary sha256=…`. Diff the two
machines' blocks. `[sys]`-tagged mismatches are the first thing to fix.

**Is the bundle metadata honest?**
```bash
python scripts/packaging/audit_dist_info.py \
    msys64/mingw64/bin/newton-runtime/site-packages --repair
```
(The bundler now runs this itself and `--verify` fails on duplicates.)

**Short-horizon bitwise gate (Tier A/B, and the BUG detector):**
```bash
OMNISIM_GOLDEN_DUMP=/path/a.npz  <launch the deploy headless>   # machine A
OMNISIM_GOLDEN_DUMP=/path/b.npz  <same launch, same seed>        # machine B
python projects/policies/training/golden_compare.py a.npz b.npz
```
Records the first 400 ticks of (qpos, qvel). Verdicts: `IDENTICAL` / `MATCH`
(fine), `CHAOS` (agrees early, drifts later — expected across GPUs), `BUG`
(differs from ~tick 0 — the runs did not start in the same state: diff the
fingerprints, not the physics).

⚠️ **`MATCH` at 400 ticks is not a determinism verdict, and this gate is named
"short-horizon" for a reason.** On the GPU solver, divergence grows five orders of magnitude
between 120 and 1000 steps, so a `MATCH` here is consistent with metres of separation later
([determinism-scope.md](../benchmarks/determinism-scope.md) §2). Use it as the **BUG
detector** it is described as — a tick-0 mismatch means the runs started differently — and
never as evidence that a policy rollout is replayable.

**Statistical conformance (Tier C, the cross-device contract):**
```bash
# on each machine:
python projects/policies/training/machine_conformance.py run \
    --skill g1_walk --seeds 1,2,3 --duration 90 --out conf_$(hostname).json
# anywhere:
python projects/policies/training/machine_conformance.py compare conf_a.json conf_b.json
```
Runs the skill through the skill library's canonical env, one headless run per
seed, and compares fall record (hard gate), pace (±25 % rel), pelvis height
(±2 cm), roll/pitch std (±0.02 rad). `EQUIVALENT` means the machines agree on
everything a demo can honestly promise.

## Reducing the residual variance (training side)

The trainer already applies the domain randomization that makes policies
insensitive to exactly these axes: observation noise (`OBS_NOISE`, default
0.02), motor-strength randomization (`MOTOR_RAND`, ±8 %), scheduled pushes
(`PUSH_INTERVAL`/`PUSH_VEL`), and IC randomization (`IC_RAND_*`). If Tier-C
conformance fails on pace/att bands with an otherwise-identical stack, the fix
is more DR at train time — never parity-hunting in deploy.

## The decision tree

```
same outcome, micro-gait differs        -> Tier C working as designed; run
                                           conformance to prove EQUIVALENT
outcome differs (fall / missed segment) -> diff [envfp] blocks first ([sys]
                                           packages, binary sha, seeds);
                                           then golden_compare:
                                             BUG   -> stack/env mismatch, fix it
                                             CHAOS -> retrain with more DR or
                                                      accept per-machine seeds
```
