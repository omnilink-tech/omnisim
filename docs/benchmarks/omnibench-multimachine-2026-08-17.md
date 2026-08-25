# OmniBench multi-machine — the headline numbers stop being n=1 machine

**2026-08-17.** Every OmniSim performance number this repo published came from
one box: `9722d23d12a3`, a laptop RTX 3060. The README said so honestly, and that
honesty was the weakest part of an otherwise strong comparison. This campaign
re-measures the same lane-2 rows on **three** machines, runs the lane-1
correctness sweep on **two**, and settles a question
[determinism-scope.md](determinism-scope.md) had recorded as **UNTESTED**.

It also found three defects on the way — two in the benchmark, one in a training
launcher — and one of them had **silently deleted the OmniSim column from lane
2**. Read §5 before quoting any lane-2 number measured between the newton 1.5.0
upgrade and today.

> ⚠️ **THIS CAMPAIGN COVERED THROUGHPUT AND CORRECTNESS ONLY.** The capability
> matrix, the CPU real-time envelope and the cloth step-cost rows were left on
> one laptop and were moved off it later the same day, in a second campaign:
> **[lane4-multimachine-2026-08-17.md](lane4-multimachine-2026-08-17.md)** ($0.15,
> RunPod RTX A4500). Read that one before quoting the 78% capability figure, the
> "200 bodies at real time" row, or any cloth ms/step. The **cloth-grasp table**
> in README §5 is still one machine, and that document says why.

---

## 0. Provenance

| | `9722d23d12a3` | `c72ce5632c81` | `b5dadd645b1f` |
|---|---|---|---|
| GPU | RTX 3060 Laptop, 6 GB | RTX 4000 Ada Generation, 20 GB | RTX 4090, 24 GB |
| driver | 596.36 | 550.127.05 | 570.195.03 |
| CPU | AMD Ryzen, 16 threads | AMD EPYC 7352, 48 threads | not captured, 32 threads |
| OS | Windows 11 | Ubuntu 22.04 (RunPod Secure, EU-RO-1) | Ubuntu 22.04 (RunPod Secure, EU-RO-1) |
| engine binary | own build, sha256 `6aac9ae1b461f567` | volume build, sha256 `6f7e2217426a2088` | **same** `6f7e2217426a2088` |
| runner python | 3.12.9 | 3.11 (engine links 3.10) | 3.11 (engine links 3.10) |
| torch | 2.5.1+cu121 | 2.4.1+cu124 | 2.4.1+cu124 |
| physics stack | newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0 / mujoco-warp 3.11.0 | **same** | **same** |
| price | — | $0.28/hr | $0.74/hr |

**The two pods share one binary**, off a shared network volume (built
2026-08-16 14:47 UTC). That is what makes the cross-machine comparisons in §2
and §3 comparisons of *machines* rather than of builds. The laptop's binary is a
separate Windows build of a nearby tree, so every laptop-vs-pod difference below
confounds machine with compiler and OS — stated wherever it matters.

⚠ **The pods' CPU model is recorded for one pod and not the other**, because
`env_fingerprint.py` reported `"cpu": "x86_64"` on Linux. Pod A's EPYC was
recovered by SSHing in before it was reaped; pod B's was already gone. Fixed in
`8b6f346a6` — reporting only, the `id` hash is unchanged so today's ids stay
joinable.

Raw rows: `tests/benchmarks/omnibench/results/<machine-id>/2026-08-17/`.
Campaign kit: `cloud/runpod/campaigns/campaign_omnibench_mm.sh`, driven by the
existing `launch_omnibench_pod.sh`. ⚠️ `cloud/` is OmniLink's internal ops tree
(provider accounts, cost controls) and is **not in the public snapshot**, so the
pod recipe in §7 is a record of how these rows were produced, not something a
reader outside OmniLink can run. The local commands at the end of §7 reproduce
the same rows from committed sources.

---

## 1. Lane 2 — throughput on three GPUs

Go2, contacts ON, actions never idle, one control step = 16 ms = 8 substeps.
Medians; where n>1 the range is given. `raw` = `mujoco_warp` stepping the
engine-exported MjModel, CUDA-graphed. `Ag` = the same model through OmniSim's
embedded Newton `SolverMuJoCo`, also graphed — **the like-for-like pair**. `A` is
the same OmniSim path *ungraphed*, kept because
[lane2-graphed-ab-2026-08-08.md](lane2-graphed-ab-2026-08-08.md) obliges anyone
quoting `Ag` to quote `A` beside it.

### `raw` — mujoco_warp baseline, env-steps/s

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 54,954 *(n=4, 38,492–56,072)* | 69,730 *(n=4, 69,528–69,974)* | 80,992 *(n=4, 80,696–81,366)* |
| 1024 | 142,921 *(n=3, 96,860–144,789)* | 210,572 | 282,516 |
| 4096 | 229,450 *(n=4, 153,223–236,104)* | 403,988 *(n=4, 403,560–404,797)* | 747,491 *(n=4, 746,679–748,330)* |

### `Ag` — OmniSim embedded solver, graphed, env-steps/s

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 43,141 *(n=4)* | 55,400 *(n=4)* | 65,471 *(n=4)* |
| 1024 | 107,839 *(n=3)* | 157,979 | 226,089 |
| 4096 | **165,369** *(n=4, 157,645–168,667)* | **280,820** *(n=4, 280,440–280,943)* | **535,377** *(n=4, 534,970–536,755)* |

### `A` — the same path ungraphed (the conservative lower bound)

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 2,219 | 3,589 | 10,388 |
| 1024 | 6,883 | 14,003 | 40,668 |
| 4096 | 18,593 | 53,867 | 157,419 |

### `C` — full in-engine PPO through `omnisim-bin`, env-steps/s

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 8,813 | 20,874 | 46,717 |
| 1024 | — | 77,636 | 168,334 |
| 2048 | — | 132,078 | 298,405 |
| 4096 | **98,136** | **201,850** | **499,734** |

### The overhead ratio holds across machines

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 1.27× | 1.26× | 1.24× |
| 1024 | 1.33× | 1.33× | 1.25× |
| 4096 | 1.39× | 1.44× | 1.40× |

**This is the campaign's most quotable result.** OmniSim's embedded deploy solver
costs **1.24–1.44×** raw `mujoco_warp` on the identical model, and the ratio is a
property of the *batch size*, not of the GPU: at 4096 the three machines span
3.3× in absolute throughput and agree on the ratio to within 0.05×. The previous
published range, **1.21–1.34×**, came from one machine at two batch sizes and did
not include 4096 (a 6 GB card was thought unable to host it — it can).

### ⚠ Tier B on a pod is a CPU benchmark, not a GPU one

| batch | RTX 3060 | RTX 4000 Ada | RTX 4090 |
|---:|---:|---:|---:|
| 256 | 33,444 | 12,571 | 50,893 |
| 4096 | 163,940 | 58,540 | 362,852 |

The RTX 4000 Ada pod is **2.8× slower than the laptop 3060** on tier B while
being 1.76× faster on tier A. Tier B runs the champion ONNX policy through
`CPUExecutionProvider` in the loop, so it is bounded by single-thread CPU speed;
the pod's EPYC 7352 clocks far below the laptop's Ryzen. Do not read tier B as a
GPU ranking, and do not compare tier B across machines with different CPUs
without saying so.

---

## 2. Lane 1 — is correctness machine-independent?

The full SPEC sweep (7 scenes × dt ∈ {1,2,4,8,16,32} ms × {mujoco, pybullet,
omnisim-newton}) ran on **both pods** — 126 rows each, 42 of them
`omnisim-newton`. The two pods run **the same binary**, so this is the
comparison the earlier census could not make.

> **107 of 114 comparable `omnisim-newton` metric cells are bit-identical
> across the two machines. All 7 that differ do so at rel ≤ 2.7e-16 — one ULP
> of double precision, in the offline scorer's arithmetic, not in the
> trajectories.**

| rel | scene | dt | metric | RTX 4000 Ada | RTX 4090 |
|---:|---|---:|---|---|---|
| 2.741e-16 | momentum | 2 | `angular_momentum_drift_rel_v0` | 0.025312608635784373 | 0.025312608635784366 |
| 2.128e-16 | momentum | 2 | `angular_momentum_drift_rel` | 0.00815073395193209 | 0.008150733951932089 |
| 2.067e-16 | momentum | 2 | `angular_momentum_drift_abs` | 0.00419525250207628 | 0.004195252502076279 |
| 1.693e-16 | stack | 16 | `settle_creep_m_s` | 41.96813142043104 | 41.96813142043105 |
| 1.691e-16 | momentum | 32 | `angular_momentum_drift_rel` | 0.6565057295637232 | 0.6565057295637231 |
| 1.374e-16 | momentum | 32 | `angular_momentum_drift_abs` | 0.8078936541588807 | 0.8078936541588806 |
| 1.158e-16 | momentum | 32 | `angular_momentum_drift_rel_v0` | 0.958427921274626 | 0.9584279212746258 |

**Put beside the earlier census, this is the finding.**
[simulator-comparison.md](../developer/simulator-comparison.md) records a
cell-level cross-machine census from the 2026-07-24 campaign: 3060 vs a 4090
pod, **124 of 180 bit-identical, 13 at rel ≥ 1e-3**. Those two machines ran
*different builds on different operating systems*. Hold the build fixed and the
spread collapses to one ULP. So the honest reading is:

> Lane-1 correctness is **not** machine-sensitive on the CPU `mj_step` path. The
> 2026-07-24 spread was a build/OS difference wearing a machine's clothes.

⚠ **What this does not say.** It compares two Linux pods; the Windows arm was
not re-swept today, so the build/OS half of that claim is inferred from the
older census rather than re-measured. `mujoco_warp` was not exercised — lane 1
runs the CPU solver. And "bit-identical metrics" is a statement about the
scored scalars, not a proof that every recorded trajectory sample matched.

The **translation audit** ran as a campaign stage on both pods and returned
**0 errors, 0 warnings** over the 13 lane-1 worlds: gravity, up-axis, timestep
and per-geom friction all reach the model as authored (e.g. `2/2 geoms carry the
declared mu 0.5`).

---

## 3. Cross-machine determinism — the UNTESTED box gets an answer

[determinism-scope.md](determinism-scope.md) listed cross-machine
reproducibility as untested, with a warning attached: "this document's sibling
census found 56 of 180 lane-1 cells differing between two machines … so do not
assume it." Two scenes were run cold twice on each of three machines and the raw
`cold1` recordings hashed
([`results/cross_machine_determinism_2026-08-17.jsonl`](../../tests/benchmarks/omnibench/results/cross_machine_determinism_2026-08-17.jsonl)):

| scene | steps | `9722d23d12a3` (own Windows build) | `c72ce5632c81` | `b5dadd645b1f` |
|---|---:|---|---|---|
| `lane3_determinism.wbt` — 5 spheres | 400 | `232c0407…` | `e0833955…` | **`e0833955…`** |
| `lane3_det_probe_dense_cpu.wbt` — 10 robots + 64-box tower, **336 contacts**, ten live controllers | 120 | `96708d5d…` | `83976c8d…` | **`83976c8d…`** |

Every machine graded `bitwise` against itself. Across machines:

> **On the CPU MuJoCo path, two different machines running the same binary
> produced byte-identical recordings — on a light scene and on a contact-rich
> one.** The pods differ in GPU (RTX 4000 Ada vs RTX 4090), in host, and in core
> count (48 vs 32 threads); pod A's CPU is an AMD EPYC 7352.

And the negative half, which matters just as much:

> **A different build of the same source does not reproduce.** The Windows
> laptop's recordings differ from the pods' on both scenes. That is expected —
> different compiler, different libm, different OS — and it means the claim is
> scoped to *the same binary*, never to "the same version of OmniSim".

⚠ Scope, precisely: CPU `mj_step` only, one solver, two scenes, 1.6 s and 3.84 s
of sim, n=1 pair per scene, both pods RunPod Secure hosts in one datacentre with
the same OS image and the same pinned wheel stack. Nothing here says anything
about `mujoco_warp`, which
[determinism-scope.md §2](determinism-scope.md) shows is not reproducible even
run-to-run on one machine. Nothing here extends the horizon beyond 3.84 s. And
two hosts is not a fleet.

---

## 4. What disagreed with the laptop, and what changed under it

Three disagreements are worth naming; none is reconciled here.

**4.1 The laptop's raw arm has a cold first pass; the other two machines do
not.** On `9722d23d12a3` the first `raw` measurement in a fresh process reads far
low and then settles:

| batch | pass 1 | passes 2–4 |
|---:|---:|---|
| 256 | 38,492 | 56,072 · 54,715 · 55,193 |
| 4096 | 153,223 | 236,104 · 234,111 · 224,789 |

The `Ag` arm on the same passes is flat to ~1% (43,193 · 43,090 · 42,182 ·
43,431). On the RTX 4000 Ada and the RTX 4090 the first pass is already
steady-state (raw @256: 69,699 then 69,528/69,974/69,761). The 20-step warmup
does not amortise first-time kernel work for the raw arm on that card, and the
newton arm's does. **Consequence for the suite: a single-shot lane-2 raw row on
that machine is a cold row, not a measurement.** Taken at face value, pass 1
would have published "OmniSim's embedded solver is *faster* than raw
mujoco_warp at 4096 on the 3060" (166,797 vs 153,223) — a result that
disintegrates at n=2 and that SPEC's own framing says should have been
disbelieved on sight, since the embedded solver *is* mujoco_warp. Every lane-2
number in §1 is a median of the repeats for that reason.

**4.2 The 3060's own numbers moved under the newton 1.5.0 upgrade, and the
published figures predate it.** The committed 3060 rows the README quoted were
taken on newton 1.2.0 / warp 1.13.0 / mujoco 3.8.1:

| row | published (newton 1.2.0) | today (newton 1.5.0) |
|---|---:|---:|
| `Ag` @4096 | 129,431 | **165,369** |
| tier C @4096 | 75,659 | **98,136** |
| tier C @256 | 10,576 | 8,813 |

The two 4096 rows are up 28% and 30%; the 256 tier-C row is *down* 17%. This is
reported, not explained: n=1 per cell for tier C, and the stack, the engine
build and the driver all moved between the two dates. **Do not attribute the
change to the newton upgrade on this evidence.** The point for a reader is
narrower and firm: the README's old figures were stale, and they are replaced
above.

**4.3 The pods' 4090 is not the published 4090.** The committed 4090 rows
(`65dd6587d5c9`, 2026-07-25, build `95f5a2c`, newton 1.2.0) read `Ag` @4096
500,105 and tier C @4096 333,036. Today's 4090 (`b5dadd645b1f`) reads 535,377
and 499,734. Different host, different build, different stack — three variables,
one comparison. Both sets stay in the tree; neither supersedes the other.

---

## 5. Three defects, and what each of them looked like from outside

### 5.1 Lane 2's OmniSim arm had been dead since newton 1.5, and reported it as a row

newton 1.5 removed `joint_target_pos` from the `ModelBuilder`, the `Model` and
`Control` in favour of `joint_target_q`; the old name is a `RemovedAttribute`
descriptor that **raises**. `run_newton()` — the whole `omnisim-newton` arm of
tiers A and Ag — touches it three times, so on the pinned stack every point died
with

```
AttributeError: ModelBuilder.joint_target_pos was removed in Newton 1.5
```

and emitted `[lane2] ROW engine=omnisim-newton tier=sim_only batch=256
env-steps/s=None status=dropped`. The raw arm reported normally and the process
exited 0. A campaign run in that state publishes a lane-2 table with the OmniSim
column simply *absent* and no failing exit code anywhere.

The engine's own runtime was migrated version-tolerantly when the stack moved
(`omnisim_newton_runtime._ctl_target_pos`); the benchmark was not. Fixed in
`a27bfd2e2` by resolving the array by attribute per call, matching the engine's
idiom, so one source drives both runtimes.

### 5.2 newton 1.5.0 does not import in the engine's Python 3.10 — and the preflight said it did

The Linux engine embeds the distro python3.10. newton 1.5 targets 3.11+ and
annotates with PEP-604 unions (`wp.array[wp.bool] | None`) evaluated at def
time, which 3.10 rejects. **`import newton` still succeeds** — the failure is
inside `ModelBuilder()` and the solver modules — so `launch_omnibench_pod.sh`'s
version-only preflight passed and the break surfaced twenty minutes later, once
per engine launch, as

```
FATAL: [OmNewtonBackend] ... the Newton runtime is INSTALLED but did not come
       up: the FFI smoke check failed
```

Measured on this campaign's first pod: **15 engine launches, zero `.newton.json`
sidecars, the entire omnisim arm of lane 1 recorded as gaps**, while the mujoco
and pybullet arms produced 84 rows and the campaign looked healthy. The fold
campaign already carried the patch (`py310_patch.py`); it is now a driver step,
and the preflight now constructs a `ModelBuilder` and imports the solvers,
because asserting a version string is exactly what let this through — the
version was correct the whole time (`0f923a9ee`).

The patch lands in the **container's** dist-packages, which no network volume
carries, so every fresh pod needs it again.

### 5.3 The `.omniworld` migration renamed the trainer's world out from under it

`run_quad_walk_rl.sh` resolved
`projects/policies/research/worlds/${ROBOT}_walk_deploy.wbt`, hardcoded. The
extension migration renamed that file; the g1 runners were updated and this one
was not. On a migrated tree the quadruped trainer dies at `World not found`
before a single step, taking **lane 2 tier C — the README's PPO row** with it:
`status=not_run`, `env_steps_per_s: null`, twice, with a clean launcher rc.

It only surfaced because the same tier ran fine on a pod whose checkout still
carried the pre-migration filename. Fixed in `2779def9c` by applying the
dual-read rule AGENTS.md already states. Tier C produced real rows immediately
afterwards, which is where the 3060 column of §1 comes from.

**The shape all three share:** each failed *inside* an rc=0 run and removed a
whole column from a results table rather than stopping anything. That is the
failure mode SPEC.md's honesty rules exist for, and none of the three would have
been caught by a run that only checked exit codes.

---

## 6. Cost, and the pod discipline

| | |
|---|---|
| balance before | $7.9011 |
| balance after | $7.4954 |
| **total spend** | **$0.4057** (both pods + the standing volume's slice) |
| pod A | RTX 4000 Ada, $0.28/hr, ~40 min — lane 1, lane 2 all tiers, lane 3a, both determinism scenes |
| pod B | RTX 4090, $0.74/hr, ~15 min — lane 2 all tiers, lane 3a, both determinism scenes, lane 1 |
| `GET /v1/pods` at the end | `[]` |

Both pods were watchdog-armed **before any other command** with the root-only
REST-DELETE script (`runpodctl` still returns `Unauthorized` on this image), and
both were TERMINATED, not stopped. Everything was written to the network volume
as it was produced.

Four operational notes worth keeping:

1. **The cheap pod did more of the work than the expensive one, on purpose.**
   Lane 1 is CPU-bound — the GPU idles through it — so it ran on the $0.28/hr
   card, and the $0.74/hr card was taken only for the GPU lanes plus a lane-1
   arm added at the end specifically to get the same-binary comparison in §2.
   The 4090 was live for about a quarter as long as the Ada.
2. **The volume already carried a built engine** (from the fold campaign,
   2026-08-16), so no round paid the multi-hour rebuild that
   the internal RunPod ops README (§6a-bis) warns about. That single fact
   is most of why this campaign cost forty cents.
3. **Polling a pod over one-shot SSH exhausts the local machine's ephemeral
   ports.** ~12,000 sockets in `TIME_WAIT` out of a 16,384 range, and every
   subsequent `ssh`/`scp` failed with `Address already in use` for several
   minutes — during which a pod is still billing. Use `ControlMaster`
   multiplexing, or one long session that does several things, rather than a
   poll loop of short connections.
4. **A chained "wait for X then run Y" job must be serialised against the other
   chains.** A lane-2 repeat pass and a lane-1 gap retry were both keyed to
   "campaign complete" and would have run concurrently, contending for the same
   box and contaminating the throughput medians. They were re-keyed so the
   timing-sensitive one runs alone.

---

## 7. Reproducing this

The pod half below references the internal `cloud/runpod/` kit (see §0); it is
recorded for provenance. The local commands after the block need none of it.

```bash
# local prep (free): payload from COMMITTED state, never the working tree
git archive --format=tar HEAD \
  tests/benchmarks/omnibench cloud/runpod/campaigns/_campaign_lib.sh \
  cloud/runpod/campaigns/campaign_omnibench_mm.sh \
  cloud/runpod/campaigns/fold/py310_patch.py \
  projects/policies/common/env_fingerprint.py \
  scripts/packaging/newton_runtime_pins.py | gzip -9 > omnibench_mm_payload.tar.gz

# on the pod, after arming the watchdog (cloud/runpod/README.md §6.1):
PAYLOAD=/workspace/omnibench_mm_payload.tar.gz \
BOOTSTRAP=/workspace/linux_bootstrap_current.sh \
CAMPAIGN=/workspace/omnisim/cloud/runpod/campaigns/campaign_omnibench_mm.sh \
OMNIBENCH_MM_TAG=<gpu-tag> OMNIBENCH_MM_LANES=lane1,lane2,lane2c,lane3det \
  bash /workspace/launch_omnibench_pod.sh
```

Locally, the same rows come from
`tests/benchmarks/omnibench/lane2/run_throughput.py --tiers raw,A,Ag,B` and
`--tiers C --tierc-envs N`, plus `lane3/determinism.py`. **Run lane 2 at least
three times and take medians** — see §4.1.
