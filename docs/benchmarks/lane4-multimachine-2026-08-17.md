# Lane 4 and cloth, on a second machine — the 78% figure stops being one laptop

**2026-08-17, second campaign of the day.**
[omnibench-multimachine-2026-08-17.md](omnibench-multimachine-2026-08-17.md) moved
OmniSim's **throughput** rows off `9722d23d12a3` and onto three machines. It left
everything else where it was, and the rows it left behind include the number this
project publishes most prominently: **the capability matrix — 45 probes, "78% of
the capabilities that exist work"** — which had never run anywhere but that laptop.

This campaign runs lane 4 (capability matrix, resource envelope, no-CUDA probe)
and the cloth step-cost matrix on a **RunPod RTX A4500 / AMD EPYC 7352**, and
re-runs the same three lanes on the laptop the same day so the comparison is not
against week-old rows. It cost **$0.15**.

Four things came out of it, and only one of them is "the number reproduced":

1. **78% reproduces exactly — by three offsetting changes, not by replication.**
   Say "78% on both machines" only with that sentence attached.
2. **`device.lidar` had been published as `no result` while working.** The pod
   measured it working; the laptop then re-measured it working in 3.7 s. The
   stale row is corrected here.
3. ⭐ **The constraint-overflow cliff FIRED, on both machines.** Lane 4b's
   silent-degradation detector had never been observed to go red, and
   [AGENTS.md] refused to read its green as evidence for exactly that reason.
   16 driven rovers peak at **`nefc` 336 (laptop) and 328 (pod) against a
   `njmax` that stays pinned at 256** — a real truncation, with a clean log and
   exit code 0. Both summary rows now read `cliff_detector_validated: true`.
4. **The cloth rows this repo publishes were stale by a factor of ten**, and the
   discovery is a *runtime* finding rather than a machine one.

---

## 0. Provenance

| | `9722d23d12a3` | `8ab788c4c833` |
|---|---|---|
| GPU | RTX 3060 Laptop, 6 GB (driver 596.36) | RTX A4500, 20 GB (driver 570.195.03) |
| CPU | AMD Ryzen (Fam 25 Mod 80), 16 threads | **AMD EPYC 7352 24-Core, 48 threads, 2.3 GHz max** |
| OS | Windows 11 | Ubuntu 22.04 (RunPod Secure, EU-RO-1) |
| engine binary | own build `13906cc6f12451eb`, sources 2026-08-17 | volume build `6f7e2217426a2088`, sources **2026-08-16 14:47Z** |
| runner python | 3.12.9 | 3.11.10 (engine links 3.10) |
| physics stack | newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0 / mujoco-warp 3.11.0 | **same** |
| price | — | $0.25/hr |

⚠ **The binary confound, stated before any result.** The two machines do not run
the same build and cannot: one is a MinGW/Windows binary, the other a gcc/Linux
one off the network volume, and the volume's was built about 20 hours earlier
from committed sources. Rebuilding it would have cost hours of pod time against a
$3 ceiling, so this campaign **pre-registered** the disagreements the commit log
predicts and then re-tested each one on the laptop under the matching revert
hatch. That is what §1.2 is.

Raw rows:
`tests/benchmarks/omnibench/results/8ab788c4c833/2026-08-17/` (pod) and
`.../results/9722d23d12a3/2026-08-17/` (laptop).
Campaign kit: `cloud/runpod/campaigns/campaign_lane4_mm.sh`, driven by the
existing `launch_omnibench_pod.sh`. ⚠️ `cloud/` is OmniLink's internal ops tree
(provider accounts, cost controls) and is **not in the public snapshot**, so the
pod recipe in §7 is a record of how these rows were produced, not something a
reader outside OmniLink can run. Every row is also reproducible locally from
committed sources — see the local commands at the end of §7.

---

## 1. Lane 4a — the capability matrix

### 1.1 The headline reproduces, and the way it reproduces matters

| | `9722d23d12a3` | `8ab788c4c833` |
|---|---:|---:|
| PASS (works) | 31 | 31 |
| PARTIAL (degraded) | 5 | 4 |
| BROKEN | 4 | 5 |
| absent | 4 | 4 |
| no result | 1 | 1 |
| **of what exists, works** | **77.5%** | **77.5%** |

(The laptop column is the state this campaign started from, i.e. before §1.3
corrects `device.lidar`.)

**42 of 45 probes agree at this point**, and **43 of 45 once §1.3 corrects the
laptop's stale `device.lidar` row** — which is the number the regenerated matrix
shows, because it renders the corrected file. The identical percentage is not a
clean replication; it is three changes that happen to cancel:

* `joint.hinge2_motor` `works → broken` on the pod (−1 works, +1 broken),
* `device.lidar` `no result → works` on the pod (+1 works),
* `object.deformable_cloth` `degraded → no result` on the pod (−1 degraded).

Two of those three are not statements about either machine. Quote the number as
*"78% on two machines, with the three per-probe disagreements attributed"*, never
as *"the matrix reproduced"*.

### 1.2 The three disagreements, attributed

**`joint.hinge2_motor` — the engine build, pre-registered and confirmed.**
`2094660ef` (2026-08-17 10:58) flipped `OMNISIM_NEWTON_BALL_HINGE2` to default
ON; the pod's binary was built 2026-08-16 14:47, so it runs the gate OFF. This
was written down before the sweep ran, and the control settles it:

| run | verdict | arm displacement | joint angle travel |
|---|---|---:|---:|
| laptop, shipped default | works | 0.19507756 m | 0.800000012 rad |
| **pod, gate OFF by build date** | **broken** | **1.4276448946637377e-14 m** | **0.0016 rad** |
| **laptop, `OMNISIM_NEWTON_BALL_HINGE2=0` control** | **broken** | **1.4276448946637377e-14 m** | **0.0016 rad** |

The pod's number and the laptop's revert-hatch control agree **to all 17 printed
digits** on a different OS, a different compiler and a different CPU. ⚠ That is
one scalar, not a trajectory: it says the two runs took the same code path, and
it does **not** extend [determinism-scope.md](determinism-scope.md), which
measures whole recordings and finds that different builds do *not* reproduce.

The control was possible because `run_coverage.py` gained `--env KEY=VAL` in this
campaign. Rows produced that way are marked `CONTROL RUN` in `deviations` and are
refused by the merge tool, because a hatch measurement is not a measurement of
the shipped default.

**`object.deformable_cloth` — the pod's instrument, not the engine.** The probe
timed out at the 200 s per-probe cap (`rc=-9`, tree-killed) on its cold warp JIT:
the pod's warp kernel cache lives on the *container* disk, which is empty on
every fresh pod, and the VBD solver is the most expensive thing in the sweep to
compile. `inconclusive` is excluded from the score by design, which is why it
costs a `degraded` and not a `works`. Raise `LANE4_MM_PROBE_TIMEOUT` on a cold
pod, or put the warp cache on the volume.

**`device.lidar` — a stale laptop row, and the reason to run a second machine.**
See below.

### 1.3 The finding: a capability was published as `no result` while it worked

`device.lidar` has been `no result` in the published matrix since 2026-08-15,
where its prober crashed (`rc=1`, *"'omnibench_prober' controller crashed"*). The
pod measured it cleanly — **min range 2.90079665 m against an expected 2.9,
32 finite returns from 32 beams** — so the laptop was re-run on its current
binary and it now measures `works` **in 3.7 seconds**.

So the row was not a hard instrument problem and was not machine-specific: it was
an instrument failure on a binary that has since been replaced, and it sat in the
published matrix for two days because nothing re-ran it. **A second machine is
how it was noticed.** That is a smaller claim than "cross-machine testing found
an engine bug" and a more useful one: the failure mode this lane is most exposed
to is a stale row, and a second machine is a cheap detector for stale rows.

Corrected laptop matrix, after topping up that one probe:

| | before | after |
|---|---:|---:|
| PASS | 31 | **32** |
| PARTIAL | 5 | 5 |
| BROKEN | 4 | 4 |
| absent | 4 | 4 |
| no result | **1** | **0** |
| of what exists, works | 77.5% | **78.0%** |

The top-up is now a script rather than a text edit
([`lane4/merge_coverage.py`](../../tests/benchmarks/omnibench/lane4/merge_coverage.py)):
it replaces named probe rows, leaves every other row **byte-identical** (verified:
44 of 46 lines unchanged, the two being the lidar row and the derived summary),
recomputes the summary instead of hand-editing a percentage, and refuses three
things outright — a row from a different machine (that would overwrite verdicts
rather than add a column), a `CONTROL RUN` row, and a row older than the one it
would replace.

---

## 2. Lane 4b — the resource envelope

### 2.1 "200 rigid bodies at 1.33× real time" holds on server silicon

Both sweeps are same-day, boxes on the CPU `mj_step` solver, 2 repeats per N.

| N | `9722d23d12a3` ms/step | realtime | `8ab788c4c833` ms/step | realtime |
|---:|---:|---:|---:|---:|
| 1 | 0.3245 | 12.33× | 0.5334 | 7.50× |
| 5 | 0.3663 | 10.92× | 0.5931 | 6.74× |
| 20 | 0.5243 | 7.63× | 0.7433 | 5.38× |
| 50 | 0.8409 | 4.76× | 1.1004 | 3.63× |
| 100 | 1.4056 | 2.85× | 1.8307 | 2.18× |
| **200** | **2.7678** | **1.45×** | **2.9604** | **1.35×** |

> **The claim survives.** 200 bodies stay above real time on both machines and
> neither sweep reaches its ceiling. The EPYC is **1.64× slower at N=1 and only
> 1.07× slower at N=200** — its lower clock costs it the fixed per-step overhead,
> and the per-body term converges. So the README sentence was not
> over-generalised, which is the opposite of what a 2.3 GHz server part versus a
> laptop Ryzen would suggest, and is worth saying plainly.

⚠ The committed laptop envelope rows were from **2026-08-10** and read 1.33× at
N=200 on a different binary. They are kept; this campaign's laptop arm is a
separate, same-day file.

### 2.2 ⭐ The silent-overflow cliff fired

Rovers, `mujoco_warp`, every wheel driven for the whole window, both machines
same-day.

| N | `9722d23d12a3` peak `nefc` / `njmax` | `8ab788c4c833` peak `nefc` / `njmax` | overflow |
|---:|---:|---:|---|
| 1 | 16 / 256 | 16 / 256 | no |
| 2 | 32 / 256 | 32 / 256 | no |
| 4 | 64 / 256 | 64 / 256 | no |
| 8 | 128 / 256 | 128 / 256 | no |
| 12 | 232 / 256 | 208 / 256 | no |
| **16** | **336 / 256** | **328 / 256** | **YES, on both** |

[AGENTS.md] records this detector as **unvalidated**: *"three attempts to force
one all read 384/384 … a green that cannot be made to go red is not evidence."*
**It has now gone red on both machines**, and both summary rows carry
`cliff_detector_validated: true`. The cap stayed pinned at the built-in 256 while
demand reached 328–336, so **constraint rows were dropped with nothing in the log
and exit code 0** — the exact failure the lane was built to catch.

Three consequences:

* the lane-4b "no overflow" greens elsewhere may now be read as real greens,
  because the instrument is demonstrably red-capable;
* the buffer does **not** auto-size. On the 2026-08-10 laptop rows the cap tracked
  demand (384/384 at N=12, 512/512 at N=16) and nothing ever overflowed, which is
  what made the detector unfireable; on **today's** binaries, on both machines, it
  does not move off 256. That was an artefact of the older build, not of the
  laptop. `WorldInfo.newtonNjmax` matters, and "mujoco_warp will size it for you"
  is refuted;
* the two machines' peaks differ slightly at the top (336 vs 328 at N=16, 232 vs
  208 at N=12) — the contact set is not identical tick for tick — but they agree
  exactly at N ≤ 8 and cross the cap at the same N.

⚠ The per-rover constraint count also moved: **16 rows per rover on both machines
today, against 32·N on the 2026-08-10 laptop sweep.** Both of today's machines
agree, so that is a dated engine change and not a machine difference — but it
means the "N wheeled robots ≈ 32·N rows, so 256 runs out at 9" rule of thumb in
AGENTS.md now reads **≈16·N up to N=8, super-linear above it, and the cap is
crossed at N=16**. (That rule was measured on the 10-Husky world, not on these
generated rovers, so it is narrowed rather than refuted.)

---

## 3. Lane 4c — no CUDA device visible

| machine | verdict | Newton finalised | rest z (m) | expected | identical to the GPU-visible run |
|---|---|---|---:|---:|---|
| `9722d23d12a3` | PASS | true | 0.6499 | 0.65 | true |
| `8ab788c4c833` | PASS | true | 0.6499 | 0.65 | true (max deviation 0.0 m) |

The README's *"with no CUDA device visible to the process, trajectory
bit-identical to the GPU-visible run"* is now measured on two machines and two
operating systems.

---

## 4. Cloth step cost — the least confounded comparison, and the most surprising

`cloth_bench.py` drives `src/omnisim/physics/omnisim_newton_runtime.py` directly
against newton/warp and **never launches the engine**, so this is the one lane
here with no binary confound at all. Both machines ran HEAD's runtime (the pod
copy is overlaid after every engine phase, precisely so the engine never sees a
runtime its C++ was not built against).

| cell | `9722d23d12a3` ms/step | realtime | `8ab788c4c833` ms/step | realtime |
|---|---:|---:|---:|---:|
| drape 289p, as shipped | 2.847 | 2.81× | 2.359 | 3.39× |
| drape 289p, `mujoco_warp` | 2.627 | 3.05× | 2.359 | 3.39× |
| drape 289p, whole-world VBD | 2.518 | 3.18× | 2.305 | 3.47× |
| t-shirt 616p, as shipped | 2.922 | 2.74× | 2.415 | 3.31× |
| t-shirt 616p, `mujoco_warp` | 2.774 | 2.88× | 2.421 | 3.30× |
| t-shirt 616p, whole-world VBD | 2.579 | 3.10× | 2.345 | 3.41× |
| drape, self-contact ON | 2.386 | 3.35× | 2.372 | 3.37× |
| drape, self-contact OFF | 2.243 | 3.57× | 1.388 | 5.76× |
| drape, VBD iters 2 | 1.284 | 6.23× | 0.604 | 13.24× |
| drape, VBD iters 5 | 2.257 | 3.54× | 0.906 | 8.83× |
| drape, VBD iters 10 | 3.733 | 2.14× | 1.404 | 5.70× |

The A4500 is **1.0–2.1× faster**, with the gap widening as the cell gets cheaper
— the pattern of a launch-bound workload on a bigger card, not of a different
algorithm.

⚠ **The committed cloth rows were stale, and by a lot.** The rows this repo
publishes were taken 2026-08-15 15:30, **29 minutes before `1fb7f135f`** ("cloth:
above real-time (10-12x)"). On that file the shipped drape cell reads
**30.323 ms/step at 0.264× real time with no CUDA graph captured**; re-measured
today on HEAD's runtime it is **2.847 ms at 2.81× with a graph**. That is a
runtime change, not a machine one — both of today's machines see the fast number
— but it means every cloth figure quoted from that file is an order of magnitude
wrong.

⚠ **`sweep.py`'s solver labels do not say what device ran.** The cell labelled
*"AS SHIPPED (coupled, CPU mj_step)"* captured a CUDA graph on both machines
today, which a CPU `mj_step` cannot do — the label is the script's prose, not a
measurement. `sweep.py` now stamps each row with a full machine fingerprint;
recording the device is the remaining hole, and it is exactly the kind of hole
that made the next paragraph's figure unfalsifiable for three days.

### "Cloth on the CPU is 6.7 fps" — stale by 2.9×, and still one machine

That figure is a **device-forced** measurement, so the sweep above does not touch
it. Re-measured directly, `cloth_bench.py --scene drape --solver mujoco+vbd
--device cpu`, on the same laptop that produced the original:

| | `ms/step` | frames/s | realtime |
|---|---:|---:|---:|
| recorded in [cloth-simulation.md](../developer/cloth-simulation.md) | ~149 | **6.7** | 0.054× |
| **measured 2026-08-17, laptop, HEAD runtime** | **51.866** | **19.3** | **0.154×** |

`solver_obj` reports `SolverCoupledProxy(mjc cpu=True)`, `device: cpu`,
`graph_armed: false` — so this is genuinely the CPU path. The conclusion the
figure supports is unchanged (**cloth is a GPU feature**: 0.154× is still not a
simulation anyone can drive, against 2.81–3.39× on the GPU), but the number is
2.9× off and is now corrected. ⚠ **This one is still a single machine** — the pod
was terminated before the device-forced arm was thought of, and re-renting one
for a single cell was not worth $0.25/hr against the campaign's remaining
purpose.

---

## 5. What is still one machine, and why

* **The cloth-grasp table (README §5)** — tracking error, slip, jaw gap and the
  negative control. Not attempted here, and deliberately: the numbers depend on
  `e87f42c33` and `b368937dd` (2026-08-16 11:39 and 16:25), and the volume's
  engine binary was built at **14:47** that day — between them. Measuring on that
  pod would have produced numbers about a different engine and published them
  next to the laptop's as if they were comparable. It needs a pod rebuild.
* **"Cloth on the CPU, 6.7 fps"** — see §4.
* **Everything in README §7** — those are GAUGE's published figures, not ours.

---

## 6. Cost, and the pod discipline

| | |
|---|---|
| balance before | $7.4832 |
| balance after | $7.3316 |
| **total spend** | **$0.1516** (pod + the standing volume's slice) |
| pod | RTX A4500, $0.25/hr, ~35 min — lane 4a/4b/4c + cloth |
| `GET /v1/pods` at the end | `[]` |

The delete watchdog was armed **before anything else ran** (root-only REST
DELETE; `runpodctl` still returns `Unauthorized` on this image), with a local
backup timer, and the pod was TERMINATED, not stopped, the moment its exit
condition was met. Everything was written under `/workspace`.

Two operational notes:

1. **`launch_omnibench_pod.sh` runs the py310 newton patch BEFORE it extracts the
   payload**, so shipping the patch in the payload does not help — the file has
   to already be on the pod. `cloud/` is not in the public snapshot, so a fresh
   pod's checkout never has it. Cost here: one failed driver run, ~7 minutes.
2. **The warp kernel cache is on the container disk (`/root/.cache/warp`), not
   the volume**, so every fresh pod pays the cold JIT. That is what timed out
   `object.deformable_cloth` at the 200 s cap.

---

## 7. Reproducing this

The pod half below references the internal `cloud/runpod/` kit (see §0); it is
recorded for provenance. The local commands after the block need none of it.

```bash
# local prep (free): payload from COMMITTED state, never the working tree
S=/tmp/pay && rm -rf $S && mkdir -p $S
git archive --format=tar HEAD \
  tests/benchmarks/omnibench tests/benchmarks/cloth_step_cost \
  cloud/runpod/campaigns/_campaign_lib.sh \
  cloud/runpod/campaigns/campaign_lane4_mm.sh \
  cloud/runpod/campaigns/fold/py310_patch.py \
  projects/policies/common/env_fingerprint.py \
  scripts/packaging/newton_runtime_pins.py | tar -x -C $S
mkdir -p $S/tests/benchmarks/cloth_step_cost/_runtime
git show HEAD:src/omnisim/physics/omnisim_newton_runtime.py \
  > $S/tests/benchmarks/cloth_step_cost/_runtime/omnisim_newton_runtime.py
( cd $S && tar -czf /tmp/lane4_mm_payload.tar.gz . )

# on the pod, AFTER arming the watchdog and after scp'ing py310_patch.py to
# /workspace/omnisim/cloud/runpod/campaigns/fold/ (see §6 note 1):
PAYLOAD=/workspace/lane4_mm_payload.tar.gz \
BOOTSTRAP=/workspace/linux_bootstrap_current.sh \
CAMPAIGN=/workspace/omnisim/cloud/runpod/campaigns/campaign_lane4_mm.sh \
OMNIBENCH_MM_TAG=<gpu-tag> LANE4_MM_PROBE_TIMEOUT=300 \
  bash /workspace/launch_omnibench_pod.sh
```

Locally the same rows come from
`tests/benchmarks/omnibench/lane4/run_coverage.py`,
`.../run_envelope.py --sweep both`, `.../cpu_only.py` and
`tests/benchmarks/cloth_step_cost/sweep.py all`. Render the two-machine matrix
with:

```bash
python tests/benchmarks/omnibench/lane4/report.py \
  --cross tests/benchmarks/omnibench/results/8ab788c4c833/2026-08-17/lane4 \
  --out docs/benchmarks/lane4-capability-matrix.md
```

[AGENTS.md]: ../../AGENTS.md
