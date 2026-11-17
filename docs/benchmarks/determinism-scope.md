# Determinism: what OmniSim actually reproduces, and what it does not

> ⚠️ **RE-SCOPED 2026-08-08: THE ODE HALF OF THIS DOCUMENT IS NO LONGER
> VERIFIABLE.** `bdc02139` deleted the ODE backend. Every ODE row below was
> honestly measured on 2026-07-26 and is preserved as a **historical
> measurement that can never be re-run** — there is no ODE arm to re-measure
> against. The same applies to the XPBD row: XPBD was removed in `94f04222`.
>
> **The externally quotable claim is now narrower.** Use this sentence and no
> other:
>
> > On the CPU MuJoCo path (`newtonSolver "mujoco"`, the default), OmniSim
> > reproduces contact-rich ten-robot scenes bitwise across cold launches on one
> > machine. On the GPU path (`mujoco_warp`) it does not.
>
> What that deliberately does **not** say: nothing about ODE (unverifiable),
> nothing about a world reload at ten-robot scale (untested), and nothing about
> horizons beyond the 3.84 s of sim actually measured.
>
> ✅ **The cross-machine clause was removed from that list on 2026-08-17**: it is
> now measured, and it holds *for one binary across two machines* — see the
> first bullet of §1's "What is untested". The externally quotable sentence may
> therefore be extended to *"…across cold launches on one machine, and across
> two machines running the same binary"* — and no further: a different **build**
> of the same source measurably does not reproduce. ⚠ And note what the deletion removed from the
> *argument*, not just the table: the CPU-bitwise rows used to be corroborated by
> an independent serial CPU backend agreeing bitwise on the same scenes. That
> corroboration is gone. The Newton CPU result stands on its own three pairs.

**2026-07-26.** Measured scope of OmniSim's run-to-run reproducibility, the
false positive that briefly inflated it, and the refutation that settled it.
Machine `9722d23d12a3` (RTX 3060 Laptop, driver 596.36, Ryzen 16-core, Windows
11), build `3c995c9c`, binary sha256 `1c0581cc9c06684e`, newton 1.2.0 / warp
1.13.0 / mujoco_warp 3.8.0.3.

Harness: [`tests/benchmarks/omnibench/lane3/determinism.py`](../../tests/benchmarks/omnibench/lane3/determinism.py).
Grades a pair of recordings `bitwise` / `tolerance` / `divergent` / `no_motion`
/ `no_data` from per-step pose+velocity CSVs written at `%.17g`.

---

## 1. The claim, scoped

Pair counts are stated per row so the totals are derivable rather than asserted.

| configuration | scene | steps (sim time) | pairs | result |
|---|---|---|---|---|
| ~~**ODE**~~ *(backend deleted — historical)* | 5 DEF'd spheres onto a pedestal, cold-cold **and** cold-warm-reload | 400 (1.6 s) | 4 | **bitwise**, `max_abs_dev = 0.0` |
| ~~**Newton, XPBD**~~ *(solver removed — historical)* | same 5-sphere scene, cold-cold and cold-warm | 400 (1.6 s) | 6 | **bitwise** |
| ~~**ODE**~~ *(backend deleted — historical)* | 10 robots on a ring, 80/320 contacts | 120 (3.84 s) | 1 | **bitwise** |
| **Newton, `newtonSolver "mujoco"` (CPU `mj_step`)** | 10 robots + 64-box tower, 336 contacts / 1344 rows, **ten live controllers** | 120 (3.84 s) | 3 | **bitwise** |
| **Newton, `"mujoco"` (CPU)** | 64-box collapsing pile, **no controllers** | 120 (3.84 s) | 1 | **bitwise** |
| **Newton, `"mujoco_warp"` (GPU)** | six scenes, 80 → 336 concurrent contacts | 120 and 1000 | **24** | **NOT reproducible — 0 bitwise** |

The three struck rows were measured, not estimated, on the date in the header.
They are struck because **the configurations they name no longer exist in the
tree** (ODE deleted `bdc02139`, XPBD removed `94f04222`), so they can never be
confirmed or refuted again. Keep them for the record; never cite them as current
capability.

The "**5 of 5**" figure quoted elsewhere is the contact-rich CPU-path subset —
the last three rows (1 + 3 + 1) — i.e. the like-for-like counterparts of the
GPU scenes. ⚠ **That subset is unaffected by the deletion** — all three are
Newton CPU rows — so "5 of 5" remains a live figure. The **10** bitwise rows in
the omnibench campaign report are the two 5-sphere rows across three machines,
and **one of those two rows was the ODE arm**, so that 10 is now half historical.

So the honest sentence, and the only one to use externally:

> On the CPU MuJoCo path (`newtonSolver "mujoco"`, the default), OmniSim
> reproduces contact-rich ten-robot scenes bitwise across cold launches on one
> machine. On the GPU path (`mujoco_warp`) it does not — consistent with
> MuJoCo-Warp's own documentation.

⚠️ **Do not extend that sentence with "including across a world reload"** — the
cold-warm reload evidence is the **5-sphere** scene only. Reload reproducibility
at ten-robot scale is untested.

### What is untested, and one of these is uncomfortable

- ✅ **Cross-machine reproducibility — MEASURED 2026-08-17, and it HOLDS, scoped
  to one binary.** Two RunPod hosts running **the same** Linux binary (sha256
  `6f7e2217426a2088`) — an RTX 4000 Ada / EPYC 7352 / 48 threads and an RTX 4090
  / 32 threads — produced **byte-identical** `cold1` recordings on both the
  5-sphere scene (400 steps) and the contact-rich one (10 robots + 64-box tower,
  336 contacts, ten live controllers, 120 steps). Hashes, one row per machine
  per scene, in
  [`results/cross_machine_determinism_2026-08-17.jsonl`](../../tests/benchmarks/omnibench/results/cross_machine_determinism_2026-08-17.jsonl);
  campaign write-up in
  [omnibench-multimachine-2026-08-17.md](omnibench-multimachine-2026-08-17.md) §3.
  Lane 1 agrees: 107 of 114 comparable metric cells bit-identical across those
  two machines, the other 7 at rel ≤ 2.7e-16 (one ULP, in the offline scorer).

  ⚠️ **The scope is "same binary", NOT "same version of OmniSim", and the
  difference is measured, not hypothetical.** The Windows laptop —
  `9722d23d12a3`, its own build of a nearby tree — differs from the pods on both
  scenes. Different compiler, libm and OS; expected, and exactly why the claim
  cannot be widened.

  ⚠️ Also still narrow: CPU `mj_step` only, two scenes, 1.6 s and 3.84 s of sim,
  n=1 pair per scene, both hosts in one datacentre on one OS image with one
  pinned wheel stack. MuJoCo's own guarantee ("same version, same architecture")
  is not exceeded by this — it is corroborated on two hosts.

  ⚠️ The 56-of-180 census this bullet used to cite as the reason for doubt is
  **still a real measurement**, but it is now attributable: those two machines
  ran *different builds on different operating systems*. Hold the build fixed
  and the spread collapses to one ULP. Cite it as a build/OS result, not a
  machine one.

  Untouched by all of this: `mujoco_warp`, which §2 shows is not reproducible
  even run-to-run on one machine.
- ~~**XPBD at contact density AND at horizon.**~~ **Moot** — XPBD was removed
  (`94f04222`) while still unproven at density and horizon. The caution it
  carried is worth keeping in general form: a bitwise result on *one light scene
  at 1.6 s* is weak evidence for a warp-kernel solver, because §2 shows GPU-path
  divergence growing five orders of magnitude between 120 and 1000 steps.
- **Horizon on the CPU rows.** The CPU `mj_step` rows are 3.84 s of sim and have
  not been run long. If a claim depends on a 60-second rollout being replayable,
  measure it. ⚠ This bullet used to lean on ODE ("CPU and serial, so it should
  hold indefinitely") as the reassuring case; that reasoning is no longer
  available, and `mj_step` being CPU is **not** by itself a guarantee at horizon.

## 2. How much the GPU path diverges

Same configuration, cold launches, `determinism.diff()` unmodified:

| scene | contacts (ncon/nefc) | steps | pairs | max abs deviation |
|---|---|---|---|---|
| 10 robots, ring | 80 / 320 | 120 | 10 | 4.15e-5 … 7.10e-5 m |
| 10 robots, ring, **GPU contended** | 80 / 320 | 120 | 3 | 1.39e-4 … 2.30e-4 m |
| 10 robots, ring | 80 / 320 | **1000** | 1 | **9.152 m** |
| 10 robots + 64-box tower | 336 / 1344 | 120 | 10 | 0.808 … 1.527 m |
| 64-box collapsing pile, no controllers | high | 120 | 6 | 2.661 … 3.415 m |

Two things this table settles. Divergence **grows with simulated time** — five
orders of magnitude between 120 and 1000 steps — so a short run understates it.
And it is **not** an artifact of controller timing or of our recorder: the
64-box pile has no controllers at all, and every one of these worlds is bitwise
on the CPU solver.

### The mechanism, confirmed in source

mujoco_warp claims contact slots with `pairid = wp.atomic_add(ncollision_out, 0, 1)`
([`collision_driver.py:343`](https://github.com/google-deepmind/mujoco_warp)), so
**buffer order is thread-arrival order**; there are ~130 `atomic_add` sites
across its collision, constraint and solver modules. The fingerprint matches:
at step 0 of the dense scene only 37 robot *velocity* channels differ, by
1.9e-6 on values of ~5.6 — about 4 float32 ULP — with every position and every
box channel bit-identical; by step 2 the deviation is 0.92 m across 231
channels. That is a different summation order amplified by contact chaos, not a
different initial condition.

Scheduling is implicated directly: saturating the GPU with an unrelated CUDA
process moved the ring scene's spread from 4.1e-5…7.1e-5 to 1.39e-4…2.30e-4 —
roughly 3×, with **no overlap** against the quiet-GPU maximum.

### ⚠ A single-contact scene DID reproduce on the GPU — which sharpens the scope rather than contradicting it

**2026-08-09**, machine `9722d23d12a3`. Lane-1's T3 (one rolling sphere on a
plane — **one contact**) run three times cold on `mujoco_warp`, sidecar-confirmed
`solver: "MuJoCo (mujoco_warp, WorldInfo.newtonSolver)"` on every run, scored
**bit-identical** all three times (`roll_accel_rel_err` 0.0005358915299204134,
`slip_ratio` 0.00036659356341496287).

That is **not** a refutation of the 0-of-24 result above, and must not be quoted
as one. §2 establishes the mechanism as *contact buffer order is thread-arrival
order* (`pairid = wp.atomic_add(...)`), and every scene in that table carries
80–336 concurrent contacts. A scene with one contact has no ordering to get
wrong. The two results are consistent and together say something sharper than
either alone:

> `mujoco_warp`'s non-reproducibility is **contact-ordering** non-reproducibility.
> It needs contacts. Contact-rich scenes diverge (0 of 24 bitwise, up to 9.152 m
> by 1000 steps); a single-contact scene reproduced across 3 cold runs.

⚠ **n=3, one scene, one dt, one machine, 2.5 s of sim.** Do not generalise this
to "light GPU scenes are deterministic" — the honest external claim is still the
sentence in the header box, and this paragraph is a scope note on *why* it is
phrased around contact density.

### Newton's `deterministic=True` does not apply to us

Newton documents an opt-in `deterministic=True` on its `CollisionPipeline`
(default off) that fingerprint-sorts contacts. It is a **red herring on this
path**: `OmNewtonBackend` sets `_skip_collide = True` whenever the solver is
`SolverMuJoCo` with `_use_mujoco_contacts` (the default), so Newton's
`model.collide()` never runs and collision happens inside mujoco_warp. We
neither benefit from that flag nor are penalised for leaving it off.

## 3. ⚠️ `newtonNjmax` set to the measured peak is a trap

Setting `newtonNjmax` to exactly the measured peak `nefc` (320 for ten 4WD
robots) puts the buffer at capacity, rows are silently truncated, and results
move **8.81 m** versus every other buffer size — with run-to-run spread blowing
out to 1.71 m. 512, 2048 and 4096 agree with each other to within run noise
(6.7e-5 … 1.0e-4). Size the buffer with headroom, not to the measurement.

## 4. The false positive, and the three defects that produced it

A first pass reported "GPU physics is bitwise deterministic" — `max_abs_dev = 0.0`
over 120 steps on a ten-robot `mujoco_warp` scene. **The result was void.**

1. **The recorder skipped every node without a DEF name.** The robots are
   `URDFRobot { ... }` blocks with no DEF, so the graded CSV contained exactly
   one body — `SUN_MARKER`, parked at z = 100000, constant for 120 steps. Two
   identical tables of a static marker compared bitwise. The engine's own step
   trace, written into the *same directory*, showed the robots diverging 0.333 m
   by step 120.
2. **Nothing asserted that the recording moved.**
3. **`capture()` accepted any run with ≥ `steps // 2` rows** and graded it on the
   step intersection, so a run that died halfway passed as a full result.

Independently, that world could not have tested determinism anyway:
`husky_random` seeds itself `random.Random(abs(hash(robot.getName())))`, and
CPython randomises `str` hashing per process — three launches produced three
different seeds. The actuation was non-reproducible by construction.

All three are fixed (`e0316523`): undeffed bodies are recorded as
`IDX<i>_<type>` and the roster is logged; `diff()` returns a `no_motion` grade
carrying *"this run proves NOTHING about determinism"*; the short-run threshold
is 95%. Verified by replaying the original failing CSVs — they now grade
`no_motion` instead of `bitwise`.

## 5. The rule this cost us twice

Within one week this repo produced **two** green assertions that could not fail:
A1.3's robot-robot contact check (the engine reported `ContactPoint.node_id` as
the queried solid's own id, so pairs were always `(id, id)`), and this
determinism grade on a stationary object.

> **An assertion that has never gone red should be assumed broken until you make
> it go red on purpose.**

Practically: every check needs a negative fixture that targets *it
specifically*. A null agent that fails everything is not a negative fixture —
that is precisely how both of these hid. The lane-3 harness now ships the
`no_motion` guard; the graders ship falsifiability witnesses that report
`vacuous: witness absent` when the evidence needed to fail a clause is missing.

## 6. Reproducing this

```bash
# CPU MuJoCo path, contact-rich, expect bitwise
python tests/benchmarks/omnibench/lane3/determinism.py --backend newton \
  --world tests/benchmarks/omnibench/lane3/worlds/lane3_det_probe_dense_cpu.wbt \
  --dt-ms 32 --steps 120 --skip-coldwarm --out <out.jsonl>

# GPU path, same scene shape, expect divergence
python tests/benchmarks/omnibench/lane3/determinism.py --backend newton \
  --world tests/benchmarks/omnibench/lane3/worlds/lane3_determinism_gpu_fleet.wbt \
  --dt-ms 32 --steps 1000 --skip-coldwarm --out <out.jsonl>
```

Pin actuation before drawing conclusions (`PYTHONHASHSEED`, or a controller that
does not seed from `hash()`), and check the recorder's stderr roster line to
confirm the bodies you care about are actually being tracked.
