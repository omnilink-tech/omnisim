# OmniSim in context: a performance comparison with today's simulators and game engines

**2026-06-14.** OmniSim numbers are first-hand on this checkout (laptop RTX 3060) or
attributed to the project's recorded benchmarks. Competitor numbers are from published
vendor docs, peer-reviewed papers, and one independent academic benchmark, each cited
inline (see [References](#references)). Raw OmniSim data:
[measurements-2026-06-14.md](measurements-2026-06-14.md).

> **This paper covers *throughput* only.** For the non-speed axes — licence and cost,
> hardware floor, ROS integration, rendering class, agent-facing APIs, sim-to-real track
> record, and per-project maintenance status as of mid-2026 — see the capability companion
> [docs/developer/simulator-comparison.md](../developer/simulator-comparison.md)
> (2026-07-10). It also records what OmniSim *loses* on, which this paper only touches in §6.

---

## Abstract

OmniSim is an agent-native robotics simulator derived from Webots, finishing a
migration of its physics engine (ODE → **Newton**, GPU-batched, on NVIDIA Warp) and
its renderer (WREN/OpenGL → **wgpu**, cross-platform GPU). *[Note 2026-07-09: the
physics half of that migration has since landed as the default — `physicsBackend
"auto"` now resolves to Newton where the runtime is present, and a stock `make
release` bundles the runtime; see the dated note in §2.]* This paper places OmniSim's
measured performance beside the simulators and engines practitioners actually choose
between today: NVIDIA Isaac Gym / Isaac Lab / Isaac Sim, Gazebo, MuJoCo / MJX, Brax,
PyBullet, Webots, Genesis, and the game engines Unreal Engine 5 and Unity.

We measured two things first-hand on a modest 2021-class laptop GPU (RTX 3060 Laptop,
6 GiB): single-environment rigid-body step rate, and GPU-batched **physics** throughput.
We then compare — carefully, with caveats — against published competitor numbers.

> ⚠️ **We did NOT measure end-to-end RL throughput.** Our batched figure (§3.2) is a
> bare `mjw.step()` physics loop — no policy, no reward, no learning. Competitor
> "env-steps/s" headlines are usually **end-to-end RL**. Those are different units and
> this paper does not pretend otherwise (§3.2, §5.1).

The honest headline: **cross-simulator performance numbers are not directly
comparable**, and any single "FPS leaderboard" is misleading. Within those limits, the
result is that OmniSim's new stack moves it out of the "accurate but CPU-bound"
Webots/Gazebo tier and into the GPU-accelerated tier occupied by Isaac and MJX —
measured here at **~17–33× faster contact physics than its own ODE baseline**, **3–4
orders of magnitude more batched *physics* throughput than the legacy CPU pipeline**,
and a renderer that is **~2.4× cheaper per frame in GPU cost** (21.5 ms → 8–9 ms) —
though, honestly, **that render saving does not currently show up end-to-end**: on the
one scene we have measured, render-bound FPS actually *drops* 30.8 → ~25 (§3.3). All of
this while keeping Webots' breadth of robots/sensors/worlds and an agent-first
workflow. Notably, OmniSim's GPU path is built on **MuJoCo-Warp (Newton)** — the same
underlying technology Isaac Lab is itself migrating toward — so this is adoption of the
field's winning physics stack, not a fork off in a different direction.

---

## 1. Why this comparison is hard (read this first)

There is no honest single "simulator FPS leaderboard." The reasons:

1. **"A step" means different things.** A MuJoCo `mj_step`, a PhysX GPU substep, an ODE
   quickstep iteration, and an XPBD projection pass are not the same unit of work or
   accuracy. Steps/sec across engines is an order-of-magnitude indicator at best.
2. **The scene defines everything.** One free-floating ant ≠ eight quadrupeds colliding
   ≠ a city of 48 cars. Contact count, joint count, and mesh complexity dominate.
3. **Hardware spread is enormous.** Competitor headlines are typically on A100/H100,
   TPUv3, or RTX 4090. Our first-hand numbers are on a **laptop RTX 3060** — roughly an
   order of magnitude less compute than a 4090. We annotate hardware on every row.
4. **Physics vs rendering vs end-to-end.** Some numbers are physics-only, some
   render-bound, some full closed-loop. We label which.
5. **Marketing vs reproduced.** Vendor "N-million-FPS" numbers are best-case, batched,
   physics-only, on top-tier hardware. We separate first-party claims from independently
   reproduced figures, and flag contested ones (see Genesis, §4.8).

**Units never to mix in one column:** Isaac Gym/Lab "env-steps/sec" aggregates across
thousands of parallel envs; MJX "steps/sec" is batched physics throughput; Isaac Sim's
"physics steps/sec (Hz)" is a *single-scene* rate; and "real-time factor (RTF)" is
wall-clock-relative for one scene. A 1.1 M Isaac Lab Cartpole "FPS" and a Webots RTF of
1.7 are not the same measurement.

So this paper does not crown a winner. It positions OmniSim on the axes the field uses,
with every number carrying its hardware, its scene, and its caveat.

### 1.1 First-hand vs cited

- **First-hand (this machine, 2026-06-14):** OmniSim Newton single-env physics scaling;
  OmniSim GPU-batched MuJoCo-Warp **physics** throughput (§3.1–3.2). **Not** measured
  first-hand: end-to-end RL throughput (rollout + inference + learning) — see §3.2.
- **OmniSim project's own recorded benchmarks:** ODE baseline, wgpu-vs-WREN render cost,
  city real-time factor (§3.1, §3.3) — attributed to specific repo docs.
- **External published sources:** all competitor numbers (§4–§5), cited.

### 1.2 Test machine for first-hand OmniSim numbers

| | |
|---|---|
| CPU | AMD Ryzen 7 5800H (8C/16T) |
| GPU | NVIDIA RTX 3060 **Laptop**, 6 GiB, sm_86 |
| Stack | Warp 1.13.0, Newton 1.2.0, MuJoCo 3.8.1, CUDA 12.9 |
| Class | ~2021 mid-range gaming laptop — *not* a datacenter card |

---

## 2. What OmniSim is, architecturally

OmniSim is a fork of Webots (Apache-2.0 code; brand protected separately) re-engineered
around two modern, GPU-first backends. It is **mid-migration**: both new backends are
compiled into the default binary, but the stock runtime still defaults to the legacy
ODE+WREN path for byte-for-byte reproducibility, with the new backends opt-in per world
or via environment flags. This dual stack is the key to reading its numbers — the
"before" and "after" are the *same simulator on the same scenes*.

> **[Superseded 2026-07-09 — the physics default has since flipped.]** As of the v4
> engine defaults, `physicsBackend "auto"` resolves to **Newton** wherever its Python
> runtime (`newton`/`warp`) is reachable, and a stock `make release` **bundles the
> Newton runtime by default** (`BUNDLE_NEWTON ?= 1`); ODE is now the fallback, not the
> stock path (opt back in with `OMNISIM_LEGACY=1` / `OMNISIM_FORCE_ODE=1`). The
> main-view renderer remains WREN with wgpu opt-in. See [AGENTS.md](../../AGENTS.md)
> "Engine defaults (v4)". The paragraph above and all measurements in this paper are
> kept as written — they describe the 2026-06-14 snapshot.

| Layer | Legacy (default fallback) | Modern (migration target) |
|---|---|---|
| Physics | **ODE** — single-threaded CPU reference solver | **Newton** — GPU-batched on NVIDIA Warp; XPBD solver for speed, MuJoCo / MuJoCo-Warp solver for accurate contact & grasping |
| Rendering | **WREN** — in-house OpenGL renderer | **wgpu** — cross-platform GPU renderer (wgpu-native / Vulkan) |
| RL env stepping | CPU Webots envs (~30–150 env-steps/s) | GPU-batched MuJoCo-Warp / MJX (10⁵–10⁶ **physics** env-steps/s; §3.2 — this is the *physics* half of an RL step, not an end-to-end RL rate) |

The most honest OmniSim "benchmark" is therefore the **self-comparison**: same robots,
same worlds, old backend vs new.

---

## 3. OmniSim measured performance

All §3.1–3.2 numbers were measured on this checkout on 2026-06-14 on the laptop RTX
3060 (§1.2). Raw logs: [measurements-2026-06-14.md](measurements-2026-06-14.md).

### 3.1 Single-environment rigid-body physics (Newton XPBD) — first-hand

Husky robots (chassis + 4 wheels + 4 actuated revolute joints each), SolverXPBD, 10
iterations, dt = 1/60 s, 500 timed steps with per-step host readback of body pose.

| Huskies | Bodies | DOF | ms/step | physics fps | real-time factor* |
|--------:|-------:|----:|--------:|------------:|------------------:|
| 1  | 5  | 10  | 3.68 | 271 | 4.53× |
| 2  | 10 | 20  | 3.65 | 274 | 4.57× |
| 5  | 25 | 50  | 3.65 | 274 | 4.56× |
| 10 | 50 | 100 | 3.72 | 269 | 4.48× |

\* relative to a 60 Hz (16.67 ms) tick budget.

**Reading:** step time is essentially flat as body count grows 10× — near-perfect weak
scaling, because a GPU solver at this scale is dominated by fixed per-step overhead, not
body count. ~270 physics fps / ~4.5× real-time on a laptop GPU.

> **Reproducibility (2026-07-10).** The probe that produced this specific SolverXPBD table
> was later removed, so these four rows stand as a recorded 2026-06-14 snapshot. A
> rerunnable successor now exists — [`tests/benchmarks/newton_scaling_bench.py`](../../tests/benchmarks/newton_scaling_bench.py)
> — for the ms/step + real-time-factor quantity; it measures the MuJoCo-Warp solver path
> (not SolverXPBD) and reproduces the same *flat weak-scaling shape* (see Appendix A).

For contrast, OmniSim's **own ODE baseline** on contact-heavy husky scenes
(project benchmark, `archive/fps-optimization-journey.md`): a 2-husky head-on collision
runs at **10.2 fps (0.16× real-time)**; adding a 200-particle pool drops it to **1.3 fps
(0.02×)**. The project's recorded Newton-vs-ODE sweep (`engine-migration-plan.md` §13.5)
puts the per-robot speedup at **~17× (1 husky), ~33× (2 huskies), and effectively
unbounded at 10+ robots** — ODE fails to keep up while Newton holds 2.9–3.4 ms/step.

### 3.2 GPU-batched **physics** throughput (MuJoCo-Warp) — first-hand

> ### ⚠️ THIS IS NOT AN RL NUMBER. READ THIS BEFORE QUOTING IT.
> The measurement below is a **bare batched-physics stepping loop**. The script that
> produces it ([`mjwarp_throughput_poc.py`](../../projects/policies/research/tools/mjwarp_throughput_poc.py))
> calls `mjw.step()` in a timing loop and **nothing else**: it never writes `ctrl`,
> never runs a policy forward pass, never computes observations, rewards, advantages
> or a loss, and never does a backward pass. Its own docstring states its purpose —
> *"validates the GPU-trainer premise **before building the full PPO stack**."*
> An end-to-end RL step is strictly more expensive than this, so **this number is an
> upper bound on RL throughput, not a measurement of it.** It is a legitimate
> *physics* number; it is not a legitimate *RL* number, and it must not be set beside
> anyone else's end-to-end RL figure (§5.1 spells out why).
>
> *(An earlier revision of this paper titled this section "GPU-batched RL throughput"
> and put the figure head-to-head against Isaac Gym's end-to-end RL number. That was
> exactly the unit-mixing §1 forbids. Corrected 2026-07-11.)*

OmniSim's quadruped, exported to the *exact* MuJoCo model Newton simulates (zero
sim-to-sim gap), batched to N parallel worlds on the GPU, 200 timed steps of
`mjw.step()`. Model: **nq=19, nv=18, nu=24, nbody=14, ngeom=5**.

| Parallel envs | physics env-steps/sec (`mjw.step()` only — no policy, no learning) |
|--------------:|--------------:|
| 256   | 40,258  |
| 1,024 | 162,604 |
| 4,096 | 661,636 |

⚠️ **`ngeom=5` matters and is not a typo.** A 14-body quadruped carrying only **5
collision geoms** is a nearly contact-free model. Contact resolution is the dominant
cost in almost every real robotics workload, so this scene is *cheap*, and the
throughput above is correspondingly *optimistic* relative to a contact-rich scene. We
state it because it is the same weakness this paper criticises elsewhere (§4.8) —
being on the receiving end of it does not make it go away. (`ngeom=5` is in the raw
data: [measurements-2026-06-14.md](measurements-2026-06-14.md) — an earlier revision
of this section silently dropped it.)

**Reading:** near-linear scaling; the GPU is **not saturated** even at 4,096 envs (wall
time per 200 steps stays ~1.25 s while env count grows 16×). The legacy CPU Webots
pipeline does ~30–150 *env*-steps/s for 1–2 envs, so the GPU **physics** path is 3–4
orders of magnitude faster on the same laptop — a real and large gap, but a
physics-vs-physics one. On a datacenter GPU this scales much further (§4).

**What we have NOT measured here:** OmniSim's actual end-to-end RL throughput
(rollout + policy inference + learning). The project *does* have real in-engine RL
throughput figures — ~140–200 k env-steps/s on a laptop RTX 5070 Ti through the
in-engine trainer (see [AGENTS.md](../../AGENTS.md) and
[rl-current-state.md](../developer/rl-current-state.md)) — and note that these are
**~3–5× below** the physics-only ceiling above, which is precisely the point: the
policy and learning cost is real. Those are the numbers to use for any RL comparison;
they are not first-hand to this paper and are not benchmarked against competitors here.

### 3.3 Rendering (wgpu vs WREN) — project benchmark

From the project's recorded render benchmark (`wgpu-renderer-status.md`) on the city
baseline scene (1896×1113, 3,523 draw calls, RTX 3060):

| Metric | WREN (legacy) | wgpu (modern) |
|---|---:|---:|
| Render **cost** / frame (GPU only) | 21.5 ms (~46 fps render-only) | 8–9 ms (~110–125 fps render-only; **~2.4× cheaper**) |
| Whole-sim FPS (full traffic sim) | 14.0 | 14.3 (sim-bound tie) |
| Render-bound FPS (idle controllers) | **30.8** | **~25** ⛔ *slower* |

⚠️ **State both numbers, always.** The **~2.4×** is a *GPU-cost-per-frame* win and
nothing more. **End-to-end it does not (yet) pay off**: on the render-bound row of this
very table wgpu is **slower** (30.8 → ~25 FPS), and on the full traffic sim it is a tie
(14.0 → 14.3, physics/controller-bound). The saved GPU milliseconds are evidently being
given back on the CPU side of the frame. So:

- ✅ Defensible: *"wgpu cuts GPU render cost per frame ~2.4× (21.5 ms → 8–9 ms)."*
- ⛔ Not defensible: *"wgpu is ~2.4× faster."* Quoting the GPU-cost ratio as an
  end-to-end speedup contradicts the table directly above it.

An older "2.7× faster main view" headline exists in project material; it is the same
GPU-cost quantity and carries the same caveat. This is exactly the render-only
vs end-to-end trap §1 warns about — and we walked into it ourselves. wgpu remains
**opt-in** (`renderBackend "wgpu"`); WREN is still the default main-view renderer.

### 3.4 RL training wall-clock — project benchmark

- **Spot** (model + residual PPO, 50 k steps): ODE backend **52 s** and batched Warp
  ~80 k env-steps/s on an RTX 5070 (`spot-residual-rl.md`); Newton backend **84 s**
  (`rl-current-state.md`).
- **G1 humanoid stand** on this RTX 3060: verified **~27 k–62 k env-steps/s** (a "132 k
  on RTX 5070" figure exists in docs but is unverified on the documented hardware, so we
  cite the 3060-verified range).

---

## 4. The competitive landscape (cited)

Each entry gives the headline number, the hardware, the scene, the key caveat, and
whether it is vendor self-reported or independently reproduced.

### 4.1 NVIDIA Isaac Gym (the GPU-RL predecessor) — *vendor self-reported*

Peak end-to-end RL throughput on a **single A100 + i7-8700K** [1]:
- **Ant: up to ~700 K env-steps/s** (8,192 envs; "can go as high as"; sustained ~540 K)
- **Humanoid: ~200 K** (4,096 envs)
- **Shadow Hand cube reorientation: ~150 K** (16,384 agents)

Training wall-clock (same hardware): Ant reaches reward 3000 in ~20 s and converges
<2 min; Humanoid hits 5000 in <4 min. Shadow Hand trains in ~1 h (feed-forward) vs
OpenAI's original ~30 h on a 6,144-CPU-core cluster + 8 V100s [1]. **Caveat:** vendor
numbers, not independently reproduced; "700 K" is a peak.

### 4.2 NVIDIA Isaac Lab (the modern successor) — *vendor self-reported*

On a **single RTX 4090 + Ryzen 9 7950X** (headless) [2]:
- **Cartpole** (trivial): ~**1,100,000** env-step FPS @ 4,096 envs (910 K with
  inference, 510 K with training).
- **G1 humanoid locomotion** (`Isaac-Velocity-Rough-G1-v0`): ~**94,000** env-step FPS @
  4,096 envs (88 K / 82 K with inference / training). On an L40: 72 K / 64 K / 62 K.

**Caveat:** vendor figures; absolute throughput is hardware-tier sensitive; Isaac Lab is
itself migrating toward **Newton** physics, so these will shift.

### 4.3 NVIDIA Isaac Sim — *vendor self-reported (single-scene, not RL)*

Isaac Sim 4.5.0's published benchmark page reports **single-scene physics step rates**,
not batched RL: the O3dyn sample tops out at **~636.9 Hz on an RTX 4080**; consumer GPUs
land ~570–637 Hz, while *datacenter* GPUs are counterintuitively lower (L40 ~135 Hz
single-GPU) [3]. The page publishes **no** RL throughput, parallel-env counts, or RTF —
RL-scale Isaac numbers must come from Isaac Lab/Gym (§4.1–4.2), and Isaac Sim's headline
strength is its photoreal **RTX renderer**, not step rate.

### 4.4 MuJoCo and MJX — *vendor docs (primary)*

MuJoCo's own docs report, for a single humanoid [4]:
- **MJX (GPU): ~950 K steps/s** on an A100 (batch 8,192); **2.7 M steps/s** on an
  8-chip v5 TPU (batch 16,384).
- **CPU MuJoCo (same model): 650 K steps/s** (Apple M3 Max); **1.8 M** (64-core AMD
  3995WX).

**Caveat (developer-disclosed):** "MJX works best simulating thousands to tens of
thousands of scenes in parallel"; for a *single* scene, MJX-JAX can be **~10× slower**
than CPU-optimized MuJoCo [4]. (A separately circulated "MJX-Warp 3.35 M steps/s" figure
did not survive verification and is **not** cited here.)

### 4.5 Brax — *vendor blog + paper (primary)*

Brax (JAX, GPU/TPU) scales **linearly to ~10,000 envs on a single TPU** before hitting
memory-bandwidth limits, and reaches **hundreds of millions of physics steps/s on a
TPUv3 8×8 (64 chips)** [5]. The Ant task trains in **~10 s on accelerator hardware vs
~3 h** on a traditional GPU+CPU setup; the authors claim **100–1000× faster training**
overall [5]. **Caveat:** the eye-popping "hundreds of millions" needs 64 TPU chips;
single-device throughput is in the millions, like MJX.

### 4.6 PyBullet — *independent academic benchmark*

In the independent benchmark [6] (i7-8700, 1 ms timestep), PyBullet's **real-time factor
on a UR10e robot scene was 0.8** (slower than real time) — attributed to its default
cone-friction model — but **1.3 on a 216-sphere contact scene** (competitive). PyBullet
ran **single-threaded, CPU-only**. (No standalone vendor throughput benchmark survived
verification.)

### 4.7 Webots (the parent project) — *independent academic benchmark*

Same benchmark [6]: Webots' **RTF was 1.7 on the UR10e scene** and **1.2 on the
216-sphere scene**, with **multi-threaded CPU** physics. This is the most direct read on
OmniSim's *inherited* baseline — and precisely what the Newton migration replaces. (The
Webots vendor "speed & performance" guide [7] confirms the threading/timestep levers but
gives no portable FPS number.)

### 4.8 Genesis — *contested first-party marketing*

Genesis's launch headline was **"43 million FPS / 430,000× real-time"** for a Franka-arm
scene (Zhou Xian, Dec 2024) [8]. An independent reproduction (Stone Tao, ManiSkill
author) and GitHub issue #181 attribute the figure to **inflating methodology**:
substeps=1 (vs the 2–4 used elsewhere), **one action then ~999 no-op idle steps**, and
**disabled self-collisions** by default [9]. The exact figures are **not in the public
benchmark repo** (only harness scripts that compute FPS at runtime) [10]. The corrected
throughput is itself **contested** — Tao's RTX 4090 retest ~**0.29 M FPS** vs the Genesis
team's revised ~**27 M FPS** (with self-collisions + random actions). **Verdict:** the
*direction* of the critique is well-supported; the corrected magnitude is a disputed
range — do not cite any single Genesis number as settled.

### 4.9 Unreal Engine 5 — *vendor tech blog (single-world, real-time)*

UE5 is a real-time game renderer, not a batched-physics RL trainer; its physics
(**Chaos**, which replaced PhysX) is single-world and real-time. Epic's own UE5.0
benchmark vs PhysX 3 [11]:

| Chaos scene | PhysX 3 | Chaos (UE5.0) |
|---|---:|---:|
| Tumbler, 512 dynamic convex objects | 3.17 ms | 4.76 ms |
| 64 ragdolls onto a plane | 5.30 ms | 8.18 ms |
| Terrain, 100 k static + 512 dynamic | 10.82 ms | **5.34 ms** (Chaos faster) |

Early UE4.26 reports of "Chaos ~22× slower than PhysX for rigid bodies" [12] were an
early-development artifact; UE5.0 narrowed the gap and wins on landscape-heavy scenes.
**Caveat:** these are single-world frame-time numbers (~3–10 ms ≈ 100–300 physics fps on
those scenes); UE5's value is photoreal real-time rendering (Nanite/Lumen, typically
targeting 30–120 fps at 1080p–4K on consumer GPUs), not RL throughput.

### 4.10 Unity / Unity ML-Agents — *vendor blog + qualitative survey*

Unity ML-Agents' headline is **"train ~7× faster with concurrent environments"** [13] —
CPU-parallel envs, not GPU-batched. A 2024 academic survey [14] judges Unity the **most
user-friendly** engine but with **poor performance / scalability** for large-scale
training (not optimized for parallel/GPU compute). **Caveat:** the survey is qualitative
(it relied partly on vendor claims), and no measured Unity RL-throughput or rendering-FPS
figure survived verification — treat Unity as the usability-leading, throughput-lagging
option.

---

## 5. Head-to-head

Each column carries its hardware and caveat — per §1 the numbers are otherwise not
comparable. OmniSim rows are first-hand (§3) or attributed; competitor rows cite §4.

### 5.1 GPU-batched throughput — ⚠️ **the rows below DO NOT MEASURE THE SAME THING**

> ### ⛔ DO NOT READ THIS AS A LEADERBOARD.
> **Our row is physics-only. Most of the others are end-to-end RL.** Concretely:
> Isaac Gym's ~700 K is described by its own vendor source (and by §4.1 of this paper)
> as *"peak **end-to-end RL** throughput"* — rollout **plus** policy inference **plus**
> training. OmniSim's 661,636 is a bare `mjw.step()` loop with **no policy and no
> learning at all** (§3.2). Those two numbers are **not comparable**, and ranking them
> in one sorted column — as an earlier revision of this table did — is precisely the
> unit-mixing this paper's own §1 forbids (*"Units never to mix in one column"*).
> The table is therefore **split by unit** and **not sorted into a single ranking**.
> Also mind the ~10× hardware spread, and that our model is a **contact-light**
> 14-body / **5-geom** quadruped (§3.2) — cheaper than the humanoids in the RL rows.

**Group A — end-to-end RL throughput** (rollout + inference + learning). *OmniSim has
no first-hand entry in this group; we did not measure it.*

| Stack | Robot / task | Envs | env-steps/sec | GPU/TPU | Type | Src |
|---|---|---:|---:|---|---|---|
| Isaac Lab | Cartpole (trivial) | 4,096 | ~1,100,000 | RTX 4090 | vendor | [2] |
| Isaac Gym | Ant | 8,192 | ~700,000 (peak) | A100 | vendor | [1] |
| Isaac Gym | Humanoid | 4,096 | ~200,000 | A100 | vendor | [1] |
| Isaac Lab | G1 humanoid | 4,096 | ~94,000 | RTX 4090 | vendor | [2] |
| **OmniSim** | — | — | **not measured** | — | — | — |

**Group B — batched *physics* stepping throughput** (no policy, no learning). This is
the only group OmniSim has a first-hand number in.

| Stack | Robot / task | Envs | physics env-steps/sec | GPU/TPU | Type | Src |
|---|---|---:|---:|---|---|---|
| MuJoCo MJX | humanoid | 16,384 | 2,700,000 | v5 TPU (8-chip) | vendor | [4] |
| MuJoCo MJX | humanoid | 8,192 | 950,000 | A100 | vendor | [4] |
| **OmniSim (Newton/MuJoCo-Warp)** | **quadruped (14 body, 5 geom)** | **4,096** | **661,636** | **RTX 3060 Laptop** | **first-hand** | §3.2 |
| **OmniSim (Newton/MuJoCo-Warp)** | **quadruped (14 body, 5 geom)** | **1,024** | **162,604** | **RTX 3060 Laptop** | **first-hand** | §3.2 |
| Brax | locomotion | (scales to 64 chips) | hundreds of millions | TPUv3 8×8 | vendor | [5] |

**How to read this honestly.** The only *like-for-like* comparison available to us is
**Group B**, where OmniSim's 662 K on a *laptop 3060* sits within an order of magnitude
of MJX on an A100 — largely because OmniSim's path **is** MuJoCo-Warp, the same engine
family as MJX, and because our contact-light 14-body/5-geom quadruped is a cheaper model
than their humanoid. The fair statement: *for comparable model complexity, OmniSim's GPU
**physics** path delivers throughput in the same league as MJX, on markedly weaker
hardware, because it runs the same class of GPU physics.* It does **not** beat A100/TPU
systems in absolute terms once you put OmniSim on the same card, and we make **no**
first-hand claim at all against the end-to-end RL numbers in Group A.

### 5.2 Single-environment / real-time-factor (the classical CPU axis)

Where Gazebo/Webots/PyBullet/MuJoCo-CPU live. RTF rows from the independent 2021
benchmark [6] (i7-8700, 1 ms, UR10e scene unless noted).

| Stack | Scene | Step rate / RTF | Hardware | Type | Src |
|---|---|---:|---|---|---|
| **OmniSim (Newton XPBD)** | 1 husky, 5 bodies | 271 fps / **4.5× RT** ⚠️ *2026-06-14 snapshot from a since-deleted probe — see the reproducibility note in §3.1. The rerunnable successor (MuJoCo-Warp path, RTX 5070 Ti) reports ~180 fps / **~3.0× RT**.* | RTX 3060 Laptop | first-hand | §3.1 |
| Gazebo (ODE) | UR10e | **4.4× RT** | i7-8700 (CPU) | independent | [6] |
| MuJoCo (CPU) | UR10e | 2.8× RT | i7-8700 (CPU) | independent | [6] |
| Webots | UR10e | 1.7× RT | i7-8700 (CPU) | independent | [6] |
| **OmniSim (ODE, legacy)** | 2-husky head-on (contact) | **0.16× RT** (10.2 fps) | (project bench) | project | §3.1 |
| PyBullet | UR10e | 0.8× RT | i7-8700 (CPU) | independent | [6] |
| Isaac Sim 4.5 | O3dyn (single scene) | ~637 Hz | RTX 4080 | vendor | [3] |

(Note the legacy-ODE row is a *contact-heavy collision* scene, harder than the UR10e
arm; it is shown to anchor the 17–33× Newton speedup, not as a like-for-like RTF.)

### 5.3 Rendering

| Engine | Scene | Frame cost | Hardware | Src |
|---|---|---:|---|---|
| **OmniSim (wgpu)** | city, 3,523 draws @1896×1113 | **8–9 ms** (~110–125 fps) | RTX 3060 | §3.3 |
| **OmniSim (WREN, legacy)** | same | 21.5 ms (~46 fps) | RTX 3060 | §3.3 |
| Unreal Engine 5 (Chaos) | 512 dynamic convex (physics only) | 4.76 ms | (Epic bench) | [11] |
| Isaac Sim | RTX path-traced photoreal | (no portable FPS) | RTX class | [3] |

OmniSim's wgpu renderer is real-time-competitive for robotics visualization
(~110+ fps on a busy city scene). It is **not** photoreal like Isaac Sim's RTX renderer
or a fully lit UE5 scene — that is an explicit, acknowledged gap (§6).

### 5.4 Qualitative axes (not raw speed)

| Axis | OmniSim | Isaac Sim/Lab | Gazebo | MuJoCo/MJX | Unreal/Unity |
|---|---|---|---|---|---|
| Robot/sensor breadth | High (Webots base) | High | High | Medium | Low (build-your-own) |
| GPU-batched RL | Yes (Newton/MJX-Warp) | Yes | No | Yes (MJX) | Partial (CPU-parallel) |
| Photoreal rendering | Improving (wgpu) | Yes (RTX) | Limited | None built-in | Yes |
| Hardware floor | **Laptop GPU** | High-end NVIDIA | CPU | Laptop GPU+ | Mid GPU |
| Agent-native workflow | Designed for it | No | No | No | No |
| License | Apache-2.0 | Proprietary/EULA | Apache-2.0 | Apache-2.0 | Proprietary |
| Maturity of fast path | Young (mid-migration) | Mature | Mature | Mature | Mature |

---

## 6. Analysis: where OmniSim actually sits

**The migration is the story.** OmniSim started as Webots — an RTF-1.7, CPU,
multi-threaded simulator [6]. By adopting Newton (XPBD + MuJoCo-Warp) it leapfrogs its
own ODE/Webots-tier physics into the GPU-batched tier: **~17–33× faster contact physics**
than its ODE baseline, and **3–4 orders of magnitude more batched *physics* throughput**
by moving from CPU Webots envs (~30–150 env-steps/s) to GPU batches (662 K **physics**
env-steps/s on a laptop, on a contact-light 14-body/5-geom model — §3.2). We have **not**
measured OmniSim's end-to-end RL throughput, so no claim is made on that axis here.

**Where OmniSim wins (honestly):**
- **Hardware floor.** It runs the GPU-batched **physics** path on a *laptop RTX 3060*,
  reaching 662 K physics env-steps/s for a quadruped. The comparable vendor headlines
  assume A100/4090/TPU — though most of those are *end-to-end RL*, a strictly heavier
  unit than ours (§5.1). Accessibility on commodity hardware is nonetheless a real,
  measured advantage.
- **Same winning engine, more breadth.** Its fast path is MuJoCo-Warp/Newton — the
  direction Isaac Lab is *also* migrating to [2] — but wrapped in Webots' large
  robot/sensor/world library and an Apache-2.0 license.
- **Self-consistent sim→train→deploy.** The throughput PoC steps the *exact*
  MuJoCo model the deploy backend simulates (zero sim-to-sim gap, §3.2). (It does not
  itself train — see the §3.2 warning.)
- **Agent-native authoring** — designed to be driven by AI coding agents — which the
  incumbent simulators are not.

**Where OmniSim loses (honestly):**
- **Absolute top-end throughput** belongs to datacenter hardware. Put OmniSim on an
  A100 and it would be MJX-class, not magically faster — and MJX/Isaac on TPU/H100 reach
  millions of env-steps/s we cannot match on a laptop.
- **Photorealism.** Isaac Sim's RTX renderer and UE5's Nanite/Lumen are in a different
  visual class; OmniSim's wgpu is fast and improving but single-sun, no path-tracing (§5.3).
- **Maturity & coverage.** Newton is mid-migration — the project's own notes record only
  ~5/8 robot showcase worlds as fully faithful under Newton today, with ODE+WREN still the
  shipped default. Isaac/MuJoCo/Gazebo are battle-tested over many years.
- **Not a contender for raw game rendering.** UE5/Unity win that outright; OmniSim is a
  *robotics* simulator that renders well enough, not a game engine.

**Where the comparison is a category error:** Genesis's contested 43 M-FPS-style claims
(§4.8) show why marketing FPS numbers are dangerous; we deliberately avoid that framing
for OmniSim and report only measured, reproducible figures with hardware attached.

### 6.1 Is OmniSim the fastest simulator? (the question everyone asks)

**No — and it is important to say so plainly.** In absolute throughput, MuJoCo MJX
(2.7 M steps/s on a TPU pod), NVIDIA Isaac Lab / Isaac Gym (700 K–1.1 M env-steps/s on
A100/RTX 4090 — *end-to-end RL*, a heavier unit than ours), and Brax (hundreds of
millions on a 64-chip TPU) all exceed the 661,636 **physics** env-steps/s we measured.
Anyone claiming OmniSim is "the fastest simulator" would be refuted in five minutes with
a single A100 benchmark — and anyone comparing our physics-only figure to an end-to-end
RL figure is not measuring the same thing in the first place (§5.1).

But the rank understates the result, for two concrete reasons:

1. **Hardware.** Our 662 K was on a *laptop RTX 3060* — roughly an order of magnitude
   less compute than a 4090, and far less than an A100 or a TPU pod. Normalized for the
   card, OmniSim sits squarely in the top tier, not behind it.
2. **Same engine.** OmniSim's fast path *is* MuJoCo-Warp / Newton — the same technology
   as MJX, and the direction Isaac Lab is itself migrating toward [2][4]. We are not
   slower because of inferior tech; we run the field's winning solver.

The honest corollaries cut both ways:

- **On equal hardware, OmniSim does not beat MJX or Isaac** — put it on an A100 and it
  would be MJX-class, because under the hood it *is* that class of solver. There is no
  secret sauce that makes the same engine faster inside OmniSim.
- **Our test robot (a 14-body quadruped) is simpler than a humanoid**, which flatters
  the 662 K relative to Isaac Lab's 94 K G1-humanoid figure. A like-for-like humanoid
  comparison would narrow the gap.

So the defensible claim is **not** "fastest simulator." It is:

> **Top-tier GPU-accelerated robotics simulation speed on the hardware practitioners
> actually own — built on the best available physics engine, wrapped in Webots'
> breadth, an Apache-2.0 license, and an agent-native workflow.**

That claim survives an adversarial reading. "Best performing" does not, and OmniSim's
credibility is worth more than the headline.

---

## 7. Conclusion

On a modest 2021 laptop GPU, OmniSim's new stack delivers **flat weak scaling in
single-scene physics** and **661,636 batched *physics* env-steps/s at 4,096 parallel
quadrupeds** — moving it from the CPU-bound Webots/Gazebo tier into the GPU-accelerated
tier of Isaac Lab and MJX, on hardware an order of magnitude cheaper than those systems'
reference machines. It does this by adopting MuJoCo-Warp / Newton — the same physics
direction the leading commercial stack is migrating toward — while retaining Webots'
breadth, an Apache-2.0 license, and an agent-first workflow.

Three honesty notes that belong in the conclusion, not buried:

- **The 661,636 is physics, not RL.** No policy, no reward, no learning is in that loop
  (§3.2). It is an *upper bound* on RL throughput, and it is measured on a
  **contact-light** model (nbody=14, **ngeom=5**). We make no first-hand end-to-end RL
  throughput claim against anyone.
- **The renderer's ~2.4× is a GPU-cost win only.** End-to-end it has not paid off yet —
  the one scene measured shows render-bound FPS *falling* 30.8 → ~25 (§3.3). wgpu is
  opt-in; WREN remains the default.
- **The single-scene physics-fps figure is dated.** The original §3.1 SolverXPBD table
  (a ~271 fps / ~4.5× real-time snapshot, 2026-06-14) came from a probe that has since
  been **deleted**, so it is not rerunnable. Its **rerunnable successor** measures the
  MuJoCo-Warp path and reports **~5.5 ms/step, ~180 physics-fps, ~3.0× real-time** on a
  *better* GPU (RTX 5070 Ti Laptop) — different solver, different card, so not a
  refutation, but it is the live number and the one to quote. The **flat weak-scaling
  shape** — the actual finding — reproduces on both.

OmniSim is **not** the fastest simulator in absolute terms; that title needs datacenter
GPUs and belongs to MJX/Isaac/Brax on A100/H100/TPU. Its measured, defensible claim is
narrower and more useful: **competitive GPU-accelerated robotics simulation on the
hardware practitioners actually own, built on the field's winning physics engine, with
the breadth and openness of Webots.** Every number here carries its hardware — which,
in a field full of unqualified FPS headlines, is the point. Reproduction pointers are in
Appendix A; note that the original physics-scaling probe has since been retired (see the
2026-07-09 note there), so the §3.1 scaling numbers stand as a recorded 2026-06-14
measurement rather than a rerunnable script.

---

## Appendix A — reproducing the first-hand numbers

- Physics scaling: originally `python scripts/xpbd_probes/bench_newton_scaling.py` —
  **[2026-07-09: that probe was removed** (the removal is recorded in
  [engine-migration-plan.md](../developer/engine-migration-plan.md)); the §3.1 numbers
  remain as a recorded 2026-06-14 SolverXPBD measurement. **[2026-07-10: a rerunnable
  successor now exists](../../tests/benchmarks/newton_scaling_bench.py)** —
  `python tests/benchmarks/newton_scaling_bench.py --worlds 1 2 5 10` — so the
  step-time / real-time-factor quantity is no longer an unreproducible headline. It
  reuses the proven throughput-PoC machinery (build → export the exact MuJoCo model →
  step under mujoco_warp), so it measures the **MuJoCo-Warp** solver step time in the
  §3.1 *units* (ms/step, physics-fps, RTF), NOT the retired SolverXPBD path — treat it
  as the living home for that quantity going forward, not a re-measurement of the XPBD
  figures. First run of the successor (2026-07-10, **RTX 5070 Ti Laptop**, 14-body
  quad, mujoco_warp): **~5.5 ms/step holding flat from 1→10 worlds** (182 → 180
  phys-fps, ~3.0× RT) — the same near-perfect weak-scaling *shape* §3.1 reports, on a
  different card and solver. The other surviving harness,
  [`tests/benchmarks/optim_bench.py`](../../tests/benchmarks/optim_bench.py)
  (subcommands `many-robots`, `many-cameras`, `multi-instance`, `all`, `compare`, …),
  e.g. `python tests/benchmarks/optim_bench.py multi-instance --sizes 4 --steps 600`,
  benchmarks whole `omnisim-bin` instances, not the raw solver loop, so it is a
  coarser successor.**]**
- GPU batched-**physics** throughput (§3.2 — *not* an RL number):
  `python projects/policies/research/tools/mjwarp_throughput_poc.py`
  ⚠️ **Two preconditions, or it will not run as written:**
  1. The script hard-codes its MJCF export path to `C:\tmp\quad_newton_export.xml`, so
     it is **Windows-only** and **`C:\tmp` must already exist** (`mkdir C:\tmp`). It
     does not create the directory.
  2. It still prepends the pre-rename `projects/rl/tools` to `sys.path`. That directory
     no longer exists (it is now `projects/policies/research/tools`); the stale entry is
     harmless *only* because the script's own directory is already on `sys.path` and its
     `newton_friction_probe` dependency is a sibling there — so **run it by path, from
     the repo root, exactly as above**, and do not copy it elsewhere.

  (Both are defects in the script, not in the measurement. The script lives under
  `projects/policies/` and is out of scope for this doc's edit; fixing them is tracked
  separately.)
- Both require the Warp/Newton/MuJoCo runtime; run from PowerShell so the embedded
  interpreter resolves the user-site `warp` package (see the project's Newton-verification
  notes). Numbers vary with GPU; ours are a laptop RTX 3060 (§1.2).

---

## References

[1] Makoviychuk et al., *Isaac Gym: High Performance GPU-Based Physics Simulation for
Robot Learning*, NeurIPS 2021 D&B. https://arxiv.org/pdf/2108.10470
[2] NVIDIA, *Isaac Lab — RL Performance Benchmarks* (RTX 4090 table). https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/performance_benchmarks.html
[3] NVIDIA, *Isaac Sim 4.5.0 Benchmarks* (single-scene physics step rates). https://docs.isaacsim.omniverse.nvidia.com/4.5.0/reference_material/benchmarks.html
[4] DeepMind/Google, *MuJoCo MJX documentation* (steps/sec figures). https://mujoco.readthedocs.io/en/stable/mjx.html
[5] Freeman et al., *Brax* (paper + Google Research blog). https://research.google/blog/speeding-up-reinforcement-learning-with-a-new-physics-simulation-engine/ ; https://arxiv.org/abs/2106.13281
[6] Körber et al., *Comparing Popular Simulation Environments in the Scope of Robotics and RL*, 2021 (independent; public repro repo). https://arxiv.org/pdf/2103.04616 ; https://github.com/zal/simenvbenchmark
[7] Cyberbotics, *Webots — Speed/Performance*. https://cyberbotics.com/doc/guide/speed-performance
[8] The Decoder, *Genesis … 430,000× faster than reality*. https://the-decoder.com/genesis-speeds-up-ai-robot-training-with-simulations-430000x-faster-than-reality/
[9] Genesis GitHub issue #181 (methodology critique); Stone Tao reproduction. https://github.com/Genesis-Embodied-AI/Genesis/issues/181 ; https://stoneztao.substack.com/p/the-new-hyped-genesis-simulator-is
[10] Genesis speed-benchmark repo (no results files, harness only). https://github.com/zhouxian/genesis-speed-benchmark
[11] Epic Games, *Chaos Scene Queries and Rigid Body Engine in UE5* (Chaos vs PhysX 3 timings). https://www.unrealengine.com/en-US/tech-blog/chaos-scene-queries-and-rigid-body-engine-in-ue5
[12] Epic Developer Community forum, *Chaos 22× slower than PhysX for rigid bodies?* (early UE4.26). https://forums.unrealengine.com/t/chaos-22-times-slower-than-physx-for-rigid-bodies/155843
[13] Unity, *Training your agents 7× faster with ML-Agents*. https://unity.com/blog/engine-platform/training-your-agents-7-times-faster-with-ml-agents
[14] Kaup/Wolff et al., *A Review of Nine Physics Engines for RL Research*, 2024 (qualitative survey). https://arxiv.org/abs/2407.08590

*Source-type note: only [6] (Gazebo/MuJoCo/PyBullet/Webots RTF) and the Stone Tao
reproduction in [9] are independent measurements; Isaac Gym/Lab/Sim, MJX, Brax, Genesis,
UE5, and Unity figures are first-party vendor/author-reported and labeled as such above.*
