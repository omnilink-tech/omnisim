# OmniSim in context: a capability comparison with today's robotics simulators

**2026-07-10.** This is the *capability and positioning* companion to
[performance-comparison.md](../benchmarks/performance-comparison.md) (2026-06-14), which
covers **throughput** — steps/sec, env-steps/sec, render frame cost. Read that one for
"how fast"; read this one for "what it is, what it costs, what it can't do."

Nothing here restates a number from the performance paper. Where speed matters to a
capability claim, this doc links across rather than re-deriving.

---

## 0. How to read this

Cross-simulator comparison is adversarial territory: vendors publish best-case numbers,
listicles copy each other, and licences change between releases.

> ⚠️ **Read this asymmetry before you read the matrix.** The evidence markers below are
> applied to the **competitor cells only**. The **OmniSim column carries no markers at
> all** — its cells are **self-attested**: checked by us against this checkout, by us,
> with no external primary source and no independent audit. That is a *weaker* class of
> evidence than the ✅ we award a competitor for a fetched vendor doc, not a stronger
> one. We are not going to dress up "we vouch for ourselves" as verification. Treat
> every OmniSim cell as **⊘ self-attested (this checkout, 2026-07-10)** and check it
> yourself — the repo is right there, and §4 lists what OmniSim loses on.

| Marker | Meaning |
|---|---|
| ✅ | **Verified against a primary source** (official docs, release page, or repo) — either by 3-of-3 adversarial verification (the Newton/RL cluster) or by direct fetch of the cited source in the 2026-07-10 completion pass (the licence/maintenance/hardware/ROS rows). *Competitor cells only.* |
| ◐ | **Primary-source extraction, not audited.** Pulled with a direct quote from an official doc/repo, but neither verification path was run on it. Treat as *probably true, not audited.* *Competitor cells only.* |
| ⚠️ | **Vendor claim or contested.** First-party marketing, an unstated baseline, or an actively disputed figure. |
| — | Not established. We looked and did not find a citable answer. |
| *(no marker, OmniSim column)* | **⊘ Self-attested against this checkout.** Not independently verified. This is the weakest evidence class in the table, and it is the class every one of our own cells is in. |

Two structural warnings, carried over from
[performance-comparison.md §1](../benchmarks/performance-comparison.md#1-why-this-comparison-is-hard-read-this-first):

1. **"A step" is not a portable unit** across engines, and neither is "FPS."
2. **The scene defines everything.** A free-floating ant is not eight colliding quadrupeds.

And one specific to *this* doc: **licence and maintenance facts rot fast.** Every claim
below is date-stamped to when it was fetched. Isaac Sim went from proprietary EULA to
Apache-2.0 within this document's memory; Newton went from announcement to Linux
Foundation to 1.0 GA in eighteen months. Re-check before quoting externally.

---

## 1. The comparison matrix

Competitor cells carry evidence markers and are cited in §3. **OmniSim cells carry no
markers — they are self-attested against this checkout** (see the warning in §0); read
them as claims to be checked, not as verified findings.

| | **OmniSim** | Isaac Sim / Isaac Lab | Gazebo (gz-sim) | MuJoCo (+MJX/MJWarp) | Upstream Webots | Genesis | ManiSkill 3 / SAPIEN | PyBullet | CoppeliaSim | Unity / Colosseum |
|---|---|---|---|---|---|---|---|---|---|---|
| **Licence** | Apache-2.0 | Apache-2.0 ✅ (Sim) · BSD-3 ✅ (Lab) | Apache-2.0 | Apache-2.0 | Apache-2.0 | Apache-2.0 ✅ | Apache-2.0 code, **CC BY-NC assets** ✅ | Zlib | **Paid commercial** ✅ | Proprietary / MIT (Colosseum) ◐ |
| **Cost to ship a product** | Free | Free | Free | Free | Free | Free | Free (code); assets non-commercial ✅ | Free | **Contact vendor** ✅ | Unity licence / free |
| **Default physics** | Newton (MuJoCo-Warp / XPBD); ODE fallback | PhysX 5; Newton **experimental** ✅ | DART; Bullet pluggable ◐ | MuJoCo | ODE | multi-solver (rigid/FEM/MPM/PBD) ◐ | SAPIEN/PhysX | Bullet | Bullet/ODE/Vortex/Newton | PhysX / Chaos |
| **GPU physics** | Yes (Warp) | Yes | No | Yes (MJX/MJWarp) | No | Yes ◐ | Yes ◐ | No | No | No |
| **Batched RL in-engine** | Yes | Yes (Isaac Lab) | No | Yes | No | Yes ◐ | Yes ◐ | No | No | CPU-parallel only |
| **Hardware floor** | Laptop GPU; **CPU-only via ODE** | **RTX 4080 + 16 GB VRAM** ✅; A100/H100 **unsupported** ✅ | CPU | Laptop GPU (CPU MuJoCo fine) | CPU | CUDA / ROCm / Metal ✅ | NVIDIA GPU | CPU | CPU | Mid GPU |
| **Photoreal rendering** | No (WREN; wgpu opt-in) | **Yes — RTX path tracing** | Limited | None built-in | Limited | Ray tracer (Luisa) ◐ | Rasterized, GPU-parallel ◐ | Minimal | Limited | **Yes** |
| **Native URDF import** | Yes (`URDFRobot` node) | Yes | Yes (via SDF) | Yes | Yes | Yes | Yes | Yes | Yes | Yes (Unity importer) ◐ |
| **First-party ROS 2** | **No** — assets only | Bridge ✅; `ros2_control` *community* ✅ | **Yes — ros-controls hosted** ✅ | `ros2_control` **hosted** ✅ | `webots_ros2` (community `ros2_control`) ✅ | No ✅ | No ✅ | No ✅ | No ✅ | Community-grade ◐ |
| **Agent-facing HTTP authoring API** | **Yes — first-party** | Third-party MCP only ◐ | Third-party MCP only ◐ | No | No | No | No | No | No | No |
| **Windows** | **Primary** | Supported ✅ (Win 11) | Weak | Yes | Yes ✅ | Yes ✅ | — | Yes | Yes | Yes |
| **Maintenance (mid-2026)** | Active | Active ✅ (Sim v6.0.1, Jun 2026) | Active ✅ (Jetty LTS) | Active | Active ✅ (R2025a) | Active ✅ (v1.2.1, Jul 2026) | Active ✅ (v3.0.1, Apr 2026) | **Maintenance mode** ✅ | Active | **Unity Robotics dormant** ◐ |

---

## 2. Where OmniSim is actually differentiated

Three claims survive an adversarial reading. Everything else about OmniSim is either
inherited from Webots or matched by someone else.

### 2.1 A first-party, agent-facing HTTP authoring loop

This is the strongest claim, and the research supports it more cleanly than expected.

Agent-driven simulation authoring **does** exist in the wild — but everywhere else it is
**third-party glue bolted onto a simulator that was not designed for it**:

- `omni-mcp/isaac-sim-mcp` — an MCP server for Isaac Sim. MIT, ~180 stars, requires Isaac
  Sim 4.2+, assumes the Cursor editor as MCP client; latest release v0.3.0, April 2025 ◐.
- `kvgork/gazebo-mcp` — a community MCP server letting an assistant drive Gazebo ◐.

Neither is a vendor feature. No established simulator ships a **first-party** HTTP surface
built for the coding-agent iteration loop — `POST /world/load` → `GET /scene/tree` →
`GET /world/render_stats` → `POST /world/screenshot` → edit → hot-reload in seconds without
relaunching the simulator.

OmniSim ships exactly that, versioned and specified: [PROTOCOL.md](../../PROTOCOL.md)
defines four independently-versioned surfaces (Robot Bridge `:8765`, World Harness `:6789`,
Capture `:6791`, Twin Shadow), with structured load diagnostics (`PROTO_NOT_FOUND`,
`WORLD_PARSE_SYNTAX_ERROR`) an agent can branch on instead of regex-matching stderr, and a
unified `/sim/events` stream. It also meets the ecosystem on its own terms: a first-party
**MCP server** ([`packages/omnisim-mcp/`](../../packages/omnisim-mcp/)) exposes that harness
to Claude Desktop, Cursor, and any MCP client — so where competitors have *third-party* MCP
servers wrapping a non-agent-native sim, OmniSim has a *first-party* one over a surface built
for the job.

**The honest caveat:** "nobody else ships this" is an *absence of evidence* from a search
that was scoped to established simulators and MCP registries. It is not proof of
uniqueness. The defensible phrasing is *first-party where others are third-party*, not
*the only one in existence*.

### 2.2 Newton as the shipped default, not an experiment

OmniSim's physics bet is the one the field converged on — and OmniSim defaults to it today.

Verified ✅: Newton is Apache-2.0, co-developed by **NVIDIA, Google DeepMind, and Disney
Research**, announced at GTC (March 2025), public beta 2025-09-29, contributed to the
**Linux Foundation** the same day for vendor-neutral governance; 1.0 GA shipped at GTC 2026.
It is built on **NVIDIA Warp + OpenUSD**, is differentiable, and has an extensible
multi-solver design with **MuJoCo Warp as its key solver**.

Verified ✅: **Isaac Lab's Newton integration is explicitly experimental** as of mid-2026 —
develop branch, Isaac Lab 3.0 Beta, "under heavy development," a limited set of classic-RL
and flat-terrain locomotion examples, with breaking-change and no-production-support
warnings. Isaac Lab's *production* physics remains PhysX 5.

So the precise claim is: **OmniSim runs Newton as its default in-engine backend today,
while the flagship commercial stack still carries Newton as an experimental option.** That
is a statement about *default posture*, not about maturity — PhysX 5 is battle-tested and
Newton is eighteen months old. Do not upgrade this into "OmniSim's physics is more mature
than Isaac's." It is not.

Verified ✅ that the bet is the field's: MJX's own developers name migration to Warp as the
fix for JAX's structural limits (1–3 min JIT compiles; contact cost scaling with *possible*
rather than *active* contacts). MJWarp is jointly maintained by DeepMind and NVIDIA as part
of Newton, and is being adopted across both dominant GPU-RL ecosystems (Isaac Lab via
Newton; MuJoCo Playground; `mjlab`).

### 2.3 Train == deploy, bit-exact, in one engine

Most stacks train in one simulator and deploy in another, then spend the project fighting
the gap. OmniSim's trainer and its Newton deploy runtime derive their physical model from
one source of truth — [`g1_physics.json`](../../projects/policies/research/backends/g1_physics.json)
+ `g1_physics_spec.py` + the prim URDF — enforced in CI by
[`tests/test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py).
See [g1-single-source-of-truth.md](g1-single-source-of-truth.md).

The field's benchmark for RL credibility is **MuJoCo Playground** ✅: policies train in
minutes on consumer GPUs (quadruped joystick locomotion in ~5 min; Unitree G1 walking in
under 30 min on 2× RTX 4090, flat ground), and its authors deployed zero-shot sim-to-real
across **six** real platforms — Unitree Go1 and G1, Berkeley Humanoid, Booster T1, LEAP
Hand, Franka arm — in under eight weeks.

**That is the bar, and OmniSim has not cleared it.** OmniSim's sim-to-real story is
sim-to-*deploy* (trainer → in-engine Newton), not sim-to-real-hardware. The canonical,
unflattering status lives in [rl-current-state.md](rl-current-state.md). Anyone comparing
the two should read that file, not this table.

---

## 3. Per-simulator notes

### Isaac Sim / Isaac Lab — the photoreal, high-floor incumbent
Isaac Sim is now **open source under Apache-2.0** ✅ (the `isaac-sim/IsaacSim` repo), latest
release **v6.0.1, 2026-06-22** ✅. Isaac Lab is BSD-3-Clause ✅ (with the `isaaclab_mimic`
extension under Apache-2.0), latest **v3.0.0-beta2.patch1, 2026-07-02** ✅, Windows-64
supported ✅, with ready-to-train envs for RSL RL, SKRL, RL Games, and Stable Baselines ◐.

The differentiator is the **RTX renderer** — genuinely photoreal, and the reason Isaac wins
perception and synthetic-data work outright. The cost is the **hardware floor**, which is
steep and verified: minimum **GeForce RTX 4080**, **16 GB VRAM** floor (48 GB "ideal") ✅,
and — counterintuitively — **GPUs without RT cores (A100, H100) are not supported** ✅. A
datacenter card you'd train on cannot run the simulator. Add 32 GB RAM minimum and ≥50 GB
storage ◐, plus an internet connection for asset/extension access ◐.

Windows 11 is officially supported ✅ (Docker/container support is Linux-only ◐).

### Gazebo — the ROS default
Apache-2.0, CPU physics, **DART default with Bullet pluggable** ◐ through the `gz-physics`
abstraction ◐. Release cadence is yearly: **Jetty (Sept 2025) is LTS to May 2031**; Harmonic
is LTS to May 2029; **Ionic is *not* LTS** and EOLs Dec 2026 ✅ — worth knowing before
pinning.

Its moat is ROS. Gazebo's `ros2_control` integration is **hosted first-party by the
ros-controls organization itself** ✅, alongside MuJoCo and a topic-based interface. Isaac
Sim's and Webots' `ros2_control` integrations are classified **community-contributed** ✅.
CoppeliaSim, PyBullet, Genesis, SAPIEN, Unity — and OmniSim — **do not appear on that list
at all** ✅ (the registry is PR-editable and self-reported, so this reflects ecosystem
posture, not a certification).

No GPU physics, no batched RL, and Windows support is weak.

### Upstream Webots — OmniSim's parent
Apache-2.0, still actively maintained: **R2025a** shipped in early 2025 ✅ (the changelog
dates it 2025-01-31; the GitHub release tag renders "04 Feb" — a few days' normal
changelog-vs-tag skew, both 2025), with an in-development R2025b adding a Blender-style
viewport mode and MATLAB API additions ◐. R2025a names **improved ROS 2 support** as a
headline ✅, and ships Windows 10 / Linux (incl. snap + Docker) / macOS (Apple Silicon +
Intel) builds ✅.

Webots is the honest baseline for what OmniSim inherited: broad robot/sensor/world library,
CPU ODE physics, competent-but-not-photoreal rendering. Everything OmniSim adds (Newton,
wgpu, harness, RL pipeline, omniworld, CUDA granular) sits on top of that.

### MuJoCo / MJX / MJWarp — the RL physics reference
Apache-2.0, and the accuracy/speed reference for contact-rich control. **MJWarp requires an
NVIDIA GPU** for fast simulation (CPU only for development/debugging) ✅ — Warp targets CUDA,
so there is no AMD path. NVIDIA's announced **>70× (humanoid) / 100× (in-hand manipulation)**
speedups ⚠️ carry an unstated baseline; treat as vendor figures.

MuJoCo is not a *robot simulator* in the Webots/Gazebo sense — no world editor, no sensor
zoo, no GUI scenario authoring. It is a physics engine plus an ecosystem.

### Genesis — the cautionary tale, now a real project
Apache-2.0 ✅, ~29.5k stars ✅, actively maintained (**v1.2.1, 2026-07-03**) ✅, unifying rigid/
FEM/MPM/PBD solvers with three render paths including a ray tracer ◐, and running on CUDA,
ROCm, **and Apple Metal** ✅ (plus Vulkan / x86 / ARM64) — the broadest GPU-vendor support in
this table.

Its Dec-2024 launch claimed **43 million FPS / 430,000× real-time** ⚠️. ManiSkill lead Stone
Tao filed issue #181 alleging the benchmark used single-substep settings unlike the project's
own examples, robots idle >90% of the test, self-collisions disabled, stationary objects
hibernated ◐. Genesis published a corrected benchmark (~Jan 2025); **Tao acknowledged the
correction as comprehensive and credited Genesis with genuinely strong scalability — while
explicitly not independently verifying the new numbers** ◐. The README no longer advertises
43M FPS ◐.

The lesson OmniSim takes from this is procedural, not competitive: publish measured numbers
with hardware attached, or don't publish them.

### ManiSkill 3 / SAPIEN — the manipulation specialist
Actively maintained (**v3.0.1, 2026-04-21**) ✅, RSS 2025 paper ◐, GPU-parallelizes **both
simulation and rendering** — claiming RGBD + segmentation at **30,000+ FPS on one RTX 4090**
⚠️ and 10–1000× faster sim-with-rendering at 2–3× less GPU memory ⚠️. Heterogeneous parallel
scenes (each env a different scene) ◐ is a capability OmniSim does not have.

**Licence trap:** code is Apache-2.0, but bundled **assets are CC BY-NC 4.0 — non-commercial**
✅. In physics-only contact-rich cube-picking, Genesis reportedly runs 3–10× *slower* than
ManiSkill/SAPIEN ◐.

### PyBullet — in maintenance mode
Zlib. Last release **PyBullet 3.2.5, 2023-04-24** ✅ — roughly three years without a release
as of mid-2026, and that final one was bugfixes only ✅. Cadence collapsed after 2020–21 ◐.
Still excellent for quick CPU rigid-body work and still widely cited; not where new
GPU-parallel work goes.

### CoppeliaSim — the licence outlier
Three editions — **Edu / Pro / Lite** ✅. The free **Edu** edition is restricted to *students,
teachers, and professors of schools and universities* — it **excludes companies, research
institutions, non-profits, and foundations** alike ✅ — and **cannot be used for any commercial
purpose** ✅. Pro/Lite pricing is **not published**; you contact the vendor ✅. Everything else
in this table is free to ship a product with (modulo ManiSkill's assets).

### Unity Robotics / Colosseum — the dormant lane
Unity Robotics Hub provides ROS TCP Connector, URDF Importer, ROS TCP Endpoint with ROS 2
tutorials — but **community/tutorial-grade rather than a maintained first-party bridge** ◐.
Community members regarded the project as abandoned as early as 2024 ◐, and a self-identified
former Unity robotics team member stated in Jan 2025 that **the team was laid off years
earlier** ◐, with no official staff response in the thread ◐.

Microsoft shut down AirSim in July 2022; **Colosseum** is the MIT-licensed community fork ◐,
targeting Unreal Engine 5.6 on its main branch with in-tree ROS 2 ◐ — but its newest *tagged*
release is **v2.1.0 from June 2023** ◐, so 2025–26 use means building from main.

---

## 4. Where OmniSim loses

State these before anyone else does.

- **No ROS 2 bridge.** This is the big one. OmniSim retains Webots' ROS-derived *robot and
  sensor assets* (Rosbot, TurtleBot3, velodyne/sick xacro descriptions) and inherited
  historical docs — but there is **no `webots_ros2` equivalent, no `rclcpp`/`rclpy`, no live
  bridge** in this tree. OmniSim does not appear in the `ros2_control` simulator registry ✅,
  where Gazebo and MuJoCo are first-party-hosted and even Isaac Sim/Webots are only
  community-contributed. Its agent interface is [PROTOCOL.md](../../PROTOCOL.md)'s HTTP/JSON
  surface, deliberately — the full stance, what it costs, when to use Gazebo instead, and a
  working external ROS 2 bridge recipe are written up in
  [ros2-integration.md](ros2-integration.md). For a ROS-centric lab, **Gazebo is the correct
  choice and we should say so.**
- **Not photoreal.** Isaac Sim's RTX renderer and UE5's Nanite/Lumen are a different visual
  class. OmniSim's default main view is still WREN (OpenGL); wgpu is opt-in per world. See
  [wgpu-renderer-status.md](wgpu-renderer-status.md).
- **Sim-to-real is unproven.** OmniSim demonstrates sim-to-*deploy* parity in-engine. MuJoCo
  Playground demonstrated zero-shot transfer to six physical robots ✅. Those are not the same
  achievement, and the table in §1 does not capture the difference.
- **Young fork, narrow shoulders.** Isaac, Gazebo, MuJoCo, and Webots each carry years of
  battle-testing and a real contributor community. The Newton migration is incomplete
  ([engine-migration-plan.md](engine-migration-plan.md)), and per-robot locomotion maturity
  ranges from solid to open research ([rl-current-state.md](rl-current-state.md)).
- **NVIDIA-shaped GPU path.** Warp targets CUDA. Newton's fast path needs an NVIDIA GPU;
  Genesis's ROCm/Metal support ✅ is broader. OmniSim's mitigation is that **ODE still runs
  everything on CPU** — a real fallback Isaac Sim does not have.

The counterweight to that last row is worth stating plainly: **Isaac Sim will not start
without an RT-core GPU** ✅ (no A100, no H100, RTX 4080 floor, 16 GB VRAM ✅). OmniSim runs on
a CPU-only box via ODE, and reaches the GPU-batched tier on a laptop RTX 3060 (numbers:
[performance-comparison.md §3.2](../benchmarks/performance-comparison.md)). Accessibility is a
real, measured axis — and the one where OmniSim's advantage is least contestable.

---

## 5. The one-paragraph positioning

> OmniSim is an Apache-2.0 robotics simulator that inherits Webots' breadth of robots,
> sensors, and worlds, and adds three things the incumbents don't combine: it **defaults to
> Newton** (the Linux-Foundation, NVIDIA/DeepMind/Disney GPU physics engine that Isaac Lab
> still carries as experimental), it ships a **first-party HTTP surface designed for coding
> agents** to author and debug worlds (where Isaac and Gazebo have only third-party MCP
> servers), and it runs the **GPU-batched RL path on a laptop GPU** — with a CPU-only ODE
> fallback — where Isaac Sim requires an RT-core card and refuses to run on an A100. It is
> not photoreal, it has no ROS 2 bridge, and its policies have not been transferred to
> physical hardware. For ROS-centric integration work, use Gazebo. For photoreal perception
> and synthetic data, use Isaac Sim. For an agent-driven simulator you can talk to, on the
> hardware you already own, use OmniSim.

---

## 6. What we could not verify

The licence, maintenance, hardware-floor, and ROS-registry rows left unaudited by the
first research pass were **since verified by direct fetch of each primary source**
(2026-07-10): Isaac Lab BSD-3 + Windows, Gazebo LTS dates, Webots R2025a, all CoppeliaSim
licence rows, Genesis licence/status, ManiSkill's Apache-code/CC-BY-NC-assets split,
PyBullet's last release, and the `ros2_control` registry classification are now ✅. The
earlier "Webots date conflict" turned out to be an artifact — both sources place R2025a in
early 2025 (see §3). What genuinely remains open:

- **A handful of ◐ rows were not re-fetched this pass** and stay unaudited: Gazebo's
  **DART-default / Bullet-pluggable** physics detail, Genesis's and ManiSkill's **GPU-physics
  and render-path** specifics, **Colosseum's** status, and the **Unity-Robotics-dormant**
  read (which rests on forum sentiment and is inherently soft). None is load-bearing for the
  positioning in §5.
- **The 70×/100× MJWarp speedups** ⚠️ have no stated baseline (vs MJX? vs CPU MuJoCo? at what
  parallelism?) and no independent third-party benchmark of Newton/MJWarp throughput was
  found.
- **Genesis's corrected FPS magnitude remains contested** — Tao's RTX 4090 retest and the
  Genesis team's revised figure differ by roughly two orders of magnitude, and Tao did not
  independently verify the correction ◐. Cite no single Genesis number as settled.
- **"No other simulator ships a first-party agent HTTP authoring API"** is an absence of
  evidence over established simulators and MCP registries, not a proof of uniqueness (§2.1).
- **Newton 1.0 GA maturity inside Isaac Lab 3.0** — whether the integration has exited
  experimental status since the docs were fetched (2026-07-10), and what gaps remain vs
  PhysX (sensors, terrain, deformables) — is open. Beta 2 release notes mention rough-terrain
  improvements, so the "flat terrain only" detail may already be stale.

---

## 7. Method

Two `deep-research` workflow passes (fan-out web search → primary-source fetch → 3-vote
adversarial verification → synthesis), 209 subagents total. Pass 1 (102 agents) covered the
Newton / MJWarp / MJX / Isaac Lab GPU-RL cluster: 98 claims extracted, 25 verified, **25
confirmed 3-0, 0 refuted**. Pass 2 (107 agents) targeted the remaining simulators and the
agent-authoring question: 120 claims extracted from 25 sources, 5 confirmed 3-0, but the
adversarial vote for the other 20 died on a session-budget limit. **Those 20 were then
verified by direct WebFetch of each cited primary source** (2026-07-10) — licence,
maintenance, hardware-floor, and ROS-registry rows are ✅ as a result; the residual ◐ rows
are the ones §6 lists as not re-fetched (soft or non-load-bearing).

OmniSim-side facts were verified against this checkout (`src/omnisim/nodes/`,
`src/controller/launcher/`, `PROTOCOL.md`, `scripts/harness/`), not against its own docs.

**References** (primary sources): Newton — [developer.nvidia.com/newton-physics](https://developer.nvidia.com/newton-physics),
[Linux Foundation announcement](https://www.linuxfoundation.org/press/linux-foundation-announces-contribution-of-newton-by-disney-research-google-deepmind-and-nvidia-to-accelerate-open-robot-learning),
[Isaac Lab Newton integration](https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/index.html) ·
MJWarp — [github.com/google-deepmind/mujoco_warp](https://github.com/google-deepmind/mujoco_warp) ·
MuJoCo Playground — [technical report](https://playground.mujoco.org/assets/playground_technical_report.pdf), [arXiv 2502.08844](https://arxiv.org/abs/2502.08844) ·
Isaac Sim — [github.com/isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim), [5.1.0 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) ·
Gazebo — [releases](https://gazebosim.org/docs/latest/releases/), [gz-sim physics tutorial](https://github.com/gazebosim/gz-sim/blob/gz-sim9/tutorials/physics.md) ·
Webots — [R2025a release](https://github.com/cyberbotics/webots/releases/tag/R2025a) ·
CoppeliaSim — [licensing](https://manual.coppeliarobotics.com/en/licensing.htm) ·
PyBullet — [bullet3 releases](https://github.com/bulletphysics/bullet3/releases) ·
Genesis — [genesis-world](https://github.com/Genesis-Embodied-AI/genesis-world), [issue #181](https://github.com/Genesis-Embodied-AI/Genesis/issues/181) ·
ManiSkill — [github.com/haosulab/ManiSkill](https://github.com/haosulab/ManiSkill), [arXiv 2410.00425](https://arxiv.org/abs/2410.00425) ·
`ros2_control` simulators — [control.ros.org](https://control.ros.org/jazzy/doc/simulators/simulators.html) ·
agent tooling — [omni-mcp/isaac-sim-mcp](https://github.com/omni-mcp/isaac-sim-mcp), [kvgork/gazebo-mcp](https://github.com/kvgork/gazebo-mcp) ·
Unity/Colosseum — [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub), [CodexLabsLLC/Colosseum](https://github.com/CodexLabsLLC/Colosseum).

When this doc and the code disagree, the code wins — and update this doc in the same change.
