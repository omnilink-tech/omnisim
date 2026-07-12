# OmniSim

**The simulator you talk to.** OmniSim is an open-source robotics simulator built **by agents, for agents** — **every line OmniSim adds on top of its [Webots](https://github.com/cyberbotics/webots) base was written by an AI agent under human direction.** Describe a scene, an agent builds it; describe a behavior, an agent wires the controller. It's the reference simulation environment for the [OmniLink](https://www.omnilink-agents.com) agentic platform, and it is distributed under the [Apache License 2.0](LICENSE).

Release notes for every version live in [CHANGELOG.md](CHANGELOG.md). Engine status: [engine migration plan](docs/developer/engine-migration-plan.md).

## Get started

OmniSim's own subsystems were built end-to-end by a [Claude Code](https://claude.com/claude-code) agent, and the repo is laid out so an agent can drive it the same way — the recommended path is to let one set everything up for you.

**1. Clone:**

```bash
git clone --recurse-submodules https://github.com/omnilink-tech/omnisim.git
```

> **On Windows** (the primary platform), enable long paths once before cloning into a deeply-nested folder: `git config --global core.longpaths true`. Cloning into a short path such as `C:\omnisim` also avoids the inherited-asset path-length limit.

**2. Open the folder in Claude Code** — recommended: the [VS Code extension](https://marketplace.visualstudio.com/items?itemName=anthropic.claude-code) — and pick the **newest Claude model available** in the model picker. OmniSim is developed against the latest models, and [AGENTS.md](AGENTS.md) (auto-loaded as the agent's entry point) assumes a capable one.

**3. Give it this first prompt:**

```text
Set up OmniSim from this fresh clone — install whatever the toolchain needs, build it,
and then launch the warehouse Husky demo so I can watch it.
```

That one prompt takes a fresh clone to a live simulation: the agent follows [AGENTS.md](AGENTS.md) and the [Developer Quickstart](docs/developer/quickstart.md) (one-time MSYS2 toolchain setup, `build_omni.bat` — 5–25 minutes the first time, running at low priority so your machine stays responsive — then the runtime DLLs), and opens the warehouse world. Click the Husky and press `F5` to follow it (or right-click → *Follow Object*); hold `LMB + WASD` to fly. For Newton physics (the v4 default backend), ask the agent to also run the one-time runtime bundle step below — until then, worlds run on the ODE fallback.

### Building by hand instead

**Windows** (MSYS2/MinGW64 — see [Developer Quickstart](docs/developer/quickstart.md) for the one-time toolchain setup):

```bat
build_omni.bat
```

**Linux / macOS**:

```bash
export OMNISIM_HOME=$(pwd)
python scripts/dev/omnisim_dev.py build all
```

### Engine defaults

OmniSim's defaults are **Newton physics** and **WREN rendering**:

- **Physics — Newton.** `physicsBackend "auto"` (the default on every Solid) resolves to the [Newton](https://github.com/newton-physics/newton) GPU-accelerated backend. **Newton requires Windows and an NVIDIA (CUDA) GPU today** — it runs on the `newton`/`warp` runtime, which the build vendors next to the binary (~600 MB; `make -C src/omnisim bundle-newton-runtime`, see [Newton runtime bundle](docs/developer/newton-runtime-bundle.md)). **On Linux and macOS the bundler is a no-op and worlds silently fall back to ODE.** ODE is a fully supported CPU backend — everything loads and runs — but the GPU physics, the RL pipeline, and the locomotion demos are Windows + NVIDIA only. Confirm which backend actually drove a run via the `<log>.newton.json` verdict sidecar the engine writes at world finalisation.
- **Rendering — WREN.** The OpenGL renderer inherited from Webots remains the default main-view renderer. The new wgpu backend is opt-in per world (`renderBackend "wgpu"` or `WorldInfo.defaultRenderBackend`); official binaries ship it compiled in, while a plain source build compiles an inert stub unless `WGPU_NATIVE_HOME` points at a [wgpu-native](https://github.com/gfx-rs/wgpu-native) checkout at make time. Current status and how to enable it: [wgpu renderer status](docs/developer/wgpu-renderer-status.md).
- **Full legacy rollback:** `OMNISIM_LEGACY=1` reverts the whole stack to ODE + WREN; `OMNISIM_FORCE_ODE=1` is the physics-only lever.

## How OmniSim compares

Robotics simulators are chosen on axes that rarely fit one table, and most published
comparisons quietly mix vendor marketing with measured numbers. Ours separates them: the
full capability comparison — licence, physics, rendering, RL, ROS, hardware floor,
maintenance status, with every claim marked verified / unaudited / vendor-claim — lives in
[docs/developer/simulator-comparison.md](docs/developer/simulator-comparison.md), and the
throughput measurements live in [docs/benchmarks/performance-comparison.md](docs/benchmarks/performance-comparison.md).

The short version — what OmniSim does that the incumbents don't combine:

- **Newton by default.** [Newton](https://github.com/newton-physics/newton) — the Apache-2.0, Linux-Foundation GPU physics engine co-developed by NVIDIA, Google DeepMind, and Disney Research — is OmniSim's default in-engine backend today. NVIDIA's own Isaac Lab still carries its Newton integration as [experimental](https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/index.html) (Isaac Lab 3.0 Beta, as of 2026-07). *Default posture, not maturity — PhysX 5 is battle-tested and Newton is young. And our Newton path is **Windows + NVIDIA only** right now; on Linux/macOS you get the ODE CPU backend.*
- **A first-party HTTP surface for coding agents.** Load a world, inspect the scene tree, check exposure, screenshot, hot-reload — in seconds, without relaunching the simulator ([PROTOCOL.md](PROTOCOL.md)), with a first-party [MCP server](packages/omnisim-mcp/) that plugs it straight into Claude Desktop and Cursor. Isaac Sim and Gazebo have agent control only through *third-party* community MCP servers.
- **It runs on the hardware you own.** The GPU-batched RL path reaches top-tier throughput on a laptop RTX 3060, and ODE still runs everything CPU-only. Isaac Sim requires an RT-core GPU — minimum RTX 4080, 16 GB VRAM, and [A100/H100 are explicitly unsupported](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html).

And what it doesn't do — stated before you find out the hard way:

- **No ROS 2 bridge.** OmniSim inherits Webots' ROS-derived robot and sensor *assets*, but there is no live bridge in this tree; the agent interface is HTTP/JSON by design. **For ROS-centric work, use [Gazebo](https://gazebosim.org)** — its `ros2_control` integration is hosted first-party by the ROS controls org.
- **Not photoreal.** Isaac Sim's RTX renderer is a different visual class. OmniSim's main view is WREN (OpenGL); wgpu is opt-in.
- **Sim-to-real is unproven.** OmniSim demonstrates train==deploy parity *in-engine*. [MuJoCo Playground](https://playground.mujoco.org) demonstrated zero-shot transfer to six physical robots. Those are not the same achievement, and the honest per-robot status is in [rl-current-state.md](docs/developer/rl-current-state.md).

## Then just say what you want

That first prompt is the whole workflow — once it's running, keep talking to the agent:

```text
"Launch the warehouse demo."
"Run the Living City and follow one of the cars."
"Generate a Mars world with a 5-Husky fleet and run it headless for 30 seconds."
"Add three red cylinders in front of the Husky and make it avoid them."
"Wire a Jackal on a flat platform, expose it on HTTP, and drive it forward 2 m."
"Why isn't the camera seeing the red cylinder?"
```

The agent finds or builds the world, edits `.wbt` files and controllers, runs the simulator in the GUI or headless, and verifies its own work with the [validation harness](scripts/harness/) (load → screenshot → hot-reload in a few seconds, without relaunching the simulator) — all while staying in the conversation with you.

For **runtime control of robots** in a live scene, point [OmniLink agents](https://www.omnilink-agents.com) at the per-robot HTTP bridges. They speak the [OmniSim Wire Protocol](PROTOCOL.md), with voice I/O, short-term memory, and per-turn usage telemetry built in.

## Supported Brands

OmniSim ships native models for robots from eight manufacturers. Maturity varies by robot: mobile bases and the drone drive interactively out of the box, while quadruped and humanoid locomotion run through the RL-deploy pipeline and range from production-solid to active research. The honest, per-robot deploy status always lives in [`DEMOS.md`](DEMOS.md) and the canonical [`docs/developer/rl-current-state.md`](docs/developer/rl-current-state.md).

- **Clearpath Robotics** — Husky · Jackal mobile bases
- **Husarion** — Rosbot / Rosbot XL mobile bases
- **Robotis** — TurtleBot3 (Burger / Waffle / Waffle Pi)
- **Boston Dynamics** — Spot quadruped · Atlas humanoid
- **Unitree** — Go2 · B2 quadrupeds · G1 · H1 humanoids
- **NASA** — Valkyrie humanoid
- **Universal Robots** — UR3e · UR5e · UR10e arms
- **DJI** — Mavic 2 Pro drone

## Teaching robots to move — Shadowing, Skills, BATON

Legged-locomotion policies in OmniSim are made by **Shadowing**: train a policy to track a reference
trajectory — *the ghost* — that has been **verified feasible before any learning starts**. The thesis is
that the reference, not the algorithm, is the bottleneck; an infeasible ghost cannot be tracked no matter
how long you train, so `ghost_validator.py` gates every new ghost against seven design rules *first*.

Training is **in-engine and local**. The trainer steps `omnisim-bin` itself (Newton / mujoco_warp, K≈4096
parallel worlds), so **train == deploy is bit-exact** — the policy learns in the same physics it ships in —
and the whole loop runs on the GPU you already own. There is no cloud path, and none is needed.

Trained policies are packaged in the **Skill Library** ([`projects/policies/skills/`](projects/policies/skills/)):
one versioned manifest per skill, binding its ghost, its validator verdict, its deploy env, its champion
checkpoint, and its provenance into a single file.

```bash
python projects/policies/skills/skill_lib.py list                # the catalogue
python projects/policies/skills/skill_lib.py preview walk        # watch the ghost before you train it
python projects/policies/skills/skill_lib.py train walk          # in-engine, on your local GPU
python projects/policies/skills/skill_lib.py sequence box_delivery   # run a BATON demo
```

Ten skills ship today across the G1, H1, Go2 and Spot — walk, turn, carry, stand, climb, arm motion,
balance. **BATON** composes them into task sequences by handing control from one specialist policy to the
next; four sequences ship, including box delivery (walk → carry → place) and walk-turn-walk.

**What is honestly not done.** Our *own* Shadowing-trained humanoid demos — the G1 walk, box delivery,
walk-turn-walk — run on a **weight-bearing balance harness**: a partial-support rig (λ=0.9) that carries up
to ~2× the 34 kg G1's weight upward plus attitude torque. They are **not free-standing walks**, and no
champion of ours has yet been shown with the harness fully off. The stair climb is the partial exception —
it takes **no vertical support**, so the legs do all the lifting — and it tops out at a measured **3 cm
riser** over 5 steps. Quadrupeds use no harness at all.

To be precise about where the limit actually is: **Unitree's own official G1 and H1 policies, re-hosted
unchanged inside OmniSim, walk free-standing** (G1: 33.7 m, 0 falls, 0.48 m/s — no harness). So the engine
carries an unassisted humanoid walk fine. **The open problem is ours: training a policy that does it.**
Per-robot status, with numbers, always lives in the canonical
[rl-current-state.md](docs/developer/rl-current-state.md).

Method and rules: [Shadowing](docs/developer/shadowing.md) · [ghost design rules](docs/developer/ghost-design-rules.md) · [Skill Library](docs/developer/skill-library.md) · [canonical RL status](docs/developer/rl-current-state.md).

## Docs

| Audience | Start with |
|---|---|
| AI coding agent in this repo | [AGENTS.md](AGENTS.md) |
| Picking a demo to run | [DEMOS.md](DEMOS.md) — the full demo catalogue |
| First-time human contributor | [Developer Quickstart](docs/developer/quickstart.md) |
| Engine / simulator developer | [docs/developer/](docs/developer/) — start with the [engine migration plan](docs/developer/engine-migration-plan.md) |
| World-author / scenario builder | [Simulation Authoring for Coding Agents](docs/developer/simulation-authoring-for-coding-agents.md) |
| Driving OmniSim from outside the repo | [PROTOCOL.md](PROTOCOL.md) — the canonical HTTP-over-JSON spec |
| Capturing videos and 4K screenshots | [scripts/capture/README.md](scripts/capture/README.md) |
| Contribution process | [CONTRIBUTING.md](CONTRIBUTING.md) |
| End-user manual (inherited from Webots) | [docs/guide/](docs/guide/) |
| Brand & visual identity | [resources/branding/omnisim/BRAND.md](resources/branding/omnisim/BRAND.md) |

## Bugs · License · Brand · Funding

- **Bugs**: [GitHub Issues](../../issues/new/choose).
- **Code license**: Apache 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE). OmniSim is built on [Webots](https://github.com/cyberbotics/webots) (open-sourced under Apache 2.0 in 2018); upstream copyright on derived files is preserved.
- **Brand**: The OmniSim name and orb mark are trademarks of OmniLink, governed by [TRADEMARKS.md](TRADEMARKS.md). Open code, protected brand — same pattern as Mozilla Firefox. You can fork and ship; if you ship a modified fork, rename it and replace the OmniLink branding.
- **Funding**: Recurring sponsorships fund the Newton and wgpu engine migrations. [Sponsor on GitHub](https://github.com/sponsors/omnilink-tech) · current backers in [SPONSORS.md](SPONSORS.md).

## Attribution

OmniSim is built on [Webots](https://github.com/cyberbotics/webots) (Cyberbotics Ltd., open-sourced under Apache 2.0 in 2018). Upstream copyright on derived files is preserved — files that retain Cyberbotics headers remain © Cyberbotics. OmniSim is an independent fork, not affiliated with or endorsed by Cyberbotics. See [NOTICE](NOTICE) for the full derivation and third-party attributions.

## Platform support

**Be aware of what works where before you invest in a build.**

| | Windows 10/11 | Linux | macOS |
|---|---|---|---|
| Build from source | ✅ documented ([quickstart](docs/developer/quickstart.md)) | ⚠️ builds, but the toolchain setup is not yet written up | ⚠️ untested; no setup guide |
| Simulator, worlds, URDF import, harness, capture | ✅ | ✅ | ✅ |
| **Newton GPU physics** | ✅ (needs an NVIDIA/CUDA GPU) | ❌ falls back to ODE | ❌ falls back to ODE |
| **RL training + locomotion demos** | ✅ | ❌ | ❌ |

Windows is the primary and only fully-supported platform today. On Linux and macOS the simulator itself runs on the ODE CPU backend — worlds load, controllers run, the agent harness works — but **the Newton GPU backend, the RL pipeline, and every quadruped/humanoid locomotion demo are Windows + NVIDIA only** (the demo launchers are PowerShell, and the runtime bundler no-ops off Windows). We would rather say this here than have you find out after a 25-minute build.

The RL/locomotion demos additionally need `torch` and `onnxruntime` in the Python environment that runs the controllers.

---

Maintained by **[OmniLink](https://www.omnilink-agents.com)**.
