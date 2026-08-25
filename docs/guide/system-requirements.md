## System Requirements

### Hardware

- A 64-bit (x86-64) PC. A quad-core CPU and 8 GB of RAM are a comfortable baseline; building from source is much faster with more cores (~7 minutes on 24 cores, up to ~25 minutes on a laptop).
- An OpenGL 3.3-capable graphics adapter for the WREN main-view renderer. NVIDIA and AMD adapters work well; Intel integrated graphics often has weaker OpenGL support and may cause rendering problems.
- **The Newton physics runtime is mandatory, not optional.** Newton/MuJoCo has been the only physics backend since ODE was deleted on 2026-08-08 (commit `bdc02139`), so there is no CPU fallback engine any more: a build that cannot import `newton` / `warp` / `mujoco` through its embedded interpreter has **no physics at all**. On Windows a stock release bundles the runtime for you (~600 MB); on Linux, pip the wheels into the **system** `python3`.
- **A CUDA GPU is needed only for the GPU solver.** The default `WorldInfo.newtonSolver` is the **CPU** `mj_step`, which runs on any supported machine — so demos, world authoring and single-robot deploy do not require NVIDIA hardware. An **NVIDIA GPU with CUDA** is required for `newtonSolver "mujoco_warp"`, and therefore for batched RL training and the locomotion training pipeline.
- ~10 GB free disk space for the toolchain, dependencies, and build artifacts, plus ~1 GB for the Newton runtime on Windows.

### Operating systems

- **Windows** — fully supported. OmniSim runs on Windows 11 and Windows 10 (64-bit only). This is the primary development and release platform; see the [Developer Quickstart](../developer/quickstart.md).
- **Linux** — supported as of v5.1 (x86-64 only). Verified end-to-end on Ubuntu (build from source, Newton GPU physics, locomotion demos). Specifics:
  - **Ubuntu 22.04 / 24.04 are recommended** (Python 3.10 / 3.12 — the safest targets for the Newton GPU wheels). Ubuntu 26.04 / Python 3.14 works today but is wheel-fragile.
  - The supported setup path is `scripts/install/linux_bootstrap.sh` (v5.1); manual steps are in the [quickstart's Linux section](../developer/quickstart.md#linux-quickstart-ubuntu).
  - Newton needs its wheels (`torch warp-lang newton mujoco mujoco-warp`) installed into the **system** `python3` — the engine's embedded interpreter ignores virtualenvs.
  - Headless runs require **Xvfb**: a Qt/XCB context is created even with `--no-rendering`.
- **macOS** — **untested; no setup guide.** No support claims are made for macOS. (Note also that `warp`'s Apple support story differs from its CUDA one, so the Newton GPU path would not carry over as-is.)

Versions of the above operating systems older than those listed may work but are not supported.
