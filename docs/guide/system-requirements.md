## System Requirements

### Hardware

- A 64-bit (x86-64) PC. A quad-core CPU and 8 GB of RAM are a comfortable baseline; building from source is much faster with more cores (~7 minutes on 24 cores, up to ~25 minutes on a laptop).
- **A GPU and driver supporting Vulkan 1.2 or D3D12.** OmniSim renders through **wgpu-native**, and since WREN was deleted on 2026-08-23 (commit `976b9449d`) it is the only renderer — there is no OpenGL path and no fallback tier. A host whose wgpu-native cannot initialise has **no renderer at all**: it logs one line and keeps going, so physics and controllers still run and nothing draws. Recent NVIDIA and AMD adapters with vendor drivers are the tested configuration; Intel integrated graphics generally satisfies Vulkan 1.2 but is not tested here.
- **The Newton physics runtime is mandatory, not optional.** Newton/MuJoCo has been the only physics backend since ODE was deleted on 2026-08-08 (commit `bdc02139`), so there is no CPU fallback engine any more: a build that cannot import `newton` / `warp` / `mujoco` through its embedded interpreter has **no physics at all**. On Windows a stock release bundles the runtime for you (~600 MB); on Linux, pip the wheels into the **system** `python3`.
- **A CUDA GPU is needed only for the GPU solver.** The default `WorldInfo.newtonSolver` is the **CPU** `mj_step`, which runs on any supported machine — so demos, world authoring and single-robot deploy do not require NVIDIA hardware. An **NVIDIA GPU with CUDA** is required for `newtonSolver "mujoco_warp"`, and therefore for batched RL training and the locomotion training pipeline.
- ~10 GB free disk space for the toolchain, dependencies, and build artifacts, plus ~1 GB for the Newton runtime on Windows.

### Operating systems

- **Windows** — fully supported. OmniSim runs on Windows 11 and Windows 10 (64-bit only). This is the primary development and release platform; see the [Developer Quickstart](../developer/quickstart.md).
- **Linux** — supported as of v5.1 (x86-64 only). Verified end-to-end on Ubuntu (build from source, Newton GPU physics, locomotion demos). Specifics:
  - **Ubuntu 24.04 is required** (system Python 3.12). ⚠ **22.04 does NOT work**: its Python is 3.10, and `newton` 1.5.0 raises `TypeError: Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool].` at `ModelBuilder()` there — despite every package in the stack declaring `Requires-Python >=3.10`. Measured on both, same wheels, by [`physics-runtime-check`](../../.github/workflows/physics-runtime-check.yml). The engine embeds and links the **system** interpreter on Linux, so the distro release chooses it, and on 22.04 you get a simulator that loads worlds and stands still. Ubuntu 26.04 / Python 3.14 is wheel-fragile.
  - The supported setup path is `scripts/install/linux_bootstrap.sh` (v5.1); manual steps are in the [quickstart's Linux section](../developer/quickstart.md#linux-quickstart-ubuntu).
  - Newton needs its wheels (`torch warp-lang newton mujoco mujoco-warp`) installed into the **system** `python3` — the engine's embedded interpreter ignores virtualenvs.
  - Headless runs require **Xvfb**: a Qt/XCB context is created even with `--no-rendering`.
- **macOS** — **not supported: there is no package, no verified build, and Newton physics is unverified. Use Windows or Ubuntu 24.04.** There is also no `linux/arm64` container image, so Apple Silicon would emulate the x86-64 one, which is equally unverified. (`warp`'s Apple support story differs from its CUDA one, so the Newton GPU path would not carry over as-is either.)

Versions of the above operating systems older than those listed may work but are not supported.

### Python

**Python 3.12** is the version to install. It is not a preference: on Linux the
engine embeds and links the *system* interpreter, and `newton` 1.5.0 raises
`TypeError: Union[arg, ...]: each arg must be a type.` at `ModelBuilder()` under
Python 3.10 — so a 3.10 host gives you a simulator that loads worlds and stands
still. That is why Ubuntu 24.04 is required and 22.04 is not.

- **Linux** — the distro release picks the interpreter, so use 24.04 and leave
  the system `python3` alone. Newton's wheels must go into that **system**
  `python3`; the embedded interpreter ignores virtualenvs.
- **Windows** — install Python 3.12 from [python.org](https://www.python.org/downloads/)
  and tick *Add python.exe to PATH*. It is recommended rather than required: the
  installed package ships `omnisim.bat` in the install root, which prefers a
  system Python and falls back to the CPython 3.12 bundled with the Newton
  runtime, so `omnisim.bat doctor` works on a box with no Python at all. Install
  it anyway if you want the OmniLink chat demos, whose bridges import
  `omnisim_bridges` from a system interpreter.

The controller API itself is stdlib-only and works on older Python 3; the 3.12
floor is the *engine's* requirement, not the API's.
