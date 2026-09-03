## System Requirements

### Hardware

- A 64-bit (x86-64) PC. A quad-core CPU and 8 GB of RAM are a comfortable baseline; building from source is faster with more cores (a first build takes 5–15 min).
- **A GPU and driver supporting Vulkan 1.2 or D3D12.** OmniSim renders through **wgpu-native**, and since WREN was deleted on 2026-08-23 (commit `976b9449d`) it is the only renderer — there is no OpenGL path and no fallback tier. A host whose wgpu-native cannot initialise has **no renderer at all**: it logs one line and keeps going, so physics and controllers still run and nothing draws. Recent NVIDIA and AMD adapters with vendor drivers are the tested configuration; Intel integrated graphics generally satisfies Vulkan 1.2 but is not tested here.
- **The Newton physics runtime is mandatory, not optional.** Newton/MuJoCo has been the only physics backend since ODE was deleted on 2026-08-08 (commit `bdc02139`), so there is no CPU fallback engine any more: a build that cannot import `newton` / `warp` / `mujoco` through its embedded interpreter has **no physics at all**. On Windows a stock release bundles the runtime for you (~600 MB); on Linux, pip the wheels into the **system** `python3`.
- **A CUDA GPU is needed only for the GPU solver.** The default `WorldInfo.newtonSolver` is the **CPU** `mj_step`, which runs on any supported machine — so demos, world authoring and single-robot deploy do not require NVIDIA hardware. An **NVIDIA GPU with CUDA** is required for `newtonSolver "mujoco_warp"`, and therefore for batched RL training and the locomotion training pipeline.
- ~10 GB free disk space for the toolchain, dependencies, and build artifacts, plus ~1 GB for the Newton runtime on Windows.

### Operating systems

- **Windows** — fully supported. OmniSim runs on Windows 11 and Windows 10 (64-bit only). This is the primary development and release platform; see the [Developer Quickstart](../developer/quickstart.md).
- **Linux** — supported as of v5.1 (x86-64 only). Verified end-to-end on Ubuntu (build from source, Newton GPU physics, locomotion demos). Specifics:
  - **Ubuntu 24.04 and 22.04 are both supported.** 24.04 is the simple case: the engine embeds the system Python 3.12. On 22.04 the system python3 is 3.10, where `newton` 1.5.0 raises `TypeError: Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool].` at `ModelBuilder()` — despite every package in the stack declaring `Requires-Python >=3.10`, and identically on newton 1.5.1, so it is not fixed by a bump. The bootstrap therefore installs **Python 3.12 from deadsnakes** on 22.04 and the engine embeds *that* interpreter; the system `python3` keeps running controllers and the CLI, which is fine at 3.10 because controllers never import newton. The build refuses to link a <3.11 interpreter rather than produce a simulator that loads worlds and stands still. Both paths — and the 3.10 floor itself — are measured on every change by [`physics-runtime-check`](../../.github/workflows/physics-runtime-check.yml) and the two legs of [`linux-build`](../../.github/workflows/linux-build.yml). Ubuntu 26.04 / Python 3.14 is wheel-fragile.
  - The supported setup path is `scripts/install/linux_bootstrap.sh`; manual steps are in the [quickstart's Linux section](../developer/quickstart.md#linux-quickstart-ubuntu).
  - Newton needs its wheels installed into the interpreter **the engine links** — the system `python3` on 24.04, the deadsnakes `python3.12` on 22.04. `bash scripts/install/linux_bootstrap.sh gpu` resolves that automatically (it reads the binary with `ldd`); the embedded interpreter ignores virtualenvs either way.
  - Headless runs require **Xvfb**: a Qt/XCB context is created even with `--no-rendering`.
  - **No GPU at all?** Then the main view renders through **lavapipe**, Mesa's software Vulkan, and on Ubuntu 22.04 (Mesa 23.2) a texture-heavy world can spend *minutes* compiling its first frame on two cores while physics waits behind it -- measured on a 2-vCPU GitHub runner, where the warehouse demo never produced a frame in ten minutes that a real 22.04 machine with a GPU finishes in half a second. Ubuntu 24.04's Mesa 25.2 gets through in seconds. For a headless run that only needs physics and camera images, set **`OMNISIM_NO_WINDOW=1`**: no main view is built at all, camera devices still render offscreen through wgpu, and the world finalises on the physics timeline. That is how the Linux CI smokes run on GPU-less hosts.
- **macOS** — **not supported: there is no package, no verified build, and Newton physics is unverified. Use Windows or Ubuntu 24.04.** There is also no `linux/arm64` container image, so Apple Silicon would emulate the x86-64 one, which is equally unverified. (`warp`'s Apple support story differs from its CUDA one, so the Newton GPU path would not carry over as-is either.)

Versions of the above operating systems older than those listed may work but are not supported.

### Python

**Python 3.12** is the version the engine embeds. It is not a preference:
`newton` 1.5.0 raises `TypeError: Union[arg, ...]: each arg must be a type.` at
`ModelBuilder()` under Python 3.10 — so an engine embedding 3.10 loads worlds
and stands still. What changed on 2026-08-28: the bootstrap no longer treats
that as a reason to refuse 22.04 — it installs 3.12 (deadsnakes) there and the
build embeds it, asserting the link on the produced binary.

- **Linux** — on 24.04 the engine embeds the system `python3` (3.12); on 22.04
  it embeds the deadsnakes `python3.12` the bootstrap installs, while the
  system 3.10 keeps running controllers and the CLI. Newton's wheels go into
  the linked interpreter; the embedded interpreter ignores virtualenvs.
- **Windows** — install Python 3.12 from [python.org](https://www.python.org/downloads/)
  and tick *Add python.exe to PATH*. It is recommended rather than required: the
  installed package ships `omnisim.bat` in the install root, which prefers a
  system Python and falls back to the CPython 3.12 bundled with the Newton
  runtime, so `omnisim.bat doctor` works on a box with no Python at all. Install
  it anyway if you want the OmniLink chat demos, whose bridges import
  `omnisim_bridges` from a system interpreter.

The controller API itself is stdlib-only and works on older Python 3; the 3.12
floor is the *engine's* requirement, not the API's.
