# OmniSim containers

Two images live here. They are for different jobs and are not interchangeable.

| File | Image | Purpose | Base | Rough size |
|---|---|---|---|---|
| [`Dockerfile.runtime`](Dockerfile.runtime) | `ghcr.io/omnilink-tech/omnisim` (not published — see Status) | **Run OmniSim.** The no-build onboarding path. | `ubuntu:24.04` | ~1.2-1.6 GB (est.) |
| [`Dockerfile.train`](Dockerfile.train) | `ghcr.io/omnilink-tech/omnisim-train` | In-engine RL training. Needs a GPU. | `nvidia/cuda:12.8.1-devel` | ~12-16 GB (est.) |

Both size figures are **estimates from layer contents, not measurements** — no
one has run `docker image inspect` on either. The runtime workflow prints the
real number on every build; replace these once it has.

**The Ubuntu tag is load-bearing, not cosmetic.** `Dockerfile.runtime` sets
`ARG UBUNTU_TAG=24.04` and then asserts the system interpreter at build time
(`test "$(python3 -c ...)" = "3.12"`, `Dockerfile.runtime:80`), failing the
build otherwise. 24.04 gives Python 3.12; 22.04 gives 3.10, on which newton
1.5.0 is broken. Overriding `--build-arg UBUNTU_TAG=22.04` does not produce an
older-but-working image, it produces a build that stops with `FATAL: system
python is not 3.12`. The engine also links `libpython3.12` and the build
asserts that too (`:116`).

Both images are built **amd64-only** — neither workflow sets a `platforms:`
key, so nothing publishes an arm64 manifest. On Apple Silicon Docker would fall
back to emulation. That has never been tried here; treat it as unverified
rather than as supported.

---

## The runtime image

### Status

⚠️ **Not published yet.** [`runtime-image.yml`](../.github/workflows/runtime-image.yml)
exists but has never run, so no `ghcr.io/omnilink-tech/omnisim` tag is on the
registry. Until it has run, build locally (below). Publish it with:

```bash
gh workflow run runtime-image.yml -f omnisim_tag=v8.1.6
```

### Why it exists

Before this image, the ways into OmniSim were a ~600 MB Windows installer or a
10-25 minute source build. On Linux and macOS there was **no download at all**,
so every non-Windows user compiled 586 translation units to watch a robot move.

The evidence that this was the actual funnel: on the public repo the Windows
installer for v8.1.5 and v8.1.6 has **0 downloads**, while the
`deps-linux-v1` build-dependency archives — which are fetched *by the Makefile
during a source build* — have 27 and 25. Essentially everyone who showed up
ended up compiling.

### There is no desktop GUI in this image

The entrypoint wraps **every** invocation in `xvfb-run -a`
([`omnisim-entrypoint.sh`](omnisim-entrypoint.sh)) — the default `omnisim`
subcommand path, `bash`/`sh`, and `--exec` alike. That is deliberate and cannot
be opted out of: the engine constructs a Qt main window for any world-running
invocation, so without a display it dies with an empty log while the launcher
still exits 0.

The consequence is that the two windowed verbs the root README leads with —
**`run-world`, and `demo` without `--headless`** — do not do here what they do
on a desktop. They open the GUI onto a throwaway X display nobody can see, so
from outside the container it looks like a hang, right up until you interrupt
it. There is no X forwarding, no VNC, and no `--stream` viewer wired up here.

Use the headless surface instead, which is the whole of what this image is for:

- `run-headless <world> --until-finalized` for "does it load and step", and
  `demo <id> --headless` for the same check on a catalogue demo.
- `harness --host 0.0.0.0` plus `POST /world/screenshot` when you want to
  *see* something — the harness renders offscreen and hands back a PNG. (Read
  the screenshot caveat under "What it does NOT cover" first: whether the
  software Vulkan adapter comes up in this image is unverified.)

For an interactive desktop OmniSim, use the Windows package or a source build.

### Use it

```bash
# Is the install coherent?
docker run --rm ghcr.io/omnilink-tech/omnisim:latest doctor

# Load a world, step it, and prove Newton drove the physics.
docker run --rm ghcr.io/omnilink-tech/omnisim:latest \
  run-headless projects/samples/demos/worlds/showcase/warehouse_husky.omniworld \
  --until-finalized

# The agent-facing HTTP harness, reachable from the host on loopback only.
docker run --rm -p 127.0.0.1:6789:6789 ghcr.io/omnilink-tech/omnisim:latest \
  harness --host 0.0.0.0

# A shell inside, still under Xvfb.
docker run --rm -it ghcr.io/omnilink-tech/omnisim:latest bash

# Anything that is not an `omnisim` subcommand.
docker run --rm ghcr.io/omnilink-tech/omnisim:latest --exec python3 -c 'import newton; print(newton.__version__)'
```

**The two `-p` forms are not equivalent.** `-p 6789:6789` binds every host
interface, publishing an endpoint that loads arbitrary world files and spawns
controller processes to anything that can reach the machine. `--host 0.0.0.0`
*inside* the container is still required — the harness has to bind the
container's own external interface for the port mapping to reach it, and
binding `127.0.0.1` inside would make it unreachable from the host — but the
host side belongs on loopback.

Mount your own worlds and read results back out:

```bash
docker run --rm -v "$PWD/myworlds:/work" \
  -e OMNISIM_LOG_PATH=/work/omnisim_log.txt \
  ghcr.io/omnilink-tech/omnisim:latest \
  run-headless /work/my_scene.omniworld --until-finalized

# Now on the host: the engine log, and the Newton verdict sidecar that proves
# physics actually drove the run.
cat myworlds/omnisim_log.txt.newton.json
```

**`OMNISIM_LOG_PATH` is what makes that example produce anything.** Left unset,
the engine writes `omnisim_log.txt` and its `.newton.json` sidecar into
`$OMNISIM_HOME` *inside* the container, and `--rm` deletes them along with the
container — so the run "worked" and left nothing behind to read. Point it into
the mount and both files land on the host.

The image runs as **root** (no `USER` directive in `Dockerfile.runtime`), so
anything written into a bind mount is root-owned on the host. Add
`--user "$(id -u):$(id -g)"` if that matters, and expect the usual permission
consequences inside the container if you do.

### Build it locally

```bash
git checkout v8.1.6        # or any branch/SHA you want the image built from
docker build -f docker/Dockerfile.runtime -t omnisim:local --build-arg JOBS=8 .
```

The image is built from your **working tree**, not from a published tag -- the
Dockerfile `COPY`s the build context. That is deliberate: a tag-clone can only
build already-released code, so it can never verify a fix before release.

The build compiles the engine, so it takes the usual 10-25 minutes **once**.
That is the point: it happens on a builder, not on every user's machine.

### What it covers

Physics, controllers, the supervisor, and the whole non-pixel agent surface:
`run-headless`, `--fail-on-runaway`, the HTTP harness (`/scene/tree`,
`/sim/step`, `/sim/reset`, `/sim/contacts`, `/sim/events`, `/robots`,
`/robot/<def>/joints`, `/joints/set`, `/ik`, snapshots), MCP, and ROS 2.

**No GPU is required.** The default solver is CPU `mj_step`: `WorldInfo.wrl:36`
leaves `newtonSolver ""`, `omnisim_newton_runtime.py:6144` maps that to
`use_mujoco_cpu=True`, and the model is pinned to the CPU device at `:5911`.
`import warp` performs no CUDA check, so the stack imports with no driver
present.

### What it does NOT cover — read before promising anything

- **GranularBed (MPM) does not run on CPU.** It raises unless the model is on
  CUDA (`omnisim_newton_runtime.py:5979-6011`).
- **Cloth and soft-body worlds are unusably slow on CPU** — forced-CPU cloth is
  measured at 51.9 ms/step, about 0.15x real time. Do not demo them from this
  image.
- **Whether screenshots work is UNVERIFIED.** The image installs wgpu-native
  (which `Dockerfile.train` does not, so the *published training image contains
  a binary with no renderer at all*) plus `mesa-vulkan-drivers` for a software
  Vulkan adapter. wgpu's `request_adapter` should fall through to lavapipe when
  it is the only device — but that is **reasoned from wgpu semantics and has
  never been measured here**. The workflow prints the adapter actually chosen
  via `OMNISIM_WGPU_INITLOG`; read that output before claiming
  `/world/screenshot` works. If there is no renderer, physics is unaffected and
  screenshot calls degrade to a clean `502 SCREENSHOT_EMPTY` rather than
  crashing.
- **CPU-only is asserted, not yet fully measured.** OmniBench lane 4c measures
  CPU-only physics by *hiding* CUDA from the process, not by running on a box
  that never had it. Running
  [`lane4/cpu_only.py`](../tests/benchmarks/omnibench/lane4/cpu_only.py) inside
  this container is the measurement that closes that gap. It has not been run —
  and it cannot be run from the image as shipped: `tests` is in
  [`.dockerignore`](../.dockerignore) so it never enters the build context, and
  `Dockerfile.runtime:132` prunes it again for good measure. Bind-mount it from
  a checkout:

  ```bash
  docker run --rm -v "$PWD/tests:/opt/omnisim/tests" \
    -e OMNISIM_LOG_PATH=/opt/omnisim/tests/omnisim_log.txt \
    omnisim:local --exec python3 tests/benchmarks/omnibench/lane4/cpu_only.py
  ```

  Untried. The mount is writable so the probe can persist its results, which
  means they come back root-owned (see above).

### Design notes

- **No torch.** `import torch` appears nowhere under `src/` or
  `lib/controller/`; it is training-only and costs ~3 GB. The hard inference
  dependency is `onnxruntime`.
- **Physics pins come from the repo's SSOT**
  ([`newton_runtime_pins.py`](../scripts/packaging/newton_runtime_pins.py)), read
  at build time rather than hand-copied, so the image cannot drift from the
  bundle the Windows installer ships.
- **`mujoco-warp` is installed even on the pure-CPU path** — newton's
  `SolverMuJoCo.import_mujoco()` imports both `mujoco` and `mujoco_warp`
  unconditionally and raises `ImportError` if either is missing.
- **`PYTHONPATH` is deliberately not set.** Two packages in this tree are both
  importable as `omnisim` — the CLI at `/opt/omnisim/omnisim` and the controller
  API at `lib/controller/python/omnisim` — and they shadow each other. A global
  `PYTHONPATH` would leak the CLI into every controller process and turn
  `from omnisim import Robot` into an `AttributeError`. The entrypoint `cd`s to
  `$OMNISIM_HOME` instead.
- **Xvfb is mandatory, not decorative.** The engine constructs a Qt main window
  for any world-running invocation, so an XCB context exists even under
  `--no-rendering`. Without a display the engine aborts in Qt's platform-plugin
  init with a header-only log. ⚠ This paragraph used to say "while the launcher
  still exits 0" — that was measured on 2026-07-25 against a `run-headless` that
  predated `03e988c58` (2026-07-26); `run-headless` now FAILs (exit 1, "simulator
  exited early"), and since 2026-08-29 it also prints the engine's `Qt Fatal:`
  line and the fix, and the raw `bin/omnisim` launcher propagates the engine's
  own exit status (public issue #6). Of the window-free modes,
  `OMNISIM_NO_WINDOW=1` **works** (the Linux CI smokes run under it; the
  "deadlocks Newton at `add_joint_revolute`" note was a stale 2026-05 finding,
  public issue #5) and is the natural mode for a container — this image keeps
  Xvfb because the default `--minimize` path still realises a main window;
  `OMNISIM_NO_GL` does not reliably step.
- **`src/omnisim/physics/*.py` is kept in the image on purpose.** The engine
  probes `$OMNISIM_HOME/src/omnisim/physics/` for `omnisim_newton_runtime.py`
  and puts it on `sys.path`. Pruning `src/` wholesale would leave a world that
  finalises with no Newton at all.
- **The workflow smoke-tests before it pushes.** It runs `doctor`, then a real
  world, and asserts the Newton verdict *sidecar* — the only race-free proof
  that physics actually drove the sim. A build that succeeds proves the layers
  resolved; only a run proves the product works. The repo has already shipped
  one front door that built perfectly and could not execute its own documented
  first command.

## What is still missing

- **A native Linux tarball.** `scripts/packaging/linux_distro.py` can emit
  `omnisim-<version>-x86-64.tar.bz2` (plus a `.deb`) and `make distrib` wires
  it, but it has never run in CI and has real bitrot: a module-level
  `import distro` that is in no requirements file, ~25 hardcoded library
  SONAMEs, and no entry for `libwgpu_native.so` or the Newton runtime.
- **macOS.** Neither packaged nor verified.
- **A slim installer.** `bundle_newton_runtime.py` documents a `--mode
  bootstrap` that would stage only the interpreter and pip the physics stack on
  first launch — "the honest middle ground if a ~600 MB installer is
  undesirable". It writes a `FIRST_RUN_INSTALL.txt` marker that **nothing in the
  tree reads**. Implementing that consumer would turn the ~600 MB Windows
  installer into roughly ~145 MB.
