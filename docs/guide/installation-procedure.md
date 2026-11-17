## Installation Procedure

Pick the fastest route that exists for your platform. **On Windows, do not
build from source** — there is an installer, and compiling costs 10-25 minutes
you do not need to spend ([quickstart.md](../developer/quickstart.md) records
"5-25 minutes"; [`linux_bootstrap.sh`](../../scripts/install/linux_bootstrap.sh)
records "10-25 min"). On Linux a build is currently the only route, so the
question there is only whether you run the bootstrap script or drive `make`
yourself.

> This page previously said OmniSim "is currently distributed as source" and
> that a prebuilt binary would come "once OmniSim ships numbered releases".
> That had been out of date since v8.1.5: a Windows installer has shipped on
> every release since 2026-08-25. The old text routed every reader, on every
> platform, into a source build they did not need.

| Platform | Fastest route | Time | What you need first |
|---|---|---|---|
| **Windows 10/11** | The installer asset on the [latest release](https://github.com/omnilink-tech/omnisim/releases/latest) | one ~600 MB download + install | nothing (admin rights for the installer) |
| **Ubuntu 24.04 / 22.04** | `bash scripts/install/linux_bootstrap.sh` | 25-45 min | a clone and `sudo` |
| **macOS** | *none — not supported* | — | — |
| **Any**, if you intend to modify the engine | Source build | 10-25 min compile | a full toolchain |

There is **no one-image-pull route on any platform today**: the container
exists but has never been published (see below), so it too has to be built
locally. Windows is the only platform with a download.

Versions — OS, Python, GPU — live in one place, [System Requirements](system-requirements.md);
this page does not restate them.

### Windows — the installer

Download and run the `omnisim-<version>_setup.exe` asset from the
[latest release](https://github.com/omnilink-tech/omnisim/releases/latest). It
sets `OMNISIM_HOME` for you, registers the `.omniworld`/`.wbt` file types, and
creates Start Menu and desktop shortcuts.

Once it is installed, the first command is:

```bat
omnisim.bat doctor
```

`omnisim.bat` sits in the install root. It resolves an interpreter — a system
Python first, falling back to the CPython 3.12 bundled with the Newton runtime —
and sets `PYTHONPATH`, so the command works from any directory and on a machine
with no Python installed. If you do have Python 3.12 on `PATH`, plain
`python -m omnisim doctor` from the install root is the same thing.

`doctor` prints a VERDICT line and exits non-zero when the install cannot run.
Then `python -m omnisim demo` runs the flagship demo, and `python -m omnisim demos`
lists every runnable demo.

⚠️ **Known gap, v8.1.6 and earlier.** Those installers contain neither the
`omnisim` Python CLI nor `omnisim.bat`, so `python -m omnisim doctor` — the
command README.md and BETA.md both open with — fails there with
`No module named omnisim`, and there is no `.bat` to fall back to. Both are now
in the packaging manifest ([`files_core.txt`](../../scripts/packaging/files_core.txt)),
so this is fixed from the next release onward. On v8.1.6 itself, either use the
GUI (the Start Menu shortcut, then File → Open World) or clone the repository
and run the CLI from the clone root.

### Linux — the bootstrap script

There is no native Linux package yet, so on Linux you build. The scripted path
does the whole sequence for you:

```bash
git clone https://github.com/omnilink-tech/omnisim.git
cd omnisim
bash scripts/install/linux_bootstrap.sh          # or one phase at a time
```

Phases are `deps | python | fetch | wgpu | build | gpu | smoke | all`; `all` is the
default. Budget **25-45 minutes** — the compile is the bulk of it, and `gpu`
pulls the Newton wheels.

**Ubuntu 24.04 or 22.04.** On 24.04 the engine embeds the system Python 3.12.
On 22.04 the system python3 is 3.10 — where `newton` 1.5.0 raises at
`ModelBuilder()` — so the bootstrap's `python` phase installs 3.12 from
deadsnakes and the build embeds that instead, refusing to link a 3.10 rather
than produce a simulator that loads worlds and stands still. Details in
[System Requirements](system-requirements.md#python).

**No GPU on the box?** Headless physics runs should set `OMNISIM_NO_WINDOW=1`:
without a GPU the main view is drawn by Mesa's software Vulkan, and on 22.04
that can hold a texture-heavy world's first frame -- and the physics behind it
-- for minutes. Measured, and explained, in
[System Requirements](system-requirements.md#operating-systems).

A native Linux tarball is a real gap rather than an impossibility — the
packaging code for it exists but has never been exercised in CI. See
[`docker/README.md`](../../docker/README.md#what-is-still-missing).

### macOS

**macOS is not supported: there is no package, no verified build, and Newton
physics is unverified. Use Windows or Ubuntu 24.04.**

The container is not a way around this either. It has no `linux/arm64` build,
so on Apple Silicon it would run under x86-64 emulation — which nobody has
tested, on top of a physics stack nobody has verified on the platform.

### The container

⚠️ **The image is not published.** There is no
`ghcr.io/omnilink-tech/omnisim` tag on the registry:
[`runtime-image.yml`](../../.github/workflows/runtime-image.yml) exists but has
never run. Any instruction to `docker pull` or `docker run ghcr.io/...` is
wrong today. Build it locally instead:

```bash
docker build -f docker/Dockerfile.runtime -t omnisim:local .
docker run --rm omnisim:local doctor
```

The build compiles the engine from your working tree, so it costs the usual
10-25 minutes **once**; after that it is the no-build path it is meant to be.
Physics runs on the CPU by default, so no GPU is required. What it does and
does not cover: [`docker/README.md`](../../docker/README.md).

### Building from source

Only necessary if you are changing the engine itself.

- [AGENTS.md §2 — Build](../../AGENTS.md#2-build-only-if-the-binary-is-missing) —
  the canonical copy-paste build path.
- [Developer Quickstart](../developer/quickstart.md) — full prerequisites
  (Qt6, GLM, stb, platform notes).

On Windows, prefer `scripts/install/msys64_installer.sh --all` over the
hand-typed `pacman` list in the quickstart: it is the exact package set CI uses
([`release.yml`](../../.github/workflows/release.yml)), and it also builds the
Qt header mirror. On Linux, `bash scripts/install/linux_bootstrap.sh` does the
whole sequence.

---

The legacy Webots installation guide (Debian/Snap/APT, macOS DMG) is preserved
in the [upstream Webots documentation](https://github.com/cyberbotics/webots/blob/released/docs/guide/installation-procedure.md)
for reference — those installers do not produce an OmniSim binary.
