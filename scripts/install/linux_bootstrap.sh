#!/usr/bin/env bash
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# OmniSim — Linux / RunPod bootstrap (the SUPPORTED Linux recipe).
#
# Builds OmniSim from source on a stock Ubuntu (+ NVIDIA CUDA) machine and
# brings up the Newton (mujoco_warp) physics backend so the in-engine RL
# trainers can run.
#
# SUPPORTED TARGETS:
#   * Ubuntu 24.04 -- the engine embeds the system Python 3.12. Nothing extra.
#   * Ubuntu 22.04 -- the system python3 is 3.10, where newton 1.5.0 raises
#     TypeError at ModelBuilder() (typing.Union rejects wp.array[wp.bool]
#     before 3.11; identical in 1.5.1, so not fixed by a bump). phase_python
#     installs 3.12 from deadsnakes and phase_build embeds THAT interpreter;
#     python3 remains the controller/CLI interpreter, which is fine at 3.10 --
#     controllers never import newton.
#   * Ubuntu 26.04 / py3.14 passes the version guard but is wheel-fragile; the
#     pinned physics stack has no guaranteed 3.14 wheels.
# All measured by .github/workflows/physics-runtime-check.yml and the 22.04 +
# 24.04 legs of linux-build.yml.
#
#   bash scripts/install/linux_bootstrap.sh deps    # apt prerequisites
#   bash scripts/install/linux_bootstrap.sh python  # >=3.11 interpreter where the distro lacks one
#   bash scripts/install/linux_bootstrap.sh fetch   # clone repo + glm/stb submodules
#   bash scripts/install/linux_bootstrap.sh wgpu    # wgpu-native: the ONLY renderer
#   bash scripts/install/linux_bootstrap.sh build   # make release (fetches its own Qt 6.5.3)
#   bash scripts/install/linux_bootstrap.sh gpu     # torch/warp/newton/mujoco_warp -> SYSTEM python3
#   bash scripts/install/linux_bootstrap.sh smoke   # headless demo world under Xvfb  <-- acceptance test
#   bash scripts/install/linux_bootstrap.sh all     # everything, in order
#
# IMPORTANT — the ML wheels MUST land in the interpreter THE BINARY LINKS:
#   The engine embeds CPython (Py_InitializeEx) and does `import warp` /
#   `import newton` from THAT interpreter. A venv is invisible to it. And on
#   many cloud images "python3 on PATH" is NOT that interpreter: ML pod
#   images (e.g. runpod/pytorch) repoint /usr/bin/python3 at their own 3.11
#   while apt's python3-dev/python3-config still belong to the distro's 3.10
#   — so the engine links libpython3.10 and a plain `python3 -m pip` puts
#   the wheels where the engine will never look. The gpu phase therefore
#   reads `ldd bin/omnisim-bin` and installs into the LINKED interpreter.
#   Ubuntu 24.04 (py3.12) is the target. 22.04 (py3.10) is NOT usable any
#   more: newton 1.5.0 raises at ModelBuilder() on 3.10 even though the wheel
#   declares Requires-Python >=3.10 (measured both ways by the
#   physics-runtime-check workflow). The guard in phase_gpu below refuses it.
#
# NOTE — no separate Qt phase:
#   `make release` runs dependencies/Makefile.linux's own aqtinstall target
#   (Qt 6.5.3, keyed off lib/webots/libQt6Core.so.6.5.3) regardless of any
#   system Qt. Mixing apt Qt headers (e.g. 6.10) with the vendored 6.5.3
#   libs causes a version clash — do NOT apt-install Qt6 dev packages.
set -euo pipefail

# Network volumes (RunPod et al.) are NFS-backed with root_squash: chown is
# forbidden even for root, and GNU tar AS ROOT defaults to --same-owner, so
# dependency-archive extraction dies with "Cannot change ownership". Neutral-
# ize globally; harmless on local disks.
export TAR_OPTIONS=--no-same-owner

REPO_URL="${REPO_URL:-https://github.com/omnilink-tech/omnisim.git}"
# /workspace is the RunPod-style network volume and is the right home on a pod.
# On an ordinary desktop it does not exist, and defaulting there sends the clone
# somewhere the user typically cannot create — so pick the home from the machine.
# If we are ALREADY standing in an OmniSim checkout, that checkout is the home.
# This script ships inside the repo, so "clone it and run the bootstrap" is the
# common case -- and the old default appended /omnisim to $PWD, which collides
# with the repo's own omnisim/ CLI package directory: git refuses to clone into
# a non-empty dir and `set -euo pipefail` aborts the only Linux install command
# the README documents, with an error naming nothing the user did.
if [ -d "$PWD/.git" ] && [ -f "$PWD/AGENTS.md" ] && [ -d "$PWD/src/omnisim" ]; then
  _DEFAULT_HOME="$PWD"
elif [ -d /workspace ]; then
  _DEFAULT_HOME=/workspace/omnisim
else
  _DEFAULT_HOME="$PWD/omnisim"
fi
OMNISIM_HOME="${OMNISIM_HOME:-$_DEFAULT_HOME}"
# Containers report the HOST's core count (a RunPod A4000 pod says 112),
# so a bare -j$(nproc) can massively over-parallelize; cap the default.
_NPROC=$(nproc)
JOBS="${JOBS:-$(( _NPROC > 32 ? 32 : _NPROC ))}"
PHASE="${1:-all}"

log() { printf '\n\033[1;36m=== %s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31mFAIL: %s\033[0m\n' "$*" >&2; exit 1; }

# This script was written for root containers, so every apt-get call was bare.
# On an ordinary desktop that fails on the very first command of the very first
# phase, which is also the only Linux install command the README documents.
if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
  SUDO_H=""
elif command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
  # -H for pip: without it root writes a cache into the invoking user's $HOME and
  # leaves it root-owned, so their next ordinary pip run fails on its own cache.
  SUDO_H="sudo -H"
else
  die "not running as root and sudo is not installed — install sudo, or re-run this as root"
fi

phase_deps() {
  log "deps: apt prerequisites"
  export DEBIAN_FRONTEND=noninteractive
  $SUDO apt-get update
  # No qt6-*-dev here on purpose: the build vendors its own Qt 6.5.3 (see header).
  #
  # libdbus is NOT optional and was a known, self-documented hole in this list:
  # docker/Dockerfile.train:82 records that the vendored libQt6DBus needs it AT
  # LINK TIME and that the build dies after ~6 minutes of compiling without it.
  # The list got away with it because CI runners and ML base images ship libdbus
  # already; a minimal Ubuntu cloud image does not.
  #
  # libvulkan1 + mesa-vulkan-drivers give wgpu-native a software Vulkan adapter
  # (lavapipe). Since the WREN deletion wgpu is the ONLY renderer, and a
  # wgpu-native failure is a non-unwinding Rust panic across the C FFI boundary
  # -- it aborts the process rather than returning an error the engine could
  # degrade on. docker/Dockerfile.runtime installs both for exactly this.
  $SUDO apt-get install -y --no-install-recommends \
    build-essential make git wget curl unzip zip pkg-config ca-certificates \
    cmake swig libglu1-mesa-dev libglib2.0-dev libfreeimage3 libfreetype-dev \
    libxml2-dev libboost-dev libssh-gcrypt-dev libzip-dev libreadline-dev \
    libopenal-dev libssl-dev libgl1-mesa-dev libxi-dev libxrandr-dev pbzip2 \
    libfontconfig1-dev libxkbcommon-dev libxkbcommon-x11-dev \
    libdbus-1-dev libdbus-1-3 \
    libvulkan1 mesa-vulkan-drivers \
    python3 python3-pip python3-dev python-is-python3 \
    xvfb x11-utils \
    libxcb-cursor0 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-render-util0 libxcb-xinerama0 libxkbcommon-x11-0
  log "deps: OK  (gcc $(gcc -dumpversion), python3 $(python3 --version 2>&1 | cut -d' ' -f2))"
}

phase_fetch() {
  log "fetch: repo + submodules -> $OMNISIM_HOME"
  if [ ! -d "$OMNISIM_HOME/.git" ]; then
    mkdir -p "$(dirname "$OMNISIM_HOME")"
    git clone "$REPO_URL" "$OMNISIM_HOME"
  else
    echo "repo already present, skipping clone"
  fi
  cd "$OMNISIM_HOME"

  # glm pinned to 1.0.1 (newer breaks on modern GCC), stb from the omichel patch-1 branch
  if [ ! -f src/glm/CMakeLists.txt ]; then
    rm -rf src/glm
    git clone https://github.com/g-truc/glm.git src/glm
    ( cd src/glm && git checkout -q 1.0.1 )
  fi
  if [ ! -f src/stb/stb_image.h ]; then
    rm -rf src/stb
    git clone -q -b patch-1 https://github.com/omichel/stb.git src/stb
  fi

  find "$OMNISIM_HOME/scripts" -name '*.sh' -exec chmod +x {} \; 2>/dev/null || true
  log "fetch: OK  (glm $(cd src/glm && git describe --tags 2>/dev/null || echo '?'))"
}

# The interpreter the ENGINE should embed. newton 1.5.0 raises
# `TypeError: Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool].`
# at ModelBuilder() on CPython 3.10 -- warp's array.__or__ builds typing.Union
# at annotation-evaluation time, and the stdlib only accepts that from 3.11.
# Measured on both newton 1.5.0 and 1.5.1 (identical annotation in solver.py),
# so this is not fixed by a version bump. The fix is to embed a >=3.11
# interpreter; the system python3 stays the CONTROLLER interpreter and is fine
# at 3.10 -- controllers never import newton, physics lives in the engine.
embed_python() {
  for cand in python3.13 python3.12 python3.11; do
    command -v "$cand" >/dev/null 2>&1 && { echo "$cand"; return; }
  done
  echo python3
}

phase_python() {
  # On a distro whose system python3 is older than 3.11 (Ubuntu 22.04 ships
  # 3.10), install Python 3.12 from the deadsnakes PPA so the engine has a
  # physics-capable interpreter to embed. A no-op on 24.04.
  PYV_SYS=$(python3 -c 'import sys;print("%d%02d"%sys.version_info[:2])' 2>/dev/null || echo 0)
  if [ "$PYV_SYS" -ge 311 ]; then
    log "python: system python3 $(python3 -V 2>&1 | cut -d' ' -f2) is >=3.11 -- nothing to install"
    return
  fi
  log "python: system python3 is <3.11; newton 1.5.0 raises at ModelBuilder() there."
  log "python: installing 3.12 from deadsnakes -- the engine embeds it, python3 keeps running controllers"
  export DEBIAN_FRONTEND=noninteractive
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends software-properties-common
  $SUDO add-apt-repository -y ppa:deadsnakes/ppa
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends python3.12 python3.12-dev python3.12-venv
  # pip for the new interpreter -- via get-pip.py, NEVER ensurepip. Measured
  # on the ubuntu-22.04 CI image: `python3.12 -m ensurepip --upgrade` as root
  # half-replaces the pip the SYSTEM 3.10 also sees through Debian's shared
  # /usr/lib/python3/dist-packages, leaving BOTH interpreters with a broken
  # pip ("No module named pip._internal.cli.main") -- which then killed the
  # Qt dependency fetch inside make, two phases later, in a different tool.
  # get-pip.py installs into the version-specific dist-packages and touches
  # nothing shared.
  if ! python3.12 -m pip --version >/dev/null 2>&1; then
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
    $SUDO python3.12 /tmp/get-pip.py
    rm -f /tmp/get-pip.py
  fi
  python3.12 -m pip --version
  # The system interpreter's pip must have SURVIVED the install above -- it
  # fetches Qt via aqtinstall during make, and a corrupted one fails there
  # with a message nobody would trace back to this phase. Repair it if not.
  if ! python3 -m pip --version >/dev/null 2>&1; then
    log "python: system pip was damaged -- reinstalling python3-pip"
    $SUDO apt-get install -y --reinstall python3-pip
  fi
  log "python: $(python3.12 -V 2>&1) ready"
}

phase_wgpu() {
  # The ONE step the user recipe omitted and CI added privately.
  #
  # src/omnisim/Makefile auto-discovers WGPU_NATIVE_HOME from exactly one path,
  # $OMNISIM_HOME/_scratch/wgpu-native, and the only thing that creates it is
  # setup_wgpu_native.sh. Without it WB_WGPU_NATIVE_AVAILABLE is never defined,
  # every wgpu call compiles out, and since the WREN deletion (976b9449d) there
  # is no second renderer -- so the build is green and nothing draws: no
  # screenshots, no capture service, no Camera device.
  #
  # linux-build.yml runs this before the build and asserts the .so afterwards.
  # That workflow says its purpose is that "a green tick here is evidence about
  # THEIR experience", so the step belongs in the script the user runs.
  log "wgpu: fetching wgpu-native (the only renderer since the WREN deletion)"
  cd "$OMNISIM_HOME"
  bash scripts/dev/setup_wgpu_native.sh
  log "wgpu: OK"
}

phase_build() {
  log "build: make release -j$JOBS  (first build: 10-25 min; fetches Qt 6.5.3 itself)"
  cd "$OMNISIM_HOME"
  export OMNISIM_HOME
  export WEBOTS_HOME="$OMNISIM_HOME"   # legacy alias ~20 sub-makefiles still consume
  # Embed a >=3.11 interpreter where the system python3 cannot run newton.
  # Passed as a make ARGUMENT, not env: the Makefile honours an explicit
  # PYTHON_CONFIG and only then falls back to plain python3-config.
  # ALWAYS name the embed interpreter's own -config, never trust a bare
  # python3-config. Measured on a RunPod ubuntu-22.04 image: /usr/bin/python3
  # is 3.11 but /usr/bin/python3-config is 3.10's, so the Makefile's default
  # embedded libpython3.10 -- a newton-incapable interpreter -- while every
  # "python3 -V" check in this script said 3.11. The versioned -config is the
  # only spelling that cannot be shadowed like that.
  EMBED_PY=$(embed_python)
  PY_ARGS=""
  if [ "$EMBED_PY" != "python3" ] && command -v "${EMBED_PY}-config" >/dev/null 2>&1; then
    PY_ARGS="PYTHON_CONFIG=${EMBED_PY}-config"
    log "build: embedding $EMBED_PY ($PY_ARGS)"
  else
    PYV_SYS=$(python3 -c 'import sys;print("%d%02d"%sys.version_info[:2])' 2>/dev/null || echo 0)
    [ "$PYV_SYS" -ge 311 ]       || die "system python3 is <3.11 and no newer interpreter is installed. Run: bash scripts/install/linux_bootstrap.sh python"
    log "build: embedding the system python3 (python3-config)"
  fi
  # A STALE binary survives a changed PYTHON_CONFIG. make tracks file mtimes,
  # not flags: with objects and binary already present, a rebuild after
  # switching interpreters recompiles nothing and relinks nothing -- measured
  # (0 relinks, 0 recompiles) -- and the old libpython3.10 link is kept while
  # the log says "embedding python3.11". Only two TUs include Python.h, and
  # the Python C-API layout differs between minor versions, so both must be
  # recompiled, not merely relinked. Purge them and the binary when the
  # binary on disk links a different libpython than the one selected.
  WANT_PYV=$("${EMBED_PY}" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || true)
  HAVE_BIN="$OMNISIM_HOME/bin/omnisim-bin"
  if [ -n "$WANT_PYV" ] && [ -x "$HAVE_BIN" ]; then
    HAVE_PYV=$(ldd "$HAVE_BIN" 2>/dev/null | grep -oE 'libpython3\.[0-9]+' | head -1 | sed 's/libpython//')
    if [ -n "$HAVE_PYV" ] && [ "$HAVE_PYV" != "$WANT_PYV" ]; then
      log "build: existing omnisim-bin links python$HAVE_PYV but python$WANT_PYV was selected -- purging it and the Python-including objects so make rebuilds them"
      rm -f "$HAVE_BIN"
      find "$OMNISIM_HOME/src/omnisim" -type f \( -name 'OmNewtonBackend*.o' -o -name 'newton_embed_smoke*.o' \) -delete
    fi
  fi
  make -C "$OMNISIM_HOME" -j"$JOBS" release $PY_ARGS
  BIN="$OMNISIM_HOME/bin/omnisim-bin"
  [ -x "$BIN" ] || BIN="$(find "$OMNISIM_HOME" -maxdepth 3 -name 'omnisim-bin' -type f -perm -u+x | head -1)"
  [ -n "$BIN" ] && [ -x "$BIN" ] || die "omnisim-bin not produced"
  # The physics floor, asserted on the BINARY rather than trusted from flags:
  # whatever python3-config the make resolved is now baked into the link.
  PYLIB=$(ldd "$BIN" 2>/dev/null | grep -oE 'libpython3\.[0-9]+' | head -1 || true)
  case "$PYLIB" in
    libpython3.1[1-9]|libpython3.[2-9][0-9])
      log "build: engine embeds ${PYLIB#lib} (>=3.11, newton-capable)" ;;
    "")
      die "omnisim-bin links no libpython at all -- the Newton runtime cannot come up" ;;
    *)
      die "omnisim-bin links ${PYLIB#lib}, and newton 1.5.0 raises at ModelBuilder() before 3.11.
     Run: bash scripts/install/linux_bootstrap.sh python   then rebuild." ;;
  esac
  # A build can succeed and still have no renderer: the wgpu link is conditional
  # on WGPU_NATIVE_HOME resolving. Fail loudly here rather than letting the user
  # discover it later as an empty screenshot.
  [ -f "$OMNISIM_HOME/lib/webots/libwgpu_native.so" ] \
    || die "libwgpu_native.so is not in lib/webots -- this build has NO renderer. Run: bash scripts/install/linux_bootstrap.sh wgpu   and then rebuild."
  log "build: OK -> $BIN"
}

# The interpreter the ENGINE will import from: whatever libpython the binary
# links. On single-python systems this is plain python3; on dual-python cloud
# images it is NOT (see the header). Empirically caught on a RunPod pod:
# binary linked libpython3.10 while `python3 -m pip` fed a 3.11.
linked_python() {
  BIN="$OMNISIM_HOME/bin/omnisim-bin"
  if [ -x "$BIN" ]; then
    PYLIB=$(ldd "$BIN" 2>/dev/null | grep -oE 'libpython3\.[0-9]+' | head -1 || true)
    if [ -n "$PYLIB" ] && command -v "python${PYLIB#libpython}" >/dev/null 2>&1; then
      echo "python${PYLIB#libpython}"
      return
    fi
  fi
  echo python3
}

pip_flags_for() {
  # PEP-668 pips demand --break-system-packages; older pips reject the flag.
  #
  # --ignore-installed rides along on the SAME condition, and it is not
  # optional on a distro-managed interpreter: apt ships some of these wheels
  # itself (python3-typing-extensions on Ubuntu 24.04), and an apt-installed
  # package has no RECORD file, so pip cannot uninstall it to upgrade:
  #   ERROR: Cannot uninstall typing_extensions 4.10.0, RECORD file not found.
  #          Hint: The package was installed by debian.
  # That aborts the whole physics install. --ignore-installed makes pip install
  # over the distro copy instead of trying to remove it.
  if "$1" -m pip install --help 2>/dev/null | grep -q break-system-packages; then
    echo "--break-system-packages --ignore-installed"
  fi
}

# The interpreter the CONTROLLERS run in -- NOT necessarily the one above.
# Python controllers are spawned as a separate process, as plain `python3` from
# PATH (src/controller/launcher/launcher.c). The engine's embedded interpreter
# (linked_python) is a different thing entirely. On dual-python cloud images the
# two diverge, and then the deploy controllers -- which `import onnxruntime` to
# run their ONNX policies -- import from an interpreter nobody installed wheels
# into. Measured on a RunPod pod (2026-07-12): binary linked 3.10 (wheels landed
# there), controllers ran 3.11, `import onnxruntime` failed, and every ONNX
# deploy controller silently fell back to ZERO residual -- printing one warning
# line, walking the bare baseline, and exiting 0. That is a SILENT wrong answer:
# the demo "passes" while the policy under test never ran.
controller_python() { command -v python3 2>/dev/null || echo python3; }

phase_gpu() {
  # CRITICAL: the wheels go into the interpreter the BINARY LINKS (see header)
  # -- never a venv (the embedded interpreter ignores venvs). Package name is
  # `newton` (NOT `newton-physics`).
  PY=$(linked_python)
  # A deadsnakes interpreter arrives without pip. NOT ensurepip -- see
  # phase_python: as root it corrupts the shared dist-packages pip that the
  # system 3.10 reads. get-pip.py is version-scoped.
  if ! "$PY" -m pip --version >/dev/null 2>&1; then
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
    $SUDO "$PY" /tmp/get-pip.py
    rm -f /tmp/get-pip.py
  fi
  PIPFLAGS=$(pip_flags_for "$PY")
  # ⛔ Refuse 3.10 loudly rather than install a stack that cannot run. newton
  # 1.5.0 raises "Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool]."
  # at ModelBuilder() on CPython 3.10, so the engine comes up with NO physics --
  # the world loads and stands still, which is far harder to diagnose than a
  # failed install. Measured on 3.10.12 (fails) and 3.12.3 (works), same wheels.
  PYV_GPU=$("$PY" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
  case "$PYV_GPU" in
    3.10|3.9|3.8|3.7)
      die "the engine links Python $PYV_GPU, but newton 1.5.0 needs >= 3.11 in practice (it raises at ModelBuilder() on 3.10 despite declaring >=3.10). Run: bash scripts/install/linux_bootstrap.sh python   then rebuild -- the build embeds the newer interpreter automatically." ;;
  esac
  log "gpu: torch / warp / newton / mujoco_warp into $PY (the interpreter the binary links)"
  # $SUDO_H, not --user: the wheels have to land in the SYSTEM interpreter the
  # binary links. --user would install somewhere this script cannot prove the
  # embedded interpreter reads, and a wheel the engine cannot see fails silently
  # -- exactly the onnxruntime failure documented above.
  # torch is TRAINING-ONLY: `import torch` appears nowhere under src/ or
  # lib/controller/, and this script already says a miss is a warning, not a
  # failure. It is also ~2.5 GB from the CUDA index, which a GPU-less CI runner
  # can neither use nor afford. OMNISIM_SKIP_TORCH=1 opts out; the hard
  # inference dependency (onnxruntime) is installed either way below.
  # Default to AUTO rather than "install it": on a machine with no NVIDIA GPU
  # this is ~2.5 GB from the CUDA index for a package the engine never imports,
  # and it is by far the largest item in the install. The variable existed but
  # appeared in exactly two files -- this script and linux-build.yml -- and in
  # no document, so no interactive user ever set it.
  if [ "${OMNISIM_SKIP_TORCH:-auto}" = "auto" ]; then
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
      OMNISIM_SKIP_TORCH=0
    else
      OMNISIM_SKIP_TORCH=1
      log "gpu: no CUDA device visible -- skipping torch (training-only, ~2.5 GB)."
      log "gpu: set OMNISIM_SKIP_TORCH=0 to install it anyway."
    fi
  fi
  if [ "${OMNISIM_SKIP_TORCH:-0}" = "1" ]; then
    log "gpu: SKIPPING torch (OMNISIM_SKIP_TORCH=1) -- training-only, ~2.5 GB"
  else
    $SUDO_H "$PY" -m pip install $PIPFLAGS torch --index-url https://download.pytorch.org/whl/cu128
  fi
  # ⛔ PIN the physics stack to the repo's single source of truth
  # (scripts/packaging/newton_runtime_pins.py). An unpinned install broke a pod
  # on 2026-07-17: PyPI's newton had moved to 1.4.0, whose ModelBuilder.add_link()
  # dropped the `armature` kwarg the engine passed AT THE TIME, so EVERY world
  # died at load ("add_body raised: ... unexpected keyword argument 'armature'" →
  # newton-enforce FATAL). That specific breakage is now MOOT — the engine was
  # migrated to newton 1.5.0 and no longer passes `armature` to add_link — but
  # the rule stands: an unpinned physics stack silently desyncs train==deploy.
  PINS_PY="$OMNISIM_HOME/scripts/packaging/newton_runtime_pins.py"
  if [ -f "$PINS_PY" ]; then
    PHYS_SPECS=$("$PY" "$PINS_PY" | tr '\n' ' ')
  else
    # repo not fetched yet (gpu phase run standalone) — frozen copy of the SSOT;
    # keep in sync with newton_runtime_pins.py (tests/test_newton_pins_parity.py)
    # Kept identical to scripts/packaging/newton_runtime_pins.py. It had drifted:
    # newton-usd-schemas was added to the SSOT and not here, and without it
    # newton.add_usd hard-fails on this path.
    PHYS_SPECS="warp-lang==1.16.0 mujoco-warp==3.11.0 mujoco==3.11.0 newton==1.5.0 usd-core==26.5 newton-usd-schemas==0.5.0"
  fi
  log "gpu: physics stack pinned: $PHYS_SPECS"
  # `mujoco` is NOT in the SSOT (it arrives transitively via mujoco-warp), so it
  # is pinned HERE — outside the $PHYS_SPECS branch, so both the SSOT path and
  # the frozen-fallback path get it. It must match mujoco-warp's expected major.
  $SUDO_H "$PY" -m pip install $PIPFLAGS $PHYS_SPECS "mujoco==3.11.0" numpy onnx onnxscript onnxruntime

  log "gpu: verifying $PY can import the stack"
  "$PY" - <<'PY_EOF'
import warp, newton, mujoco
try:
    import torch
    print("torch    ", torch.__version__, "cuda:", torch.cuda.is_available(),
          torch.cuda.get_device_name(0) if torch.cuda.is_available() else "")
except ImportError:
    print("torch     NOT INSTALLED (training-only; fine unless you load .pt policies)")
print("warp     ", warp.config.version)
print("newton   ", getattr(newton, "__version__", "?"))
print("mujoco   ", mujoco.__version__)
import mujoco_warp; print("mujoco_warp OK")
PY_EOF

  # The engine's interpreter is now good. The CONTROLLERS' interpreter is a
  # separate process (see controller_python) and needs its own wheels, or the
  # ONNX deploy demos degrade SILENTLY to the bare baseline.
  CPY=$(controller_python)
  PYV=$("$PY" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
  CPYV=$("$CPY" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
  if [ "$PYV" != "$CPYV" ]; then
    log "gpu: controllers run on a DIFFERENT interpreter ($CPY = $CPYV, engine = $PYV)"
    log "gpu: installing the controller-side deps there too"
    CPIPFLAGS=$(pip_flags_for "$CPY")
    # scipy + mujoco: the shadowing fold/gate tooling (feasibility_certificate
    # imports scipy.optimize; build_quad_shadow_ghost imports mujoco) runs in
    # THIS interpreter on pods. Each missing wheel cost a campaign phase on
    # 2026-07-17 -- and the crash was misread as a gate FAIL.
    $SUDO_H "$CPY" -m pip install $CPIPFLAGS numpy scipy "mujoco==3.11.0" onnxruntime
  fi

  # HARD GATE. Without onnxruntime in the controller interpreter, every ONNX
  # deploy controller runs with ZERO residual and still exits 0 -- so we assert
  # rather than trust, and fail the install instead of shipping a silent lie.
  log "gpu: verifying the CONTROLLER interpreter ($CPY) can import onnxruntime"
  "$CPY" -c 'import onnxruntime, numpy; print("controller deps OK: onnxruntime", onnxruntime.__version__, "numpy", numpy.__version__)' \
    || die "controller interpreter ($CPY) cannot import onnxruntime -- ONNX deploy demos would silently run with ZERO residual"
  # torch is needed only by the handful of research controllers that load .pt
  # policies directly; a miss is a warning, not a failure.
  "$CPY" -c 'import torch' 2>/dev/null \
    || log "gpu: NOTE - $CPY has no torch; the few research controllers that load .pt policies will not run (ONNX deploy demos are fine)"
  log "gpu: OK"
}

phase_smoke() {
  # Acceptance test: a demo world loads and steps headless under Xvfb, with
  # Newton REQUIRED (fail loudly rather than silently falling back to ODE),
  # then the race-free backend-verdict sidecar is checked.
  log "smoke: headless demo world under Xvfb (OMNISIM_REQUIRE_NEWTON=1)"
  cd "$OMNISIM_HOME"
  export OMNISIM_HOME WEBOTS_HOME="$OMNISIM_HOME"
  export OMNISIM_LOG_PATH="$OMNISIM_HOME/omnisim_log.txt"

  # Linux runtime env (required): vendored Qt/ICU libs, XCB platform under
  # Xvfb, a writable tmpdir, and software GL for GPU-less/headless hosts.
  export LD_LIBRARY_PATH="$OMNISIM_HOME/lib/webots${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export QT_QPA_PLATFORM=xcb
  export WEBOTS_TMPDIR=/tmp
  export LIBGL_ALWAYS_SOFTWARE=1

  WORLD="${WORLD:-projects/samples/demos/worlds/showcase/warehouse_husky.omniworld}"
  [ -f "$WORLD" ] || die "world not found: $WORLD"

  # OMNISIM_REQUIRE_NEWTON=1 -> non-zero exit instead of a silent ODE fallback.
  export OMNISIM_REQUIRE_NEWTON="${OMNISIM_REQUIRE_NEWTON:-1}"

  # A COLD first run needs well over 15 s to reach world-finalize (45 s was
  # measured on WSL2). On a PRISTINE machine there is a second, bigger cold
  # cost: warp compiles its CUDA kernels on first use, which can take minutes
  # (measured on a fresh RunPod A4000: 45 s insufficient, ~5 min sufficient).
  # So: try DURATION, and if the runtime clearly loaded but finalize was not
  # reached, retry ONCE with a kernel-compile-sized window.
  # ONE run, --until-finalized, generous CEILING. This replaces a fixed 45 s
  # run plus a 300 s retry gated on a heuristic ("imports OK" AND "Newton
  # bodies" in the log). Measured 2026-08-28 on the ubuntu-22.04 CI leg: the
  # embedded 3.12 came up ("warp + newton imports OK; FFI smoke OK"), the
  # renderer came up, and the world still had not FINALISED at 45 s -- the
  # demo world declares mujoco_warp, whose kernels compile on first use --
  # so no "Newton bodies" line existed and the retry never fired. Verdict:
  # "Newton did NOT drive this run", on an install where it demonstrably had.
  # The 24.04 leg brushed the same edge from the other side ("FINALISED but
  # NEVER STEPPED": finalize inside the window, first step outside it).
  #
  # --until-finalized stops the moment finalize AND the first physics step
  # are observed, so a warm box pays seconds; --duration is then only the
  # ceiling a cold kernel compile may take, and --step-wait-timeout matches
  # it so the first step is given the same budget as the finalize. Both the
  # old variable names keep working; the larger of the two is the ceiling.
  DURATION="${DURATION:-45}"
  RETRY_DURATION="${RETRY_DURATION:-480}"
  CEILING=$(( DURATION > RETRY_DURATION ? DURATION : RETRY_DURATION ))

  rm -f "$OMNISIM_LOG_PATH" "$OMNISIM_LOG_PATH.newton.json"
  log "smoke: run-headless --until-finalized (ceiling ${CEILING}s; a cold warp kernel compile can take minutes)"
  # The renderer assertion below greps for the main view's lazy wgpu-native init line. Since
  # 2026-09-02 a --no-rendering run draws no main-view frame (and so never initialises wgpu)
  # unless asked to prove the renderer exists -- which is exactly what this smoke is for.
  export OMNISIM_RENDERER_PROBE=1
  xvfb-run -a --server-args="-screen 0 1280x1024x24" \
    python3 -m omnisim run-headless "$WORLD" \
      --until-finalized --duration "$CEILING" --step-wait-timeout "$CEILING" \
    || die "headless run failed (see $OMNISIM_LOG_PATH)"

  # RENDERER acceptance. The sidecar below proves PHYSICS drove the world; it
  # says nothing about whether anything could be drawn, and those two failed
  # independently here. Until 2026-08-28 the engine opted into wgpu's GL
  # backend by accident, and on a headless host wgpu-hal's GLES adapter
  # panicked inside Rust (egl.rs:182, BadAccess) -- a non-unwinding panic, so
  # the process aborted outright rather than degrading. Restricting the
  # instance to the primary backends removed that, but "does not abort" is not
  # the same claim as "renders", so assert the renderer came up.
  log "smoke: renderer (wgpu-native must reach instance + adapter + device)"
  if [ -n "${OMNISIM_NO_WINDOW:-}" ]; then
    # No-window mode builds no main view, so there is no main-view renderer to
    # assert; camera devices still render offscreen through wgpu.
    log "smoke: renderer check skipped -- OMNISIM_NO_WINDOW builds no main view"
  elif grep -q '\[OmWgpuBackend\] wgpu-native init OK' "$OMNISIM_LOG_PATH" 2>/dev/null; then
    grep -m1 'wgpu-native init OK' "$OMNISIM_LOG_PATH"
    grep -m1 -i 'adapter.*backend\|backend.*Vulkan\|rendering through the wgpu' "$OMNISIM_LOG_PATH" || true
    log "smoke: renderer OK"
  else
    grep -i 'wgpu\|vulkan\|adapter' "$OMNISIM_LOG_PATH" 2>/dev/null | head -20 || true
    die "wgpu-native did not initialise -- this build has NO renderer (physics may still be fine).
     On a GPU-less host this is usually a missing software Vulkan driver: the
     deps phase installs libvulkan1 + mesa-vulkan-drivers (lavapipe) for exactly
     this. Check with: vulkaninfo --summary
     To see which backends were offered:  OMNISIM_WGPU_BACKENDS=all
     To A/B the pre-2026-08-28 behaviour:  OMNISIM_WGPU_BACKENDS=gl  (expect an abort)"
  fi

  log "smoke: Newton verdict sidecar (the race-free 'did Newton drive it' signal)"
  if [ -f "$OMNISIM_LOG_PATH.newton.json" ]; then
    cat "$OMNISIM_LOG_PATH.newton.json"
    grep -q '"degraded": *false' "$OMNISIM_LOG_PATH.newton.json" || die "Newton ran DEGRADED (XPBD/FAILED solver fallback)"
    grep -q '"finalised": *true' "$OMNISIM_LOG_PATH.newton.json" || die "Newton did not finalise"
  else
    # (Wb|Om)NewtonBackend: the engine's C++ classes are being renamed Wb* -> Om*
    # and the bracketed log tag follows the class name, so match both prefixes.
    grep -iE '(Wb|Om)NewtonBackend|world finalised|import warp' "$OMNISIM_LOG_PATH" | tail -20 || true
    die "NO SIDECAR -> Newton did NOT drive this run (ODE fallback)"
  fi
  log "smoke: done"
}

case "$PHASE" in
  deps)   phase_deps ;;
  python) phase_python ;;
  fetch)  phase_fetch ;;
  wgpu)   phase_wgpu ;;
  build)  phase_build ;;
  gpu)    phase_gpu ;;
  smoke)  phase_smoke ;;
  all)    phase_deps; phase_python; phase_fetch; phase_wgpu; phase_build; phase_gpu; phase_smoke ;;
  *)      die "unknown phase: $PHASE (deps|python|fetch|wgpu|build|gpu|smoke|all)" ;;
esac
