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
# SUPPORTED TARGET: Ubuntu 24.04 (system Python 3.12).
#
# The two venues this header used to name as "verified end-to-end" are both
# unsupported now, and naming them was misleading:
#   * Ubuntu 22.04.5 / py3.10 -- REFUSED by phase_gpu's own guard below, because
#     newton 1.5.0 raises TypeError at ModelBuilder() on 3.10. A build there
#     loads worlds and nothing moves.
#   * Ubuntu 26.04 / py3.14   -- passes that guard but is wheel-fragile; the
#     pinned physics stack has no guaranteed 3.14 wheels.
# Both are measured by .github/workflows/physics-runtime-check.yml.
#
#   bash scripts/install/linux_bootstrap.sh deps    # apt prerequisites
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
  make -C "$OMNISIM_HOME" -j"$JOBS" release
  BIN="$OMNISIM_HOME/bin/omnisim-bin"
  [ -x "$BIN" ] || BIN="$(find "$OMNISIM_HOME" -maxdepth 3 -name 'omnisim-bin' -type f -perm -u+x | head -1)"
  [ -n "$BIN" ] && [ -x "$BIN" ] || die "omnisim-bin not produced"
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
  PIPFLAGS=$(pip_flags_for "$PY")
  # ⛔ Refuse 3.10 loudly rather than install a stack that cannot run. newton
  # 1.5.0 raises "Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool]."
  # at ModelBuilder() on CPython 3.10, so the engine comes up with NO physics --
  # the world loads and stands still, which is far harder to diagnose than a
  # failed install. Measured on 3.10.12 (fails) and 3.12.3 (works), same wheels.
  PYV_GPU=$("$PY" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo "?")
  case "$PYV_GPU" in
    3.10|3.9|3.8|3.7)
      die "the engine links Python $PYV_GPU, but newton 1.5.0 needs >= 3.11 in practice (it raises at ModelBuilder() on 3.10 despite declaring >=3.10). Use Ubuntu 24.04 (python 3.12), or rebuild the engine against a newer interpreter with PYTHON_HOME=." ;;
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
  DURATION="${DURATION:-45}"
  RETRY_DURATION="${RETRY_DURATION:-300}"

  run_smoke_once() {
    rm -f "$OMNISIM_LOG_PATH" "$OMNISIM_LOG_PATH.newton.json"
    xvfb-run -a --server-args="-screen 0 1280x1024x24" \
      python3 -m omnisim run-headless "$WORLD" --duration "$1"
  }

  run_smoke_once "$DURATION" || true
  if [ ! -f "$OMNISIM_LOG_PATH.newton.json" ] \
     && grep -q 'imports OK' "$OMNISIM_LOG_PATH" 2>/dev/null \
     && grep -q 'Newton bodies' "$OMNISIM_LOG_PATH" 2>/dev/null; then
    log "smoke: runtime loaded but no finalize in ${DURATION}s -- cold warp kernel compile; retrying once with ${RETRY_DURATION}s"
    run_smoke_once "$RETRY_DURATION" || die "headless run failed on the long retry"
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
  deps)  phase_deps ;;
  fetch) phase_fetch ;;
  wgpu)  phase_wgpu ;;
  build) phase_build ;;
  gpu)   phase_gpu ;;
  smoke) phase_smoke ;;
  all)   phase_deps; phase_fetch; phase_wgpu; phase_build; phase_gpu; phase_smoke ;;
  *)     die "unknown phase: $PHASE (deps|fetch|wgpu|build|gpu|smoke|all)" ;;
esac
