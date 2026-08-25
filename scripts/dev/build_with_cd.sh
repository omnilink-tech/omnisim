#!/bin/bash
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

# Build OmniSim release. Pins cwd to the repo root (derived from this
# script's own location) because `bash -lc` resets cwd to ~/, breaking
# the relative `make -C src/omnisim`. Works from any directory on any
# machine — never hardcode the checkout path.
#
# OMNISIM_WITH_NEWTON=ON is REQUIRED for the Newton backend (the post-
# step joint-limit clamp, the Newton-specific physics opt-ins, the
# embedded-Python helper module). Without it the Makefile silently
# compiles to `Newton not available` stubs and the simulator falls
# back to ODE without any runtime warning — a real source of confusion
# (we burned a full debug cycle on this on 2026-05-26).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"
echo "[build_with_cd] pwd=$(pwd)"

# Auto-detect Python install (the Makefile default is Python314 but
# this machine has Python312). Caller can override either var.
# MSYS2 bash doesn't always export $USERNAME, fall back to $USER,
# then to whoami as a last resort.
USERNAME_FOR_PATH="${USERNAME:-${USER:-$(whoami)}}"
if [ -z "$PYTHON_HOME" ]; then
  for V in 314 312 311 310; do
    cand="C:/Users/${USERNAME_FOR_PATH}/AppData/Local/Programs/Python/Python${V}"
    if [ -f "${cand}/include/Python.h" ]; then
      PYTHON_HOME="$cand"
      PYTHON_LIB_DEFAULT="-lpython${V}"
      break
    fi
  done
fi
: "${PYTHON_LIB:=${PYTHON_LIB_DEFAULT:--lpython312}}"
echo "[build_with_cd] PYTHON_HOME=$PYTHON_HOME PYTHON_LIB=$PYTHON_LIB"

# Pass build flags as make COMMAND-LINE args, not env prefixes: env-inherited
# vars don't reliably reach the recursive `make -C` (OMNISIM_WITH_VULKAN set in
# a caller's env was silently dropped, compiling the WB_WGPU_NATIVE_AVAILABLE=0
# stub even though the dep was installed). Command-line args are unambiguous and
# override the Makefile's `?=` defaults. wgpu-ON build:
#   OMNISIM_WITH_VULKAN=ON WGPU_NATIVE_HOME=$PWD/_scratch/wgpu-native \
#     bash scripts/dev/build_with_cd.sh
echo "[build_with_cd] VULKAN=${OMNISIM_WITH_VULKAN:-OFF} WGPU_NATIVE_HOME=${WGPU_NATIVE_HOME:-<unset>}"
# PARALLELISM: this make invocation carried no -j for its whole life, so
# every build compiled ~800 TUs one at a time. Measured on a 16-core box:
# 240 s with -j16 vs ~1000+ s serial (4-5x). Defaults to the core count;
# set OMNISIM_JOBS=1 to serialize (e.g. to read an error stream in order).
OMNISIM_JOBS="${OMNISIM_JOBS:-$(nproc 2>/dev/null || echo 4)}"
echo "[build_with_cd] parallel jobs: $OMNISIM_JOBS"
make -C src/omnisim release -j"$OMNISIM_JOBS" \
  OMNISIM_WITH_CUDA=OFF \
  OMNISIM_WITH_NEWTON="${OMNISIM_WITH_NEWTON:-ON}" \
  OMNISIM_WITH_VULKAN="${OMNISIM_WITH_VULKAN:-OFF}" \
  WGPU_NATIVE_HOME="${WGPU_NATIVE_HOME:-}" \
  PYTHON_HOME="$PYTHON_HOME" \
  PYTHON_LIB="$PYTHON_LIB" \
  $OMNISIM_EXTRA_MAKE_ARGS
