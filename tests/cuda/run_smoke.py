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

"""Run the CUDA M0 smoke test (build then execute).

This is the agent-facing entry point: builds `cuda_smoke` with `nvcc` and runs
it. Returns 0 on success, 1 on real failure, and 77 if no CUDA device is
present on this machine (the autotools "skipped" convention — agents and CI
should treat this as not-a-failure on non-NVIDIA boxes).

The runner invokes nvcc directly rather than going through `make`, so it
works on bare Windows + Python without needing MSYS2 on PATH. On Windows,
nvcc requires MSVC's `cl.exe` as the host compiler — if it isn't already on
PATH, the runner locates a VS install and sources `vcvars64.bat` before
calling nvcc.

See docs/developer/physics-roadmap.md §2 (M0).
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXE_NAME = "cuda_smoke.exe" if os.name == "nt" else "cuda_smoke"

# Target SM list per the plan: Turing through Hopper.
NVCC_SM_FLAGS = [
    "-gencode=arch=compute_75,code=sm_75",
    "-gencode=arch=compute_80,code=sm_80",
    "-gencode=arch=compute_86,code=sm_86",
    "-gencode=arch=compute_89,code=sm_89",
    "-gencode=arch=compute_90,code=sm_90",
]


def _find_vcvars64() -> Path | None:
    """Locate a VS install with a usable vcvars64.bat. Returns None if none found."""
    candidates = [
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"),
    ]
    for path in candidates:
        if path.is_file():
            return path
    return None


def _build_windows(nvcc: str, src: Path, exe: Path) -> subprocess.CompletedProcess:
    """nvcc on Windows needs cl.exe in PATH. Source vcvars64.bat if missing."""
    nvcc_args = [nvcc, "-O2", "-std=c++14", *NVCC_SM_FLAGS, str(src), "-o", str(exe)]
    if shutil.which("cl") is not None:
        print("[run_smoke] building (cl.exe already on PATH):", " ".join(nvcc_args))
        return subprocess.run(nvcc_args, capture_output=True, text=True)

    vcvars = _find_vcvars64()
    if vcvars is None:
        return subprocess.CompletedProcess(
            args=nvcc_args,
            returncode=1,
            stdout="",
            stderr="[run_smoke] FAIL: cl.exe not on PATH and no Visual Studio "
                   "install with vcvars64.bat was found. Install VS Build Tools "
                   "or open a Developer Command Prompt before running.\n",
        )
    quoted = " ".join(f'"{a}"' if " " in a else a for a in nvcc_args)
    cmd_line = f'call "{vcvars}" >NUL && {quoted}'
    print(f"[run_smoke] building via vcvars64.bat ({vcvars}):")
    print("           ", " ".join(nvcc_args))
    # shell=True with a single string lets cmd.exe parse the quoted vcvars
    # path correctly. Passing a list to ["cmd","/c",...] mangles the quotes.
    return subprocess.run(cmd_line, shell=True, capture_output=True, text=True)


def _standalone_smoke() -> int:
    """Build cuda_smoke.cu via nvcc and run it. Returns its exit code."""
    nvcc = shutil.which("nvcc")
    if nvcc is None:
        print("[run_smoke] SKIP standalone: nvcc not on PATH.")
        return 77

    src = HERE / "cuda_smoke.cu"
    exe = HERE / EXE_NAME
    if os.name == "nt":
        build = _build_windows(nvcc, src, exe)
    else:
        cmd = [nvcc, "-O2", "-std=c++14", *NVCC_SM_FLAGS, str(src), "-o", str(exe)]
        print("[run_smoke] building:", " ".join(cmd))
        build = subprocess.run(cmd, capture_output=True, text=True)

    if build.returncode != 0:
        print(build.stdout)
        print(build.stderr, file=sys.stderr)
        print("[run_smoke] FAIL: nvcc build returned", build.returncode)
        return 1
    if build.stdout:
        print(build.stdout.strip())
    if not exe.exists():
        print(f"[run_smoke] FAIL: expected binary {exe} was not produced.")
        return 1

    print(f"[run_smoke] running {exe} ...")
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    print(run.stdout, end="")
    if run.stderr:
        print(run.stderr, end="", file=sys.stderr)
    return run.returncode


def _in_process_smoke() -> int:
    """Run a brief headless world with OMNISIM_CUDA_SMOKE=1 and grep the log.

    Verifies that WbCudaBuffer round-trips inside the actual omnisim-bin ABI —
    complementary to the standalone test which only proves nvcc + the GPU
    work in isolation. Returns 0 on PASS, 1 on FAIL, 77 if omnisim-bin or the
    headless runner is unavailable.
    """
    repo_root = HERE.parents[1]
    headless_runner = repo_root / "scripts" / "dev" / "headless_runner.py"
    world = repo_root / "projects" / "samples" / "demos" / "worlds" / "cube_bot.wbt"
    log_path = repo_root / "omnisim_log.txt"

    if not headless_runner.exists() or not world.exists():
        print("[run_smoke] SKIP in-process: headless_runner.py or cube_bot.wbt missing.")
        return 77
    binaries = [
        repo_root / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
        repo_root / "bin" / "omnisim-bin",
        repo_root / "Contents" / "MacOS" / "webots",
    ]
    if not any(b.exists() for b in binaries):
        print("[run_smoke] SKIP in-process: omnisim-bin not built.")
        return 77

    env = os.environ.copy()
    env["OMNISIM_CUDA_SMOKE"] = "1"
    env["OMNISIM_STRUCTURED_LOG"] = "1"
    env.setdefault("OMNISIM_HOME", str(repo_root))
    cmd = [sys.executable, str(headless_runner), str(world), "--duration", "3"]
    print("[run_smoke] in-process: launching headless with OMNISIM_CUDA_SMOKE=1 ...")
    run = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if run.returncode != 0:
        print(run.stdout)
        print(run.stderr, file=sys.stderr)
        print("[run_smoke] FAIL in-process: headless runner returned", run.returncode)
        return 1

    if not log_path.exists():
        print(f"[run_smoke] FAIL in-process: {log_path} not produced.")
        return 1
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    if "CUDA smoke PASS" in log_text:
        print("[run_smoke] in-process PASS: WbCudaBuffer round trip verified inside omnisim-bin.")
        return 0
    if "CUDA smoke FAIL" in log_text:
        print("[run_smoke] FAIL in-process: PASS line absent, FAIL line present.")
        return 1
    print("[run_smoke] FAIL in-process: no 'CUDA smoke PASS' line in omnisim_log.txt.")
    return 1


def main() -> int:
    standalone = _standalone_smoke()
    if standalone not in (0, 77):
        return standalone
    in_process = _in_process_smoke()
    if in_process not in (0, 77):
        return in_process
    if standalone == 77 and in_process == 77:
        return 77
    return 0


if __name__ == "__main__":
    sys.exit(main())
