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

"""What every showcase still shares: a world rewritten into showcase/worlds/ (its relative
URLs re-pointed), a photographer Camera added, one headless engine run, and the frames
downsampled to their published size.

The engine is run the way the repo's own render tests run it: `OMNISIM_NO_WINDOW=1` (no
window at all; Camera devices render through the wgpu offscreen path), `--batch
--mode=fast`, its own `OMNISIM_LOG_PATH`. Nothing opens on the desktop.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Callable, List, Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
WORLDS = HERE / "worlds"
OUT = ROOT / "docs/images/manufacturing"
SCRATCH = ROOT / ".tmp_showcase"          # engine logs and raw frames (not published)
BINARY = Path(os.environ.get("OMNISIM_BINARY") or ROOT / "msys64/mingw64/bin/omnisim-bin.exe")

_URL = re.compile(r'(url\s*(?:\[\s*)?)"([^"]+)"')
_EXTERN = re.compile(r'(EXTERNPROTO\s+)"([^"]+)"')


def _repoint(text: str, src_dir: Path, dst_dir: Path) -> str:
    """Re-point every relative url / EXTERNPROTO of a world moved from src_dir to dst_dir."""
    def fix(m):
        u = m.group(2)
        if re.match(r"^[a-z]+://", u) or os.path.isabs(u):
            return m.group(0)
        p = (src_dir / u).resolve()
        rel = os.path.relpath(p, dst_dir).replace("\\", "/")
        return f'{m.group(1)}"{rel}"'
    text = _URL.sub(fix, text)
    return _EXTERN.sub(fix, text)


def read_world(src: Path, dst_dir: Path = WORLDS) -> str:
    return _repoint(src.read_text(encoding="utf-8"), src.parent, dst_dir)


def photographer(shots_json: Path, out_dir: Path, width: int, height: int, fov: float = 0.6,
                 near: float = 0.01, far: float = 60.0, exposure: float = 1.0) -> str:
    """The photographer robot's node text: a kinematic supervisor carrying the camera."""
    args = f'"--shots" "{shots_json.as_posix()}" "--out" "{out_dir.as_posix()}"'
    return (f'DEF SHOWCASE_CAMERA Robot {{ name "showcase_camera" translation 0 0 5 supervisor TRUE '
            f'controller "showcase_camera" controllerArgs [ {args} ] children [ '
            f'Camera {{ name "showcase_camera" width {width} height {height} fieldOfView {fov} '
            f'near {near} far {far} exposure {exposure} antiAliasing TRUE }} ] }}\n')


def count_engines() -> int:
    try:
        out = subprocess.run(["tasklist"], capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.SubprocessError):
        return 0
    return sum(1 for ln in out.splitlines() if "omnisim-bin" in ln.lower())


def gpu_temp_c() -> Optional[float]:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=20).stdout
        return float(out.strip().splitlines()[0])
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def wait_for_room(max_engines: int = 6, max_gpu_c: float = 75.0) -> None:
    """The shared machine's rule: no launch while 6+ engines run or the GPU is at 75 C+."""
    while True:
        n, t = count_engines(), gpu_temp_c()
        if n < max_engines and (t is None or t < max_gpu_c):
            return
        print(f"[showcase] waiting: {n} engines, GPU {t} C", flush=True)
        time.sleep(15)


def engine_env(log: Path, extra: Optional[dict] = None) -> dict:
    env = dict(os.environ)
    env.update(OMNISIM_HOME=str(ROOT), OMNISIM_LOG_PATH=str(log), OMNISIM_NO_WINDOW="1",
               OMNISIM_SUN_MARKER_HIDDEN="1",
               OMNISIM_EXTRA_PROJECT_PATH=os.pathsep.join([str(HERE), str(ROOT / "projects/samples/demos")]))
    if os.name == "nt":
        runtime = BINARY.parent / "newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(BINARY.parent) + os.pathsep + env.get("PATH", "")
        site = runtime / "site-packages"
        if site.is_dir():
            env["PYTHONPATH"] = str(site) + os.pathsep + env.get("PYTHONPATH", "")
    env.update(extra or {})
    return env


def kill_tree(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    if os.name == "nt":       # the engine and its controllers are this process's children
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    else:
        proc.kill()
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        pass


def launch(world: Path, run_dir: Path, extra_env: Optional[dict] = None) -> subprocess.Popen:
    wait_for_room()
    run_dir.mkdir(parents=True, exist_ok=True)
    out = (run_dir / "engine.out").open("w", encoding="utf-8")
    env = engine_env(run_dir / "omnisim_log.txt", extra_env)
    return subprocess.Popen([str(BINARY), str(world), "--batch", "--mode=fast", "--stdout", "--stderr"],
                            cwd=ROOT, env=env, stdout=out, stderr=subprocess.STDOUT)


def run_dir(name: str) -> Path:
    return SCRATCH / name


def trigger_file(name: str) -> Path:
    """The file a shot with {"trigger": ...} waits for (see showcase_camera)."""
    return run_dir(name) / "trigger"


def shoot(name: str, world_text: str, shots: List[dict], width: int, height: int, fov: float = 0.6,
          timeout_s: float = 600.0, extra_env: Optional[dict] = None, quit: bool = True,
          near: float = 0.01, exposure: float = 1.0, far: float = 3.0,
          during: Optional[Callable[[subprocess.Popen, Path], None]] = None) -> Path:
    """Write the world with the photographer, run it until every shot is taken, return the run dir.

    far also sizes the camera's shadow map: its box is max(2, 0.75 far) m a side half-width on a
    2048^2 map (OmCamera.cpp), so a close product shot wants far ~3 (2 mm texels). `during(proc,
    run)` runs once the engine is up (e.g. streams a job and creates the trigger file)."""
    run = run_dir(name)
    if run.exists():
        shutil.rmtree(run)
    frames = run / "frames"
    frames.mkdir(parents=True)
    shots_json = run / "shots.json"
    shots_json.write_text(json.dumps({"quit": quit, "shots": shots}, indent=2), encoding="utf-8")
    WORLDS.mkdir(parents=True, exist_ok=True)
    world = WORLDS / f".gen_{name}.omniworld"
    world.write_text(world_text.rstrip() + "\n" + photographer(shots_json, frames, width, height, fov, near,
                                                                far=far, exposure=exposure),
                     encoding="utf-8", newline="\n")
    proc = launch(world, run, extra_env)
    t0 = time.time()
    try:
        if during is not None:
            during(proc, run)
        log = run / "omnisim_log.txt"
        while not (frames / "done").exists():
            if proc.poll() is not None:
                break
            dead = [ln for ln in (log.read_text(encoding="utf-8", errors="replace").splitlines() if log.exists() else [])
                    if "controller exited with status: 1" in ln]
            if dead:
                print(f"[showcase] {name}: a controller failed ({dead[0].strip()}); see {log}", flush=True)
                break
            if time.time() - t0 > timeout_s:
                print(f"[showcase] {name}: timed out after {timeout_s:.0f} s", flush=True)
                break
            time.sleep(1.0)
    finally:
        time.sleep(1.0)
        kill_tree(proc)
        world.unlink(missing_ok=True)
    print(f"[showcase] {name}: {len(list(frames.glob('*.png')))} frame(s) in {time.time() - t0:.0f} s", flush=True)
    return run


def publish(src: Path, dst: Path, width: int = 1600, crop: Optional[tuple] = None) -> Path:
    """Downsample a raw frame (rendered larger for anti-aliasing) to its published width."""
    from PIL import Image
    im = Image.open(src).convert("RGB")
    if crop:
        im = im.crop(crop)
    if im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
    dst.parent.mkdir(parents=True, exist_ok=True)
    im.save(dst, optimize=True)
    return dst
