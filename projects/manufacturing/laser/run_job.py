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

"""Run a job on a twin, headless, and keep what it made.

Starts the world in its own headless engine (with OMNISIM_MILLING_STOCK_DUMP and the
controller's trace pointed into --out), streams the G-code over the twin's GRBL TCP
port exactly as a sender does (grbl_link.GrblLink, character counting), waits for the
machine to finish and stand still, then stops the engine it started.

    python projects/manufacturing/laser/run_job.py \\
        projects/samples/demos/worlds/manufacturing/laser_longmill_laserbeam.omniworld \\
        projects/manufacturing/laser/jobs/tag_3mm_plywood.gcode --out .tmp/laser-run
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "projects/manufacturing/cnc"))

from grbl_link import GrblError, GrblLink  # noqa: E402


def _stop(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    if os.name == "nt":   # the engine and its controllers are this process's children
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    else:
        proc.terminate()
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()


def run(world: Path, job: Path, out: Path, port: int = 2323, timeout_s: float = 1800.0) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["OMNISIM_MILLING_STOCK_DUMP"] = str((out / "sheet.zmap").resolve())
    env["OMNISIM_CNC_TWIN_TRACE"] = str((out / "trace.jsonl").resolve())
    env["OMNISIM_LOG_PATH"] = str((out / "omnisim_log.txt").resolve())
    log = (out / "engine.out").open("w", encoding="utf-8")
    proc = subprocess.Popen([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", str(int(timeout_s))],
                            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    started = time.time()
    result = {"world": str(world), "job": str(job)}
    try:
        link = None
        while link is None:
            if proc.poll() is not None:
                raise RuntimeError(f"the engine exited before the twin listened (see {out / 'engine.out'})")
            try:
                link = GrblLink("127.0.0.1", port, connect_timeout_s=5.0)
            except (OSError, GrblError):     # the engine is still starting the controller
                time.sleep(1.0)
        result["connected_after_s"] = round(time.time() - started, 1)
        j = link.stream(job.read_text(encoding="utf-8"))
        while j.state == "running":
            if proc.poll() is not None:
                raise RuntimeError("the engine exited during the job")
            if time.time() - started > timeout_s:
                raise RuntimeError("the job did not finish in time")
            time.sleep(0.5)
        status = link.wait_idle(timeout_s=600.0)
        time.sleep(2.0)            # one more sheet dump (at most once a second while burning)
        result.update(job=j.snapshot(), status=status, twin=link.twin(),
                      wall_s=round(time.time() - started, 1))
        link.close()
    finally:
        _stop(proc)
        log.close()
    (out / "run.json").write_text(json.dumps(result, indent=1, default=str), encoding="utf-8")
    return result


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Run a G-code job on a twin world, headless, and keep the result.")
    p.add_argument("world")
    p.add_argument("job")
    p.add_argument("--out", required=True)
    p.add_argument("--port", type=int, default=2323)
    p.add_argument("--timeout", type=float, default=1800.0)
    a = p.parse_args(argv)
    r = run(Path(a.world), Path(a.job), Path(a.out), a.port, a.timeout)
    print(json.dumps({k: r.get(k) for k in ("job", "wall_s", "connected_after_s")}, default=str))
    return 0 if (r.get("job") or {}).get("state") == "done" else 1


if __name__ == "__main__":
    raise SystemExit(main())
