#!/usr/bin/env python3
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

"""g1_walk_jobserver -- persistent-sim job server (owner go-ahead 2026-07-06: faster iterations).

The simulator's cold boot (engine + embedded Python + CUDA graph + world build) costs 2-4 min
PER LAUNCH and dominated experiment wall time. This pymod keeps ONE simulator alive and runs
queued jobs (eval / record / short train bursts) back to back: boot is paid once per session.

Launch (a long-lived session; JOBSERVER=1 keeps the launcher watchdog from killing on DONE):
  bash projects/policies/training/run_walk_rl.sh 14400 jobsrv train headless \
    OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_jobserver:g1_walk_jobserver \
    JOBSERVER=1 JOBS_DIR=$OMNISIM_HOME/_scratch/jobs <shared physics envs...>

Submit: write <name>.job.json into JOBS_DIR:  {"env": {"EVAL_ONLY": "1", ...}}
  - job env is APPLIED ON TOP of the session env for that job, then restored;
  - result: <name>.result.txt ("done" | "error: ..."); the trainer's own logs stream to the
    session RES_LOG as usual.
Poll with scripts-side helper submit_job.py (writes, waits, tails).

v1 scope: each job re-runs the recipe's setup (maps/tensors/CUDA-graph recapture ~20-40 s) --
still 3-6x faster than a fresh boot. Idle timeout JOBS_IDLE_S (default 3600) ends the session.
"""
import json
import os
import time

from projects.policies.training import g1_residual_inengine as R
from projects.policies.training import g1_walk_recipe as RCP


def g1_walk_jobserver(world):
    if getattr(world, "_js_started", False):
        return
    if not hasattr(world, "_js_ctr"):
        world._js_ctr = 0
    world._js_ctr += 1
    if world._js_ctr < int(os.environ.get("WALK_WARM_TICKS", "360")):
        return
    world._js_started = True

    jobs_dir = os.environ.get("JOBS_DIR") or os.path.join(
        os.environ.get("OMNISIM_HOME", "."), "_scratch", "jobs")
    os.makedirs(jobs_dir, exist_ok=True)
    idle_s = float(os.environ.get("JOBS_IDLE_S", "3600"))
    res_log = os.environ.get("RES_LOG", "")
    R._log(world, "JOBSERVER up: dir=%s idle=%ds (submit <name>.job.json; boot paid once)"
           % (jobs_dir, int(idle_s)))
    last_work = time.time()
    n_done = 0
    while time.time() - last_work < idle_s:
        pend = sorted(f for f in os.listdir(jobs_dir) if f.endswith(".job.json"))
        if not pend:
            time.sleep(1.0)
            if res_log:                      # heartbeat so the launcher STALL watchdog stays calm
                try:
                    os.utime(res_log, None)
                except OSError:
                    pass
            continue
        jf = os.path.join(jobs_dir, pend[0])
        name = pend[0][:-len(".job.json")]
        try:
            job = json.load(open(jf))
        except Exception as e:
            open(os.path.join(jobs_dir, name + ".result.txt"), "w").write(f"error: bad job json {e!r}")
            os.remove(jf)
            continue
        os.remove(jf)
        overrides = {k: str(v) for k, v in job.get("env", {}).items()}
        saved = {k: os.environ.get(k) for k in overrides}
        os.environ.update(overrides)
        t0 = time.time()
        R._log(world, "JOB %s start: %s" % (name, " ".join(f"{k}={v}" for k, v in overrides.items())))
        try:
            import importlib
            importlib.reload(RCP)            # hot-reload: recipe code edits take effect per job
            world._wr_started = False        # let the recipe entry run again on this world
            RCP._walk_recipe_train(world)
            status = "done in %.0fs" % (time.time() - t0)
        except Exception as e:
            import traceback
            status = "error: %r" % (e,)
            R._log(world, "JOB %s ERROR: %s" % (name, traceback.format_exc()))
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        open(os.path.join(jobs_dir, name + ".result.txt"), "w").write(status)
        R._log(world, "JOB %s %s" % (name, status))
        n_done += 1
        last_work = time.time()
    R._log(world, "JOBSERVER idle timeout: %d jobs served -- shutting down" % n_done)
    RCP._status_write_final_done()
