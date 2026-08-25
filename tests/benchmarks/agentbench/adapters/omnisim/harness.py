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

"""OmniSim adapter: the live harness pass (AgentBench SPEC 2.3 "Phase A").

Starts ``scripts/harness/omnisim_harness.py`` on **per-run unique ports**
(SPEC 4.3: the 6789/6790 defaults collide and "a benchmark must not rely on"
the harness self-detecting that), loads the agent's world, and reads back the
robot roster and the contact set.

What the harness can and cannot answer, measured rather than assumed:

  * ``GET /robots`` gives node id, name, controller and joint count -- all
    static properties of the built scene, so free-running does not corrupt
    them. A1.1 / A1.2 / A1.4 are graded from here.
  * ``GET /sim/contacts`` and any "at t=0" question are NOT answerable here:
    the injected supervisor is ``synchronization FALSE`` and the engine runs
    ``--mode=fast``, so the world free-runs between RPCs. Measured on a
    10-Husky world: ``/world/load`` 36.9 s, ``/robots`` 22.1 s, by which point
    every robot had driven >10 m from spawn. A1.3 is therefore measured by the
    grader-owned recorder at a genuinely frozen t=0 (adapters/omnisim/
    headless.py); the harness numbers are still recorded, as observations.

Latencies are recorded in every row: SPEC 8.1.5 makes failures of our own
surface first-class results.
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.common.paths import REPO  # noqa: E402

HARNESS_SCRIPT = REPO / "scripts" / "harness" / "omnisim_harness.py"


def port_free(port, host="127.0.0.1", timeout=0.25):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def pick_port_pair(start=6900, end=7000):
    """A free (http, supervisor) pair. Both are pinned per run."""
    for p in range(start, end, 2):
        if port_free(p) and port_free(p + 1):
            return p, p + 1
    raise RuntimeError("no free harness port pair in [%d, %d)" % (start, end))


class HarnessProbe:
    """Result of one live harness pass. Never raises out of ``run``."""

    def __init__(self):
        self.ok = False
        self.error = None
        self.port = None
        self.load = None
        self.robots = []
        self.contacts = []
        self.timings = {}
        self.diagnostics = []

    def as_dict(self):
        return {"ok": self.ok, "error": self.error, "port": self.port,
                "n_robots": len(self.robots), "timings_s": self.timings,
                "diagnostics": self.diagnostics,
                "robots": self.robots, "contacts": self.contacts}


class HarnessSession:
    def __init__(self, run_dir, *, port=None, supervisor_port=None,
                 log_path=None):
        if port is None:
            port, supervisor_port = pick_port_pair()
        self.port = port
        self.supervisor_port = supervisor_port or (port + 1)
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = Path(log_path or (self.run_dir / "harness_engine.log"))
        self.stdout_path = self.run_dir / "harness_stdout.log"
        self.proc = None
        self._fh = None

    @property
    def base(self):
        return "http://127.0.0.1:%d" % self.port

    def start(self, wait_s=60.0):
        env = dict(os.environ)
        env["OMNISIM_HOME"] = str(REPO)
        env["WEBOTS_HOME"] = str(REPO)
        env["OMNISIM_LOG_PATH"] = str(self.log_path)
        self._fh = open(self.stdout_path, "w", encoding="utf-8",
                        errors="replace")
        kwargs = {}
        if os.name != "nt":
            kwargs["start_new_session"] = True
        self.proc = subprocess.Popen(
            [sys.executable, str(HARNESS_SCRIPT), "--port", str(self.port),
             "--supervisor-port", str(self.supervisor_port)],
            env=env, cwd=str(REPO), stdout=self._fh,
            stderr=subprocess.STDOUT, **kwargs)
        deadline = time.time() + wait_s
        while time.time() < deadline:
            try:
                self.get("/healthz", timeout=2.0)
                return True
            except Exception:
                if self.proc.poll() is not None:
                    return False
                time.sleep(0.4)
        return False

    def stop(self):
        if self.proc is not None:
            try:
                from agentbench.common.paths import engine_launch
                engine_launch.kill_tree(self.proc)
                self.proc.wait(timeout=30)
            except Exception:
                pass
            self.proc = None
        if self._fh is not None:
            try:
                self._fh.close()
            except Exception:
                pass
            self._fh = None

    # -- HTTP --------------------------------------------------------------
    def _call(self, path, body=None, timeout=600.0):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base + path, data=data,
            method="POST" if body is not None else "GET",
            headers={"Content-Type": "application/json"})
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                raw = r.read()
            code = 200
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            code = exc.code
        dt = time.perf_counter() - t0
        try:
            payload = json.loads(raw)
        except ValueError:
            payload = {"raw": raw[:400].decode("utf-8", "replace")}
        return dt, code, payload

    def get(self, path, timeout=600.0):
        return self._call(path, None, timeout)

    def post(self, path, body, timeout=900.0):
        return self._call(path, body, timeout)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


EXCLUDE_NAMES = {"harness_supervisor", "agentbench_recorder"}


def probe_world(world, run_dir, *, wait_s=300.0, want_contacts=True):
    """Load ``world`` under a private harness and read the roster back."""
    probe = HarnessProbe()
    sess = HarnessSession(run_dir)
    probe.port = sess.port
    try:
        t0 = time.perf_counter()
        if not sess.start():
            probe.error = "harness did not become healthy"
            probe.timings["start"] = time.perf_counter() - t0
            return probe
        probe.timings["start"] = time.perf_counter() - t0

        dt, code, payload = sess.post(
            "/world/load", {"path": str(Path(world).resolve()),
                            "wait_s": wait_s})
        probe.timings["world_load"] = dt
        probe.load = {"http": code, "ok": payload.get("ok")}
        probe.diagnostics = payload.get("diagnostics", []) or []
        if code != 200 or not payload.get("ok"):
            probe.error = "world/load failed (http %s)" % code
            return probe

        dt, code, payload = sess.get("/robots")
        probe.timings["robots"] = dt
        if code == 200:
            probe.robots = [r for r in payload.get("robots", [])
                            if (r.get("name") or "") not in EXCLUDE_NAMES]
        else:
            probe.error = "GET /robots http %s" % code
            return probe

        if want_contacts:
            dt, code, payload = sess.get("/sim/contacts")
            probe.timings["contacts"] = dt
            if code == 200:
                probe.contacts = payload.get("contacts", payload) \
                    if isinstance(payload, dict) else payload
        probe.ok = True
        return probe
    except Exception as exc:  # noqa: BLE001
        probe.error = repr(exc)
        return probe
    finally:
        sess.stop()
