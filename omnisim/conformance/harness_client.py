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

"""Minimal HTTP client + lifecycle for the OmniSim validation harness.

Starts `scripts/harness/omnisim_harness.py` on its own ports, drives it over
HTTP (stdlib urllib), tears it down. Used by the conformance `harness` run mode
to measure render exposure + free-running robot displacement without writing a
controller.

Free-running on wall-clock by design: `/sim/step` proved fragile (returns 503
then hangs), and the world's own controllers drive the robots in real time, so
we load, let the sim free-run, and diff `/robots` poses. Displacement is
therefore a *coarse* "is it moving" signal, kept SOFT with a generous band.
"""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from ..paths import REPO_ROOT
from ..dev.runner import omnisim_env

_HARNESS = REPO_ROOT / "scripts" / "harness" / "omnisim_harness.py"


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])
    finally:
        s.close()


class HarnessError(RuntimeError):
    pass


class HarnessSession:
    """Context manager that owns a harness subprocess on isolated ports."""

    def __init__(self, start_timeout: float = 30.0):
        self.port = _free_port()
        self.sup_port = _free_port()
        while self.sup_port == self.port:
            self.sup_port = _free_port()
        self.start_timeout = start_timeout
        self.proc = None

    def _url(self, path: str) -> str:
        return f"http://127.0.0.1:{self.port}{path}"

    def _http(self, method: str, path: str, payload=None, timeout: float = 30.0):
        data = json.dumps(payload).encode() if payload is not None else None
        req = urllib.request.Request(self._url(path), data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                body = r.read()
                return getattr(r, "status", 200), json.loads(body or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, json.loads(exc.read() or b"{}")
            except Exception:
                return exc.code, {}

    def __enter__(self):
        cmd = [sys.executable, str(_HARNESS), "--host", "127.0.0.1",
               "--port", str(self.port), "--supervisor-port", str(self.sup_port)]
        self.proc = subprocess.Popen(cmd, env=omnisim_env(), cwd=str(REPO_ROOT),
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + self.start_timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise HarnessError(f"harness exited during startup (code {self.proc.returncode})")
            try:
                if self._http("GET", "/healthz", timeout=3)[0] == 200:
                    return self
            except Exception:
                pass
            time.sleep(0.5)
        raise HarnessError("harness did not become healthy in time")

    def load(self, world: str, wait_s: float = 45.0):
        return self._http("POST", "/world/load",
                          {"path": world, "wait_s": wait_s, "with_supervisor": True},
                          timeout=wait_s + 20)

    def robots(self):
        return self._http("GET", "/robots", timeout=20)

    def render_stats(self):
        return self._http("GET", "/world/render_stats", timeout=25)

    def world_error_count(self):
        code, res = self._http("GET", "/sim/events?types=world.error&limit=128", timeout=15)
        if code != 200:
            return None
        return len([e for e in res.get("events", []) if e.get("type") == "world.error"])

    def __exit__(self, *exc):
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=10)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
        return False
