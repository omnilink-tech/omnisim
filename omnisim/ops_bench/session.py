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
"""One owned REALTIME simulator per episode, a fixture supervisor, a sampler.

REALTIME, not FAST: this suite schedules messages "0.5 m into a drive" and
orders like "in 15 seconds", so the operator's clock and the robot's clock
must be the same clock. Sim time is still recorded on every sample.

The fixture supervisor is the harness's own `harness_supervisor`, injected
into a private copy of the world exactly as the harness injects it, on a
private port. The agent never sees it: it is how the TEST moves props and
shoves the robot, not a tool.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import uuid

from omnisim.paths import REPO_ROOT, resolve_omnisim_binary, linux_runtime_env
from omnisim.control_bench.engine import InfrastructureError, request
from .suite import PROPS, ROBOTS

SUPERVISOR_STANZA = """
Robot {
  name "harness_supervisor"
  controller "harness_supervisor"
  controllerArgs [ "--light" ]
  supervisor TRUE
  synchronization FALSE
}
"""


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0)); return s.getsockname()[1]


def prop_vrml(name, park):
    p = PROPS[name]
    sx, sy, sz = p["size"]; r, g, b = p["color"]
    return f"""
DEF OPS_{name.upper()} Solid {{
  translation {park[0]} {park[1]} {sz / 2 + 0.01}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.7 metalness 0 }}
                     geometry Box {{ size {sx} {sy} {sz} }} }} ]
  name "ops_{name}"
  boundingObject Box {{ size {sx} {sy} {sz} }}
  physics Physics {{ mass {p['mass']} }}
}}
"""


class Session:
    """Private engine + bridge + fixture supervisor. Nothing here is shared."""

    def __init__(self, robot, directory, key, props=(), relay_env=None, mode="realtime"):
        if not key: raise ValueError("An OmniKey is required for every live episode")
        self.robot = robot
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.proc = self.world = self.log = None
        self.port, self.sup_port = _free_port(), _free_port()
        self.url = f"http://127.0.0.1:{self.port}"
        self.park = ROBOTS[robot]["park"]
        self.props = list(props)
        source = REPO_ROOT / "projects/samples/demos/worlds/chat" / ROBOTS[robot]["world"]
        text = source.read_text(encoding="utf-8")
        if '"8765"' not in text: raise ValueError("World has no replaceable bridge port")
        text = text.replace('"8765"', f'"{self.port}"') + SUPERVISOR_STANZA
        if self.props:
            # Under Newton a URDFRobot's chassis is a 1 mm placeholder and only
            # its wheels collide, so the Husky's front wheels ended INSIDE the
            # block (calibration-01). The per-world opt-in gives the chassis
            # its own URDF box; safe for the Husky, whose box clears the
            # wheel-contact line. Only worlds with props need it.
            if "WorldInfo {" not in text: raise ValueError("World has no WorldInfo")
            text = text.replace("WorldInfo {", "WorldInfo {\n  newtonRobotColliders TRUE", 1)
        for i, name in enumerate(self.props):
            text += prop_vrml(name, [self.park[0], self.park[1] - 1.5 * i])
        self.world = source.with_name(f".ops_bench_{uuid.uuid4().hex}.omniworld")
        self.world.write_text(text, encoding="utf-8")
        binary = resolve_omnisim_binary()
        if not binary: raise InfrastructureError("OmniSim binary not found")
        env = linux_runtime_env(REPO_ROOT)
        env.update(OMNISIM_HOME=str(REPO_ROOT), OMNISIM_NO_WINDOW="1",
                   WARP_CACHE_PATH=str(REPO_ROOT / ".tmp/control-bench-warp-cache"),
                   OMNISIM_LOG_PATH=str(self.directory / "engine.log"), OMNI_KEY=key,
                   OMNISIM_HARNESS_SUPERVISOR_PORT=str(self.sup_port),
                   # The damage tracker must not act on the robot under test.
                   OMNISIM_HARNESS_DAMAGE_ROBOT="__ops_bench_none__",
                   OMNILINK_INTENT_STATE_DIR=str(self.directory / "intents"),
                   OMNILINK_TRACE=str(self.directory / "relay_trace.jsonl"))
        # Isolation defaults; an arm overrides only what it declares.
        env.update(OMNILINK_MEMORY="0", OMNILINK_PROFILE_SYNC="0", OMNILINK_EDGE="0",
                   OMNILINK_USAGE="0", OMNILINK_PRESENCE="0", OMNILINK_EVENT_WAKE="0")
        env.update({k: str(v) for k, v in (relay_env or {}).items()})
        self.env_used = {k: env[k] for k in sorted(env) if k.startswith(("OMNILINK_", "OMNISIM_HARNESS"))}
        runtime = Path(binary).parent / "newton-runtime"
        env["PATH"] = os.pathsep.join([str(runtime), str(Path(binary).parent), env.get("PATH", "")])
        self.log = (self.directory / "stdout.log").open("w", encoding="utf-8")
        try:
            self.proc = subprocess.Popen(
                [binary, "--batch", f"--mode={mode}", "--no-rendering", "--minimize",
                 "--stdout", "--stderr", str(self.world)],
                cwd=REPO_ROOT, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            self._wait_ready()
        except BaseException:
            self.close(); raise

    # ── readiness ────────────────────────────────────────────────────
    def _wait_ready(self):
        end = time.monotonic() + 120
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                raise InfrastructureError("Owned engine exited during startup")
            logfile = self.directory / "engine.log"
            if logfile.exists():
                t = logfile.read_text(encoding="utf-8", errors="replace")
                if "NO PHYSICS BACKEND IS AVAILABLE" in t or "FATAL:" in t:
                    raise InfrastructureError("Physics startup failed; see engine.log")
            try:
                st = self.state()
                sidecar = self.directory / "engine.log.newton.json"
                if sidecar.exists() and st.get("mode") in (None, "idle", "hold"):
                    proof = json.loads(sidecar.read_text())
                    if not proof.get("finalised") or proof.get("degraded"):
                        raise InfrastructureError("No non-degraded physics runtime")
                    self.sup("ping")
                    self.physics = proof
                    time.sleep(0.5)
                    return
            except (InfrastructureError, OSError, ValueError):
                pass
            time.sleep(0.25)
        raise InfrastructureError("Bridge/supervisor did not become ready with physics proof")

    # ── the robot's own surface (what an agent uses) ─────────────────
    def state(self):
        st = request(self.url + "/state", timeout=5)
        for k in ("x", "y", "yaw"):
            if not isinstance(st.get(k), (int, float)):
                raise ValueError("Missing measured pose")
        return st

    def post(self, path, body, timeout=90):
        return request(self.url + path, body, timeout=timeout)

    def tool(self, name, args, utterance=""):
        body = {"tool": name, **args, "surface": "mobile"}
        if name in ("drive_forward", "turn", "drive_to"): body["wait"] = True
        if utterance: body["utterance"] = utterance
        return self.post("/tool", body, timeout=120)

    # ── the fixture supervisor (what the TEST uses) ──────────────────
    def sup(self, cmd, args=None, timeout=10):
        with socket.create_connection(("127.0.0.1", self.sup_port), timeout=timeout) as s:
            body = json.dumps({"id": 1, "cmd": cmd, "args": args or {}}).encode()
            s.sendall(struct.pack(">I", len(body)) + body)
            head = self._exact(s, 4)
            reply = json.loads(self._exact(s, struct.unpack(">I", head)[0]))
        if not reply.get("ok"):
            raise InfrastructureError(f"supervisor {cmd}: {reply.get('error')}")
        return reply.get("result", {})

    @staticmethod
    def _exact(s, n):
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk: raise InfrastructureError("supervisor closed the connection")
            buf += chunk
        return buf

    def prop_position(self, name):
        node = self.sup("scene_node", {"def": f"OPS_{name.upper()}"})
        return node.get("position")

    def place(self, name, xy):
        z = PROPS[name]["size"][2] / 2 + 0.01
        return self.sup("scene_set_pose", {"def": f"OPS_{name.upper()}",
                                           "translation": [xy[0], xy[1], z],
                                           "rotation": [0, 0, 1, 0], "reset_physics": True})

    def shove(self, dx, dy):
        """Displace the robot bodily, as a collision or a push would."""
        robot_def = {"husky": "HUSKY", "tb3_burger": "TB3_BURGER"}[self.robot]
        node = self.sup("scene_node", {"def": robot_def})
        p = node["position"]
        return self.sup("scene_set_pose", {"def": robot_def,
                                           "translation": [p[0] + dx, p[1] + dy, p[2]],
                                           "reset_physics": True})

    def restart_robot(self, timeout_s=90.0):
        """Restart the robot's controller -- the bridge, and an OmniLink relay
        inside it -- and wait until it answers again. Returns the seconds it
        was down. What survives is whatever each system persists itself."""
        robot_def = {"husky": "HUSKY", "tb3_burger": "TB3_BURGER"}[self.robot]
        t0 = time.monotonic()
        self.sup("restart_controller", {"def": robot_def})
        # First see the old process go (a state() right after the command can
        # still reach it -- the smoke test measured a "2.0 s" gap that was just
        # this sleep), then wait for the new one to answer.
        gone_by = time.monotonic() + 15.0
        while time.monotonic() < gone_by:
            try:
                self.state()
                time.sleep(0.2)
            except InfrastructureError:
                break
        while time.monotonic() - t0 < timeout_s:
            try:
                self.state()
                return round(time.monotonic() - t0, 1)
            except InfrastructureError:
                time.sleep(1.0)
        raise InfrastructureError(f"bridge did not come back within {timeout_s:.0f} s of a restart")

    def events(self, since=0):
        try:
            return request(f"{self.url}/events?since={since}", timeout=5)
        except InfrastructureError as exc:
            return {"error": str(exc)}

    # ── teardown ─────────────────────────────────────────────────────
    def close(self):
        errors = []
        if self.proc is not None and self.proc.poll() is None:
            import psutil
            try:
                parent = psutil.Process(self.proc.pid)
                owned = parent.children(recursive=True) + [parent]
                for p in owned:
                    try: p.terminate()
                    except psutil.NoSuchProcess: pass
                _, alive = psutil.wait_procs(owned, timeout=4)
                for p in alive:
                    try: p.kill()
                    except psutil.NoSuchProcess: pass
                self.proc.wait(timeout=5)
            except psutil.NoSuchProcess:
                pass
            except (psutil.AccessDenied, subprocess.TimeoutExpired) as exc:
                errors.append(type(exc).__name__)
                try: self.proc.terminate(); self.proc.wait(timeout=5)
                except (OSError, subprocess.TimeoutExpired) as e2: errors.append(type(e2).__name__)
        if self.log: self.log.close()
        if self.world: self.world.unlink(missing_ok=True)
        return errors


class Sampler:
    """Continuous pose trace at ~30 ms wall intervals, from the robot's /state.

    With `sim_clock`, the episode's clock is SIMULATOR time: every trigger,
    fired step and sample time is in simulated seconds since the first
    sample, matching a runtime that runs on OMNILINK_CLOCK=sim. Polling is
    faster (10 ms wall) because the world can run several times real time.
    """

    def __init__(self, session, period=0.03, sim_clock=False):
        self.session, self.sim_clock = session, sim_clock
        self.period = 0.01 if sim_clock else period
        self.t0 = time.monotonic()
        self.sim0 = None
        self.samples, self.errors = [], []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def now(self):
        if self.sim_clock:
            last = self.samples[-1] if self.samples else None
            return 0.0 if last is None else last["t"]
        return time.monotonic() - self.t0

    def _sample_t(self, st):
        if not self.sim_clock:
            return self.now()
        sim = float(st.get("sim_time") or 0.0)
        if self.sim0 is None:
            self.sim0 = sim
        return sim - self.sim0

    def _run(self):
        misses = 0
        while not self._stop.wait(self.period):
            try:
                st = self.session.state()
                self.samples.append({"t": self._sample_t(st), "sim": st.get("sim_time"),
                                     "x": st["x"], "y": st["y"], "yaw": st["yaw"],
                                     "mode": st.get("mode")})
                misses = 0
            except Exception as exc:
                misses += 1
                # A single dropped read is noise; a run of them is a blind spot.
                if misses >= 5: self.errors.append(type(exc).__name__)

    def latest(self):
        return self.samples[-1] if self.samples else None

    def stop(self):
        self._stop.set(); self._thread.join(timeout=3)
        if self._thread.is_alive(): self.errors.append("sampler_not_stopped")
