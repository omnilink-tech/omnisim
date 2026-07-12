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

"""SpotEnv — Gymnasium environment wrapping an OmniSim subprocess.

Each instance launches a headless OmniSim process running
the spot_rl_agent controller, which exchanges fixed-size observation
and action packets over TCP. See spot_rl_agent.py for the protocol.

Usage:
    from spot_env import SpotEnv
    env = SpotEnv(env_id=0)
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(action)
    env.close()
"""

from __future__ import annotations

import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())  # projects/policies/research/envs -> repo
# Default training world (Newton physics since 2026-06-11). Override via
# SPOT_TRAIN_WORLD env var or the world_path constructor arg.
WORLD_PATH = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_rl.wbt"
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
TMP_DIR = Path(r"C:\tmp\omnisim_rl")

OBS_DIM = int(os.environ.get("SPOT_OBS_DIM", "50"))
ACT_DIM = 12

# TCP base port; env_id is added so multiple envs don't collide.
BASE_PORT = 7100


class SpotEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        env_id: int = 0,
        host: str = "127.0.0.1",
        omnisim_bin: Optional[Path] = None,
        world_path: Optional[Path] = None,
        connect_timeout_s: float = 90.0,
        verbose: bool = False,
    ):
        super().__init__()
        self.env_id = env_id
        self.host = host
        # Session-isolated port: two parallel agent sessions get disjoint
        # port bases (derived from OMNISIM_SESSION), so they never collide
        # and never need to kill each other's controllers. free_port_near
        # steps to the next free slot if our own preferred port is held by
        # a zombie -- it never kills the occupant.
        from session_isolation import env_port, free_port_near, session_id
        self.session = session_id()
        self.port = free_port_near(env_port(env_id, self.session))
        self.omnisim_bin = Path(omnisim_bin) if omnisim_bin else OMNISIM_BIN
        # Resolution order for the world path:
        #   1. constructor arg (explicit override)
        #   2. SPOT_TRAIN_WORLD env var (lets the launcher pick Newton
        #      vs ODE without code changes; subproc envs inherit the
        #      variable from the parent so all parallel envs get the
        #      same world)
        #   3. WORLD_PATH default (spot_rl.wbt, ODE)
        if world_path:
            self.world_path = Path(world_path)
        elif os.environ.get("SPOT_TRAIN_WORLD"):
            self.world_path = Path(os.environ["SPOT_TRAIN_WORLD"])
        else:
            self.world_path = WORLD_PATH
        self.connect_timeout_s = connect_timeout_s
        self.verbose = verbose

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(ACT_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float32)

        self.proc: Optional[subprocess.Popen] = None
        self.sock: Optional[socket.socket] = None
        # Cached most-recent unsent packet from agent. The protocol sends obs
        # first each tick, so after reset() we have a queued obs ready to
        # return without sending an action (the next step's action is sent
        # to advance the sim).
        self._pending_obs: Optional[np.ndarray] = None
        self._pending_reward: float = 0.0
        self._pending_done: bool = False

        self._cmd_bytes = 1 + ACT_DIM * 4
        self._obs_bytes = OBS_DIM * 4 + 4 + 1

        self._stdout_log: Optional[object] = None

        TMP_DIR.mkdir(parents=True, exist_ok=True)

    # ── Subprocess lifecycle ──────────────────────────────────────────

    def _materialize_world(self) -> Path:
        """Write a per-env copy of the world file with this env's TCP port
        baked into the controllerArgs. The original world's '7100' is
        replaced with the env's actual port -- avoids relying on env-var
        passthrough to the controller subprocess that Webots spawns."""
        template = self.world_path.read_text()
        replaced = template.replace('"7100"', f'"{self.port}"')
        # Per-session, per-env world path: include the session id so two
        # parallel agent sessions materializing the same source world
        # don't write (and launch from) the same dotfile. Kept next to
        # the original so its EXTERNPROTO relative paths still resolve.
        from session_isolation import scratch_world_name
        out_path = self.world_path.parent / scratch_world_name(
            self.world_path.stem, self.env_id, self.session)
        out_path.write_text(replaced)
        return out_path

    def _kill_port_squatter(self) -> None:
        """No-op retained for call-site compatibility.

        DELIBERATELY does nothing. The old implementation netstat'd our
        TCP port and Stop-Process'd whatever PID was listening -- which,
        with two parallel agent sessions sharing the box, would kill the
        OTHER session's live controller. Session-isolated ports
        (free_port_near in __init__) make squatting impossible by
        construction: we simply pick a port nobody is using, so there is
        never anything to kill. See projects/policies/research/envs/session_isolation.py.
        """
        return

    def _kill_port_squatter_DISABLED(self) -> None:
        if sys.platform != "win32":
            return
        import subprocess as _sp
        try:
            out = _sp.run(
                ["netstat", "-ano"],
                capture_output=True, text=True, timeout=5,
            ).stdout
        except Exception:
            return
        wanted = f"127.0.0.1:{self.port} "
        for line in out.splitlines():
            if "LISTENING" in line and wanted in line:
                parts = line.split()
                if not parts:
                    continue
                pid = parts[-1]
                if self.verbose:
                    print(f"[SpotEnv {self.env_id}] killing port {self.port} squatter pid={pid}")
                try:
                    _sp.run(
                        ["powershell.exe", "-Command",
                         f"Stop-Process -Id {pid} -Force -ErrorAction SilentlyContinue"],
                        timeout=3, check=False,
                    )
                except Exception:
                    pass
        time.sleep(0.5)  # give the OS a moment to release the port

    def _start_sim(self) -> None:
        """Spawn the OmniSim subprocess with the per-env world + isolated log."""
        if self.proc is not None and self.proc.poll() is None:
            return  # already running
        # Stagger launches per env_id so a DummyVecEnv reset() doesn't fire
        # 4 concurrent CreateProcess calls on the same .exe -- Windows
        # intermittently returns ERROR_ACCESS_DENIED in that case.
        time.sleep(0.5 * self.env_id)
        self._kill_port_squatter()  # no-op; kept for call-site stability
        world = self._materialize_world()
        from session_isolation import session_tmp_dir, sim_extern_port
        sdir = session_tmp_dir(self.session)
        env = os.environ.copy()
        env["OMNISIM_HOME"] = str(REPO_ROOT)
        # Per-session, per-env log so two parallel sessions don't fight
        # over the shared omnisim_log.txt lock or each other's logs.
        env["OMNISIM_LOG_PATH"] = str(sdir / f"spot_rl_{self.env_id}.log")
        env["SPOT_RL_PORT"] = str(self.port)  # belt+suspenders

        cmd = [
            str(self.omnisim_bin),
            str(world),
            "--batch",
            "--mode=fast",
            "--no-rendering",
            "--minimize",
            "--stdout",
            "--stderr",
            # OmniSim's own extern-controller port -- session-scoped so two
            # parallel sessions don't fight over it (the old fixed
            # 1300+env*10 collided across sessions).
            f"--port={sim_extern_port(self.env_id, self.session)}",
        ]
        stdout_path = sdir / f"spot_rl_{self.env_id}.stdout.log"
        self._stdout_log = open(stdout_path, "w")
        if self.verbose:
            print(f"[SpotEnv {self.env_id}] launching {cmd[0]} world={self.world_path.name} port={self.port}")
        last_err: Optional[Exception] = None
        for attempt in range(5):
            try:
                self.proc = subprocess.Popen(
                    cmd, env=env, stdout=self._stdout_log, stderr=subprocess.STDOUT
                )
                last_err = None
                break
            except PermissionError as e:
                # Windows intermittent ERROR_ACCESS_DENIED when several
                # processes launch the same .exe within microseconds.
                # Back off and retry.
                last_err = e
                time.sleep(0.6 * (attempt + 1))
        if last_err is not None:
            raise RuntimeError(
                f"SpotEnv {self.env_id}: subprocess.Popen failed after retries: {last_err}"
            )

    def _connect(self) -> None:
        """Connect the trainer-side socket to the controller's TCP server."""
        deadline = time.time() + self.connect_timeout_s
        last_err = None
        while time.time() < deadline:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = s
                if self.verbose:
                    print(f"[SpotEnv {self.env_id}] connected to {self.host}:{self.port}")
                return
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                last_err = e
                time.sleep(0.2)
        raise RuntimeError(f"SpotEnv {self.env_id}: could not connect to {self.host}:{self.port}: {last_err}")

    # ── Low-level I/O ─────────────────────────────────────────────────

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("controller closed connection")
            buf.extend(chunk)
        return bytes(buf)

    def _read_obs_packet(self) -> Tuple[np.ndarray, float, bool]:
        raw = self._recv_exact(self._obs_bytes)
        obs = np.frombuffer(raw, dtype=np.float32, count=OBS_DIM).copy()
        reward = struct.unpack_from("<f", raw, OBS_DIM * 4)[0]
        done = raw[OBS_DIM * 4 + 4] != 0
        return obs, float(reward), bool(done)

    def _send_cmd(self, tag: int, action: Optional[np.ndarray] = None) -> None:
        if action is None:
            action = np.zeros(ACT_DIM, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32).reshape(ACT_DIM)
        pkt = bytes([tag]) + action.tobytes()
        self.sock.sendall(pkt)

    # ── Gym API ───────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if self.proc is None or self.proc.poll() is not None:
            # Stagger N parallel env startups so they don't race each other
            # for OS resources during their initial OmniSim launch + bind.
            time.sleep(self.env_id * 0.5)
            self._start_sim()
            self._connect()
            # First packet from controller is the initial obs (no prior action).
            obs, reward, done = self._read_obs_packet()
            self._pending_obs = obs
            self._pending_reward = reward
            self._pending_done = done
            return obs, {}
        # Process already running: send RESET command, read post-reset obs.
        self._send_cmd(tag=1)
        # Controller processes RESET, then sends the next tick's obs.
        obs, reward, done = self._read_obs_packet()
        self._pending_obs = obs
        return obs, {}

    def step(self, action):
        # Send action; receive next obs + reward + done.
        self._send_cmd(tag=0, action=action)
        obs, reward, done = self._read_obs_packet()
        terminated = bool(done)
        truncated = False
        return obs, reward, terminated, truncated, {}

    def close(self):
        # Try to gracefully tell the controller to quit.
        try:
            if self.sock is not None:
                self._send_cmd(tag=2)
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        if self.proc is not None:
            # Kill the whole process tree (omnisim-bin spawns python.exe as
            # the controller; .terminate() on omnisim-bin doesn't reliably
            # take the controller down on Windows, and a stale controller
            # holds our TCP port hostage until killed).
            try:
                import psutil
                try:
                    parent = psutil.Process(self.proc.pid)
                    for child in parent.children(recursive=True):
                        try:
                            child.kill()
                        except Exception:
                            pass
                    parent.kill()
                except psutil.NoSuchProcess:
                    pass
            except ImportError:
                try:
                    self.proc.terminate()
                    try:
                        self.proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.proc.kill()
                        self.proc.wait()
                except Exception:
                    pass
            try:
                self.proc.wait(timeout=3)
            except Exception:
                pass
        self.proc = None
        if self._stdout_log is not None:
            try:
                self._stdout_log.close()
            except Exception:
                pass
            self._stdout_log = None


def make_spot_env(env_id: int = 0, **kwargs):
    """Factory for SubprocVecEnv compatibility."""
    def _thunk():
        return SpotEnv(env_id=env_id, **kwargs)
    return _thunk
