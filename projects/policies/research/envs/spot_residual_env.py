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

"""SpotResidualEnv — Gymnasium env for residual-RL on Spot.

Mirrors the SpotEnv TCP protocol but with a smaller observation/action
shape: 18-dim obs and 12-dim action (per-leg foot offsets, ±1 each,
scaled to ±3 cm by the controller). The controller wraps the model
walker (gait + IK + balance) so the policy only commands a small
residual.

The OmniSim world (spot_residual_train.wbt) launches the
spot_residual_agent controller.
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


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
WORLD_PATH = REPO_ROOT / "projects" / "rl" / "worlds" / "spot_residual_train.wbt"
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
TMP_DIR = Path(r"C:\tmp\omnisim_rl_res")

OBS_DIM = 18
ACT_DIM = 12

BASE_PORT = 7300  # disjoint from SpotEnv's 7100 to avoid collision


class SpotResidualEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        env_id: int = 0,
        host: str = "127.0.0.1",
        connect_timeout_s: float = 90.0,
        verbose: bool = False,
        world_path: Optional[Path] = None,
    ):
        super().__init__()
        self.env_id = env_id
        self.host = host
        from session_isolation import env_port, free_port_near, session_id
        self.session = session_id()
        # Residual envs get the base+30..59 sub-band of the session's
        # 200-port slot (SpotEnv agents use base+0..29, recovery base+60..89,
        # sim extern servers base+100..189). The previous "+200" offset
        # landed EXACTLY on this env's own simulator --port
        # (sim_extern_port+100 == base+200+id), so the trainer's connect
        # raced the agent's listener against the simulator's dual-stack
        # one -- ~50% of training launches deadlocked at startup (sim
        # spinning, agent 0% CPU, empty monitor.csv).
        self.port = free_port_near(env_port(env_id % 30, self.session) + 30)
        self.verbose = verbose
        # World resolution mirrors SpotEnv: ctor arg > SPOT_TRAIN_WORLD env > default.
        # Lets the Newton launcher swap in spot_residual_train_newton.wbt
        # without code changes.
        if world_path:
            self.world_path = Path(world_path)
        elif os.environ.get("SPOT_TRAIN_WORLD"):
            self.world_path = Path(os.environ["SPOT_TRAIN_WORLD"])
        else:
            self.world_path = WORLD_PATH

        self.action_space = spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)

        self.proc: Optional[subprocess.Popen] = None
        self.sock: Optional[socket.socket] = None
        self._pending_obs = None

        self._cmd_bytes = 1 + ACT_DIM * 4
        self._obs_bytes = OBS_DIM * 4 + 4 + 1
        self._stdout_log = None

        TMP_DIR.mkdir(parents=True, exist_ok=True)

    def _materialize_world(self) -> Path:
        template = self.world_path.read_text()
        replaced = template.replace('"7100"', f'"{self.port}"')
        from session_isolation import scratch_world_name
        out = self.world_path.parent / scratch_world_name(
            self.world_path.stem, self.env_id, self.session)
        out.write_text(replaced)
        return out

    def _start_sim(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            return
        time.sleep(0.5 * self.env_id)
        world = self._materialize_world()
        from session_isolation import session_tmp_dir, sim_extern_port
        sdir = session_tmp_dir(self.session)
        env = os.environ.copy()
        env["OMNISIM_HOME"] = str(REPO_ROOT)
        env["OMNISIM_LOG_PATH"] = str(sdir / f"spot_res_{self.env_id}.log")
        env["SPOT_RL_PORT"] = str(self.port)
        cmd = [
            str(OMNISIM_BIN), str(world),
            "--batch", "--mode=fast", "--no-rendering", "--minimize",
            "--stdout", "--stderr",
            f"--port={sim_extern_port(self.env_id % 30, self.session) + 30}",
        ]
        stdout_path = sdir / f"spot_res_{self.env_id}.stdout.log"
        self._stdout_log = open(stdout_path, "w")
        last_err = None
        for attempt in range(5):
            try:
                self.proc = subprocess.Popen(
                    cmd, env=env, stdout=self._stdout_log,
                    stderr=subprocess.STDOUT)
                last_err = None
                break
            except PermissionError as e:
                last_err = e
                time.sleep(0.6 * (attempt + 1))
        if last_err is not None:
            raise RuntimeError(f"SpotResidualEnv {self.env_id}: Popen failed: {last_err}")

    def _connect(self) -> None:
        deadline = time.time() + 90.0
        last_err = None
        while time.time() < deadline:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = s
                return
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                last_err = e
                time.sleep(0.2)
        raise RuntimeError(f"SpotResidualEnv {self.env_id}: connect failed: {last_err}")

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("controller closed connection")
            buf.extend(chunk)
        return bytes(buf)

    def _read_obs(self) -> Tuple[np.ndarray, float, bool]:
        raw = self._recv_exact(self._obs_bytes)
        obs = np.frombuffer(raw, dtype=np.float32, count=OBS_DIM).copy()
        reward = struct.unpack_from("<f", raw, OBS_DIM * 4)[0]
        done = raw[OBS_DIM * 4 + 4] != 0
        return obs, float(reward), bool(done)

    def _send_cmd(self, tag: int, action: Optional[np.ndarray] = None) -> None:
        if action is None:
            action = np.zeros(ACT_DIM, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32).reshape(ACT_DIM)
        self.sock.sendall(bytes([tag]) + action.tobytes())

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if self.proc is None or self.proc.poll() is not None:
            time.sleep(self.env_id * 0.5)
            self._start_sim()
            # Give OmniSim time to actually start up the controller
            # before we connect. The recently-rebuilt omnisim-bin +
            # widened URDF takes longer to come up than the previous
            # build (more contact points to set up, slower physics
            # init). Without enough buffer here _connect succeeds
            # against a not-yet-listening agent (Windows kernel
            # buffers SYN) and the first recv hits an RST as the
            # kernel closes the half-open connection. 5s is safe;
            # tunable via SPOT_ENV_STARTUP_S env var.
            startup = float(os.environ.get("SPOT_ENV_STARTUP_S", "5.0"))
            time.sleep(startup)
            self._connect()
            obs, _, _ = self._read_obs()
            return obs, {}
        self._send_cmd(tag=1)
        obs, _, _ = self._read_obs()
        return obs, {}

    def step(self, action):
        self._send_cmd(tag=0, action=action)
        obs, reward, done = self._read_obs()
        return obs, reward, bool(done), False, {}

    def close(self):
        try:
            if self.sock is not None:
                self._send_cmd(tag=2)
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        if self.proc is not None:
            try:
                import psutil
                try:
                    parent = psutil.Process(self.proc.pid)
                    for child in parent.children(recursive=True):
                        try: child.kill()
                        except Exception: pass
                    parent.kill()
                except psutil.NoSuchProcess:
                    pass
            except ImportError:
                try:
                    self.proc.terminate()
                    try: self.proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.proc.kill(); self.proc.wait()
                except Exception:
                    pass
            try: self.proc.wait(timeout=3)
            except Exception: pass
        self.proc = None
        if self._stdout_log is not None:
            try: self._stdout_log.close()
            except Exception: pass
            self._stdout_log = None
