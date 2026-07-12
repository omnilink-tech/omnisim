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

"""Session isolation for parallel Claude Code / agent runs.

Multiple agents can share one OmniSim checkout, one omnisim-bin.exe, and
one GPU. Without isolation they collide three ways:

  1. TCP ports -- SpotEnv's fixed BASE_PORT + env_id means two sessions
     both grab 7100, 7101, ... and one session's controller binds the
     port the other session is already using.
  2. Cross-session process kills -- the old "kill whatever PID is
     squatting on my port" logic would kill the *other* session's live
     controller. (SpotEnv.close() correctly kills only its own process
     tree; the squatter-killer was the dangerous part.)
  3. Scratch files -- per-env world copies, stdout logs, and the deploy
     body-trace CSV all had fixed names two sessions would stomp.

This module hands each session a stable ID and derives a disjoint port
base + private scratch directory from it, so two sessions never touch
the same port or file and never need to kill anything they don't own.

Session ID resolution (first hit wins):
  1. OMNISIM_SESSION env var (set it explicitly to coordinate a group
     of processes -- the training launchers export one so the trainer
     and every SpotEnv subprocess agree).
  2. A per-process-tree fallback derived from the parent PID, so an
     un-coordinated launch still gets a stable-within-itself ID.
"""
from __future__ import annotations

import hashlib
import os
import socket
import uuid
from pathlib import Path


def session_id() -> str:
    """Stable session identifier. Honors OMNISIM_SESSION; otherwise
    derives one from the parent PID and caches it back into the env so
    every child in this process tree agrees."""
    sid = os.environ.get("OMNISIM_SESSION")
    if sid:
        return sid
    # Fallback: parent PID is stable for the lifetime of a launch and
    # distinct between two independently-started agent sessions.
    sid = f"ppid{os.getppid()}"
    os.environ["OMNISIM_SESSION"] = sid
    return sid


def new_session_id() -> str:
    """Generate a fresh random session id (uuid4 hex, 8 chars). Launchers
    call this once and export OMNISIM_SESSION so all child processes
    inherit the same namespace."""
    return uuid.uuid4().hex[:8]


def _session_hash(sid: str) -> int:
    """Stable 0..N hash of a session id (not Python's salted hash())."""
    h = hashlib.sha1(sid.encode("utf-8")).hexdigest()
    return int(h[:8], 16)


def port_base(sid: str | None = None) -> int:
    """Disjoint TCP port base for a session. Two distinct session ids map
    to bases >= 200 apart in [20000, 60000], leaving room for env_id
    offsets and the OmniSim extern-controller port without overlap."""
    if sid is None:
        sid = session_id()
    # 200 distinct slots of 200 ports each across a 40000-port span.
    slot = _session_hash(sid) % 200
    return 20000 + slot * 200


def env_port(env_id: int, sid: str | None = None) -> int:
    """TCP port for SpotEnv <-> controller for this session + env."""
    return port_base(sid) + (env_id % 90)  # 0..89, leaves 90..199 free


def sim_extern_port(env_id: int, sid: str | None = None) -> int:
    """OmniSim's own extern-controller port, kept clear of env_port by
    living in the upper part of the session's 200-port slot."""
    return port_base(sid) + 100 + (env_id % 90)  # 100..189


def is_port_free(port: int, host: str = "127.0.0.1") -> bool:
    """True if we can bind the port right now (i.e. nothing is listening)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def free_port_near(preferred: int, host: str = "127.0.0.1", span: int = 90) -> int:
    """Return `preferred` if free, else the next free port within
    [preferred, preferred+span). Never kills anyone -- if our preferred
    port is occupied (a zombie of ours, or another session that hashed
    nearby), we just step to the next free slot. Falls back to an
    OS-assigned ephemeral port if the whole window is busy."""
    for p in range(preferred, preferred + span):
        if is_port_free(p, host):
            return p
    # Whole window busy -- let the OS pick anything free.
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, 0))
    p = s.getsockname()[1]
    s.close()
    return p


def session_tmp_dir(sid: str | None = None) -> Path:
    """Private scratch dir for this session under the shared tmp root.
    Created on first use."""
    if sid is None:
        sid = session_id()
    d = Path(r"C:\tmp\omnisim_rl") / f"session_{sid}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def scratch_world_name(world_stem: str, env_id: int, sid: str | None = None) -> str:
    """Per-session, per-env scratch world filename. Includes the session
    id so two sessions materializing the same source world don't write
    the same dotfile."""
    if sid is None:
        sid = session_id()
    return f".{world_stem}_{sid}_env{env_id}.wbt"
