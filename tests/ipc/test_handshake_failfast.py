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

"""I1 kill test: a build-mismatched engine/libController pair MUST fail loudly and fast.

Acceptance test for the OmniSim IPC handshake (docs/developer/core-evolution-plan.md,
Phase I1). Historically, a stale libController paired with a newer engine produced the
worst failure mode this project has: the controller finalized, armed, ticked ZERO times,
exited 0 -- and a headless run still printed PASS (the trap `omnisim doctor` gates
out-of-band, commit 6eea9d76). The handshake makes that state impossible; this test
proves it stays impossible.

Method (no shared-state mutation -- safe to run while other OmniSim sessions are live):
launch a private engine instance on an <extern>-controller world, connect to its IPC
pipe pretending to be a PRE-HANDSHAKE libController (send the legacy 8-byte first
packet instead of the 16-byte hello echo), and require ALL of:
  1. the engine sends its hello immediately on connect (magic OMSH, version, nonce);
  2. the pipe name itself is nonce-protected (contains the engine PID -- Phase I3);
  3. the engine rejects a legacy packet immediately rather than consuming the longer
     valid-controller startup allowance (never the old silent hang);
  4. an attributed ERROR naming the handshake and 'omnisim doctor' lands in the log.

Windows-only (named-pipe transport). Run: python tests/ipc/test_handshake_failfast.py
"""

import os
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
BINARY = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
WORLD = Path(__file__).resolve().parent / "worlds" / "handshake_probe.omniworld"
MAGIC = b"OMSH"
HELLO_SIZE = 16
CONNECT_DEADLINE_S = 60   # engine cold start + world load
CLOSE_DEADLINE_S = 5      # wrong magic is rejected immediately; allow loaded-host slack


def wait_for_pipe(pid: int, deadline_s: float) -> str:
    """The extern pipe is webots-<tmpId>-<enginePid>-probe; poll until it appears."""
    wanted = f"-{pid}-probe"
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        try:
            for name in os.listdir(r"\\.\pipe"):
                if name.startswith("webots-") and name.endswith(wanted):
                    return r"\\.\pipe" + "\\" + name
        except OSError:
            pass
        time.sleep(0.5)
    raise TimeoutError(f"extern pipe *{wanted} never appeared within {deadline_s}s")


def main() -> int:
    if os.name != "nt":
        print("SKIP: named-pipe kill test is Windows-only.")
        return 3
    if not BINARY.exists():
        print(f"FAIL: engine binary not found at {BINARY}")
        return 1

    log_path = Path(tempfile.gettempdir()) / f"omnisim_handshake_killtest_{os.getpid()}.log"
    if log_path.exists():
        log_path.unlink()
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)

    engine = subprocess.Popen(
        [str(BINARY), "--batch", "--mode=fast", "--no-rendering", "--minimize", str(WORLD)],
        env=env, cwd=REPO_ROOT)
    failures = []
    try:
        pipe_path = wait_for_pipe(engine.pid, CONNECT_DEADLINE_S)
        print(f"extern pipe found: {pipe_path}")
        # (2) nonce-protected extern pipe name (Phase I3) is implied by wait_for_pipe
        # matching on the engine PID; reaching here proves it.

        with open(pipe_path, "r+b", buffering=0) as pipe:
            hello = pipe.read(HELLO_SIZE)
            if len(hello) != HELLO_SIZE or hello[:4] != MAGIC:
                failures.append(f"engine hello malformed: {hello!r}")
            else:
                version = struct.unpack("<H", hello[4:6])[0]
                nonce = struct.unpack("<Q", hello[8:16])[0]
                print(f"engine hello OK: version={version} nonce={nonce}")
                if nonce != engine.pid:
                    failures.append(f"hello nonce {nonce} != engine pid {engine.pid}")

            # impersonate a pre-handshake libController: legacy step-0 packet, no echo
            pipe.write(struct.pack("<II", 8, 0))
            closed_at = None
            start = time.monotonic()
            try:
                data = pipe.read(1)
                if data == b"":
                    closed_at = time.monotonic() - start
            except OSError:
                closed_at = time.monotonic() - start
            if closed_at is None:
                failures.append("engine kept the mismatched connection open (sent data?)")
            elif closed_at > CLOSE_DEADLINE_S:
                failures.append(f"engine took {closed_at:.1f}s to drop the connection "
                                f"(deadline {CLOSE_DEADLINE_S}s)")
            else:
                print(f"engine dropped the mismatched connection after {closed_at:.1f}s")

        # (4) attributed diagnostic in the log
        deadline = time.monotonic() + 5
        log_text = ""
        while time.monotonic() < deadline:
            log_text = log_path.read_text(errors="replace") if log_path.exists() else ""
            if "OmniSim IPC handshake" in log_text:
                break
            time.sleep(0.5)
        if "OmniSim IPC handshake" not in log_text:
            failures.append("no attributed 'OmniSim IPC handshake' ERROR in the engine log")
        elif "doctor" not in log_text:
            failures.append("handshake ERROR does not point at 'omnisim doctor'")
        else:
            line = next(l for l in log_text.splitlines() if "OmniSim IPC handshake" in l)
            print(f"log diagnostic OK: {line[:120]}...")
    except Exception as error:  # noqa: BLE001 - report, don't crash the harness
        failures.append(f"{type(error).__name__}: {error}")
    finally:
        engine.kill()
        engine.wait(timeout=30)

    if failures:
        print("KILL TEST FAILED:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("KILL TEST PASSED: a pre-handshake libController cannot silently pair -- "
          "fail-fast, attributed, nonce-protected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
