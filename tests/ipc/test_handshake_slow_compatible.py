# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""A compatible controller delayed past the former five-second limit must pair.

This is the regression for the 100-controller BuildScale failure found on 2026-08-13.
It deliberately holds a valid echo for six seconds, then requires the engine to accept it.
The companion ``test_handshake_failfast.py`` proves wrong protocol bytes are still rejected
immediately despite the longer valid-start allowance.
"""

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# pytest.ini pins --import-mode=importlib for the whole repo (two packages ship
# a tests/test_smoke.py and the default prepend mode sees one basename twice).
# importlib imports by location and does NOT put the test's own directory on
# sys.path, so this sibling import raises ModuleNotFoundError under a plain
# `pytest tests/ipc` without the insert below. Same pattern as the AgentBench
# adapter tests.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_handshake_failfast import BINARY, HELLO_SIZE, MAGIC, REPO_ROOT, WORLD, wait_for_pipe

DELAY_S = 6
TIMEOUT_MS = 12000


def main() -> int:
    if os.name != "nt":
        print("SKIP: named-pipe handshake test is Windows-only.")
        return 3
    if not BINARY.exists():
        print(f"FAIL: engine binary not found at {BINARY}")
        return 1

    log_path = Path(tempfile.gettempdir()) / f"omnisim_handshake_slow_{os.getpid()}.log"
    if log_path.exists():
        log_path.unlink()
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    env["OMNISIM_IPC_HANDSHAKE_TIMEOUT_MS"] = str(TIMEOUT_MS)

    engine = subprocess.Popen(
        [str(BINARY), "--batch", "--mode=fast", "--no-rendering", "--minimize", str(WORLD)],
        env=env,
        cwd=REPO_ROOT,
    )
    failures = []
    try:
        pipe_path = wait_for_pipe(engine.pid, 60)
        with open(pipe_path, "r+b", buffering=0) as pipe:
            hello = pipe.read(HELLO_SIZE)
            if len(hello) != HELLO_SIZE or hello[:4] != MAGIC:
                failures.append(f"engine hello malformed: {hello!r}")
            else:
                print(f"valid hello received; delaying echo for {DELAY_S}s")
                time.sleep(DELAY_S)
                try:
                    pipe.write(hello)
                except OSError as error:
                    failures.append(f"engine rejected the delayed compatible echo: {error}")
                time.sleep(0.5)

        log_text = log_path.read_text(errors="replace") if log_path.exists() else ""
        if "did not complete the OmniSim IPC handshake" in log_text:
            failures.append("engine logged a handshake timeout for a compatible delayed echo")
        if engine.poll() is not None:
            failures.append(f"engine exited unexpectedly with code {engine.returncode}")
    except Exception as error:  # noqa: BLE001 - report all integration failures together
        failures.append(f"{type(error).__name__}: {error}")
    finally:
        engine.kill()
        engine.wait(timeout=30)

    if failures:
        print("SLOW-COMPATIBLE TEST FAILED:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(f"SLOW-COMPATIBLE TEST PASSED: valid echo accepted after {DELAY_S}s "
          f"with {TIMEOUT_MS}ms configured deadline.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
