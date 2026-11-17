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

"""Run a small benchmark set through the existing test suite."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_FILE = Path(__file__).with_name("benchmark_worlds.json")
LOG_DIR = Path(__file__).with_name("logs")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run OmniSim benchmark worlds")
    parser.add_argument("--nomake", action="store_true", help="Skip the initial controller rebuild")
    args = parser.parse_args()

    LOG_DIR.mkdir(exist_ok=True)
    benchmarks = json.loads(BENCHMARK_FILE.read_text())
    env = os.environ.copy()
    env.setdefault("OMNISIM_HOME", str(REPO_ROOT))

    overall_rc = 0
    for benchmark in benchmarks:
      log_path = LOG_DIR / f"{benchmark['name']}.log"
      cmd = [
          sys.executable,
          str(REPO_ROOT / "tests" / "test_suite.py"),
          "--performance-log",
          str(log_path),
      ]
      if args.nomake:
          cmd.append("--nomake")
      cmd.append(str(REPO_ROOT / benchmark["world"]))

      print(f"[bench] {benchmark['name']} ({benchmark['category']})")
      rc = subprocess.run(cmd, cwd=REPO_ROOT, env=env, check=False).returncode
      if rc != 0:
          overall_rc = rc

    return overall_rc


if __name__ == "__main__":
    sys.exit(main())
