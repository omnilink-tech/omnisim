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
"""Record one take of the warehouse-shift film.

    PYTHONPATH=. python agents/omnilink_demos/warehouse_shift/film/take.py <take_dir> [--model claude-opus-5-5]

This IS the benchmark: the seen 30-minute shift `shift_holdout_v2.json`, run
by ops-bench's own runner against OmniLink (arm `omnilink`), unchanged, with
the robot driven only through its real /prompt door. The film adds only
what scene.vrml() adds to the world -- visual-only floor markings and two
fixed cameras saving frames on simulator time -- and runs on the dev loop's
simulator clock (--sim-clock), so the timeline follows simulated time even
when rendering slows the world below real time.

Writes:
  <take>/run/     ops-bench evidence: rows.jsonl (every step fired, every
                  reply, the full pose trace), the relay trace, model replies
  <take>/frames/  wide/ and tall/ JPEGs named by simulator milliseconds
"""
import argparse
import os
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import scene  # noqa: E402

from omnisim.ops_bench import session as ops_session  # noqa: E402
from omnisim.ops_bench.cli import main as ops_bench  # noqa: E402

SUITE = "tests/benchmarks/robot_ops/suites/shift_holdout_v2.json"
RATES = {"claude-opus-5-5": "4.0,0.20,20.0", "claude-sonnet-5-5": "2.0,0.20,10.0"}

ap = argparse.ArgumentParser()
ap.add_argument("take")
ap.add_argument("--model", default="claude-opus-5-5")
ap.add_argument("--key-file", default="O:/omnilink-keys/omni_key.txt")
ap.add_argument("--frame-ms", type=int, default=125, help="simulated ms between frames (125 = 8 per sim second)")
a = ap.parse_args()

take = Path(a.take).resolve()
(take / "frames").mkdir(parents=True, exist_ok=True)
os.environ["OMNISIM_FILM_DIR"] = str(take / "frames")
# The cameras and markings ride in with the stanza the runner already appends.
ops_session.SUPERVISOR_STANZA = ops_session.SUPERVISOR_STANZA + scene.vrml(a.frame_ms)

sys.exit(ops_bench([
    "run", "--suite", SUITE, "--arms", "omnilink", "--engine", "g4-engine", "--model", a.model,
    "--rates", RATES.get(a.model, "4.0,0.20,20.0"), "--max-requests", "1000", "--cap-usd", "10",
    "--key-file", a.key_file, "--out", str(take / "run"), "--sim-clock",
]))
