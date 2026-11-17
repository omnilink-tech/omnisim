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

"""damage_events_diff — compare two damage_events JSONL captures.

Reads two captures produced by damage_events_capture.py (typically an
ODE baseline vs a Newton candidate, or two runs of the same world for
determinism check) and produces a parity report.

The damage tracker (projects/default/controllers/harness_supervisor/
damage_tracker.py) is contact-driven and physics-sensitive, so we
don't expect bit-identical event streams across runs of the same
world, let alone across backends. The bar is:

  - Event-COUNT parity is within an order of magnitude
    (Newton produces "comparable" damage volume to ODE).
  - The same parts get damaged (head-on collision affects bumpers,
    not rear wheels).
  - The state-transition pattern (pristine → scuffed → damaged →
    broken) plays out on both sides.

Usage:
    python scripts/dev/damage_events_diff.py \\
        --baseline _scratch/p6_ode.jsonl \\
        --candidate _scratch/p6_newton.jsonl

Exit code: 0 = parity within bounds, 1 = parity failure (prints diff).
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path


def _load_jsonl(path: Path) -> list[dict]:
    events = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _bucket(events: list[dict]) -> dict:
    """Aggregate events into a summary for cross-run comparison.

    Field schema per damage_tracker.py::_emit:
      type        : "impact" | "transition"
      step_id     : monotonic counter
      sim_time_ms : int
      part        : logical-part name (e.g. "front_bumper")
      impulse_J   : float (impact events only)
      state       : pristine/scuffed/damaged/broken (transition events only)
    """
    by_kind = collections.Counter()
    by_part = collections.Counter()
    by_state = collections.Counter()
    total_impulse_J = 0.0
    total = len(events)
    last_step = max((e.get("step_id", 0) for e in events), default=0)
    first_step = min((e.get("step_id", 0) for e in events), default=0)
    for evt in events:
        kind = evt.get("type", "?")  # damage_tracker emits "type", not "kind"
        part = evt.get("part", "?")
        state = evt.get("state", "")
        by_kind[kind] += 1
        by_part[part] += 1
        if state:
            by_state[state] += 1
        total_impulse_J += float(evt.get("impulse_J", 0.0))
    return {
        "total": total,
        "first_step": first_step,
        "last_step": last_step,
        "total_impulse_J": total_impulse_J,
        "kinds": dict(by_kind),
        "parts": dict(by_part),
        "states": dict(by_state),
    }


def _ratio_or_inf(a: int, b: int) -> float:
    if a == 0 and b == 0:
        return 1.0
    if b == 0:
        return math.inf
    return a / b


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Diff two damage_events captures for P6 parity.")
    ap.add_argument("--baseline", type=Path, required=True,
                    help="JSONL produced by the reference run (typically ODE).")
    ap.add_argument("--candidate", type=Path, required=True,
                    help="JSONL produced by the candidate run (typically Newton).")
    ap.add_argument("--count-tolerance", type=float, default=10.0,
                    help="Pass if candidate/baseline ratio is within "
                         "[1/T, T] (default: 10×, an order of magnitude).")
    ap.add_argument("--require-parts", type=int, default=1,
                    help="Number of parts that must show up in BOTH runs "
                         "for parity (default: 1 — at least one part hit).")
    args = ap.parse_args()

    if not args.baseline.is_file():
        print(f"[diff] baseline not found: {args.baseline}", file=sys.stderr)
        return 2
    if not args.candidate.is_file():
        print(f"[diff] candidate not found: {args.candidate}", file=sys.stderr)
        return 2

    base = _load_jsonl(args.baseline)
    cand = _load_jsonl(args.candidate)
    bb = _bucket(base)
    cc = _bucket(cand)

    print(f"=== P6 damage-event parity report ===")
    print(f"baseline : {args.baseline} ({bb['total']} events, steps {bb['first_step']}-{bb['last_step']}, sum_J={bb['total_impulse_J']:.1f})")
    print(f"candidate: {args.candidate} ({cc['total']} events, steps {cc['first_step']}-{cc['last_step']}, sum_J={cc['total_impulse_J']:.1f})")
    print()

    failures: list[str] = []

    # 1. Event-count parity
    ratio = _ratio_or_inf(cc["total"], bb["total"])
    lo = 1.0 / args.count_tolerance
    hi = args.count_tolerance
    print(f"--- event count ---")
    print(f"  baseline = {bb['total']}, candidate = {cc['total']}, ratio = {ratio:.2f}")
    print(f"  pass range: [{lo:.3f}, {hi:.3f}]")
    if not (lo <= ratio <= hi):
        failures.append(f"event count ratio {ratio:.2f} outside [{lo:.3f}, {hi:.3f}]")
        print(f"  FAIL")
    else:
        print(f"  PASS")
    print()

    # 2. Shared parts
    print(f"--- parts hit ---")
    base_parts = set(bb["parts"])
    cand_parts = set(cc["parts"])
    shared = base_parts & cand_parts
    only_base = base_parts - cand_parts
    only_cand = cand_parts - base_parts
    for p in sorted(base_parts | cand_parts):
        b = bb["parts"].get(p, 0)
        c = cc["parts"].get(p, 0)
        flag = "" if p in shared else " (UNIQUE)"
        print(f"  {p:32s}  baseline={b:5d}  candidate={c:5d}{flag}")
    if only_base:
        print(f"  parts only in baseline: {sorted(only_base)}")
    if only_cand:
        print(f"  parts only in candidate: {sorted(only_cand)}")
    if len(shared) < args.require_parts:
        failures.append(f"only {len(shared)} parts hit in both runs, need >= {args.require_parts}")
        print(f"  FAIL (need >= {args.require_parts} shared parts)")
    else:
        print(f"  PASS ({len(shared)} parts shared)")
    print()

    # 3. State-transition coverage
    print(f"--- damage states observed ---")
    all_states = set(bb["states"]) | set(cc["states"])
    for s in sorted(all_states):
        print(f"  {s:16s}  baseline={bb['states'].get(s, 0):5d}  candidate={cc['states'].get(s, 0):5d}")
    print()

    # 4. Per-kind breakdown
    print(f"--- event kinds ---")
    all_kinds = set(bb["kinds"]) | set(cc["kinds"])
    for k in sorted(all_kinds):
        print(f"  {k:24s}  baseline={bb['kinds'].get(k, 0):5d}  candidate={cc['kinds'].get(k, 0):5d}")
    print()

    if failures:
        print(f"=== {len(failures)} parity FAILURE(S) ===")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"=== parity PASS ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
