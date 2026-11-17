"""Shift v3's real-time validity rule, for run_campaign_v3.sh.

    python wave_rt.py <wave dir>      -> prints the median worst 5 s real-time
                                         factor of the wave's episodes, and
                                         exits 0 if it is at least 0.8x, 3 if
                                         it is below (VOID), 2 if unmeasurable

The measure is analyze_shift.py's own (analyze_holdout.worst_window over each
episode's trace), taken on timing alone: nothing is graded or judged here.
PREREGISTRATION_V3.md, "Real-time validity".
"""
from __future__ import annotations

import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_shift import load  # noqa: E402

VOID_BELOW = 0.8


def median_worst(wave: str):
    worst = [r["_worst"] for r in load([wave]) if r.get("_worst") is not None]
    return statistics.median(worst) if worst else None


if __name__ == "__main__":
    m = median_worst(sys.argv[1])
    if m is None:
        print("unmeasurable")
        sys.exit(2)
    print(f"{m:.3f}")
    sys.exit(0 if m >= VOID_BELOW else 3)
