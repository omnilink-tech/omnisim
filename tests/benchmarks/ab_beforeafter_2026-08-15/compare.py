"""Aggregate the before/after arms into the tables the report needs.

    python compare.py --a <cellA1> <cellA2> ... --b <cellB1> <cellB2> ...
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from defect_scan import scan_cell  # noqa: E402

COLS = [("outcome", "outcome"), ("assertions_passed", "asserts"),
        ("tool_calls", "tool calls"),
        ("defect_encounters", "defects"), ("calls_lost_to_defects", "calls lost"), ("error_calls", "err calls"),
        ("error_call_pct", "err %"),
        ("time_to_first_working_scene_s", "t_1st_scene_s"),
        ("t_agent_s", "t_agent_s"), ("usd", "usd")]


def fmt(v):
    if isinstance(v, float):
        return "%.2f" % v
    return str(v)


def med(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def arm_table(name, cells):
    rows = [scan_cell(c) for c in cells]
    print("\n### ARM %s" % name)
    hdr = "cell".ljust(26) + "".join(h.rjust(15) for _k, h in COLS)
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(r["cell"][-26:].ljust(26) +
              "".join(fmt(r.get(k)).rjust(15) for k, _h in COLS))
    print("MEDIAN".ljust(26) +
          "".join(fmt(med([r.get(k) for r in rows])).rjust(15)
                  if k not in ("outcome",) else "".rjust(15)
                  for k, _h in COLS))
    passes = sum(1 for r in rows if r.get("outcome") == "PASS")
    print("outcome: %d PASS / %d cells" % (passes, len(rows)))
    return rows


def defect_table(a_rows, b_rows):
    sigs = set()
    for r in a_rows + b_rows:
        sigs |= set(r["by_signature"])
    print("\n### DEFECT ENCOUNTERS BY SIGNATURE (the headline)")
    print("%-30s %14s %14s" % ("signature", "ARM A (before)", "ARM B (after)"))
    print("-" * 60)
    ta = tb = 0
    for s in sorted(sigs):
        ca = [r["by_signature"].get(s, 0) for r in a_rows]
        cb = [r["by_signature"].get(s, 0) for r in b_rows]
        ta += sum(ca)
        tb += sum(cb)
        print("%-30s %14s %14s"
              % (s, "%d  (%s)" % (sum(ca), "/".join(map(str, ca))),
                 "%d  (%s)" % (sum(cb), "/".join(map(str, cb)))))
    print("-" * 60)
    print("%-30s %14d %14d" % ("TOTAL", ta, tb))
    print("%-30s %14.1f %14.1f"
          % ("per cell", ta / max(len(a_rows), 1), tb / max(len(b_rows), 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", nargs="+", required=True)
    ap.add_argument("--b", nargs="+", default=[])
    ap.add_argument("--json", default=None)
    x = ap.parse_args()
    a = arm_table("A (before / pre-fix product)", x.a)
    b = arm_table("B (after / HEAD product)", x.b) if x.b else []
    if b:
        defect_table(a, b)
    if x.json:
        json.dump({"arm_a": a, "arm_b": b}, open(x.json, "w", encoding="utf-8"),
                  indent=1)
        print("\nwrote %s" % x.json)


if __name__ == "__main__":
    main()
