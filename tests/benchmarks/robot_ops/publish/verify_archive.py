"""Offline verifier for the OmniLink warehouse-shift benchmark evidence.

Run it from the extracted archive's folder with Python 3.10+:

    python -I -S verify_archive.py

No network, no API key, no simulator, no paid call. It checks three things:
  1. every file matches the SHA-256 list in FILES_SHA256.txt;
  2. re-grading each episode from its recorded pose trace, prop positions and
     replies (with the bundled grader, grader/suite.py) gives exactly the
     recorded grade;
  3. the scores, unsafe counts and verdicts recomputed from those grades match
     results.json.
Hashes establish consistency; they are not independent validation of the run.
"""
import gzip
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_grader():
    spec = importlib.util.spec_from_file_location("suite", HERE / "grader" / "suite.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def trace_of(row, run_dir):
    tr = row.get("trace") or {}
    if "file" not in tr:
        return tr
    with gzip.open(run_dir / "episodes" / row["episode_dir"] / tr["file"], "rt", encoding="utf-8") as f:
        return json.load(f)


def main():
    ok = True
    # 1. hashes
    bad = []
    for line in (HERE / "FILES_SHA256.txt").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        digest, rel = line.split("  ", 1)
        p = HERE / rel
        if not p.exists() or sha256(p) != digest:
            bad.append(rel)
    print(f"[1] file hashes: {'OK' if not bad else 'MISMATCH ' + str(bad[:5])}")
    ok &= not bad

    # 2. re-grade every episode
    suite = load_grader()
    results = json.loads((HERE / "results.json").read_text(encoding="utf-8"))
    task = json.loads((HERE / "suites" / results["suite_file"]).read_text(encoding="utf-8"))["tasks"][0]
    rows, mismatches = [], []
    for run in results["run_dirs"]:
        run_dir = HERE / "runs" / run
        for line in (run_dir / "rows.jsonl").read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            r = json.loads(line)
            r["_run"] = run
            rows.append(r)
            if r.get("outcome") == "ERROR" and not r.get("trace"):
                continue
            o, reasons, unsafe, _ = suite.grade(task, trace_of(r, run_dir), r["fired"], r["replies"],
                                                r["pose0"])
            if (o, reasons, unsafe) != (r["outcome"], r["reasons"], r["unsafe"]):
                mismatches.append((run, r["arm"]))
    print(f"[2] re-graded {len(rows)} episodes: {'OK' if not mismatches else 'MISMATCH ' + str(mismatches)}")
    ok &= not mismatches

    # 3. scores and verdicts
    drop = set(str(x) for x in results["dropped_checks"])
    unsafe_reasons = set(results["unsafe_reasons"])

    def score(r):
        keys = [k for k in r["checks"] if k[0].isdigit() and k.split(":")[0] not in drop]
        failed = {x.split(":")[0] for x in r["reasons"]}
        return sum(1 for k in keys if k.split(":")[0] not in failed) / len(keys)

    def unsafe_n(r):
        return sum(1 for x in r["reasons"] if x.split(":")[-1] in unsafe_reasons and x.split(":")[0] not in drop)

    by_arm = {}
    for r in rows:
        if r["outcome"] != "ERROR":
            by_arm.setdefault(r["arm"], []).append(r)
    diffs = []
    for arm, rec in results["arms"].items():
        got = sorted(round(score(r), 3) for r in by_arm.get(arm, []))
        if got != sorted(rec["scores"]) or sum(map(unsafe_n, by_arm.get(arm, []))) != rec["unsafe_total"]:
            diffs.append(arm)
    for claim in results["claims"].values():
        for arm, pair in claim["pairs"].items():
            o = [score(r) for r in by_arm["omnilink"]]
            x = [score(r) for r in by_arm[arm]]
            win = (min(o) > max(x) and sum(map(unsafe_n, by_arm["omnilink"])) <= sum(map(unsafe_n, by_arm[arm]))
                   and len(o) >= 3 and len(x) >= 3)
            if win != (pair["verdict"] == "clear win"):
                diffs.append(f"verdict {arm}")
    print(f"[3] scores, unsafe counts and verdicts: {'OK' if not diffs else 'MISMATCH ' + str(diffs)}")
    ok &= not diffs
    print("VERIFIED" if ok else "NOT VERIFIED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
