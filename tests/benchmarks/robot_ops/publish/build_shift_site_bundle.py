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
"""Build the public evidence bundle for the warehouse-shift benchmark page.

Writes, into <site dir> (the OmniLink site's public/benchmarks/operations-shift):
  evidence.zip   every run's rows, manifests, health probes and per-episode
                 traces; the suite; the grader; the preregistration, freeze,
                 amendment, author brief and calibration feedback; the results
                 write-up; results.json; and verify_archive.py
  results.json   the numbers the page shows, recomputed here from the evidence
  SHA256SUMS.txt the archive's hash
Secrets are scrubbed first (the REDACTIONS list is written into the archive).

    python tests/benchmarks/robot_ops/publish/build_shift_site_bundle.py <site dir>
"""
from __future__ import annotations

import hashlib
import json
import shutil
import statistics
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OPS = HERE.parent
REPO = OPS.parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(OPS))

import analyze_shift as A  # noqa: E402

RUNS = ["shift-holdout-v1-wave0", "shift-holdout-v1-wave1", "shift-holdout-v1-wave2",
        "shift-holdout-v1-rerun"]
DROP = {"5", "10"}
REDACT_KEYS = ("OMNILINK_REMOTE_USER_KEY",)
EPISODE_FILES = ("trace.json.gz", "relay_trace.jsonl", "stdout.log", "engine.log.newton.json")
DOCS = ["shift/PREREGISTRATION.md", "shift/FREEZE.md", "shift/FREEZE_AMENDMENT_01.md",
        "shift/AUTHOR_BRIEF.md", "shift/CALIBRATION_FEEDBACK_01.txt", "shift/CALIBRATION_FEEDBACK_02.txt",
        "shift/CALIBRATION_FEEDBACK_03.txt", "shift/CALIBRATION_FEEDBACK_04.txt",
        "evidence/SHIFT_V1_RESULTS.md", "REPRODUCE.md"]


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def scrub_row(r):
    env = r.get("env") or {}
    hit = [k for k in REDACT_KEYS if k in env]
    for k in hit:
        env[k] = "<redacted>"
    return hit


def results_json():
    A.DROPPED.clear(); A.DROPPED.update(DROP)
    dirs = [OPS / "evidence" / d for d in RUNS]
    rows = A.load([str(d) for d in dirs])
    main = A.analyse(rows)
    arms = {}
    for r in rows:
        a = arms.setdefault(r["arm"], {"scores": [], "unsafe": [], "usd_all_attempts": 0.0,
                                       "requests_all_attempts": 0, "error_episodes": 0})
        a["usd_all_attempts"] += (r.get("cost") or {}).get("usd_estimated", 0)
        a["requests_all_attempts"] += (r.get("cost") or {}).get("requests", 0)
        if r["outcome"] == "ERROR":
            a["error_episodes"] += 1
            continue
        a["scores"].append(round(A.score(r), 3))
        a["unsafe"].append(A.unsafe_count(r))
    for a in arms.values():
        a["mean_score"] = round(statistics.mean(a["scores"]), 3)
        a["unsafe_total"] = sum(a["unsafe"])
        a["usd_all_attempts"] = round(a["usd_all_attempts"], 4)
    # robustness (post hoc): without keyword/question reply checks, and without any reply check
    suite = json.loads((OPS / "suites" / "shift_holdout_v1.json").read_text(encoding="utf-8"))["tasks"][0]
    kw = {str(i) for i, c in enumerate(suite["checks"])
          if c["type"] == "reply" and ("any_of" in c or "none_of" in c or c.get("question"))}
    allr = {str(i) for i, c in enumerate(suite["checks"]) if c["type"] == "reply"}
    robust = {}
    for label, extra in (("without_keyword_reply_checks", kw), ("without_any_reply_check", allr)):
        A.DROPPED.clear(); A.DROPPED.update(DROP | extra)
        robust[label] = {t: {x: {"verdict": p["verdict"], "omnilink_mean": p["omnilink_mean"],
                                 "other_mean": p["other_mean"]} for x, p in c["pairs"].items()}
                         for t, c in A.analyse(rows)["claims"].items()}
    A.DROPPED.clear(); A.DROPPED.update(DROP)
    return {
        "benchmark": "OmniLink warehouse shift v1 (ops-bench, OmniSim)",
        "suite_file": "shift_holdout_v1.json",
        "suite_sha256": A_sha(OPS / "suites" / "shift_holdout_v1.json"),
        "run_dirs": RUNS,
        "model": "gemini-3.5-flash via g1-engine, every arm",
        "repeats": 3, "checks_scored": 33, "dropped_checks": sorted(DROP, key=int),
        "unsafe_reasons": list(A.UNSAFE_REASONS),
        "decision_rule": "clear win = every OmniLink score above every other score (3 v 3, exact one-sided "
                         "Mann-Whitney p = 0.05), OmniLink unsafe count <= other, no episode left ERROR",
        "claims": {t: {"met": c["met"], "pairs": {x: {k: p[k] for k in (
            "omnilink_scores", "other_scores", "omnilink_mean", "other_mean", "unsafe_omnilink",
            "unsafe_other", "verdict")} for x, p in c["pairs"].items()}} for t, c in main["claims"].items()},
        "arms": arms,
        "robustness_post_hoc": robust,
        "deviations": [
            "checks 5 and 10 dropped before the freeze (oracle_bad could not fail them)",
            "wave 0: Google rate-limited the project for 10 minutes; all six competitor episodes of "
            "wave 0 and one of wave 1 were ERROR and were re-run together (FREEZE_AMENDMENT_01)",
            "the preregistered real-time sensitivity result could not be computed: every wave had an "
            "episode below 0.9x real time on a shared laptop",
        ],
    }


def A_sha(p):
    return sha256(p)


def main(site_dir):
    site = Path(site_dir)
    site.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix="shift_bundle_"))
    redactions = []
    for d in RUNS:
        src, dst = OPS / "evidence" / d, stage / "runs" / d
        (dst / "episodes").mkdir(parents=True)
        for name in ("manifest.json", "health.jsonl"):
            if (src / name).exists():
                shutil.copy2(src / name, dst / name)
        lines = []
        for line in (src / "rows.jsonl").read_text(encoding="utf-8").splitlines():
            if line.strip():
                r = json.loads(line)
                for k in scrub_row(r):
                    redactions.append(f"runs/{d}/rows.jsonl: env.{k} ({r['arm']})")
                lines.append(json.dumps(r, ensure_ascii=False))
        (dst / "rows.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
        for ep in sorted((src / "episodes").iterdir()):
            (dst / "episodes" / ep.name).mkdir()
            for name in EPISODE_FILES:
                if (ep / name).exists():
                    shutil.copy2(ep / name, dst / "episodes" / ep.name / name)
    (stage / "suites").mkdir()
    shutil.copy2(OPS / "suites" / "shift_holdout_v1.json", stage / "suites" / "shift_holdout_v1.json")
    (stage / "grader").mkdir()
    shutil.copy2(REPO / "omnisim" / "ops_bench" / "suite.py", stage / "grader" / "suite.py")
    (stage / "docs").mkdir()
    for rel in DOCS:
        shutil.copy2(OPS / rel, stage / "docs" / Path(rel).name)
    shutil.copy2(HERE / "verify_archive.py", stage / "verify_archive.py")
    res = results_json()
    (stage / "results.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    (stage / "REDACTIONS.txt").write_text(
        "Values replaced with <redacted> before publication (identifiers, not scored data):\n"
        + "\n".join(sorted(set(redactions))) + "\n", encoding="utf-8")
    # A final secret scan over everything staged.
    import re
    pat = re.compile(r"olink_[A-Za-z0-9]{8,}|AIza[0-9A-Za-z_-]{20,}|sk-[A-Za-z0-9]{20,}|\"private_key\"|"
                     r"e51f760f-e4dc-49b1-bdc9-75f4c3debadc")
    leaks = []
    for p in stage.rglob("*"):
        if p.is_file() and p.suffix != ".gz":
            if pat.search(p.read_text(encoding="utf-8", errors="ignore")):
                leaks.append(str(p.relative_to(stage)))
    if leaks:
        raise SystemExit(f"secret scan failed: {leaks}")
    files = sorted(p for p in stage.rglob("*") if p.is_file())
    (stage / "FILES_SHA256.txt").write_text(
        "".join(f"{sha256(p)}  {p.relative_to(stage).as_posix()}\n" for p in files), encoding="utf-8")
    zpath = site / "evidence.zip"
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(stage.rglob("*")):
            if p.is_file():
                z.write(p, p.relative_to(stage).as_posix())
    (site / "results.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    (site / "SHA256SUMS.txt").write_text(f"{sha256(zpath)}  evidence.zip\n", encoding="utf-8")
    shutil.rmtree(stage, ignore_errors=True)
    print(json.dumps({"zip_mb": round(zpath.stat().st_size / 1e6, 1), "files": len(files),
                      "redactions": len(redactions)}))


if __name__ == "__main__":
    main(sys.argv[1])
