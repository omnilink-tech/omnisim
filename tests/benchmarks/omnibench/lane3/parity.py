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

"""OmniBench lane 3b — train==deploy structural parity as a benchmark row.

Wraps projects/policies/research/training/g1_golden_parity.py --structural
(the field-by-field MjModel diff of the G1 trainer / literal-deploy / canonical-
MJCF models) and scores its verdict into the SPEC row schema:

    metrics: {real_physics_gaps: N, representational_diffs: M, pass: N==0}
    deviations: the diffed field names, and the P2 trustworthiness verdict.

Runs the diff TWICE by default: once in the legacy deploy config (env defaults,
deploy COM at link origin) and once with OMNISIM_NEWTON_USE_LINK_COM=1 (URDF
link-COM parity) — one row each, so the number competitors' trainer!=deployer
gap sits next to is the honest default AND the best-config value.

CPU-only (--structural never touches CUDA). Needs newton + mujoco importable by
this interpreter (on this repo they ship via the bundle .pth).

Usage:
    python tests/benchmarks/omnibench/lane3/parity.py
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import os
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _row  # noqa: E402

REPO = _row.repo_root()
SCRIPT = REPO / "projects" / "policies" / "research" / "training" / "g1_golden_parity.py"
TIMEOUT_S = 900


def parse_verdict(out: str) -> dict:
    """Parse the OVERALL STRUCTURAL VERDICT block of g1_golden_parity output."""
    res = {
        "p2_trustworthy": None,
        "real_fields": [],       # [(field, pairs)]
        "repr_fields": [],
        "all_match": False,
        "parsed": False,
    }
    m = re.search(r"P2 deploy driver: (\w+)", out)
    if m:
        res["p2_trustworthy"] = (m.group(1) == "TRUSTWORTHY")

    tail = out.split("OVERALL STRUCTURAL VERDICT", 1)
    if len(tail) < 2:
        return res
    tail = tail[1]
    res["parsed"] = True

    if "ALL PAIRS MATCH" in tail:
        res["all_match"] = True
        return res

    bucket = None
    for line in tail.splitlines():
        if "REAL PHYSICS GAP(S) remain" in line:
            bucket = "real"
            continue
        if "NO REAL PHYSICS GAPS" in line:
            bucket = "repr"
            continue
        if "plus REPRESENTATIONAL diffs" in line:
            bucket = "repr"
            continue
        if line.strip().startswith("Note:"):
            break
        fm = re.match(r"\s+\*\s+(\S+)\s+\[(.*)\]", line)
        if fm and bucket:
            entry = (fm.group(1), fm.group(2))
            (res["real_fields"] if bucket == "real" else res["repr_fields"]).append(entry)
    return res


def run_variant(label: str, extra_env: dict, out_path: str) -> int:
    env = dict(os.environ)
    env.update(extra_env)
    print("[l3-par] running g1_golden_parity --structural  [%s] ..." % label)
    t0 = time.time()
    try:
        proc = subprocess.run(
            [sys.executable, str(SCRIPT), "--structural"],
            cwd=str(REPO), env=env, timeout=TIMEOUT_S,
            capture_output=True, text=True)
        out = (proc.stdout or "") + (proc.stderr or "")
        exec_err = None
    except subprocess.TimeoutExpired:
        out, exec_err = "", "timeout after %ds" % TIMEOUT_S
    except OSError as e:
        out, exec_err = "", repr(e)
    wall = time.time() - t0

    deviations = ["config: %s" % label]
    if exec_err:
        # Couldn't run at all — record why honestly, score as not-run.
        metrics = {"real_physics_gaps": -1, "representational_diffs": -1,
                   "pass": False, "ran": False}
        deviations.append("g1_golden_parity did not run: %s" % exec_err)
        row_ok = 1
    else:
        v = parse_verdict(out)
        if not v["parsed"]:
            metrics = {"real_physics_gaps": -1, "representational_diffs": -1,
                       "pass": False, "ran": False}
            deviations.append("could not parse verdict block; exit=%s; last output: %s"
                              % (proc.returncode, out.strip().splitlines()[-5:]))
            row_ok = 1
        else:
            n_real = len(v["real_fields"])
            n_repr = len(v["repr_fields"])
            metrics = {"real_physics_gaps": n_real,
                       "representational_diffs": n_repr,
                       "pass": n_real == 0,
                       "ran": True,
                       "p2_trustworthy": v["p2_trustworthy"]}
            for f, pairs in v["real_fields"]:
                deviations.append("REAL gap: %s [%s]" % (f, pairs))
            for f, pairs in v["repr_fields"]:
                deviations.append("representational: %s [%s]" % (f, pairs))
            if v["p2_trustworthy"] is False:
                deviations.append("P2 deploy driver INCONCLUSIVE — gaps count untrustworthy")
            row_ok = 0 if (n_real == 0 and v["p2_trustworthy"]) else 1

    row = _row.make_row(
        test="parity_structural", engine="omnisim-newton", dt_ms=0.0,
        metrics=metrics, wall_ms_per_step=0.0, steps=0, sim_seconds=0.0,
        deviations=deviations + [
            "not a stepping benchmark: wall_s=%.1f for the full 3-model build+diff" % wall])
    _row.write_row(out_path, row)
    print("[l3-par]   [%s] real_physics_gaps=%s repr_diffs=%s pass=%s (wall %.1fs)"
          % (label, metrics["real_physics_gaps"],
             metrics["representational_diffs"], metrics["pass"], wall))
    return row_ok


def main():
    ap = argparse.ArgumentParser(description="OmniBench lane 3b train==deploy parity")
    ap.add_argument("--out", default=str(_row.default_out("parity.jsonl")))
    ap.add_argument("--legacy-only", action="store_true",
                    help="skip the OMNISIM_NEWTON_USE_LINK_COM=1 variant")
    args = ap.parse_args()

    rc = run_variant("deploy-default (legacy COM-at-link-origin)",
                     {}, args.out)
    if not args.legacy_only:
        rc2 = run_variant("OMNISIM_NEWTON_USE_LINK_COM=1 (URDF link-COM parity)",
                          {"OMNISIM_NEWTON_USE_LINK_COM": "1"}, args.out)
        rc = min(rc, rc2)  # exit reflects the BEST config; both rows are recorded
    print("[l3-par] rows appended to %s" % args.out)
    return rc


if __name__ == "__main__":
    sys.exit(main())
