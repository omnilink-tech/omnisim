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
"""Run the crew order many times and keep score.

    python agents/omnilink_demos/crew_order/campaign.py <out_dir> --runs 100 --parallel 3 --tile-runs 16

Every run is a full, independent take.py: its own engine, its own three
bridges on its own ports, the same briefings and the same one order. Each
PARALLEL SLOT has its own agent identity (OMNILINK_AGENT_TAG crew-c00<slot>),
reused run after run with memory off -- a tag per RUN registered three agent
profiles per run on the account and hit the plan's 100-agent limit at run 23
(402 AGENT_LIMIT_REACHED, 2026-10-21). Runs that
already have a result.json are skipped, so the campaign resumes; a run the
ACCOUNT refused (agent quota, or provider credit exhausted) is redone.

Writes <out_dir>/ledger.jsonl (one verdict per run, appended as runs finish)
and <out_dir>/summary.json. A run that crashes before it can judge itself is
recorded as a failure with its exit code -- it is never dropped. A START
failure (the engine or its controllers never came up, so no robot was ever
briefed) is marked as such; re-running the same command redoes it, and the
summary counts how many were retried.
"""
import argparse
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


# The ACCOUNT, not a robot, refused the run: the plan's agent-profile quota, or
# the model provider's credit ran out mid-run (campaign_sonnet run 80,
# 2026-10-21). Such a run says nothing about the crew; it is set aside and redone.
_ACCOUNT_FAULTS = ("AGENT_LIMIT_REACHED", "credit balance is too low")


def _quota(r: dict) -> bool:
    return any(f in str(e) for e in (r.get("delivery_errors") or []) for f in _ACCOUNT_FAULTS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--parallel", type=int, default=3)
    ap.add_argument("--tile-runs", type=int, default=16, help="the first N runs film the montage tile camera")
    ap.add_argument("--model", default="claude-opus-5-5")
    ap.add_argument("--port-base", type=int, default=8800)
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    ledger = out / "ledger.jsonl"
    lock = threading.Lock()
    def done(i):
        res = out / f"run_{i:03d}" / "result.json"
        return res.exists() and not _quota(json.loads(res.read_text(encoding="utf-8")))
    todo = [i for i in range(1, a.runs + 1) if not done(i)]
    print(f"campaign: {a.runs} runs, {len(todo)} to do, {a.parallel} in parallel", flush=True)

    def record(i, verdict):
        with lock:
            with open(ledger, "a", encoding="utf-8") as f:
                f.write(json.dumps({"run": i, **verdict}) + "\n")
            print(f"run {i:03d}: {'OK ' if verdict.get('ok') else 'FAIL'} "
                  f"packed={verdict.get('packed')} reported={verdict.get('reported')} "
                  f"msgs={verdict.get('messages')} refusals={len(verdict.get('refusals') or [])} "
                  f"cost=${verdict.get('cost_usd')} wall={verdict.get('wall_s')}s", flush=True)

    def worker(slot):
        while True:
            with lock:
                if not todo:
                    return
                i = todo.pop(0)
            run = out / f"run_{i:03d}"
            cams = "tile" if i <= a.tile_runs else ""
            cmd = [sys.executable, str(HERE / "take.py"), str(run), "--port-base", str(a.port_base + 10 * slot),
                   "--cams", cams, "--frame-ms", "256", "--tag", f"crew-c{slot + 1:03d}", "--quiet-s", "30",
                   "--wall-limit-s", "900", "--model", a.model]
            t0 = time.monotonic()
            with open(out / f"run_{i:03d}.console.txt", "w", encoding="utf-8") as con:
                rc = subprocess.run(cmd, cwd=REPO, stdout=con, stderr=subprocess.STDOUT).returncode
            res = run / "result.json"
            if res.exists():
                record(i, json.loads(res.read_text(encoding="utf-8")))
            else:
                console = (out / f"run_{i:03d}.console.txt").read_text(encoding="utf-8", errors="replace")
                # The engine or its controllers never came up: no robot was briefed,
                # nothing was asked of OmniLink. Recorded, and the run is redone on
                # the next (resumed) pass -- robot failures are never retried.
                start_failure = "bridges never came up" in console
                record(i, {"ok": False, "crashed": True, "start_failure": start_failure, "exit_code": rc,
                           "wall_s": round(time.monotonic() - t0, 1)})

    threads = [threading.Thread(target=worker, args=(s,)) for s in range(a.parallel)]
    for t in threads:
        t.start()
        time.sleep(20)          # stagger engine start-ups
    for t in threads:
        t.join()
    rows = [json.loads(x) for x in ledger.read_text(encoding="utf-8").splitlines()] if ledger.exists() else []
    last = {}
    for r in rows:
        last[r["run"]] = r
    rs = [last[k] for k in sorted(last)]
    summary = {"runs": len(rs), "ok": sum(1 for r in rs if r.get("ok")),
               "packed": sum(1 for r in rs if r.get("packed")),
               "reported": sum(1 for r in rs if r.get("reported")),
               "crashed": sum(1 for r in rs if r.get("crashed")),
               # every start failure across all attempts, including ones a resumed pass redid
               "start_failures_retried": sum(1 for r in rows if r.get("start_failure"))
               - sum(1 for r in rs if r.get("start_failure")),
               # runs the ACCOUNT refused (agent-profile quota, provider credit: a
               # test-setup fault, not robot behaviour), set aside and redone
               "quota_failures_retried": sum(1 for r in rows if _quota(r)) - sum(1 for r in rs if _quota(r)),
               "runs_with_refusals": sum(1 for r in rs if r.get("refusals")),
               "delivery_errors": sum(len(r.get("delivery_errors") or []) for r in rs),
               "messages_total": sum(r.get("messages") or 0 for r in rs),
               "cost_usd": round(sum(r.get("cost_usd") or 0 for r in rs), 2),
               "model": a.model, "failed_runs": [r["run"] for r in rs if not r.get("ok")]}
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print("SUMMARY", json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
