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
"""Provider-health gate: is the model answering at its normal speed?

holdout-v1 run 1 was abandoned because the provider, not the agents, set
the result: gemini-3.8-flash on Vertex's shared pool answered in a median
40 s (up to 115 s) while every other Flash model answered in 2.5-5.4 s.
A realtime benchmark measured through a congested pool measures the pool.

The gate is a fixed probe, independent of every arm's outcome: N short
requests through the SAME transport the arms use. Healthy means every
request answered and the median is at most the threshold. The runner probes
before each task block and waits while it is unhealthy, so a block never
starts in a congested window and no arm is favoured by when it ran.
"""
from __future__ import annotations

import json
import statistics
import time
from pathlib import Path

from omnisim.control_bench.engine import InfrastructureError

PROBE_SYSTEM = 'You control a wheeled robot. Reply with JSON only: {"reply": str, "actions": []}.'
PROBE_MESSAGES = [{"role": "user", "content": "Drive forward one metre, then tell me where you are."}]


def probe(key, engine, model, n=5, gap_s=2.0):
    from .competitors import ModelClient
    records = []
    # The cap counts transport retries too: n requests may take 3 attempts each.
    client = ModelClient(key, engine, model, "OmniSim-husky", n * (ModelClient.RETRIES + 1),
                         log=records)
    elapsed, errors = [], []
    for _ in range(n):
        t0 = time.monotonic()
        try:
            client.complete(PROBE_SYSTEM, PROBE_MESSAGES)
            elapsed.append(time.monotonic() - t0)
        except InfrastructureError as exc:
            errors.append(str(exc)[:200])
        time.sleep(gap_s)
    return {"t": time.time(), "n": n, "answered": len(elapsed), "errors": errors,
            "median_s": round(statistics.median(elapsed), 2) if elapsed else None,
            "max_s": round(max(elapsed), 2) if elapsed else None,
            "models_returned": sorted({str(r.get("model_returned")) for r in records}),
            "requests": records}


def healthy(result, median_s):
    return (result["answered"] == result["n"] and result["median_s"] is not None
            and result["median_s"] <= median_s)


def wait_until_healthy(key, engine, model, median_s, log_path, max_wait_s,
                       retry_s=600.0, cost=None):
    """Probe until healthy. Returns (ok, spent_usd). Every probe is logged."""
    deadline = time.monotonic() + max_wait_s
    spent = 0.0
    while True:
        res = probe(key, engine, model)
        usd = cost(res["requests"]) if cost else 0.0
        spent += usd
        ok = healthy(res, median_s)
        res.update(healthy=ok, threshold_median_s=median_s, usd_estimated=round(usd, 6))
        with Path(log_path).open("a", encoding="utf-8") as f:
            f.write(json.dumps(res, default=str) + "\n")
        print(f"health: {'OK' if ok else 'UNHEALTHY'} median {res['median_s']}s max {res['max_s']}s "
              f"answered {res['answered']}/{res['n']}", flush=True)
        if ok:
            return True, spent
        if time.monotonic() + retry_s > deadline:
            return False, spent
        time.sleep(retry_s)
