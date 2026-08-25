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

# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Turn OmniLink usage units into real money.

Why this exists
---------------
`GET /api/omni-key-usage` already carries a dollar figure -- its `credits`
field is derived at READ time from a per-model price table in
``api/omni-key-usage.ts``. But two things make it unusable as-is for
"what is this agent costing me per hour":

1. ``credits_24h`` / ``credits_7d`` / ``credits_30d`` are **always 0**. The
   engine handlers never write a `credits` value at log time, and the rollup
   tables only aggregate what was written. Verified live 2026-07-22:
   ``input_units_24h`` 1,473,563 alongside ``credits_24h`` 0.
2. ``total_credits`` is derived only over the event page the endpoint happens
   to return (100 rows), so on a busy key it silently **understates**.

So we recompute locally from `input_units` / `output_units`, which ARE
populated, using the same constants the server uses. Same arithmetic, same
answer -- verified: this module reproduces the server's own `total_credits`
to the digit (0.143287 vs 0.14328712).

UNITS ARE NOT TOKENS. For Gemini and GPT the server meters **characters**;
for Grok and Claude it meters **tokens**. Anything that prints a
character count labelled "tokens" overstates by roughly 4x. `unit_name()`
exists so callers can label output honestly.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Tuple

# Mirrors DEFAULT_USAGE_PRICES in omnilink/api/omni-key-usage.ts.
# (input_price_per_unit, output_price_per_unit) in USD.
PRICES: Dict[str, Tuple[float, float]] = {
    "gemini": (0.000000125, 0.00000075),   # Gemini 3 Flash, per CHARACTER
    "gpt5":   (0.0000025,   0.000015),     # GPT-5.4,        per CHARACTER
    "grok":   (0.000002,    0.000006),     # Grok 4.5,       per TOKEN
    "claude": (0.000003,    0.000015),     # Claude Sonnet 4.6, per TOKEN
}

_UNIT_NAME = {"gemini": "chars", "gpt5": "chars", "grok": "tokens", "claude": "tokens"}

ENGINE_KIND = {
    "g1-engine": "gemini",
    "g2-engine": "gpt5",
    "g3-engine": "grok",
    "g4-engine": "claude",
}


def kind_for_engine(engine: Optional[str], endpoint: str = "") -> Optional[str]:
    """Map an engine id (or endpoint path) to a price bucket.

    ``/api/chat`` is deliberately unpriced. Every chat writes TWO usage rows
    under one request_id -- ``/api/chat`` plus the ``/api/gN-engine`` it
    delegates to -- and the chat row copies the engine's id onto itself, so it
    looks priceable. Pricing both charges every chat about twice. Verified
    against live data 2026-07-22: 27 of 27 request_ids carried exactly two
    events, with the chat row ~1.5x larger than the engine's (25,387 vs 16,848
    input units on the same request). The engine row is authoritative.
    """
    ep = (endpoint or "").lower()
    if ep == "/api/chat" or ep.endswith("/api/chat"):
        return None
    eng = (engine or "").lower().strip()
    if eng in ENGINE_KIND:
        return ENGINE_KIND[eng]
    for slug, kind in ENGINE_KIND.items():
        if f"/{slug}" in ep:
            return kind
    return None


def unit_name(engine: Optional[str]) -> str:
    """'chars' or 'tokens' -- what this engine's units actually are."""
    kind = kind_for_engine(engine)
    return _UNIT_NAME.get(kind or "", "units")


def cost_usd(engine: Optional[str], input_units: float, output_units: float) -> float:
    """USD for one call's units. Returns 0.0 for non-engine endpoints."""
    kind = kind_for_engine(engine)
    if kind is None:
        return 0.0
    price_in, price_out = PRICES[kind]
    return (input_units or 0) * price_in + (output_units or 0) * price_out


def cost_of_events(events: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Aggregate a list of usage events into per-engine and total USD."""
    per_kind: Dict[str, Dict[str, float]] = {}
    total = 0.0
    counted = skipped = 0
    for ev in events or ():
        kind = kind_for_engine(ev.get("engine"), ev.get("endpoint") or "")
        if kind is None:
            skipped += 1
            continue
        i = float(ev.get("input_units") or 0)
        o = float(ev.get("output_units") or 0)
        usd = cost_usd(ev.get("engine") or f"/{kind}", i, o)
        bucket = per_kind.setdefault(kind, {"calls": 0, "input_units": 0.0,
                                            "output_units": 0.0, "usd": 0.0})
        bucket["calls"] += 1
        bucket["input_units"] += i
        bucket["output_units"] += o
        bucket["usd"] += usd
        total += usd
        counted += 1
    return {"total_usd": total, "per_engine": per_kind,
            "events_counted": counted, "events_skipped": skipped}


def cost_since(client: Any, since_iso: str, limit: int = 500,
               engine: Optional[str] = None) -> Dict[str, Any]:
    """Exact USD for usage events at or after ``since_iso``.

    Preferred over :func:`rate_report` for a single run. ``UsageMeter`` diffs
    the platform ROLLUP, whose ``input_units_*`` sum both the ``/api/chat``
    envelope row and the engine row for every request — so a rollup-based
    figure over-reports units (and therefore cost) by roughly 1.5-2x for chat
    traffic. Summing the events directly, with the envelope excluded, counts
    each request exactly once.
    """
    try:
        payload = client.get_usage()
    except Exception as exc:  # noqa: BLE001
        return {"error": f"{type(exc).__name__}: {exc}", "total_usd": 0.0}
    events = payload.get("events") or []
    fresh = [e for e in events if str(e.get("occurred_at") or "") >= since_iso]
    # A time window alone attributes ANY traffic on this key to the caller.
    # Measured 2026-07-22: a g5 (local Ollama) benchmark was billed $0.107
    # because a concurrent session was making Gemini calls on the same key --
    # local inference costs nothing, so every cent of that was foreign. Filter
    # to the engine actually under test when the caller names one.
    if engine:
        fresh = [e for e in fresh if (e.get("engine") or "") == engine]
    result = cost_of_events(fresh)
    result["engine_filter"] = engine or "(none -- whole key, may include other traffic)"
    result["window_start"] = since_iso
    # The endpoint returns a bounded page; if the run filled it, older events
    # may have been cut and the total would understate. Say so rather than
    # quietly reporting a low number.
    result["page_truncated"] = len(events) >= limit
    return result


def run_report(client: Any, since_iso: str, elapsed_s: float,
               engine: Optional[str] = None) -> str:
    """One-line, exactly-once cost summary for a completed run."""
    data = cost_since(client, since_iso, engine=engine)
    if data.get("error"):
        return f"[cost] unavailable ({data['error']})"
    usd = float(data.get("total_usd") or 0.0)
    elapsed = max(float(elapsed_s), 1e-6)
    per_hour = usd * 3600.0 / elapsed
    parts = []
    for kind, b in sorted(data.get("per_engine", {}).items()):
        parts.append(f"{kind} {b['calls']}x ${b['usd']:.4f}")
    detail = ", ".join(parts) or "no priced calls"
    warn = "  [page truncated -- may understate]" if data.get("page_truncated") else ""
    if not engine:
        warn += "  [whole-key: may include other sessions' traffic]"
    return (f"[cost] {usd:.4f} USD this run ({detail}, {elapsed:.0f}s) "
            f"-> {per_hour:.2f} USD/hour sustained{warn}")


def rate_report(delta: Any, engine: str) -> str:
    """One-line honest cost line for a run.

    `delta` is an omnilink.usage_meter.UsageDelta. It reports whole-key usage
    over the window, so on a shared key this is "what the key spent while this
    agent ran", not strictly this agent alone -- say so rather than implying
    per-agent attribution we cannot prove.
    """
    try:
        elapsed = max(float(getattr(delta, "elapsed_s", 0.0)), 1e-6)
        inp = float(getattr(delta, "input_units", 0) or 0)
        out = float(getattr(delta, "output_units", 0) or 0)
    except (TypeError, ValueError):
        return "[cost] unavailable"
    usd = cost_usd(engine, inp, out)
    per_hour = usd * 3600.0 / elapsed
    units = unit_name(engine)
    return (f"[cost] {usd:.4f} USD this run "
            f"({inp:,.0f} in + {out:,.0f} out {units}, {elapsed:.0f}s) "
            f"-> {per_hour:.2f} USD/hour sustained  [{engine}]")
