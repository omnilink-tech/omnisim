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
"""Derived model cost per episode, from the usage each provider reported.

Estimates at list rates, never invoices. OmniLink's relay reports per-turn
token totals in its OMNILINK_TRACE file (`relay_usage`); a competitor arm
records each request's raw usage (`competitor_requests`). A request whose
usage is unknown is charged at the most expensive request seen so far -- the
preregistered rule -- and counted, never treated as free.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple


# Anthropic bills a 5-minute prompt-cache write at 1.25x the input rate.
# Gemini and OpenAI caches have no write charge, so their writes are always 0.
CACHE_WRITE_MULTIPLIER = 1.25


def _usd(prompt: int, cached: int, cache_write: int, output: int, rates: Dict[str, float]) -> float:
    cached = min(cached, prompt)
    cache_write = min(cache_write, prompt - cached)
    fresh = prompt - cached - cache_write
    return (fresh * rates["input"] + cached * rates["cached"]
            + cache_write * rates["input"] * CACHE_WRITE_MULTIPLIER + output * rates["output"]) / 1e6


def request_cost(usage: Any, rates: Dict[str, float]) -> Optional[float]:
    """Gemini `usageMetadata`, Anthropic `usage` or OpenAI-style `usage` -> USD, or None."""
    if not isinstance(usage, dict):
        return None
    if "promptTokenCount" in usage:
        p = int(usage.get("promptTokenCount") or 0)
        c = int(usage.get("cachedContentTokenCount") or 0)
        w = 0
        o = int(usage.get("candidatesTokenCount") or 0) + int(usage.get("thoughtsTokenCount") or 0)
    elif "input_tokens" in usage:
        # Anthropic: input_tokens excludes cache reads and writes; output includes thinking.
        c = int(usage.get("cache_read_input_tokens") or 0)
        w = int(usage.get("cache_creation_input_tokens") or 0)
        p = int(usage.get("input_tokens") or 0) + c + w
        o = int(usage.get("output_tokens") or 0)
    elif "prompt_tokens" in usage:
        p = int(usage.get("prompt_tokens") or 0)
        c = int(((usage.get("prompt_tokens_details") or {}).get("cached_tokens")) or 0)
        w = 0
        o = int(usage.get("completion_tokens") or 0)
    else:
        return None
    return _usd(p, c, w, o, rates)


def episode_cost(rec: Dict[str, Any], rates: Dict[str, float],
                 worst_request: float) -> Tuple[float, int, int, float]:
    """(usd, requests, unknown_requests, most expensive single request)."""
    usd, n, unknown, worst = 0.0, 0, 0, worst_request
    ru = rec.get("relay_usage") or {}
    rounds = int(ru.get("rounds") or 0)
    if rounds:
        t = ru.get("totals") or {}
        known = rounds - int(ru.get("unknown_usage_rounds") or 0)
        cost = _usd(t.get("prompt", 0), t.get("cached", 0), t.get("cache_write", 0),
                    t.get("output", 0) + t.get("thoughts", 0), rates)
        usd += cost
        if known:
            worst = max(worst, cost / known)
        unknown += rounds - known
        n += rounds
    for r in rec.get("competitor_requests") or []:
        n += 1
        c = request_cost(r.get("usage"), rates)
        if c is None and r.get("http") == 429:
            # Refused before any model ran: no tokens, no charge. The relay's
            # 429 attempts are retries inside one traced round and cost
            # nothing there either; charging them here would tilt the cost
            # comparison against the competitors.
            continue
        if c is None:
            unknown += 1
        else:
            usd += c
            worst = max(worst, c)
    usd += unknown * worst
    return usd, n, unknown, worst
