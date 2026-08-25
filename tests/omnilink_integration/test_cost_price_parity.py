# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Guard against price drift between OmniSim's cost readout and OmniLink.

`agents/production/_lib/cost.py` carries a copy of OmniLink's per-model price
table so an agent run can print real money without a round-trip. A copy is a
drift risk: if OmniLink re-tunes a price and this one doesn't follow, every
cost figure OmniSim prints becomes quietly wrong — and a wrong cost number is
worse than none, because nobody re-checks a number that looks plausible.

This test reads the constants straight out of OmniLink's
`api/_usage-pricing.ts` and asserts they match. It SKIPS when the OmniLink
checkout isn't present, so a public clone of OmniSim (which does not ship
OmniLink) stays green.

Point it somewhere else with OMNILINK_REPO=/path/to/omnilink.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "agents" / "production"))

# Each OmniSim price maps to the TS constant it was copied from.
FIELD_MAP = {
    ("gemini", 0): "geminiInputChars",
    ("gemini", 1): "geminiOutputChars",
    ("gpt5", 0): "gpt5InputChars",
    ("gpt5", 1): "gpt5OutputChars",
    ("grok", 0): "grokInputTokens",
    ("grok", 1): "grokOutputTokens",
    ("claude", 0): "claudeInputTokens",
    ("claude", 1): "claudeOutputTokens",
}


def _omnilink_pricing_file() -> Path | None:
    configured = os.environ.get("OMNILINK_REPO", "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured))
    # Default layout: a sibling `omnilink/` checkout next to the omnisim tree.
    candidates.append(REPO_ROOT.parent / "omnilink")
    for base in candidates:
        path = base / "api" / "_usage-pricing.ts"
        if path.is_file():
            return path
    return None


def _parse_ts_prices(source: str) -> dict[str, float]:
    """Pull DEFAULT_USAGE_PRICES out of the TS module."""
    block = re.search(
        r"DEFAULT_USAGE_PRICES\s*:\s*UsagePrices\s*=\s*\{(.*?)\};",
        source,
        re.DOTALL,
    )
    assert block, "could not locate DEFAULT_USAGE_PRICES in _usage-pricing.ts"
    found: dict[str, float] = {}
    for name, value in re.findall(r"(\w+)\s*:\s*([0-9.eE+-]+)\s*,", block.group(1)):
        found[name] = float(value)
    return found


@pytest.mark.parametrize("kind_index,ts_field", sorted(FIELD_MAP.items(), key=lambda kv: kv[1]))
def test_price_matches_omnilink(kind_index, ts_field):
    pricing_file = _omnilink_pricing_file()
    if pricing_file is None:
        pytest.skip(
            "OmniLink checkout not found — set OMNILINK_REPO to enable the "
            "price-parity check (expected a sibling ../omnilink)"
        )
    from _lib.cost import PRICES  # noqa: PLC0415 - skip-guarded import

    kind, index = kind_index
    ts_prices = _parse_ts_prices(pricing_file.read_text(encoding="utf-8"))
    assert ts_field in ts_prices, f"{ts_field} missing from OmniLink's price table"
    assert PRICES[kind][index] == pytest.approx(ts_prices[ts_field], rel=1e-12), (
        f"price drift: OmniSim _lib/cost.py PRICES[{kind!r}][{index}] = "
        f"{PRICES[kind][index]} but OmniLink {ts_field} = {ts_prices[ts_field]}.\n"
        f"Update agents/production/_lib/cost.py to match {pricing_file}, or the "
        f"USD figures OmniSim prints will be silently wrong."
    )


def test_engine_kind_mapping_matches_omnilink():
    """g1->gemini, g2->gpt5, g3->grok, g4->claude must agree on both sides."""
    pricing_file = _omnilink_pricing_file()
    if pricing_file is None:
        pytest.skip("OmniLink checkout not found")
    from _lib.cost import kind_for_engine  # noqa: PLC0415

    source = pricing_file.read_text(encoding="utf-8")
    for engine, expected in (
        ("g1-engine", "gemini"), ("g2-engine", "gpt5"),
        ("g3-engine", "grok"), ("g4-engine", "claude"),
    ):
        assert kind_for_engine(engine) == expected
        assert f"'{engine}'" in source, f"{engine} no longer referenced by OmniLink"
