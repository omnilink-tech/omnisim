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
"""Validate and sync the published comparison.svg into the OmniSim README.

Usage: python shift_comparison_svg.py <published-evidence-folder>
The folder contains results.json, codex-results.json, claude-results.json and
comparison.svg. OmniLink's generate-benchmark-chart.mjs is the renderer; this
checks exact published means and bar widths rather than independently rounding
or hardcoding the original seven scores.
"""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[4]
OUT = REPO / "docs/media/benchmarks/omnilink-shift-comparison.svg"
NAMES = {"omnilink": "OmniLink", "langgraph_full": "LangGraph", "plain_full": "Plain model loop",
         "lobster_full": "Lobster SDK", "plain_basic": "Plain model loop",
         "lobster_basic": "Lobster SDK", "langgraph_basic": "LangGraph",
         "codex_full": "Codex", "claude_full": "Claude Code"}


def sync(folder):
    folder = Path(folder)
    original = json.loads((folder / "results.json").read_text(encoding="utf-8"))
    arms = dict(original["arms"])
    for filename in ("codex-results.json", "claude-results.json"):
        extension = json.loads((folder / filename).read_text(encoding="utf-8"))
        assert extension["suite_sha256"] == original["suite_sha256"]
        assert extension["repeats"] == original["repeats"] == 3
        assert extension["retrospective"] and extension["usd_estimated"] is None
        assert abs(sum(extension["scores"]) / 3 - extension["mean_score"]) < 1e-10
        arms[extension["configuration"]] = extension
    assert set(arms) == set(NAMES)
    rows = sorted(arms.items(), key=lambda pair: -pair[1]["mean_score"])
    data = (folder / "comparison.svg").read_bytes()
    tree = ET.fromstring(data)
    ns = {"s": "http://www.w3.org/2000/svg"}
    bars = [node for node in tree.findall(".//s:rect", ns)
            if node.get("x") == "300" and node.get("height") == "26"]
    labels = [node.text for node in tree.findall(".//s:text", ns) if node.get("x") == "34" and node.get("font-size") == "18"]
    values = [node.text for node in tree.findall(".//s:text", ns) if node.get("x") == "950"]
    assert len(bars) == len(rows) == 9
    assert labels == [NAMES[key] for key, _ in rows]
    assert values == [f"{row['mean_score'] * 100:.1f}%" for _, row in rows]
    for bar, (_, row) in zip(bars, rows):
        assert 0 <= row["mean_score"] <= 1
        assert abs(float(bar.get("width")) - row["mean_score"] * 570) < .0051
    assert "retrospective" in tree.find("s:desc", ns).text
    OUT.write_bytes(data)
    print("Verified nine published scores and exact bar widths; README SVG synchronized.")


if __name__ == "__main__":
    sync(sys.argv[1])
