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

"""The SparkRover story guide cannot drift from its evidence.

docs/guide/manufacturing-concept-to-robot.md tells how the SparkRover Mini went from concept
sheets to a made, wired, assembled and driven design. Every number in it is listed in
docs/guide/manufacturing-concept-to-robot.facts.json with the file it comes from and how to
read it back. For each fact this test:

- re-reads the source (a JSON key, a regex over a text file, a literal constant in a Python
  module read with ast, or a file count) and asserts the value still matches;
- asserts the fact's quote, the words the guide uses for it, is still in the guide.

So a re-run that changes a number (release.py regenerating REPORT.md, a new drive result, a
reprint) fails here until the guide and its facts are updated together. Facts of kind
`image` were read by eye from the concept sheets: only the image's presence is checked.

Engine-free: it reads files only and imports nothing from the product.
"""
import ast
import json
import math
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FACTS_PATH = ROOT / "docs/guide/manufacturing-concept-to-robot.facts.json"
FACTS = json.loads(FACTS_PATH.read_text(encoding="utf-8"))
GUIDE_PATH = ROOT / FACTS["guide"]


def _norm(text: str) -> str:
    """Collapse whitespace, so a quote matches across the guide's line wraps."""
    return re.sub(r"\s+", " ", text).strip()


GUIDE = _norm(GUIDE_PATH.read_text(encoding="utf-8"))


def _number(s: str):
    s = s.replace(",", "")
    try:
        return float(s)
    except ValueError:
        return s          # a clock time such as 4:40:19 stays a string


def _walk(doc, path):
    for k in path:
        doc = doc[k]
    return doc


def _rows(ex: dict, doc) -> list:
    rows = _walk(doc, ex.get("path", []))
    unless = ex.get("unless")
    return [r for r in rows if not (unless and isinstance(r, dict) and r.get(unless))]


def _py_const(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        targets = node.targets if isinstance(node, ast.Assign) else [node.target] if isinstance(node, ast.AnnAssign) else []
        if any(isinstance(t, ast.Name) and t.id == name for t in targets) and node.value is not None:
            return ast.literal_eval(node.value)
    raise AssertionError(f"{path}: no literal top-level assignment to {name}")


def extract(fact: dict):
    """The fact's value as its source says it today."""
    ex = fact["extract"]
    kind = ex["kind"]
    src = ROOT / fact["source"]
    if kind == "glob_count":
        return float(sum(len(list(src.glob(g))) for g in ex["glob"]))
    text = src.read_text(encoding="utf-8")
    if kind == "regex":
        m = re.search(ex["pattern"], text, re.S)
        if fact["value"] is True:
            return m is not None
        assert m, f"{fact['source']}: /{ex['pattern']}/ not found"
        vals = [_number(g) for g in m.groups()]
        return vals if len(vals) > 1 else vals[0]
    if kind == "regex_count":
        return float(len(re.findall(ex["pattern"], text, re.M)))
    if kind == "py_const":
        v = _py_const(src, ex["name"])
        if "key" in ex:
            v = v[ex["key"]]
        if "index" in ex:
            v = v[ex["index"]]
        if "contains" in ex:
            return ex["contains"].lower() in str(v).lower()
        return v
    doc = json.loads(text)
    if kind == "json":
        if "paths" in ex:
            return [_walk(doc, p) for p in ex["paths"]]
        return _walk(doc, ex["path"])
    if kind == "json_count":
        return float(len(_rows(ex, doc)))
    if kind in ("json_sum", "json_min"):
        vals = [r[ex["field"]] for r in _rows(ex, doc) if r.get(ex["field"]) is not None]
        return float(sum(vals) if kind == "json_sum" else min(vals))
    raise AssertionError(f"unknown extract kind {kind!r}")


def _shape(v, ex: dict):
    if isinstance(v, list):
        return [_shape(x, ex) for x in v]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return v
    v = v * ex.get("scale", 1.0)
    if "round" in ex:
        v = round(v, ex["round"])
    return v


def _same(got, want) -> bool:
    if isinstance(want, list):
        return isinstance(got, list) and len(got) == len(want) and all(_same(g, w) for g, w in zip(got, want))
    if isinstance(want, bool) or isinstance(got, bool):
        return got is want
    if isinstance(want, (int, float)) and isinstance(got, (int, float)):
        return math.isclose(float(got), float(want), rel_tol=0.0, abs_tol=1e-9)
    return got == want


IDS = [f["id"] for f in FACTS["facts"]]


def test_fact_ids_are_unique():
    assert len(IDS) == len(set(IDS))


@pytest.mark.parametrize("fact", FACTS["facts"], ids=IDS)
def test_fact_matches_its_source(fact):
    src = ROOT / fact["source"]
    assert src.exists(), f"{fact['id']}: source {fact['source']} is missing"
    if fact["extract"]["kind"] == "image":
        return                      # read by eye from the concept sheet; presence is all a test can check
    got = _shape(extract(fact), fact["extract"])
    assert _same(got, fact["value"]), (
        f"{fact['id']} ({fact['claim']}): {fact['source']} now gives {got!r}, the guide says {fact['value']!r}. "
        f"Update the guide and {FACTS_PATH.name} together.")


@pytest.mark.parametrize("fact", FACTS["facts"], ids=IDS)
def test_fact_is_in_the_guide(fact):
    quotes = fact["quote"] if isinstance(fact["quote"], list) else [fact["quote"]]
    for q in quotes:
        assert _norm(q) in GUIDE, f"{fact['id']}: the guide no longer says {q!r}"
