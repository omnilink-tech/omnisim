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

"""Pins on the `/capabilities` `not_supported` table and the ROUTES summaries.

AGENTS.md tells agents to call `GET /capabilities` first and to trust its
`not_supported` block over their own priors, so a stale entry there is worse
than no entry: it installs a false belief with the authority of a measurement.
These tests pin the three ways the table has already gone stale once:

* an entry still describing a feature as UNIMPLEMENTED after it was measured
  working (the motorised BallJoint / Hinge2Joint pair, 2026-08-17 / 2026-09-01);
* a `source` field citing `file:NNN` -- a line number that drifts with every
  edit to the engine -- instead of `file (symbol)`;
* a `source` naming a file that no longer exists (the WREN deletion removed
  several).

Pure Python: importing `omnisim_harness` starts nothing.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

import omnisim_harness as h  # noqa: E402

_SOURCE_FILE_RE = re.compile(r"[\w/.-]+\.(?:cpp|hpp|h|py|wrl)")
_LINE_CITE_RE = re.compile(r"\.(?:cpp|hpp|h|py|wrl):\d+")

_REVISED_JOINT_KEYS = (
    "joint.motorised_balljoint_actuation",
    "joint.motorised_hinge2joint_actuation",
)


def _entry(feature: str) -> dict:
    hits = [e for e in h.ENGINE_NOT_SUPPORTED if e["feature"] == feature]
    assert len(hits) == 1, f"expected exactly one entry for {feature!r}, got {len(hits)}"
    return hits[0]


@pytest.mark.parametrize("feature", _REVISED_JOINT_KEYS)
def test_revised_joint_entries_no_longer_say_unimplemented(feature):
    text = json.dumps(_entry(feature))
    assert "UNIMPLEMENTED" not in text
    assert "gated OFF" not in text and "default-OFF" not in text
    # The gate has been DEFAULT ON since 2026-08-17; the entry must say so and
    # must name the revert hatch, because that hatch disables both joint types.
    assert "DEFAULT ON" in text
    assert "OMNISIM_NEWTON_BALL_HINGE2" in text
    assert _entry(feature).get("status", "").startswith("WORKS")


def test_balljoint_entry_keeps_the_real_residual_gap():
    text = json.dumps(_entry("joint.motorised_balljoint_actuation"))
    # The one thing still not supported is per-axis stops on the BALL element.
    assert "limited: False" in text
    assert "0.1884" in text and "0.1947" in text  # the 2026-09-01 measurement


@pytest.mark.parametrize("entry", h.ENGINE_NOT_SUPPORTED,
                         ids=[e["feature"] for e in h.ENGINE_NOT_SUPPORTED])
def test_entry_shape(entry):
    for key in ("feature", "scope", "reason", "symptom", "source", "workaround"):
        assert entry.get(key), f"{entry['feature']}: missing {key}"
    assert "diagnostic" in entry  # None is a legal, meaningful value (SILENT)
    assert entry["scope"] == "engine"


@pytest.mark.parametrize("entry", h.ENGINE_NOT_SUPPORTED,
                         ids=[e["feature"] for e in h.ENGINE_NOT_SUPPORTED])
def test_source_cites_symbols_not_line_numbers(entry):
    assert not _LINE_CITE_RE.search(entry["source"]), (
        f"{entry['feature']}: cite `file (symbol)`, not a line number -- {entry['source']}")


@pytest.mark.parametrize("entry", h.ENGINE_NOT_SUPPORTED,
                         ids=[e["feature"] for e in h.ENGINE_NOT_SUPPORTED])
def test_every_cited_source_file_exists(entry):
    files = _SOURCE_FILE_RE.findall(entry["source"])
    assert files, f"{entry['feature']}: source names no file"
    missing = [f for f in files if not (REPO_ROOT / f).exists()]
    assert not missing, f"{entry['feature']}: cited files do not exist: {missing}"


def test_no_entry_recommends_the_deleted_renderer():
    text = json.dumps(h.ENGINE_NOT_SUPPORTED)
    assert "OMNISIM_FORCE_WREN" not in text
    assert 'renderBackend \\"wren\\"' not in text


def test_runtime_mutation_entry_names_the_rebuild_verb():
    entry = _entry("scene.runtime_mutation_physics (/scene/spawn and /scene/delete reach the solver)")
    assert "/sim/rebuild_physics" in entry["workaround"]
    assert '"physics": "rebuild"' in entry["workaround"]


def test_spawn_and_delete_routes_advertise_the_rebuild_opt_in():
    routes = {r["path"]: r for r in h.ROUTES}
    for path in ("/scene/spawn", "/scene/delete"):
        r = routes[path]
        assert "physics" in r["body"], f"{path} body must accept the `physics` key"
        assert "rebuild" in r["summary"], f"{path} summary must point at the rebuild opt-in"
        assert "/sim/rebuild_physics" in r["summary"]
    assert "97-267 ms" in routes["/sim/rebuild_physics"]["summary"]


def test_feature_keys_are_unique():
    keys = [e["feature"] for e in h.ENGINE_NOT_SUPPORTED]
    assert len(keys) == len(set(keys))
