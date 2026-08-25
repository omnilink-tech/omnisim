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

"""Structural tests for OmniBench lane 4 (capability coverage).

No engine, no GPU, milliseconds. These guard the properties that make a
capability verdict mean something, all of which are easy to break by accident
and none of which a failing probe would announce:

  * the committed worlds match the registry (a probe edited without
    regenerating measures a world nobody can reproduce)
  * every dynamic probe carries an assertion WITH a docstring, because the
    docstring is published as the physical claim under test
  * assertions return real Verdicts on degenerate input rather than raising
    (a raising assertion scores `inconclusive`, which silently removes the
    probe from the matrix instead of failing loudly)
  * `absent` markers stay specific -- a generic token like "unknown" would
    let an unrelated diagnostic certify a feature as absent
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
LANE4 = REPO / "tests" / "benchmarks" / "omnibench" / "lane4"
for p in (str(LANE4), str(LANE4.parent)):
    if p not in sys.path:
        sys.path.insert(0, p)

capabilities = pytest.importorskip("capabilities")
gen_worlds = pytest.importorskip("gen_worlds")


def test_registry_is_structurally_valid():
    problems = capabilities.validate_registry()
    assert problems == [], "registry problems:\n  " + "\n  ".join(problems)


def test_committed_worlds_match_the_registry():
    """`gen_worlds.py --check` must be clean.

    A probe whose world on disk no longer matches what the registry produces
    is measuring something nobody can regenerate -- which is exactly the
    self-attestation this lane exists to remove.
    """
    _written, drift, stale = gen_worlds.generate(check=True)
    assert not drift and not stale, (
        "lane4 worlds are stale -- run "
        "`python tests/benchmarks/omnibench/lane4/gen_worlds.py`.\n"
        "  drift: %s\n  orphans: %s" % (drift, stale))


def test_every_probe_has_a_world_file():
    missing = [p.id for p in capabilities.PROBES
               if not gen_worlds.world_path(p).exists()]
    assert missing == [], "probes with no committed world: %s" % missing


@pytest.mark.parametrize("probe", [p for p in capabilities.PROBES
                                   if p.kind == capabilities.KIND_DYNAMIC],
                         ids=lambda p: p.id)
def test_assertion_survives_empty_input(probe):
    """An assertion handed nothing must return a Verdict, not raise.

    run_coverage catches exceptions and scores them `inconclusive`, so a
    raising assertion does not crash the campaign -- it quietly drops the
    capability out of the matrix while the run still reports success. That is
    the failure mode this test exists to make loud.
    """
    verdict = probe.assertion({})
    assert isinstance(verdict, capabilities.Verdict)
    assert verdict.verdict in capabilities.VERDICTS


@pytest.mark.parametrize("probe", [p for p in capabilities.PROBES
                                   if p.kind == capabilities.KIND_STATIC],
                         ids=lambda p: p.id)
def test_absent_markers_are_specific(probe):
    """Static probes establish `absent` by matching a diagnostic token. Generic
    words match unrelated engine warnings, so an unrelated diagnostic would
    certify a feature as absent."""
    generic = {"unknown", "skipped", "error", "warning", "not", "field",
               "node", "physics"}
    for tok in probe.absent_markers:
        assert len(tok) >= 4, "%s: marker %r is too short" % (probe.id, tok)
        assert tok.lower() not in generic, (
            "%s: marker %r is a generic diagnostic word and would match "
            "unrelated engine output" % (probe.id, tok))


def test_verdict_rejects_unknown_values():
    with pytest.raises(ValueError):
        capabilities.Verdict("mostly-fine")


def test_documented_as_claims_are_real_verdicts():
    bad = [(p.id, p.documented_as) for p in capabilities.PROBES
           if p.documented_as is not None
           and p.documented_as not in capabilities.VERDICTS]
    assert bad == [], "documented_as must be a verdict: %s" % bad


def test_probe_ids_are_namespaced_by_family():
    """`family.name` — the report groups by family and the runner's --probes
    filter is a prefix match, so a mis-namespaced id is unselectable."""
    bad = [p.id for p in capabilities.PROBES
           if not p.id.startswith(p.family + ".")]
    assert bad == [], "ids not prefixed with their family: %s" % bad


def test_report_renders_without_any_results():
    """The matrix generator must survive empty inputs.

    It runs at the end of a campaign, after the lanes that produce its data --
    so the one time it is guaranteed to be called with nothing is the run
    where something upstream already failed, which is exactly when a crash
    here would bury the real error.
    """
    report = pytest.importorskip("report")
    text = report.render([], [], [])
    assert "capability matrix" in text.lower()


def test_report_number_formatter_never_mangles_integers():
    """`_num(20, 0)` must be "20". An unguarded rstrip("0") returns "2" -- a
    silently wrong number in a table nobody would re-derive."""
    report = pytest.importorskip("report")
    assert report._num(20.0, 0) == "20"
    assert report._num(200.0, 0) == "200"
    assert report._num(0.65, 4) == "0.65"
    assert report._num(0.0, 0) == "0"


def test_touch_rig_rest_heights_stay_separable():
    """The TouchSensor probes distinguish 'the pad touched' from 'the body
    touched' by a 10 mm difference in rest height. If those two constants ever
    converge, both verdicts silently become unattributable."""
    gap = abs(capabilities.TOUCH_REST_Z - capabilities.BODY_REST_Z)
    assert gap >= 0.009, (
        "pad-contact and body-contact rest heights are only %.4f m apart; the "
        "TouchSensor probes can no longer tell them apart" % gap)
