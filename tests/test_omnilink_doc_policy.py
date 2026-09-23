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

"""The OmniLink access policy, enforced on the pages a customer reads.

WHY THIS EXISTS -- do not delete it as pedantry.

On 2026-09-22 the owner set the access policy that the rest of this file
measures against:

    An OmniKey is required for every OmniLink AI experience, including
    Free. Keyless chat, automatic local-model selection, and a
    basic-command fallback after a connection error must not be
    restored. Ordinary OmniSim typed controls and Stop remain available.

The code already worked that way. The documentation did not. `SUPPORT.md`
told readers "OmniSim runs fully offline and needs no account -- the demos
fall back to a local model and then to an offline intent router", which was
both against the policy and false in code: all four bridges answer a
keyless `/prompt` with 401 `omnikey_required` or 503 and actuate nothing.
That sentence sat unread from 2026-08-25 until a release audit found it.
`docs/guide/omnilink-add-your-robot.md` advertised an "offline mode
(parser only, no key)" and pointed at a keyword ladder that was being
deleted the same week.

Four families of claim drift back in whenever someone describes the
product from memory instead of from the code, so all four are checked:

  1. keyless / no-account access to the AI experience,
  2. an automatic local-model or offline fallback,
  3. the retired keyword ladder ("intent router", "regex router"),
     described as a live code path,
  4. the WITHDRAWN cost comparisons ("zero cost", "zero account",
     "costs nothing", "free per task", "cheaper than ...").

Family 4 was added on 2026-09-22 after the first sweep. The scanner covered
keyless and ladder language but nothing about price, and
`projects/samples/demos/worlds/flagship/WAREHOUSE_OMNILINK.md:112` proved the
gap was real: one table row sold a keyless path *and* priced it, "Zero
account, zero cost, runs on your GPU." The owner withdrew every cost
comparison, so a customer-facing page may state what a plan requires and must
not compare what it costs.

WHAT THIS TEST DOES NOT DO. A bare phrase list would be useless here,
because the same words appear legitimately all over the tree:

  - `CHANGELOG.md` is the historical development record and is covered by
    the disclaimer at the top of that file. It is exempt, permanently, and
    `test_changelog_is_not_in_scope` pins that.
  - A sentence that marks the thing as retired, removed or historical is
    the *fix*, not the violation -- this page's own corrected rows say
    "keyword ladder" more often than the broken ones did.
  - A negated sentence ("there is no keyless chat mode") is how the policy
    is stated. It must not trip.
  - Unrelated senses exist: `positioning.md` describes a regex SELF-SCAN of
    emit call sites, and `ladder0` is the name of a physics benchmark
    scene. Word boundaries and two-word phrases keep both out.

So the scan is scoped to a named list of customer-facing pages, runs on
prose only (fenced code blocks are blanked), and clears a match whose
enclosing sentence -- or whose whole table row -- carries a retirement or
negation marker. `test_scanner_*` pins that behaviour in both directions so
the scanner itself cannot rot into a rubber stamp.

NOT YET IN SCOPE: nothing. `PROTOCOL.md` was the last page on this list and
joined `CUSTOMER_FACING` on 2026-09-22, when §5.7.1's fourth cascade stage and
§16's matching gap bullet were rewritten to say the ladders are deleted and a
relay-less bridge answers `401 omnikey_required`.

DELIBERATELY NOT IN SCOPE, with a reason rather than an oversight:

  - `tests/benchmarks/**/results*/` and every other recorded run file. A
    result is evidence. Rows made in the ladder era say so in their own
    fields and are never edited to match a later policy; the how-to pages
    that describe those runs are in scope instead, and say which rows are
    history.
  - Source docstrings and `--help` strings (`matrix.py`, `ol_driver.py`,
    `run.py`, `bench_omnilink.py`, `courier_intent.py`). They belong to the
    code that will be rewritten with them; this file scans prose pages.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# --------------------------------------------------------------------------- #
# Scope                                                                        #
# --------------------------------------------------------------------------- #

#: Pages a customer or an integrator reads to learn how access works.
#: Paths are relative to the repository root. Add a page here once it is
#: clean; never add `CHANGELOG.md` (see the module docstring).
CUSTOMER_FACING: tuple[str, ...] = (
    "SUPPORT.md",
    "ARCHITECTURE.md",
    "docs/guide/omnilink-chat-demos.md",
    "docs/guide/omnilink-add-your-robot.md",
    "docs/guide/omnilink-sim-to-real.md",
    "docs/guide/omnilink-key-and-api.md",
    "docs/developer/positioning.md",
    "packages/omnisim-bridges/README.md",
    "packages/omnisim-bridges/GATE_COVERAGE.md",
    "projects/samples/demos/worlds/chat/LOCAL_OLLAMA.md",
    "projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md",
    # Added 2026-09-22, second sweep: the demo, agent and benchmark pages a
    # reader reaches from DEMOS.md / WORLDS.md / the agent registry. Every
    # one of these described the deleted keyword ladder as a live
    # fall-through, and the first one priced it as well.
    "projects/samples/demos/worlds/flagship/WAREHOUSE_OMNILINK.md",
    "agents/production/omnitug500_warehouse/README.md",
    "agents/production/omnitug500_warehouse/docs/OVERVIEW.md",
    "agents/production/husky_swarm/README.md",
    "docs/developer/agents-reference-sections.md",
    "docs/developer/simulator-comparison.md",
    # docs/why-omnilink.md was listed here and is NOT customer-facing: it is on
    # scripts/release/publish_deny.txt, so it never reaches the public tree. On
    # the public CI run of v9.0.0-rc.2 the file was absent and this module failed
    # twice -- `test_every_scoped_page_exists` and the per-page policy check.
    # It is still read internally and was still corrected; it just is not in
    # scope for a gate about what customers read. See
    # test_no_scoped_page_is_held_back_from_the_public_tree below.
    "tests/benchmarks/omnilink_tasks/README.md",
    "tests/benchmarks/warehouse/BENCH_OMNILINK.md",
    "tests/benchmarks/warehouse/GOALS_SUITE.md",
    "tests/benchmarks/commandbench/README.md",
    # Added 2026-09-22, third sweep: the wire contract itself. §5.7.1 described
    # the deleted keyword ladder as a live fourth cascade stage, and §16
    # repeated it as an open gate-coverage hole.
    "PROTOCOL.md",
)

# --------------------------------------------------------------------------- #
# The forbidden claims                                                         #
# --------------------------------------------------------------------------- #

#: (label, pattern). Each is a CLAIM about how the product works today.
#: Two-word phrases and word boundaries on purpose: `IntentRouter` the class
#: name, `OMNISIM_BRIDGE_LEGACY_ROUTER` the variable and `ladder0` the
#: benchmark scene are all identifiers, not claims, and none of them match.
FORBIDDEN: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("keyless access", re.compile(r"\bkeyless\b", re.I)),
    ("offline mode", re.compile(r"\boffline\s+mode\b", re.I)),
    ("no account needed", re.compile(r"\b(?:no|zero)\s+account\b", re.I)),
    ("no key needed", re.compile(r"\bneeds?\s+no\s+key\b", re.I)),
    ("local-model fallback",
     re.compile(r"\bfalls?\s+back\s+(?:down\s+)?to\s+(?:a\s+|the\s+)?local\s+model", re.I)),
    ("keyword ladder", re.compile(r"\bkeyword\s+ladder", re.I)),
    ("legacy ladder", re.compile(r"\blegacy\s+ladder", re.I)),
    ("intent router", re.compile(r"\bintent\s+router", re.I)),
    ("regex router", re.compile(r"\bregex\s+router", re.I)),
    # Family 4: the WITHDRAWN cost comparisons. Two-word phrases for the
    # same reason as the rest -- a benchmark row reading `$0.00` in a cost
    # column, or the sentence "the Free plan", is a measurement or a plan
    # name, not a comparison, and neither matches.
    ("zero cost", re.compile(r"\b(?:zero|no)\s+cost\b", re.I)),
    ("costs nothing", re.compile(r"\bcosts?\s+(?:you\s+|us\s+|them\s+)?nothing\b", re.I)),
    ("free of charge",
     re.compile(r"\bfree\s+(?:forever|of\s+charge|to\s+run|per\s+\w+)\b", re.I)),
    ("cheaper than", re.compile(r"\b(?:cheaper|costs?\s+less)\s+than\b", re.I)),
)

#: A match inside a unit carrying one of these is a retirement notice, a
#: negation or a historical note -- i.e. the policy being stated correctly.
MARKERS: tuple[re.Pattern[str], ...] = (
    re.compile(r"\bretire(?:d|s|ment)?\b", re.I),
    re.compile(r"\bremov(?:e|es|ed|ing|al)\b", re.I),
    re.compile(r"\bdelet(?:e|es|ed|ing|ion)\b", re.I),
    re.compile(r"\bdeprecat(?:e|ed|ion)\b", re.I),
    re.compile(r"\bhistor(?:y|ical)\b", re.I),
    re.compile(r"\bsupersed(?:e|es|ed)\b", re.I),
    re.compile(r"\bwithdrawn\b", re.I),
    re.compile(r"\bno longer\b", re.I),
    re.compile(r"\bused to\b", re.I),
    re.compile(r"\bnever\b", re.I),
    re.compile(r"\b(?:there|that|this|it)\s+(?:is|are|was|were)\s+no\b", re.I),
    re.compile(r"\b(?:is|are|was|were|do|does|did|must|should|will|can|could)\s+not\b", re.I),
    re.compile(r"\bnot\s+a\b", re.I),
    re.compile(r"~~"),  # struck-through row or phrase
)

#: A negation sitting immediately in front of the match ("No keyless chat
#: mode", "no automatic local-model fallback", "without a keyless agent").
NEGATION_BEFORE = re.compile(
    r"\b(?:no|not|never|without|nor|neither|zero)\b(?:\s+\S+){0,3}\s*$", re.I)

#: Labels for which NEGATION_BEFORE must NOT excuse a match.
#:
#: For families 1-3 a leading "no"/"zero" flips the claim: "no keyless chat"
#: is the policy. For the cost family the leading word IS the claim -- "zero
#: cost", "no cost" -- and "Zero setup, zero cost" (WAREHOUSE_OMNILINK.md:112,
#: the row that motivated this family) would otherwise excuse itself, because
#: the FIRST "zero" sits three words in front of the second. A retirement or
#: negation MARKER still clears these; only the proximity rule is withdrawn.
NO_NEGATION_EXCUSE: frozenset[str] = frozenset({
    "zero cost", "costs nothing", "free of charge", "cheaper than",
})

FENCE = re.compile(r"^\s*(?:```|~~~)")
SENTENCE_BOUNDARY = re.compile(r"(?<=[.!?:;])\s")


def _strip_code_fences(text: str) -> str:
    """Blank out fenced blocks, preserving line numbering."""
    out, inside = [], False
    for line in text.splitlines():
        if FENCE.match(line):
            inside = not inside
            out.append("")
            continue
        out.append("" if inside else line)
    return "\n".join(out)


def _unit(text: str, start: int, end: int) -> str:
    """The sentence around a match -- or the whole row, inside a table.

    A markdown table splits one statement across cells: the phrase lands in
    the first cell and the retirement notice in the next. Cell-level context
    would call that a violation, so a table row is taken whole.
    """
    line_start = text.rfind("\n", 0, start) + 1
    line_end = text.find("\n", end)
    line_end = len(text) if line_end == -1 else line_end
    line = text[line_start:line_end]
    if line.lstrip().startswith("|"):
        return line
    # Paragraph = the run of non-blank lines around the match.
    para_start = text.rfind("\n\n", 0, start)
    para_start = 0 if para_start == -1 else para_start + 2
    para_end = text.find("\n\n", end)
    para_end = len(text) if para_end == -1 else para_end
    para = text[para_start:para_end]
    offset = start - para_start
    pieces, cursor = [], 0
    for split in SENTENCE_BOUNDARY.finditer(para):
        pieces.append((cursor, split.end()))
        cursor = split.end()
    pieces.append((cursor, len(para)))
    for lo, hi in pieces:
        if lo <= offset < hi:
            return para[lo:hi]
    return para


def _is_excused(unit: str, match_start_in_unit: int, label: str = "") -> bool:
    if any(m.search(unit) for m in MARKERS):
        return True
    if label in NO_NEGATION_EXCUSE:
        return False
    return bool(NEGATION_BEFORE.search(unit[:match_start_in_unit]))


def scan(text: str) -> list[tuple[int, str, str]]:
    """Return (line_no, label, sentence) for every un-excused claim."""
    prose = _strip_code_fences(text)
    hits: list[tuple[int, str, str]] = []
    for label, pattern in FORBIDDEN:
        for m in pattern.finditer(prose):
            unit = _unit(prose, m.start(), m.end())
            where = unit.find(m.group(0))
            if _is_excused(unit, where if where >= 0 else 0, label):
                continue
            line_no = prose.count("\n", 0, m.start()) + 1
            hits.append((line_no, label, " ".join(unit.split())))
    return sorted(hits)


# --------------------------------------------------------------------------- #
# The gate                                                                     #
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("rel", CUSTOMER_FACING)
def test_customer_facing_page_states_the_access_policy(rel: str) -> None:
    path = REPO_ROOT / rel
    assert path.is_file(), f"{rel} is in CUSTOMER_FACING but does not exist"
    hits = scan(path.read_text(encoding="utf-8"))
    if hits:
        report = "\n".join(
            f"  {rel}:{line}  [{label}]\n      {sentence}"
            for line, label, sentence in hits)
        pytest.fail(
            "OmniLink access policy (2026-09-22): an OmniKey is required for "
            "every OmniLink AI experience, including Free. There is no keyless "
            "chat, no automatic local-model selection and no basic-command "
            "fallback after a connection error; the keyword ladder is retired.\n"
            f"{len(hits)} sentence(s) claim otherwise as current behaviour:\n"
            f"{report}\n"
            "Rewrite each to the current truth, or mark it explicitly as "
            "RETIRED / REMOVED / HISTORICAL if it is a note about the past. "
            "See the module docstring in tests/test_omnilink_doc_policy.py.")


def test_changelog_is_not_in_scope() -> None:
    """The v8-and-earlier changelog is the historical development record."""
    assert "CHANGELOG.md" not in CUSTOMER_FACING


def test_every_scoped_page_exists() -> None:
    missing = [r for r in CUSTOMER_FACING if not (REPO_ROOT / r).is_file()]
    assert not missing, f"CUSTOMER_FACING names files that do not exist: {missing}"


# --------------------------------------------------------------------------- #
# The scanner itself                                                           #
# --------------------------------------------------------------------------- #

VIOLATIONS = [
    "OmniSim runs fully offline and needs no account -- the demos fall back "
    "to a local model and then to an offline intent router.",
    "Both **offline mode** (parser only, no key, nothing leaves the machine) "
    "and **OmniLink mode** work.",
    "Set OMNI_KEY to upgrade from the regex router to the live agent.",
    "| keyword ladders | the bridges' own dispatch | operator text |",
    "The utterance check sits in front of the legacy ladder.",
    "Chat is keyless on the Free plan.",
    # Family 4 -- the withdrawn cost comparisons. The first is verbatim from
    # WAREHOUSE_OMNILINK.md:112, which is what proved the gap.
    "| **Local Ollama** | leave `OMNI_KEY` unset | Zero account, zero cost, "
    "runs on your GPU. |",
    "Literal single commands only, matched by regex. Zero setup, zero cost.",
    "Inference stays local and it costs you nothing.",
    "Running the demo this way is free forever.",
    "Answering a turn locally is cheaper than a platform round-trip.",
]

LEGITIMATE = [
    "There is no keyless chat mode or automatic local-model fallback.",
    "No keyless AI mode or automatic basic-command fallback is provided.",
    "The keyword ladder is retired and its module is deleted.",
    "| ~~keyword ladders~~ | **RETIRED 2026-09-22.** Every ladder is gone |",
    "Each bridge used to keep a keyword ladder underneath the parser.",
    "This row is HISTORICAL: the offline mode it describes no longer exists.",
    "The mock examples' /prompt does not provide a keyless agent.",
    "A regex self-scan of the emit call sites counts the event types.",
    "The ladder0 benchmark scene stacks ten rungs.",
    "IntentRouter and OMNISIM_BRIDGE_LEGACY_ROUTER are identifiers.",
    "Removing the keyless product flow does not retract published source.",
    # Family 4, in both directions. A plan NAME, a measured cost column and a
    # retirement notice about a price claim are all legitimate.
    "An OmniKey is required on every plan, including Free.",
    "The cost comparisons were withdrawn and no page may revive them.",
    "| g5-engine | qwen3:32b | 2/4 | 159 s | $0.00 | $0.53/hr (A6000) |",
    "That row used to say zero cost; the comparison is withdrawn.",
    "This page must never claim the demo is free of charge.",
    "The deterministic parser answers the turn with no model round-trip.",
]


@pytest.mark.parametrize("text", VIOLATIONS)
def test_scanner_catches_a_violation(text: str) -> None:
    assert scan(text), f"scanner missed a policy violation: {text!r}"


@pytest.mark.parametrize("text", LEGITIMATE)
def test_scanner_allows_a_legitimate_mention(text: str) -> None:
    assert not scan(text), f"scanner false-positived on: {text!r}"


def test_fenced_code_is_not_scanned() -> None:
    assert not scan("```\ncurl --offline-mode --keyless\n```\n")


def test_failure_message_names_file_line_and_sentence() -> None:
    hits = scan("intro line\n\nChat is keyless on the Free plan.\n")
    assert hits == [(3, "keyless access", "Chat is keyless on the Free plan.")]


def _publish_denied(rel: str) -> str | None:
    """The publish deny-list rule that holds `rel` back, or None if it ships.

    Mirrors how scripts/release/publish_snapshot.sh applies the list: an exact
    path, a directory prefix ending in `/`, or a `*` glob.
    """
    deny = REPO_ROOT / "scripts" / "release" / "publish_deny.txt"
    if not deny.is_file():  # the deny-list is itself deny-listed; absent on public
        return None
    for raw in deny.read_text(encoding="utf-8").splitlines():
        rule = raw.strip()
        if not rule or rule.startswith("#"):
            continue
        if rel == rule or (rule.endswith("/") and rel.startswith(rule)):
            return rule
        if "*" in rule and re.fullmatch(re.escape(rule).replace(r"\*", ".*"), rel):
            return rule
    return None


def test_no_scoped_page_is_held_back_from_the_public_tree() -> None:
    """A deny-listed page is never customer-facing, and scoping one breaks CI.

    `docs/why-omnilink.md` sat in CUSTOMER_FACING while publish_deny.txt held it
    back, so on the public tree -- where it does not exist -- this module failed
    `test_every_scoped_page_exists` and the per-page check. That was the public
    CI run of v9.0.0-rc.2. This pins the rule so the next page added to the
    scope cannot reopen it. It checks on the private tree, where the deny-list
    exists; on the public tree the deny-list is absent and there is nothing to
    compare against, which is also where the scoped pages all genuinely exist.
    """
    held = [(page, _publish_denied(page)) for page in CUSTOMER_FACING]
    held = [(page, rule) for page, rule in held if rule]
    assert not held, (
        "CUSTOMER_FACING lists pages the publish deny-list holds back, so they "
        "never reach a customer and are absent from the public tree: "
        + ", ".join(f"{page} (rule {rule!r})" for page, rule in held)
    )
