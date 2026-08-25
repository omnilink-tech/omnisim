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

"""B1 ``overlap_audit`` -- the **sim-neutral core** of the inspect tier.

Prompt (verbatim): *"Are any two robots in this scene overlapping? Answer yes
or no and name the pair."* There is no artifact: the deliverable is the
agent's ANSWER, and the core measures the same thing itself -- in metres --
and checks the answer against the measurement. Extraction is positional regex
over the final message; the verdict is a numeric comparison. No LLM judging
anywhere (SPEC 8.1.1).

Ground truth is derived from the bundle's frozen t=0 inventory, never from a
hardcoded pair name, so the grader holds if the world file changes:

  * a pair **overlaps** when its two world-space AABBs interpenetrate by at
    least ``OVERLAP_MIN_M`` on every axis (clearance <= -OVERLAP_MIN_M);
  * a pair is **clear** when its best-axis separation is at least
    ``CLEAR_MIN_M`` (the same 0.05 m floor A1.3 uses);
  * a pair in the band between the two is **ambiguous**, and a scene carrying
    one is not gradable -- the verdict is INVALID, because whichever way we
    scored it we would be grading our own threshold, not the agent.

The AABB is a proxy for the body volume (two boxes can overlap while the
meshes do not), which is why the shipped world places its one overlapping
pair 0.3 m apart -- deep interpenetration -- and every other pair metres
clear: on that geometry the proxy and the truth cannot disagree. The
thresholds guard the same property against future world edits.

**Anti-shotgun (the B3 lesson, commit 1fb331a7).** An answer must COMMIT:

  * exactly one yes/no verdict is read -- the first committed cue in the
    message, so "Yes -- A and B overlap; no other pair does" reads as yes,
    and a message with no cue at all is "no verdict", never a keyword-scored
    pass;
  * exactly one pair is read from a yes answer. Two robot names -> that pair.
    More names -> only a pair uniquely adjacent to an overlap cue counts;
    listing every pair yields NO committed pair and fails the naming
    assertions rather than scoring a 1-in-15 lottery ticket.

**Vacuity (the A1.3 lesson, plan 5.5).** Every geometric clause names its
witness: a complete, genuinely frozen t=0 scan with a world-space AABB for
every robot. Missing pieces are INVALID (nothing to grade against); a scan
that exists but is not frozen grades with the witness marked absent, so the
row says ``vacuous`` out loud instead of quietly certifying a moved scene.
"""

from __future__ import annotations

import re

from agentbench.graders import physical as ph
from agentbench.graders.verdict import (ARTIFACT_RUNS, CORE_PHYSICAL,
                                        CORE_STRUCTURAL, GRADED_PASS, INVALID,
                                        Falsifier, Verdict)

TASK = "B1_overlap_audit"

# A pair "overlaps" at >= this interpenetration depth on its shallowest axis.
OVERLAP_MIN_M = 0.05
# A pair is "clear" at >= this best-axis separation (A1.3's floor).
CLEAR_MIN_M = 0.05


def pair_key(a, b):
    """Order-free spelling of a pair, used everywhere a pair is a dict key."""
    return "+".join(sorted((a, b)))


# --- ground truth ------------------------------------------------------------


def measure_ground_truth(bundle):
    """Pairwise AABB clearances over every robot at t=0, or ``None``.

    Returns ``None`` only when there are fewer than two robot bodies to
    compare. Everything else -- including incomplete AABBs -- is reported in
    the dict so the caller can name exactly what is missing.
    """
    robots = bundle.t0.robots
    if len(robots) < 2:
        return None
    names = [b.name or b.body_id for b in robots]
    measured = [b for b in robots if b.has_aabb]
    clearances = {}
    for i in range(len(measured)):
        for j in range(i + 1, len(measured)):
            a, b = measured[i], measured[j]
            key = pair_key(a.name or a.body_id, b.name or b.body_id)
            clearances[key] = ph.aabb_clearance(a.aabb, b.aabb)
    overlapping = sorted(k for k, c in clearances.items()
                         if c <= -OVERLAP_MIN_M)
    ambiguous = sorted(k for k, c in clearances.items()
                       if -OVERLAP_MIN_M < c < CLEAR_MIN_M)
    return {
        "names": names,
        "n_robots": len(robots),
        "measured_all": len(measured) == len(robots),
        "unmeasured": sorted((b.name or b.body_id)
                             for b in robots if not b.has_aabb),
        "frozen": bundle.t0.frozen is True,
        "pair_clearance_m": {k: round(c, 4) for k, c in clearances.items()},
        "overlapping": overlapping,
        "ambiguous": ambiguous,
    }


# --- answer extraction -------------------------------------------------------

_YES_WORD = re.compile(r"\byes\b", re.IGNORECASE)
_NO_WORD = re.compile(r"\b(?:no|none|nope)\b", re.IGNORECASE)
# Words that assert the overlap state -- committed cues only when not negated.
_OVERLAP_WORD = re.compile(
    r"\b(?:overlap\w*|interpenetrat\w*|intersect\w*|collid\w*)", re.IGNORECASE)
_NEGATION = re.compile(
    r"\b(?:no|not|none|never|neither|nor|without)\b|n[o']t\b",
    re.IGNORECASE)

# How far back a negation reaches to flip an overlap word ("do not overlap").
NEG_WINDOW_CHARS = 18
# How far from a cue a robot name may sit and still be read as that cue's
# subject (the same window b3 uses for its separation cue).
CUE_WINDOW_CHARS = 40


def _overlap_cues(text):
    """Spans of overlap words that are NOT negated just before them."""
    out = []
    for m in _OVERLAP_WORD.finditer(text):
        back = text[max(0, m.start() - NEG_WINDOW_CHARS):m.start()]
        if not _NEGATION.search(back):
            out.append(m.span())
    return out


def stated_verdict(answer):
    """``("yes"|"no"|None, how_it_was_read)`` -- the ONE verdict committed to.

    The first committed cue in the message wins, so a yes answer may go on to
    say "no other pair does" without flipping itself. Cues, in position
    order: a bare "yes"; a bare "no"/"none"; an overlap word, read as yes
    when not negated within the preceding ``NEG_WINDOW_CHARS`` characters and
    as no when it is. A message with no cue commits to nothing, which is its
    own failure -- there is no keyword-fallback scoring.
    """
    text = answer or ""
    cues = [(m.start(), "yes", "a bare 'yes'") for m in
            _YES_WORD.finditer(text)]
    cues += [(m.start(), "no", "a bare 'no'/'none'") for m in
             _NO_WORD.finditer(text)]
    for m in _OVERLAP_WORD.finditer(text):
        back = text[max(0, m.start() - NEG_WINDOW_CHARS):m.start()]
        if _NEGATION.search(back):
            cues.append((m.start(), "no",
                         "a negated overlap word (%r)" % m.group(0)))
        else:
            cues.append((m.start(), "yes",
                         "an un-negated overlap word (%r)" % m.group(0)))
    if not cues:
        return None, "the answer commits to no yes/no verdict at all"
    pos, verdict, how = min(cues)
    return verdict, "first committed cue: %s at character %d" % (how, pos)


def _name_mentions(text, names):
    """{name: [spans]} for every roster name mentioned in the answer."""
    low = text.lower()
    out = {}
    for n in names:
        if not n:
            continue
        pat = re.compile(r"(?<![A-Za-z0-9_])" + re.escape(n.lower())
                         + r"(?![A-Za-z0-9_])")
        spans = [m.span() for m in pat.finditer(low)]
        if spans:
            out[n] = spans
    return out


def _gap(a, b):
    """Characters between two (start, end) spans; 0 when they overlap."""
    (a0, a1), (b0, b1) = a, b
    if a1 <= b0:
        return b0 - a1
    if b1 <= a0:
        return a0 - b1
    return 0


def stated_pair(answer, names):
    """``(pair_key|None, how)`` -- the ONE pair the answer commits to.

    Exactly two roster names in the message -> that pair. More than two ->
    only the names within ``CUE_WINDOW_CHARS`` of an un-negated overlap cue
    are candidates, and there must be exactly two of them: an answer that
    lists every pair puts MANY names next to its cues and therefore commits
    to none. Fewer than two names -> no pair. Nothing here scores "closest
    match": a shotgun list is rejected, never sampled.
    """
    text = answer or ""
    mentions = _name_mentions(text, names)
    if len(mentions) < 2:
        return None, ("the answer names %d of the robots (two are needed "
                      "for a pair)" % len(mentions))
    if len(mentions) == 2:
        a, b = sorted(mentions)
        return pair_key(a, b), "the only two robot names in the answer"
    cues = _overlap_cues(text)
    if cues:
        near = sorted(n for n, spans in mentions.items()
                      if any(_gap(s, c) <= CUE_WINDOW_CHARS
                             for s in spans for c in cues))
        if len(near) == 2:
            return pair_key(near[0], near[1]), (
                "the two robot names adjacent to an overlap cue "
                "(of %d names mentioned)" % len(mentions))
        return None, ("%d robot names sit next to overlap cues -- no single "
                      "committed pair" % len(near))
    return None, ("%d robot names mentioned with no overlap cue to single "
                  "out a pair" % len(mentions))


# --- the grader --------------------------------------------------------------


def grade(bundle, *, answer="", self_verified=False):
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)

    # No frozen t=0 scan at all: the stack broke, so there is nothing to
    # compare the answer against.
    if bundle.t0.error:
        v.outcome = INVALID
        v.note("ground truth unavailable: %s" % bundle.t0.error)
        return v

    truth = measure_ground_truth(bundle)
    if truth is None:
        v.outcome = INVALID
        v.note("ground truth unavailable: fewer than two robot bodies were "
               "found at t=0, so no pairwise overlap can be measured")
        return v
    v.measurements["truth"] = truth
    v.measurements["answer"] = answer

    names = truth["names"]
    if len(set(names)) != len(names):
        v.outcome = INVALID
        v.note("ground truth unusable: robot names are not distinct (%s), so "
               "a named pair cannot be checked against a measured pair"
               % ", ".join(sorted(names)))
        return v
    if not truth["measured_all"]:
        v.outcome = INVALID
        v.note("ground truth incomplete: no world-space AABB at t=0 for %s -- "
               "'no other pair overlaps' cannot be certified with unmeasured "
               "robots in the scene" % ", ".join(truth["unmeasured"]))
        return v
    if truth["ambiguous"]:
        v.outcome = INVALID
        v.note("ground truth ambiguous: pair(s) %s sit between %.2f m "
               "interpenetration and %.2f m clearance, where the AABB proxy "
               "cannot honestly say 'overlapping' or 'clear'. The scene "
               "drifted; re-author it, do not re-tune the grader."
               % (", ".join(truth["ambiguous"]), OVERLAP_MIN_M, CLEAR_MIN_M))
        return v

    v.progress = ARTIFACT_RUNS
    truth_yes = bool(truth["overlapping"])
    # The witness every geometric clause depends on: complete AABBs (already
    # gated above) from a scan that is genuinely frozen at t=0.
    geometry_witness = truth["frozen"]
    if not geometry_witness:
        v.note("the t=0 inventory is not marked frozen: the geometric "
               "assertions below are graded but reported vacuous, because a "
               "free-running scan cannot answer an 'at t=0' question")

    verdict, verdict_how = stated_verdict(answer)
    pair, pair_how = stated_pair(answer, names)
    # A "no" answer's incidental names ("the closest pair is A and B") are
    # not overlap claims; only a yes answer commits a pair to the audit.
    claimed = pair if verdict == "yes" else None

    v.add("B1.1", verdict is not None,
          "the answer commits to a yes/no verdict",
          measured={"verdict": verdict, "read as": verdict_how},
          threshold="exactly one committed yes/no", basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "a verdict is present",
              "an answer that never says yes or no",
              "the agent's final message was captured", answer is not None)])

    v.add("B1.2", bool(verdict is not None
                       and (verdict == "yes") == truth_yes),
          "the verdict matches the measured scene",
          measured={"verdict": verdict,
                    "measured overlapping pairs": truth["overlapping"],
                    "pair clearances (m; negative = interpenetration)":
                        truth["pair_clearance_m"]},
          threshold={"verdict": "yes" if truth_yes else "no",
                     "overlap floor (m)": OVERLAP_MIN_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "verdict matches measured overlap",
              "a yes/no that contradicts the measured pairwise AABB "
              "interpenetration at t=0",
              "a world-space AABB for every robot from a frozen t=0 scan",
              geometry_witness, detail=bundle.t0.source)])

    v.add("B1.3", bool(verdict != "yes" or pair is not None),
          "a yes answer commits to exactly one named pair",
          measured={"robot names mentioned":
                        sorted(_name_mentions(answer or "", names)),
                    "committed pair": pair, "read as": pair_how},
          threshold="exactly two robot names offered as the pair",
          basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "one pair is named",
              "a yes answer naming no pair, or blanketing many pairs",
              "the roster names are known and the final message was captured",
              bool(names) and answer is not None)])

    v.add("B1.4", bool(claimed in truth["overlapping"] if truth_yes
                       else claimed is None),
          "the claimed pair is the measurably overlapping one",
          measured={"claimed pair": claimed,
                    "measured overlapping pairs": truth["overlapping"],
                    "claimed pair clearance (m)":
                        truth["pair_clearance_m"].get(claimed)},
          threshold=(truth["overlapping"] if truth_yes
                     else "no pair claimed as overlapping"),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "the named pair matches the measurement",
              "naming a pair whose AABBs are measurably clear of each other, "
              "or claiming a pair when every pair is clear",
              "a world-space AABB for every robot from a frozen t=0 scan",
              geometry_witness, detail=bundle.t0.source)])

    return v.finish(GRADED_PASS)
