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

"""B3 ``measure_and_report`` -- the **sim-neutral core** of the inspect tier.

There is no artifact. The deliverable is the agent's ANSWER, and the core's job
is to measure the same two quantities itself, in metres, and check the answer
against them. Extraction is regex over the final message; the verdict is a
numeric comparison. No LLM judging anywhere (SPEC 8.1.1).

Ground truth is read from the bundle's frozen t=0 inventory:

  * ``distance_m``  -- 3-D centre-to-centre distance between the two robot
    bodies. The AABB-centre distance is measured too and either is accepted
    within tolerance, because "centre to centre" is genuinely ambiguous between
    the body origin and the geometric centre and the task must not turn on which
    one the agent picked.
  * ``taller``      -- the body with the greater world-space AABB **top z**.

⚠ Recorded ambiguity: the SPEC's initial state is "2 Huskies, one on a 0.4 m
plinth", and two Huskies are *identically tall*. Only their heights ABOVE THE
WORLD FLOOR differ. This core reads "taller" as greater top-of-AABB z, which is
what a measurement of the scene gives; an agent that answers "neither, they are
the same model" is measuring the robot rather than the scene and fails. See the
Phase 0 report -- this is a SPEC wording problem, not a grader bug.
"""

from __future__ import annotations

import math
import re

from agentbench.graders import physical as ph
from agentbench.graders.verdict import (ARTIFACT_RUNS, CORE_PHYSICAL,
                                        CORE_STRUCTURAL, GRADED_PASS, INVALID,
                                        Falsifier, Verdict)

TASK = "B3_measure_and_report"

DISTANCE_TOL_M = 0.30
# Task data: the two bodies the shipped world names.
GROUND_NAME = "husky_ground"
PLINTH_NAME = "husky_plinth"


def measure_ground_truth(bundle):
    """(distance_m, aabb_distance_m, taller_name, tops) from a frozen t=0."""
    a, b = bundle.t0.by_name(GROUND_NAME), bundle.t0.by_name(PLINTH_NAME)
    if a is None or b is None:
        return None
    dist = None
    if a.position and b.position:
        dist = math.dist(a.position, b.position)
    aabb_dist = None
    if a.aabb_center and b.aabb_center:
        aabb_dist = math.dist(a.aabb_center, b.aabb_center)
    tops = {}
    for body in (a, b):
        if body.has_aabb:
            tops[body.name] = float(body.top_z)
    taller = max(tops, key=tops.get) if len(tops) == 2 else None
    return {"distance_m": dist, "aabb_distance_m": aabb_dist,
            "taller": taller, "top_z_m": tops}


# Words that mark a number as THE separation the question asked about.
_SEPARATION_CUE = re.compile(
    r"apart|separat\w*|distance|centre.to.centre|center.to.center|between",
    re.IGNORECASE)
# How far from a cue a number may sit and still be read as that cue's number.
CUE_WINDOW_CHARS = 40


def _gap(a, b):
    """Characters between two (start, end) spans; 0 when they overlap."""
    (a0, a1), (b0, b1) = a, b
    if a1 <= b0:
        return b0 - a1
    if b1 <= a0:
        return a0 - b1
    return 0


def stated_distance(answer):
    """``(metres, how_it_was_picked)`` -- the ONE distance the answer commits
    to, or ``(None, why)``.

    B3.2 used to take ``min`` of the error over EVERY number in the answer
    against BOTH ground truths. That made a shotgun answer score a perfect
    match with no measurement at all: a list at 0.5 m spacing is never further
    than 0.25 m from any target and the tolerance is 0.30 m, so "I did not
    inspect anything; the separation is somewhere in this range: 0.5, 1.0, ...
    8.0 m" covered the whole plausible span and passed. An answer to "how far
    apart are they" has to COMMIT to a number, so exactly one is scored; the
    others are recorded for a reviewer and never compared.

    Preference order, all positional and none of it judging the answer's
    meaning:

      1. the unit-tagged distance CLOSEST to a separation cue ("4.0 m apart",
         "the separation is 4.0 m") -- nearest, not first, so an answer that
         also reports a plinth height or a bounding-box top is still scored on
         the number it offered as the separation;
      2. failing that, the first unit-tagged distance;
      3. failing that, the same rule over bare numbers.
    """
    text = answer or ""
    cues = [m.span() for m in _SEPARATION_CUE.finditer(text)]
    for spans, unit in ((ph.distance_spans(text), True),
                        (ph.number_spans(text), False)):
        if not spans:
            continue
        label = ("unit-tagged distance" if unit
                 else "number (none carried a unit)")
        if cues:
            gap, i, v = min((min(_gap((s, e), c) for c in cues), i, v)
                            for i, (v, s, e) in enumerate(spans))
            if gap <= CUE_WINDOW_CHARS:
                return v, ("the %s closest to a separation cue (%d characters "
                           "away)" % (label, gap))
        return spans[0][0], "the first %s in the answer" % label
    return None, "the answer states no number at all"


def _named_taller(answer, truth):
    """Which body the answer says is taller, or None if it does not say.

    Accepts the body's ``name`` field and nothing else. There used to be a
    keyword fallback -- any answer containing "plinth"/"raised"/"on top" was
    read as naming the plinth robot -- and because the plinth robot is BY
    CONSTRUCTION the taller one, that handed B3.3 and B3.4 to any answer that
    restated the task's own initial conditions ("the robot on the plinth is
    taller"), measured or not. Deliberately conservative: if BOTH names appear
    the one nearest a 'taller/higher' word wins; if that is undecidable, None.
    """
    text = answer or ""
    low = text.lower()
    hits = {n: [m.start() for m in re.finditer(re.escape(n.lower()), low)]
            for n in (GROUND_NAME, PLINTH_NAME)}
    marks = [m.start() for m in re.finditer(
        r"taller|higher|tallest|highest", low)]
    if marks:
        best, best_d = None, None
        for n, positions in hits.items():
            for p in positions:
                d = min(abs(p - m) for m in marks)
                if best_d is None or d < best_d:
                    best, best_d = n, d
        if best is not None and best_d is not None and best_d <= 120:
            return best
    named = [n for n, p in hits.items() if p]
    if len(named) == 1:
        return named[0]
    return None


def grade(bundle, *, answer="", self_verified=False):
    v = Verdict(TASK, self_verified=self_verified)
    v.note("'taller' is graded as greater world-space AABB top z; both robots "
           "are the same Husky model, so only their height above the floor "
           "differs (recorded SPEC ambiguity).")
    for note in bundle.notes:
        v.note(note)

    # No frozen t=0 scan at all: the stack broke, so there is nothing to
    # compare the answer against. An EMPTY scan (no error, no bodies) is a
    # different failure and falls through to the ground-truth check below.
    if bundle.t0.error:
        v.outcome = INVALID
        v.note("ground truth unavailable: %s" % bundle.t0.error)
        return v

    truth = measure_ground_truth(bundle)
    if truth is None or truth["distance_m"] is None or truth["taller"] is None:
        v.outcome = INVALID
        v.note("ground truth unavailable: the two named Huskies were not both "
               "found with bounds at t=0")
        v.measurements["truth"] = truth
        return v
    v.progress = ARTIFACT_RUNS
    v.measurements["truth"] = truth
    v.measurements["answer"] = answer

    stated = ph.distances_in(answer)
    bare = ph.numbers_in(answer)
    chosen, how = stated_distance(answer)
    v.add("B3.1", chosen is not None, "a distance is stated at all",
          measured={"unit-tagged numbers": stated, "bare numbers": bare[:8],
                    "scored (m)": chosen, "chosen by": how},
          threshold=">= 1 number in the answer", basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "a number is present",
              "an answer with no number in it at all",
              "the agent's final message was captured", answer is not None)])

    # ONE number is scored, not the best of all of them. Both readings of
    # "centre to centre" (body origin, AABB centre) are still accepted, because
    # that ambiguity is the task's, not the answer's.
    targets = [t for t in (truth["distance_m"], truth["aabb_distance_m"])
               if t is not None]
    err = (min(abs(chosen - t) for t in targets)
           if (chosen is not None and targets) else None)
    v.add("B3.2", bool(err is not None and err <= DISTANCE_TOL_M),
          "the stated distance matches the measured one",
          measured={"scored stated (m)": chosen, "chosen by": how,
                    "other numbers in the answer, not scored":
                        [x for x in (stated or bare) if x != chosen][:12],
                    "measured node-centre (m)":
                        round(truth["distance_m"], 4),
                    "measured AABB-centre (m)":
                        (round(truth["aabb_distance_m"], 4)
                         if truth["aabb_distance_m"] else None),
                    "error (m)": round(err, 4) if err is not None else None},
          threshold={"error (m)": "<= %.2f" % DISTANCE_TOL_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "distance matches",
              "a stated distance more than %.2f m from the measured one"
              % DISTANCE_TOL_M,
              "a measured ground-truth distance from t=0 geometry",
              truth["distance_m"] is not None, detail=bundle.t0.source)])

    said = _named_taller(answer, truth)
    v.add("B3.3", said is not None, "the answer says which one is taller",
          measured={"named": said},
          threshold="one of the two robot names", basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "names a body",
              "an answer that names neither body",
              "the agent's final message was captured", answer is not None)])
    v.add("B3.4", said == truth["taller"], "and it is the right one",
          measured={"named": said, "measured taller": truth["taller"],
                    "top z (m)": {k: round(x, 4)
                                  for k, x in truth["top_z_m"].items()}},
          threshold=truth["taller"], basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "names the right body",
              "naming the shorter body",
              "both bodies have a measured world-space AABB top z at t=0",
              len(truth["top_z_m"]) == 2, detail=bundle.t0.source)])

    return v.finish(GRADED_PASS)
