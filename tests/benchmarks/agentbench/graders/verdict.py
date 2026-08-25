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

"""Verdict structures: assertions, outcomes, the progress ordinal.

Rules taken straight from the SPEC and enforced here rather than per-grader:

  * completion is **binary** -- ``PASS`` requires every assertion (3.2);
  * ``progress`` (3.2) is recorded, never scored -- nothing in this module
    turns a progress level into credit;
  * every assertion carries its **measured value in physical units**, so a
    row says *why*, not just *no* (8.1.1: no LLM judging, ever);
  * a missing physics-backend attribution is ``INVALID``, not ``FAIL``
    (A1.10) -- graders raise ``Invalid`` for that class of problem.

Two things a cross-simulator row must carry, added with the sim-agnostic
grader split:

**Basis.** Every assertion says whether it was measured by the core in physical
units (``core-physical``), computed by the core over the neutral inventory
(``core-structural``), asserted by the per-sim adapter (``adapter-attested``),
or a core check consuming an adapter-attested fact (``mixed``). A reader
comparing OmniSim against upstream Webots can then see which halves of a
verdict were measured the same way on both sides -- without that, "A1.1 passed
on both" hides the fact that one side counted URDF references and the other
counted PROTO instances.

**Falsifiability.** Each assertion declares how it *could* fail and what
evidence must exist for that failure to be observable, and the grader records
whether that evidence was actually present. A1.3's robot-robot contact clause
could not fail for weeks -- the node id lookup returned the queried solid's own
id, so every contact pair was ``(id, id)`` and the filter matched nothing,
forever, on every world. It read as a pass. An assertion with an absent witness
is now reported as ``vacuous`` alongside its ``ok``: still not a FAIL (the
measurement may be perfectly true), but never again silently indistinguishable
from a working check.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# 3.3 Outcomes
PASS = "PASS"
FAIL = "FAIL"
NOT_EXPRESSIBLE = "NOT_EXPRESSIBLE"
INVALID = "INVALID"
SKIPPED = "SKIPPED"
ERROR = "ERROR"

# 3.2 Progress ordinal -- reported, never scored.
NO_ARTIFACT = 0
ARTIFACT_INVALID = 1
ARTIFACT_LOADS = 2
ARTIFACT_RUNS = 3
GRADED_PASS = 4

PROGRESS_NAMES = {
    0: "no_artifact", 1: "artifact_invalid", 2: "artifact_loads",
    3: "artifact_runs", 4: "graded_pass",
}


class Invalid(Exception):
    """The stack broke, or the run is unattributable. Not a FAIL."""


# How an assertion was established. A cross-sim reader needs this to know which
# halves of two verdicts are like-for-like.
CORE_PHYSICAL = "core-physical"        # measured by the core, in SI units
CORE_STRUCTURAL = "core-structural"    # counted by the core over the inventory
ADAPTER_ATTESTED = "adapter-attested"  # the adapter asserted it, with a citation
MIXED = "mixed"                        # a core check over an attested fact

BASES = (CORE_PHYSICAL, CORE_STRUCTURAL, ADAPTER_ATTESTED, MIXED)


@dataclass
class Falsifier:
    """One way an assertion could fail, and whether that was observable.

    ``how_to_fail`` is the counter-example: the state of the world that WOULD
    turn this clause false. ``witness`` is the evidence that must be present for
    such a state to be *reported* at all; ``present`` is whether it was. A
    clause with ``present=False`` is **vacuous** -- it passed, but it could not
    have done anything else, so it is not evidence of anything.
    """

    clause: str
    how_to_fail: str
    witness: str
    present: bool
    detail: str = ""

    def as_dict(self):
        return {"clause": self.clause, "how_to_fail": self.how_to_fail,
                "witness": self.witness, "witness_present": bool(self.present),
                "detail": self.detail}


@dataclass
class Assertion:
    id: str
    ok: bool
    what: str                       # what it asserts, in one line
    measured: object = None         # the number(s), in physical units
    threshold: object = None
    detail: str = ""
    basis: str = CORE_PHYSICAL      # see BASES
    attested: list = field(default_factory=list)   # adapter claims + citations
    falsifiers: list = field(default_factory=list)  # how it could have failed

    @property
    def vacuous_clauses(self):
        return [f for f in self.falsifiers if not f.present]

    @property
    def vacuous(self):
        """This assertion has at least one clause that could not have failed."""
        return bool(self.vacuous_clauses)

    def as_dict(self):
        return {"ok": bool(self.ok), "what": self.what,
                "measured": self.measured, "threshold": self.threshold,
                "detail": self.detail,
                "basis": self.basis,
                "attested": list(self.attested),
                "vacuous": self.vacuous,
                "falsifiability": [f.as_dict() for f in self.falsifiers]}


@dataclass
class Verdict:
    task: str
    outcome: str = FAIL
    progress: int = NO_ARTIFACT
    assertions: list = field(default_factory=list)
    self_verified: bool = False
    attribution: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    artifacts: dict = field(default_factory=dict)
    measurements: dict = field(default_factory=dict)

    # --- building ---------------------------------------------------------
    def add(self, aid, ok, what, measured=None, threshold=None, detail="",
            basis=CORE_PHYSICAL, attested=(), falsifiers=()):
        self.assertions.append(Assertion(aid, ok, what, measured, threshold,
                                         detail, basis, list(attested),
                                         list(falsifiers)))
        return ok

    def note(self, text):
        self.notes.append(text)

    # --- reading ----------------------------------------------------------
    @property
    def all_ok(self):
        return bool(self.assertions) and all(a.ok for a in self.assertions)

    @property
    def failed(self):
        return [a.id for a in self.assertions if not a.ok]

    @property
    def failed_assertion(self):
        f = self.failed
        return f[0] if f else None

    @property
    def vacuous(self):
        """{assertion id: [clause names]} for every clause that could not fail.

        Deliberately NOT folded into the outcome: a vacuous clause may still be
        stating something true, and silently failing a run because our own
        evidence was thin would be scoring the harness rather than the agent.
        It is loud in the row instead, which is all the ``(id, id)`` contact bug
        ever needed.
        """
        return {a.id: [f.clause for f in a.vacuous_clauses]
                for a in self.assertions if a.vacuous}

    def basis_summary(self):
        out = {b: [] for b in BASES}
        for a in self.assertions:
            out.setdefault(a.basis, []).append(a.id)
        return {k: v for k, v in out.items() if v}

    def finish(self, progress_if_pass=GRADED_PASS):
        """Set the outcome from the assertions. PASS == every one of them."""
        if self.outcome in (INVALID, ERROR, NOT_EXPRESSIBLE, SKIPPED):
            return self
        if self.all_ok:
            self.outcome = PASS
            self.progress = progress_if_pass
        else:
            self.outcome = FAIL
        return self

    def as_dict(self):
        return {
            "task": self.task,
            "outcome": self.outcome,
            "progress": self.progress,
            "progress_name": PROGRESS_NAMES.get(self.progress),
            "assertions": {a.id: a.as_dict() for a in self.assertions},
            "n_pass": sum(1 for a in self.assertions if a.ok),
            "n_total": len(self.assertions),
            "failed_assertion": self.failed_assertion,
            "failed_assertions": self.failed,
            "self_verified": self.self_verified,
            "attribution": self.attribution,
            "measurements": self.measurements,
            "artifacts": self.artifacts,
            "notes": self.notes,
            # --- added with the sim-agnostic split; additive, so a v0 row
            # stays comparable. See the module docstring.
            "basis": self.basis_summary(),
            "vacuous_assertions": self.vacuous,
        }

    def summary(self):
        head = "%s %s  %d/%d" % (self.task, self.outcome,
                                 sum(1 for a in self.assertions if a.ok),
                                 len(self.assertions))
        lines = [head]
        for a in self.assertions:
            lines.append("  [%s] %-6s %-8s %-46s measured=%s threshold=%s%s"
                         % ("x" if a.ok else " ", a.id, _BASIS_TAG.get(
                             a.basis, a.basis), a.what,
                            _fmt(a.measured), _fmt(a.threshold),
                            ("  -- " + a.detail) if a.detail else ""))
            for f in a.vacuous_clauses:
                lines.append("        ~vacuous: %r could not have failed -- "
                             "witness absent (%s)%s"
                             % (f.clause, f.witness,
                                ("; " + f.detail) if f.detail else ""))
        return "\n".join(lines)


_BASIS_TAG = {CORE_PHYSICAL: "phys", CORE_STRUCTURAL: "struct",
              ADAPTER_ATTESTED: "adapter", MIXED: "mixed"}


def _fmt(v):
    if isinstance(v, float):
        return "%.4g" % v
    if isinstance(v, list) and v and all(isinstance(x, float) for x in v):
        return "[" + ", ".join("%.4g" % x for x in v) + "]"
    return str(v)
