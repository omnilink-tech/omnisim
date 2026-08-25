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

"""**The adapter contract.** One module per simulator; this file is the shape.

SPEC 6.2.6 is the rule: *"Sim-agnostic graders, per-sim adapters. Every
assertion is stated in physical units (metres, seconds, contacts, exit codes).
A grader that reads an OmniSim-shaped JSON response is a bug. Adapters are
published with the scaffolding."*

An adapter's whole job is therefore one function::

    build_bundle(task_id, *, robot_identity, live_expected, **run_evidence)
        -> agentbench.graders.evidence.EvidenceBundle

``run_evidence`` is whatever that simulator's runner produced -- for OmniSim a
``HarnessProbe`` and a ``PhaseBResult``; for upstream Webots an extern-controller
recording and a process result; for Gazebo a ROS 2 bag plus service replies. The
core graders never see any of it. They see the bundle.

Rules an adapter must follow
----------------------------

1. **Never raise.** A broken simulator is a measurement, not an exception. Put
   the reason in ``bundle.notes`` / the matching ``*_error`` and leave the field
   ``None``.
2. **``None`` means "I could not answer".** It must never be spelled ``0``,
   ``False`` or ``[]``. The vacuity detector in ``graders/verdict.py`` exists
   because an unanswerable question that looks like a zero is how A1.3 silently
   passed for weeks.
3. **SI units only, world frame.** Metres, seconds, radians, kilograms. No
   pixels, no centimetres, no per-sim frame conventions leaking through.
4. **Cite every attestation.** Anything the core cannot verify itself -- the
   robot-identity answer, the engine attribution, "this body is dynamic" -- must
   carry a ``source``/``*_rule`` string naming the channel it came from. Those
   strings are published, and they are how a reader of a cross-sim table sees
   which halves of two verdicts are like-for-like.
5. **Answer the task's identity predicate in your own terms.** The task names a
   predicate key (see ``ROBOT_IDENTITIES``); the adapter decides what it means on
   that simulator and says so in ``IdentityRule``. The *count* and the
   *distinctness* stay in the core.
6. **Do not reshape a threshold.** Adapters supply evidence. Every number an
   assertion compares against lives in the core.

Registry
--------

``resolve()`` picks the adapter for the simulator under test -- from the
``sim`` argument, else ``$AGENTBENCH_SIM``, else ``omnisim``. That single line
is what makes a grader runnable against another simulator without touching the
grader.
"""

from __future__ import annotations

import importlib
import os

# The identity predicate keys a task may declare. A task names one; every
# adapter must be able to answer it in its own terms (rule 5). Adding a key here
# is a contract change: every adapter has to grow an answer for it.
ROBOT_IDENTITIES = {
    "husky": "the Clearpath Husky, as the A1 flagship task means it",
    "any_robot": "any body the simulator treats as a robot (the body under "
                 "test in a single-robot task)",
}

# sim id -> module implementing build_bundle().
#
# Two are implemented: ``omnisim`` (the system under test) and ``webots``
# (upstream R2025a, the control arm -- Phase W). The rest are registered so that
# ``resolve()`` reports "not implemented yet" for a named simulator instead of
# "unknown sim", which is a different and much less useful message.
ADAPTERS = {
    "omnisim": "agentbench.adapters.omnisim.evidence",
    "webots": "agentbench.adapters.webots.evidence",
    "gazebo": "agentbench.adapters.gazebo.evidence",
    "isaac": "agentbench.adapters.isaac.evidence",
    "mujoco": "agentbench.adapters.mujoco.evidence",
}

# What a NEW adapter has to be able to answer before its column can be
# published. Each entry is (bundle field, what the simulator must expose). This
# list is the honest cost estimate for adding a simulator -- and a per-sim
# ``NOT_EXPRESSIBLE`` justification (SPEC 6.4) must cite one of these lines.
REQUIRED_EVIDENCE = (
    ("roster", "what bodies exist, with a name, a stable id, a joint count "
               "and the behaviour/controller attached to each"),
    ("t0", "the same inventory at a genuinely FROZEN instant, with each body's "
           "world-space AABB in metres -- plus any non-robot body the task "
           "names (a floor slab, a plinth)"),
    ("contacts", "contact pairs over the first N steps, naming both "
                 "participants where the simulator can; AND the two vacuity "
                 "counters (total contacts seen, contacts naming two distinct "
                 "bodies) so an empty result can be told apart from a query "
                 "that never names anything"),
    ("trajectory", "every body's world pose once per physics step in simulated "
                   "seconds, metres, integrated by the grader -- not polled "
                   "over a network, and not resampled"),
    ("process", "the exit code, the simulator's own error-class lines, the "
                "count of behaviour processes actually started, and "
                "independent evidence that the world reached finalize (an exit "
                "code alone is never accepted)"),
    ("attribution", "which physics engine/solver drove THIS run, with a "
                    "citation: a race-free verdict file, an explicit config "
                    "pin, or a documented structural invariant of that "
                    "simulator"),
    ("identity", "the robot-identity predicate answered on live bodies, and "
                 "the count of matching robot declarations in the artifact"),
    ("view", "for camera tasks (B2): the initial and final camera pose, each "
             "as a world-frame position, unit forward axis and field of view "
             "(plus an up axis and viewport aspect where the simulator can "
             "state them), with per-pose source citations -- see "
             "graders.evidence.CameraPose / ViewEvidence / check_view"),
)


class AdapterError(Exception):
    """Raised only by ``resolve``: the adapter module is missing."""


def check_bundle(bundle):
    """Contract violations in one bundle, as a list of strings (empty == ok).

    Executable form of the rules above, for a new adapter to run against its own
    output before anyone spends tokens on a competitor cell::

        problems = adapters.check_bundle(mod.build_bundle("A1_...", ...))
        assert not problems, problems

    It checks shape and citations -- it cannot check that the numbers are true.
    """
    p = []
    if not bundle.sim:
        p.append("bundle.sim is empty: a row must name the simulator")
    if not bundle.adapter:
        p.append("bundle.adapter is empty: a row must name the adapter")
    for name, inv in (("roster", bundle.roster), ("t0", bundle.t0)):
        if inv.bodies and not inv.source:
            p.append("%s inventory has bodies but no source citation" % name)
        for b in inv.bodies:
            if not b.body_id:
                p.append("%s: a body has no body_id (distinctness needs one)"
                         % name)
                break
        if name == "t0" and inv.bodies and inv.frozen is not True:
            p.append("t0 inventory is not marked frozen: no 'at t=0' assertion "
                     "can be graded from a free-running read")
    if bundle.t0.bodies and not any(b.has_aabb for b in bundle.t0.bodies):
        p.append("t0 inventory carries no world-space AABB: the geometric "
                 "assertions cannot be graded")
    c = bundle.contacts
    if c is not None:
        if c.supported and c.total_observed is None \
                and c.distinct_named is None:
            p.append("contacts: no vacuity witness (total_observed / "
                     "distinct_named are both None), so a 'no contact' result "
                     "cannot be told apart from a broken query -- the "
                     "dependent assertion will be reported vacuous")
        if c.pairs and not c.source:
            p.append("contacts: pairs supplied with no source citation")
    t = bundle.trajectory
    if t is not None and t.xyz is not None:
        if t.t is None or len(t.t) != t.xyz.shape[1]:
            p.append("trajectory: t and xyz disagree on the sample count")
        if len(t.body_ids) != t.xyz.shape[0]:
            p.append("trajectory: body_ids and xyz disagree on the body count")
        if t.recorded_s is None:
            p.append("trajectory: recorded_s is None, so no window-length "
                     "assertion can be graded")
        if not t.source:
            p.append("trajectory: no source citation")
    if bundle.process is not None and not bundle.process.start_source:
        p.append("process: behaviour_starts has no source citation")
    if bundle.attribution is not None and not bundle.attribution.source:
        p.append("attribution: no citation -- SPEC A1.10 needs the channel "
                 "named, not just the engine")
    if bundle.identity is not None:
        if not bundle.identity.scene_rule:
            p.append("identity: the per-sim scene rule is unpublished")
    if bundle.view is not None:
        # The camera channel has its own contract checker, next to the
        # dataclasses it validates (graders/evidence.py); delegate rather
        # than duplicate. A bundle with no view field is fine -- only camera
        # tasks carry one -- so None adds nothing here.
        from agentbench.graders.evidence import check_view
        p.extend("view: %s" % v for v in check_view(bundle.view))
    return p


def resolve(sim=None):
    """The adapter module for ``sim`` (default ``$AGENTBENCH_SIM``/omnisim)."""
    sim = sim or os.environ.get("AGENTBENCH_SIM") or "omnisim"
    try:
        path = ADAPTERS[sim]
    except KeyError:
        raise AdapterError("no adapter registered for sim %r (have: %s)"
                           % (sim, ", ".join(sorted(ADAPTERS))))
    try:
        mod = importlib.import_module(path)
    except ImportError as exc:
        raise AdapterError("adapter for %r (%s) is not implemented yet: %r"
                           % (sim, path, exc))
    if not hasattr(mod, "build_bundle"):
        raise AdapterError("%s does not implement build_bundle()" % path)
    return mod


__all__ = ["ADAPTERS", "REQUIRED_EVIDENCE", "ROBOT_IDENTITIES",
           "AdapterError", "check_bundle", "resolve"]
