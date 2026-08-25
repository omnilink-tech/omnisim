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

"""B2 ``subject_in_frame`` -- the task entry point (inspect tier).

There is no artifact to run. The deliverables are the **final camera pose** and
the agent's **answer**, and :mod:`agentbench.graders.b2_core` compares both
against geometry it derives itself from the subject's world-space bounds.

This file resolves two channels and does no measuring of its own:

  * the adapter's :class:`~agentbench.graders.evidence.EvidenceBundle`, for the
    frozen t=0 inventory the subject and distractor bounds come from;
  * a :class:`~agentbench.graders.b2_core.ViewEvidence`, for the camera.

**The camera channel lives in the neutral bundle** (``graders/evidence.py``
carries ``CameraPose`` / ``ViewEvidence`` and ``EvidenceBundle.view`` since
integration). This module resolves the camera in the following order and
stashes the result in ``bundle.view``; every step of it is simulator-neutral --
nothing here opens a scene file or knows what a viewpoint node is called:

  1. an explicit ``view=`` argument (what the scripted fixtures and a runner
     that already read the camera pass);
  2. ``adapter.build_view_evidence(...)`` when the adapter defines it -- **this
     is the integration point**, and it is where a per-simulator viewpoint
     reader belongs;
  3. ``bundle.adapter_measurements["view"]``, for an adapter that would rather
     fill the existing recorded-payload slot than grow a new entry point;
  4. nothing -- in which case the core returns ``INVALID``. A missing
     instrument is never a PASS and never a FAIL (SPEC 3.3).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import b2_core  # noqa: E402
from agentbench.graders.b2_core import (  # noqa: E402,F401  (re-exported)
    ANGLE_TOL_DEG, DISTANCE_TOL_M, DISTRACTOR_NAMES, MARGIN_DEG,
    MIN_AIM_CHANGE_DEG, MIN_ANGULAR_SIZE_DEG, MIN_MOVE_M, SUBJECT_NAME, TASK,
    TIE_DEG, CameraPose, ViewEvidence, check_view, measure_ground_truth)

ROBOT_IDENTITY = "any_robot"

#: The adapter entry point this grader will use for the camera when it exists.
#: Named here so a new adapter can grep for one string and know what to write.
VIEW_HOOK = "build_view_evidence"
#: ...and the recorded-payload slot it may fill instead.
VIEW_SLOT = "view"


def resolve_view(adapter, bundle, *, view=None, **kw):
    """The camera channel for this run. See the module docstring for the order.

    Never raises: an adapter that blows up while reading a camera is a broken
    measurement, and the reason is carried in ``ViewEvidence.error`` so the
    core can report it rather than crash the campaign.
    """
    if view is not None:
        return view
    hook = getattr(adapter, VIEW_HOOK, None)
    if callable(hook):
        try:
            built = hook(TASK, bundle=bundle, **kw)
        except Exception as exc:                      # noqa: BLE001
            return b2_core.ViewEvidence(
                source=getattr(adapter, "__name__", "adapter"),
                error="the adapter failed while reading the camera: %r" % exc)
        if built is not None:
            return built
    slotted = bundle.measurement_slot(VIEW_SLOT)
    if isinstance(slotted, b2_core.ViewEvidence):
        return slotted
    return b2_core.ViewEvidence(
        source=getattr(adapter, "__name__", "adapter"),
        error="this adapter exposes no camera pose: it defines neither %s() "
              "nor an adapter_measurements[%r] entry" % (VIEW_HOOK, VIEW_SLOT))


def grade(run_dir, *, answer="", phase_b=None, self_verified=False,
          artifact=None, scratch_dir=None, sim=None, view=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    resolved = resolve_view(adapter, bundle, view=view, artifact=artifact,
                            phase_b=phase_b, scratch_dir=scratch_dir,
                            run_dir=run_dir, **kw)
    # The camera channel is part of the bundle now (evidence.py grew the
    # optional ``view`` field at integration): stash it so check_bundle and
    # provenance() see the same evidence the core grades.
    bundle.view = resolved
    return b2_core.grade(bundle, view=resolved, answer=answer,
                         self_verified=self_verified)
