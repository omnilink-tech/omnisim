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

"""Tripwire: a runtime-DELETED node is never removed from the MuJoCo model.

**This test asserts that a known defect is still present.** That is deliberate,
and it is the cheapest way to stop this one being invisible again.

THE DEFECT (measured 2026-08-08, `187a9baab`, write-up in
`docs/developer/ode-retirement-campaign.md` §4.45). `wb_supervisor_node_remove_node()`
deletes a Solid from the scene graph, but there is **no remove/unregister path
from a deleted Solid to the Newton/MuJoCo model at all** — no `removeBody`
anywhere in the backend. The geometry stays in the model, so:

  * a deleted wall still stops a robot,
  * a deleted floor still holds a body up,
  * `mj_ray` still hits deleted geometry, so a removed occluder still occludes.

Measured on `tests/physics/worlds/gravity_rest_height.omniworld`, whose FLOOR is
elevated (top z=0.50) precisely so the implicit z=0 ground plane cannot stand in
for it: the FALLER settled at **z = 0.5999**, `POST /scene/delete {"def":"FLOOR"}`
reported `all_removed: true` and `/scene/tree` no longer listed it, and the box
then stayed at **0.5999 for 61,440 steps** (~246 s of sim, ~770x the ~320 ms the
0.5 m fall would take). The same box had fallen and settled normally moments
earlier in the same process, which is the within-run control.

WHY A TRIPWIRE AND NOT A FIX. The fix is a real feature — a remove path through
the backend plus a model rebuild or body disable — and it is not attempted here.
What this file buys is that the defect stops being *structurally invisible*:
`lane1/translation_audit.py` reads the model at world-finalize and so cannot see
runtime drift at all, and no trajectory metric notices a floor that should not be
there.

WHEN SOMEONE IMPLEMENTS IT, THIS TEST FAILS ON PURPOSE and tells them what to do:
delete the tripwire and run the behavioural repro below as a real assertion.

    # the behavioural repro, for whoever fixes it (needs a built binary):
    python scripts/harness/omnisim_harness.py --auto-port
    POST /world/load  {"path": "tests/physics/worlds/gravity_rest_height.omniworld"}
    POST /sim/step    {"steps": 400}          # FALLER settles at ~0.5999
    POST /scene/delete {"def": "FLOOR"}
    POST /sim/step    {"steps": 400}          # it must now fall; today it does not
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
BACKEND_HPP = REPO / "src" / "omnisim" / "physics" / "OmNewtonBackend.hpp"
BACKEND_CPP = REPO / "src" / "omnisim" / "physics" / "OmNewtonBackend.cpp"

# A remove path would surface as one of these on the backend's own surface.
# Deliberately narrow: `removeBody` is the name the campaign write-up greps for,
# and the others are the obvious spellings of the same capability.
_REMOVE_PATH = re.compile(
    r"\b(removeBody|unregisterBody|deleteBody|removeSolid|removeShape)\b")


def _surface():
    return "\n".join(p.read_text(encoding="utf-8", errors="replace")
                     for p in (BACKEND_HPP, BACKEND_CPP) if p.exists())


def test_no_remove_path_exists_yet():
    """The tripwire. Asserts the defect is STILL PRESENT.

    If this fails, that is good news and an instruction, not a regression: a
    remove path now exists on the backend. Delete this test and replace it with
    the behavioural assertion in the module docstring, so the tree pins the
    FIXED behaviour instead of the broken one.
    """
    src = _surface()
    assert src, "backend sources not found — this tripwire is not evaluating"
    hits = sorted(set(_REMOVE_PATH.findall(src)))
    assert not hits, (
        "A body-removal path has appeared on the Newton backend (%s).\n"
        "That is the fix for the runtime-deletion defect this file pins.\n"
        "DO THIS: delete tests/test_newton_runtime_deletion.py and add the\n"
        "behavioural assertion from its docstring (delete FLOOR, step, require\n"
        "the box to FALL). Also re-enable the 'objects visible after the\n"
        "occluders are removed' assertion in tests/api/worlds/camera_recognition,\n"
        "and drop the 'blind to runtime drift' caveat from\n"
        "docs/benchmarks/correctness-scope.md and the translation audit's SPEC\n"
        "section." % ", ".join(hits))


def test_the_defect_is_documented_where_someone_will_look():
    """A defect that is only in a test is a defect nobody reads about.

    The campaign write-up carries the measurement and the recipe; the correctness
    scope carries the consequence for the audit. Both must keep saying so while
    the defect is live, or the next reader will assume the audit covers it.
    """
    campaign = (REPO / "docs" / "developer" / "ode-retirement-campaign.md")
    scope = (REPO / "docs" / "benchmarks" / "correctness-scope.md")
    if campaign.exists():
        t = campaign.read_text(encoding="utf-8", errors="replace")
        assert "removeBody" in t, \
            "the campaign write-up must keep naming the missing remove path"
    assert scope.exists()
    s = scope.read_text(encoding="utf-8", errors="replace")
    assert "runtime" in s.lower() and "finalize" in s.lower(), (
        "correctness-scope.md must keep stating that the translation audit reads "
        "the model at finalize and is therefore blind to runtime drift — "
        "otherwise a reader will assume the audit covers deletion")
