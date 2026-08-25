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

"""Targeted negative fixtures for the A1 assertions that had none.

The red-evidence rule (docs/developer/agent-edge-validation-plan.md 5.5):

    No assertion may enter a scored campaign until it has been observed
    FAILING on a deliberately wrong artifact, with that negative fixture
    named in the assertion's record. A ``null`` red validates nothing.

Before this module, six A1 assertions had no targeted negative -- A1.1, A1.2,
A1.3, A1.7, A1.9, A1.10 -- including A1.3, which was provably vacuous for
weeks (the (id,id) contact-pairing bug, fixed in ``03e988c5``) and whose
REPAIRED pairing code had never been observed catching the thing it exists to
catch. Each fixture here violates its target assertion as surgically as the
assertion's own structure allows; where collateral reds are structurally
unavoidable, the entry's comment in the ``REGISTRY`` says which and why, and
the committed verdict in ``phase0_validation/`` is the measurement.

Fixture map (target first, expected collateral in brackets):

``nine``      A1.1's **count** clause: nine Huskies instead of ten.
              [A1.2/A1.4/A1.5/A1.6/A1.7/A1.8 also gate on n == 10, so a
              count violation cascades by construction -- there is no world
              with nine robots that reds only A1.1.]
``impostor``  A1.1's **identity-corroboration** clause, surgically: ten
              robot-class bodies, but one is a renamed copy of the URDF, so
              only nine declarations match ``husky.urdf``. Everything else
              about the scene is correct.
``dupname``   A1.2: ten Huskies, two sharing the name ``husky_0``. [OmniSim
              keys a controller's IPC channel by the robot's name
              (``WbController``: ``tmp/ipc/<encodedName>``), so the twin's
              controller may not pair -- motion collateral is possible and is
              recorded from the run, not assumed.]
``overlap``   A1.3: two Huskies deliberately interpenetrating at t=0 --
              centres 0.25 m apart, AABBs overlapping on all three axes.
              This is the fixture that must drive the REPAIRED contact
              pairing red end-to-end: the recorder's per-step contact scan
              has to name the two distinct robots touching.
``meteor``    A1.7: one Husky spawned 5.6 m up, timed so it is still falling
              when the recorded window opens (settle is 1.0 s; the fall takes
              ~1.05 s). |dz| over the window is ~0.6 m >> 0.30 m. [If the
              10.3 m/s landing flips it, motion collateral follows; measured,
              not assumed.]
``errlog``    A1.9: one robot's controller (``err_beacon``, shipped in the
              world's own project) provokes exactly one non-fatal ERROR-class
              engine-log line -- a supervisor import of a PROTO not declared
              IMPORTABLE EXTERNPROTO, which ``WbNodeOperations::importNode``
              refuses via ``WbLog::error`` with **no scene mutation** -- then
              hands the robot to the standard ``husky_random`` walk. The run
              is otherwise byte-for-byte an oracle run: it completes, exits 0
              and reaches finalize; only the error-line clause goes red.

**A1.10 has no end-to-end fixture, structurally.** On a Newton-bundled engine
the ``<log>.newton.json`` sidecar is written at world finalize, so every run
that gets far enough to be graded arrives attributed; the only ways to force
ODE are the explicit world pin or the launch-env knobs -- and the pin IS an
attribution while the launcher strips the env knobs from the child on purpose
(``engine_launch.build_env``). An unattributed run therefore requires an
*engine build* without the Newton runtime and no pin (e.g. ``BUNDLE_NEWTON=0``)
-- a property of the machine, not expressible in a scripted artifact. The
red evidence that exists is core-level:
``graders/test_neutral_core.py::test_a1_core_missing_attribution_is_invalid``
drives the real ``a1_core.grade`` red on a bundle with the attribution
stripped, and :func:`core_unattributed_verdict` below regenerates that verdict
for the committed evidence set. ``COVERAGE_EXTRA`` declares all of this to the
coverage table, which flags the assertion UNVALIDATED end-to-end.
"""

from __future__ import annotations

import math
import random
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents import oracle_a1  # noqa: E402
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common.paths import HUSKY_URDF, as_wbt_path  # noqa: E402

TASK_ID = "A1_husky_swarm_10"

# Same geometry the oracle uses; the ROBOT template is re-authored (not
# imported) because these fixtures need two knobs the oracle's has no slot
# for: the spawn z (meteor) and the robot name (dupname).
_ROBOT = """
URDFRobot {
  url "%(url)s"
  translation %(x).4f %(y).4f %(z).3f
  rotation 0 0 1 %(yaw).5f
  name "%(name)s"
  controller "%(controller)s"
  supervisor TRUE
}
"""


def _emit_ring_world(path, n=10, url=None, seed=oracle_a1.SEED, jitter=0.6,
                     mutate=None):
    """The oracle's ring world, with a per-robot ``mutate(i, spec)`` hook.

    Deliberately reproduces ``oracle_a1.build_world``'s placement math (ring
    radius, outward heading, seeded jitter) so each fixture differs from the
    known-PASS world in exactly the one property under test.
    """
    url = url or as_wbt_path(HUSKY_URDF)
    rng = random.Random(seed)
    parts = [oracle_a1.HEADER % {"arena": oracle_a1.ARENA_M}]
    for i in range(n):
        a = 2.0 * math.pi * i / n
        spec = {"url": url,
                "x": oracle_a1.RING_RADIUS_M * math.cos(a),
                "y": oracle_a1.RING_RADIUS_M * math.sin(a),
                "z": 0.2,
                "yaw": a + rng.uniform(-jitter, jitter),
                "name": "husky_%d" % i,
                "controller": "husky_random"}
        if mutate is not None:
            mutate(i, spec)
        parts.append(_ROBOT % spec)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(parts), encoding="utf-8")
    return path


def _result(ctx, world, msg):
    res = AgentResult()
    res.artifacts["world"] = str(world)
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


# --- A1.1 (count clause): nine robots instead of ten ------------------------

def run_a1_nine(ctx):
    """Nine correct, moving Huskies. One short of the brief."""
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Nine Huskies on the ring -- one short of the ten asked "
                   "for; everything else as the oracle builds it.")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 9,
                                  "controller": "husky_random"})
    _emit_ring_world(world, n=9)
    return _result(ctx, world,
                   "Built %s with nine Huskies, all driving." % world.name)


# --- A1.1 (identity clause): ten robots, one not the robot the task named ---

def run_a1_impostor(ctx):
    """Ten moving robot-class bodies; only nine declare ``husky.urdf``.

    The tenth is a byte-identical copy of the Husky URDF under another
    basename (meshes copied alongside so ``package://husky_description/...``
    still resolves by the importer's walk-up rule). Structurally a Husky, but
    not *declared* as one -- the minimal violator of A1.1's corroboration
    clause, leaving its scene-count clause and every other assertion intact.
    """
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    pkg = ctx.scratch_dir / "husky_description"
    src = HUSKY_URDF.parent.parent          # .../husky_description
    ctx.trace.turn("Ten robots, but the tenth is a renamed copy of the "
                   "Husky URDF -- an impostor the artifact text cannot "
                   "corroborate as a Husky.")
    (pkg / "urdf").mkdir(parents=True, exist_ok=True)
    if not (pkg / "meshes").exists():
        shutil.copytree(src / "meshes", pkg / "meshes")
    impostor = pkg / "urdf" / "impostor_husky.urdf"
    impostor.write_text(HUSKY_URDF.read_text(encoding="utf-8"),
                        encoding="utf-8")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "impostor_url": str(impostor)})

    def mutate(i, spec):
        if i == 9:
            spec["url"] = as_wbt_path(impostor)

    _emit_ring_world(world, mutate=mutate)
    return _result(ctx, world,
                   "Built %s: ten driving robots, nine declared as "
                   "husky.urdf and one impostor." % world.name)


# --- A1.2: ten robots, two sharing a name -----------------------------------

def run_a1_dupname(ctx):
    """Ten Huskies, but ``husky_1`` is also named ``husky_0``."""
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Ten Huskies, two of them both named husky_0.")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "duplicate_name": "husky_0"})

    def mutate(i, spec):
        if i == 1:
            spec["name"] = "husky_0"

    _emit_ring_world(world, mutate=mutate)
    return _result(ctx, world,
                   "Built %s: ten Huskys, names husky_0 x2, husky_2..husky_9."
                   % world.name)


# --- A1.3: two robots deliberately interpenetrating at t=0 ------------------

# 0.25 m between chassis centres. A Husky's AABB is ~0.99 x 0.67 x 0.39 m, so
# the two boxes overlap on ALL THREE axes by design (~0.74 m of x overlap) and
# the chassis volumes genuinely intersect -- both halves of A1.3 (AABB
# separation AND the first-10-steps contact watch) must go red, and the
# contact half only goes red if the repaired (id,id) pairing can name the two
# DISTINCT robots touching.
_OVERLAP_X = oracle_a1.RING_RADIUS_M + 0.25


def run_a1_overlap(ctx):
    """husky_1 spawned inside husky_0's volume at t=0."""
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Ten Huskies, but husky_1 is placed 0.25 m from husky_0 "
                   "-- their chassis volumes interpenetrate at t=0.")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "husky_1_at": [_OVERLAP_X, 0.0, 0.2]})

    def mutate(i, spec):
        if i == 1:
            spec["x"], spec["y"] = _OVERLAP_X, 0.0

    _emit_ring_world(world, mutate=mutate)
    return _result(ctx, world,
                   "Built %s: ten Huskies, husky_0 and husky_1 "
                   "interpenetrating at spawn." % world.name)


# --- A1.7: one robot still falling when the recorded window opens -----------

# 5.6 m up: the fall to the floor takes sqrt(2*5.45/9.81) ~ 1.05 s, the
# recorder's settle is 1.0 s, so the window's first sample catches the robot
# at z ~ 0.77 m and it lands ~0.05 s later: |dz| ~ 0.6 m > 0.30 m whatever
# happens afterwards. Lower spawns land inside the settle window and hide.
_METEOR_Z = 5.6


def run_a1_meteor(ctx):
    """husky_0 spawned mid-air, landing just after recording starts."""
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Ten Huskies, but husky_0 starts %.1f m in the air and is "
                   "still falling when the measurement window opens."
                   % _METEOR_Z)
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "husky_0_z": _METEOR_Z})

    def mutate(i, spec):
        if i == 0:
            spec["z"] = _METEOR_Z

    _emit_ring_world(world, mutate=mutate)
    return _result(ctx, world,
                   "Built %s: ten Huskies, one dropped from %.1f m."
                   % (world.name, _METEOR_Z))


# --- A1.9: a clean-looking run with one ERROR-class engine-log line ---------

# The beacon controller lives in the WORLD'S OWN project (scratch/controllers/
# next to scratch/worlds/), which is legitimate: the tamper check refuses only
# grader-owned controller names. It provokes the error through a supervisor
# import the engine REFUSES -- WbNodeOperations::importNode logs
#   ERROR: In order to import the PROTO 'RectangleArena', first it must be
#          declared in the IMPORTABLE EXTERNPROTO list.
# and changes nothing -- then becomes a standard husky_random walker by
# pointing husky_random.Robot at the already-constructed Supervisor.
_ERR_BEACON = '''\
"""AgentBench negative fixture controller (A1.9 `errlog`).

Provokes exactly one non-fatal ERROR-class engine-log line: a supervisor
import of a PROTO that is declared EXTERNPROTO but not IMPORTABLE, which
WbNodeOperations::importNode refuses via WbLog::error WITHOUT touching the
scene. Then drives its robot exactly like every other one, by handing the
already-constructed Supervisor to the shipped husky_random controller.
"""
import os
import sys

from controller import Supervisor

sv = Supervisor()
ts = int(sv.getBasicTimeStep())
sv.step(ts)
try:
    sv.getRoot().getField("children").importMFNodeFromString(
        -1, "RectangleArena { }")
except Exception:
    pass
sv.step(ts)

sys.path.insert(0, os.path.join(
    os.environ.get("OMNISIM_HOME", ""),
    "projects", "default", "controllers", "husky_random"))
import husky_random  # noqa: E402

husky_random.Robot = lambda: sv
husky_random.main()
'''


def run_a1_errlog(ctx):
    """An otherwise-oracle run whose engine log carries one ERROR: line."""
    # worlds/ subdir makes scratch itself the engine's project root, so the
    # beacon controller ships inside the agent's own sandbox.
    world = ctx.scratch_dir / "worlds" / "husky_swarm_10.wbt"
    beacon = ctx.scratch_dir / "controllers" / "err_beacon" / "err_beacon.py"
    ctx.trace.turn("Ten Huskies as the oracle builds them, but husky_0's "
                   "controller first provokes one refused supervisor import "
                   "-- a single non-fatal ERROR-class log line.")
    beacon.parent.mkdir(parents=True, exist_ok=True)
    beacon.write_text(_ERR_BEACON, encoding="utf-8")
    ctx.trace.tool("write_file", {"path": str(beacon),
                                  "purpose": "one refused supervisor import "
                                             "-> one ERROR: line"})
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "husky_0_controller": "err_beacon"})

    def mutate(i, spec):
        if i == 0:
            spec["controller"] = "err_beacon"

    _emit_ring_world(world, mutate=mutate)
    return _result(ctx, world,
                   "Built %s: ten driving Huskies plus one deliberate "
                   "ERROR-class log line." % world.name)


# --- A1.10: core-level red (see the module docstring for why not end-to-end)

def core_unattributed_verdict():
    """The real ``a1_core.grade`` run on a bundle with attribution stripped.

    Reuses ``test_neutral_core.a1_bundle`` -- the same synthetic
    otherwise-passing bundle the core test suite grades -- so there is one
    definition of "a clean run minus its attribution" in the tree. The
    returned Verdict is grader output, labelled synthetic in its notes.
    """
    from agentbench.graders import test_neutral_core
    v = test_neutral_core.a1_bundle(attribution=None)
    from agentbench.graders import a1_core
    verdict = a1_core.grade(v)
    verdict.note("core-level evidence: synthetic EvidenceBundle "
                 "(graders/test_neutral_core.a1_bundle, attribution=None); "
                 "no engine was run. See COVERAGE.md, A1.10.")
    return verdict


# --- exports ----------------------------------------------------------------

_FIXTURE_FNS = {
    "nine": run_a1_nine,
    "impostor": run_a1_impostor,
    "dupname": run_a1_dupname,
    "overlap": run_a1_overlap,
    "meteor": run_a1_meteor,
    "errlog": run_a1_errlog,
}

# Predicted red sets, declared BEFORE the runs (the Phase-0 contract). Where a
# committed verdict in phase0_validation/ measured a different set, the
# REGISTRY in agents/__init__.py carries the measured set with a comment --
# these are the pre-registrations, kept for the diff.
PREDICTED_FAILURES = {
    "nine": {"A1.1", "A1.2", "A1.4", "A1.5", "A1.6", "A1.7", "A1.8"},
    "impostor": {"A1.1"},
    "dupname": {"A1.2"},
    "overlap": {"A1.3"},
    "meteor": {"A1.7"},
    "errlog": {"A1.9"},
}

# REGISTRY-shaped, merged into agents/__init__.REGISTRY (same convention as
# the b1/b2/c1 fixture modules). expect_failures starts as the prediction;
# agents/__init__.py overrides with the MEASURED set where they differ.
REGISTRY = {
    (TASK_ID, name): {"fn": fn, "expect_pass": False,
                      "expect_failures": set(PREDICTED_FAILURES[name])}
    for name, fn in _FIXTURE_FNS.items()
}

# Evidence the coverage table cannot learn from REGISTRY + verdict files:
# per-(task, assertion) records for assertions whose red evidence is not an
# end-to-end scripted-artifact run. See coverage_table.py.
COVERAGE_EXTRA = {
    (TASK_ID, "A1.10"): {
        "kind": "core-unit",
        "end_to_end": False,
        "fixture": ("graders/test_neutral_core.py::"
                    "test_a1_core_missing_attribution_is_invalid "
                    "(+ a1_fixtures_extra.core_unattributed_verdict)"),
        "reason": (
            "structural: on a Newton-bundled engine the <log>.newton.json "
            "sidecar is written at world finalize, so every gradeable run "
            "arrives attributed; forcing ODE requires the explicit world pin "
            "(which IS an attribution) or launch-env knobs the launcher "
            "strips from the child on purpose. An unattributed run needs an "
            "engine BUILD without the Newton runtime and no pin "
            "(BUNDLE_NEWTON=0) -- a machine property, not expressible in a "
            "scripted artifact. A live-run fixture would have to provision "
            "that binary and run any A1 world against it unpinned."),
    },
}
