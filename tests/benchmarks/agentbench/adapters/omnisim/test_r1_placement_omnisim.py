#!/usr/bin/env python3
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

"""Grade-time placement on the OmniSim arm -- and what it does NOT establish.

⚠ **WHAT THIS FILE ASSERTS IS STILL PLACEMENT-UP-TO-THE-SCENE, BUT THE REASON
IS NO LONGER THAT THE ARM CANNOT DRIVE.** This docstring used to say a motor
target set after Newton world-finalize had no effect, so no closed-loop
navigator could steer here at all. **That was true and is now false on both
counts** (freeze v3 Amendments 3 and 4): the engine defect was fixed in
``3cf70f120``, this arm's rover was repaired from ``maxTorque 0.4`` to ``12``,
and the oracle now PASSes 6/6 on the published layout
(``test_r1_discriminates_omnisim.py::test_the_oracle_half_of_the_gate_is_green``).

**The end-to-end half has since been MEASURED on this arm, though it is not yet
asserted by this file.** Driving the repaired oracle against five grade-time
placed layouts it had never seen (seeds ``omnisim/gen/1``…``5``, one 60 s run
each, 2026-08-11) scored **4 PASS 6/6 and 1 FAIL** -- the fifth finishing
7.8065 m short of the goal with one ``rover/OBSTACLE_1`` contact, so it failed
R1.4 and R1.5. That is a driving robot's verdict responding to the drawn
layout, which is the property the mechanism exists to produce. Wiring one such
run in here (≈60 s) would give this file its end-to-end half; until someone
does, the cheaper, larger-sample evidence remains the MuJoCo arm's
(``adapters/mujoco/test_r1_placement_end_to_end.py``: oracle 40/40, memoriser
caught 36/40).

What CAN be established here is everything up to the driving, and it is
established against the real engine rather than against our own scanner:

    1. the world text is rewritten -- and the ENGINE loads it, so the rewrite
       produced a valid ``.wbt`` and not merely a file our text helpers agree
       with;
    2. the engine's own t=0 scene scan measures the obstacles AT THE DRAWN
       POSES -- so what was written is what the simulator sees;
    3. ``graders/r1`` resolves the sidecar by itself and scores R1.3 against
       the drawn layout, which now FAILS against the published one.

That is the whole placement contract minus the robot. What is unverified **by
this file** on this arm: that a placed layout changes a DRIVING robot's
verdict. That is no longer unverifiable here -- it was measured on 2026-08-11
(above) -- it is simply not yet one of the assertions below.

**Cost.** One real headless ``omnisim-bin`` run, ~15-30 s. Skips, rather than
fails, when the engine binary is absent.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.omnisim import headless  # noqa: E402
from agentbench.common import r1_placement, worldtext  # noqa: E402
from agentbench.common.paths import REPO, engine_launch  # noqa: E402
from agentbench.graders import r1 as r1_grader  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

LANE = Path(__file__).resolve().parent / "omnisim_lane"
SOLIDS = ("OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3", "OBSTACLE_4",
          "OBSTACLE_5")
SEED = "omnisim/placement/1"

#: Shorter than the task's own 60 s window on purpose: nothing here depends on
#: the robot moving (it cannot), only on the world loading and the t=0 scan
#: reporting the obstacles.
DURATION_S = 10.0

pytestmark = pytest.mark.skipif(
    engine_launch.resolve_binary(REPO) is None,
    reason="the OmniSim arm needs a built omnisim-bin")


def _stage(dest):
    """Lay the lane's null world out as a COLLECTED DELIVERABLE would be."""
    dest = Path(dest)
    (dest / "worlds").mkdir(parents=True, exist_ok=True)
    text = (LANE / "worlds" / "r1_null.wbt").read_text(encoding="utf-8")
    world = dest / "worlds" / "lidar_nav.wbt"
    world.write_text(text, encoding="utf-8")
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("controller ") and '"' in s:
            src = LANE / "controllers" / s.split('"')[1]
            if src.is_dir():
                shutil.copytree(src, dest / "controllers" / src.name,
                                dirs_exist_ok=True)
    return world


@pytest.fixture(scope="module")
def placed(tmp_path_factory):
    """One placed world, run once, graded once -- shared by every test."""
    out = tmp_path_factory.mktemp("r1_placed_omnisim")
    world = _stage(out / "project")
    report = r1_placement.place_and_declare(world, seed=SEED,
                                            declare_dirs=(out,))
    phase_b = headless.run_standalone(
        world, out / "phaseB", duration=DURATION_S, settle=0.0,
        contact_steps=0, solids=SOLIDS, timeout_s=900.0, tag="phaseB")
    verdict = r1_grader.grade(out, artifact=world, phase_b=phase_b,
                              sim="omnisim")
    return report, world, phase_b, verdict, out


def _measured(verdict, aid):
    return [a for a in verdict.assertions if a.id == aid][0].measured


def _find(measured, *needles):
    for k, v in measured.items():
        if all(n in k for n in needles):
            return v
    raise AssertionError("no measurement matching %s in %s"
                         % (needles, list(measured)))


def test_the_placed_world_still_loads_in_the_engine(placed):
    """The rewrite produced a real ``.wbt``, not just a file we can read.

    A text edit that broke the world would show up as a load error here
    rather than as an agent whose scene failed to build -- which is why this
    is asserted before anything geometric.
    """
    _report, _world, phase_b, verdict, _out = placed
    assert phase_b.xyz is not None, "the recorder produced no samples"
    assert _find(_measured(verdict, "R1.1"), "ERROR") == 0, verdict.summary()
    assert _find(_measured(verdict, "R1.1"), "exit code") == 0
    ok = {a.id: a.ok for a in verdict.assertions}
    assert ok["R1.1"] is True, verdict.summary()


def test_the_engine_measures_the_obstacles_at_the_DRAWN_poses(placed):
    """⭐ What was written is what the simulator sees.

    The recorder's name-free t=0 scan bounds every non-robot body from the
    LOADED scene -- not from our text scan -- and R1.3 matches those bounds
    against the layout the grader resolved. Five matches means the engine
    found all five obstacles where placement put them.
    """
    _report, _world, _pb, verdict, _out = placed
    m = _measured(verdict, "R1.3")
    assert _find(m, "obstacles found") == r1_core.N_OBSTACLES, m
    assert _find(m, "obstacles missing") == [], m
    assert _find(m, "blocking"), (
        "the drawn layout must still block the straight line in the loaded "
        "scene, or the sensing argument does not hold on this world: %s" % m)
    assert {a.id: a.ok for a in verdict.assertions}["R1.3"] is True


def test_the_grader_resolved_the_sidecar_by_itself(placed):
    """No layout was passed to ``grade``. It found the declaration."""
    _report, _world, _pb, verdict, _out = placed
    scored = verdict.measurements["r1_graded_layout"]
    assert scored["placed"] is True
    assert r1_core.LAYOUT_SIDECAR in scored["source"]
    drawn = r1_core.sample_layout(SEED)
    assert [o["position"][:2] for o in scored["obstacles"]] == \
        [[round(float(c), 4) for c in o["position"][:2]] for o in drawn]
    assert not any("not evidence of perception" in str(n)
                   for n in verdict.notes), verdict.notes


def test_the_same_scene_no_longer_matches_the_PUBLISHED_layout(placed):
    """The other half, and the one that makes the first half mean something.

    Measured from the engine's own t=0 bodies with the grader's own matcher:
    against the published layout, nothing matches. A memorising controller
    planning from ``benchmark_assets/obstacles.json`` would be planning around
    boxes that are not there.
    """
    _report, _world, phase_b, _v, _out = placed
    from agentbench.adapters.omnisim import evidence

    bundle = evidence.build_bundle(
        "R1_lidar_nav", robot_identity="any_robot", phase_b=phase_b,
        scene_inventory=True)
    bodies = [b for b in bundle.roster.bodies if not b.robot_class]
    drawn_found, drawn_missing = r1_core.match_spec_obstacles(
        bodies, r1_core.sample_layout(SEED))
    pub_found, pub_missing = r1_core.match_spec_obstacles(
        bodies, r1_core.obstacle_spec())
    assert len(drawn_found) == r1_core.N_OBSTACLES and drawn_missing == []
    assert len(pub_found) == 0 and len(pub_missing) == r1_core.N_OBSTACLES, (
        "the loaded scene still matches the published layout: %s"
        % [b.name for b in pub_found])


def test_the_world_text_itself_was_minimally_rewritten(placed):
    """Five ``translation`` fields, nothing else -- the graded world is still
    the world that was delivered."""
    _report, world, _pb, _v, _out = placed
    original = (LANE / "worlds" / "r1_null.wbt").read_text(encoding="utf-8")
    now = world.read_text(encoding="utf-8")
    a, b = original.splitlines(), now.splitlines()
    assert len(a) == len(b)
    differing = [x for x, y in zip(a, b) if x != y]
    assert len(differing) == r1_core.N_OBSTACLES, differing
    assert all("translation" in x for x in differing), differing
    # ...and the bodies the placer moved are the ones it says it moved.
    bodies = {x.name: x for x in worldtext.scan_bodies(now)}
    for o in r1_core.sample_layout(SEED):
        centre = bodies[o["name"]].centre
        assert round(centre[0], 4) == round(float(o["position"][0]), 4)
        assert round(centre[1], 4) == round(float(o["position"][1]), 4)


def test_what_this_arm_does_NOT_establish_is_recorded_here():
    """The gate is per (task, arm), and saying so in the suite -- not only in
    a report -- is what keeps a green file from reading as a green gate.

    Placement is verified BY THIS FILE up to the loaded scene. That a placed
    layout changes a DRIVING robot's verdict is measured (see the module
    docstring: 4 PASS / 1 FAIL over five unseen layouts, 2026-08-11) but is not
    asserted here, so this file's green must not be read as the end-to-end
    green. The remedy is to wire one placed-layout oracle run in, not to
    delete this test.
    """
    assert (LANE / "controllers" / "r1_oracle" / "r1_oracle.py").exists(), (
        "the oracle is what closes this gap; it is now validated on the "
        "published layout by test_r1_discriminates_omnisim.py")
    assert not (LANE / "worlds" / "r1_oracle_verified.json").exists(), (
        "a committed per-layout oracle verdict has appeared: wire it into "
        "this file as the end-to-end half instead of leaving it on disk")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
