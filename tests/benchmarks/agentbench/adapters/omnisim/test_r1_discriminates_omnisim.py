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

"""R1 must DISCRIMINATE on the OmniSim arm -- SPEC 7.1's gate, run here.

The gate is two halves and it is worth nothing with only one of them: an
ORACLE performing the known-good solution must PASS, and a NULL doing nothing
must FAIL. Until both have been shown for a given arm, that arm's pass rate is
unfalsifiable -- which is not hypothetical. C2 shipped a world whose UNFIXED
form passed 5/5 for an entire campaign because nobody had asserted the task
could fail, and every C2 number ever recorded was uninformative as a result.

✅ **BOTH HALVES ARE NOW GREEN** (2026-08-11, freeze v3 Amendment 4). Every
row below is run by this file against the real engine, on one shared window
(60 s, ``contact_steps 0``):

    r1_null    steps, commands nothing            FAIL (R1.4, R1.5, R1.6)
    r1_blind   drives straight at the goal, no    FAIL (R1.4, R1.5) -- and
               sensing, hits OBSTACLE_2           R1.5 fails because the
                                                  contact was SEEN
    r1_oracle  perceives, plans, follows          PASS 6/6 -- 0.1506 m from
                                                  the goal, 0 contacts,
                                                  14.2671 m of path

⚠ **The oracle is a NAVIGATOR, not a universal solver, and the gate does not
claim otherwise.** R1 draws its layout at GRADE TIME, and over two independent
five-layout generalisation probes against layouts it had never seen this
navigator scored 4 PASS / 1 FAIL both times -- once failing R1.5 only (reached
the goal, grazed a box) and once failing R1.4 *and* R1.5 (did not arrive at
all, 7.8065 m short). SPEC 7.1 asks that the task be shown PASSABLE on this
arm, which is what :func:`test_the_oracle_half_of_the_gate_is_green` asserts;
it is not a claim that every drawn layout is solved. The bound is recorded on
the cell in ``preregister/oracle_verdicts.json``.

**The blocker was TWO stacked defects, and both are now fixed.** Until
2026-08-10 this file said the passing half "cannot be built on this engine
build". That was true, and it was true for two independent reasons:

1. **The engine** (fixed in ``3cf70f120``): the batched-substep path decided
   whether to copy newton's ``control`` into ``mj_data.ctrl`` by re-reading
   the ``joint_targets`` dicts *after* the same tick had already consumed and
   cleared them. The gate was therefore permanently false, and whatever ctrl
   was latched at the batch transition drove the actuators for the rest of the
   run -- so a target set after world-finalize had no effect.
2. **This world's rover** (fixed 2026-08-10): its wheels had never rolled.
   ``maxTorque 0.4`` is far below what four 0.15 kg wheels need to accelerate a
   3 kg rover, so the hinge DOFs barely turned while the chassis SLID across
   the floor -- which reads exactly like an ignored command. Repaired to
   ``maxTorque 12`` (the value proven rolling on
   ``projects/robot_combat/worlds/tests/drive_test.omniworld``), with
   ``basicTimeStep 8`` and ``newtonSubsteps 4``.

Measured on ``r1_drive_probe`` with the FIXED engine either side of the world
repair -- 6.0 rad/s on a 0.08 m wheel is 0.480 m/s of ground speed, and 0.0 is
a stop:

    commanded            6.0 rad/s          0.0 rad/s
    broken rover         1.026 m/s          0.796 m/s BACKWARD
    repaired rover       0.480 m/s          0.000 m/s

⚠ **One motor defect is NOT fixed and controllers must route around it:**
commanding *exactly* ``maxVelocity`` runs away instead of saturating (the
probe's 12.0 rad/s phase, and its uncommanded phase -- an uncommanded velocity
motor sits at ``maxVelocity`` by default -- both accelerate past 9 m/s into the
arena wall). Raising ``maxVelocity`` does not remove it, it moves it: measured
at ``maxVelocity 20`` the uncommanded phase peaked at 10.6 m/s and flipped the
rover, so the field is left at 12 and every driver here commands below it
(``r1_blind`` 8.0 rad/s, ``r1_oracle`` ≤ 10 rad/s, ``r1_null`` 0.0).

**The third defect was the navigator itself** (fixed in ``dc9bf156f``, the
controller only -- the world was not touched, so the before/after is a
controlled comparison). It commanded 15.25 rad/s against ``maxVelocity 12`` and
the rover FLIPPED at t=15.5 s; its add-only occupancy set ended the run holding
1303 cells, ~70 % of them phantom floor returns taken while the chassis was
tilted, so A* found no route from t=7 s; and because a wheel command CHANGE
kicks the chassis to ~10x its rolling speed with the kicks accumulating, both
channels are now rationed (published at 4 Hz under a 0.5 rad/s slew). Before:
FAIL, 5.7968 m from the goal, 2 contacts, 23.5842 m of path. After: PASS 6/6.

**What the blind driver is FOR, and what it fixed.** A null hits nothing, so it
leaves R1.5 -- "nothing was hit" -- completely untested, and an untested
collision assertion is how a benchmark ends up crediting a robot that ploughed
through the scene. On this arm it was doing exactly that: the recorder's only
contact channel reported ROBOT-ROBOT pairs over the first ``--contact-steps``
basic steps at t=0, and R1's task meta asks for ``contact_steps: 0``. So a
robot-vs-obstacle contact was not merely unobserved, it was **not
representable**. Measured on the recorded ``r1_settled_omnisim`` cell: an
agent's rover finished at (9.22, 17.36) -- outside a walled 10 x 10 arena --
and R1.5 reported ``robot-obstacle/wall contacts: 0`` and PASSED.
:func:`test_the_collision_channel_was_unfailable_before_the_run_long_watch`
pins the before/after on ONE run.

**Cost.** These are real headless ``omnisim-bin`` runs -- no GPU beyond the
engine's own, no network, and no model quota. Roughly 40-70 s each, so the
whole file is a couple of minutes. It skips, rather than fails, when the
engine binary is absent.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.omnisim import evidence, headless  # noqa: E402
from agentbench.common.paths import REPO, engine_launch  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

LANE = Path(__file__).resolve().parent / "omnisim_lane"

#: The task's own standalone window and solids list
#: (``tasks/R1_lidar_nav/meta.json``), so a verdict here is the verdict a
#: graded run would produce and not a fixture-shaped approximation.
DURATION_S = 60.0
SOLIDS = ("OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3", "OBSTACLE_4",
          "OBSTACLE_5")

#: R1.5 resolves a contact to the robot's own ``name`` field.
ROBOT = "rover"

pytestmark = pytest.mark.skipif(
    engine_launch.resolve_binary(REPO) is None,
    reason="the OmniSim arm needs a built omnisim-bin")


# --- running one lane world the way a deliverable is run --------------------

def _stage(world_stem, dest):
    """Lay a lane world out as a COLLECTED DELIVERABLE would be.

    ``<project>/worlds/lidar_nav.wbt`` plus ``<project>/controllers/<name>/``
    -- the layout ``cc_lane/run_cc_cell.collect_controllers`` produces and the
    one the engine resolves ``controller "<name>"`` against
    (``project_root_for_world``). Staging it any other way would test a
    filename convention this arm does not use.
    """
    dest = Path(dest)
    (dest / "worlds").mkdir(parents=True, exist_ok=True)
    text = (LANE / "worlds" / ("%s.wbt" % world_stem)).read_text("utf-8")
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


def _run(tmp_dir, world_stem, **kw):
    """``(bundle, verdict, phase_b)`` for one real OmniSim run."""
    out = Path(tmp_dir)
    world = _stage(world_stem, out / "project")
    phase_b = headless.run_standalone(
        world, out / "phaseB", duration=DURATION_S, settle=0.0,
        contact_steps=0, solids=SOLIDS, timeout_s=900.0, tag="phaseB", **kw)
    bundle = evidence.build_bundle(
        "R1_lidar_nav", robot_identity="any_robot", artifact=str(world),
        phase_b=phase_b, run_dir=out / "phaseB", scene_inventory=True)
    return bundle, r1_core.grade(bundle), phase_b


def _ok(verdict):
    return {a.id: a.ok for a in verdict.assertions}


def _measured(verdict, aid):
    return [a for a in verdict.assertions if a.id == aid][0].measured


def _find(measured, *needles):
    """A measured value by substring, so a label edit does not fail a test."""
    for k, v in measured.items():
        if all(n in k for n in needles):
            return v
    raise AssertionError("no measurement matching %s in %s"
                         % (needles, list(measured)))


@pytest.fixture(scope="module")
def null(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_null"), "r1_null")


@pytest.fixture(scope="module")
def blind(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_blind"), "r1_blind")


@pytest.fixture(scope="module")
def oracle(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("r1_oracle"), "r1_oracle")


# --- the scene itself -------------------------------------------------------

def test_the_scene_is_the_task_the_grader_thinks_it_is(null):
    """R1.3, on the world both other tests share.

    The five obstacles must be found BY GEOMETRY -- the grader never matches
    on a name -- and the straight START->GOAL line must be blocked, because
    the whole sensing argument (R1.4 + R1.5 + R1.6) rests on that and is worth
    nothing without it.
    """
    _b, v, _p = null
    m = _measured(v, "R1.3")
    assert _find(m, "obstacles found") == r1_core.N_OBSTACLES, m
    assert _find(m, "blocking"), (
        "the direct path is not blocked in this scene, so nothing here "
        "demonstrates perception: %s" % (m,))
    assert _ok(v)["R1.3"] is True, v.summary()


def test_the_run_is_newton_driven_and_clean(null):
    """A verdict from an unattributed run is not a verdict (SPEC A1.10)."""
    b, v, p = null
    assert p.sidecar.get("present") is True, p.sidecar
    assert b.attribution is not None and b.attribution.backend == "newton"
    assert _ok(v)["R1.1"] is True, v.summary()


# --- the NULL: the failing half of the gate ---------------------------------

def test_doing_nothing_fails(null):
    """SPEC 1.1 / 7.1: no task may be passable by doing nothing."""
    _b, v, _p = null
    assert v.outcome != "PASS", v.summary()


def test_the_null_fails_exactly_the_assertions_it_should(null):
    """...and for the reason claimed, which is the other half of a negative
    control. R1.1-R1.3 SHOULD pass: the run is clean, the robot is drivable
    and the obstacles are intact. Those are true of an idle agent, and a gate
    that demanded they fail would be asking the null to be a broken world
    rather than an agent that did nothing."""
    _b, v, _p = null
    assert _ok(v) == {"R1.1": True, "R1.2": True, "R1.3": True,
                      "R1.4": False, "R1.5": False, "R1.6": False}


def test_the_parked_robot_is_not_credited_with_being_collision_free(null):
    """The free pass R1.5 refuses to hand out: a robot that never moved hits
    nothing. Without ``MIN_MOTION_FOR_CREDIT_M`` the null would collect this
    assertion for doing least."""
    _b, v, _p = null
    m = _measured(v, "R1.5")
    assert _find(m, "contacts") == 0
    assert _find(m, "travelled") < r1_core.MIN_MOTION_FOR_CREDIT_M


# --- the BLIND driver: proof that R1.5 can FAIL -----------------------------

def test_a_driver_that_does_not_sense_fails(blind):
    """It drives -- so it earns the motion R1.5 requires -- and it drives into
    an obstacle, which is the whole point of a blocked direct path."""
    _b, v, _p = blind
    assert v.outcome != "PASS", v.summary()
    ok = _ok(v)
    assert ok["R1.4"] is False and ok["R1.5"] is False


def test_the_collision_is_NAMED_and_it_is_the_blocking_obstacle(blind):
    """Not just "a contact": the pair names the robot and the obstacle.

    A count with no partner cannot be checked against the obstacles the
    grader matched by geometry, and R1.5's whole question is *what* was hit.
    """
    _b, v, p = blind
    m = _measured(v, "R1.5")
    assert _find(m, "contacts") >= 1, m
    hits = _find(m, "first hits")
    assert any(ROBOT in h and "OBSTACLE" in h for h in hits), hits
    assert _find(m, "travelled") >= r1_core.MIN_MOTION_FOR_CREDIT_M, (
        "the blind driver must MOVE, or R1.5 would be failing it for being "
        "parked rather than for hitting something")
    pairs = (p.run_contacts or {}).get("pairs") or []
    named = [(q["a"], q["b"]) for q in pairs
             if q.get("b") and not q.get("b_robot")]
    assert (ROBOT, "OBSTACLE_2") in named, named


def test_the_collision_channel_was_unfailable_before_the_run_long_watch(
        tmp_path):
    """⚠ THE DEFECT THIS FILE EXISTS FOR, measured on ONE run rather than argued.

    Same world, same driver, same grader; the ONLY difference is
    ``--run-contacts=0``, which restores the recorder's pre-2026-08-09
    behaviour (robot-ROBOT pairs, sampled over the first ``--contact-steps``
    basic steps at t=0, and R1 asks for ``contact_steps: 0``).

    With the watch off, a rover physically jammed against OBSTACLE_2 for the
    rest of the run collects "nothing was hit". With it on, the same run
    reports the pair. An assertion that cannot fail is not evidence, and this
    is the measurement that says which one R1.5 was.

    Delete this test only when the old channel is gone, not when it is merely
    unused.
    """
    _b, v, p = _run(tmp_path, "r1_blind", run_contacts=0)
    assert (p.run_contacts or {}).get("supported") is False
    m = _measured(v, "R1.5")
    assert _find(m, "contacts") == 0, (
        "the pre-fix channel has started reporting collisions -- if the "
        "recorder changed, re-measure rather than deleting this")
    assert _find(m, "travelled") >= r1_core.MIN_MOTION_FOR_CREDIT_M
    assert _ok(v)["R1.5"] is True, (
        "this test asserts the FALSE PASS the old channel handed out; if R1.5 "
        "now fails here for another reason, the before/after comparison it "
        "documents is no longer the thing being shown")


def test_the_run_long_watch_leaves_the_t0_robot_robot_channel_alone(blind):
    """The fix must not re-scope A1.3.

    ``a1_core`` reads ``contacts.robot_robot_pairs`` and ``contacts.steps`` as
    the answer to a question about **t=0** ("were these robots spawned
    interpenetrating?"). The run-long watch merges only pairs that are NOT
    robot-robot, and touches none of the counters, so every already-recorded
    A1 measurement is reproducible byte-for-byte.
    """
    b, _v, p = blind
    con = b.contacts
    assert con.supported is True
    assert con.robot_robot_pairs == [], (
        "a run-long robot-robot pair reached the t=0 channel; that would "
        "change A1.3 on a world where two robots touch mid-run")
    assert con.steps == 0            # R1's own --contact-steps
    assert all(pp.a_robot and not pp.b_robot for pp in con.pairs)
    # ...and nothing is hidden: the whole run-long summary, robot-robot pairs
    # included, is published on the row. It is kept OUT of the t=0 channel,
    # not out of the evidence.
    run = b.adapter_measurements.get("contacts_run") or {}
    assert run.get("supported") is True
    assert run.get("steps_sampled", 0) > 0
    assert run.get("every_n_steps", 0) > 0
    assert isinstance(run.get("pairs"), list) and run["pairs"]


# --- the passing half: blocked, and recorded as blocked ---------------------

def _drive_probe():
    """Run the drive probe and return its report. ``None`` if it produced none."""
    import subprocess
    import tempfile
    binpath = engine_launch.resolve_binary(REPO)
    tmp = Path(tempfile.mkdtemp(prefix="r1_drive_probe_"))
    out = tmp / "probe.json"
    env = engine_launch.build_env(None, tmp / "engine.log", repo=REPO,
                                  extra={"AGENTBENCH_R1_PROBE_OUT": str(out)})
    world = LANE / "worlds" / "r1_drive_probe.wbt"
    with open(tmp / "console.log", "w") as fh:
        subprocess.run([str(binpath), str(world), "--batch", "--mode=fast",
                        "--no-rendering", "--minimize"],
                       env=env, cwd=str(REPO), stdout=fh,
                       stderr=subprocess.STDOUT, timeout=300)
    if not out.exists():
        return None
    return json.loads(out.read_text(encoding="utf-8"))


#: The rover's wheel (``Cylinder { radius 0.08 }``), so a commanded wheel
#: speed converts to the ground speed a rolling wheel must produce.
WHEEL_RADIUS_M = 0.08


def _phase_tail(doc, command, n=3):
    """Mean ground speed over the LAST ``n`` samples of one commanded phase.

    The tail, not the phase mean. Every phase opens with the rover still
    carrying the previous phase's momentum, so a whole-phase mean scores the
    transient rather than what the command settles at -- which is the number
    the assertion is about.
    """
    vs = [s[5] for s in doc["samples"]
          if str(s[1]) == str(command) and s[5] is not None]
    assert len(vs) >= n, "phase %s has too few samples: %s" % (command, vs)
    tail = vs[-n:]
    return sum(tail) / float(len(tail))


def test_the_motor_path_tracks_a_commanded_wheel_speed():
    """The rover OBEYS -- a commanded wheel speed becomes that ground speed.

    ⚠ **HISTORY, kept because it is why this file is shaped the way it is.**
    This test used to be the opposite assertion (``responds_to_command is
    False``), written to go GREEN on a broken engine and RED the day it was
    repaired. Measured **2026-08-09** on ``omnisim-bin.exe`` built 13:43 at
    ``f45e0b652``, four commanded wheel speeds -- none / 6.0 / **0.0** / 12.0
    rad/s -- produced **0.997 / 1.026 / 1.034 / 1.041 m/s**: four commands, one
    speed, and commanding zero did not stop the robot. That defect is fixed
    (engine ``3cf70f120``, plus this world's ``maxTorque`` repair -- see the
    module docstring for both), and the pin is retired rather than deleted so
    the measurement survives its own conclusion.

    What is asserted now is the physical relation, not merely "something
    changed": a rolling 0.08 m wheel commanded at 6.0 rad/s moves the rover at
    0.480 m/s, and 0.0 rad/s stops it. The broken rover -- same engine, same
    probe, ``maxTorque 0.4`` -- read 1.026 m/s and 0.796 m/s *backward* for
    those two commands, so both numbers below discriminate.

    The two at-cap phases (uncommanded, and 12.0 == ``maxVelocity``) are
    deliberately NOT asserted: they run away into the arena wall, and that
    unrelated defect is documented in the module docstring rather than pinned
    here, where a future engine fix would read as a regression.
    """
    doc = _drive_probe()
    assert doc is not None, "the drive probe produced no report"
    assert doc["responds_to_command"] is True, (
        "the motor path has stopped responding to commanded velocity -- %s"
        % (doc["mean_speed_by_command"],))

    want = 6.0 * WHEEL_RADIUS_M
    got = _phase_tail(doc, 6.0)
    assert abs(got - want) <= 0.03, (
        "commanded 6.0 rad/s on a %.2f m wheel is %.3f m/s of ground speed; "
        "measured %.3f. A rover whose wheels do not roll SLIDES at roughly "
        "1.0 m/s here, which is what this bound separates."
        % (WHEEL_RADIUS_M, want, got))

    stopped = _phase_tail(doc, 0.0)
    assert abs(stopped) <= 0.02, (
        "commanded 0.0 rad/s and the rover is still moving at %.3f m/s "
        "(negative = backwards, the broken rover's signature)" % (stopped,))


def test_the_oracle_half_of_the_gate_is_green(oracle):
    """SPEC 7.1's PASSING half, run here rather than cited from a report.

    This test used to be the opposite assertion -- it asserted the passing
    half was MISSING, and it was correct to do so for as long as it was true.
    It is asserted the other way now because the oracle was measured PASSing
    6/6, and a suite whose own test says the gate is missing while
    ``oracle_verdicts.json`` records PASS teaches readers to trust neither.

    ⚠ What this establishes is that the task is PASSABLE on this arm, which is
    all SPEC 7.1 asks. It is NOT a claim that this navigator solves every
    layout: R1 draws its layout at grade time, and two independent
    five-layout probes each scored 4 PASS / 1 FAIL. The bound is recorded on
    the cell; do not quote this green as more than the gate.
    """
    _, v, _ = oracle
    ok = _ok(v)
    assert v.outcome == "PASS", v.summary()
    assert all(ok.values()), v.summary()

    m = {a.id: a.measured for a in v.assertions}
    # The goal is REACHED, not merely approached.
    assert _find(m["R1.4"], "distance to goal") <= 0.30, m["R1.4"]
    # And it is reached WITHOUT touching anything, on the same channel that
    # catches the blind driver's OBSTACLE_2 strike -- so the zero is measured,
    # not an unanswered query. R1.5 refuses an unsupported channel outright.
    assert _find(m["R1.5"], "contacts") == 0, m["R1.5"]
    assert _find(m["R1.5"], "count against") >= 1, (
        "zero countable bodies would make 'nothing was hit' unfailable")
    # Having driven far enough that the zero was ever at risk.
    assert _find(m["R1.6"], "path length") >= r1_core.MIN_MOTION_FOR_CREDIT_M


def test_the_gate_matches_what_the_pre_registration_RECORDS(oracle, null):
    """The recorded verdict and the measured one cannot be allowed to drift.

    ``preregister/oracle_verdicts.json`` is what ``readiness.py`` reads for the
    ``discriminating`` column -- SPEC 7.1's gate, not a licence to publish;
    ``tasks/R1_lidar_nav/meta.json``'s own ``status`` still says R1's numbers
    may not be published, and that is the stricter bar. Amendment 3 was written
    because this file had said FAIL while the arm had moved on; this asserts
    the pair stays in step in BOTH directions.
    """
    import json
    doc = json.loads(
        (REPO / "tests/benchmarks/agentbench/preregister/oracle_verdicts.json")
        .read_text(encoding="utf-8"))
    cells = {c["agent"]: c for c in doc["driver_gates"]["cells"]
             if c["task"] == "R1_lidar_nav" and c["sim"] == "omnisim"}
    assert cells["oracle"]["outcome"] == oracle[1].outcome, (
        "the recorded oracle cell says %r, this run measured %r"
        % (cells["oracle"]["outcome"], oracle[1].outcome))
    assert cells["null"]["outcome"] == null[1].outcome, (
        "the recorded null cell says %r, this run measured %r"
        % (cells["null"]["outcome"], null[1].outcome))
    v = doc["driver_gates"]["verdicts"]["R1_lidar_nav"]["omnisim"]
    assert v["discriminates"] is True
    assert v["oracle"] == "PASS" and v["null"] == "FAIL"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
