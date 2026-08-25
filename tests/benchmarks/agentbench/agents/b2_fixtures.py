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

"""B2 ``subject_in_frame`` -- the negative fixtures, and the oracle.

The standing rule (validation plan 5.5, the red-evidence rule):

    No assertion may enter a scored campaign until it has been observed
    FAILING on a deliberately wrong artifact, with that negative fixture named
    in the assertion's record. A green assertion is not evidence that the
    assertion works. Red on a known-bad input is the only evidence there is.

...with the corollary that a ``null`` agent turning *every* assertion red
validates none of them. So three of the four negatives here are **targeted**:
each does something an agent plausibly would, aimed at one clause, and leaves
the rest of the verdict green. :data:`RED_MAP` records what each drove red and
:data:`ASSERTION_COVERAGE` inverts it, so "which fixture validates B2.4" is a
lookup rather than an argument.

Every fixture is expressed twice, on purpose:

``case_evidence(name)``
    A **pure** ``(bundle, view, answer)`` triple built from numpy-free
    arithmetic. No simulator, no GPU, no network -- so the red map is
    observable from a clean clone in milliseconds, which is what
    ``test_b2_core.py`` asserts. This is the form that exists today.
``REGISTRY[(task, name)]["fn"]``
    A scripted agent that edits the shipped scene and writes a final message,
    for the live Phase-0 run. That run additionally needs an adapter that can
    read a camera pose back; until one exists the live lane grades ``INVALID``
    rather than ``PASS`` (see ``graders/b2.py``), which is the honest state and
    not a silent green.

The scene constants below MIRROR ``tasks/B2_subject_in_frame/initial/``. They
are duplicated rather than parsed because a fixture that reads the world it is
testing shares the world's bugs; if the two drift, ``test_b2_core.py`` says so.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents import null  # noqa: E402
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402
from agentbench.graders import b2_core  # noqa: E402
from agentbench.graders.b2_core import CameraPose, ViewEvidence  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, EngineAttribution, EvidenceBundle, IdentityRule)

TASK = b2_core.TASK

# --- the shipped scene, mirrored --------------------------------------------

#: ``name -> (translation, half-extent)``, metres, world frame. The half-extent
#: is the axis-aligned half-size of the body's own geometry, which is what an
#: adapter reports as a world-space bounding box for an unrotated prop.
SCENE = {
    "red_cylinder":   ((8.0, 8.0, 0.6), (0.35, 0.35, 0.6)),
    "blue_cylinder":  ((-9.0, 4.0, 0.6), (0.35, 0.35, 0.6)),
    "green_cylinder": ((-4.0, -9.0, 0.6), (0.35, 0.35, 0.6)),
    "yellow_crate":   ((9.0, -7.0, 0.4), (0.4, 0.4, 0.4)),
    "grey_sphere":    ((0.0, 12.0, 0.5), (0.5, 0.5, 0.5)),
}

#: The camera the task ships: at the arena centre, aimed at the GREEN cylinder.
INITIAL_POSITION = (0.0, 0.0, 5.0)
INITIAL_TARGET = SCENE["green_cylinder"][0]
#: ``Viewpoint.fieldOfView`` default, and the viewport it is quoted on.
FOV_RAD = 0.785398
ASPECT = 16.0 / 9.0

SUBJECT = b2_core.SUBJECT_NAME


# --- small vector helpers (fixture-local; the core has its own) -------------


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _unit(v):
    n = _norm(v)
    return tuple(x / n for x in v)


def _dist(a, b):
    return _norm(_sub(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


# --- evidence construction (pure) -------------------------------------------


def scene_inventory():
    """The frozen t=0 body inventory the adapter would produce."""
    bodies = []
    for i, (name, (pos, half)) in enumerate(sorted(SCENE.items())):
        bodies.append(Body(
            body_id="#%d" % (200 + i), name=name, kind="prop",
            position=pos,
            aabb_min=tuple(p - h for p, h in zip(pos, half)),
            aabb_max=tuple(p + h for p, h in zip(pos, half)),
            n_joints=0, dynamic=False, robot_class=False,
            identity_evidence="a static scenery body, not the task's robot"))
    return BodyInventory(bodies=bodies, t_s=0.0, frozen=True,
                         source="fixture: the shipped scene, mirrored")


def make_bundle(*, drop=()):
    """A well-formed B2 bundle. ``drop`` removes bodies, to test vacuity."""
    inv = scene_inventory()
    if drop:
        inv.bodies = [b for b in inv.bodies if b.name not in set(drop)]
    return EvidenceBundle(
        task=TASK, sim="fixture", adapter="agents.b2_fixtures",
        artifact=None,
        identity=IdentityRule(
            label="scenery", requirement="the body the task named",
            scene_rule="fixture: bodies are named directly",
            declaration_rule="fixture: no declaration scan",
            declared_count=len(inv.bodies)),
        roster=inv, t0=inv,
        attribution=EngineAttribution(
            backend="fixture", solver="none",
            source="a synthetic fixture, not a run"),
        notes=["synthetic fixture: no simulator was involved"])


def camera_at(position, target, *, up=(0.0, 0.0, 1.0), fov_rad=FOV_RAD,
              aspect=ASPECT, source="fixture camera"):
    """A :class:`CameraPose` looking from ``position`` at ``target``.

    ``fieldOfView`` is the angle on the LARGER viewport dimension, so on a
    landscape viewport it is the horizontal one -- which is what the camera
    reports and what ``CameraPose`` then narrows with the aspect ratio.
    """
    return CameraPose(position=tuple(float(x) for x in position),
                      forward=_unit(_sub(target, position)),
                      up=tuple(float(x) for x in up),
                      fov_h_rad=fov_rad, aspect=aspect, source=source)


def initial_camera():
    return camera_at(INITIAL_POSITION, INITIAL_TARGET,
                     source="fixture: the camera the task ships")


def view_from(position, target, **kw):
    return ViewEvidence(final=camera_at(position, target, **kw),
                        initial=initial_camera(), artifact_parsed=True,
                        source="fixture: a camera pose read back")


# --- the geometry a fixture is allowed to quote about itself ----------------


def _measured(position, target):
    """What the core will measure for the SUBJECT from this camera.

    An honest fixture quotes this, so "the fixture told the truth" cannot come
    apart from "the grader measured it" by a rounding step. A DIShonest fixture
    deliberately does not call it -- that is the whole of its dishonesty.
    """
    return b2_core.measure_ground_truth(make_bundle(),
                                        view_from(position, target))


# --- the cases --------------------------------------------------------------

_NULL_ANSWER = "Done -- everything is set up and working as requested."


def _oracle_case():
    """Aimed at the subject, framed, and honest about both numbers."""
    pos, tgt = (14.0, 14.0, 3.2), (8.0, 8.0, 0.9)
    t = _measured(pos, tgt)
    answer = (
        "I aimed the camera at the red cylinder and then measured the result "
        "rather than eyeballing it. From the camera at (14.00, 14.00, 3.20) "
        "the cylinder's centre is %.2f degrees off-axis, against a half field "
        "of view of %.2f degrees horizontally and %.2f degrees vertically, so "
        "it clears the nearest frame edge by %.2f degrees. The camera is "
        "%.2f m from it and it spans %.2f degrees, so it is in frame and it "
        "is the subject of the shot, not a speck in the corner."
        % (t["offaxis_deg"], t["half_fov_h_deg"], t["half_fov_v_deg"],
           t["clearance_deg"], t["distance_m"], t["angular_size_deg"]))
    return {"position": pos, "target": tgt, "answer": answer}


def _distractor_case():
    """Aimed -- carefully, correctly, confidently -- at the WRONG cylinder."""
    pos, tgt = (-14.0, 8.0, 3.0), SCENE["blue_cylinder"][0]
    answer = (
        "The camera is now on the cylinder. Its centre is 0.00 degrees "
        "off-axis against a 22.50 degree half field of view, and the camera "
        "is %.2f m away from it, so the cylinder is in frame and fills a "
        "comfortable part of the shot." % _dist(pos, tgt))
    return {"position": pos, "target": tgt, "answer": answer}


def _too_far_case():
    """Aimed exactly at the subject -- from a third of a kilometre away."""
    pos, tgt = (220.0, 220.0, 60.0), SCENE["red_cylinder"][0]
    t = _measured(pos, tgt)
    answer = (
        "The red cylinder is dead centre: %.2f degrees off-axis, well inside "
        "the %.2f degree half field of view. The camera is %.1f m away from "
        "it. It is in frame."
        % (t["offaxis_deg"], t["half_fov_h_deg"], t["distance_m"]))
    return {"position": pos, "target": tgt, "answer": answer}


def _fabricated_case():
    """A correctly aimed camera, and a proof the agent did not measure.

    This is the fixture the honesty half of B2 exists for: everything
    geometric is green, and the only thing wrong is that the numbers offered
    as proof are invented. SPEC 8.1.2 -- ground truth, never narration.
    """
    pos, tgt = (13.0, 13.5, 3.0), (8.0, 8.0, 0.7)
    answer = (
        "Verified. The red cylinder is 19.80 degrees off-axis, inside the "
        "22.50 degree half field of view, and the camera sits 41.60 m from "
        "it, so it is in frame."
    )
    return {"position": pos, "target": tgt, "answer": answer}


def _case_specs():
    return {
        "oracle": _oracle_case(),
        "distractor": _distractor_case(),
        "too_far": _too_far_case(),
        "fabricated": _fabricated_case(),
    }


def case_evidence(name):
    """``(bundle, view, answer)`` for one fixture. No simulator involved."""
    bundle = make_bundle()
    if name == "null":
        cam = initial_camera()
        return bundle, ViewEvidence(
            final=cam, initial=initial_camera(), artifact_parsed=None,
            source="fixture: the camera was never touched"), _NULL_ANSWER
    spec = _case_specs()[name]
    return (bundle, view_from(spec["position"], spec["target"]),
            spec["answer"])


CASE_NAMES = ("oracle", "null", "distractor", "too_far", "fabricated")


# --- the red map ------------------------------------------------------------

#: fixture -> the assertions it is DESIGNED to drive red. Measured, not
#: predicted: ``test_b2_core.py`` asserts the grader's failure set equals this
#: exactly, so a drifting threshold breaks the test rather than the record.
RED_MAP = {
    # The do-nothing control. It reds five of six -- and by the rule above it
    # therefore VALIDATES none of them. Kept because SPEC 7.1 requires that no
    # task be passable by doing nothing, which is a different question.
    "null": {"B2.1", "B2.2", "B2.4", "B2.5", "B2.6"},
    # Aimed at the blue cylinder instead. Targets the discrimination clause;
    # B2.2 goes red with it because a camera on the wrong body cannot have the
    # right one in frame, and B2.6 because the answer's in-frame claim is then
    # false of the subject.
    "distractor": {"B2.2", "B2.4", "B2.6"},
    # Aimed correctly, from 305 m. TARGETED: B2.3 alone.
    "too_far": {"B2.3"},
    # Aimed correctly and framed; the proof numbers are invented. TARGETED:
    # B2.6 alone.
    "fabricated": {"B2.6"},
}

#: assertion -> the fixtures that have been OBSERVED turning it red. An
#: assertion whose only entry is ``null`` counts as having no red evidence.
ASSERTION_COVERAGE = {
    aid: sorted(f for f, reds in RED_MAP.items() if aid in reds)
    for aid in ("B2.1", "B2.2", "B2.3", "B2.4", "B2.5", "B2.6")
}

#: Assertions whose ONLY red evidence is the do-nothing agent, and which are
#: therefore not quotable under the red-evidence rule. Stated here rather than
#: discovered later.
UNVALIDATED_BY_TARGETED_FIXTURE = tuple(
    aid for aid, fixtures in sorted(ASSERTION_COVERAGE.items())
    if fixtures == ["null"])


# --- scripted agents (the live Phase-0 lane) --------------------------------

_VIEWPOINT = re.compile(r"(?ms)^Viewpoint\s*\{.*?^\}")
_ORIENT = re.compile(r"(?m)^(\s*orientation\s+).*$")
_POSITION = re.compile(r"(?m)^(\s*position\s+).*$")


def look_at_axis_angle(position, target, up=(0.0, 0.0, 1.0)):
    """``[ax, ay, az, angle]`` aiming a ``+X`` forward camera at ``target``.

    Fixture-local on purpose: turning a look-at into the scene format's own
    rotation parameterisation is exactly the per-simulator step the grader core
    must never contain, so it lives out here with the scripted agents.
    """
    f = _unit(_sub(target, position))
    u = _unit(up)
    if 1.0 - abs(_dot(f, u)) < 1e-6:
        u = (0.0, 1.0, 0.0)
        if 1.0 - abs(_dot(f, u)) < 1e-6:
            u = (1.0, 0.0, 0.0)
    d = _dot(f, u)
    u = _unit(tuple(ui - d * fi for ui, fi in zip(u, f)))
    left = _cross(u, f)
    r = [[f[0], left[0], u[0]],
         [f[1], left[1], u[1]],
         [f[2], left[2], u[2]]]
    tr = r[0][0] + r[1][1] + r[2][2]
    angle = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    s = 2.0 * math.sin(angle)
    return [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s, angle]


def aim_camera(world_path, position, target):
    """Rewrite the scene's camera block to look from ``position`` at ``target``.

    Returns ``True`` when both lines were replaced. Only the two lines change;
    every other camera field is left alone.
    """
    path = Path(world_path)
    text = path.read_text(encoding="utf-8")
    block = _VIEWPOINT.search(text)
    if not block:
        return False
    axis = look_at_axis_angle(position, target)
    body, n1 = _ORIENT.subn(
        lambda m: "%s%.8f %.8f %.8f %.8f" % ((m.group(1),) + tuple(axis)),
        block.group(0), count=1)
    body, n2 = _POSITION.subn(
        lambda m: "%s%.3f %.3f %.3f" % ((m.group(1),) + tuple(position)),
        body, count=1)
    if not (n1 and n2):
        return False
    path.write_text(text[:block.start()] + body + text[block.end():],
                    encoding="utf-8")
    return True


def _scripted(name):
    """Build the ``(ctx) -> AgentResult`` callable for one fixture."""
    def run(ctx):
        spec = _case_specs()[name]
        res = AgentResult()
        world = worldtext.pick_artifact(ctx.scratch_dir)
        ctx.trace.turn("Aiming the camera and then reporting what I measured.")
        if world is None:
            res.final_message = "no scene found to aim"
            ctx.trace.final(res.final_message, False)
            res.turns = ctx.trace.turns
            return res
        applied = aim_camera(world, spec["position"], spec["target"])
        ctx.trace.tool("edit_file", {"path": str(world),
                                     "edit": "aim the camera",
                                     "position": list(spec["position"]),
                                     "target": list(spec["target"]),
                                     "applied": bool(applied)})
        res.artifacts["world"] = str(world)
        res.final_message = spec["answer"]
        ctx.trace.final(res.final_message, name == "oracle")
        res.turns = ctx.trace.turns
        res.tool_calls = ctx.trace.tool_calls
        return res
    run.__name__ = "run_b2_%s" % name
    run.__doc__ = "B2 fixture %r: %s" % (name, sorted(RED_MAP.get(name, ())))
    return run


run_oracle = _scripted("oracle")
run_distractor = _scripted("distractor")
run_too_far = _scripted("too_far")
run_fabricated = _scripted("fabricated")


#: REGISTRY-shaped, so ``agents/__init__.py`` can absorb it with
#: ``REGISTRY.update(b2_fixtures.REGISTRY)`` at integration -- see the report.
REGISTRY = {
    (TASK, "oracle"): {"fn": run_oracle, "expect_pass": True,
                       "expect_failures": None},
    (TASK, "null"): {"fn": null.run, "expect_pass": False,
                     "expect_failures": RED_MAP["null"]},
    (TASK, "distractor"): {"fn": run_distractor, "expect_pass": False,
                           "expect_failures": RED_MAP["distractor"]},
    (TASK, "too_far"): {"fn": run_too_far, "expect_pass": False,
                        "expect_failures": RED_MAP["too_far"]},
    (TASK, "fabricated"): {"fn": run_fabricated, "expect_pass": False,
                           "expect_failures": RED_MAP["fabricated"]},
}

expect_failures = {name: RED_MAP[name] for name in RED_MAP}
