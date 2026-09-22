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

"""Freeze gate.py's verdicts so the TypeScript port cannot drift from them.

    python tests/benchmarks/gate_parity/generate.py
    cp tests/benchmarks/gate_parity/cases.json \\
       O:/omnilink/api/tests/gate-parity-cases.json

WHY A GENERATED FIXTURE AND NOT A HAND-WRITTEN TABLE
----------------------------------------------------
OmniLink ships a TypeScript port of `gate.py`. Two implementations of a
safety rule that can disagree silently are worse than one, because each
side's tests pass while the product refuses different sentences
depending on which path a call took.

The first attempt at a guard was a table of expectations typed out by
hand in the TS test. Verifying it against the Python needed parsing TS
source with a regex, which silently matched **4 rows out of 45** — so
41 expectations were never checked against anything and the guard was
mostly decorative. A fixture both sides read cannot do that.

⚠️ THE EXPECTED VALUES ARE NOT OPINIONS. They are whatever gate.py does
today, printed. If a case here looks wrong, gate.py is wrong — fix it
there and regenerate, do not edit the JSON.

Regenerate whenever gate.py's behaviour changes, in the same commit. If
the TS test then fails, the port needs the same change; if it passes,
the change did not affect any covered case and the table needs a new one.

THE SHAPE OF A CASE
-------------------
Every row carries the four arguments `check()` takes and the verdict it
returned, so a port can replay it exactly:

    {"utterance": str,        # check() arg 1
     "tool": str, "args": {}, # check() arg 2, as one frame
     "surface": str | null,   # check() arg 4 -- the CALLER's robot class
     "registered": bool,      # was the registry loaded before this call?
     "rules": [str]}          # the rule names, in order

`surface` is `null` wherever the call passed none -- every case before
registration, and the deliberate no-surface rows after it. That is not
a hypothetical path: the platform's `/tool` body carries no surface
today, and any bridge that does not set `self.surface` reaches the gate
without one. It is NOT a decoration either: a port that ignores it
disagrees with gate.py on `move_body`, which the drone and the
quadruped both serve with the same three argument names meaning
entirely different things.

`registered` splits the table in two, and the split is load-bearing --
see the ORDER MATTERS note in `main()`. Rows with `registered: false`
must be replayed against the hand-written SPECS; rows with
`registered: true` must be replayed after the port has loaded the tool
registry the fixture publishes. A port with no registration will fail
the second group rather than silently disagree on it, which is the
point: until 2026-09-21 an unregistered tool was not merely unknown to
the gate, it was UNCHECKED.

THE `registry` KEY IS THE REGISTRATION TABLE, AND IT IS THE SOURCE OF
TRUTH FOR EVERY PORT
--------------------------------------------------------------------
Beside `cases`, the fixture publishes what was registered and in what
order:

    "registry": [{"surface": "drone",
                  "tools": [{"name": "takeoff", "physical": true,
                             "parameters": {"type": "object",
                                            "properties": {...}}}, ...]},
                 ...]

⚠️ **A PORT MUST READ THIS, NOT MIRROR IT.** The first TypeScript port
hand-wrote a `BRIDGE_SURFACE_TOOLS` constant to match `REGISTRATION`
below, because the fixture said a row was `registered` without saying
what had been registered. A hand-written copy of a generated table is
the same defect as a stale copy of this fixture, one level up: it
passes its own tests while describing a registry that no longer exists.
`registry` carries the three fields `register_tools()` actually reads --
`name`, `physical` and the JSON-schema `parameters` block -- so a port
replays it instead of restating it. The merge semantics it must apply
are in `register_tools()` (gate.py:587) and are summarised on
`_registry_block()` below.
"""
from __future__ import annotations

import copy
import itertools
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/omnisim-bridges/src"))
from omnisim_bridges import gate  # noqa: E402

# (utterance, tool, args). Chosen to exercise EVERY rule the gate can
# emit, plus every false refusal that shipped and was fixed -- those
# matter most, because a port that is merely "more careful" than the
# original is a regression, not an improvement.
CASES = [
    # the clean path
    ("drive forward 2 metres", "drive_forward", {"distance": 2}),
    ("turn left 90 degrees", "turn", {"angle_rad": 1.5708}),
    ("drive to x 3, y -1", "drive_to", {"x": 3, "y": -1}),
    ("stop", "stop_robot", {}),
    ("what is your position?", "get_robot_state", {"kind": "pose"}),
    ("wave hello", "wave", {}),
    ("pick up the cube", "pick", {"object": "cube"}),

    # intent rules
    ("how far can you drive forward?", "drive_forward", {"distance": 1}),
    ("how many times have you had to stop?", "stop", {}),
    ("under no circumstances drive forward", "drive_forward", {"distance": 1}),
    ("refrain from moving forward", "drive_forward", {"distance": 1}),
    ("avoid turning right", "turn", {"angle_rad": -1.5708}),
    ('the manual says "drive forward 2 metres"', "drive_forward", {"distance": 2}),
    ("the handbook states that you should reverse 2 metres", "drive_forward", {"distance": -2}),
    ("dispatch radioed that we need to turn left here", "turn", {"angle_rad": 1.5708}),
    ("once the shift ends, drive forward 2 metres", "drive_forward", {"distance": 2}),
    ("drive forward 1 metre in ten minutes", "drive_forward", {"distance": 1}),
    ("reverse 2 metres - sorry, ignore that", "drive_forward", {"distance": -2}),
    ("drive forward and also stay exactly where you are", "drive_forward", {"distance": 1}),
    ("turn left and right at the same time", "turn", {"angle_rad": 1.5708}),
    ("drive forward 1 metre without moving", "drive_forward", {"distance": 1}),
    ("suppose you drove forward 3 metres, where would you be?", "drive_forward", {"distance": 3}),

    # magnitude rails, per quantity
    ("drive forward 300 metres", "drive_forward", {"distance": 300}),
    ("turn left 3000 degrees", "turn", {"angle_rad": 52.36}),
    ("set velocity 40 forward", "set_velocity", {"v": 40, "w": 0}),
    ("spin at 500", "set_velocity", {"v": 0, "w": 500}),
    ("go fast", "set_velocity", {"linear": 40, "angular": 0}),
    ("take off to 200 metres", "takeoff", {"altitude": 200}),
    ("take off to 100 metres", "takeoff", {"altitude": 100}),
    ("shift the body forward 2 metres", "move_body", {"forward": 2, "lateral": 0, "vertical": 0}),
    ("drive forward negative 2 metres", "drive_forward", {"distance": -2}),
    ("drive forward 1 metre at 99", "drive_forward", {"distance": 1, "speed": 99}),

    # schema
    ("engage the warp drive", "warp", {"factor": 9}),
    ("drive forward 1 metre", "drive_forward", {"distance": 1, "turbo": True}),
    ("drive to the bay", "drive_to", {"x": 1}),
    ("drive forward", "drive_forward", {"distance": "fast"}),
    ("put it over there", "place", {"object": "it", "target": "there"}),

    # invented magnitude, and the empty-utterance carve-out that once
    # refused every drive on the bare /tool path
    ("forward", "drive_forward", {"distance": 1}),
    ("drive forward a metre", "drive_forward", {"distance": 1}),
    ("turn around", "turn", {"angle_rad": 3.1416}),
    ("", "drive_forward", {"distance": 2}),
    ("", "turn", {"angle_rad": 1.5708}),
    ("", "set_velocity", {"v": 40, "w": 0}),
    ("", "stop_robot", {}),

    # false refusals that shipped, and must stay allowed
    ("stop moving", "stop_robot", {}),
    ("stop driving", "stop_robot", {}),
    ("do not move", "stop_robot", {}),
    ("stand down", "stop_robot", {}),
    ("never mind, stop", "stop_robot", {}),
    ("collect the reports from bay 3", "pick", {"object": "reports"}),
    ("grab the delivery notes off the counter", "pick", {"object": "notes"}),
    ("drive forward 5 metres, scratch that, make it 2 metres", "drive_forward", {"distance": 2}),
    ("reverse 3 metres. never mind, go forward 1 metre", "drive_forward", {"distance": 1}),
    ("we'll find out later. square her up - 90 degrees right.", "turn", {"angle_rad": -1.5708}),
    ("turn left 90 degrees and then drive forward 1 metre", "drive_forward", {"distance": 1}),
    ("set your speed to 0.4 and drive forward 2 metres", "drive_forward", {"distance": 2, "speed": 0.4}),
    ("do a lap of the aisle, 3 metres", "drive_forward", {"distance": 3}),
    ("have the robot drive forward 3 metres", "drive_forward", {"distance": 3}),
    ("say the word and drive forward 2 metres", "drive_forward", {"distance": 2}),
    ("tell me your position and then drive forward 1 metre", "drive_forward", {"distance": 1}),
    ("back up 1 metre, give him room", "drive_forward", {"distance": -1}),

    # the bridge vocabulary, through normalise
    ("drive forward 2 metres", "drive_forward", {"distance": 2, "wait": True}),
    ("stop", "stop_robot", {"wait": True}),
    ("hold here", "hold_until_told", {"words": "until I say"}),
]

# ── Per-surface cases ────────────────────────────────────────────────
#
# ⚠️ EVERYTHING ABOVE IS A HUSKY. That was the gap: `gate.py` declared
# tools for four robot classes and this fixture froze the verdicts of one,
# so the TypeScript port could diverge on the other three and no test on
# either side would notice.
#
# These run AFTER `register_tools()`, because a per-surface rail needs a
# surface and a surface arrives with the bridge's registration. Any port
# that does not implement registration will fail these rather than
# silently disagree, which is the point.
#
# ── THE REGISTRY, AS AN ORDERED TUPLE AND NOT A DICT ────────────────
#
# ⚠️ REGISTRATION ORDER IS THE DEFECT THIS FIXTURE EXISTS TO CATCH, so it
# is pinned here rather than left to whatever a mapping happens to
# iterate in. `register_tools()` MERGES into the module-global `SPECS`,
# and recording one surface per spec used to make the `move_body` rail
# depend on which bridge booted first: drone-then-quadruped allowed a
# 40 m "body shift", quadruped-then-drone refused every real drone climb
# (gate.py, the "THE CALLER'S SURFACE WINS" note). `surface=` became an
# argument to `check()` because of it.
#
# A comment is not a guarantee, so `main()` regenerates the whole
# post-registration table under ALL 24 permutations of the four surfaces
# and refuses to write if any of them disagrees. If that check ever
# fires, the fixture is not the problem: an order-dependent verdict is a
# defect in gate.py and must be reported, not regenerated around.
#
# Schemas are the ones the shipped bridges publish in `Tool.parameters`
# (mavic `_drone_tools`, `build_quadruped_tools`, the arm and mobile
# bridges' registries), trimmed to the tools these cases exercise. One
# deliberate widening: the Mavic publishes `move_body{forward, vertical}`
# with no `lateral`, and the quadruped entry below supplies it, so the
# merged arg set is the same either way and a `lateral` case can be
# frozen for both rails.
REGISTRATION = (
    ("drone", (("takeoff", {"altitude": "number"}),
               ("land", {}), ("hover", {}),
               ("turn", {"angle_rad": "number"}),
               ("move_body", {"forward": "number", "lateral": "number",
                              "vertical": "number"}))),
    ("quadruped", (("sit", {}), ("stand", {}),
                   ("walk", {"distance": "number"}),
                   ("move_body", {"forward": "number", "lateral": "number",
                                  "vertical": "number"}))),
    ("arm", (("grasp", {"width": "number"}),
             ("set_tcp_target", {"x": "number", "y": "number", "z": "number"}),
             ("place", {"xyz": "array"}),
             ("get_reach_envelope", {}))),
    # The mobile bridge is here so that a `surface="mobile"` case is a
    # real registered surface rather than a string nothing has heard of.
    # Its tools are already hand-declared in SPECS, so registering them
    # proves the other half of the claim: a surface changes nothing on a
    # rail that does not depend on one.
    ("mobile", (("drive_forward", {"distance": "number", "speed": "number"}),
                ("turn", {"angle_rad": "number"}),
                ("drive_to", {"x": "number", "y": "number"}),
                ("set_velocity", {"v": "number", "w": "number"}))),
)

SURFACE_CASES = [
    # (surface, utterance, tool, args) -- the surface is the CALLER's, and
    # it selects the rail where two classes share a tool.
    # drone: the altitude rail fires for the first time. Before the Mavic
    # had a gated surface, MAX_ALTITUDE_M was unreachable -- the constant
    # existed and no drone frame could get to it.
    ("drone", "get up to 200 metres and have a look", "takeoff", {"altitude": 200.0}),
    ("drone", "take off to 3 metres", "takeoff", {"altitude": 3.0}),
    # THE DEFECT THIS WHOLE SURFACE EXISTS FOR: `land` is an ordinary
    # English word, and the drone's keyword ladder landed the aircraft on
    # "where did the package land?".
    ("drone", "where did the package land?", "land", {}),
    ("drone", "land", "land", {}),
    # A drone's `vertical` is a CLIMB, not a quadruped's body shift. This
    # was refused against the 1 m body-shift rail until 2026-09-21.
    ("drone", "climb 40 metres", "move_body", {"vertical": 40.0}),
    ("drone", "climb 300 metres", "move_body", {"vertical": 300.0}),
    ("drone", "fly forward 900 metres", "move_body", {"forward": 900.0}),

    # quadruped: the SAME tool, the SAME argument, a different rail.
    ("quadruped", "shift the body down 40 metres", "move_body", {"vertical": 40.0}),
    ("quadruped", "settle down a bit", "move_body", {"vertical": -0.3}),

    ("quadruped", "slide left 2 metres", "move_body", {"lateral": 2.0}),
    ("quadruped", "walk forward 2 metres", "walk", {"distance": 2.0}),
    ("quadruped", "walk forward 300 metres", "walk", {"distance": 300.0}),

    # ⚠️ THE SHARED TOOL WITH NO CALLER SURFACE -> THE STRICTEST RAIL.
    #
    # `move_body` is registered by the drone AND the quadruped. The
    # reference `bridge_base` handler threads the bridge's own surface
    # when it has one, but the platform's `/tool` body carries none and
    # an external bridge need not set the attribute, so this path is
    # live. Without a rule those calls would inherit whichever class
    # booted first. gate.py resolves it to the GROUND rail: guessing air
    # raises a 1 m limit to 120 m, and guessing ground is wrong only in
    # the direction that refuses a legitimate climb -- which is visible,
    # and fixed by passing `surface=`.
    #
    # ⚠️ THE FALLBACK IS THE STRICTEST RAIL ONLY FOR A *SHARED* TOOL.
    # When exactly one surface registered a tool, a surface-less call
    # takes THAT surface's rail, not the strictest one. Measured on the
    # shipped tree, where only the Mavic registers `move_body`:
    # `check("...", [move_body{vertical: 119}])` with no surface is
    # ALLOWED, because the drone is the only registrant. The port must
    # reproduce that, not a blanket "no surface means ground".
    #
    # These three are the ones a port is most likely to get wrong, and
    # the only ones that tell a correct fallback from "whatever was
    # registered last".
    (None, "shift the body up 40 metres", "move_body", {"vertical": 40.0}),
    (None, "shift the body up 0.5 metres", "move_body", {"vertical": 0.5}),
    (None, "nudge forward 2 metres", "move_body", {"forward": 2.0}),

    # A drone's HORIZONTAL translation is a flight, not a body shift, so
    # it takes the ground-DISTANCE rail (50 m) and not the 1 m one. 40 m
    # is the case that separates the two: legal on one rail, refused on
    # the other.
    ("drone", "fly forward 40 metres", "move_body", {"forward": 40.0}),
    ("drone", "slide left 2 metres", "move_body", {"lateral": 2.0}),

    # The self-naming verbs. `takeoff`/`land`/`hover` are air whatever
    # the caller says it is, and `altitude` is railed at 120 m on its own
    # quantity rather than on anybody's surface -- so a wrong surface,
    # and no surface at all, must not move these verdicts.
    ("drone", "hover here", "hover", {}),
    (None, "hover here", "hover", {}),
    (None, "land", "land", {}),
    (None, "take off to 3 metres", "takeoff", {"altitude": 3.0}),
    (None, "take off to 200 metres", "takeoff", {"altitude": 200.0}),
    ("quadruped", "take off to 200 metres", "takeoff", {"altitude": 200.0}),
    ("drone", "are we clear to land?", "land", {}),
    ("drone", "how high can you get?", "takeoff", {"altitude": 100.0}),

    # arm: tools that were entirely ungated, because SPECS had never heard
    # of them and `unknown_tool` is filtered at every call site.
    ("arm", "how much reach have you got left?", "grasp", {"width": 0.05}),
    ("arm", "close on the block", "grasp", {"width": 0.05}),
    ("arm", "close on the block", "grasp", {"turbo": True}),
    ("arm", "put it on the pallet", "place", {"xyz": [0.3, 0.2, 0.1]}),
    ("arm", "how much reach have you got left?", "get_reach_envelope", {}),
    ("arm", "move the tool to the tray", "set_tcp_target",
     {"x": 0.3, "y": 0.2, "z": 0.1}),
    ("arm", "move the tool to the tray", "set_tcp_target", {"turbo": True}),

    # ── A surface must NOT move a rail that does not depend on one ────
    #
    # The other half of the claim, and the cheaper half to get wrong: a
    # port that selects rails by surface everywhere would start refusing
    # a 2 m drive on an arm, or passing a 300 m one on a drone. The
    # ground-distance, speed and yaw-rate rails are surface-blind, and
    # the same four calls with four different surfaces prove it.
    ("mobile", "drive forward 2 metres", "drive_forward", {"distance": 2}),
    ("mobile", "drive forward 300 metres", "drive_forward", {"distance": 300}),
    ("drone", "drive forward 300 metres", "drive_forward", {"distance": 300}),
    ("arm", "drive forward 300 metres", "drive_forward", {"distance": 300}),
    ("quadruped", "drive forward 300 metres", "drive_forward", {"distance": 300}),
    ("mobile", "set velocity 40 forward", "set_velocity", {"v": 40, "w": 0}),
    ("arm", "set velocity 40 forward", "set_velocity", {"v": 40, "w": 0}),
    ("mobile", "stop", "stop_robot", {}),
    ("mobile", "turn left 90 degrees", "turn", {"angle_rad": 1.5708}),
    ("drone", "turn left 90 degrees", "turn", {"angle_rad": 1.5708}),
]

# Every rule the gate can emit. A table that exercises half of them lets
# the other half drift freely, so generation FAILS if one is uncovered.
MUST_COVER = {
    "interrogative", "prohibition", "self_negating", "reported_speech",
    "retracted", "deferred", "contradiction", "invented_magnitude",
    "implausible", "sign_conflict", "unknown_tool", "unknown_arg",
    "missing_arg", "bad_type", "unresolved_referent",
}

# ⚠️ TWO RULES ARE NOT PINNED HERE, AND THIS IS THE RECORD OF WHY.
#
#   not_finite          needs an Infinity or NaN argument, and this
#                       fixture is JSON that a TypeScript test parses.
#                       Python would emit bare `Infinity`, which is not
#                       valid JSON and which JSON.parse rejects. Pinning
#                       it needs an encoding both sides agree on first.
#   needs_authorization only fires for a CONFIRM-tier tool, and SPECS
#                       currently declares none. It cannot be exercised
#                       until one exists.
#
# Neither is a judgement that they do not matter. They are both live rules
# in gate.py and in the port, and until they appear above, either side can
# change them without the other finding out.
UNPINNED = {"not_finite", "needs_authorization"}


_JSON_TYPES = {"number": {"type": "number"}, "array": {"type": "array"},
               "string": {"type": "string"}}


class _T:                           # a Tool, without importing the relay
    def __init__(self, name, props):
        self.name = name
        self.parameters = {"type": "object",
                           "properties": {k: _JSON_TYPES[v]
                                          for k, v in props.items()}}
        # `get_` is the bridges' own convention for a read-only verb, and
        # a read-only tool keeps the carve-out that lets it answer a
        # question. Registration may not GRANT that; it only declines to
        # claim the tool is physical.
        self.physical = not name.startswith("get_")


def _tools_for(surface) -> list:
    """The `Tool`-shaped objects handed to `register_tools()` for a surface.

    ⚠️ ONE CONSTRUCTOR, TWO CONSUMERS. The registration below and the
    `registry` block written into the fixture both come from here, so
    what the JSON publishes cannot drift from what the gate was actually
    taught. Building the JSON from a second literal would recreate, one
    level up, exactly the defect this whole fixture exists to kill.
    """
    return [_T(n, p) for n, p in dict(REGISTRATION)[surface]]


def _register_surfaces(order) -> None:
    """Teach the gate the per-surface tool sets before the surface cases.

    Mirrors what a bridge does at start-up. Without it the surface cases
    would measure the hand-written defaults and freeze the wrong verdicts.

    `order` is the sequence of surface names to register, so the caller
    can replay the same registry in every permutation.
    """
    for surface in order:
        gate.register_tools(_tools_for(surface), surface=surface)


def _registry_block(order) -> list:
    """The registration table, as the fixture publishes it.

    Serialises the very objects `register_tools()` is given: the tool's
    name, its `physical` flag, and its JSON-schema `parameters` block
    verbatim. Those three fields are ALL that `register_tools()` reads
    (gate.py:587), so a port that replays this block registers exactly
    what this generator registered.

    `parameters.required` is absent throughout because no tool in this
    table declares one. It is emitted verbatim when one does, and a port
    must honour it: `required` is taken from the schema ONLY for a tool
    SPECS did not already know, since tightening an existing tool's
    requirements from a bridge schema would refuse parser frames that
    are valid today.
    """
    return [{"surface": surface,
             "tools": [{"name": t.name, "physical": t.physical,
                        "parameters": t.parameters}
                       for t in _tools_for(surface)]}
            for surface in order]


class _Published:
    """A tool rebuilt from the fixture's own `registry` block.

    What a port has to work with: three JSON fields and nothing else. If
    registering these does not reproduce the verdicts, the block is
    missing something `register_tools()` reads and no port can replay it.
    """
    def __init__(self, entry):
        self.name = entry["name"]
        self.physical = entry["physical"]
        self.parameters = entry["parameters"]


def _surface_rows(order, registry=None) -> list:
    """The post-registration table, generated under one registration order.

    Restores `SPECS` afterwards, so a caller can run this once per
    permutation and compare. `register_tools()` mutates the module-global
    and nothing else in the process may inherit that state.

    With `registry`, the tools come from a published `registry` block
    instead of from `REGISTRATION` -- the round trip a port makes.
    """
    saved = copy.deepcopy(gate.SPECS)
    try:
        if registry is None:
            _register_surfaces(order)
        else:
            for entry in registry:
                gate.register_tools([_Published(t) for t in entry["tools"]],
                                    surface=entry["surface"])
        rows = []
        for surface, utterance, tool, args in SURFACE_CASES:
            verdict = [r.rule for r in
                       gate.check(utterance, [{"tool": tool, "args": args}],
                                  surface=surface)]
            rows.append({"utterance": utterance, "tool": tool, "args": args,
                         "surface": surface, "registered": True,
                         "rules": verdict})
        return rows
    finally:
        gate.SPECS.clear()
        gate.SPECS.update(saved)


def main() -> int:
    rows, covered = [], set()
    # ⚠️ ORDER MATTERS between the two blocks. Registration MERGES into
    # SPECS, so every case in `CASES` is frozen against the hand-written
    # defaults and every case in `SURFACE_CASES` against a registered
    # bridge. Running the Husky cases after registration would quietly
    # re-freeze them under different rules. `_surface_rows()` restores
    # SPECS, so this block runs against a pristine table however many
    # permutations were tried below.
    for utterance, tool, args in CASES:
        verdict = [r.rule for r in
                   gate.check(utterance, [{"tool": tool, "args": args}])]
        covered.update(verdict)
        rows.append({"utterance": utterance, "tool": tool, "args": args,
                     "surface": None, "registered": False, "rules": verdict})

    canonical_order = tuple(s for s, _ in REGISTRATION)
    surface_rows = _surface_rows(canonical_order)

    # ⚠️ THE ORDER-INDEPENDENCE PROOF. An order-dependent verdict is worse
    # than a wrong one, because it looks correct in whichever order you
    # happen to test -- and this fixture would freeze whichever answer
    # this machine's registration produced. Every permutation must agree
    # or nothing is written.
    for perm in itertools.permutations(canonical_order):
        other = _surface_rows(perm)
        if other == surface_rows:
            continue
        print(f"REFUSING TO WRITE: the verdicts depend on registration "
              f"order.\n  canonical: {canonical_order}\n  differing:  {perm}")
        for a, b in zip(surface_rows, other):
            if a != b:
                print(f"  {a['surface']} {a['tool']}{a['args']}: "
                      f"{a['rules']} vs {b['rules']}")
        print("This is a defect in gate.py, not in the fixture. Report it.")
        return 1

    # ⚠️ THE PUBLISHED REGISTRY MUST BE SUFFICIENT ON ITS OWN. The whole
    # point of shipping it is that a port replays it instead of
    # hand-writing a mirror, so it is round-tripped through JSON and
    # registered from nothing but its own three fields. A block that
    # cannot reproduce these verdicts is a block that silently sends the
    # port back to a hand-written table.
    registry = json.loads(json.dumps(_registry_block(canonical_order)))
    replayed = _surface_rows(canonical_order, registry=registry)
    if replayed != surface_rows:
        print("REFUSING TO WRITE: the published `registry` block does not "
              "reproduce the verdicts it was generated with.")
        for a, b in zip(surface_rows, replayed):
            if a != b:
                print(f"  {a['surface']} {a['tool']}{a['args']}: "
                      f"{a['rules']} vs {b['rules']}")
        print("It is missing something register_tools() reads.")
        return 1

    rows.extend(surface_rows)
    covered.update(r for row in surface_rows for r in row["rules"])

    missing = MUST_COVER - covered
    if missing:
        print(f"REFUSING TO WRITE: no case emits {sorted(missing)}.")
        print("Add one, or the rule can change behaviour unnoticed.")
        return 1

    allowed = sum(1 for r in rows if not r["rules"])
    # The TS test asserts the fixture is substantial and contains BOTH
    # kinds of row; a fixture that drifted to all-refused or all-allowed
    # would make every other assertion in it vacuous.
    if allowed == 0 or allowed == len(rows) or len(rows) < 80:
        print(f"REFUSING TO WRITE: {len(rows)} cases, {allowed} allowed. "
              "The table must hold at least 80 cases and both verdicts.")
        return 1

    out = pathlib.Path(__file__).with_name("cases.json")
    out.write_text(json.dumps(
        {"generated_from": "packages/omnisim-bridges/src/omnisim_bridges/gate.py",
         "note": "Generated. Do not hand-edit: regenerate with generate.py.",
         "registry_note": (
             "THE SOURCE OF TRUTH FOR ANY PORT'S REGISTRATION TABLE. "
             "`registry` is the tool set handed to gate.register_tools(), "
             "surface by surface, in the order it was registered. Replay it "
             "before every case with registered=true and pass case.surface "
             "as check()'s surface argument. Do not hand-write a mirror of "
             "it: a hand-written copy of a generated table is the drift "
             "this fixture exists to eliminate."),
         "registry": registry,
         "cases": rows}, indent=2) + "\n", encoding="utf-8")
    surfaced = sum(1 for r in rows if r["surface"])
    print(f"wrote {out}")
    print(f"  {len(rows)} cases, {allowed} allowed, {len(rows)-allowed} refused")
    print(f"  {len(rows)-len(surface_rows)} before registration, "
          f"{len(surface_rows)} after; {surfaced} carry a caller surface")
    print(f"  rules covered: {len(covered)}")
    print(f"  registry: {sum(len(e['tools']) for e in registry)} tools over "
          f"{len(registry)} surfaces ({', '.join(canonical_order)}), "
          f"replayed from the published block")
    print(f"  registration order-independent over "
          f"{len(list(itertools.permutations(canonical_order)))} permutations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
