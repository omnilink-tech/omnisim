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

"""The flying bridge's `/capabilities.safety_gate` must be TRUE (PROTOCOL §5.2.1).

⚠️ WHAT THIS FILE IS ABOUT, because it reads like bookkeeping and is not.
`ungated_paths` is the load-bearing half of the capability block: a client
reads it to decide whether *it* is responsible for bounding a command. A
bridge that publishes `present: true` and gets that list wrong invites the
one reading that gets somebody hurt -- "everything here is vetted".

The Mavic got it wrong in BOTH directions and neither was visible from the
block itself:

  - `UNGATED_PATHS = ["/reset", "/complete_mission"]` named `action` VERBS,
    not HTTP routes. This bridge's only POST routes are `/prompt`, `/tool`
    and `/action`; `reset` and `complete_mission` are values of the request
    body's `action` field, so neither string was a route at all.
  - `/reset` was published as UNVETTED while `_ACTION_AS_TOOL` maps it onto
    `reset_to_home` and puts it through `vet_toolcall`. Wrong in the
    conservative direction, which is still wrong.
  - the genuinely unvetted verbs were ABSENT. `goto_waypoint` flies the
    aircraft to a coordinate and has no mapping onto the gate's vocabulary,
    so it is the most consequential unchecked verb on this bridge -- and a
    client reading the published block would have believed it was checked.

So this file drives the SHIPPED source, not a copy of it: the controller
imports `omnisim` at module scope and cannot be imported off a running
engine (the same constraint `test_step_budget` works around), so every fact
below is lifted out of the file's AST. It pins the *published values*
wherever they live, so it keeps working if the constants move.

⚠️ IT IS ALSO THE DRIFT GUARD. `test_every_action_verb_the_router_serves_is_declared`
fails the moment somebody adds an `if action == "..."` branch without
declaring the verb, which is what makes the derived publication stay true
instead of merely being true once.
"""

from __future__ import annotations

import ast
import pathlib
import re
from typing import Any, Dict, List, Optional, Set

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
MAVIC = (ROOT / "projects/samples/demos/controllers/mavic_omnilink_bridge"
         / "mavic_omnilink_bridge.py")

SRC = MAVIC.read_text(encoding="utf-8")
TREE = ast.parse(SRC)


# ── lifting values out of the shipped file ───────────────────────────────

def _module_namespace() -> Dict[str, Any]:
    """Execute the file's module-level CONSTANT assignments, and only those.

    Nothing here imports, opens a socket or touches `omnisim`: a statement
    is replayed only when it is a plain assignment whose value compiles and
    runs against what earlier constants already put in the namespace. That
    is enough for the verb tuples and the derived note, and it refuses to
    become a module loader.
    """
    # ⚠️ ns is the GLOBALS, not the locals. Split into two mappings, a
    # generator expression in a replayed assignment cannot see the constants
    # the earlier statements defined, so every derived tuple here would
    # silently fail to replay and the note would come back empty -- a test
    # that checks nothing while reporting green.
    ns: Dict[str, Any] = {"__builtins__": __builtins__}
    for node in TREE.body:
        if not isinstance(node, ast.Assign):
            continue
        if not all(isinstance(t, ast.Name) for t in node.targets):
            continue
        if not all(n.id.isupper() or n.id.lstrip("_").isupper()
                   for n in node.targets):          # type: ignore[attr-defined]
            continue
        try:
            mod = ast.Module(body=[node], type_ignores=[])
            exec(compile(mod, str(MAVIC), "exec"), ns)
        except Exception:
            continue                                 # not a constant we can replay
    ns.pop("__builtins__", None)
    return ns


NS = _module_namespace()


def _resolve(node: ast.AST) -> Any:
    """A literal, a constant NAME, or `list(NAME)` / `tuple(NAME)`."""
    try:
        return ast.literal_eval(node)
    except Exception:
        pass
    if isinstance(node, ast.Name) and node.id in NS:
        return NS[node.id]
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id in ("list", "tuple", "sorted") and len(node.args) == 1):
        inner = _resolve(node.args[0])
        if inner is not None:
            return {"list": list, "tuple": tuple, "sorted": sorted}[node.func.id](inner)
    return None


def _assigned(name: str) -> Any:
    """The value assigned to `name` or `self.name`, anywhere in the file."""
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            hit = ((isinstance(target, ast.Name) and target.id == name)
                   or (isinstance(target, ast.Attribute) and target.attr == name))
            if hit:
                value = _resolve(node.value)
                if value is not None:
                    return value
    return None


def _dict_value(key: str) -> Any:
    """The value paired with `key` in any dict literal in the file."""
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Dict):
            continue
        for k, v in zip(node.keys, node.values):
            if isinstance(k, ast.Constant) and k.value == key:
                value = _resolve(v)
                if value is not None:
                    return value
    return None


def _func(name: str) -> Optional[ast.FunctionDef]:
    return next((n for n in ast.walk(TREE)
                 if isinstance(n, ast.FunctionDef) and n.name == name), None)


def _string_comparisons(fn: ast.FunctionDef, var: str) -> Set[str]:
    """Every `"<literal>"` this function compares `var` against."""
    out: Set[str] = set()
    for node in ast.walk(fn):
        if not isinstance(node, ast.Compare):
            continue
        if not (isinstance(node.left, ast.Name) and node.left.id == var):
            continue
        for comparator in node.comparators:
            if isinstance(comparator, ast.Constant) and isinstance(comparator.value, str):
                out.add(comparator.value)
    return out


ROUTE_POST = _func("_route_post")
assert ROUTE_POST is not None, "the flying bridge has no _route_post"

#: Verbs the router actually serves, from its own `if action == "..."` tests.
ROUTER_VERBS = _string_comparisons(ROUTE_POST, "action")
#: POST routes the router actually serves, from its own `p == "..."` tests.
POST_ROUTES = _string_comparisons(ROUTE_POST, "p")

#: `action` -> gate tool. The presence of a key here is the ONLY thing that
#: puts a verb through `vet_toolcall`, so this map IS the ground truth about
#: which half of the publication a verb belongs in.
ACTION_AS_TOOL: Dict[str, str] = _assigned("_ACTION_AS_TOOL") or {}

DECLARED_VERBS: List[str] = list(_assigned("ACTION_VERBS")
                                 or _dict_value("available_actions") or [])

GATED_PATHS: List[str] = list(_assigned("GATED_PATHS") or [])
UNGATED_PATHS: List[str] = list(_assigned("UNGATED_PATHS") or [])
NOTE: str = str(_dict_value("safety_gate_note") or _assigned("SAFETY_GATE_NOTE") or "")

VETTED = {v for v in ROUTER_VERBS if v in ACTION_AS_TOOL}
UNVETTED = {v for v in ROUTER_VERBS if v not in ACTION_AS_TOOL}


def _note_clause(label: str) -> Set[str]:
    """The verb set the note publishes under `label`."""
    m = re.search(re.escape(label) + r" verbs \([^)]*\): ([^.]+)\.", NOTE)
    if not m:
        return set()
    return {part.strip() for part in m.group(1).split(",") if part.strip()}


# ── the ground truth this file exists to pin ─────────────────────────────

def test_the_extractor_found_a_bridge_to_read() -> None:
    # A silent extraction failure would turn every assertion below into a
    # green "nothing to check", which is the failure mode a safety test can
    # least afford.
    assert ROUTER_VERBS, "no `if action == ...` branches found"
    assert POST_ROUTES, "no POST routes found"
    assert ACTION_AS_TOOL, "no _ACTION_AS_TOOL map found"
    assert GATED_PATHS and NOTE, "no published safety_gate to check"


def test_every_action_verb_the_router_serves_is_declared() -> None:
    # THE DRIFT GUARD. Add a verb to the router and this fails until the
    # declared inventory (and therefore the publication derived from it)
    # names it. Without this, a new unvetted verb would ship invisible --
    # exactly how `goto_waypoint` and `set_gimbal_pitch` came to be absent.
    assert set(DECLARED_VERBS) == ROUTER_VERBS, (
        "the router and the declared action list disagree; "
        f"router-only={sorted(ROUTER_VERBS - set(DECLARED_VERBS))} "
        f"declared-only={sorted(set(DECLARED_VERBS) - ROUTER_VERBS)}")


def test_the_published_path_lists_name_ROUTES_not_action_verbs() -> None:
    # `/reset` and `/complete_mission` were published as routes for a day.
    # They are values of the request body's `action` field; posting to
    # `http://127.0.0.1:6090/reset` has always been a 404.
    for path in list(GATED_PATHS) + list(UNGATED_PATHS):
        assert path in POST_ROUTES, (
            f"{path!r} is published in the safety_gate block but is not a "
            f"route this bridge serves (routes: {sorted(POST_ROUTES)}); "
            "an `action` verb is not an HTTP path")


def test_action_is_published_as_a_partially_vetted_route() -> None:
    # PROTOCOL §5.2.1: `ungated_paths` is the load-bearing half, and an
    # EMPTY one is the worst answer available -- it reads as "everything
    # here is vetted". /action carries both vetted and unvetted verbs, so
    # it belongs in both halves, with the split named in the note.
    assert "/action" in GATED_PATHS, "/action's mapped verbs ARE vetted"
    assert UNGATED_PATHS, (
        "empty ungated_paths on a bridge with three unvetted /action verbs "
        "reads as 'everything here is vetted'")
    assert "/action" in UNGATED_PATHS, (
        "/action carries verbs the gate never sees; publishing it as gated "
        "only tells a client it need not bound them")


def test_the_note_names_exactly_the_verbs_the_gate_actually_sees() -> None:
    published = _note_clause("Vetted")
    assert published == VETTED, (
        "the note's vetted list disagrees with _ACTION_AS_TOOL; "
        f"note={sorted(published)} actual={sorted(VETTED)}")


def test_the_note_names_every_verb_the_gate_never_sees() -> None:
    # `goto_waypoint` flies the aircraft to a coordinate with no rail on it.
    # If it is not discoverable as unvetted, the block is a tidy falsehood.
    published = _note_clause("UNVETTED")
    assert published == UNVETTED, (
        "the note's unvetted list disagrees with _ACTION_AS_TOOL; "
        f"note={sorted(published)} actual={sorted(UNVETTED)}")
    assert {"goto_waypoint", "set_gimbal_pitch"} <= published, (
        "the two flight verbs with no gate mapping must be discoverable")


def test_reset_is_never_published_as_unvetted() -> None:
    # Wrong in the conservative direction is still wrong: a client that
    # believes it must bound `reset` writes bounding code nobody needed,
    # and learns the publication cannot be trusted either way.
    assert "reset" in ACTION_AS_TOOL, "reset lost its gate mapping"
    assert "reset" not in _note_clause("UNVETTED")
    assert "/reset" not in UNGATED_PATHS and "/reset" not in GATED_PATHS


def test_the_note_does_not_place_a_verb_on_both_sides() -> None:
    assert not (_note_clause("Vetted") & _note_clause("UNVETTED"))


def test_every_mapped_verb_targets_a_tool_this_bridge_actually_serves() -> None:
    # A mapping onto a name the bridge does not serve is not a gate: the
    # gate deliberately DROPS `unknown_tool` (it does not police existence),
    # so a typo'd target would sail through every schema and magnitude rule
    # while the publication claimed the verb was vetted.
    tools_fn = _func("_drone_tools")
    assert tools_fn is not None
    ret = next(n for n in ast.walk(tools_fn) if isinstance(n, ast.Return))
    assert isinstance(ret.value, ast.Dict)
    served = {k.value for k in ret.value.keys
              if isinstance(k, ast.Constant)}
    missing = sorted(set(ACTION_AS_TOOL.values()) - served)
    assert not missing, f"_ACTION_AS_TOOL targets tools this bridge does not serve: {missing}"


@pytest.mark.parametrize("verb", sorted(UNVETTED))
def test_each_unvetted_verb_is_named_in_the_published_note(verb: str) -> None:
    assert verb in NOTE, f"{verb} reaches an actuator unchecked and is unpublished"


def test_the_action_refusal_carries_a_rule_and_files_an_event() -> None:
    # GATE_COVERAGE.md called /action "the one gated path carrying neither":
    # it refuses before reaching the shared handler, so its 400 had no
    # machine-readable `rule` and no `gate.refused` ever reached the ring.
    # `reason` is prose a client must not branch on, and a refusal nobody
    # can see is the one event on this path the robot's own agent never
    # learns about.
    body = ast.get_source_segment(SRC, ROUTE_POST) or ""
    assert "refusal_rule(" in body, "/action's 400 carries no machine-readable rule"
    assert "emit_refusal(" in body, "/action files no gate.refused event"
    assert 'origin="action"' in body, "the refusal must name the surface it arrived on"
