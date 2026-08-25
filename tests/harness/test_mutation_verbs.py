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

"""Unit tests for the /capabilities + scene-mutation + snapshot machinery.

Everything under test here is pure: VRML composition, the depth-aware field
rewrite the clone path depends on, the self-cross-checks /capabilities
publishes, and the backend-verdict reader. The endpoints themselves need a
running engine (see scripts/harness/README.md); these lock down the parts that
have no simulator in the loop.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SUPERVISOR_DIR = REPO_ROOT / "projects" / "default" / "controllers" / "harness_supervisor"
HARNESS_DIR = REPO_ROOT / "scripts" / "harness"

# harness_supervisor.py itself imports `from omnisim import Supervisor`, which
# only exists inside a running controller, so the pure helpers are exec'd out
# of the source instead of imported (same trick as test_observability.py, one
# step further).
sys.path.insert(0, str(SUPERVISOR_DIR))
sys.path.insert(0, str(HARNESS_DIR))


def _supervisor_helpers():
    src = (SUPERVISOR_DIR / "harness_supervisor.py").read_text(encoding="utf-8")
    start = src.index("_DEF_PREFIX_RE = ")
    end = src.index("def pose_fingerprint")
    ns: dict = {"re": __import__("re")}
    exec(compile(src[start:end], "harness_supervisor_helpers", "exec"), ns)  # noqa: S102
    # compare_fingerprints lives below pose_fingerprint and needs `math`.
    start2 = src.index("def compare_fingerprints")
    end2 = src.index("def _advance")
    ns["math"] = __import__("math")
    exec(compile(src[start2:end2], "harness_supervisor_helpers", "exec"), ns)  # noqa: S102
    return ns


# ---------------------------------------------------------------------------
# VRML composition (POST /scene/spawn)
# ---------------------------------------------------------------------------


def test_vrml_value_scalars_and_bools():
    from omnisim_harness import vrml_value
    assert vrml_value(True) == "TRUE"
    assert vrml_value(False) == "FALSE"
    assert vrml_value(3) == "3"
    assert vrml_value("a") == '"a"'
    assert vrml_value([1, 2, 0.5]) == "1 2 0.5"
    assert vrml_value(None) == "NULL"


def test_vrml_value_empty_list_is_an_empty_mf_field():
    """`children []` must not serialize to `children ` — the parser then reads
    the NEXT field name as this field's node value and the whole node is
    rejected ("Missing declaration for 'boundingObject', unknown node")."""
    from omnisim_harness import vrml_value
    assert vrml_value([]) == "[ ]"


def test_vrml_value_quotes_are_escaped():
    from omnisim_harness import vrml_value
    assert vrml_value('a"b\\c') == '"a\\"b\\\\c"'


def test_vrml_value_refuses_a_nested_dict():
    from omnisim_harness import vrml_value
    with pytest.raises(ValueError):
        vrml_value({"geometry": "Box"})


def test_compose_spawn_from_type_and_fields():
    from omnisim_harness import compose_spawn_vrml
    vrml, def_name = compose_spawn_vrml(
        {"def": "BOX", "type": "Solid", "translation": [1, 2, 3],
         "rotation": [0, 0, 1, 0.5], "fields": {"name": "box"}}, REPO_ROOT)
    assert def_name == "BOX"
    assert vrml.startswith("DEF BOX Solid {")
    assert 'name "box"' in vrml
    assert "translation 1 2 3" in vrml
    assert "rotation 0 0 1 0.5" in vrml


def test_compose_spawn_urdf_sugar_resolves_to_an_absolute_posix_url():
    from omnisim_harness import compose_spawn_vrml
    vrml, _ = compose_spawn_vrml(
        {"urdf": "projects/robots/clearpath/husky_description/urdf/husky.urdf"},
        REPO_ROOT)
    assert vrml.startswith("URDFRobot {")
    # Backslashes are not legal inside a VRML string; the engine wants posix.
    assert "\\" not in vrml
    assert vrml.count('url "') == 1
    assert "/projects/robots/clearpath/" in vrml


def test_compose_spawn_raw_vrml_gets_pose_spliced_after_the_brace():
    from omnisim_harness import compose_spawn_vrml
    vrml, def_name = compose_spawn_vrml(
        {"def": "P", "vrml": 'Solid { name "p" }', "translation": [1, 1, 1]},
        REPO_ROOT)
    assert def_name == "P"
    assert vrml.startswith("DEF P Solid {")
    assert "translation 1 1 1" in vrml
    # Splicing into the node text is what makes the body appear AT the pose,
    # so a spawn needs no settle step to read back correctly.
    assert vrml.index("translation") < vrml.index('name "p"')


def test_compose_spawn_does_not_duplicate_an_existing_pose_field():
    from omnisim_harness import compose_spawn_vrml
    vrml, _ = compose_spawn_vrml(
        {"vrml": 'Solid { translation 9 9 9 }', "translation": [1, 1, 1]},
        REPO_ROOT)
    assert vrml.count("translation") == 1
    assert "9 9 9" in vrml


def test_compose_spawn_requires_a_usable_spec():
    from omnisim_harness import compose_spawn_vrml
    with pytest.raises(ValueError):
        compose_spawn_vrml({"def": "X"}, REPO_ROOT)
    with pytest.raises(ValueError):
        compose_spawn_vrml({"type": "Solid", "translation": [1, 2]}, REPO_ROOT)


# ---------------------------------------------------------------------------
# The clone path's depth-aware field rewrite
# ---------------------------------------------------------------------------


CLONE_SAMPLE = """Robot {
  translation 1 2 0.2
  rotation 0 0 1 1.5
  children [
    Solid {
      name "wheel"
      translation 0.2 0 0
    }
  ]
  name "husky_0"
  controller "husky_random"
  supervisor TRUE
}"""


def test_replace_top_level_field_skips_nested_fields_of_the_same_name():
    """The whole point: a robot subtree is full of `name` / `translation`
    fields, and only the node's own may be rewritten."""
    ns = _supervisor_helpers()
    out, ok = ns["replace_top_level_field"](CLONE_SAMPLE, "name", '"husky_7"')
    assert ok
    assert 'name "husky_7"' in out
    assert 'name "wheel"' in out          # child untouched
    out, ok = ns["replace_top_level_field"](out, "translation", "5 6 0.2")
    assert ok
    assert "translation 5 6 0.2" in out
    assert "translation 0.2 0 0" in out   # child untouched


def test_replace_top_level_field_reports_a_miss_instead_of_guessing():
    ns = _supervisor_helpers()
    out, ok = ns["replace_top_level_field"](CLONE_SAMPLE, "nosuchfield", "1")
    assert ok is False
    assert out == CLONE_SAMPLE


def test_replace_top_level_field_refuses_a_node_valued_field():
    """`boundingObject Box { ... }` is not a scalar; rewriting it by text
    would corrupt the node, so the helper declines and the caller falls back
    to a post-import field write."""
    ns = _supervisor_helpers()
    src = 'Solid { boundingObject Box { size 1 1 1 } name "s" }'
    out, ok = ns["replace_top_level_field"](src, "boundingObject", "NULL")
    assert ok is False
    assert out == src


def test_def_in_vrml():
    ns = _supervisor_helpers()
    assert ns["def_in_vrml"]("DEF HUSKY_1 Robot { }") == "HUSKY_1"
    assert ns["def_in_vrml"]("Robot { }") is None


# ---------------------------------------------------------------------------
# The spawn composer's field detection must be depth-aware too
# ---------------------------------------------------------------------------


BOUNDING_OBJECT_SPAWN = ('Solid { name "crate" '
                         'boundingObject Pose { translation 0 0 0.5 '
                         'children [ Box { size 1 1 1 } ] } }')


def test_spawn_translation_survives_a_boundingObject_that_mentions_it():
    """The measured drop: `boundingObject Pose { translation 0 0 0.5 ... }`
    contains the substring "translation", so the old `"translation" not in
    body` test believed the node already carried the field and silently
    discarded the caller's. Rotation was applied, translation was not, and
    because the supervisor's field-write fallback is clone-only nothing
    recovered it -- the only trace was verification.pose_delta_m = 3.162."""
    from omnisim_harness import compose_spawn_vrml
    vrml, _ = compose_spawn_vrml(
        {"def": "CRATE", "vrml": BOUNDING_OBJECT_SPAWN,
         "translation": [3, 0, 1], "rotation": [0, 0, 1, 0.5]}, REPO_ROOT)
    assert "translation 3 0 1" in vrml
    assert "rotation 0 0 1 0.5" in vrml
    # the boundingObject's own nested translation is untouched
    assert "translation 0 0 0.5" in vrml


def test_top_level_field_names_ignores_nested_fields():
    from omnisim_harness import top_level_field_names
    names = top_level_field_names(BOUNDING_OBJECT_SPAWN)
    assert "name" in names
    assert "boundingObject" in names
    assert "translation" not in names      # it belongs to the nested Pose
    assert "size" not in names             # ...and this one to the Box
    assert top_level_field_names('Solid { translation 9 9 9 }') >= {"translation"}
    assert top_level_field_names("not a node") == set()


def test_top_level_field_names_agrees_with_the_supervisors_own_scan():
    """The harness and the supervisor run in different processes and cannot
    share a module, so the two copies of this scan are pinned to each other
    here: whatever `replace_top_level_field` is willing to rewrite is exactly
    what `top_level_field_names` must report as present."""
    from omnisim_harness import top_level_field_names
    ns = _supervisor_helpers()
    for sample in (CLONE_SAMPLE, BOUNDING_OBJECT_SPAWN,
                   'Solid { translation 9 9 9 }',
                   'Robot { name "a" children [ Solid { name "b" } ] }'):
        names = top_level_field_names(sample)
        for field in ("translation", "rotation", "name", "controller"):
            _out, replaced = ns["replace_top_level_field"](sample, field, "X")
            if replaced:
                assert field in names, f"{field!r} rewritable but not reported"


# ---------------------------------------------------------------------------
# scene_delete's `all_removed` must mean what it says
# ---------------------------------------------------------------------------


def test_all_removed_is_false_when_the_def_never_existed():
    """`still` is derived from `removed`, so a request naming only DEFs that do
    not exist removed nothing, had nothing to re-resolve, and used to report
    all_removed: true. POST /scene/delete {"def":"TYPO"} -> 200, removed [],
    missing ["TYPO"], all_removed true -- a false confirmation for a typo."""
    ns = _supervisor_helpers()
    out = ns["delete_verification"]([], ["TYPO"], [])
    assert out["all_removed"] is False
    assert "did not exist" in out["all_removed_reason"]
    assert out["removed_count"] == 0 and out["missing_count"] == 1


def test_all_removed_is_false_for_a_partial_delete():
    ns = _supervisor_helpers()
    out = ns["delete_verification"]([{"def": "A"}], ["B"], [])
    assert out["all_removed"] is False


def test_all_removed_is_false_when_a_removed_def_still_resolves():
    ns = _supervisor_helpers()
    out = ns["delete_verification"]([{"def": "A"}], [], ["A"])
    assert out["all_removed"] is False
    assert out["still_resolves"] == ["A"]


def test_all_removed_is_true_only_for_a_complete_delete():
    ns = _supervisor_helpers()
    out = ns["delete_verification"]([{"def": "A"}, {"def": "B"}], [], [])
    assert out["all_removed"] is True
    assert "all_removed_reason" not in out


# ---------------------------------------------------------------------------
# Reset side effects (damage state + inject schedule)
# ---------------------------------------------------------------------------


class _FakeTracker:
    def __init__(self, name, boom=False):
        self.robot_name = name
        self.boom = boom
        self.resets = 0

    def reset(self):
        if self.boom:
            raise RuntimeError("tracker exploded")
        self.resets += 1


def test_rearm_after_reset_clears_every_tracker_and_reports_what_it_did():
    """The `reset` command reports advanced_to_ms, so the main loop pulls the
    rewound clock in itself and the backwards-jump detector -- the ONLY thing
    that used to clear damage state and rearm the inject cursor -- can no
    longer fire for it. This is the explicit call that replaces it."""
    ns = _supervisor_helpers()
    a, b = _FakeTracker("husky_0"), _FakeTracker("husky_1")
    out = ns["rearm_after_reset"]((a, b, None), "reset command")
    assert a.resets == 1 and b.resets == 1
    assert out["inject_schedule_rearmed"] is True
    assert out["damage_trackers_reset"] == ["husky_0", "husky_1"]
    assert out["errors"] == []


def test_rearm_after_reset_survives_one_broken_tracker():
    ns = _supervisor_helpers()
    a, bad, c = _FakeTracker("a"), _FakeTracker("bad", boom=True), _FakeTracker("c")
    lines: list = []
    out = ns["rearm_after_reset"]((a, bad, c), "sim reset detected",
                                  log=lines.append)
    assert a.resets == 1 and c.resets == 1        # one failure stops nothing
    assert out["damage_trackers_reset"] == ["a", "c"]
    assert out["errors"] and "tracker exploded" in out["errors"][0]
    assert lines and "sim reset detected" in lines[0]


# ---------------------------------------------------------------------------
# Snapshot verification maths
# ---------------------------------------------------------------------------


def test_compare_fingerprints_reports_the_worst_node_and_exactness():
    ns = _supervisor_helpers()
    before = {"A": [0.0, 0.0, 0.0], "B": [1.0, 0.0, 0.0], "GONE": [0, 0, 0]}
    after = {"A": [0.0, 0.0, 0.0], "B": [1.0, 0.0, 2.0], "NEW": [0, 0, 0]}
    out = ns["compare_fingerprints"](before, after)
    assert out["sampled_nodes"] == 2
    assert out["max_pose_delta_m"] == 2.0
    assert out["max_pose_delta_node"] == "B"
    assert out["exact"] is False
    assert out["missing"] == ["GONE"]
    assert out["added"] == ["NEW"]


def test_compare_fingerprints_exact_when_identical():
    ns = _supervisor_helpers()
    poses = {"A": [1.0, 2.0, 3.0]}
    out = ns["compare_fingerprints"](poses, dict(poses))
    assert out["max_pose_delta_m"] == 0.0
    assert out["exact"] is True


def test_compare_fingerprints_with_no_overlap_is_honest_about_it():
    ns = _supervisor_helpers()
    out = ns["compare_fingerprints"]({"A": [0, 0, 0]}, {"B": [0, 0, 0]})
    assert out["sampled_nodes"] == 0
    assert out["max_pose_delta_m"] is None
    assert out["exact"] is None


# ---------------------------------------------------------------------------
# /capabilities self-cross-checks
# ---------------------------------------------------------------------------


def test_declared_routes_match_the_request_handler_source():
    """`endpoints_verification.verified` must be true for the shipped code:
    a route added to do_GET/do_POST without a ROUTES entry (or vice versa) is
    exactly the drift that left eight endpoints out of PROTOCOL.md §7."""
    import omnisim_harness as h
    source = (HARNESS_DIR / "omnisim_harness.py").read_text(encoding="utf-8")
    result = h.verify_routes(source)
    assert result["declared_not_found_in_source"] == []
    assert result["undeclared_literals"] == []
    assert result["verified"] is True


def test_route_table_is_well_formed():
    import omnisim_harness as h
    for route in h.ROUTES:
        assert route["method"] in ("GET", "POST")
        assert route["path"].startswith("/")
        assert route["summary"]
    paths = [(r["method"], r["path"]) for r in h.ROUTES]
    assert len(paths) == len(set(paths))


def test_log_event_types_match_their_emitters():
    import omnisim_harness as h
    source = (HARNESS_DIR / "omnisim_harness.py").read_text(encoding="utf-8")
    result = h.verify_log_event_types(source)
    assert result["verified"] is True
    assert set(result["types"]) == {"controller.log", "world.warning", "world.error"}


def test_supervisor_event_types_match_their_emitters():
    """The supervisor's seven types, cross-checked against the emit() calls in
    event_bus.py + harness_supervisor.py — the same check the live
    `capabilities` RPC runs."""
    import event_bus
    sources = [
        (SUPERVISOR_DIR / "event_bus.py").read_text(encoding="utf-8"),
        (SUPERVISOR_DIR / "harness_supervisor.py").read_text(encoding="utf-8"),
    ]
    result = event_bus.verify_event_types(*sources)
    assert result["undeclared"] == []
    assert result["declared_not_emitted"] == []
    assert result["verified"] is True
    assert len(result["types"]) == 7


def test_ten_event_types_total():
    """PROTOCOL.md §10.1 says exactly ten. This is that claim, as a test."""
    import event_bus
    import omnisim_harness as h
    assert len(set(event_bus.SUPERVISOR_EVENT_TYPES) | set(h.LOG_EVENT_TYPES)) == 10


def test_every_event_type_has_a_named_producer():
    import event_bus
    assert set(event_bus.EVENT_TYPE_PRODUCERS) == set(event_bus.SUPERVISOR_EVENT_TYPES)
    for producer in event_bus.LIGHT_MODE_DISABLED_PRODUCERS:
        assert producer in set(event_bus.EVENT_TYPE_PRODUCERS.values())


# ---------------------------------------------------------------------------
# Boolean environment parsing (light mode)
# ---------------------------------------------------------------------------


def test_env_flag_off_means_off(monkeypatch):
    """`bool(os.environ.get(...))` is TRUE for the string "0", so
    OMNISIM_HARNESS_LIGHT=0 -- the obvious way to turn light mode OFF -- turned
    it ON, silently costing /sim/grips and every contact / joint-limit / grip
    event."""
    import omnisim_harness as h
    for raw in ("0", "false", "FALSE", "No", " 0 ", ""):
        monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", raw)
        assert h.env_flag("OMNISIM_HARNESS_LIGHT") is False, raw


def test_env_flag_on_means_on(monkeypatch):
    import omnisim_harness as h
    for raw in ("1", "true", "TRUE", "yes", "on"):
        monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", raw)
        assert h.env_flag("OMNISIM_HARNESS_LIGHT") is True, raw


def test_env_flag_unset_takes_the_default(monkeypatch):
    import omnisim_harness as h
    monkeypatch.delenv("OMNISIM_HARNESS_LIGHT", raising=False)
    assert h.env_flag("OMNISIM_HARNESS_LIGHT") is False
    assert h.env_flag("OMNISIM_HARNESS_LIGHT", default=True) is True


def test_harness_state_light_default_honours_a_falsey_env(monkeypatch, tmp_path):
    """The end-to-end shape of the same bug: the field the injected supervisor's
    --light flag comes from."""
    import omnisim_harness as h
    monkeypatch.setattr(h, "resolve_omnisim_binary", lambda home: tmp_path / "fake-bin")
    monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", "0")
    assert h.HarnessState(tmp_path).light_supervisor is False
    monkeypatch.setenv("OMNISIM_HARNESS_LIGHT", "1")
    assert h.HarnessState(tmp_path).light_supervisor is True


# ---------------------------------------------------------------------------
# Transparent-retry eligibility (a read that mutates is not a read)
# ---------------------------------------------------------------------------


def test_sim_contacts_is_retryable_with_or_without_wake():
    """?wake=1 USED TO rewrite WorldInfo.physicsDisableTime and advance the sim
    by settle_steps, which made a documented read replay-unsafe.

    It is a no-op as of 2026-08-08: there is no body sleep in this engine (ODE is
    deleted and `physicsDisableTime` has no reader), so the steps bought no
    information -- they mutated the world during a read and measured nothing.
    Removing them makes `sim_contacts` a pure read again with or without the
    parameter, which is the point of this test: the read is retryable, and the
    argument-sensitive carve-out is gone rather than merely re-worded."""
    import omnisim_harness as h
    assert h.is_retryable_supervisor_call("sim_contacts", {}) is True
    assert h.is_retryable_supervisor_call("sim_contacts", None) is True
    assert h.is_retryable_supervisor_call("sim_contacts", {"wake": True}) is True
    assert h.is_retryable_supervisor_call(
        "sim_contacts", {"wake": True, "settle_steps": 4}) is True


def test_mutating_commands_are_never_retryable():
    import omnisim_harness as h
    for cmd in ("scene_spawn", "scene_delete", "scene_set_pose", "scene_set_poses", "step",
                "reset", "world_load", "sim_snapshot", "sim_restore"):
        assert h.is_retryable_supervisor_call(cmd) is False, cmd
    for cmd in ("ping", "scene_tree", "robots_list", "sim_grips", "capabilities"):
        assert h.is_retryable_supervisor_call(cmd) is True, cmd


# ---------------------------------------------------------------------------
# Light mode publishes what it actually breaks
# ---------------------------------------------------------------------------


def test_light_mode_does_not_claim_sim_contacts_is_dead():
    """/sim/contacts is served by observe.collect_contacts, which walks the
    scene per call and never touches ContactTracker -- so declaring it
    unsupported in light mode was false, and false in the expensive direction
    (the workaround recommended a ~790x-cost reload nobody needed)."""
    import inspect
    import omnisim_harness as h
    src = inspect.getsource(h.HarnessState.capabilities)
    entry = src[src.index("if light:"):]
    assert '"feature": "sim.grips + events.contact/grip/joint"' in entry
    assert "sim.contacts +" not in entry
    assert "UNAFFECTED" in entry


def test_sim_contacts_is_not_advertised_as_empty_in_light_mode():
    import omnisim_harness as h
    routes = {(r["method"], r["path"]): r for r in h.ROUTES}
    contacts = routes[("GET", "/sim/contacts")]["summary"]
    assert "Empty in light mode" not in contacts
    assert "light mode" in contacts
    grips = routes[("GET", "/sim/grips")]["summary"]
    assert "tracking.enabled=false" in grips


def test_def_taken_is_a_conflict_not_a_503():
    """A spawn onto an existing DEF used to mutate one node and report another
    (getFromDef answers with the FIRST match, and the importer does not rename
    a duplicate). It is now refused with a branchable code."""
    import omnisim_harness as h
    codes = {code: (needle, status)
             for needle, status, code in h.SUPERVISOR_ERROR_CODE_MAP}
    assert "DEF_TAKEN" in codes
    needle, status = codes["DEF_TAKEN"]
    assert status == 409
    supervisor_src = (SUPERVISOR_DIR / "harness_supervisor.py").read_text(encoding="utf-8")
    assert needle in supervisor_src          # the map still matches the raiser
    assert "DEF_TAKEN" in h.known_request_error_codes()


def test_diagnostic_and_request_error_code_sets_are_disjoint_and_populated():
    import omnisim_harness as h
    load_codes = h.known_diagnostic_codes()
    request_codes = h.known_request_error_codes()
    assert "WORLD_FILE_NOT_FOUND" in load_codes
    assert "UNKNOWN" in load_codes
    for code in h.HARNESS_DIAGNOSTIC_CODES:
        assert code in load_codes
    assert "SPAWN_REJECTED" in request_codes
    assert "DEF_NOT_FOUND" in request_codes
    assert not set(load_codes) & set(request_codes)


# ---------------------------------------------------------------------------
# Backend attribution (physics block)
# ---------------------------------------------------------------------------


def test_newton_verdict_prefers_the_sidecar(tmp_path):
    import omnisim_harness as h
    log = tmp_path / "engine.log"
    log.write_text("INFO: whatever\n", encoding="utf-8")
    (tmp_path / "engine.log.newton.json").write_text(json.dumps({
        "backend": "newton", "solver": "MuJoCo (cpu/mj_step)",
        "degraded": False, "finalised": True}), encoding="utf-8")
    out = h.read_newton_verdict(log)
    assert out["backend"] == "newton"
    assert out["source"] == "sidecar"
    assert out["solver"].startswith("MuJoCo")
    assert out["degraded"] is False


@pytest.mark.parametrize("tag", ["Wb", "Om"])
def test_newton_verdict_falls_back_to_the_finalise_line(tmp_path, tag):
    """The scrape must read the WHOLE log: a tail-only read used to miss the
    load-time line and falsely report ODE (fixed in ad9fff48).

    Parametrized over the tag prefix: the bracketed tag is named after the
    emitting C++ class and those classes are being renamed Wb* -> Om*, so the
    scrape dual-accepts both permanently. The "Om" case goes red if anyone
    narrows it back to one prefix.
    """
    import omnisim_harness as h
    log = tmp_path / "engine.log"
    log.write_text(
        f"INFO: [{tag}NewtonBackend] world finalised (solver=XPBD(iters=10))\n"
        + "INFO: filler\n" * 5000, encoding="utf-8")
    out = h.read_newton_verdict(log)
    assert out["backend"] == "newton"
    assert out["source"] == "engine_log"
    # The solver value must still be sliced out exactly as before: rest of the
    # LINE with the single trailing ')' dropped, nested parens preserved.
    assert out["solver"] == "XPBD(iters=10)"


def test_newton_verdict_ignores_an_unknown_tag_prefix(tmp_path):
    """NEGATIVE CONTROL: the scrape dual-accepts Wb|Om, not ANY prefix."""
    import omnisim_harness as h
    log = tmp_path / "engine.log"
    log.write_text(
        "INFO: [XxNewtonBackend] world finalised (solver=XPBD(iters=10))\n",
        encoding="utf-8")
    out = h.read_newton_verdict(log)
    assert out["source"] != "engine_log", (
        "an [XxNewtonBackend] line was accepted -- the tag pattern is too loose")


def test_newton_verdict_absent_sidecar_says_it_proves_nothing(tmp_path):
    """A missing sidecar means UNVERIFIED -- never "ode".

    Newton is the only backend since bdc02139, so "no sidecar" cannot mean some
    other engine drove the world; it means the run never reached world finalize
    (or the runtime failed to come up). This test used to assert
    backend == "ode", which was the honest answer only while ODE shipped.
    """
    import omnisim_harness as h
    log = tmp_path / "engine.log"
    log.write_text("INFO: a run that never reached finalize\n", encoding="utf-8")
    out = h.read_newton_verdict(log)
    assert out["source"] == "sidecar_absent"
    assert out["backend"] == "unverified", (
        "a missing sidecar must not name a backend -- least of all a deleted one")
    assert "never reached world finalize" in out["detail"]
    assert "ode" not in out["backend"].lower()


def test_newton_verdict_reports_a_forced_ode_env(tmp_path, monkeypatch):
    """A set OMNISIM_FORCE_ODE is still REPORTED -- and now flagged as retired.

    The var has to keep being surfaced precisely BECAUSE it no longer works: the
    engine ignores it (verified 2026-08-08 -- the run comes up on Newton and
    writes a normal sidecar), so an agent reading /capabilities would otherwise
    believe it had asked for and received ODE. src/ode was DELETED (bdc02139), so
    the report must not read as a working ODE arm.
    """
    import omnisim_harness as h
    monkeypatch.setenv("OMNISIM_FORCE_ODE", "1")
    log = tmp_path / "engine.log"
    log.write_text("INFO: forced\n", encoding="utf-8")
    out = h.read_newton_verdict(log)
    # The var is REPORTED (so an agent learns it is set and being ignored) but
    # the backend field must state what actually ran. Reporting "ode" here while
    # the detail text said the opposite was the one self-contradicting object in
    # the surface -- and `backend` is the field an agent branches on.
    assert out["backend"] == "newton"
    assert out["source"] == "retired_selector_ignored"
    assert "IGNORED" in out["detail"] or "ignored" in out["detail"]
    assert out["forced_ode_env"] == "OMNISIM_FORCE_ODE"
    retired = out["forced_ode_env_retired"]
    assert "RETIRED" in retired
    assert "bdc02139" in retired, "the retirement note must cite the deletion commit"
    assert "does NOT give you an ODE run" in retired, \
        "the note must say what actually happens, not just that the var is old"


def test_newton_verdict_flags_a_degraded_solver(tmp_path):
    import omnisim_harness as h
    log = tmp_path / "engine.log"
    log.write_text("x\n", encoding="utf-8")
    (tmp_path / "engine.log.newton.json").write_text(json.dumps({
        "backend": "newton", "solver": "XPBD fallback", "degraded": True,
        "finalised": True}), encoding="utf-8")
    out = h.read_newton_verdict(log)
    assert out["degraded"] is True


# ---------------------------------------------------------------------------
# Step-cost telemetry (limits.step_cost)
# ---------------------------------------------------------------------------


class _FakeState:
    """Just enough HarnessState for the step-cost maths."""

    def __init__(self):
        import collections
        import threading
        self.lock = threading.Lock()
        self.step_samples = collections.deque(maxlen=32)

    note_step = None  # bound below


def test_step_cost_is_a_median_per_step_and_none_when_unmeasured():
    import omnisim_harness as h
    state = _FakeState()
    state.note_step = h.HarnessState.note_step.__get__(state)
    state.step_cost = h.HarnessState.step_cost.__get__(state)
    assert state.step_cost() is None
    state.note_step(10, 1.0)   # 0.1 s/step
    state.note_step(1, 0.3)    # 0.3 s/step
    state.note_step(2, 0.4)    # 0.2 s/step
    cost = state.step_cost()
    assert cost["samples"] == 3
    assert cost["median_s_per_step"] == pytest.approx(0.2)
    assert cost["min_s_per_step"] == pytest.approx(0.1)
    assert cost["max_s_per_step"] == pytest.approx(0.3)


def test_step_cost_ignores_nonsense_samples():
    import omnisim_harness as h
    state = _FakeState()
    state.note_step = h.HarnessState.note_step.__get__(state)
    state.step_cost = h.HarnessState.step_cost.__get__(state)
    state.note_step(0, 1.0)
    state.note_step(-3, 1.0)
    assert state.step_cost() is None


# ---------------------------------------------------------------------------
# Runtime scene mutation disclosure (internal parity plan, item W1.7 honest interim)
# ---------------------------------------------------------------------------
#
# Measured 2026-08-17: the Newton/MuJoCo model is frozen at finalizeWorld(),
# so /scene/spawn adds a node the solver never sees and /scene/delete leaves
# phantom colliders in it -- both with 0 errors, 0 warnings engine-side.
# Until the engine fix lands, every successful spawn/delete response must
# carry `physics_warning`, and the first use per verb per world-load must put
# one world.warning into the event ring. These lock the pure halves down; the
# live behaviour is verified against a running engine (scripts/harness/README.md).


class _MutationState:
    """Just enough HarnessState for runtime_mutation_warning."""

    def __init__(self):
        import threading
        import omnisim_harness as h
        self.lock = threading.Lock()
        self.log_buffer = h.LogRingBuffer()
        self._runtime_mutation_warned = set()


def _mutation_state():
    import omnisim_harness as h
    state = _MutationState()
    state.runtime_mutation_warning = h.HarnessState.runtime_mutation_warning.__get__(state)
    return state


def test_physics_warning_block_shape_and_code():
    import omnisim_harness as h
    state = _mutation_state()
    for verb, constant in (("spawn", h.SPAWN_PHYSICS_WARNING),
                           ("delete", h.DELETE_PHYSICS_WARNING)):
        block = state.runtime_mutation_warning(verb)
        assert block["code"] == "RUNTIME_MUTATION_NOT_IN_SOLVER"
        assert block == constant
        assert block is not constant  # a copy: callers must not mutate the module constant
        # The message must carry the actionable half, not just the defect.
        assert "/world/load" in block["message"]
        assert "frozen" in block["message"]


def test_one_world_warning_per_verb_per_load_not_per_request():
    state = _mutation_state()
    for _ in range(3):
        state.runtime_mutation_warning("spawn")
    events = state.log_buffer.since(0, 100)
    assert len(events) == 1
    evt = events[0]
    assert evt["type"] == "world.warning"
    assert evt["code"] == "RUNTIME_MUTATION_NOT_IN_SOLVER"
    assert "/scene/spawn" in evt["message"]
    # The other verb gets its OWN single warning.
    state.runtime_mutation_warning("delete")
    state.runtime_mutation_warning("delete")
    events = state.log_buffer.since(0, 100)
    assert len(events) == 2
    assert "/scene/delete" in events[1]["message"]
    # A reload rebuilds the solver, so the flags clear and the warning re-fires.
    state._runtime_mutation_warned.clear()
    state.runtime_mutation_warning("spawn")
    assert len(state.log_buffer.since(0, 100)) == 3


def test_load_paths_clear_the_mutation_warned_flags():
    """Both load sites (cold and hot) must reset the once-per-load latch,
    exactly where they already reset the per-world step-cost telemetry."""
    import inspect
    import omnisim_harness as h
    src = inspect.getsource(h.HarnessState)
    assert src.count("self._runtime_mutation_warned.clear()") == 2
    # And each clear sits next to the step-cost clear (same per-world scope).
    for chunk in src.split("self.step_samples.clear()")[1:]:
        assert "_runtime_mutation_warned.clear()" in chunk[:200]


def test_spawn_and_delete_handlers_attach_physics_warning():
    """The field must ride EVERY success path: spawn (all input forms funnel
    through the single 200 exit) and delete."""
    source = (HARNESS_DIR / "omnisim_harness.py").read_text(encoding="utf-8")
    spawn = source[source.index('if path == "/scene/spawn":'):
                   source.index('if path == "/scene/delete":')]
    assert 'state.runtime_mutation_warning("spawn")' in spawn
    delete = source[source.index('if path == "/scene/delete":'):
                    source.index('if path == "/scene/set_pose":')]
    assert 'state.runtime_mutation_warning("delete")' in delete


def test_capabilities_advertises_the_runtime_mutation_gap():
    import omnisim_harness as h
    routes = {(r["method"], r["path"]): r for r in h.ROUTES}
    for path in ("/scene/spawn", "/scene/delete"):
        summary = routes[("POST", path)]["summary"]
        assert "physics_warning" in summary
        assert "RUNTIME_MUTATION_NOT_IN_SOLVER" in summary
    features = [e["feature"] for e in h.ENGINE_NOT_SUPPORTED]
    assert any("scene.runtime_mutation_physics" in f for f in features)


def test_runtime_mutation_code_is_not_a_request_error_code():
    """It rides a 200 body (the request SUCCEEDED at the scene-graph layer),
    so it must not leak into the 4xx request-error enum."""
    import omnisim_harness as h
    assert "RUNTIME_MUTATION_NOT_IN_SOLVER" not in h.known_request_error_codes()
