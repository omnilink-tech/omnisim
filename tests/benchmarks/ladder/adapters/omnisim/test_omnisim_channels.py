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

"""The OmniSim column's T2/T3/T4 channels, with no engine and no GPU.

Everything here runs off a synthetic tier document in the shape the
grader-owned sampler writes. That is the whole point of keeping
``channels.py`` pure: the contract a cell rests on can be exercised in
milliseconds, and a regression in it does not need a simulator to find.

The fixture is a **miniature of the measured T3 probe** -- one robot whose
root body carries the scene's name and not the URDF root link's, a declared
floor, a wall, per-link child bodies with masses that sum to the robot's
declared mass, and a contact record with both sides named -- because every
one of those was a real reading and a fixture that disagrees with the engine
tests nothing.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest

BENCHMARKS = Path(__file__).resolve().parents[3]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from ladder import adapters as ladder_adapters               # noqa: E402
from ladder.adapters.omnisim import channels as ch           # noqa: E402
from ladder.adapters.omnisim import evidence as oev          # noqa: E402
from ladder.graders import ladder_evidence as lev            # noqa: E402
from ladder.graders import t3 as t3_shim                     # noqa: E402
from ladder.graders import t3_evidence as t3ev               # noqa: E402
from ladder.graders import t4 as t4_shim                     # noqa: E402
from ladder.graders import t4_evidence as t4ev               # noqa: E402

N = 8
GROUND_NAMES = ("ground", "floor", "terrain", "ground_plane", "plane",
                "flat_ground", "arena", "arena_floor")


def _identity_rot():
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def _body(name, ident, *, robot=False, depth=1, x0=0.0, owner=None,
          sampler=False):
    return {"name": name, "def": "", "id": ident, "type":
            "Robot" if robot else "Solid", "depth": depth,
            "robot_class": robot, "is_grader_sampler": sampler,
            "owner_robot": owner,
            "xyz": [[x0 + 0.1 * i, 0.0, 0.5] for i in range(N)],
            "rot": [_identity_rot() for _ in range(N)]}


def _inv(name, ident, *, robot=False, depth=1, static=False, mass=None,
         subtree=None, lo=None, hi=None, owner=None, sampler=False,
         mass_error=None):
    return {"name": name, "def": "", "id": ident,
            "type": "Robot" if robot else "Solid", "depth": depth,
            "robot_class": robot, "is_grader_sampler": sampler,
            "ident": "#%d" % ident, "owner_robot": owner,
            "has_physics": not static, "static": static,
            "mass_kg": mass, "mass_error": mass_error,
            "subtree_mass_kg": subtree, "subtree_bodies_with_mass": 1,
            "subtree_bodies_mass_unreadable": 0, "connectors_in_subtree": 0,
            "aabb_min": lo, "aabb_max": hi, "bounds_error": None,
            "bounds_exact": True, "position": [0.0, 0.0, 0.0],
            "parent": None}


@pytest.fixture()
def doc():
    times = [round(0.032 * i, 6) for i in range(N)]
    contacts = []
    for step, t in enumerate(times):
        # A foot on the ground on every scan, alternating feet, so the core
        # can see make-and-break.
        foot = "shank_fl" if step % 2 == 0 else "shank_fr"
        contacts.append({"a": "GROUND", "b": "#%d" % (110 + step % 2),
                         "a_name": "ground", "b_name": foot,
                         "a_robot": None, "b_robot": "#15",
                         "a_is_ground": True, "b_is_ground": False,
                         "point": [0.0, 0.0, 0.05], "t_s": t, "step": step,
                         "paired": True})
    return {
        "tier": "t3", "schema": ch.SCHEMA, "basic_timestep_ms": 16.0,
        "record_stride_steps": 2, "contact_stride_steps": 2,
        "window_s": times[-1], "t_s": times,
        "pose_source": "POSE",
        "inventory_source": "INVENTORY", "mass_source": "MASS",
        "arena_source": "ARENA", "declared_surfaces": list(GROUND_NAMES),
        "inventory_frozen": True,
        "bodies": [
            _body("walker", 15, robot=True, depth=1, owner="#15"),
            _body("shank_fl", 110, depth=4, owner="#15"),
            _body("shank_fr", 111, depth=4, owner="#15"),
            _body("ground", 5, depth=1),
            _body("wall_north", 6, depth=1),
            _body("ladder_recorder", 173, robot=True, depth=1, sampler=True,
                  owner="#173"),
        ],
        "inventory": [
            _inv("walker", 15, robot=True, mass=6.0, subtree=10.8,
                 owner="#15", lo=[-0.3, -0.3, 0.1], hi=[0.3, 0.3, 0.6]),
            _inv("shank_fl", 110, depth=4, mass=0.25, subtree=0.25,
                 owner="#15", lo=[-0.1, -0.1, 0.0], hi=[0.1, 0.1, 0.2]),
            _inv("shank_fr", 111, depth=4, mass=0.25, subtree=0.25,
                 owner="#15", lo=[-0.1, -0.1, 0.0], hi=[0.1, 0.1, 0.2]),
            _inv("ground", 5, static=True, lo=[-20.0, -20.0, -0.05],
                 hi=[20.0, 20.0, 0.05]),
            _inv("wall_north", 6, static=True, lo=[-20.0, 19.9, 0.0],
                 hi=[20.0, 20.1, 2.0]),
            _inv("ladder_recorder", 173, robot=True, static=True,
                 sampler=True, owner="#173"),
        ],
        "world": {"gravity_vec_mps2": [0.0, 0.0, -9.81], "gravity_mps2": 9.81,
                  "source": "WORLDINFO", "error": None},
        "contacts": {"supported": True, "steps": N, "window_s": times[-1],
                     "total_observed": len(contacts),
                     "distinct_named": len(contacts),
                     "sample_times": times, "emit_stride_steps": 2,
                     "source": "CONTACTS", "error": None,
                     "pairs": contacts},
        "structure": {"foreign_supervisors": [],
                      "our_sampler": "ladder_recorder",
                      "robots_without_physics": [],
                      "robots_parented_into_a_body": [],
                      "robots_with_connectors": [],
                      "physics_plugin": None, "routes_open": [],
                      "attested": True, "source": "STRUCTURE"},
        "controllers": [{"robot": "walker", "controller": "gait",
                         "declared": True, "joints": 12, "joints_moved": 12,
                         "loaded": True, "evidence": "EVIDENCE",
                         "source": "CONTROLLER"}],
        "errors": [],
    }


def _surface():
    return t3ev.WalkingSurface(names=GROUND_NAMES, source="task")


def _roles(obj="walker", eff="shank_fl", con="ground"):
    return lev.RoleNames(object_name=obj, end_effector_name=eff,
                         container_name=con, source="task")


# --- the hook contract --------------------------------------------------------


def test_every_hook_the_shims_look_up_by_name_exists():
    for name in ("build_bundle", "run_standalone", "t1_run_standalone",
                 "t2_run_standalone", "t3_run_standalone",
                 "t4_run_standalone", "support_observation", "t2_channels",
                 "t3_channels", "t4_channels"):
        assert hasattr(oev, name), name


def test_the_column_is_registered_and_resolves():
    mod = ladder_adapters.resolve_ladder_channels("omnisim")
    assert mod is oev


def test_channel_key_tuples_match_what_each_shim_reads():
    assert set(oev.T2_CHANNEL_KEYS) == set(ladder_adapters.T2_CHANNELS)
    assert set(oev.T3_CHANNEL_KEYS) == set(t3_shim.T3_CHANNELS)
    assert set(oev.T4_CHANNEL_KEYS) == set(t4_shim.T4_CHANNELS)


def test_hooks_return_empty_dict_when_there_was_no_run(tmp_path):
    # A key omitted means "unmeasured"; a key supplied empty means "measured
    # and there was nothing". With no run at all the honest answer is the
    # former for every key, i.e. {}.
    assert oev.t2_channels(None, roles=_roles()) == {}
    assert oev.t3_channels(None, surface=_surface()) == {}
    assert oev.t4_channels(str(tmp_path), surface=_surface()) == {}


def test_hooks_never_raise_on_rubbish():
    for bad in (object(), 17, {"schema": "wrong"}, b"\x00"):
        assert oev.t2_channels(bad, roles=_roles()) in ({}, )
        assert oev.t3_channels(bad, surface=_surface()) in ({}, )


def test_t1_run_standalone_pins_the_cheap_tier_and_the_generic_one_does_not():
    seen = {}

    class _Res:
        error = None
        t = [0.0, 1.0]
        xyz = [[0.0, 0.0, 0.0]]
        rc = 0
        timed_out = False
        recorder_complete = True

    def fake(world, run_dir, **kw):
        seen.update(kw)
        return _Res()

    real = oev._run_once                       # noqa: SLF001
    try:
        oev._run_once = fake                   # noqa: SLF001
        oev.t1_run_standalone("w", "d", attempts=1)
        assert seen["tier"] == "t1"
        seen.clear()
        oev.run_standalone("w", "d", attempts=1)
        # ladder.graders.t2 calls the GENERIC name and nothing else, so the
        # generic name must produce the tier document or a T2 verdict blames
        # our scaffolding for channels the run could have answered.
        assert seen["tier"] == "all"
        assert seen["record_stride"] == oev.RECORD_STRIDE
    finally:
        oev._run_once = real                   # noqa: SLF001


def test_the_sampler_stanza_carries_the_tier_and_the_stride(tmp_path):
    world = tmp_path / "w.wbt"
    world.write_text("WorldInfo {\n}\n", encoding="utf-8")
    sib = oev.inject_sampler(world, tmp_path / "out.csv", duration=1.0,
                             settle=0.5, stride=2, tier="t3",
                             record_stride=4)
    text = sib.read_text(encoding="utf-8")
    assert '"--tier=t3"' in text
    assert '"--record-stride=4"' in text


# --- the document schema ------------------------------------------------------


def test_check_document_passes_a_well_formed_document(doc):
    assert ch.check_document(doc) == []


@pytest.mark.parametrize("mutate,fragment", [
    (lambda d: d.update(schema="nope"), "schema"),
    (lambda d: d.update(t_s=[0.0]), "t_s"),
    (lambda d: d.update(inventory=[]), "inventory"),
    (lambda d: d["structure"].update(attested=None), "structural"),
    (lambda d: d["world"].update(gravity_mps2=None), "gravity"),
    (lambda d: d["contacts"].update(sample_times=[]), "query times"),
    (lambda d: d["bodies"][0].update(xyz=[[0, 0, 0]]), "pose samples"),
])
def test_check_document_names_each_defect(doc, mutate, fragment):
    d = copy.deepcopy(doc)
    mutate(d)
    assert any(fragment in p for p in ch.check_document(d)), \
        ch.check_document(d)


# --- selection ----------------------------------------------------------------


def test_a_declared_name_wins_over_the_structural_fallback(doc):
    d = copy.deepcopy(doc)
    d["inventory"][0]["name"] = "base_link"
    rec, note, err = ch.resolve_base(d["inventory"], "base_link")
    assert err is None and note == "" and rec["name"] == "base_link"


def test_the_single_robot_substitutes_when_the_declared_name_matches_nothing(
        doc):
    rec, note, err = ch.resolve_base(doc["inventory"], "base_link")
    assert err is None
    assert rec["name"] == "walker"
    # The substitution must travel in the citation, never silently.
    assert "SUBSTITUTION" in note and "base_link" in note and "walker" in note


def test_the_graders_own_sampler_is_never_a_substitute(doc):
    d = copy.deepcopy(doc)
    d["inventory"] = [r for r in d["inventory"] if r["name"] != "walker"]
    rec, _note, err = ch.resolve_base(d["inventory"], "base_link")
    assert rec is None and "no robot-class body" in err


def test_two_candidate_robots_refuse_rather_than_pick_the_luckiest(doc):
    d = copy.deepcopy(doc)
    d["inventory"].append(_inv("strider", 99, robot=True, owner="#99"))
    rec, _note, err = ch.resolve_base(d["inventory"], "base_link")
    assert rec is None
    assert "no single unambiguous substitute" in err
    assert "strider" in err and "walker" in err


def test_a_duplicated_name_takes_the_shallowest_and_names_the_rest(doc):
    d = copy.deepcopy(doc)
    d["bodies"].append(_body("walker", 900, depth=7, owner="#15"))
    s = ch.pose_series(d, "walker")
    assert s.body == "walker"
    assert "WARNING" in s.source and "900" in s.source


# --- the channels themselves --------------------------------------------------


def test_pose_series_carries_a_world_from_body_rotation(doc):
    s = ch.pose_series(doc, "walker")
    assert s.usable and s.has_orientation
    assert s.rot.shape == (N, 3, 3)
    assert s.n_samples == N


def test_a_missing_body_yields_an_errored_series_that_names_what_was_there(
        doc):
    s = ch.pose_series(doc, "nothing_like_this")
    assert not s.usable
    assert "carries" in (s.error or "") and "walker" in (s.error or "")


def test_container_geometry_takes_the_rim_from_the_subtree_box(doc):
    c = ch.container_geometry(doc, "walker")
    assert c.usable and c.rim_attested
    assert c.rim_z == pytest.approx(0.6)


def test_support_surfaces_are_structural_not_a_name_list(doc):
    names = {s.body for s in ch.support_surfaces(doc)}
    assert names == {"ground", "wall_north"}
    for s in ch.support_surfaces(doc):
        assert s.usable and s.static is True


def test_a_body_that_can_move_is_never_a_support_surface(doc):
    d = copy.deepcopy(doc)
    d["inventory"].append(_inv("crate", 77, static=False, mass=1.0,
                               lo=[0, 0, 0], hi=[1, 1, 1]))
    assert "crate" not in {s.body for s in ch.support_surfaces(d)}


def test_mass_is_the_bodys_own_and_says_so_where_the_tier_needs_it(doc):
    p = ch.body_physics(doc, "walker", disclose_mass=True)
    assert p.mass_kg == pytest.approx(6.0) and p.dynamic is True
    assert "6.0000 kg" in p.source and "10.8000 kg" in p.source
    assert "STRICTER" in p.source


def test_a_density_based_mass_reports_null_with_the_reason(doc):
    d = copy.deepcopy(doc)
    d["inventory"][0]["mass_kg"] = None
    d["inventory"][0]["mass_error"] = "DENSITY-based"
    p = ch.body_physics(d, "walker")
    assert p.mass_kg is None and "DENSITY" in p.error


def test_gravity_arrives_as_a_magnitude_and_a_vector(doc):
    w = ch.world_physics(doc)
    assert w.gravity_mps2 == pytest.approx(9.81)
    assert w.gravity_vec == (0.0, 0.0, -9.81)


def test_gait_contacts_name_the_robot_part_and_classify_the_ground(doc):
    g = ch.gait_contacts(doc, _surface(), "walker")
    assert g.supported and g.has_sample_times and g.classified
    assert {c.robot_body for c in g.contacts} == {"shank_fl", "shank_fr"}
    assert all(c.other_body == "ground" for c in g.contacts)
    assert all(c.other_is_ground is True for c in g.contacts)


def test_a_ground_the_task_did_not_declare_is_not_the_ground(doc):
    d = copy.deepcopy(doc)
    for p in d["contacts"]["pairs"]:
        p["a_name"] = "mystery_slab"
    g = ch.gait_contacts(d, _surface(), "walker")
    assert all(c.other_is_ground is False for c in g.contacts)


def test_gait_without_query_times_cannot_claim_them(doc):
    d = copy.deepcopy(doc)
    d["contacts"]["sample_times"] = []
    g = ch.gait_contacts(d, _surface(), "walker")
    assert not g.has_sample_times


def test_applied_support_attests_zero_only_when_no_route_is_open(doc):
    s = ch.applied_support(doc)
    assert s.attested is True and s.usable
    assert s.peak_force_n == 0.0 and s.peak_torque_nm == 0.0
    assert s.force.shape == (N, 3)
    assert "NO ROUTE WAS OPEN" in s.source
    assert "NOT a wrench read back" in s.source


@pytest.mark.parametrize("route", [
    "a Supervisor robot other than the grader's sampler is present",
    "carries no Physics node",
])
def test_an_open_route_makes_the_support_unverified_never_zero(doc, route):
    d = copy.deepcopy(doc)
    d["structure"]["routes_open"] = [route]
    d["structure"]["attested"] = False
    s = ch.applied_support(d)
    assert s.attested is None          # unverified: not failed, not credited
    assert not s.usable
    assert route in (s.error or "") and route in s.source


def test_a_structural_probe_that_never_ran_is_unverified_not_attested(doc):
    d = copy.deepcopy(doc)
    d["structure"] = {}
    assert ch.applied_support(d).attested is None


def test_arena_is_the_declared_floors_and_names_the_walls(doc):
    a = ch.arena_bounds(doc, _surface())
    assert a.usable
    assert a.aabb_min[0] == pytest.approx(-20.0)
    assert a.aabb_max[1] == pytest.approx(20.0)
    assert a.boundary_bodies == ["wall_north"]


def test_controller_load_is_a_positive_attestation(doc):
    c = ch.controller_load(doc, "walker")
    assert c.loaded is True and c.identity == "gait" and c.attested


def test_a_declared_controller_that_moved_nothing_is_not_loaded(doc):
    d = copy.deepcopy(doc)
    d["controllers"][0].update(loaded=False, joints_moved=0)
    assert ch.controller_load(d, "walker").loaded is False


def test_grip_reports_friction_when_contact_holds_and_nothing_binds(doc):
    d = copy.deepcopy(doc)
    for p in d["contacts"]["pairs"]:
        p.update(a_name="tool", a_robot="#20", b_name="block", b_robot="#30",
                 a_is_ground=False, b_is_ground=False)
    d["inventory"].append(_inv("block", 30, robot=True, mass=0.2,
                               owner="#30"))
    d["inventory"].append(_inv("tool", 20, depth=5, mass=0.1, owner="#20"))
    g = ch.grip_observation(d, _roles(obj="block", eff="tool", con="ground"))
    assert g.mechanism == "friction" and g.contacts
    assert g.contacts[0].held_body == "block"
    assert g.contacts[0].holder_body == "tool"


def test_grip_reports_attachment_when_a_connector_is_in_the_scene(doc):
    d = copy.deepcopy(doc)
    d["structure"]["robots_with_connectors"] = [{"robot": "arm",
                                                 "connectors": 1}]
    g = ch.grip_observation(d, _roles(obj="block", eff="tool"))
    assert g.mechanism == "attachment" and g.attachment is True


def test_grip_is_unknown_rather_than_guessed(doc):
    g = ch.grip_observation(doc, _roles(obj="block", eff="tool"))
    assert g.mechanism == "unknown" and g.contacts == []


# --- the assembled channel dicts ----------------------------------------------


def test_build_t3_supplies_every_key_the_shim_reads(doc):
    out = ch.build_t3(doc, _surface(), "base_link")
    assert set(out) == set(t3_shim.T3_CHANNELS)


def test_build_t2_supplies_every_key_it_can(doc):
    d = copy.deepcopy(doc)
    d["inventory"].append(_inv("block", 30, robot=True, mass=0.2, owner="#30",
                               lo=[0.4, -0.05, 0.05], hi=[0.5, 0.05, 0.1]))
    d["bodies"].append(_body("block", 30, robot=True, owner="#30"))
    d["inventory"].append(_inv("tool", 20, depth=5, mass=0.1, owner="#20"))
    d["bodies"].append(_body("tool", 20, depth=5, owner="#20"))
    out = ch.build_t2(d, _roles(obj="block", eff="tool", con="ground"))
    assert set(out) == set(ladder_adapters.T2_CHANNELS)


def test_the_substitution_note_reaches_every_base_channel(doc):
    out = ch.build_t3(doc, _surface(), "base_link")
    for key in ("standing", "gait", "base_physics", "controller",
                "base_pose"):
        assert "SUBSTITUTION" in out[key].source, key


def test_an_unresolvable_base_supplies_the_reason_rather_than_omitting(doc):
    d = copy.deepcopy(doc)
    d["inventory"].append(_inv("strider", 99, robot=True, owner="#99"))
    out = ch.build_t3(d, _surface(), "base_link")
    # Supplied, not omitted: the instrument ran, so this is a fact about the
    # scene and not a channel we never built.
    assert set(out) == set(t3_shim.T3_CHANNELS)
    assert "unambiguous" in out["base_pose"].error


# --- end to end through the real shims ----------------------------------------


def test_t3_evidence_built_from_these_channels_has_no_unanswered_channel(doc):
    ev = t3_shim.build_evidence(
        "T3_quadruped", sim="omnisim", run_dir=None,
        channels=ch.build_t3(doc, _surface(), "base_link"),
        surface=_surface(), artifact=None)
    assert t3ev.unanswered_channels(ev) == {}


def test_t4_evidence_built_from_these_channels_has_no_unanswered_channel(doc):
    ev = t4_shim.build_evidence(
        "T4_humanoid", sim="omnisim", run_dir=None,
        channels=ch.build_t4(doc, _surface(), "base_link"),
        surface=_surface(), artifact=None)
    assert t4ev.unanswered_channels(ev) == {}


def test_t2_evidence_built_from_these_channels_has_no_unanswered_channel(doc):
    d = copy.deepcopy(doc)
    d["inventory"].append(_inv("block", 30, robot=True, mass=0.2, owner="#30",
                               lo=[0.4, -0.05, 0.05], hi=[0.5, 0.05, 0.1]))
    d["bodies"].append(_body("block", 30, robot=True, owner="#30"))
    d["inventory"].append(_inv("tool", 20, depth=5, mass=0.1, owner="#20"))
    d["bodies"].append(_body("tool", 20, depth=5, owner="#20"))
    for p in d["contacts"]["pairs"]:
        p.update(a_name="tool", a_robot="#20", b_name="block", b_robot="#30")
    roles = _roles(obj="block", eff="tool", con="ground")
    ev = ladder_adapters.build_t2_evidence(
        "T2_transfer", sim="omnisim", run_dir=None,
        channels=ch.build_t2(d, roles), roles=roles)
    assert ladder_adapters.t2_unanswered_channels(ev) == {}


def test_a_document_on_disk_is_found_by_the_hook(tmp_path, doc):
    (tmp_path / "phaseB.csv.channels.json").write_text(
        json.dumps(doc), encoding="utf-8")
    out = oev.t3_channels(str(tmp_path), surface=_surface())
    assert set(out) == set(t3_shim.T3_CHANNELS)
    assert isinstance(out["base_pose"].t, np.ndarray)


# --- the Newton contact-blindness disclosure ----------------------------------


class _Run:
    """A phase-B result carrying only what ``backend_note`` reads."""

    def __init__(self, backend, solver="", support=None):
        self.sidecar = {"backend": backend, "solver": solver}
        self.support = support


def test_an_ode_run_carries_no_backend_note():
    assert oev.backend_note(_Run("ode", "ode")) == ""
    assert oev.backend_note(None) == ""


def test_a_newton_run_names_the_measured_contact_gap():
    note = oev.backend_note(_Run("newton", "XPBD(iters=10)"))
    assert "1008 support contacts on ODE and 0 on Newton" in note
    assert "no_measurement_surface" in note
    assert "newton/XPBD(iters=10)" in note


def test_an_empty_contact_record_on_newton_says_why_in_the_channel(doc):
    d = copy.deepcopy(doc)
    d["contacts"]["pairs"] = []
    out = ch.build_t3(d, _surface(), "base_link",
                      backend_note=oev.backend_note(_Run("newton")))
    assert "no_measurement_surface" in (out["gait"].error or "")


def test_a_populated_contact_record_is_never_blamed_on_the_backend(doc):
    out = ch.build_t3(doc, _surface(), "base_link",
                      backend_note=oev.backend_note(_Run("newton")))
    # The note still rides in the citation, but the channel is NOT errored:
    # contacts were observed, so nothing about them is unmeasured.
    assert out["gait"].error is None
    assert "no_measurement_surface" in out["gait"].source


def test_support_observation_attributes_an_empty_record_to_the_backend():
    run = _Run("newton", "XPBD(iters=10)",
               support={"pairs": [],
                        "witness": {"supported": True, "total_observed": 0,
                                    "distinct_named": 0, "steps_sampled": 126,
                                    "error": None},
                        "window_s": 20.0})
    obs = oev.support_observation(run)
    assert obs.pairs == [] and obs.supported is True
    assert "no_measurement_surface" in (obs.error or "")


def test_the_declared_base_name_comes_from_the_task_file():
    assert oev._declared_robot_name("t3") == "base_link"   # noqa: SLF001
    assert oev._declared_robot_name("t4") == "base_link"   # noqa: SLF001
