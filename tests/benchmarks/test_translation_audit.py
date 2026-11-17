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

"""Gate: the translation audit must be able to go RED.

`lane1/translation_audit.py` is the instrument that replaced the second
in-engine arm deleted with ODE (bdc02139): it compares the `.wbt`'s authored
contract against the mjModel the solver actually stepped, which is the one layer
no external arm can see (bare `mujoco` and `pybullet` validate the SOLVER; they
do not read `.wbt`).

An audit is only worth its green. These tests are the negative arms -- every one
hands the comparator a model that contradicts the world and requires it to say
so -- plus the parser cases that have already bitten once.

All cheap: no engine, no GPU, no network. The LIVE half (probe worlds through a
real engine, per coordinate system and gravity value) is
`translation_audit.py --self-test`, which needs a built binary and so does not
run here.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
_LANE1 = _HERE / "omnibench" / "lane1"


@pytest.fixture(scope="module")
def ta():
    sys.path.insert(0, str(_HERE / "omnibench"))
    spec = importlib.util.spec_from_file_location(
        "ob_translation_audit", _LANE1 / "translation_audit.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ob_translation_audit"] = mod
    spec.loader.exec_module(mod)
    return mod


OK_VERDICT = {"present": True, "degraded": False, "finalised": True,
              "solver": "mujoco"}


def _world(**over):
    w = {"gravity": 9.81, "coordinateSystem": "ENU", "basicTimeStep": 4.0,
         "declared_ignored": [], "coulombFriction": None,
         "newtonGroundMu": None, "n_solid": 2, "n_bounding_objects": 2}
    w.update(over)
    return w


def _model(**over):
    m = {"gravity": [0.0, 0.0, -9.81], "timestep": 0.002,
         "bodies": [{"id": 0, "name": "world", "mass": 0.0},
                    {"id": 1, "name": "ball", "mass": 1.0}],
         "geoms": [{"id": 0, "name": "g", "body": 0, "type": 0, "condim": 3,
                    "contype": 1, "conaff": 1, "friction": [1.0, 0.005, 0.0]}],
         "dofs": [], "n_act": 0}
    m.update(over)
    return m


def _sev(findings, check):
    return [f["severity"] for f in findings if f["check"] == check]


# --- the up-axis mapping --------------------------------------------------

@pytest.mark.parametrize("cs,axis", [("ENU", 2), ("NUE", 1), ("EUN", 1)])
def test_gravity_vector_follows_the_up_axis(ta, cs, axis):
    """`coordinateSystem` names Up by the position of 'U'. Getting this pairing
    wrong is how 210 NUE worlds ran at gravity 0 until c77cbe98."""
    v = ta.expected_gravity_vec(9.81, cs)
    assert v[axis] == pytest.approx(-9.81)
    assert sum(abs(c) for c in v) == pytest.approx(9.81), "one axis only"


def test_gravity_magnitude_is_read_from_the_field(ta):
    assert ta.expected_gravity_vec(3.72, "ENU")[2] == pytest.approx(-3.72)


# --- negative arms: the audit must go red ---------------------------------

def test_wrong_gravity_magnitude_is_an_error(ta):
    """The 'gravity never plumbed' class: the model ignores the field."""
    f = ta.audit(_world(gravity=3.72), _model(), {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "gravity")
    msg = [x["message"] for x in f if x["check"] == "gravity"][0]
    assert "3.72" in msg and "9.81" in msg, "name BOTH sides"


def test_wrong_gravity_axis_is_an_error(ta):
    """The c77cbe98 class: coordinateSystem never reached the solver."""
    f = ta.audit(_world(coordinateSystem="NUE"), _model(),
                 {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "gravity")


def test_zero_gravity_raises_its_own_named_signature(ta):
    f = ta.audit(_world(), _model(gravity=[0.0, 0.0, 0.0]),
                 {"verdict": OK_VERDICT})
    assert any("ZERO gravity" in x["message"] for x in f)
    assert any("c77cbe98" in x["message"] for x in f), \
        "name the commit so the reader can find the precedent"


def test_matching_gravity_is_reported_not_silent(ta):
    """A check that passes must SAY it evaluated -- silence is unfalsifiable."""
    f = ta.audit(_world(), _model(), {"verdict": OK_VERDICT})
    assert "INFO" in _sev(f, "gravity")
    assert not [x for x in f if x["severity"] == "ERROR"]


def test_unevaluatable_gravity_is_a_finding_not_silence(ta):
    """THE BUG THIS INSTRUMENT SHIPPED WITH: the dump emits numpy reprs, the
    number parse returned 6 values instead of 3, and the flagship check skipped
    itself while the report read green."""
    f = ta.audit(_world(), _model(gravity=[1.0, 2.0]), {"verdict": OK_VERDICT})
    assert "WARN" in _sev(f, "gravity")
    assert any("cannot evaluate" in x["message"] for x in f)


def test_numpy_reprs_parse_to_three_floats(ta):
    """The dump interpolates live arrays: `np.float64(-9.81)`. A naive number
    regex matches the 64 in the TYPE NAME."""
    got = ta._floats("[np.float64(-0.0), np.float64(-0.0), "
                     "np.float64(-9.8100004196167)]")
    assert len(got) == 3, got
    assert got[2] == pytest.approx(-9.81, abs=1e-5)


def test_frictionless_contact_under_a_declared_mu_is_an_error(ta):
    """condim=1 means mu is not consulted AT ALL -- friction fidelity is then
    unmeasurable rather than merely wrong."""
    m = _model()
    m["geoms"][0]["condim"] = 1
    f = ta.audit(_world(newtonGroundMu=0.5), m, {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "condim")


def test_declared_coulomb_friction_that_never_reaches_the_model(ta):
    """~202 worlds declare an ODE-path friction the Newton backend cannot read.
    The audit must show the CONTRADICTION with both numbers, not just name the
    field."""
    f = ta.audit(_world(coulombFriction=0.0), _model(),
                 {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "friction_contract")
    msg = [x["message"] for x in f if x["check"] == "friction_contract"][0]
    assert "1.0" in msg, "must report the friction the model actually got"


def test_a_world_with_no_geoms_cannot_collide(ta):
    f = ta.audit(_world(), _model(geoms=[]), {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "geoms")


def test_all_massless_bodies_is_an_error(ta):
    m = _model(bodies=[{"id": 0, "name": "world", "mass": 0.0},
                       {"id": 1, "name": "ball", "mass": 0.0}])
    f = ta.audit(_world(), m, {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "mass")


def test_a_massless_worldbody_alone_is_not_an_error(ta):
    """MuJoCo's body 0 is the static worldbody and is massless by construction;
    flagging it would make every world fail."""
    f = ta.audit(_world(), _model(), {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "mass")


def test_timestep_that_is_not_a_whole_number_of_substeps(ta):
    f = ta.audit(_world(basicTimeStep=5.0), _model(timestep=0.002),
                 {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "timestep")


# --- provenance -----------------------------------------------------------

def test_an_unverified_run_cannot_be_audited(ta):
    """The model must be attributable to a proven Newton run, or the audit is
    describing a model nothing confirmed was stepped."""
    f = ta.audit(_world(), _model(), {"verdict": {"present": False}})
    assert "ERROR" in _sev(f, "attribution")
    assert any("omnisim-unverified" in x["message"] for x in f)


def test_a_failed_dump_is_an_error_not_an_empty_pass(ta):
    f = ta.audit(_world(), {"error": "no dump written"}, {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "dump")


# --- the .wbt reader ------------------------------------------------------

def test_parse_world_reads_the_contract(ta, tmp_path):
    w = tmp_path / "x.wbt"
    w.write_text(
        'WorldInfo {\n  gravity 3.72\n  basicTimeStep 8\n'
        '  coordinateSystem "NUE"\n  newtonGroundMu 0.4\n'
        '  contactProperties [\n    ContactProperties {\n'
        '      coulombFriction [ 0.8 ]\n    }\n  ]\n}\n'
        'DEF F Solid { boundingObject USE S }\n', encoding="utf-8")
    got = ta.parse_world(str(w))
    assert got["gravity"] == 3.72
    assert got["basicTimeStep"] == 8
    assert got["coordinateSystem"] == "NUE"
    assert got["newtonGroundMu"] == 0.4
    assert got["coulombFriction"] == 0.8
    assert "contactProperties" in got["declared_ignored"]
    assert "coulombFriction" in got["declared_ignored"]


def test_worldinfo_block_is_brace_matched(ta, tmp_path):
    """contactProperties nests, so a first-`}` scan would truncate the block and
    silently lose every field after it."""
    w = tmp_path / "x.wbt"
    w.write_text(
        'WorldInfo {\n  contactProperties [\n    ContactProperties {\n'
        '      coulombFriction [ 0.8 ]\n    }\n  ]\n  gravity 1.62\n}\n',
        encoding="utf-8")
    assert ta.parse_world(str(w))["gravity"] == 1.62


def test_missing_coordinate_system_defaults_to_enu(ta, tmp_path):
    w = tmp_path / "x.wbt"
    w.write_text('WorldInfo {\n  gravity 9.81\n}\n', encoding="utf-8")
    assert ta.parse_world(str(w))["coordinateSystem"] == "ENU"


def test_omitted_worldinfo_fields_use_the_schema_defaults(ta, tmp_path):
    """Most worlds in the tree declare neither `gravity` nor `basicTimeStep`.
    Treating an omitted field as unknown would make the audit report "cannot
    evaluate" across the whole corpus -- useless on exactly what it exists to
    check. worldinfo.md documents the defaults; use them, and record that they
    were defaulted rather than declared."""
    w = tmp_path / "x.wbt"
    w.write_text('WorldInfo {\n  coordinateSystem "ENU"\n}\n', encoding="utf-8")
    got = ta.parse_world(str(w))
    assert got["gravity"] == 9.81
    assert got["gravity_declared"] is False
    assert got["basicTimeStep"] == 32.0
    assert got["basicTimeStep_declared"] is False


def test_declared_fields_are_marked_declared(ta, tmp_path):
    w = tmp_path / "x.wbt"
    w.write_text('WorldInfo {\n  gravity 1.62\n  basicTimeStep 8\n}\n',
                 encoding="utf-8")
    got = ta.parse_world(str(w))
    assert got["gravity_declared"] is True
    assert got["basicTimeStep_declared"] is True


# --- the static sweep and the migration ------------------------------------
# The corpus-scale half. The systemic defect is statically decidable:
# coulombFriction is not read and newtonGroundMu defaults to 1.0, so a world
# declaring 0.5 and no newtonGroundMu provably runs at 1.0.

def _write(tmp_path, name, worldinfo_body):
    p = tmp_path / name
    p.write_text("WorldInfo {\n%s}\n" % worldinfo_body, encoding="utf-8")
    return p


def test_static_flags_a_friction_that_cannot_reach_the_solver(ta, tmp_path):
    p = _write(tmp_path, "a.wbt",
               "  contactProperties [\n    ContactProperties "
               "{\n      coulombFriction [ 0.5 ]\n    }\n  ]\n")
    _, findings = ta.audit_static(str(p))
    errs = [f for f in findings if f["severity"] == "ERROR"]
    assert errs and errs[0]["check"] == "friction_unreachable"
    assert errs[0]["effective"] == 1.0


def test_static_is_quiet_when_the_world_declares_the_field(ta, tmp_path):
    p = _write(tmp_path, "b.wbt",
               "  newtonGroundMu 0.5\n  contactProperties [\n    "
               "ContactProperties {\n      coulombFriction [ 0.5 ]\n    }\n  ]\n")
    _, findings = ta.audit_static(str(p))
    assert not [f for f in findings if f["severity"] == "ERROR"]


def test_static_flags_two_frictions_that_disagree(ta, tmp_path):
    p = _write(tmp_path, "c.wbt",
               "  newtonGroundMu 3\n  contactProperties [\n    "
               "ContactProperties {\n      coulombFriction [ 5 ]\n    }\n  ]\n")
    _, findings = ta.audit_static(str(p))
    assert any(f["check"] == "friction_contradiction" for f in findings)


def test_per_material_frictions_are_a_DIFFERENT_defect(ta, tmp_path):
    """contactProperties is a LIST, one entry per material pair; newtonGroundMu
    is a single global value. No rewrite can represent that, so it must not be
    reported as the same finding -- that would imply the migration fixes it."""
    p = _write(tmp_path, "d.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 0.2 ]\n    }\n    ContactProperties {\n"
               "      coulombFriction [ 0.8 ]\n    }\n  ]\n")
    w, findings = ta.audit_static(str(p))
    assert w["coulombFriction_uniform"] is False
    assert any(f["check"] == "friction_per_material" for f in findings)
    changed, why = ta.apply_fix(str(p))
    assert changed is False and "per-material" in why


def test_fix_declares_the_worlds_own_friction(ta, tmp_path):
    p = _write(tmp_path, "e.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 5 ]\n    }\n  ]\n")
    changed, why = ta.apply_fix(str(p))
    assert changed and "5" in why
    assert ta.parse_world(str(p))["newtonGroundMu"] == 5.0
    # and the world is now clean
    assert not [f for f in ta.audit_static(str(p))[1]
                if f["severity"] == "ERROR"]


def test_fix_is_idempotent(ta, tmp_path):
    p = _write(tmp_path, "f.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 2 ]\n    }\n  ]\n")
    assert ta.apply_fix(str(p))[0] is True
    before = p.read_text(encoding="utf-8")
    assert ta.apply_fix(str(p))[0] is False, "second run must be a no-op"
    assert p.read_text(encoding="utf-8") == before


def test_mu_zero_is_now_migratable(ta, tmp_path):
    """μ=0 USED to be unsayable: `newtonGroundMu 0` meant "unset -> 1.0", so a
    frictionless world could not state itself and this migration refused it.

    The sentinel was fixed 2026-08-09 -- the field defaults to -1 and negative
    means unset -- across all four sites that had baked in the `> 0` rule:
    WorldInfo.wrl, OmSolid's resolvedNewtonGroundMu (the ONE resolution point),
    the C++ prefs gate, and the embedded runtime's set_contact_solver_params.
    Verified live: undeclared -> 1.0, `newtonGroundMu 0` -> 0.0.
    """
    p = _write(tmp_path, "g.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 0 ]\n    }\n  ]\n")
    changed, why = ta.apply_fix(str(p))
    assert changed is True, why
    assert ta.parse_world(str(p))["newtonGroundMu"] == 0.0
    assert not [f for f in ta.audit_static(str(p))[1]
                if f["severity"] == "ERROR"]


def test_fix_preserves_crlf(ta, tmp_path):
    p = tmp_path / "h.wbt"
    p.write_bytes(b"WorldInfo {\r\n  contactProperties [\r\n    "
                  b"ContactProperties {\r\n      coulombFriction [ 2 ]\r\n"
                  b"    }\r\n  ]\r\n}\r\n")
    assert ta.apply_fix(str(p))[0] is True
    raw = p.read_bytes()
    assert b"\r\n" in raw and raw.count(b"\n") == raw.count(b"\r\n"), \
        "a rewrite must not flip the file's line endings"


def test_sweep_excludes_recorded_artifacts(ta, tmp_path):
    """A world under results/ is a RECORD of what an agent produced on a date.
    Counting it inflates the defect number and invites falsifying the record."""
    body = ("  contactProperties [\n    ContactProperties {\n"
            "      coulombFriction [ 0.5 ]\n    }\n  ]\n")
    (tmp_path / "live").mkdir()
    _write(tmp_path / "live", "real.wbt", body)
    (tmp_path / "results" / "cell").mkdir(parents=True)
    _write(tmp_path / "results" / "cell", "artifact.wbt", body)
    _write(tmp_path / "live", ".omnisim_probe_scratch.wbt", body)

    _, counts = ta.sweep(str(tmp_path))
    assert counts["worlds"] == 1, "only the live world counts"
    _, counts_all = ta.sweep(str(tmp_path), include_artifacts=True)
    assert counts_all["worlds"] == 3


def test_fix_refuses_when_a_launcher_forces_a_different_friction(ta, tmp_path):
    """THE MISTAKE THIS ENCODES: three OMNIARM6 demos were rewritten to declare
    `newtonGroundMu 5` from their own coulombFriction while their run_*.ps1
    exports 1.5. The edit is inert (env beats field) but makes the file
    AUTHORITATIVELY state a friction the sanctioned run does not use -- the same
    objection that keeps this migration away from projects/policies. A sweep
    cannot decide which number is intended."""
    w = _write(tmp_path, "demo.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 5 ]\n    }\n  ]\n")
    (tmp_path / "run_demo.ps1").write_text(
        '$env:OMNISIM_NEWTON_GROUND_MU          = "1.5"\n', encoding="utf-8")
    assert ta.launcher_friction(str(w)) == 1.5, \
        "aligned assignments must still parse -- a tight padding bound made " \
        "this silently return None and the guard passed"
    changed, why = ta.apply_fix(str(w))
    assert changed is False
    assert "1.5" in why and "disagree" in why


def test_fix_proceeds_when_the_launcher_agrees(ta, tmp_path):
    w = _write(tmp_path, "demo2.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 2 ]\n    }\n  ]\n")
    (tmp_path / "run_demo2.sh").write_text(
        "export OMNISIM_NEWTON_GROUND_MU=2.0\n", encoding="utf-8")
    assert ta.launcher_friction(str(w)) == 2.0
    assert ta.apply_fix(str(w))[0] is True


def test_no_launcher_is_not_a_conflict(ta, tmp_path):
    w = _write(tmp_path, "demo3.wbt",
               "  contactProperties [\n    ContactProperties {\n"
               "      coulombFriction [ 3 ]\n    }\n  ]\n")
    assert ta.launcher_friction(str(w)) is None
    assert ta.apply_fix(str(w))[0] is True


# --- the extended checks: inertia, joint limits, actuators -----------------
# Enabled by extending the engine's model dump with body_inertia and joint
# ranges. Inertia is checkable from FIRST PRINCIPLES -- no reference model and no
# scene knowledge -- which is what makes it a real instrument rather than a
# golden comparison.

def _model_i(inertia, mass=1.0):
    return _model(bodies=[{"id": 0, "name": "world", "mass": 0.0,
                           "inertia": [0.0, 0.0, 0.0]},
                          {"id": 1, "name": "link", "mass": mass,
                           "inertia": inertia}])


def test_negative_principal_moment_is_impossible(ta):
    f = ta.audit(_world(), _model_i([0.1, -0.2, 0.1]), {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "inertia")


def test_inertia_violating_the_triangle_inequality_is_caught(ta):
    """Ia + Ib >= Ic for every permutation, or no rigid body has that tensor.
    This is the generic form of the husky-wheel-preset defect: a fabricated
    inertia rather than one derived from the body's geometry."""
    f = ta.audit(_world(), _model_i([0.01, 0.01, 5.0]), {"verdict": OK_VERDICT})
    errs = [x for x in f if x["check"] == "inertia"]
    assert errs and errs[0]["severity"] == "ERROR"
    assert "triangle" in errs[0]["message"]


def test_a_physically_valid_inertia_passes(ta):
    # solid box 1x1x1, m=1 -> I = m/12*(a^2+b^2) = 0.1667 each: valid.
    f = ta.audit(_world(), _model_i([0.1667, 0.1667, 0.1667]),
                 {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "inertia")


def test_a_thin_rod_inertia_passes(ta):
    """Degenerate-looking but legal: Ix ~ 0, Iy = Iz. The check must not reject
    slender bodies, which are everywhere in robot models."""
    f = ta.audit(_world(), _model_i([1e-6, 0.083, 0.083]), {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "inertia")


def test_static_and_world_bodies_are_exempt(ta):
    """MuJoCo's worldbody is massless with zero inertia by construction, and a
    static body carries none -- flagging either would make every world fail."""
    f = ta.audit(_world(), _model_i([0.0, 0.0, 0.0], mass=0.0),
                 {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "inertia")


def test_an_inverted_joint_limit_is_caught(ta):
    m = _model()
    m["joints"] = [{"id": 0, "name": "hinge", "type": 3, "limited": 1,
                    "range": [1.0, -1.0]}]
    f = ta.audit(_world(), m, {"verdict": OK_VERDICT})
    assert "ERROR" in _sev(f, "joint_limits")


def test_unlimited_joints_are_not_flagged(ta):
    m = _model()
    m["joints"] = [{"id": 0, "name": "wheel", "type": 3, "limited": 0,
                    "range": [0.0, 0.0]}]
    f = ta.audit(_world(), m, {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "joint_limits")


def test_declared_motors_that_produced_no_actuators(ta):
    """AGENTS.md: motorised BallJoint/Hinge2Joint are accepted and SILENTLY
    ignored. Otherwise visible only as 'the robot does not move'."""
    f = ta.audit(_world(n_motor_devices=2), _model(n_act=0),
                 {"verdict": OK_VERDICT})
    errs = [x for x in f if x["check"] == "actuators"]
    assert errs and errs[0]["severity"] == "ERROR"
    assert "silently ignored" in errs[0]["message"]


def test_motors_that_did_produce_actuators_are_fine(ta):
    f = ta.audit(_world(n_motor_devices=2), _model(n_act=2),
                 {"verdict": OK_VERDICT})
    assert "ERROR" not in _sev(f, "actuators")
