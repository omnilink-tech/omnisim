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

"""Optional flight-realism terms in mavic_dynamics (2026-09-25).

Rotor lag, ground effect, quadratic drag and seeded wind are each OFF unless an
airframe declares them, so the shipped Mavic -- which declares nothing -- must
apply exactly the forces it always did. Measured on a PX4 x500 with all four
on (1080p, sim-time recorder): hover sway 2.2 cm, a soft touchdown, heading
held within 2 deg once the bridge's heading hold was enabled.
"""
from __future__ import annotations

import ast
import importlib.util
import math
import pathlib

import pytest

CTRL = (pathlib.Path(__file__).resolve().parents[3]
        / "projects" / "samples" / "demos" / "controllers" / "mavic_omnilink_bridge")


def _dynamics():
    spec = importlib.util.spec_from_file_location("mavic_dynamics_rt", CTRL / "mavic_dynamics.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Node:
    def __init__(self, z=1.0, vel=(0.0, 0.0, 0.0)):
        self.forces, self.torques, self.world_forces = [], [], []
        self.z, self.vel = z, vel

    def addForceWithOffset(self, f, o, rel):
        self.forces.append((list(f), list(o)))

    def addTorque(self, t, rel):
        self.torques.append(list(t))

    def addForce(self, f, rel):
        self.world_forces.append(list(f))

    def getPosition(self):
        return [0.0, 0.0, self.z]

    def getOrientation(self):
        return [1, 0, 0, 0, 1, 0, 0, 0, 1]

    def getVelocity(self):
        return list(self.vel) + [0.0, 0.0, 0.0]


def test_an_airframe_that_declares_nothing_gets_the_old_forces_exactly():
    md = _dynamics()
    node = Node()
    md.RotorDynamics(node).step(70.0, -69.0, -71.0, 68.0, dt=0.008)
    k = 0.00054
    assert [f[0][2] for f in node.forces] == pytest.approx(
        [k * 70 * 70, k * -69 * 69, k * -71 * 71, k * 68 * 68])
    assert node.world_forces == []                     # no drag
    assert node.torques[0][:2] == [0.0, 0.0]           # no turbulence
    fl2, fr2, rl2, rr2 = 70 * 70, -69 * 69, -71 * 71, 68 * 68
    assert node.torques[0][2] == pytest.approx(0.0000052 * ((fr2 + rl2) - (fl2 + rr2)))


def test_ground_effect_is_one_far_up_and_capped_near_the_ground():
    ge = _dynamics().RotorDynamics.ground_effect
    assert ge(0.165, 10.0) == pytest.approx(1.0, abs=1e-4)
    assert ge(0.165, 0.2) > ge(0.165, 0.4) > ge(0.165, 1.0) > 1.0
    assert ge(0.165, 0.0) == pytest.approx(1.0 / 0.75)   # capped at R / 4z = 0.5


def test_rotor_lag_delays_thrust_and_idle_spins_it_down():
    md = _dynamics()
    rd = md.RotorDynamics(Node(), {"motor_tau_s": 0.04})
    rd.step(68.5, 68.5, 68.5, 68.5, dt=0.008)
    assert 0.0 < rd.rotor_speed() < 68.5 * 0.2          # one tick: ~18% of the way
    for _ in range(200):
        rd.step(68.5, 68.5, 68.5, 68.5, dt=0.008)
    assert rd.rotor_speed() == pytest.approx(68.5, rel=1e-3)
    rd.idle()
    assert rd.rotor_speed() == 0.0


def _run_wind(seed, n=50):
    md = _dynamics()
    node = Node(vel=(0.0, 0.0, 0.0))
    rd = md.RotorDynamics(node, {"drag_coeff": 0.03, "wind": {
        "mean": [0.3, 0.1, 0.0], "gust_sigma": 0.4, "tau_s": 2.0, "seed": seed,
        "torque_sigma": 0.004}})
    for _ in range(n):
        rd.step(68.5, 68.5, 68.5, 68.5, dt=0.008)
    return node.world_forces, node.torques


def test_wind_is_deterministic_for_a_seed_and_differs_across_seeds():
    assert _run_wind(7) == _run_wind(7)
    assert _run_wind(7) != _run_wind(8)


def test_drag_opposes_motion_relative_to_the_air():
    md = _dynamics()
    node = Node(vel=(2.0, 0.0, 0.0))
    md.RotorDynamics(node, {"drag_coeff": 0.03}).step(0, 0, 0, 0, dt=0.008)
    f = node.world_forces[0]
    assert f[0] == pytest.approx(-0.03 * 2.0 * 2.0) and f[1] == 0.0 and f[2] == 0.0


def test_turbulence_barely_yaws():
    _, torques = _run_wind(7, n=400)
    rms = lambda i: math.sqrt(sum(t[i] ** 2 for t in torques) / len(torques))
    assert rms(2) < 0.2 * min(rms(0), rms(1))


# ── bridge wiring, pinned from source (the flight loop is a closure) ──────

BRIDGE = CTRL / "mavic_omnilink_bridge.py"


def test_landing_cuts_on_the_measured_rest_height_not_a_fixed_0_4_m():
    src = BRIDGE.read_text(encoding="utf-8")
    assert "altitude < 0.4 and abs(state.v_z) < 0.3" not in src
    assert "h = altitude - rest" in src and "state.rest_z = altitude" in src


def test_realism_controls_default_off_and_are_opt_in_gains():
    tree = ast.parse(BRIDGE.read_text(encoding="utf-8"))
    consts = {t.id: n.value for n in tree.body if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    assert ast.literal_eval(consts["HEADING_HOLD"]) == 0.0
    assert ast.unparse(consts["XY_I_RADIUS_M"]) == "POS_HOLD_RADIUS_M"
    gains = ast.literal_eval(consts["TUNABLE_GAINS"])
    assert {"HEADING_HOLD", "XY_I_RADIUS_M", "K_POS", "MAX_YAW_DISTURBANCE"} <= set(gains)


# ── /capabilities disclosure + rotor-disc lookup by material name ────────

def _bridge_funcs(*names):
    tree = ast.parse(BRIDGE.read_text(encoding="utf-8"))
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in body} == set(names)
    ns = {}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(BRIDGE), "exec"), ns)
    return ns


def test_an_undeclared_airframe_discloses_nothing_modelled():
    disclose = _bridge_funcs("_airframe_disclosure")["_airframe_disclosure"]
    assert disclose(None) == {"declared": False, "modelled_effects": [], "control_overrides": []}


def test_a_declared_wind_is_disclosed_as_modelled():
    disclose = _bridge_funcs("_airframe_disclosure")["_airframe_disclosure"]
    out = disclose({"k_thrust": 0.0011, "motor_tau_s": 0.04, "drag_coeff": 0.03,
                    "wind": {"mean": [0.3, 0.1, 0.0], "gust_sigma": 0.4, "tau_s": 2.0, "seed": 7},
                    "control": {"HEADING_HOLD": 1}})
    assert out["declared"] is True
    assert out["modelled_effects"] == ["rotor_spin_up_lag", "quadratic_drag", "wind_and_turbulence"]
    assert out["wind"]["kind"].startswith("MODELLED")
    assert out["wind"]["seed"] == 7 and out["control_overrides"] == ["HEADING_HOLD"]


class _Field:
    def __init__(self, sf=None, mf=None, s=None):
        self.sf, self.mf, self.s = sf, mf, s

    def getSFNode(self):
        return self.sf

    def getMFNode(self, i):
        return self.mf[i]

    def getCount(self):
        return len(self.mf)

    def getSFString(self):
        return self.s


class _Node:
    def __init__(self, type_name, **fields):
        self.type_name, self.fields = type_name, fields

    def getTypeName(self):
        return self.type_name

    def getField(self, name):
        return self.fields.get(name)


def test_rotor_discs_are_found_by_the_urdf_material_name():
    find = _bridge_funcs("_appearances_named")["_appearances_named"]
    disc = lambda n: _Node("PBRAppearance", name=_Field(s=n), transparency=_Field())
    d1, d2, other = disc("x500_prop_disc"), disc("x500_prop_disc"), disc("x500_prop")
    prop = lambda app: _Node("Solid", children=_Field(mf=[_Node("Shape", appearance=_Field(sf=app))]))
    root = _Node("Robot", children=_Field(mf=[
        _Node("HingeJoint", endPoint=_Field(sf=prop(d1))),
        _Node("HingeJoint", endPoint=_Field(sf=prop(d2))),
        _Node("Shape", appearance=_Field(sf=other)),
    ]))
    found = find(root, "x500_prop_disc")
    assert len(found) == 2 and d1 in found and d2 in found and other not in found
