"""Regression for the short-turn rate cliff, without loading controller DLLs.

Compile the production policy methods directly. Physical convergence is checked
separately by control-bench recovery-turns and the live recovery matrix.
"""
import ast
import math
from pathlib import Path

import pytest


def policy():
    path = Path(__file__).resolve().parents[1]/'projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    source = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MobileBridge')
    names = {'TURN_RATE_HORIZON_S','TURN_SLOW_W','TURN_TOL_MIN_RAD','TURN_TOL_RAD','TURN_TOL_FRAC'}
    nodes = [n for n in source.body if
             isinstance(n,ast.FunctionDef) and n.name in ('_turn_pulse_rate','_turn_tol') or
             isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in n.targets)]
    cls = ast.ClassDef(name='Policy', bases=[], keywords=[], body=nodes, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[]))
    namespace = {}
    exec(compile(module,str(path),'exec'),namespace)
    return namespace['Policy']()


@pytest.mark.parametrize('ceiling',[.04,.6,1.0])
def test_crossing_old_boundary_has_no_abrupt_speed_jump(ceiling):
    bridge=policy(); bridge.spin_speed=ceiling
    assert abs(bridge._turn_pulse_rate(.251)-bridge._turn_pulse_rate(.249)) < .005


@pytest.mark.parametrize('angle',[.003,.05,.1,.24,.26,.38,.65,math.pi/2,math.pi,3*math.pi/2])
def test_rate_is_symmetric_bounded_and_resolves_small_turns(angle):
    bridge=policy(); bridge.spin_speed=.6
    rate=bridge._turn_pulse_rate(angle)
    assert 0 < rate <= bridge.spin_speed
    assert rate == bridge._turn_pulse_rate(-angle)
    if angle <= .38:
        # One basic 16ms step must not consume the whole terminal tolerance.
        assert rate*.016 < bridge._turn_tol(angle)


def test_large_turn_retains_cruise_rate():
    bridge=policy(); bridge.spin_speed=.6
    assert bridge._turn_pulse_rate(math.pi) == bridge.spin_speed


def test_short_turns_have_monotone_continuous_rates():
    bridge=policy(); bridge.spin_speed=.6
    rates=[bridge._turn_pulse_rate(i/1000) for i in range(1,1501)]
    assert all(0 <= b-a < .001 for a,b in zip(rates,rates[1:]))
