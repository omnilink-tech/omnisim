"""Mission authority and truth checks, independent of any model provider."""
import importlib.util
import threading
import time
from pathlib import Path
import pytest

PATH=Path(__file__).resolve().parents[1]/'projects/services/husky_mission/mission_runtime.py'
spec=importlib.util.spec_from_file_location('mission_runtime',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Driver:
    def __init__(self):
        self.x=self.y=0.;self.t=1.;self.latched=True;self.calls=[];self.fake=False;self.frozen=False
        self.wait=None
    def get_state(self):return dict(x=self.x,y=self.y,sim_time=self.t,wall_time=time.time()-(3 if self.frozen else 0),measured_speed_m_s=0.,fault=None)
    def release_mission_latch(self):self.latched=False
    def hold_mission_latch(self):self.latched=True
    def act_stop(self):self.latched=True;return {'settled':True}
    def act_drive_to(self,x,y,wait=True):
        self.calls.append((x,y))
        if self.wait:self.wait.wait(2)
        if not self.latched and not self.fake:self.x=x;self.y=y;self.t+=1
        return {'arrived':True}


def setup():
    d=Driver();r=m.MissionRuntime(d);r.bind_turn('Visit station A, then station B, and return to base.');return d,r


def finish(r):
    r.thread.join(2);assert not r.thread.is_alive();return r.snapshot()


def test_full_route_needs_three_measured_ordered_receipts():
    d,r=setup();assert r.start(['A','B','base'])['accepted'];s=finish(r)
    assert s['status']=='completed'
    assert [v['station'] for v in s['visits']]==['A','B','base']
    assert all(v['error_m']==0 for v in s['visits'])
    assert r.validate_report(s['mission_id'],'completed',['A','B','base'])['verified']


@pytest.mark.parametrize('route',[['C'],['B','A','base'],['A'],[float('nan')],[],['A']*7])
def test_unknown_or_invented_or_reordered_routes_cannot_move(route):
    d,r=setup();assert not r.start(route)['accepted'];assert not d.calls


def test_false_arrival_from_driver_cannot_create_a_visit_or_completion():
    d,r=setup();d.fake=True;r.start(['A','B','base']);s=finish(r)
    assert s['status']=='failed' and not s['visits']
    assert not r.validate_report(s['mission_id'],'completed',['A','B','base'])['verified']


def test_cancel_before_leg_returns_cannot_start_the_next_station():
    d,r=setup();d.wait=threading.Event();r.start(['A','B','base'])
    deadline=time.monotonic()+1
    while not d.calls and time.monotonic()<deadline:time.sleep(.001)
    r.bind_turn('cancel mission');r.stop();d.wait.set();s=finish(r)
    assert s['status']=='cancelled' and len(d.calls)==1 and d.latched
    assert not r.validate_report(s['mission_id'],'completed',['A','B','base'])['verified']


def test_stale_telemetry_fails_before_motion():
    d,r=setup();d.frozen=True;assert r.start(['A','B','base'])['error']=='stale_telemetry';assert not d.calls


def test_duplicate_start_in_same_turn_does_not_repeat_a_completed_mission():
    d,r=setup();r.start(['A','B','base']);finish(r)
    assert r.start(['A','B','base'])['error']=='duplicate_start_in_same_turn'
    assert len(d.calls)==3


def test_restart_constructs_idle_state_and_never_resumes_a_previous_order():
    d,r=setup();r.start(['A','B','base']);finish(r)
    new=m.MissionRuntime(d);assert new.snapshot()['status']=='idle' and new.thread is None


def test_operator_order_still_requires_an_action_verb():
    d,r=setup();r.bind_turn('The stations are A, B and base.')
    assert not r.start(['A','B','base'])['accepted'];assert not d.calls


def test_model_fabricated_completion_cannot_become_the_public_outcome():
    d,r=setup()
    raw={'response':'I visited A and B and successfully returned to base.'}
    out=r.public_reply(raw)
    assert out['response']=='Mission idle. Verified stations: none.'
    assert out['verified_mission']['status']=='idle'
    assert not r.validate_report('invented-id','completed',['A','B','base'])['verified']


def test_model_connection_error_remains_an_error_instead_of_falling_back_to_success():
    d,r=setup();out=r.public_reply({'error':'model_unavailable','response':'Connection failed.'})
    assert out['error']=='model_unavailable'
    assert out['response'].startswith('OmniLink request failed.')


@pytest.mark.parametrize('utterance',[
    'How long does it take to visit station A, then B?',
    'Do not visit station A, then B.',
    'The manual says "visit station A, then B".',
])
def test_canonical_gate_rejects_motion_proposal_for_non_orders(utterance):
    import sys
    sys.path.insert(0,str(PATH.parents[3]/'packages/omnisim-bridges/src'))
    from omnisim_bridges.tool import Tool
    from omnisim_bridges.gate import register_tools
    from omnisim_bridges.bridge_base import vet_toolcall
    register_tools([Tool('start_mission','Visit named stations',{'type':'object','properties':{'stations':{'type':'array','items':{'type':'string'}}},'required':['stations']},lambda args:None,physical=True,surface='mobile')],surface='mobile')
    assert vet_toolcall('start_mission',{'stations':['A','B']},utterance,surface='mobile')


def test_native_chat_also_uses_verified_state_instead_of_model_prose():
    import ast
    import io
    from types import SimpleNamespace
    d,r=setup();source=PATH.parents[3]/'projects/samples/demos/controllers/husky_mission_agent/husky_mission_agent.py'
    tree=ast.parse(source.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='event')
    delivered=[];bridge=SimpleNamespace(mission=r,_journal=io.StringIO())
    ns={'json':__import__('json'),'original_event':lambda b,k,p:delivered.append((k,p))}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
    ns['event'](bridge,'agent',{'text':'I completed A, B and base.'})
    assert delivered[-1][1]['text']=='Mission idle. Verified stations: none.'
    ns['event'](bridge,'audio_out',{'text':'I completed the mission.'})
    assert len(delivered)==1
    ns['event'](bridge,'error',{'text':'Connection failed'})
    assert delivered[-1][1]['text']=='Connection failed'


def test_waypoint_deadline_and_stationary_arrival():
    import sys
    sys.path.insert(0,str(PATH.parent))
    from navigation import Waypoint
    nav=Waypoint((1.5,0),0)
    state={'x':1.48,'y':0.,'yaw':0.,'sim_time':1.,'measured_speed_m_s':0.}
    assert nav.update(state)==(0.,0.)
    state['sim_time']=1.5;nav.update(state);assert nav.result['arrived']
    nav=Waypoint((1.5,0),0);state.update(x=0,sim_time=61);nav.update(state)
    assert not nav.result['arrived']
