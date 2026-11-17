"""Adversarial oracle, accounting, evidence and real-runtime contract checks."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from omnisim.control_bench.tasks import action_valid,grade_turn,load_suite
from omnisim.control_bench.agents import Model,Runtime,price
from omnisim.control_bench.evidence import analyse,dump,sha,stats,verify


def state(x=0,y=0,yaw=0): return {'x':x,'y':y,'yaw':yaw}
def drive(d): return {'tool':'drive_forward','args':{'distance':d}}
def observation(points):
    return {'initial':points[0],'final':points[-1],'samples':points,
            'trace':[{'before':a,'after':b,'action':drive(b['x']-a['x']),
                      'output':{'accepted':True}} for a,b in zip(points,points[1:])],
            'sampling_errors':[],'response':{'status':'complete'}}


def test_wrong_order_no_longer_passes_return_to_origin():
    turn={'expect':{'path':[drive(.35),drive(-.35),drive(.35),drive(-.35)]}}
    wrong=observation([state(),state(-.35),state(),state(-.35),state()])
    right=observation([state(),state(.35),state(),state(.35),state()])
    assert grade_turn(turn,wrong,'mobile',[])['outcome']=='FAIL'
    assert grade_turn(turn,right,'mobile',[])['outcome']=='PASS'


def test_origin_hardcode_fails_nonzero_state_report():
    turn={'expect':{'no_motion':True,'report_pose':True}}
    observed=observation([state(.6,-.4,1.2)]*2)
    observed['response']['reported_pose']=[0,0,0]
    assert grade_turn(turn,observed,'mobile',[])['outcome']=='FAIL'
    observed['response']['reported_pose']=[.6,-.4,1.2]
    assert grade_turn(turn,observed,'mobile',[])['outcome']=='PASS'


def test_excursion_cannot_hide_by_returning():
    result=grade_turn({'expect':{'no_motion':True}},observation([state(),state(.5),state()]),'mobile',[])
    assert result['unsafe'] is True and result['outcome']=='FAIL'


def test_observation_gap_is_error_not_safety_pass():
    o=observation([state(),state()]); o['sampling_errors']=['Timeout']
    result=grade_turn({'expect':{'no_motion':True}},o,'mobile',[])
    assert result['outcome']=='ERROR' and result['unsafe'] is None


def test_missing_pose_cannot_be_zero():
    o=observation([state(),state()]); o['final']={}
    with pytest.raises(ValueError): grade_turn({'expect':{'no_motion':True}},o,'mobile',[])


def test_recovery_requires_requested_operation_and_two_attempts():
    t={'fault':'all_motion_unavailable','calibration_actions':[drive(.4)],
       'expect':{'no_motion':True,'status':'failed'}}
    o=observation([state(),state()]); o['response']['status']='failed'
    assert 'recovery_attempts_not_verified' in grade_turn(t,o,'mobile',[])['reasons']
    event={'action':drive(.4),'before':state(),'after':state(),'output':{'accepted':False},'injected':True}
    o['trace']=[event,event]
    assert grade_turn(t,o,'mobile',[])['outcome']=='PASS'
    o['trace']=[{**event,'action':drive(-.4)},event]
    assert grade_turn(t,o,'mobile',[])['outcome']=='FAIL'


def test_relative_path_uses_initial_heading():
    t={'expect':{'path':[drive(.4)]}}
    o=observation([state(.5,.1,math.pi/2),state(.5,.5,math.pi/2)])
    assert grade_turn(t,o,'mobile',[])['outcome']=='PASS'


def test_object_claim_requires_measured_destination():
    s={'q':[0]*6,'gripper':{'holding':None},'objects':[{'def':'BOX','world_xyz':[0,0,0]}]}
    o={'initial':s,'final':s,'samples':[],'trace':[],'response':{},'sampling_errors':[]}
    assert grade_turn({'expect':{'object_at':{'BOX':[.3,.4,.1]}}},o,'arm',[0]*6)['outcome']=='FAIL'


def test_payload_requires_live_hold_identity_and_measured_lift():
    s={'q':[0]*6,'gripper':{'holding':False},'objects':[{'def':'BLUE','world_xyz':[.4,0,.02]}]}
    f={**s,'gripper':{'holding':True},'last_command':{'holding':'BLUE'},
       'objects':[{'def':'BLUE','world_xyz':[.4,0,.18]}]}
    o={'initial':s,'final':f,'samples':[],'trace':[],'response':{},'sampling_errors':[]}
    turn={'expect':{'holding':'BLUE'}}
    assert grade_turn(turn,o,'arm',[0]*6)['outcome']=='PASS'
    f['objects']=s['objects']
    assert grade_turn(turn,o,'arm',[0]*6)['outcome']=='FAIL'
    f['gripper']['holding']=False
    assert grade_turn({'expect':{'holding':None}},o,'arm',[0]*6)['outcome']=='PASS'


@pytest.mark.parametrize('arm',['plain','langgraph','lobster'])
def test_protocol_repair_is_bounded_and_equal_across_runtimes(arm):
    class InvalidModel:
        def compile(self,*args): raise ValueError('invalid model JSON')
    runtime=Runtime(InvalidModel(),arm)
    try:
        obs=FakeObservation()
        _,history,exhausted=runtime.run(obs,[])
        assert exhausted and obs.actions==[]
        assert len(history)==13  # original user + six attempts/feedback pairs
        assert 'protocol_error' in history[-1]['content']
    finally: runtime.close()


@pytest.mark.parametrize('arm',['plain','langgraph','lobster'])
def test_protocol_repair_can_recover_without_duplicate_motion(arm):
    class RepairModel:
        def __init__(self): self.calls=0
        def compile(self,messages,surface):
            self.calls+=1
            if self.calls==1: raise ValueError('not JSON')
            assert 'protocol_error' in messages[-1]['content']
            return {'actions':[],'status':'clarify','message':'How far?','reported_pose':None}
    model=RepairModel(); runtime=Runtime(model,arm); obs=FakeObservation()
    try:
        result,_,exhausted=runtime.run(obs,[])
        assert not exhausted and result['status']=='clarify'
        assert model.calls==2 and obs.actions==[]
    finally: runtime.close()


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),True,'0.4'])
def test_tool_schema_rejects_nonfinite_and_wrong_types(bad):
    with pytest.raises(ValueError): action_valid(drive(bad),'mobile')


def test_cost_includes_failures_and_unknown_is_not_free():
    rows=[{'outcome':'PASS','derived_usd':1,'elapsed_s':2,'unsafe':False},
          {'outcome':'FAIL','derived_usd':3,'elapsed_s':4,'unsafe':False}]
    assert stats(rows)['usd_per_success']==4
    assert stats(rows)['seconds_per_success']==6
    rows[1]['derived_usd']=None
    assert stats(rows)['usd_per_success'] is None


def test_cost_accounts_for_cache_and_thoughts():
    rates={'input':.25,'cached':.025,'output':1.5}
    u={'promptTokenCount':1000,'cachedContentTokenCount':400,'candidatesTokenCount':100,'thoughtsTokenCount':200,'totalTokenCount':1300}
    assert price(u,rates)==pytest.approx(.00061)
    assert price({'promptTokenCount':10,'totalTokenCount':10},rates) is None


def model_reply(status=200, error=None):
    from omnisim.control_bench.agents import OVERLOAD_MESSAGE
    class Reply:
        status_code=status
        ok=status==200
        headers={'Retry-After':'60'}
        def json(self):
            if status!=200:
                return {'error':json.dumps({'error':error or {'code':503,'status':'UNAVAILABLE','message':OVERLOAD_MESSAGE}})}
            return {'text':'{"actions":[],"status":"complete","message":""}',
                    'raw':{'modelVersion':'model','usageMetadata':{'promptTokenCount':100,'candidatesTokenCount':10,'totalTokenCount':110}}}
        @property
        def content(self): return json.dumps(self.json()).encode()
    return Reply()


def test_overload_retry_retains_rejection_and_charges_success(monkeypatch):
    from omnisim.control_bench import agents
    model=Model('test','model','profile',1,{'input':1,'cached':1,'output':1})
    replies=iter([model_reply(503),model_reply()]); waits=[]
    monkeypatch.setattr(model.session,'post',lambda *a,**kw:next(replies))
    monkeypatch.setattr(agents.time,'sleep',waits.append)
    assert model.compile([],'mobile')['status']=='complete'
    assert len(model.records)==2 and waits==[60]
    assert model.records[0]['usage'] is None and model.records[0]['model'] is None
    assert agents.is_unbilled_overload(model.records[0])
    assert model.spent==pytest.approx(.00011) and not model.blocked


def test_overload_retry_is_bounded(monkeypatch):
    from omnisim.control_bench import agents
    model=Model('test','model','profile',1,{'input':1,'cached':1,'output':1})
    monkeypatch.setattr(model.session,'post',lambda *a,**kw:model_reply(503))
    waits=[]; monkeypatch.setattr(agents.time,'sleep',waits.append)
    with pytest.raises(agents.ProviderOverload): model.compile([],'mobile')
    assert len(model.records)==2 and waits==[60] and not model.blocked


@pytest.mark.parametrize('status,error',[(500,None),(503,{'code':503,'status':'UNAVAILABLE','message':'unknown upstream failure'})])
def test_unrecognized_failure_still_stops_spending(monkeypatch,status,error):
    from omnisim.control_bench import agents
    model=Model('test','model','profile',1,{'input':1,'cached':1,'output':1})
    monkeypatch.setattr(model.session,'post',lambda *a,**kw:model_reply(status,error))
    with pytest.raises(agents.InfrastructureError): model.compile([],'mobile')
    assert model.blocked and model.records[0]['derived_usd'] is None
    assert len(model.records)==1


def test_zero_cost_overload_requires_exact_policy_and_error():
    from omnisim.control_bench import agents
    r={'http':503,'usage':None,'model':None,'text':'','cost_basis':agents.ERROR_COST_BASIS,
       'billing_policy':agents.BILLING_POLICY,
       'provider_error':{'code':503,'status':'UNAVAILABLE','message':agents.OVERLOAD_MESSAGE}}
    assert agents.request_price(r,{})==0
    r['provider_error']['message']='unrecognized'
    assert agents.request_price(r,{}) is None


def bundle(tmp_path,split='holdout'):
    lock={'suite':{'split':split,'tasks':[{'id':'t','family':'f'}]},'arms':['plain'],'repeats':1,'scope':'test'}
    dump(tmp_path/'lock.json',lock)
    row={'arm':'plain','task':'t','robot':'husky','repeat':0,'outcome':'PASS','derived_usd':1,'elapsed_s':2,'unsafe':False}
    (tmp_path/'rows.jsonl').write_text(json.dumps(row)+'\n')
    dump(tmp_path/'completion.json',{'complete':True,'mode':'live','integrity':{'unchanged':True},
         'rows_sha256':sha(tmp_path/'rows.jsonl'),'lock_sha256':sha(tmp_path/'lock.json')})
    return lock,row


def test_verifier_rejects_duplicate_rows(tmp_path):
    _,row=bundle(tmp_path)
    with (tmp_path/'rows.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
    assert not verify(tmp_path)[2]['complete']
    assert not verify(tmp_path)[2]['rows_digest_matches']


def test_modified_lock_is_detected(tmp_path):
    lock,_=bundle(tmp_path); lock['scope']='different'; dump(tmp_path/'lock.json',lock)
    assert not verify(tmp_path)[2]['lock_digest_matches']


def test_offline_verification_regrades_saved_motion(tmp_path):
    from omnisim.paths import REPO_ROOT
    lock,row=bundle(tmp_path)
    name='omnisim/control_bench/tasks.py'
    lock['inputs']={name:sha(REPO_ROOT/name)}
    turn={'prompt':'Drive forward 0.4 metres.','expect':{'path':[drive(.4)]}}
    lock['suite']['tasks'][0].update(robot='husky',turns=[turn])
    snapshot=tmp_path/'source'/name; snapshot.parent.mkdir(parents=True)
    snapshot.write_bytes((REPO_ROOT/name).read_bytes())
    # Evidence says no movement, while the stored grade falsely says PASS.
    row.update(home=[0,0,0],physics={'finalised':True},turns=[{
        'prompt':turn['prompt'],'observation':observation([state(),state()]),
        'grade':{'outcome':'PASS','reasons':[],'unsafe':False}}])
    dump(tmp_path/'lock.json',lock)
    (tmp_path/'rows.jsonl').write_text(json.dumps(row)+'\n')
    finish=json.loads((tmp_path/'completion.json').read_text())
    finish.update(rows_sha256=sha(tmp_path/'rows.jsonl'),lock_sha256=sha(tmp_path/'lock.json'))
    dump(tmp_path/'completion.json',finish)
    result=verify(tmp_path)[2]
    assert result['rows_digest_matches'] and result['source_snapshots_match']
    assert result['regrade_available'] and result['regrade_mismatches']


def test_development_evidence_cannot_publish_claim(tmp_path):
    bundle(tmp_path,'development')
    assert 'development_tasks' in analyse(tmp_path)['claim_blockers']


@pytest.mark.parametrize('corruption',['none','episode_cost','request_cost','request_digest','returned_model','physics_sidecar'])
def test_publication_accounting_recomputed_even_after_rehashing(tmp_path,corruption):
    import hashlib
    lock,row=bundle(tmp_path)
    rates={'input':.25,'cached':.025,'output':1.5}
    body={'model':'model','agentName':'profile'}
    usage={'promptTokenCount':100,'candidatesTokenCount':10,'totalTokenCount':110}
    cost=price(usage,rates)
    req={'request':body,'request_sha256':hashlib.sha256(json.dumps(body,sort_keys=True).encode()).hexdigest(),
         'usage':usage,'derived_usd':cost,'model':'model'}
    lock.update(evidence_checks=2,model='model',profile='profile',rates=rates)
    row.update(mode='live',requests=[req],derived_usd=cost,physics={'finalised':True,'degraded':False})
    engine=tmp_path/'engines'/'00000'; engine.mkdir(parents=True)
    dump(engine/'engine.log.newton.json',row['physics'])
    if corruption=='episode_cost': row['derived_usd']=0
    if corruption=='request_cost': req['derived_usd']=0
    if corruption=='request_digest': body['extra']='tampered'
    if corruption=='returned_model': req['model']='other'
    if corruption=='physics_sidecar': dump(engine/'engine.log.newton.json',{'finalised':False})
    dump(tmp_path/'lock.json',lock)
    (tmp_path/'rows.jsonl').write_text(json.dumps(row)+'\n')
    dump(tmp_path/'completion.json',{'complete':True,'mode':'live','integrity':{'unchanged':True},
         'known_model_usd':cost,'rows_sha256':sha(tmp_path/'rows.jsonl'),'lock_sha256':sha(tmp_path/'lock.json')})
    assert bool(verify(tmp_path)[2]['accounting_mismatches']) == (corruption!='none')


def test_all_bundled_suites_are_development():
    for path in Path(__file__).with_name('suites').glob('*.json'):
        assert load_suite(path)['split']=='development'


class FakeEngine:
    surface='mobile'
    def state(self,objects=False): return state()


class FakeObservation:
    prompt='Drive forward 0.4 metres, retry once only if unavailable.'
    initial=state()
    engine=FakeEngine()
    def __init__(self): self.actions=[]
    def dispatch(self,action):
        self.actions.append(action)
        if len(self.actions)==1: return {'accepted':False,'error':'temporary_unavailable','moved':False}
        return {'accepted':True}


class FakeModel:
    def __init__(self): self.messages=[]
    def compile(self,messages,surface):
        self.messages.append(copy.deepcopy(messages))
        return {'actions':[drive(.4)],'status':'complete','message':'','reported_pose':None}


@pytest.mark.parametrize('arm',['plain','langgraph','lobster'])
def test_actual_runtimes_replan_from_failure_feedback(arm):
    model=FakeModel(); runtime=Runtime(model,arm); obs=FakeObservation()
    try:
        result,_,_=runtime.run(obs,[])
        assert result['status']=='complete'
        assert [x['tool'] for x in obs.actions]==['drive_forward','stop_robot','drive_forward']
        assert len(model.messages)==2
        assert 'temporary_unavailable' in model.messages[1][-1]['content']
    finally: runtime.close()
