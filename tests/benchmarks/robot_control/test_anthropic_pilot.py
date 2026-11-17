"""No network: spending guards, provider controls and faithful failure accounting."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from omnisim.control_bench.anthropic_model import AnthropicModel, CONFIG, MODEL, price
from omnisim.control_bench.engine import InfrastructureError


class Reply:
    def __init__(self,data,status=200):
        self.data=data; self.status_code=status; self.ok=status==200
        self.content=json.dumps(data).encode()
    def json(self): return self.data


class Session:
    def __init__(self,reply): self.reply=reply; self.calls=[]
    def post(self,url,**kw):
        self.calls.append((url,kw))
        if url.endswith('/count_tokens'): return Reply({'input_tokens':1000})
        if isinstance(self.reply,Exception): raise self.reply
        return self.reply


def model(tmp_path,**updates):
    response={'model':MODEL,'usage':{'input_tokens':1000,'output_tokens':100},
              'stop_reason':'end_turn','content':[{'type':'text','text':json.dumps(
                  {'actions':[],'status':'clarify','message':'How far?'})}]}
    response.update(updates)
    m=AnthropicModel('sk-ant-unit-test',MODEL,1,copy.deepcopy(CONFIG['rates']),tmp_path/'journal.jsonl')
    m.session=Session(Reply(response)); return m


def test_wire_controls_and_accounting(tmp_path):
    m=model(tmp_path)
    assert m.compile([{'role':'user','content':'hello'}],'mobile')['status']=='clarify'
    body=m.session.calls[-1][1]['json']
    assert body['output_config']=={'effort':'low'} and body['max_tokens']==2048
    assert body['service_tier']=='standard_only'
    assert not {'temperature','thinking','cache_control','tools','speed'} & body.keys()
    assert m.spent==pytest.approx(.006)
    assert m.records[0]['derived_usd']<=m.records[0]['reserved_usd']
    assert 'sk-ant' not in m.journal.read_text()


def test_reserves_output_and_input_before_generating(tmp_path):
    m=model(tmp_path); m.cap=.001
    with pytest.raises(InfrastructureError,match='budget'): m.compile([], 'mobile')
    assert len(m.session.calls)==1 and m.blocked and m.spent==0
    assert not m.records


@pytest.mark.parametrize('usage',[None,{}, {'input_tokens':False,'output_tokens':1},
    {'input_tokens':1,'output_tokens':-1}, {'input_tokens':1,'output_tokens':1,'cache_creation_input_tokens':4}])
def test_unknown_accounting_stops_further_spend(tmp_path,usage):
    m=model(tmp_path,usage=usage)
    with pytest.raises(InfrastructureError): m.compile([], 'arm')
    with pytest.raises(InfrastructureError): m.compile([], 'arm')
    assert len(m.session.calls)==2 and m.records[0]['usage']==usage


def test_output_truncation_is_retained_and_charged(tmp_path):
    m=model(tmp_path,stop_reason='max_tokens',usage={'input_tokens':1000,'output_tokens':2048})
    with pytest.raises(ValueError,match='max_tokens'): m.compile([], 'arm')
    assert m.spent==pytest.approx(.04496) and not m.blocked


def test_wrong_model_halts(tmp_path):
    m=model(tmp_path,model='wrong-model')
    with pytest.raises(InfrastructureError,match='model'): m.compile([], 'arm')
    assert m.blocked and m.spent>0


def test_transport_failure_is_not_retried(tmp_path):
    m=model(tmp_path); m.session.reply=TimeoutError('test')
    with pytest.raises(InfrastructureError): m.compile([], 'mobile')
    with pytest.raises(InfrastructureError): m.compile([], 'mobile')
    assert len(m.session.calls)==2 and m.records[0]['derived_usd'] is None
    events=[json.loads(s) for s in m.journal.read_text().splitlines()]
    assert [e['event'] for e in events]==['reserved','finished']


def test_request_cap_prevents_even_count_query(tmp_path):
    m=model(tmp_path); m.records=[{}]*24
    with pytest.raises(InfrastructureError): m.compile([], 'mobile')
    assert not m.session.calls


def test_cached_input_is_separate_from_uncached_input():
    assert price({'input_tokens':100,'output_tokens':10,'cache_read_input_tokens':1000},CONFIG['rates'])==pytest.approx(.0008)


def test_pilot_budget_cannot_silently_expand(tmp_path):
    with pytest.raises(ValueError,match='budget'):
        AnthropicModel('sk-ant-test',MODEL,1.01,CONFIG['rates'],tmp_path/'bad.jsonl')
    assert not (tmp_path/'bad.jsonl').exists()


def test_comparison_budget_has_independent_bound(tmp_path):
    from omnisim.control_bench.anthropic_model import COMPARISON_CONFIG
    m=AnthropicModel('sk-ant-test',MODEL,5,CONFIG['rates'],tmp_path/'ok.jsonl',COMPARISON_CONFIG)
    assert m.config['max_requests']==450
    with pytest.raises(ValueError,match='budget'):
        AnthropicModel('sk-ant-test',MODEL,5.01,CONFIG['rates'],tmp_path/'bad.jsonl',COMPARISON_CONFIG)


def test_comparison_request_limit_is_enforced(tmp_path):
    from omnisim.control_bench.anthropic_model import COMPARISON_CONFIG
    m=AnthropicModel('sk-ant-test',MODEL,5,CONFIG['rates'],tmp_path/'ok.jsonl',COMPARISON_CONFIG)
    m.session=Session(None); m.records=[{}]*450
    with pytest.raises(InfrastructureError): m.compile([], 'mobile')
    assert not m.session.calls
