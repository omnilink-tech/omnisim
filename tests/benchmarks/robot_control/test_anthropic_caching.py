"""Offline contracts for the separately frozen cached comparison."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from omnisim.control_bench.anthropic_model import AnthropicModel, CONFIG, CACHED_CONFIG, MODEL, price, reservation
from omnisim.control_bench.engine import InfrastructureError


def mixed_usage():
    return {'input_tokens':100,'output_tokens':10,'cache_read_input_tokens':1000,
            'cache_creation_input_tokens':200,
            'cache_creation':{'ephemeral_5m_input_tokens':200,'ephemeral_1h_input_tokens':0}}


def test_three_input_categories_are_billed_separately():
    assert price(mixed_usage(),CONFIG['rates'],True)==pytest.approx(.0018)
    assert price(mixed_usage(),CONFIG['rates']) is None


@pytest.mark.parametrize('creation',[{}, {'ephemeral_5m_input_tokens':199},
    {'ephemeral_5m_input_tokens':200,'ephemeral_1h_input_tokens':1},
    {'ephemeral_5m_input_tokens':True}, {'ephemeral_5m_input_tokens':-1}, None])
def test_unknown_or_wrong_ttl_is_not_guessed(creation):
    usage=mixed_usage();usage['cache_creation']=creation
    assert price(usage,CONFIG['rates'],True) is None


def test_full_hit_and_cold_write():
    assert price({'input_tokens':0,'output_tokens':0,'cache_read_input_tokens':1000},CONFIG['rates'],True)==pytest.approx(.0002)
    u={'input_tokens':0,'output_tokens':0,'cache_creation_input_tokens':1000,
       'cache_creation':{'ephemeral_5m_input_tokens':1000,'ephemeral_1h_input_tokens':0}}
    assert price(u,CONFIG['rates'],True)==pytest.approx(.005)


def cached(tmp_path):
    return AnthropicModel('sk-ant-test',MODEL,5,CONFIG['rates'],tmp_path/'journal.jsonl',CACHED_CONFIG)


def test_cache_controls_preserve_prompt_and_model_settings(tmp_path):
    m=cached(tmp_path)
    b=m.body([{'role':'user','content':'hello'}],'arm')
    plain=AnthropicModel('sk-ant-test',MODEL,1,CONFIG['rates'],tmp_path/'plain.jsonl')
    original=plain.body(b['messages'],'arm')
    assert b['system'][0]['text']==original['system']
    assert b['cache_control']==b['system'][0]['cache_control']=={'type':'ephemeral','ttl':'5m'}
    assert b['max_tokens']==2048 and b['output_config']=={'effort':'low'}
    bound,cost=reservation(b,1000,CONFIG['rates'])
    assert cost==pytest.approx((bound*5+2048*20)/1e6)


class Reply:
    ok=True; status_code=200
    def __init__(self,data): self.data=data;self.content=json.dumps(data).encode()
    def json(self): return self.data


class FakeProvider:
    def __init__(self): self.calls=[]
    def post(self,url,**kw):
        self.calls.append((url,kw))
        if url.endswith('/count_tokens'): return Reply({'input_tokens':1000})
        body=kw['json']; warm=body['max_tokens']==0
        usage={'input_tokens':100,'output_tokens':0 if warm else 10,'cache_read_input_tokens':0 if warm else 600,
               'cache_creation_input_tokens':600 if warm else 100,
               'cache_creation':{'ephemeral_5m_input_tokens':600 if warm else 100,'ephemeral_1h_input_tokens':0}}
        return Reply({'model':MODEL,'usage':usage,'stop_reason':'max_tokens' if warm else 'end_turn',
                      'content':[] if warm else [{'type':'text','text':'{"actions":[],"status":"clarify","message":"How far?"}'}]})


def test_warmup_is_metered_and_has_zero_output(tmp_path):
    m=cached(tmp_path);m.session=FakeProvider()
    m.warmup({'mobile','arm'})
    assert len(m.records)==2 and m.spent==pytest.approx(.0068)
    assert [q['purpose'] for q in m.records]==['common_cache_warmup:arm','common_cache_warmup:mobile']
    for q in m.records:
        assert q['request']['max_tokens']==0 and 'cache_control' not in q['request']
        assert q['usage']['output_tokens']==0
        assert q['reserved_usd']>=q['derived_usd']
    m.compile([{'role':'user','content':'hello'}],'mobile')
    assert len(m.records)==3 and m.records[-1]['purpose']=='scored'
    assert m.spent==pytest.approx(.0068+.00122)
    assert 'sk-ant' not in m.journal.read_text()


def test_warmup_is_subject_to_the_same_budget(tmp_path):
    m=cached(tmp_path);m.cap=.001;m.session=FakeProvider()
    with pytest.raises(InfrastructureError,match='budget'):m.warmup({'mobile'})
    assert m.spent==0 and len(m.session.calls)==1 and m.blocked


def test_warmup_without_a_real_cache_write_or_hit_stops(tmp_path):
    m=cached(tmp_path)
    class NoCache(FakeProvider):
        def post(self,url,**kw):
            r=super().post(url,**kw)
            if not url.endswith('/count_tokens'):
                r.data['usage']={'input_tokens':100,'output_tokens':0,
                                 'cache_read_input_tokens':0,'cache_creation_input_tokens':0}
            return r
    m.session=NoCache()
    with pytest.raises(InfrastructureError,match='did not cache'):m.warmup({'mobile'})
    assert m.blocked and m.spent>0 and len(m.records)==1


@pytest.mark.parametrize('corruption',['none','setup_cost','missing_setup','cache_disabled','wrong_ttl','double_count'])
def test_offline_verifier_includes_warmup_and_cache_charges(tmp_path,corruption):
    import hashlib
    from omnisim.control_bench.evidence import dump,sha,verify
    m=cached(tmp_path);m.session=FakeProvider();m.warmup({'mobile'})
    m.compile([{'role':'user','content':'hello'}],'mobile')
    setup,q=m.records
    lock={'suite':{'split':'development','tasks':[{'id':'t','family':'f','robot':'husky'}]},
          'arms':['plain'],'repeats':1,'scope':'test','provider':CACHED_CONFIG,
          'evidence_checks':3,'model':MODEL,'rates':CONFIG['rates'],'profile':'test'}
    row={'arm':'plain','task':'t','robot':'husky','repeat':0,'mode':'live','outcome':'PASS',
         'derived_usd':q['derived_usd'],'elapsed_s':1,'unsafe':False,'requests':[q],
         'physics':{'finalised':True,'degraded':False}}
    engine=tmp_path/'engines/00000';engine.mkdir(parents=True)
    dump(engine/'engine.log.newton.json',row['physics'])
    if corruption=='setup_cost':setup['derived_usd']=0
    if corruption=='cache_disabled':q['request'].pop('cache_control')
    if corruption=='wrong_ttl':q['usage']['cache_creation']['ephemeral_1h_input_tokens']=100
    if corruption=='double_count':q['derived_usd']+=.004;row['derived_usd']=q['derived_usd']
    q['request_sha256']=hashlib.sha256(json.dumps(q['request'],sort_keys=True).encode()).hexdigest()
    if corruption!='missing_setup':dump(tmp_path/'provider-setup.json',{'requests':[setup]})
    dump(tmp_path/'lock.json',lock)
    (tmp_path/'rows.jsonl').write_text(json.dumps(row)+'\n',encoding='utf-8')
    dump(tmp_path/'completion.json',{'complete':True,'mode':'live','integrity':{'unchanged':True},
         'known_model_usd':m.spent,'rows_sha256':sha(tmp_path/'rows.jsonl'),'lock_sha256':sha(tmp_path/'lock.json')})
    assert bool(verify(tmp_path)[2]['accounting_mismatches'])==(corruption!='none')
