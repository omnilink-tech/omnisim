"""Offline journal/budget audit and honest report for a matched Anthropic run."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from omnisim.control_bench.anthropic_model import CONFIG, COMPARISON_CONFIG, CACHED_CONFIG, price, reservation
from omnisim.control_bench.evidence import verify, stats, paired, sha, dump


def audit(directory, baseline, keys=()):
    lock,rows,checks=verify(directory)
    assert lock['provider'] in (COMPARISON_CONFIG,CACHED_CONFIG)
    cached=lock['provider']==CACHED_CONFIG
    assert checks['rows_digest_matches'] and checks['lock_digest_matches']
    assert checks['source_snapshots_match'] and checks['regrade_available']
    assert not checks['regrade_mismatches']
    assert checks['integrity']['unchanged']
    if checks['accounting_mismatches']:
        raise ValueError('Accounting does not verify: '+json.dumps(checks['accounting_mismatches']))
    assert len({(r['task'],r['arm'],r['repeat']) for r in rows})==len(rows)
    old=json.loads((baseline/'lock.json').read_text(encoding='utf-8'))
    products={p:h for p,h in old['inputs'].items() if not p.startswith('omnisim/control_bench/')}
    assert all(lock['inputs'].get(p)==h for p,h in products.items())
    assert lock['suite']['tasks']==old['suite']['tasks']
    assert set(lock['arms'])==set(old['arms']) and lock['repeats']==1
    protocol=lock['protocol']; assert sha(ROOT/protocol['path'])==protocol['sha256']
    journal=[json.loads(s) for s in (directory/'provider-journal.jsonl').read_text(encoding='utf-8').splitlines()]
    starts=[e for e in journal if e['event']=='reserved']
    ends=[e for e in journal if e['event']=='finished']
    not_sent=[e for e in journal if e['event']=='not_sent']
    setup=json.loads((directory/'provider-setup.json').read_text(encoding='utf-8'))['requests'] if cached else []
    requests=[*setup,*[q for r in rows for q in r['requests']]]
    assert len(starts)==len(ends)==len(requests)<=lock['provider']['max_requests']
    spent=0; unknown=[]
    for i,(start,end,q) in enumerate(zip(starts,ends,requests)):
        assert start['index']==end['index']==i
        assert start['record']['request']==q['request'] and end['record']==q
        bound,reserve=reservation(q['request'],q['counted_input_tokens'],lock['rates'])
        assert math.isclose(q['reserved_usd'],reserve,abs_tol=1e-12)
        assert spent+reserve<=lock['max_model_usd']
        cost=price(q['usage'],lock['rates'],allow_cache_writes=cached)
        if cost is None:
            unknown.append(i)
            assert i==len(requests)-1  # unknown billing must stop subsequent calls
            break
        assert math.isclose(cost,q['derived_usd'],abs_tol=1e-12)
        assert cost<=reserve and q['response']['usage']==q['usage']
        assert q['response']['model']==lock['model']
        system=q['request']['system'][0]['text'] if cached else q['request']['system']
        assert system.startswith('You control a robot')
        assert q['request']['output_config']=={'effort':'low'}
        warm=i<len(setup)
        assert q['request']['max_tokens']==(0 if warm else 2048) and q['request']['service_tier']=='standard_only'
        assert not {'tools','speed','temperature'} & q['request'].keys()
        if cached:
            control={'type':'ephemeral','ttl':'5m'}
            assert q['request']['system'][0]['cache_control']==control
            assert q['request'].get('cache_control')==(None if warm else control)
        else: assert 'cache_control' not in q['request']
        spent+=cost
    assert spent<=lock['max_model_usd']
    for keypath in keys:
        secret=keypath.read_text(encoding='utf-8-sig').strip().encode()
        assert secret
        for p in directory.rglob('*'):
            if p.is_file() and secret in p.read_bytes(): raise ValueError('Credential in '+str(p))
    summary={arm:stats([r for r in rows if r['arm']==arm]) for arm in lock['arms']}
    shared_setup=sum(q['derived_usd'] for q in setup)
    scored_uncached_equivalent=sum((q['usage']['input_tokens']+q['usage'].get('cache_read_input_tokens',0)
        +q['usage'].get('cache_creation_input_tokens',0))*lock['rates']['input']/1e6
        +q['usage']['output_tokens']*lock['rates']['output']/1e6 for q in requests[len(setup):] if q['usage'])
    cache_counts={a:{k:sum(q['usage'].get(k,0) for r in rows if r['arm']==a for q in r['requests'])
                    for k in ('input_tokens','cache_read_input_tokens','cache_creation_input_tokens','output_tokens')}
                  for a in lock['arms']}
    observed={(r['task'],r['arm'],r['repeat']) for r in rows}
    order=[(t['id'],arm,0) for t in lock['suite']['tasks'] for arm in lock['arms']]
    random.Random(lock['seed']).shuffle(order)
    remaining=[{'task':task,'arm':arm,'repeat':repeat} for task,arm,repeat in order if (task,arm,repeat) not in observed]
    if remaining:
        dump(directory/'REMAINING_EPISODES.json',{'parent_lock_sha256':sha(directory/'lock.json'),
            'parent_rows_sha256':sha(directory/'rows.jsonl'),
            'purpose':'Inventory only, no authorization to spend or rerun completed episodes',
            'remaining_in_original_order':remaining})
    failures=[{'arm':r['arm'],'task':r['task'],'outcome':r['outcome'],
               'reasons':[s for t in r['turns'] for s in t['grade']['reasons']],
               'error':r.get('error')} for r in rows if r['outcome']!='PASS']
    result={'audited_utc':datetime.now(timezone.utc).isoformat(),'complete':checks['complete'],
            'planned':checks['planned'],'observed':len(rows),'known_model_usd':spent,
            'remaining_episodes':len(remaining),
            'budget_usd':lock['max_model_usd'],'generation_requests':len(requests),
            'scored_model_requests':len(requests)-len(setup),'warmup_requests':len(setup),
            'preflight_not_sent':len(not_sent),'unknown_usage_requests':unknown,
            'input_tokens':sum(q['usage']['input_tokens'] for q in requests if q['usage']),
            'cache_read_input_tokens':sum(q['usage'].get('cache_read_input_tokens',0) for q in requests if q['usage']),
            'cache_creation_input_tokens':sum(q['usage'].get('cache_creation_input_tokens',0) for q in requests if q['usage']),
            'shared_cache_setup_usd':shared_setup,'cache_counts_by_arm':cache_counts,
            'uncached_list_price_equivalent_for_recorded_scored_tokens_usd':scored_uncached_equivalent,
            'cache_cost_saving_against_same_token_equivalent_usd':scored_uncached_equivalent-spent if cached else None,
            'usd_per_success_with_equal_setup_allocation':{a:(s['derived_usd']+shared_setup/len(lock['arms']))/s['pass']
                if s['pass'] and s['derived_usd'] is not None else None for a,s in summary.items()},
            'output_tokens':sum(q['usage']['output_tokens'] for q in requests if q['usage']),
            'truncated_responses':sum(q.get('stop_reason')=='max_tokens' for q in requests[len(setup):]),
            'returned_models':sorted({q['model'] for q in requests if q['model']}),
            'summary':summary,'failures':failures,'verification':checks,
            'product_fixture_hashes_match':len(products),'keys_scanned':len(keys),
            'journal_reconciles':True,'rows_sha256':sha(directory/'rows.jsonl'),
            'journal_sha256':sha(directory/'provider-journal.jsonl'),
            'comparisons':{a:paired(rows,a,lock['suite']) for a in lock['arms'] if a!='omnilink_parser'} if checks['complete'] else {},
            'leadership_claim_allowed':False}
    dump(directory/'ANTHROPIC_AUDIT.json',result)
    lines=['# Matched Opus robotics-control comparison','',
        f"Coverage: **{len(rows)}/{checks['planned']} episodes**. Complete: **{checks['complete']}**.",
        f"New model spending: **${spent:.6f}** from {len(requests)} generation requests, within the ${lock['max_model_usd']:.2f} ceiling.",
        '', 'All six integrations use Opus 5.5 at low effort with identical direct-provider settings, system instruction, controller, grading rules and task definitions. No production model switch or website publication is performed.',
        '', '| Implementation | Attempted | PASS | FAIL | ERROR | Unwanted motion | Model USD |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for a,s in summary.items():
        cost='unknown' if s['derived_usd'] is None else f"{s['derived_usd']:.6f}"
        lines.append(f"| {a} | {s['episodes']} | {s['pass']} | {s['fail']} | {s['error']} | {s['unsafe']} | {cost} |")
    if cached:
        lines+=['',f'Shared cache setup: ${shared_setup:.6f}, included in the campaign total above. Per-arm table costs are scored calls only. ANTHROPIC_AUDIT.json also shows each arm with one-sixth of setup cost allocated.',
                '',f"Cache-read tokens: {result['cache_read_input_tokens']}; five-minute cache-write tokens: {result['cache_creation_input_tokens']}. Counters by implementation are retained in the audit JSON.",
                '',f'At uncached list rates, the recorded scored input/output token volume would cost ${scored_uncached_equivalent:.6f}. Actual cached usage including warm-up costs ${spent:.6f}. This is a rate-based calculation on observed tokens, not a separate measured uncached rerun or invoice reconciliation.']
    if not checks['complete']:
        lines+=['','**Incomplete: these counts have unequal task coverage and must not be ranked or presented as a completed benchmark.** The frozen stopping rule was honored; no tasks were dropped to improve scores.']
    else:
        lines+=['','## Paired results','',
            '| OmniLink versus | Success difference (pp) | Cost/success ratio | 95% family bootstrap cost interval |',
            '|---|---:|---:|---|']
        for a,p in result['comparisons'].items():
            point=p['point']; interval=p['family_bootstrap_95'].get('cost_per_success_ratio')
            ratio=point['cost_per_success_ratio']
            lines.append(f"| {a} | {point['success_difference_pp']:.2f} | {ratio:.3f} | {interval} |")
    lines+=['','## Failures and errors','']
    if not failures: lines.append('None observed in the attempted episodes.')
    for f in failures: lines.append(f"- {f['arm']} / {f['task']}: {f['outcome']}; {', '.join(f['reasons']) or f['error']}")
    lines+=['','## Scope and next step','',
        'This is a previously seen, developer-authored development suite: 33 tasks, 19 families, three simulated robots, one repeat. Assisted manipulation remains assisted. These are controlled integrations, not the complete hosted products or every competitor in the market. A full pass or cost difference does not demonstrate market-wide capability leadership.',
        '', 'Historical Gemini results use a different hosted transport and prompt wrapper. Treat cross-model differences as descriptive configuration differences. The matched comparison within this run gives every implementation the same Opus model and settings.',
        '', 'Keep the existing public claims unchanged. If incomplete, first decide whether finishing the preregistered comparison warrants a separately approved budget. If complete, use the paired results to decide what product capability needs improvement and validate on an independently reviewed, harder task set before a leadership claim.',
        '', '## Verification','',
        f"All saved physical grades reproduce; source snapshots, frozen protocol, robot assets and runtime hashes verify. {len(requests)} paid requests reconcile against their durable reservations and returned usage. Input tokens {result['input_tokens']}; output tokens including thinking {result['output_tokens']}; truncated replies {result['truncated_responses']}. No keys found in the {len(keys)} credential scans.",
        '', 'See ANTHROPIC_AUDIT.json, scorecard.json, rows.jsonl, provider-journal.jsonl and per-episode physics sidecars in this directory. Recompute with `python tests/benchmarks/robot_control/audit_anthropic_campaign.py <evidence-directory>`. Dollar amounts use published standard token rates, not invoice or total-platform-cost accounting.']
    (directory/'MATCHED_COMPARISON.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--baseline',type=Path,default=ROOT/'tests/benchmarks/robot_control/evidence/publication-04')
    p.add_argument('--key-file',type=Path,action='append',default=[])
    a=p.parse_args(); result=audit(a.directory,a.baseline,a.key_file)
    print(json.dumps({k:result[k] for k in ('complete','observed','planned','known_model_usd','generation_requests','summary','failures')},indent=2))
