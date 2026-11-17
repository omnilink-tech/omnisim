"""Verify an extracted publication archive using Python's standard library only."""
from pathlib import Path
import hashlib
import json
import sys


def main():
    root=Path(__file__).resolve().parent
    manifest=json.loads((root/'MANIFEST.json').read_text(encoding='utf-8'))
    errors=[]
    for name,digest in manifest['sha256'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            errors.append(name+': missing or unsafe path')
        elif hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            errors.append(name+': digest mismatch')
    if errors:
        print(json.dumps({'verified':False,'errors':errors},indent=2)); return 1
    sys.path.insert(0,str(root))
    from omnisim.control_bench.evidence import verify, stats
    evidence=(root/manifest['evidence_directory']).resolve()
    if not evidence.is_relative_to(root): raise ValueError('Unsafe evidence path')
    lock,rows,result=verify(evidence)
    from omnisim.control_bench.agents import price
    # Older archives predate the explicit, disclosed provider-overload estimate.
    if lock.get('evidence_checks',0)>=3:
        from omnisim.control_bench.agents import request_price, is_unbilled_overload
    import math
    accounting=[]
    anthropic=lock.get('provider',{}).get('provider')=='anthropic-direct'
    cached=bool(lock.get('provider',{}).get('cache_policy'))
    if anthropic:
        from omnisim.control_bench.anthropic_model import price as anthropic_price, reservation
    for index,row in enumerate(rows):
        costs=[anthropic_price(req['usage'],lock['rates'],allow_cache_writes=cached) if anthropic else
               request_price(req,lock['rates']) if lock.get('evidence_checks',0)>=3
               else price(req['usage'],lock['rates']) for req in row['requests']]
        if any(c is None for c in costs) or row['derived_usd'] is None or not math.isclose(sum(costs),row['derived_usd'],abs_tol=1e-12):
            accounting.append([index,'episode_cost'])
        for req,cost in zip(row['requests'],costs):
            if cost is None or req['derived_usd'] is None or not math.isclose(cost,req['derived_usd'],abs_tol=1e-12): accounting.append([index,'request_cost'])
            overload=lock.get('evidence_checks',0)>=3 and is_unbilled_overload(req)
            if not overload and req['model']!=lock['model']: accounting.append([index,'model'])
        if row['outcome']!='ERROR':
            proof=json.loads((evidence/'engines'/f'{index:05d}'/'engine.log.newton.json').read_text())
            if proof!=row['physics']: accounting.append([index,'physics'])
    if anthropic:
        try:
            setup=json.loads((evidence/'provider-setup.json').read_text(encoding='utf-8'))['requests'] if cached else []
            requests=setup+[q for row in rows for q in row['requests']]
            journal=[json.loads(s) for s in (evidence/'provider-journal.jsonl').read_text(encoding='utf-8').splitlines()]
            starts=[e for e in journal if e['event']=='reserved']
            ends=[e for e in journal if e['event']=='finished']
            if len(starts)!=len(ends) or len(ends)!=len(requests): accounting.append(['journal_coverage'])
            spent=0
            for i,(before,after,q) in enumerate(zip(starts,ends,requests)):
                if before['index']!=i or after['index']!=i or after['record']!=q or before['record']['request']!=q['request']:
                    accounting.append([i,'journal_record'])
                bound,reserve=reservation(q['request'],q['counted_input_tokens'],lock['rates'])
                if spent+reserve>lock['max_model_usd'] or q['reserved_input_tokens']!=bound:
                    accounting.append([i,'budget_reservation'])
                cost=anthropic_price(q['usage'],lock['rates'],allow_cache_writes=cached)
                if cost is None: accounting.append([i,'unknown_cost']); break
                spent+=cost
            if spent>lock['max_model_usd'] or len(requests)>lock['provider']['max_requests']:
                accounting.append(['campaign_budget'])
        except (KeyError,ValueError,TypeError,OSError): accounting.append(['journal_unreadable'])
    ok=(result['complete'] and result['rows_digest_matches'] and result['lock_digest_matches']
        and result['source_snapshots_match'] and result['regrade_available']
        and not result['regrade_mismatches'] and not result.get('accounting_mismatches') and not accounting
        and result['integrity'].get('unchanged'))
    print(json.dumps({'verified':bool(ok),'files_hashed':len(manifest['sha256']),
          'evidence':result,'additional_accounting_mismatches':accounting,'recomputed_results':{arm:stats([r for r in rows if r['arm']==arm])
          for arm in lock['arms']},},indent=2))
    return 0 if ok else 1


if __name__=='__main__': raise SystemExit(main())
