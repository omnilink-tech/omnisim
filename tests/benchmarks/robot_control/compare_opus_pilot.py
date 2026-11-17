"""Recompute the small model pilot comparison without any provider calls."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from omnisim.control_bench.evidence import verify, stats, sha, dump


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot',type=Path,default=ROOT/'tests/benchmarks/robot_control/evidence/opus-pilot-01')
    parser.add_argument('--baseline',type=Path,default=ROOT/'tests/benchmarks/robot_control/evidence/publication-04')
    parser.add_argument('--key-file',type=Path,action='append',default=[])
    args=parser.parse_args()
    lock,rows,checks=verify(args.pilot)
    base,old,base_checks=verify(args.baseline)
    # Windows' default text encoding is not UTF-8. Compare the actual recorded
    # Unicode text (including degree symbols) against the UTF-8 journal.
    rows=[json.loads(s) for s in (args.pilot/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
    old=[json.loads(s) for s in (args.baseline/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
    for c in (checks,base_checks):
        assert c['complete'] and c['rows_digest_matches'] and c['lock_digest_matches']
        assert c['source_snapshots_match'] and c['regrade_available'] and not c['regrade_mismatches']
        assert not c['accounting_mismatches'] and c['integrity']['unchanged']
    ids={t['id'] for t in lock['suite']['tasks']}
    tasks={t['id']:t for t in base['suite']['tasks']}
    assert all(tasks[t['id']]==t for t in lock['suite']['tasks'])
    old=[r for r in old if r['task'] in ids and r['arm']=='omnilink_parser' and r['repeat']==0]
    assert len(old)==len(rows)==len(ids)
    product={p:h for p,h in base['inputs'].items() if not p.startswith('omnisim/control_bench/')}
    assert all(lock['inputs'].get(p)==h for p,h in product.items())
    # Prove all attempted paid calls are present exactly once in the rows.
    journal=[json.loads(s) for s in (args.pilot/'provider-journal.jsonl').read_text(encoding='utf-8').splitlines()]
    reserved=[e for e in journal if e['event']=='reserved']
    finished=[e for e in journal if e['event']=='finished']
    requests=[q for r in rows for q in r['requests']]
    assert len(reserved)==len(finished)==len(requests)<=lock['provider']['max_requests']
    spent=0
    for i,(before,after,q) in enumerate(zip(reserved,finished,requests)):
        assert before['index']==after['index']==i
        assert before['record']['request']==q['request'] and after['record']==q
        assert spent+before['record']['reserved_usd']<=lock['max_model_usd']
        assert q['http']==200 and q['response']['model']==lock['model']
        assert q['usage']==q['response']['usage']
        spent+=q['derived_usd']
    assert spent<=1
    for keypath in args.key_file:
        key=keypath.read_text(encoding='utf-8-sig').strip().encode()
        assert key
        for p in args.pilot.rglob('*'):
            if p.is_file() and key in p.read_bytes(): raise ValueError('Credential found in evidence: '+str(p))
    old_by_id={r['task']:r for r in old}
    a,b=stats(rows),stats(old)
    table=[]
    for r in rows:
        prev=old_by_id[r['task']]
        table.append({'task':r['task'],'opus':r['outcome'],'gemini':prev['outcome'],
                      'opus_usd':r['derived_usd'],'gemini_usd':prev['derived_usd'],
                      'opus_parser_hits':r['parser_hits'],'gemini_parser_hits':prev['parser_hits'],
                      'opus_reasons':[s for t in r['turns'] for s in t['grade']['reasons']]})
    result={'purpose':'Exploratory diagnostic model comparison, not a publishable leadership claim',
            'opus':a,'historical_gemini':b,'tasks':table,'verification':checks,
            'baseline_verification':base_checks,'all_paid_requests_reconciled':True,
            'credentials_scanned':len(args.key_file),'product_fixture_hashes_match':len(product),
            'input_tokens':sum(q['usage']['input_tokens'] for q in requests),
            'output_tokens':sum(q['usage']['output_tokens'] for q in requests),
            'truncated_responses':sum(q['stop_reason']=='max_tokens' for q in requests),
            'pilot_rows_sha256':sha(args.pilot/'rows.jsonl'),
            'baseline_rows_sha256':sha(args.baseline/'rows.jsonl')}
    dump(args.pilot/'model-comparison.json',result)
    lines=['# Opus 5.5 low-effort diagnostic pilot','',
        f"Opus completed **{a['pass']}/{a['episodes']}**; historical Gemini 3.5 Flash completed **{b['pass']}/{b['episodes']}** on these same tasks.",
        f"New Opus list-price model usage: **${spent:.6f}**, from {a['model_calls']} generation requests; budget $1. No new Gemini requests.",'',
        '| Metric | Opus 5.5 low | Historical Gemini 3.5 Flash |',
        '|---|---:|---:|',
        f"| PASS | {a['pass']}/{a['episodes']} | {b['pass']}/{b['episodes']} |",
        f"| Unwanted-motion episodes | {a['unsafe']} | {b['unsafe']} |",
        f"| Model requests | {a['model_calls']} | {b['model_calls']} |",
        f"| Total model USD | {a['derived_usd']:.6f} | {b['derived_usd']:.6f} |",
        f"| Model USD / successful episode | {a['usd_per_success']:.6f} | {b['usd_per_success']:.6f} |",'',
        '| Task | Opus | Gemini | Opus USD | Gemini USD |', '|---|---|---|---:|---:|']
    for t in table:
        lines.append(f"| {t['task']} | {t['opus']} | {t['gemini']} | {t['opus_usd']:.6f} | {t['gemini_usd']:.6f} |")
    lines+=['','## Interpretation limits','',
        'These are nine deliberately selected development tasks, including the known Gemini arm ambiguity failure. One observation per task is not a reliable estimate of general capability or stochastic repeatability. Tasks handled entirely by the shared parser do not measure the model.',
        '', 'Robot/product inputs and task definitions match the earlier run. The transport differs: direct Anthropic with the common benchmark prompt versus the hosted Gemini connector, which builds its own system wrapper. Output/effort settings are not matched across providers. Attribute results to these tested configurations, not solely to model quality or framework design. Historical wall times are not a controlled latency comparison.',
        '', 'No competitor framework was rerun with Opus. No market leadership, hardware, general robot capability, or total-platform-cost conclusion follows. Assisted manipulation remains assisted.',
        '', '## Evidence checks','',
        f"All {len(rows)} physical grades reproduce, source snapshots and input hashes verify, and every one of {len(requests)} paid requests reconciles with its durable preflight reservation and final usage. Credentials scanned: {len(args.key_file)}. Input tokens: {result['input_tokens']}; output tokens including thinking: {result['output_tokens']}. Truncated responses: {result['truncated_responses']}.",
        '', 'Recompute: `python tests/benchmarks/robot_control/compare_opus_pilot.py`. Protocol: `tests/benchmarks/robot_control/OPUS_PILOT_01.md`. Raw traces and separate physics sidecars are in this directory. Dollar figures are derived from standard published rates, not billing invoices.']
    (args.pilot/'MODEL_COMPARISON.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'opus':a,'historical_gemini':b,'verification':'passed','report':str(args.pilot/'MODEL_COMPARISON.md')},indent=2))


if __name__=='__main__': main()
