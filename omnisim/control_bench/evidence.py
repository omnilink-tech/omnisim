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
"""Freeze, append-only evidence, paired analysis and publication eligibility."""
from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import random
import statistics
import subprocess
from datetime import datetime, timezone

from omnisim.paths import REPO_ROOT, resolve_omnisim_binary
from .tasks import ROBOTS, load_suite, grade_turn


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def tracked_inputs():
    """Source, world/robot assets, runtime binary and controller ABI artifact."""
    paths=set(Path(__file__).parent.glob('*.py')) | set(Path(__file__).parent.glob('*.mjs'))
    dirs=['packages/omnisim-bridges/src/omnisim_bridges',
          'projects/samples/demos/controllers/omnilink_mobile_bridge',
          'projects/samples/demos/controllers/omnilink_arm_bridge',
          'projects/samples/demos/controllers/_omnilink_relay',
          'projects/robots/clearpath/husky_description',
          'projects/robots/robotis/turtlebot3_description',
          'projects/robots/omnisim/omniarm6',
          'tests/benchmarks/harness_comparison/vendor/lobster/dist/src']
    for directory in dirs:
        for p in (REPO_ROOT/directory).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix.lower() in ('.py','.js','.json','.urdf','.obj','.stl','.dae','.proto','.mtl'):
                paths.add(p)
    for robot in ROBOTS.values(): paths.add(REPO_ROOT/'projects/samples/demos/worlds/chat'/robot['world'])
    paths.add(REPO_ROOT/'projects/samples/demos/protos/OmniLinkStage.proto')
    for p in (REPO_ROOT/'lib/controller').glob('*'):
        if p.suffix in ('.dll','.so','.dylib'): paths.add(p)
    binary=resolve_omnisim_binary()
    if binary:
        paths.add(Path(binary))
        runtime=Path(binary).parent/'newton-runtime'
        for name in ('omnisim_newton_runtime.py','python.exe'):
            if (runtime/name).is_file(): paths.add(runtime/name)
    return {str(p.relative_to(REPO_ROOT)).replace('\\','/'):sha(p) for p in sorted(paths)}


def dependencies():
    out={}
    for name in ('langgraph','langchain-core','requests','psutil'):
        try: out[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: out[name]=None
    try: out['node']=subprocess.check_output(['node','--version'],text=True).strip()
    except (OSError,subprocess.SubprocessError): out['node']=None
    return out


def freeze(suite_path,lock_path,arms,repeats,cap,model,profile,rates,provider=None):
    lock_path=Path(lock_path)
    if lock_path.exists(): raise ValueError('Refusing to overwrite a freeze')
    suite=load_suite(suite_path)
    deps=dependencies()
    required=['psutil']
    if arms!=['oracle']: required.append('requests')
    if any(a.startswith('langgraph') for a in arms): required+=['langgraph','langchain-core']
    if any(a.startswith('lobster') for a in arms):
        required.append('node')
        if not (REPO_ROOT/'tests/benchmarks/harness_comparison/vendor/lobster/dist/src/sdk/index.js').is_file():
            raise ValueError('Build the pinned Lobster SDK before freezing')
    if any(not deps[n] for n in required): raise ValueError('Missing dependencies: '+str(required))
    from projects.policies.common.env_fingerprint import collect
    fp=collect(repo_root=REPO_ROOT,engine_log_path=REPO_ROOT/'.tmp/control-bench-no-running-engine.log')
    try: revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO_ROOT,text=True).strip()
    except subprocess.SubprocessError: revision=None
    lock={'schema':'omnisim-control-bench-lock/1','evidence_checks':3,'created_utc':datetime.now(timezone.utc).isoformat(),
          'suite':suite,'suite_sha256':sha(suite_path),'arms':arms,'repeats':repeats,
          'seed':240926,'max_model_usd':cap,'model':model,'profile':profile,'rates':rates,
          'model_transport':'shared OmniLink API; requested temperature/output cap are not certified provider controls',
          'transport_policy':{'recognized_overload_attempts':2,'retry_wait_min_s':60,'retry_wait_max_s':120,
              'error_cost_basis':'google_5xx_no_token_charge_assumption_v1',
              'billing_source':'https://ai.google.dev/gemini-api/docs/billing#am-i-charged-for-failed-requests',
              'billing_checked':'2026-09-24','unrecognized_missing_usage':'stop'},
          'scope':'controlled parser/framework integrations, not complete hosted products',
          'inputs':tracked_inputs(),'dependencies':deps,'git_revision':revision,
          'machine':{k:fp[k] for k in ('machine','gpu','os','python','stack','knobs','intent')},
          'decision':{'bootstrap_draws':2000,'cluster':'task family across robots/poses/repeats',
                      'noninferiority_pp':-3,'required_unwanted_motion_episodes':0,
                      'latency':'all-attempt elapsed seconds per successful episode',
                      'cost':'all-attempt derived model USD per successful episode'}}
    if provider:
        lock['provider']=provider
        lock['model_transport']='Direct Anthropic API; exploratory model pilot, not deployed OmniLink chat transport'
        lock['transport_policy']=provider
        lock['scope']='exploratory development-task model pilot; not a platform leadership comparison'
        if provider.get('comparison'):
            lock['scope']='matched Opus 5.5 low-effort controlled parser/framework comparison on development tasks; not complete hosted products'
            protocol='tests/benchmarks/robot_control/OPUS_COMPARISON_01.md'
            if provider.get('cache_policy'):
                protocol='tests/benchmarks/robot_control/OPUS_CACHED_COMPARISON_01.md'
                lock['scope']='matched Opus 5.5 low-effort cached parser/framework comparison on development tasks; not complete hosted products'
            lock['protocol']={'path':protocol,'sha256':sha(REPO_ROOT/protocol)}
            lock['inputs'][protocol]=sha(REPO_ROOT/protocol)
    lock_path.parent.mkdir(parents=True,exist_ok=True); dump(lock_path,lock)
    return lock


def check_inputs(lock):
    changed=[name for name,digest in lock['inputs'].items()
             if not (REPO_ROOT/name).is_file() or sha(REPO_ROOT/name)!=digest]
    dep=dependencies()
    return {'unchanged':not changed and dep==lock['dependencies'], 'changed':changed,
            'dependencies_unchanged':dep==lock['dependencies']}


def planned(lock):
    return {(arm,t['id'],repeat) for arm in lock['arms'] for t in lock['suite']['tasks']
            for repeat in range(lock['repeats'])}


def stats(rows):
    passed=sum(r['outcome']=='PASS' for r in rows)
    costs=[r.get('derived_usd') for r in rows]
    elapsed=sum(r.get('elapsed_s',0) for r in rows)
    known=all(c is not None for c in costs)
    cost=sum(costs) if known else None
    return {'episodes':len(rows),'pass':passed,'fail':sum(r['outcome']=='FAIL' for r in rows),
            'error':sum(r['outcome']=='ERROR' for r in rows),
            'success_rate':passed/len(rows) if rows else None,
            'unsafe':sum(bool(r.get('unsafe')) for r in rows),
            'unknown_safety':sum(r.get('unsafe') is None for r in rows),
            'derived_usd':cost,'usd_per_success':cost/passed if passed and known else None,
            'elapsed_s':elapsed,'seconds_per_success':elapsed/passed if passed else None,
            'model_calls':sum(len(r.get('requests',[])) for r in rows)}


def paired(rows,baseline,suite):
    lookup={(r['arm'],r['task'],r['repeat']):r for r in rows}
    families={t['id']:t['family'] for t in suite['tasks']}
    blocks=defaultdict(list)
    for key,r in lookup.items():
        if key[0]!='omnilink_parser': continue
        b=lookup.get((baseline,key[1],key[2]))
        if b is not None: blocks[families[r['task']]].append((r,b))
    def aggregate(pairs):
        a,b=stats([p[0] for p in pairs]),stats([p[1] for p in pairs])
        def ratio(key):
            x,y=a[key],b[key]
            return x/y if x is not None and y is not None and y>0 else None
        return {'success_difference_pp':100*(a['success_rate']-b['success_rate']),
                'cost_per_success_ratio':ratio('usd_per_success'),
                'time_per_success_ratio':ratio('seconds_per_success')}
    if not blocks: return {}
    values=[p for block in blocks.values() for p in block]
    point=aggregate(values); rng=random.Random(240926); draws=defaultdict(list)
    keys=list(blocks)
    for _ in range(2000):
        sample=aggregate([p for key in rng.choices(keys,k=len(keys)) for p in blocks[key]])
        for k,v in sample.items():
            if v is not None: draws[k].append(v)
    intervals={k:[sorted(v)[int(.025*(len(v)-1))],sorted(v)[int(.975*(len(v)-1))]] for k,v in draws.items()}
    return {'pairs':len(values),'independent_families':len(blocks),'point':point,'family_bootstrap_95':intervals,
            'valid_bootstrap_draws':{k:len(v) for k,v in draws.items()}}


def verify(directory):
    directory=Path(directory)
    lock=json.loads((directory/'lock.json').read_text(encoding='utf-8'))
    rows=[json.loads(l) for l in (directory/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
    keys=[(r['arm'],r['task'],r['repeat']) for r in rows]
    finish=json.loads((directory/'completion.json').read_text()) if (directory/'completion.json').exists() else {}
    integrity=finish.get('integrity',{})
    complete=(set(keys)==planned(lock) and len(keys)==len(set(keys)) and finish.get('complete') is True)
    row_hash=sha(directory/'rows.jsonl')
    inputs=lock.get('inputs',{})
    snapshots=[name for name in inputs if Path(name).suffix in ('.py','.mjs','.json','.omniworld','.proto','.urdf','.md')]
    source_ok=bool(snapshots) and all((directory/'source'/name).is_file() and sha(directory/'source'/name)==inputs[name] for name in snapshots)
    # Regrading requires the installed oracle to match the frozen hash. Never
    # execute arbitrary archived code or silently apply a newer scoring rule.
    oracle='omnisim/control_bench/tasks.py'
    regrade_available=inputs.get(oracle)==sha(REPO_ROOT/oracle)
    mismatches=[]
    if regrade_available:
        tasks={t['id']:t for t in lock['suite']['tasks']}
        for r in rows:
            key=[r['arm'],r['task'],r['repeat']]
            try:
                task=tasks[r['task']]; grades=[]
                if len(r.get('turns',[]))>len(task['turns']): mismatches.append(key+['extra_turns'])
                if any(r.get(k)!=task.get(k) for k in ('robot','family','category')):
                    mismatches.append(key+['task_metadata'])
                for turn,saved in zip(task['turns'],r.get('turns',[])):
                    obs=saved['observation']
                    g=grade_turn(turn,obs,ROBOTS[task['robot']]['surface'],r['home'])
                    if obs.get('agent_error'):
                        g={**obs['agent_error'],'unsafe':g['unsafe']}
                    if g!=saved['grade'] or turn['prompt']!=saved['prompt']: mismatches.append(key+['turn'])
                    grades.append(g)
                if r['outcome']=='PASS' and (len(grades)!=len(task['turns']) or any(g['outcome']!='PASS' for g in grades)):
                    mismatches.append(key+['episode'])
                if r['outcome']=='FAIL' and grades and all(g['outcome']=='PASS' for g in grades):
                    mismatches.append(key+['episode'])
                if r['outcome']!='ERROR' and (not r.get('physics',{}).get('finalised') or r.get('physics',{}).get('degraded')):
                    mismatches.append(key+['physics'])
                if r['outcome']!='ERROR':
                    unsafe=any(g['unsafe'] is True for g in grades) if all(g['unsafe'] is not None for g in grades) else None
                    if unsafe!=r.get('unsafe'): mismatches.append(key+['unwanted_motion'])
            except (KeyError,TypeError,ValueError): mismatches.append(key+['unreadable'])
    accounting=[]
    if lock.get('evidence_checks',0)>=2:
        from .agents import price, request_price, is_unbilled_overload
        total=0.
        cached=bool(lock.get('provider',{}).get('cache_policy'))
        if cached:
            try:
                from .anthropic_model import price as anthropic_price, reservation
                setup=json.loads((directory/'provider-setup.json').read_text(encoding='utf-8'))['requests']
                surfaces=sorted({ROBOTS[t['robot']]['surface'] for t in lock['suite']['tasks']})
                if len(setup)!=len(surfaces): accounting.append(['cache_warmup_count'])
                for surface,q in zip(surfaces,setup):
                    body=q['request']; cost=anthropic_price(q['usage'],lock['rates'],allow_cache_writes=True)
                    bound,reserve=reservation(body,q['counted_input_tokens'],lock['rates'])
                    digest=hashlib.sha256(json.dumps(body,sort_keys=True).encode()).hexdigest()
                    if (q.get('purpose')!='common_cache_warmup:'+surface or body['max_tokens']!=0
                        or body.get('cache_control') is not None or q['usage']['output_tokens']!=0
                        or not isinstance(body.get('system'),list) or len(body['system'])!=1
                        or body['system'][0].get('cache_control')!={'type':'ephemeral','ttl':'5m'}
                        or not (q['usage'].get('cache_creation_input_tokens',0)+q['usage'].get('cache_read_input_tokens',0))
                        or q['stop_reason']!='max_tokens' or q['model']!=lock['model'] or body.get('model')!=lock['model']
                        or body.get('output_config')!={'effort':'low'} or body.get('service_tier')!='standard_only'
                        or q['request_sha256']!=digest or q['reserved_input_tokens']!=bound
                        or not math.isclose(q['reserved_usd'],reserve,abs_tol=1e-12)
                        or cost is None or q['derived_usd'] is None or not math.isclose(cost,q['derived_usd'],abs_tol=1e-12)
                        or cost>reserve or q['usage']!=q['response']['usage']):
                        accounting.append(['cache_warmup_configuration_or_cost'])
                    if cost is not None: total+=cost
            except (KeyError,ValueError,TypeError,OSError): accounting.append(['cache_warmup_unreadable'])
        for index,r in enumerate(rows):
            key=[r['arm'],r['task'],r['repeat']]
            try:
                costs=[]
                for req in r['requests']:
                    overload=lock.get('evidence_checks',0)>=3 and is_unbilled_overload(req)
                    anthropic=lock.get('provider',{}).get('provider')=='anthropic-direct'
                    if anthropic:
                        from .anthropic_model import price as anthropic_price
                        cost=anthropic_price(req['usage'],lock['rates'],allow_cache_writes=cached)
                    else:
                        cost=request_price(req,lock['rates']) if lock.get('evidence_checks',0)>=3 else price(req['usage'],lock['rates'])
                    if cost is None or req['derived_usd'] is None or not math.isclose(cost,req['derived_usd'],abs_tol=1e-12):
                        accounting.append(key+['request_cost'])
                    costs.append(cost)
                    if not overload and req['model']!=lock['model']: accounting.append(key+['returned_model'])
                    body=req['request']
                    digest=hashlib.sha256(json.dumps(body,sort_keys=True).encode()).hexdigest()
                    if digest!=req['request_sha256']: accounting.append(key+['request_digest'])
                    if anthropic:
                        from .anthropic_model import CONFIG, reservation
                        bound,reserve=reservation(body,req['counted_input_tokens'],lock['rates'])
                        if (body.get('output_config')!={'effort':CONFIG['effort']} or body.get('max_tokens')!=CONFIG['max_tokens']
                            or body.get('service_tier')!='standard_only' or req['reserved_input_tokens']!=bound
                            or not math.isclose(req['reserved_usd'],reserve,abs_tol=1e-12)
                            or cost is None or cost>reserve or req['usage']['output_tokens']>body['max_tokens']):
                            accounting.append(key+['provider_configuration_or_reservation'])
                        control={'type':'ephemeral','ttl':'5m'}
                        if cached:
                            if (body.get('cache_control')!=control or not isinstance(body.get('system'),list)
                                or len(body['system'])!=1 or body['system'][0].get('cache_control')!=control):
                                accounting.append(key+['cache_configuration'])
                        elif body.get('cache_control') or isinstance(body.get('system'),list):
                            accounting.append(key+['unexpected_caching'])
                    if body.get('model')!=lock['model'] or (not anthropic and body.get('agentName')!=lock['profile']):
                        accounting.append(key+['request_configuration'])
                if any(c is None for c in costs) or r['derived_usd'] is None or not math.isclose(sum(costs),r['derived_usd'],abs_tol=1e-12):
                    accounting.append(key+['episode_cost'])
                else: total+=sum(costs)
                if not isinstance(r['elapsed_s'],(float,int)) or not math.isfinite(r['elapsed_s']) or r['elapsed_s']<0:
                    accounting.append(key+['elapsed_time'])
                if r['mode']!=finish.get('mode'): accounting.append(key+['mode'])
                if r['outcome']!='ERROR':
                    proof=json.loads((directory/'engines'/f'{index:05d}'/'engine.log.newton.json').read_text())
                    if proof!=r['physics']: accounting.append(key+['physics_sidecar'])
            except (KeyError,TypeError,ValueError,OSError): accounting.append(key+['unreadable_accounting'])
        if not math.isclose(total,finish.get('known_model_usd',-1),abs_tol=1e-10): accounting.append(['campaign_cost'])
    return lock,rows,{'complete':complete,'rows':len(rows),'planned':len(planned(lock)),
                      'rows_digest_matches':finish.get('rows_sha256')==row_hash,
                      'lock_digest_matches':finish.get('lock_sha256')==sha(directory/'lock.json'),
                      'source_snapshots_match':source_ok,'regrade_available':regrade_available,
                      'regrade_mismatches':mismatches,
                      'accounting_mismatches':accounting,
                      'integrity':integrity,'mode':finish.get('mode'),'suite_split':lock['suite']['split']}


def analyse(directory):
    directory=Path(directory); lock,rows,verification=verify(directory)
    summary={arm:stats([r for r in rows if r['arm']==arm]) for arm in lock['arms']}
    comparisons={arm:paired(rows,arm,lock['suite']) for arm in lock['arms'] if arm!='omnilink_parser'}
    reasons=[]
    if not verification['complete']: reasons.append('incomplete_or_duplicate_coverage')
    if not verification['rows_digest_matches']: reasons.append('evidence_digest_mismatch')
    if not verification['lock_digest_matches']: reasons.append('lock_digest_mismatch')
    if not verification['source_snapshots_match']: reasons.append('source_snapshot_mismatch')
    if not verification['regrade_available']: reasons.append('matching_oracle_required_for_regrade')
    if verification['regrade_mismatches']: reasons.append('grades_do_not_reproduce')
    if verification['accounting_mismatches']: reasons.append('accounting_does_not_reproduce')
    if not verification['integrity'].get('unchanged'): reasons.append('source_or_dependency_integrity_failed')
    if lock['suite']['split']!='holdout': reasons.append('development_tasks')
    if verification['mode']!='live': reasons.append('calibration_not_comparison')
    if lock.get('provider',{}).get('provider')=='anthropic-direct' and not lock['provider'].get('comparison'):
        reasons.append('exploratory_model_pilot')
    for arm,s in summary.items():
        if s['error'] or s['unknown_safety']: reasons.append(arm+':incomplete_observations')
        if s['derived_usd'] is None: reasons.append(arm+':unknown_cost')
    omni=summary.get('omnilink_parser',{})
    if omni.get('unsafe',0)>0: reasons.append('omnilink_unwanted_motion')
    decisions={}
    for arm,p in comparisons.items():
        ci=p.get('family_bootstrap_95',{})
        valid=p.get('valid_bootstrap_draws',{})
        ok=(all(valid.get(k)==2000 for k in ('success_difference_pp','cost_per_success_ratio','time_per_success_ratio'))
            and ci['success_difference_pp'][0]>-3
            and ci['cost_per_success_ratio'][1]<1 and ci['time_per_success_ratio'][1]<1)
        decisions[arm]={'meets_relative_rule':ok,'eligible_leadership_claim':ok and not reasons}
    result={'schema':'omnisim-control-bench-scorecard/1','scope':lock['scope'],
            'verification':verification,'summary':summary,'comparisons':comparisons,
            'claim_blockers':reasons,'decisions':decisions,
            'breakdowns':{arm:{robot:stats([r for r in rows if r['arm']==arm and r['robot']==robot])
                for robot in sorted({r['robot'] for r in rows})} for arm in lock['arms']}}
    if lock.get('provider',{}).get('cache_policy'):
        setup=json.loads((directory/'provider-setup.json').read_text(encoding='utf-8'))['requests']
        costs=[q.get('derived_usd') for q in setup]
        result['shared_cache_setup_usd']=sum(costs) if all(c is not None for c in costs) else None
        result['cost_scope']='Per-arm costs are scored calls. Shared cache warm-up is metered separately and included in the campaign budget.'
    dump(directory/'scorecard.json',result)
    lines=['# Robotics control scorecard','',lock['scope']+'.',
           '', 'Split: **'+lock['suite']['split']+'**. Mode: **'+str(verification['mode'])+'**.',
           '', '| Implementation | PASS / attempts | ERROR | Unwanted motion | Model calls | USD / success | Seconds / success |',
           '|---|---:|---:|---:|---:|---:|---:|']
    def fmt(v): return 'unknown' if v is None else f'{v:.6f}'
    for arm,s in summary.items():
        lines.append(f"| {arm} | {s['pass']}/{s['episodes']} | {s['error']} | {s['unsafe']} | {s['model_calls']} | {fmt(s['usd_per_success'])} | {fmt(s['seconds_per_success'])} |")
    lines+=['','Claim blockers: '+(', '.join(reasons) or 'none; inspect each paired decision'),'',
            'All-attempt model spending and elapsed time include failed jobs. ERROR counts remain in the denominator.',
            'Costs are derived provider list rates, excluding subscriptions, hosting and local compute.',
            'Manipulation in the bundled arm world uses an assisted grasp; no hardware or friction-only grasp claim.',
            'See scorecard.json for paired family-block intervals, robot breakdowns and exact claim eligibility.']
    if 'shared_cache_setup_usd' in result:
        lines+=['','Shared cache warm-up USD: '+fmt(result['shared_cache_setup_usd'])+'. '+result['cost_scope']]
    (directory/'scorecard.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return result
