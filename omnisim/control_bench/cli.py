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
"""The public reproduction command: python -m omnisim control-bench."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import random
import shutil
import sys
import time

from omnisim.paths import REPO_ROOT
from .agents import ARMS, Model, Runtime
from .engine import Engine, InfrastructureError, Observation, failed
from .evidence import analyse, check_inputs, dump, freeze, planned, sha, verify
from .tasks import ROBOTS, grade_turn, load_suite, position

DEFAULT_SUITE=REPO_ROOT/'tests/benchmarks/robot_control/suites/development.json'


def oracle_turn(obs,turn):
    """Fixture/grader calibration ONLY; never called by a scored agent."""
    status=turn['expect'].get('status','complete')
    for action in turn.get('calibration_actions',turn['expect'].get('path',[])):
        out=obs.dispatch(action)
        if out.get('error')=='temporary_unavailable': out=obs.dispatch(action)
        if failed(out): status='failed'; break
    return {'status':status,'message':'Calibration oracle; not agent evidence',
            'reported_pose':position(obs.engine.state(),obs.engine.surface)}


def run(lock_path,out,key,calibration=False,anthropic_key=None):
    lock=json.loads(Path(lock_path).read_text(encoding='utf-8'))
    if lock.get('schema')!='omnisim-control-bench-lock/1': raise ValueError('Unknown lock schema')
    if calibration != (lock['arms']==['oracle']):
        raise ValueError('Calibration requires a separate lock with --arms oracle')
    check=check_inputs(lock)
    if not check['unchanged']: raise ValueError('Freeze no longer matches source/dependencies: '+json.dumps(check))
    if not key: raise ValueError('Set OMNI_KEY or pass --key-file; never put a key in a suite or lock')
    out=Path(out).resolve()
    out.mkdir(parents=True,exist_ok=False)
    dump(out/'lock.json',lock)
    # Snapshot reviewable source bytes; heavy runtime/assets stay hash-pinned.
    for name in lock['inputs']:
        if Path(name).suffix in ('.py','.mjs','.json','.omniworld','.proto','.urdf','.md'):
            dest=out/'source'/name; dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(REPO_ROOT/name,dest)
    (out/'rows.jsonl').touch()
    if lock.get('provider',{}).get('provider')=='anthropic-direct':
        from .anthropic_model import AnthropicModel
        model=AnthropicModel(anthropic_key or '',lock['model'],lock['max_model_usd'],lock['rates'],out/'provider-journal.jsonl',lock['provider'])
    else:
        model=Model(key,lock['model'],lock['profile'],lock['max_model_usd'],lock['rates'])
    rows=[]; interrupted=False
    try:
        if lock.get('provider',{}).get('cache_policy'):
            try:
                model.warmup({ROBOTS[t['robot']]['surface'] for t in lock['suite']['tasks']})
            finally:
                dump(out/'provider-setup.json',{'purpose':'Shared cache warm-up; charged to campaign, reported separately from scored episodes',
                                               'requests':list(model.records)})
        for repeat in range(lock['repeats']):
            order=[(task,arm) for task in lock['suite']['tasks'] for arm in lock['arms']]
            random.Random(lock['seed']+repeat).shuffle(order)
            for task,arm in order:
                if not calibration and (model.blocked or model.spent+.10>model.cap):
                    interrupted=True; break
                record={'task':task['id'],'family':task['family'],'category':task['category'],
                        'robot':task['robot'],'arm':arm,'repeat':repeat,'turns':[],
                        'outcome':'ERROR','unsafe':None,'mode':'calibration' if calibration else 'live'}
                engine=runtime=None; n0=len(model.records); started=time.monotonic()
                try:
                    engine=Engine(task['robot'],out/'engines'/f'{len(rows):05d}',key)
                    record['home']=engine.home
                    record['physics']=engine.physics
                    engine.setup(task.get('setup',[]))
                    record['setup_trace']=engine.setup_trace
                    record['setup_s']=time.monotonic()-started
                    started=time.monotonic()
                    if not calibration: runtime=Runtime(model,arm)
                    history=[]; outcomes=[]
                    for turn in task['turns']:
                        obs=Observation(engine,turn['prompt'],turn.get('fault'))
                        response={}; exhausted=False; error=None
                        try:
                            if calibration: response=oracle_turn(obs,turn)
                            else: response,history,exhausted=runtime.run(obs,history)
                        except TimeoutError:
                            exhausted=True
                        except (ValueError,KeyError,TypeError) as exc:
                            error={'outcome':'FAIL','reasons':['invalid_agent_output:'+type(exc).__name__],'unsafe':False}
                        except InfrastructureError as exc:
                            error={'outcome':'ERROR','reasons':[str(exc)],'unsafe':None}
                        finally:
                            observation=obs.finish(response,exhausted)
                        grade=grade_turn(turn,observation,engine.surface,engine.home)
                        if error:
                            error['unsafe']=grade['unsafe']
                            observation['agent_error']=error
                            grade=error
                        record['turns'].append({'prompt':turn['prompt'],'grade':grade,'observation':observation})
                        outcomes.append(grade)
                        if grade['outcome']=='ERROR': break
                    record['outcome']='ERROR' if any(g['outcome']=='ERROR' for g in outcomes) else (
                        'PASS' if len(outcomes)==len(task['turns']) and all(g['outcome']=='PASS' for g in outcomes) else 'FAIL')
                    record['unsafe']=any(g['unsafe'] is True for g in outcomes) if all(g['unsafe'] is not None for g in outcomes) else None
                    record['parser_hits']=runtime.parser_hits if runtime else 0
                except Exception as exc:
                    record['error']=type(exc).__name__+': '+str(exc)
                finally:
                    record['elapsed_s']=time.monotonic()-started
                    record['requests']=model.records[n0:]
                    costs=[r['derived_usd'] for r in record['requests']]
                    record['derived_usd']=sum(costs) if all(c is not None for c in costs) else None
                    if runtime: runtime.close()
                    if engine:
                        cleanup=engine.close()
                        if cleanup:
                            record['cleanup_errors']=cleanup
                            record['outcome']='ERROR'
                rows.append(record)
                with (out/'rows.jsonl').open('a',encoding='utf-8') as f:
                    f.write(json.dumps(record,ensure_ascii=False)+'\n'); f.flush(); os.fsync(f.fileno())
                print(f"{len(rows)}/{len(planned(lock))} {arm} {task['id']} {record['outcome']} ${model.spent:.4f}",flush=True)
            if interrupted: break
    except KeyboardInterrupt:
        interrupted=True
    finally:
        integrity=check_inputs(lock)
        dump(out/'completion.json',{'complete':len(rows)==len(planned(lock)) and not interrupted,
             'mode':'calibration' if calibration else 'live','ended_utc':datetime.now(timezone.utc).isoformat(),
             'integrity':integrity,'rows_sha256':sha(out/'rows.jsonl'),'lock_sha256':sha(out/'lock.json'),
             'known_model_usd':model.spent,'unknown_usage':model.blocked})
    report=analyse(out)
    print(str(out/'scorecard.md'))
    valid=report['verification']['complete'] and integrity['unchanged']
    if calibration: valid=valid and all(r['outcome']=='PASS' for r in rows)
    return 0 if valid else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    v=sub.add_parser('validate',help='Validate task schema, without models or simulator')
    v.add_argument('--suite',type=Path,default=DEFAULT_SUITE)
    f=sub.add_parser('freeze',help='Freeze tasks, implementations, budgets and provenance before running')
    f.add_argument('--suite',type=Path,default=DEFAULT_SUITE); f.add_argument('--lock',type=Path,required=True)
    f.add_argument('--arms',nargs='+',choices=(*ARMS,'oracle'),default=list(ARMS))
    f.add_argument('--repeats',type=int,default=3); f.add_argument('--cap-usd',type=float,default=5)
    f.add_argument('--model',default='gemini-3.1-flash-lite'); f.add_argument('--profile',default='HuskySwarm')
    f.add_argument('--input-rate',type=float,default=.25); f.add_argument('--cached-rate',type=float,default=.025)
    f.add_argument('--output-rate',type=float,default=1.5)
    f.add_argument('--anthropic-pilot',action='store_true',help='Direct Opus 5.5 low; fixed rates, max $1, exploratory only')
    f.add_argument('--anthropic-comparison',action='store_true',help='Matched six-arm Opus comparison; fixed rates, max $5')
    f.add_argument('--anthropic-cached-comparison',action='store_true',help='Matched six-arm Opus comparison with system/history caching and metered shared warm-up; max $5')
    r=sub.add_parser('run',help='Run a frozen plan in owned OmniSim engines')
    r.add_argument('--lock',type=Path,required=True); r.add_argument('--out',type=Path,required=True)
    r.add_argument('--key-file',type=Path); r.add_argument('--calibrate',action='store_true')
    r.add_argument('--anthropic-key-file',type=Path,help='Separate provider key; --key-file remains the OmniKey')
    for name in ('verify','report'):
        s=sub.add_parser(name); s.add_argument('directory',type=Path)
    args=p.parse_args(argv)
    try:
        if args.command=='validate':
            suite=load_suite(args.suite)
            print(json.dumps({'tasks':len(suite['tasks']),'split':suite['split'],'robots':sorted({t['robot'] for t in suite['tasks']})}))
            return 0
        if args.command=='freeze':
            if args.repeats<1 or not math.isfinite(args.cap_usd) or args.cap_usd<=0 or len(set(args.arms))!=len(args.arms): raise ValueError('Invalid repeats/budget/arms')
            if any(not math.isfinite(v) or v<0 for v in (args.input_rate,args.cached_rate,args.output_rate)):
                raise ValueError('Token rates must be finite and nonnegative')
            if 'oracle' in args.arms and args.arms!=['oracle']: raise ValueError('Oracle is calibration only')
            provider=None
            rates={'input':args.input_rate,'cached':args.cached_rate,'output':args.output_rate}
            if args.anthropic_pilot or args.anthropic_comparison or args.anthropic_cached_comparison:
                from .anthropic_model import CONFIG, COMPARISON_CONFIG, CACHED_CONFIG, MODEL
                if sum((args.anthropic_pilot,args.anthropic_comparison,args.anthropic_cached_comparison))!=1:
                    raise ValueError('Choose one Anthropic run type')
                comparison=args.anthropic_comparison or args.anthropic_cached_comparison
                limit=5 if comparison else 1
                if args.cap_usd>limit or args.arms==['oracle']: raise ValueError(f'Anthropic cap must be <= ${limit}; live only')
                if comparison and (set(args.arms)!=set(ARMS) or args.repeats!=1):
                    raise ValueError('This comparison requires all six arms and one repeat')
                args.model=MODEL; rates=CONFIG['rates']
                provider=CACHED_CONFIG if args.anthropic_cached_comparison else (COMPARISON_CONFIG if comparison else CONFIG)
            freeze(args.suite,args.lock,args.arms,args.repeats,args.cap_usd,args.model,args.profile,rates,provider=provider)
            print(args.lock); return 0
        if args.command=='run':
            key=args.key_file.read_text().strip() if args.key_file else os.environ.get('OMNI_KEY','').strip()
            anthropic_key=args.anthropic_key_file.read_text(encoding='utf-8-sig').strip() if args.anthropic_key_file else None
            return run(args.lock,args.out,key,args.calibrate,anthropic_key)
        if args.command=='report':
            result=analyse(args.directory)
            print(json.dumps({'claim_blockers':result['claim_blockers'],'decisions':result['decisions']},indent=2)); return 0
        _,_,result=verify(args.directory); print(json.dumps(result,indent=2))
        return 0 if (result['complete'] and result['rows_digest_matches'] and result['lock_digest_matches']
                     and result['integrity'].get('unchanged') and result['source_snapshots_match']
                     and result['regrade_available'] and not result['regrade_mismatches']
                     and not result['accounting_mismatches']) else 1
    except (ValueError,OSError,KeyError,InfrastructureError) as exc:
        print(f'control-bench: {exc}',file=sys.stderr); return 2


if __name__=='__main__': raise SystemExit(main())
