"""Preserve a user-interrupted run without inventing a completion or response."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from omnisim.control_bench.evidence import check_inputs, sha
from omnisim.control_bench.anthropic_model import price
from omnisim.control_bench.tasks import grade_turn, ROBOTS


def main():
    directory=ROOT/'tests/benchmarks/robot_control/evidence/opus-comparison-01'
    lock=json.loads((directory/'lock.json').read_text(encoding='utf-8'))
    rows=[json.loads(s) for s in (directory/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
    journal=[json.loads(s) for s in (directory/'provider-journal.jsonl').read_text(encoding='utf-8').splitlines()]
    integrity=check_inputs(lock); assert integrity['unchanged']
    assert not (directory/'completion.json').exists()
    tasks={t['id']:t for t in lock['suite']['tasks']}
    for i,r in enumerate(rows):
        task=tasks[r['task']]
        assert len(task['turns'])==len(r['turns'])
        for turn,record in zip(task['turns'],r['turns']):
            assert grade_turn(turn,record['observation'],ROBOTS[r['robot']]['surface'],r['home'])==record['grade']
        proof=json.loads((directory/'engines'/f'{i:05d}'/'engine.log.newton.json').read_text(encoding='utf-8'))
        assert proof==r['physics'] and proof['finalised'] and not proof.get('degraded')
    reserved={e['index']:e['record'] for e in journal if e['event']=='reserved'}
    finished={e['index']:e['record'] for e in journal if e['event']=='finished'}
    rq=[q for r in rows for q in r['requests']]
    assert all(finished[i]==q for i,q in enumerate(rq))
    known=0
    for i,q in finished.items():
        cost=price(q['usage'],lock['rates'])
        assert cost is not None and cost==q['derived_usd']
        assert reserved[i]['request']==q['request']
        assert known+reserved[i]['reserved_usd']<=lock['max_model_usd']
        known+=cost
    unresolved=[{'index':i,'reserved_usd':q['reserved_usd'],'request_sha256':q['request_sha256']}
                for i,q in reserved.items() if i not in finished]
    unfinished_known=sum(q['derived_usd'] for i,q in finished.items() if i>=len(rq))
    result={'state':'user_interrupted','complete':False,'sealed_utc':datetime.now(timezone.utc).isoformat(),
        'reason':'Owner requested: Stop now; prepare cached comparison',
        'completed_episodes':len(rows),'planned_episodes':198,'completed_pass':sum(r['outcome']=='PASS' for r in rows),
        'unfinished_episode_index':len(rows),'known_model_usd':known,
        'known_model_usd_outside_completed_rows':unfinished_known,
        'unresolved_paid_requests':unresolved,
        'conservative_usage_upper_usd':known+sum(x['reserved_usd'] for x in unresolved),
        'rows_sha256':sha(directory/'rows.jsonl'),'lock_sha256':sha(directory/'lock.json'),
        'journal_sha256':sha(directory/'provider-journal.jsonl'),
        'integrity_at_stop':integrity,'all_completed_physical_grades_reproduced':True,
        'completion_json_intentionally_absent':True,
        'cost_ranking_allowed':False,'leader_claim_allowed':False}
    with (directory/'interruption.json').open('x',encoding='utf-8') as f:
        json.dump(result,f,indent=2);f.write('\n')
    (directory/'STOP_REPORT.md').write_text(
        '# Uncached comparison stopped at owner request\n\n'
        f"{len(rows)}/198 completed episodes, all {result['completed_pass']} passing. Unequal coverage: no implementation ranking.\n\n"
        f"Known model usage: ${known:.6f}, including ${unfinished_known:.6f} in the unfinished episode. "
        f"One unresolved in-flight request has a ${sum(x['reserved_usd'] for x in unresolved):.6f} reservation. "
        f"The conservative total under the reservation assumptions is ${result['conservative_usage_upper_usd']:.6f}; this is not a confirmed invoice.\n\n"
        'Raw completed rows, the unfinished engine logs, all provider reservations and returned responses remain intact. '
        'All completed physical grades were rechecked and source integrity was unchanged when stopped. '
        'No completion record or missing model response was fabricated. See interruption.json.\n\n'
        'This run explicitly had no prompt caching. It was stopped to prepare a separately declared cached comparison. '
        'Do not pool its observed costs with a cached run or restart it under a fresh budget.\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
