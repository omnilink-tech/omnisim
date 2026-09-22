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
"""Run sequential guarded physical probes and save machine-attributed evidence."""
import argparse
from datetime import datetime
import json
import hashlib
import math
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0,str(ROOT))
from omnisim.dev.runner import omnisim_env
from build_foundry import build


def assess(data):
    """Numerical acceptance gates; a headless load PASS is insufficient."""
    if data['case'] == 'match':
        events = data['events']
        finishes = [e for e in events if e['type'] == 'result']
        knockouts = [e for e in events if e['type'] == 'immobilized']
        checks = {'completed':len(finishes) == 1, 'single_physical_knockout':len(knockouts) == 1,
                  'no_controller_errors':not any(e['type'] == 'error' for e in events),
                  'mass_conserved':abs(data['initial_mass_kg']-data['final_audit']['mass_kg']) < 1e-6}
        metrics = {'detachments':[{'t':e['t'],'robot':e['robot'],'part':e['part']}
                                  for e in events if e['type'] == 'detach']}
        clearance = {name:min((e['bots'][name]['position'][2]-sum(h*abs(r) for h,r in
                                  zip((.56,.39,.15),e['bots'][name]['rotation'][6:9]))
                              for e in events if e['type'] == 'sample'),default=-math.inf)
                     for name in ('ANVIL','RAZOR')}
        checks['no_chassis_floor_penetration'] = min(clearance.values()) >= -.05
        metrics['minimum_chassis_floor_bound_m'] = clearance
        if len(knockouts) == 1:
            knockout = knockouts[0]
            name, end = knockout['robot'], knockout['t']
            window = [e for e in events if e['type'] == 'sample' and
                      e['phase'] == 'combat' and end-10 <= e['t'] < end]
            checks['full_powered_observation'] = (len(window) >= 40 and
                window[0]['t'] <= end-9.75 and window[-1]['t'] >= end-.25 and
                all(0 < b['t']-a['t'] <= .25 for a,b in zip(window,window[1:])) and
                all(max(map(abs,e['bots'][name]['pilot']['powertrain']['drive_request'])) >= 6
                    and e['bots'][name]['pilot']['powertrain']['live'] for e in window))
            checks['fresh_drive_telemetry'] = bool(window) and all(
                0 <= e['t']-e['bots'][name]['pilot'].get('t',-math.inf) < .1 for e in window)
            checks['no_opponent_pin'] = bool(window) and all(
                not e['mobility'][name].get('opponent_contact',True) for e in window)
            checks['combat_disablement_precedes_countout'] = any(
                e['robot'] == name and e['t'] < end-10 for e in metrics['detachments']) or any(
                e['type'] == 'overturned' and e['robot'] == name and e['t'] < end-10 and
                e['t']-e['last_opponent_contact_t'] < 2 for e in events)
            checks['ten_second_count'] = knockout['mobility']['stationary_s'] >= 10
            positions = [e['bots'][name]['position'] for e in window]
            rotations = [e['bots'][name]['rotation'] for e in window]
            span = max((math.dist(a,b) for a in positions for b in positions),default=math.inf)
            angle = max((math.acos(max(-1,min(1,(sum(x*y for x,y in zip(a,b))-1)/2)))
                         for a in rotations for b in rotations),default=math.inf)
            checks['independent_motion_check'] = span < .15 and angle < .25
            winner = 'RAZOR' if name == 'ANVIL' else 'ANVIL'
            winner_positions = [e['bots'][winner]['position'] for e in events if e['type'] == 'sample'
                                and e['phase'] == 'combat' and end-15 <= e['t'] < end]
            winner_span = max((math.dist(a,b) for a in winner_positions for b in winner_positions),default=0)
            checks['winner_demonstrated_mobility'] = winner_span > .5
            metrics.update(loser=name, cause=knockout.get('cause'), knockout_time_s=end, mobility=knockout['mobility'],
                           translation_span_m=span, rotation_span_rad=angle, winner_translation_span_m=winner_span)
        budgets = {}
        for event in events:
            if event['type'] == 'impact':
                entry = budgets.setdefault(event['t'],[0.,event['assembly_loss_budget_j']])
                entry[0] += event['plastic_work_j']
        checks['plastic_work_within_loss_budget'] = all(work <= budget+1e-6 for work,budget in budgets.values())
        return {'case':'match','passed':all(checks.values()),'checks':checks,'metrics':metrics}
    records = data['records']
    case = data['case']
    powered = min(records,key=lambda r:abs(r['t']-4.496))
    checks = {'completed':records[-1]['t'] >= 6,
              'mass_constant':max(r['audit']['mass_kg'] for r in records)-
                              min(r['audit']['mass_kg'] for r in records) < 1e-6}
    metrics = {}
    if case == 'spin':
        error = abs(powered['audit']['energy_j']-powered['work_j'])/powered['work_j']
        metrics.update(motor_work_j=powered['work_j'],kinetic_j=powered['audit']['energy_j'],
                       work_energy_relative_error=error,
                       weapon_rpm=powered['motors']['weapon']['omega_rad_s']*60/math.tau)
        checks.update(work_energy=error < .01,weapon_spun=metrics['weapon_rpm'] > 600,
                      reaction_momentum=max(math.dist(r['audit']['momentum'],[0]*3) for r in records) < .5,
                      reaction_angular_momentum=max(math.dist(r['audit']['angular_momentum'],[0]*3) for r in records) < .5,
                      coast_dissipates=records[-1]['audit']['energy_j'] < powered['audit']['energy_j'])
    elif case == 'drum':
        rpm = powered['motors']['weapon']['omega_rad_s']*60/math.tau
        metrics['weapon_rpm'] = rpm
        checks['drum_spins_clear_of_floor'] = -1500 < rpm < -1200
    elif case == 'drive':
        speed = math.hypot(*powered['velocity'][:2])
        rim_speed = .17*sum(m['omega_rad_s'] for m in powered['motors'].values())/4
        stop = next((r for r in records if r['t'] > 4.51 and math.hypot(*r['velocity'][:2]) < .05),None)
        metrics.update(speed_m_s=speed,rim_speed_m_s=rim_speed,
                       stopping_distance_m=math.dist(powered['position'][:2],stop['position'][:2]) if stop else None)
        checks.update(drives=3.5 < speed < 4.5,rolling=abs(speed-rim_speed)<.08,brakes=stop is not None)
    elif case in ('rebuild','detach'):
        before = data['before_detach']
        after = next(r['audit'] for r in records if r['t'] >= 4.511)
        delta = abs(after['energy_j']-before['energy_j'])/max(1,before['energy_j'])
        dp = math.dist(before['momentum'],after['momentum'])
        dl = math.dist(before['angular_momentum'],after['angular_momentum'])
        metrics.update(energy_relative_change=delta,momentum_change_ns=dp,angular_momentum_change_nms=dl)
        checks.update(energy_retained=delta < .01,momentum_retained=dp < .2,angular_momentum_retained=dl < .2)
        if case == 'detach':
            checks['no_release_energy_gain'] = max(r['audit']['energy_j'] for r in records if r['t'] > 4.511) < before['energy_j']*1.01
    return {'case':case,'passed':all(checks.values()),'checks':checks,'metrics':metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=['spin','drum','drive','rebuild','detach','match'],default=['spin','drum','drive','rebuild','detach'])
    parser.add_argument('--render',action='store_true',help='record the full match and aftermath in a visible, guarded preview')
    args = parser.parse_args()
    if args.render and args.cases != ['match']:
        parser.error('--render requires --cases match')
    output = ROOT/'_scratch/foundry_realism'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    print(output,flush=True)
    fingerprint = subprocess.run([sys.executable,str(ROOT/'projects/policies/common/env_fingerprint.py')],
                                 cwd=ROOT,capture_output=True,text=True)
    (output/'machine.txt').write_text(fingerprint.stdout+fingerprint.stderr)
    reports = []
    sources = [HERE/'build_foundry.py',HERE/'shared/foundry_logic.py',HERE/'shared/foundry_physics.py',
               HERE/'controllers/orc_foundry_director/orc_foundry_director.py',
               HERE/'controllers/orc_foundry_pilot/orc_foundry_pilot.py',
               HERE/'controllers/orc_foundry_probe/orc_foundry_probe.py',Path(__file__)]
    source_hashes = {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (output/'sources.json').write_text(json.dumps(source_hashes,indent=2))
    os.environ.setdefault('PYTHON_HOME',str(Path(sys.executable).parent))
    for case in args.cases:
        env = omnisim_env()
        env.update(ORC_FOUNDRY_PROBE=case,ORC_FOUNDRY_PROBE_OUTPUT=str(output/(case+'.json')),
                   ORC_FOUNDRY_OUTPUT=str(output/(case+'.events.jsonl')),
                   OMNISIM_LOG_PATH=str(output/(case+'.engine.log')),
                   WARP_CACHE_PATH=str(ROOT/'_scratch/foundry/warp_cache'))
        env['ORC_FOUNDRY_CAPTURE'] = '1' if args.render else '0'
        text = build()
        if case != 'match':
            text = text.replace('controller "orc_foundry_pilot"','controller "<none>"')
        text = text.replace('controller "orc_foundry_director"','controller "orc_foundry_probe"')
        if case not in ('drive','drum','match'):
            text = text.replace('basicTimeStep 8','gravity 0 basicTimeStep 8')
            text = text.replace('translation -3.1 0 0.2','translation -3.1 0 3')
            text = text.replace('translation 3.1 0 0.2','translation 3.1 0 3')
        elif case != 'match':
            text = text.replace('translation 3.1 0 0.2','translation 3.1 8.5 0.2')
        world = HERE/'worlds'/f'.foundry_probe_{case}_{output.name}.omniworld'
        world.write_text(text,encoding='utf-8')
        command = [sys.executable,str(ROOT/'scripts/dev/thermal_guard.py'),'run',
                   '--ceiling','75','--interval','.5','--precool','65','--',
                   sys.executable,str(ROOT/'scripts/dev/headless_runner.py'),str(world),
                   '--duration','900' if case == 'match' else '24','--wait-for-step',
                   '--gui' if args.render else '--no-window','--realtime',
                   '--race-attempts','1']
        try:
            with (output/(case+'.run.log')).open('w',encoding='utf-8') as log:
                result = subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
        finally:
            world.unlink(missing_ok=True)
        for line in (output/(case+'.run.log')).read_text(errors='replace').splitlines():
            if line.startswith('thermal_guard:') or '[headless] Results:' in line:
                print(line,flush=True)
        if result.returncode or not (output/(case+'.json')).exists():
            raise SystemExit(f'{case} probe failed; inspect {output}')
        reports.append(assess(json.loads((output/(case+'.json')).read_text())))
        (output/'summary.json').write_text(json.dumps(reports,indent=2))
        print(json.dumps(reports[-1]),flush=True)
        if not reports[-1]['passed']:
            raise SystemExit(f'{case} numerical checks failed; inspect {output}')
    print(f'Probe recordings: {output}',flush=True)


if __name__ == '__main__':
    main()
