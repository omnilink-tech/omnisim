from pathlib import Path
import json, hashlib
from PIL import Image
repo=Path(__file__).resolve().parents[2]
out=repo/'.local-runs/temporal-pipeline'
rows=json.loads((out/'house/results.json').read_text())+json.loads((out/'other-scenes/results.json').read_text())
evidence={'date':'2026-09-12','machine_id':'9722d23d12a3','os':'Windows 11',
    'gpu':'NVIDIA GeForce RTX 3060 Laptop GPU','driver':'596.36',
    'scope':'GPU timestamp diagnostics and temporal image controls. Concurrent simulator jobs; no speedup or FPS claim.',
    'baseline_commit':'37aa0fc84',
    'installed_candidate_sha256':hashlib.sha256((repo/'msys64/mingw64/bin/omnisim-bin-temporal-final.exe').read_bytes()).hexdigest(),
    'candidate_difference':'Final build adds render-start interval reporting, failed-frame history invalidation, and explicit binding invalidation when the scene buffer grows. Shader and projection match measured build.',
    'shader_cases':['uncovered background rejects stale surface','coplanar color change reduces stale-history error by more than 35 percent in controlled fixture',
                    'stable detail still accumulates','bilinear taps reject the wrong surface','initial history, sky, and off-screen camera cuts use current pixels'],
    'scenes':rows}
target=repo/'docs/benchmarks/data/temporal-rendering-2026-09-12.json'
checks=(repo/'.local-runs/beauty-realism/temporal-release-checks.stdout').read_text()
for proof in ('11 passed in','1 passed in','1290 passed','16 passed','2 passed'):
    assert proof in checks, proof
evidence['validation']={'focused_passed':11,'repeated_scene_growth_passed':1,'unit_passed':1290,
    'unit_skipped':11,'unit_deselected':40,'unit_subtests_passed':21,'docs_passed':16,'license_passed':2,
    'growth_states_per_arm':16,'growth_arms':3,'growth_runs':2}
evidence['final_controls']=[{'scene':row['scene'],'baseline_image_check':row['baseline_image_check'],
    'binary_sha256':row['result']['binary_sha256'],'gpu_profile':row['result']['gpu_profile']}
    for row in json.loads((out/'final-controls/results.json').read_text())]
target.write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
report=['# Rendering comparison\n','The new live rendering path checks whether old pixels still belong to the visible surface, aligns anti-aliasing camera offsets, and reduces trails when color changes. Optional GPU measurements now identify the cost of each rendering stage.\n',
        'The captures below compare the former and new temporal resolves at the same fixed view. They are stationary image controls; motion correctness is checked separately with controlled GPU tests. Indirect-light baking was disabled equally for these checks.\n',
        'Across the house, warehouse, and city, disabling anti-aliasing produced an exact pixel match with the previous installed renderer. Turning GPU measurement on also preserved those pixels exactly. Other simulator tasks were running, so these measurements do not establish an FPS improvement.\n']
for row in rows:
    scene=row['scene'];folder=out/('house' if scene=='house' else 'other-scenes')
    report += [f'## {scene.title()}\n',f'Before — previous temporal resolve:\n\n![{scene} before]({(folder/f"{scene}-0-legacy.png").as_posix()})\n',
               f'After — depth-checked temporal resolve:\n\n![{scene} after]({(folder/f"{scene}-0-validated.png").as_posix()})\n']
    im=Image.open(folder/f'{scene}-0-validated.png').convert('RGB')
    im.save(folder/'review.png',compress_level=9)
report += ['## Remaining work\n','Fast objects and deforming surfaces still need their own motion vectors for accurate history tracking. Dynamic indirect light/reflections and acceleration of optional Photo mode are separate follow-on work.\n']
(out/'before-after.md').write_text('\n'.join(report),encoding='utf-8')
print('Recorded three scene comparisons; image checks:',[(r['scene'],r['timing_image_check']['pixels_over_threshold'],r['baseline_image_check']['pixels_over_threshold']) for r in rows])
