import hashlib
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root))
from scripts.dev.render_ab import diff

rows = []
noise = {}
for name, folder in [('house', 'house-capped'), ('warehouse', 'warehouse-static'), ('city', 'city-capped')]:
    base = root / '.local-runs/shadow-pipeline' / folder
    records = json.loads((base / 'results.json').read_text())
    assert len(records) == 2 and all(r['verdict'] == 'MATCH' for r in records)
    for record in records:
        record['capture_frame'] = 90
        for arm in ['before', 'after']:
            prefix = base / f'{name}-{record["repeat"]}-{arm}'
            record[arm]['image_sha256'] = hashlib.sha256(prefix.with_suffix('.png').read_bytes()).hexdigest()
            record[arm]['profile_log_sha256'] = hashlib.sha256(prefix.with_suffix('.report').read_bytes()).hexdigest()
        rows.append(record)
    noise[name] = {arm: diff(base / f'{name}-0-{arm}.png', base / f'{name}-1-{arm}.png', 3)
                   for arm in ['before', 'after']}
document = {
    'date': '2026-09-12',
    'machine': {'id': '9722d23d12a3', 'os': 'Windows 11', 'logical_cores': 16,
                'cpu': 'AMD64 Family 25 Model 80 Stepping 0, AuthenticAMD',
                'gpu': 'NVIDIA GeForce RTX 3060 Laptop GPU', 'gpu_driver': '596.36'},
    'build': {'candidate_sha256': '85f9f1920022787e69056ee5cbe48c9d99c679f85f35689b7e0fb7c362b7e139',
              'final_validated_sha256': '280d7b1cd46a30dc43d23c877d071057a69e70f63fc54962db02c483a692a4cc',
              'changes_after_measurement': 'Draw-input subscription cleanup moved behind a Qt-free API; shadow culling and cache algorithms unchanged. Final engine validation waits for queued screenshot completion before the next scene edit.',
              'source_state': 'Uncommitted shadow optimization over the previously installed af23053e2 rendering milestone; candidate binary identifies the measured build.'},
    'validation': {'engine_edit_states_main_and_camera': 13, 'focused_tests_passed': 5,
                   'native_visible_point_witnesses': 119155, 'unit_tests_passed': 1289,
                   'unit_tests_skipped': 11, 'unit_tests_deselected': 40,
                   'documentation_tests_passed': 16, 'license_tests_passed': 2},
    'method': 'Same candidate binary, cache/culling disabled versus enabled. Two repeats with alternating arm order. 1896x1113 captures, frame 90, 5 FPS cap, first 60 frames excluded. CPU render phase includes setup/submission/readback waits; these are not GPU timestamps or maximum FPS.',
    'scene_controls': 'GI bake, TAA, camera effects and autoexposure disabled equally. Warehouse variant enables ceiling shadows and stops conveyor texture animation. City traffic controller disabled. Authored scene geometry and viewpoints preserved.',
    'repeat_image_differences': noise,
    'results': rows,
}
document['city_timing_controls'] = {}
for arm, folder in [('disabled', 'city-noop'), ('enabled', 'city-noop-enabled')]:
    base = root / '.local-runs/shadow-pipeline' / folder
    document['city_timing_controls'][arm] = {
        'runs': json.loads((base / 'results.json').read_text()),
        'repeat_image_diff': diff(base / '0.png', base / '1.png', 3),
    }
target = root / 'docs/benchmarks/data/local-shadow-2026-09-12.json'
target.write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
for name in ['house', 'warehouse', 'city']:
    group = [r for r in rows if r['scene'] == name]
    print(name, {arm: {'render_p50_ms': [round(r[arm]['profile']['renderUs']['p50']/1000, 3) for r in group],
                       'shadow_p50_us': [r[arm]['profile']['localShadowUs']['p50'] for r in group],
                       'draws': [r[arm]['profile']['localDraws']['p50'] for r in group]}
                 for arm in ['before', 'after']}, 'repeat differences:', noise[name])
