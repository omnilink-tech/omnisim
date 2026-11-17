import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root))
from scripts.dev.render_shadow_bench import benchmark_world
from scripts.dev.render_ab import render

enabled = len(sys.argv) > 1 and sys.argv[1] == 'enabled'
dest = root / '.local-runs/shadow-pipeline' / ('city-noop-enabled' if enabled else 'city-noop')
dest.mkdir(exist_ok=True)
env = {'OMNISIM_BINARY': str(root / 'msys64/mingw64/bin/omnisim-bin-shadow.exe'),
       'OMNISIM_WGPU_TAA': '0', 'OMNISIM_WGPU_CAMFX': '0', 'OMNISIM_WGPU_AUTOEXP': '0',
       'OMNISIM_WGPU_REPORT_EVERY': '1', 'OMNILIGHT': '0',
       'OMNISIM_WGPU_LOCAL_SHADOW_CACHE': str(int(enabled)), 'OMNISIM_WGPU_LOCAL_SHADOW_CULL': str(int(enabled))}
results = []
with benchmark_world('city', 5) as (world, _):
    for i in range(2):
        prefix = dest / str(i)
        record = render(world, prefix.with_suffix('.png'), 90, env,
                        prefix.with_suffix('.log'), prefix.with_suffix('.report'), 35)
        assert record['dumped'] and not record['errors']
        results.append(record)
        print(i, json.dumps(record['profile']), flush=True)
(dest / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
