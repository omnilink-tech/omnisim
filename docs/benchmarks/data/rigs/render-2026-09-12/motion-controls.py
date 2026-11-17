from pathlib import Path
import os,sys,json
repo=Path(__file__).resolve().parents[2];sys.path.insert(0,str(repo))
from scripts.dev.render_ab import render,diff
from scripts.dev.render_shadow_bench import benchmark_world
out=repo/'.local-runs/motion-pipeline/controls';out.mkdir(parents=True,exist_ok=True)
runtime=repo/'msys64/mingw64/bin/newton-runtime'
env={'OMNISIM_BINARY':str(repo/'msys64/mingw64/bin/omnisim-bin-motion.exe'),
     'OMNILIGHT':'0','OMNISIM_WGPU_TAA':'0','OMNISIM_WGPU_CAMFX':'0','OMNISIM_WGPU_AUTOEXP':'0',
     'PATH':str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+os.environ['PATH']}
records=[]
for scene in ('house','warehouse','city'):
    prefix=out/scene
    with benchmark_world(scene,5) as (world,_):
        row=render(world,prefix.with_suffix('.png'),96,dict(env,OMNISIM_WGPU_GPU_TIMING=str(prefix.with_suffix('.gpu'))),
                   prefix.with_suffix('.log'),prefix.with_suffix('.cpu'),35)
    reference=repo/'.local-runs/temporal-pipeline'/('house' if scene=='house' else 'other-scenes')/f'{scene}-0-baseline.png'
    comparison=diff(reference,prefix.with_suffix('.png'),0)
    row['environment'].pop('PATH',None)
    records.append({'scene':scene,'result':row,'baseline_image_check':comparison})
    (out/'results.json').write_text(json.dumps(records,indent=2))
    print(scene,comparison,flush=True)
    assert row['dumped'] and row['errors']==0 and comparison['pixels_over_threshold']==0
    assert any('cpuIntervalUs' in t and t['gpuSpanUs']['p50']>0 for t in row['gpu_profile'])
