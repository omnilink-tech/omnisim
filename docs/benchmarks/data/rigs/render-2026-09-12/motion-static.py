from pathlib import Path
import sys, os, json
repo=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(repo))
from scripts.dev.render_ab import render,diff
from scripts.dev.render_shadow_bench import benchmark_world
out=repo/'.local-runs/motion-pipeline/static';out.mkdir(parents=True,exist_ok=True)
runtime=repo/'msys64/mingw64/bin/newton-runtime'
env={'OMNISIM_BINARY':str(repo/'msys64/mingw64/bin/omnisim-bin-motion.exe'),'OMNILIGHT':'0',
     'OMNISIM_WGPU_TAA':'1','OMNISIM_WGPU_CAMFX':'0','OMNISIM_WGPU_AUTOEXP':'0',
     'PATH':str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+os.environ['PATH']}
results=[]
for scene in ('house','warehouse','city'):
    record={'scene':scene,'arms':{}}
    with benchmark_world(scene,5) as (world,source):
        for motion in (0,1):
            prefix=out/f'{scene}-{motion}'
            row=render(world,prefix.with_suffix('.png'),96,
                dict(env,OMNISIM_WGPU_MOTION_VECTORS=str(motion),OMNISIM_WGPU_GPU_TIMING=str(prefix.with_suffix('.gpu'))),
                prefix.with_suffix('.log'),prefix.with_suffix('.cpu'),35)
            row['environment'].pop('PATH',None)
            record['arms'][str(motion)]=row
            print(scene,motion,row['errors'],row['gpu_profile'],flush=True)
            assert row['dumped'] and row['errors']==0
    record['difference']=diff(out/f'{scene}-0.png',out/f'{scene}-1.png',0)
    results.append(record);(out/'results.json').write_text(json.dumps(results,indent=2))
