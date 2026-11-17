from pathlib import Path
import sys, os, json
repo=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(repo))
os.environ['PATH']=str(repo/'msys64/mingw64/bin/newton-runtime')+os.pathsep+str(repo/'msys64/mingw64/bin')+os.pathsep+os.environ['PATH']
from scripts.dev.render_ab import render,summarize_gpu_profile
from scripts.dev.render_shadow_bench import benchmark_world
out=repo/'.local-runs/temporal-pipeline';out.mkdir(exist_ok=True)
with benchmark_world('house',5) as (world,_):
    result=render(world,out/'smoke.png',96,{'OMNISIM_BINARY':str(repo/'msys64/mingw64/bin/omnisim-bin-temporal.exe'),'OMNILIGHT':'0','OMNISIM_WGPU_CAMFX':'0','OMNISIM_WGPU_AUTOEXP':'0','OMNISIM_WGPU_GPU_TIMING':str(out/'smoke.gpu')},out/'smoke.log',out/'smoke.cpu',35)
print(json.dumps(result),flush=True)
print(json.dumps(summarize_gpu_profile((out/'smoke.gpu').read_text())),flush=True)
assert result['dumped'] and result['errors']==0
