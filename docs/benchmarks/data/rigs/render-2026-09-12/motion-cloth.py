from pathlib import Path
import os,sys,json,re,tempfile
repo=Path(__file__).resolve().parents[2];sys.path.insert(0,str(repo))
from scripts.dev.render_ab import render,summarize_gpu_profile
out=repo/'.local-runs/motion-pipeline/cloth';out.mkdir(parents=True,exist_ok=True)
runtime=repo/'msys64/mingw64/bin/newton-runtime'
env={'OMNISIM_BINARY':str(repo/'msys64/mingw64/bin/omnisim-bin-motion-final.exe'),
     'OMNILIGHT':'0','OMNISIM_WGPU_TAA':'1','OMNISIM_WGPU_MOTION_VECTORS':'1',
     'OMNISIM_WGPU_CAMFX':'0','OMNISIM_WGPU_AUTOEXP':'0','OMNISIM_WGPU_GPU_TIMING':str(out/'render.gpu'),
     'OMNISIM_CAM_SAMPLE_DIR':str(out),'OMNISIM_CAM_SAMPLE_STEPS':'8,64',
     'PATH':str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+os.environ['PATH']}
source=repo/'projects/samples/demos/worlds/rendering/camera_cloth_wgpu_smoke.omniworld'
fd,name=tempfile.mkstemp(prefix='.motion_cloth_',suffix='.omniworld',dir=source.parent);os.close(fd)
world=Path(name)
try:
    text=source.read_text().replace('WorldInfo {','WorldInfo {\n FPS 8',1)
    text=re.sub(r'Viewpoint\s*\{[^}]*\}', 'Viewpoint { position -1.6 0 .85 orientation 0 0 1 0 fieldOfView .9 }', text, count=1)
    world.write_text(text)
    row=render(world,out/'cloth.png',40,env,out/'engine.log',out/'render.cpu',40)
    row['environment'].pop('PATH',None)
    row['gpu_profile']=summarize_gpu_profile((out/'render.gpu').read_text(),warmup_frames=0)
    (out/'result.json').write_text(json.dumps(row,indent=2))
    print(row,flush=True)
    assert row['dumped'] and row['errors']==0
    assert any(p.get('motionUs',{}).get('p50',0)>0 for p in row['gpu_profile'])
    from PIL import Image
    import numpy as np
    a=np.asarray(Image.open(out/'sample_0008.ppm'),dtype='int16')
    b=np.asarray(Image.open(out/'sample_0064.ppm'),dtype='int16')
    delta=int((np.abs(a-b)>5).sum())
    print('cloth sensor changed channels:',delta,flush=True)
    assert delta>50
finally:
    world.unlink(missing_ok=True)
