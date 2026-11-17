from pathlib import Path
import os, sys, subprocess
root=Path(__file__).resolve().parents[2]
os.environ['OMNISIM_HOME']=str(root)
os.environ['OMNISIM_BINARY']=str(root/'msys64/mingw64/bin/omnisim-bin-temporal-final.exe')
runtime=root/'msys64/mingw64/bin/newton-runtime'
os.environ['PATH']=str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+str(Path(sys.executable).parent)+os.pathsep+os.environ['PATH']
subprocess.run([sys.executable,'-m','pytest','tests/rendering/test_taa_validation_gpu.py',
    'tests/test_gpu_profile.py','tests/test_local_shadow_cache.py','tests/test_local_light_shadows.py',
    'tests/test_render_ab_views.py','-q','-s'],check=True)
subprocess.run([sys.executable,'-m','pytest','tests/test_local_shadow_cache.py','-q'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/shadow-broad-checks.py'],check=True)
