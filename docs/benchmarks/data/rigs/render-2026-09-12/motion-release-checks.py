from pathlib import Path
import sys,subprocess,os
repo=Path(__file__).resolve().parents[2]
os.environ['OMNISIM_BINARY']=str(repo/'msys64/mingw64/bin/omnisim-bin-motion-final.exe')
os.environ['OMNISIM_TEST_MOTION_OUTPUT']=str(repo/'.local-runs/motion-pipeline/release-moving')
runtime=repo/'msys64/mingw64/bin/newton-runtime'
os.environ['PATH']=str(runtime)+os.pathsep+str(runtime.parent)+os.pathsep+str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH','')
subprocess.run([sys.executable,'-m','pytest','tests/test_object_motion_scene.py','tests/test_local_shadow_cache.py','tests/test_local_light_shadows.py','tests/test_gpu_profile.py','tests/test_render_ab_views.py','-q'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-final-controls.py'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/shadow-broad-checks.py'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-quality.py'],check=True)
