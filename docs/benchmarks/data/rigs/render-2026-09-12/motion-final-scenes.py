from pathlib import Path
import os,sys,subprocess
repo=Path(__file__).resolve().parents[2]
os.environ['OMNISIM_BINARY']=str(repo/'msys64/mingw64/bin/omnisim-bin-motion-final.exe')
os.environ['OMNISIM_TEST_MOTION_OUTPUT']=str(repo/'.local-runs/motion-pipeline/final-moving')
subprocess.run([sys.executable,'-m','pytest','tests/test_object_motion_scene.py','-q','-s'],check=True)

subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-cloth.py'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-final-controls.py'],check=True)
