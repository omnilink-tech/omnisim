from pathlib import Path
import os,sys,subprocess
repo=Path(__file__).resolve().parents[2]
os.environ['OMNISIM_BINARY']=str(repo/'msys64/mingw64/bin/omnisim-bin-motion.exe')
os.environ['OMNISIM_TEST_MOTION_OUTPUT']=str(repo/'.local-runs/motion-pipeline/moving')
subprocess.run([sys.executable,'-m','pytest','tests/test_object_motion_scene.py','-q','-s'],check=True)
