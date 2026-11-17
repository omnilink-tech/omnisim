import sys,subprocess
subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-cloth.py'],check=True)
subprocess.run([sys.executable,'.local-runs/beauty-realism/motion-final-controls.py'],check=True)
