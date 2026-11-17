import os,sys,subprocess
os.environ['OMNISIM_HOME']='O:/omnisim'
for command in (['-m','pytest','docs/tests','-q'],['-m','pytest','tests/sources/test_license.py','-q'],['-m','pytest','tests/test_env_reference_current.py','-q'],['.local-runs/beauty-realism/motion-quality.py']):
    subprocess.run([sys.executable,*command],check=True)
