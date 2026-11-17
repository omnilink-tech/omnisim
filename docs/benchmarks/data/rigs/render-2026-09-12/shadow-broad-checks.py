import os
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
os.chdir(root)
os.environ['OMNISIM_HOME'] = str(root)
files = subprocess.check_output([sys.executable, 'tests/conftest.py', '--list-engine-free', 'tests'], text=True).split()
commands = [
    ['-m', 'pytest', '-m', 'not engine', '-q', 'tests/harness', 'tests/python', 'tests/packaging', 'tests/dev', *files],
    ['-m', 'pytest', 'docs/tests', '-q'],
    ['-m', 'pytest', 'tests/sources/test_license.py', '-q'],
]
for command in commands:
    print('CHECK:', ' '.join(command[:9]), flush=True)
    subprocess.run([sys.executable, *command], check=True)
