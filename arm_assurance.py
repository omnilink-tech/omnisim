"""Public OmniArm 6 assurance pilot launcher; no credentials are bundled."""
import argparse
from pathlib import Path
import runpy
import sys

def main():
    p=argparse.ArgumentParser(description='Run or verify the bounded OmniArm 6 simulation assessment.')
    p.add_argument('action',choices=['run','suite','controlled','verify'])
    args=p.parse_args(sys.argv[1:2]);remaining=sys.argv[2:]
    directory=Path(__file__).resolve().parent/'projects/services/arm_assurance'
    scripts={'run':'qualification.py','suite':'qualification.py','controlled':'proposals.py','verify':'verify.py'}
    if args.action=='run' and not any(a=='--episode' or a.startswith('--episode=') for a in remaining):remaining=['--episode','0',*remaining]
    script=directory/scripts[args.action]
    sys.path.insert(0,str(directory));sys.argv=[str(script),*remaining];runpy.run_path(str(script),run_name='__main__')

if __name__=='__main__':main()
