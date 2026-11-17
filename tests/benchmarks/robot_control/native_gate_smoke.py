"""Check the real /prompt and /tool paths; run after timing comparisons."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from omnisim.control_bench.engine import Engine,Observation,request,failed
from omnisim.control_bench.tasks import grade_turn
from omnisim.control_bench.evidence import dump,sha
from omnisim.paths import REPO_ROOT


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--key-file',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    engine=None
    records=[]
    try:
        engine=Engine('husky',args.out/'engine',args.key_file.read_text().strip())
        prompt='Back up a bit.'
        obs=Observation(engine,prompt)
        reply={}
        try: reply=request(engine.url+'/prompt',{'text':prompt})
        finally: measured=obs.finish({})
        grade=grade_turn({'expect':{'no_motion':True}},measured,'mobile',engine.home)
        records.append({'endpoint':'/prompt','reply':reply,'grade':grade,'observation':measured})
        assert grade['outcome']=='PASS' and reply.get('via')=='parser', records[-1]['grade']
        assert any(a.get('tool')=='clarify' for a in reply.get('actions',[])), reply

        obs=Observation(engine,'Robot 2, back up a bit.')
        try: reply=obs.dispatch({'tool':'drive_forward','args':{'distance':-.5}})
        finally: measured=obs.finish({})
        grade=grade_turn({'expect':{'no_motion':True}},measured,'mobile',engine.home)
        records.append({'endpoint':'/tool','reply':reply,'grade':grade,'observation':measured})
        assert grade['outcome']=='PASS' and failed(reply), grade
    finally:
        cleanup=engine.close() if engine else []
        dump(args.out/'results.json',{'checks':records,'cleanup_errors':cleanup,
             'source_sha256':{n:sha(REPO_ROOT/n) for n in (
                 'packages/omnisim-bridges/src/omnisim_bridges/gate.py',
                 'packages/omnisim-bridges/src/omnisim_bridges/interpret.py')},
             'physics':engine.physics if engine else None})
    if cleanup: return 1
    print(json.dumps({'checks':len(records),'passed':True,'directory':str(args.out)}))
    return 0


if __name__=='__main__': raise SystemExit(main())
