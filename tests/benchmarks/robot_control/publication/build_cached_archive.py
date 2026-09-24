"""Package only a completed, audited cached comparison for offline verification."""
import argparse
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
BENCH=HERE.parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(BENCH))
from audit_anthropic_campaign import audit
from build import pack
from omnisim.control_bench.evidence import sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign',default='opus-cached-comparison-01')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--key-file',type=Path,action='append',required=True)
    a=p.parse_args();evidence=BENCH/'evidence'/a.campaign
    if a.out.exists():raise ValueError('Output already exists')
    result=audit(evidence,BENCH/'evidence/publication-04',a.key_file)
    if not result['complete']:raise ValueError('Do not package an incomplete comparison as a completed benchmark')
    lock=json.loads((evidence/'lock.json').read_text(encoding='utf-8'))
    if not lock['provider'].get('cache_policy'):raise ValueError('Cached comparison required')
    a.out.mkdir(parents=True)
    files={}
    for f in evidence.rglob('*'):
        if f.is_file():files['evidence/'+a.campaign+'/'+f.relative_to(evidence).as_posix()]=f
    for f in (evidence/'source/omnisim/control_bench').glob('*'):
        if f.suffix in ('.py','.mjs'):files['omnisim/control_bench/'+f.name]=f
    for name in ('__init__.py','paths.py'):files['omnisim/'+name]=ROOT/'omnisim'/name
    files['verify_archive.py']=HERE/'verify_archive.py'
    files['LICENSE']=ROOT/'LICENSE'
    files['reproduction/suite.json']=(json.dumps(lock['suite'],indent=2)+'\n').encode()
    files['reproduction/requirements.lock.txt']=ROOT/'tests/benchmarks/harness_comparison/requirements.lock.txt'
    files['reproduction/PROTOCOL.md']=evidence/'source'/lock['protocol']['path']
    files['reproduction/SPEC.md']=BENCH/'SPEC.md'
    for name,digest in lock['inputs'].items():
        if name.startswith(('msys64/','lib/controller/')) or (evidence/'source'/name).exists():continue
        f=ROOT/name
        if sha(f)!=digest:raise ValueError('Input changed: '+name)
        files['reproduction/extra-inputs/'+name]=f
    vendor=ROOT/'tests/benchmarks/harness_comparison/vendor/lobster'
    for name in ('LICENSE','README.md','package.json','package-lock.json','pnpm-lock.yaml'):
        files['reproduction/extra-inputs/tests/benchmarks/harness_comparison/vendor/lobster/'+name]=vendor/name
    files['reproduction/bridge-LICENSE']=ROOT/'packages/omnisim-bridges/LICENSE'
    readme='''# Reproduce the cached robotics-control comparison

Extract into a new directory and run `python -I -S verify_archive.py` with
Python 3.12. This uses only the standard library: no key, network, simulator or
private checkout. It rechecks manifest hashes, full coverage, measured grades,
model/usage accounting, physics sidecars and the durable budget journal.
Hashes establish consistency, not independent authorship or external validation.

MATCHED_COMPARISON.md and ANTHROPIC_AUDIT.json under the evidence directory
contain all six results and limitations. The raw evidence is byte-preserved.
Per-arm costs cover scored calls; shared cache warm-up is charged separately
and included in the campaign total. The audit also allocates it equally across
arms as a sensitivity check. Cache effects use actual provider usage counters.

A LIVE rerun is different from offline verification. It requires the matching
OmniSim 9.0.0 Windows runtime, robot assets, bridge SDK and dependencies recorded
in lock.json; the model may change server-side. Restore source/ and
reproduction/extra-inputs/ into a separate matching checkout, install the pinned
requirements, use Node 22.19.0, and provide your own separate OmniKey and
Anthropic API key in private files. See reproduction/PROTOCOL.md for exact
freeze/run commands and the $5 ceiling, including warm-up. Never overwrite the
supplied evidence. Other operating systems need a build and separate calibration.

The archive does not include the multi-gigabyte engine or all runtime binaries.
Public live reproduction requires publishing the matching source/runtime release;
do not substitute an older public release. Offline verification works now.
This is a developer-authored development suite, one repeat, in simulation with
assisted grasp. It is not a market-wide capability ranking, independent holdout
or proof of real-hardware performance. No homepage deployment is performed here.
'''
    files['REPRODUCE.md']=readme.encode()
    redactions=ROOT/'scripts/release/public_redactions.txt'
    private=[s.split('\t')[0].encode() for s in redactions.read_text().splitlines()
             if s.strip() and not s.startswith('#')] if redactions.exists() else []
    secrets=[f.read_text(encoding='utf-8-sig').strip().encode() for f in a.key_file]
    digest=pack(files,a.out/'evidence.zip',private,'evidence/'+a.campaign,secret_tokens=secrets)
    history={}
    for directory in sorted((BENCH/'evidence').iterdir()):
        if directory==evidence or not directory.is_dir():continue
        for f in directory.rglob('*'):
            if f.is_file():history['history/'+directory.name+'/'+f.relative_to(directory).as_posix()]=f
    for f in BENCH.glob('transport-diagnosis-*.json'):history['history/'+f.name]=f
    for pattern in ('CAMPAIGN_*.md','OPUS_*.md'):
        for f in BENCH.glob(pattern):history['protocols/'+f.name]=f
    history['HISTORY_EXPORT.md']=('Earlier, failed and interrupted runs are retained separately, never pooled with the completed cached campaign.\n'
        'The stopped uncached run includes its unresolved in-flight request reservation; do not call that charge zero.\n'
        'Private identity strings may be redacted in this historical export. MANIFEST.json records original and exported hashes.\n'
        'Original local records remain unchanged; historical internal digests may refer to their original unredacted bytes.\n').encode()
    history_digest=pack(history,a.out/'development-history.zip',private,redact_identities=True,secret_tokens=secrets)
    (a.out/'SHA256SUMS.txt').write_text(digest+'  evidence.zip\n'+history_digest+'  development-history.zip\n',encoding='utf-8')
    (a.out/'REPRODUCE.md').write_text(readme,encoding='utf-8')
    for name in ('MATCHED_COMPARISON.md','ANTHROPIC_AUDIT.json'):
        (a.out/name).write_bytes((evidence/name).read_bytes())
    print(json.dumps({'archive':str(a.out/'evidence.zip'),'sha256':digest,'bytes':(a.out/'evidence.zip').stat().st_size}))


if __name__=='__main__':main()
