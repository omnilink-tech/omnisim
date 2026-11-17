"""Build a reviewable static page and immutable evidence ZIP after a complete run.

This writes only a new output directory. It does not deploy or change evidence.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
from html import escape
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from omnisim.control_bench.evidence import verify, stats, paired, sha
from omnisim.control_bench.agents import is_unbilled_overload

HERE=Path(__file__).resolve().parent
BENCH=HERE.parent
LABELS={'omnilink_parser':'OmniLink parser integration','plain':'Plain model loop',
        'langgraph':'LangGraph','lobster':'Lobster SDK',
        'langgraph_parser':'LangGraph + OmniLink parser','lobster_parser':'Lobster SDK + OmniLink parser'}


def write(path,text):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(text,encoding='utf-8')


def pack(files,path,private_tokens=(),evidence_directory=None,redact_identities=False,secret_tokens=()):
    manifest={}; redactions={}
    with zipfile.ZipFile(path,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name,source in sorted(files.items()):
            data=source if isinstance(source,bytes) else source.read_bytes()
            if any(token and token in data for token in secret_tokens):
                raise ValueError('Credential found in archive member: '+name)
            if any(token and token in data for token in private_tokens):
                if not redact_identities: raise ValueError('Private token found in archive member: '+name)
                original=hashlib.sha256(data).hexdigest()
                for token in private_tokens:
                    if token: data=data.replace(token,b'<private-identity>')
                redactions[name]={'original_sha256':original,'export_sha256':hashlib.sha256(data).hexdigest(),
                                  'change':'Private account/OS identity strings replaced; original local evidence retained.'}
            manifest[name]=hashlib.sha256(data).hexdigest()
            z.writestr(name,data)
        z.writestr('MANIFEST.json',json.dumps({'schema':'omnisim-publication-manifest/1',
                    'evidence_directory':evidence_directory,'sha256':manifest,'identity_redactions':redactions},indent=2)+'\n')
    return sha(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign',default='publication-01')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--key-file',type=Path,help='Scan bytes for this key; never archive it')
    args=parser.parse_args()
    evidence=BENCH/'evidence'/args.campaign
    lock,rows,verification=verify(evidence)
    if lock.get('provider',{}).get('provider')=='anthropic-direct':
        raise ValueError('Exploratory model pilots require a separate report; not supported by this homepage comparison builder')
    if not (verification['complete'] and verification['rows_digest_matches'] and verification['lock_digest_matches']
            and verification['source_snapshots_match'] and verification['regrade_available']
            and not verification['regrade_mismatches'] and not verification['accounting_mismatches']
            and verification['integrity'].get('unchanged')):
        raise ValueError('No publication build from incomplete or unverifiable evidence: '+json.dumps(verification))
    if any(r['derived_usd'] is None for r in rows): raise ValueError('Unknown model costs')
    out=args.out.resolve(); out.mkdir(parents=True,exist_ok=False)
    reproduce=(HERE/'REPRODUCE.md').read_text(encoding='utf-8')
    for key,value in {'{{MODEL}}':lock['model'],'{{REPEATS}}':str(lock['repeats']),
            '{{CAP}}':str(lock['max_model_usd']),'{{TASKS}}':str(len(lock['suite']['tasks'])),
            '{{TOTAL}}':str(len(rows)),**{'{{RATE_'+k.upper()+'}}':str(v) for k,v in lock['rates'].items()}}.items():
        reproduce=reproduce.replace(key,value)
    if '{{' in reproduce: raise ValueError('Unfilled reproduction instructions')
    summary={arm:stats([r for r in rows if r['arm']==arm]) for arm in lock['arms']}
    rejections={arm:sum(is_unbilled_overload(q) for r in rows if r['arm']==arm for q in r['requests']) for arm in lock['arms']}
    sensitivity={arm:(s['derived_usd']+.01*rejections[arm])/s['pass'] if s['pass'] else None for arm,s in summary.items()}
    comparisons={arm:paired(rows,arm,lock['suite']) for arm in lock['arms'] if arm!='omnilink_parser'}
    failures=[{'arm':r['arm'],'task':r['task'],'repeat':r['repeat'],'outcome':r['outcome'],
               'reasons':[reason for t in r['turns'] for reason in t['grade']['reasons']],
               'error':r.get('error')} for r in rows if r['outcome']!='PASS']
    public={'campaign':args.campaign,'scope':lock['scope'],'suite':lock['suite']['name'],
            'split':lock['suite']['split'],'provenance':lock['suite']['provenance'],
            'summary':summary,'comparisons':comparisons,'failures':failures,'verification':verification,
            'model':lock['model'],'rates':lock['rates'],'machine':lock['machine'],
            'transport_policy':lock.get('transport_policy'), 'provider_overload_rejections':rejections,
            'cost_sensitivity_usd_per_success_if_each_rejection_costs_001':sensitivity,
            'dependencies':lock['dependencies'],'lock_sha256':sha(evidence/'lock.json'),
            'rows_sha256':sha(evidence/'rows.jsonl')}
    write(out/'results.json',json.dumps(public,indent=2)+'\n')
    files={}
    # Preserve original records byte-for-byte, including all physics logs.
    for p in evidence.rglob('*'):
        if p.is_file(): files['evidence/'+args.campaign+'/'+p.relative_to(evidence).as_posix()]=p
    for p in (evidence/'source/omnisim/control_bench').glob('*'):
        if p.suffix in ('.py','.mjs'): files['omnisim/control_bench/'+p.name]=p
    for name in ('__init__.py','paths.py'): files['omnisim/'+name]=ROOT/'omnisim'/name
    files['verify_archive.py']=HERE/'verify_archive.py'
    files['REPRODUCE.md']=reproduce.encode('utf-8')
    files['reproduction/suite.json']=(json.dumps(lock['suite'],indent=2)+'\n').encode()
    files['reproduction/requirements.lock.txt']=ROOT/'tests/benchmarks/harness_comparison/requirements.lock.txt'
    for name in ('SPEC.md','CAMPAIGN_01.md','REPORT.md','FIX_REPORT.md','SERVICE_DIAGNOSIS.md'):
        files['reproduction/'+name]=BENCH/name
    for p in BENCH.glob('CAMPAIGN_*.md'): files['reproduction/'+p.name]=p
    files['LICENSE']=ROOT/'LICENSE'
    # Include hash-pinned geometry and actual compiled SDK outside source snapshots.
    for name,digest in lock['inputs'].items():
        source=ROOT/name
        if name.startswith(('msys64/','lib/controller/')) or (evidence/'source'/name).exists(): continue
        if sha(source)!=digest: raise ValueError('Input changed before packaging: '+name)
        files['reproduction/extra-inputs/'+name]=source
    vendor=ROOT/'tests/benchmarks/harness_comparison/vendor/lobster'
    for name in ('LICENSE','README.md','package.json','package-lock.json','pnpm-lock.yaml'):
        files['reproduction/extra-inputs/tests/benchmarks/harness_comparison/vendor/lobster/'+name]=vendor/name
    files['reproduction/bridge-LICENSE']=ROOT/'packages/omnisim-bridges/LICENSE'
    redactions=ROOT/'scripts/release/public_redactions.txt'
    tokens=[line.split('\t')[0].encode() for line in redactions.read_text().splitlines()
            if line.strip() and not line.startswith('#')] if redactions.exists() else []
    secrets=[args.key_file.read_bytes().strip()] if args.key_file else []
    archive_hash=pack(files,out/'evidence.zip',tokens,'evidence/'+args.campaign,secret_tokens=secrets)
    history={}
    for directory in sorted((BENCH/'evidence').iterdir()):
        if directory==evidence or not directory.is_dir(): continue
        for p in directory.rglob('*'):
            if p.is_file(): history['history/'+directory.name+'/'+p.relative_to(directory).as_posix()]=p
    for p in BENCH.glob('transport-diagnosis-*.json'): history['history/'+p.name]=p
    history['HISTORY_EXPORT.md']=('Earlier runs are retained separately and are not pooled with the main campaign.\n'
        'This is a privacy export: private account/OS identity strings in historical files may be replaced.\n'
        'MANIFEST.json lists original and exported hashes for every altered file; original local records remain unchanged.\n'
        'Historical completion/source digests refer to the original files and can differ after redaction.\n'
        'The main evidence archive is byte-preserving and separately verifies.\n').encode()
    history_hash=pack(history,out/'development-history.zip',tokens,redact_identities=True,secret_tokens=secrets)
    write(out/'SHA256SUMS.txt',f'{archive_hash}  evidence.zip\n{history_hash}  development-history.zip\n')
    write(out/'REPRODUCE.md',reproduce)
    for name in ('SPEC.md',): shutil.copyfile(BENCH/name,out/name)
    table=[]
    for arm,s in summary.items():
        table.append(f'<tr><th scope="row">{escape(LABELS[arm])}</th><td>{s["pass"]}/{s["episodes"]}</td>'
                     f'<td>{s["unsafe"]}</td><td>{s["error"]}</td><td>{s["model_calls"]}</td><td>{rejections[arm]}</td>'
                     f'<td>${s["usd_per_success"]:.5f}</td><td>{s["seconds_per_success"]:.2f}s</td></tr>')
    interval_rows=[]
    for arm,p in comparisons.items():
        def interval(key):
            a,b=p['family_bootstrap_95'][key]
            return f'{p["point"][key]:.3f} [{a:.3f}, {b:.3f}]'
        interval_rows.append(f'<tr><th scope="row">{escape(LABELS[arm])}</th><td>{interval("success_difference_pp")}</td>'
                             f'<td>{interval("cost_per_success_ratio")}</td><td>{interval("time_per_success_ratio")}</td></tr>')
    failures_html=''.join(f'<tr><th scope="row">{escape(LABELS[f["arm"]])}</th><td>{escape(f["task"])}</td>'
                          f'<td>{f["repeat"]+1}</td><td>{escape(f["outcome"])}</td>'
                          f'<td>{escape(", ".join(f["reasons"]) or str(f["error"]))}</td></tr>' for f in failures)
    omni=summary['omnilink_parser']
    baseline=summary.get('plain')
    comparative_note=''
    if baseline and baseline['usd_per_success'] and omni['usd_per_success'] is not None:
        change=100*(omni['usd_per_success']/baseline['usd_per_success']-1)
        direction='lower' if change<0 else 'higher'
        comparative_note=(f'In this run, OmniLink’s estimated model cost per successful task was {abs(change):.1f}% {direction} '
            f'than the tested plain model loop (${baseline["usd_per_success"]:.5f}). '
            f'Completion: OmniLink {omni["pass"]}/{omni["episodes"]}; plain loop {baseline["pass"]}/{baseline["episodes"]}. '
            'The full table also includes both frameworks with the same OmniLink parser; use those controls when interpreting attribution.')
    families=len({t['family'] for t in lock['suite']['tasks']})
    substitutions={'{{TABLE}}':''.join(table),'{{INTERVALS}}':''.join(interval_rows),
                   '{{OVERLOADS}}':str(sum(rejections.values())),
                   '{{COMPARATIVE_NOTE}}':escape(comparative_note),
                   '{{ERROR_COST_NOTE}}':('Cost estimates assume recognized provider-overload errors incur no token charge under Google’s published policy. Rejections, retry waiting time and an alternative cost assumption are disclosed in the full results.'
                       if sum(rejections.values()) else 'All model requests in this campaign returned usage. No overload-error cost assumption was needed.'),
                   '{{SENSITIVITY}}':''.join(f'<tr><th scope="row">{escape(LABELS[arm])}</th><td>{rejections[arm]}</td>'
                       f'<td>${summary[arm]["usd_per_success"]:.5f}</td><td>${sensitivity[arm]:.5f}</td></tr>' for arm in lock['arms']),
                   '{{FAILURES}}':failures_html or '<tr><td colspan="5">No failed episodes.</td></tr>',
                   '{{PASS}}':str(omni['pass']),'{{ATTEMPTS}}':str(omni['episodes']),
                   '{{PERCENT}}':f'{100*omni["success_rate"]:.1f}',
                   '{{COST}}':f'{omni["usd_per_success"]:.5f}',
                   '{{MOTION}}':str(omni['unsafe']),'{{TOTAL}}':str(len(rows)),
                   '{{TASKS}}':str(len(lock['suite']['tasks'])),'{{REPEATS}}':str(lock['repeats']),
                   '{{FAMILIES}}':str(families),'{{ZIPHASH}}':archive_hash,
                   '{{MODEL}}':escape(lock['model']),
                   **{'{{RATE_'+k.upper()+'}}':str(v) for k,v in lock['rates'].items()},
                   '{{CAMPAIGN}}':escape(args.campaign),'{{LOCKHASH}}':sha(evidence/'lock.json'),
                   '{{ROWHASH}}':sha(evidence/'rows.jsonl')}
    html=(HERE/'template.html').read_text(encoding='utf-8')
    for key,value in substitutions.items(): html=html.replace(key,value)
    if '{{' in html: raise ValueError('Unfilled page template')
    write(out/'index.html',html)
    homepage=(HERE/'homepage-template.html').read_text(encoding='utf-8')
    for key,value in substitutions.items(): homepage=homepage.replace(key,value)
    if '{{' in homepage: raise ValueError('Unfilled homepage template')
    write(out/'homepage-section.html',homepage)
    shutil.copyfile(HERE/'homepage.css',out/'homepage.css')
    claim=f'OmniLink’s parser integration completed {omni["pass"]} of {omni["episodes"]} trials ({100*omni["success_rate"]:.1f}%) on our {len(lock["suite"]["tasks"])}-task robotics control development suite in OmniSim.'
    write(out/'HOMEPAGE_COPY.md',f'# Measured robotics control\n\n{claim}\n\n'
          f'{comparative_note}\n\n'
          f'All six implementations, all {len(rows)} trials, source code, physics logs and offline verification are included. '
          'Developer-authored tasks; simulation only; assisted arm grasping. These results do not establish a market-wide winner.\n\n'
          f'Model: {lock["model"]}. Recognized provider-overload rejections: {sum(rejections.values())}. '
          'Where present, their estimated zero token charge is a disclosed billing-policy assumption, with null usage retained and a sensitivity table.\n\n'
          'Publish alongside the matching OmniSim 9.0.0 source/runtime release. Link to the full benchmark page and evidence archive.\n')
    print(json.dumps({'page':str(out/'index.html'),'episodes':len(rows),'evidence_sha256':archive_hash,
                      'archive_bytes':(out/'evidence.zip').stat().st_size,'known_model_usd':sum(r['derived_usd'] for r in rows)},indent=2))


if __name__=='__main__': main()
