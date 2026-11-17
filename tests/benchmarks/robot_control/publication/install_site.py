"""Stage verified benchmark artifacts and homepage copy; never deploy."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

p=argparse.ArgumentParser()
p.add_argument('--artifacts',type=Path,required=True)
p.add_argument('--website',type=Path,required=True)
p.add_argument('--patch',type=Path,required=True)
p.add_argument('--apply',action='store_true')
a=p.parse_args()
artifacts=a.artifacts.resolve()
site=a.website.resolve()
results=json.loads((artifacts/'results.json').read_text(encoding='utf-8'))
if not results['verification']['complete']: raise SystemExit('Incomplete campaign')
for line in (artifacts/'SHA256SUMS.txt').read_text().splitlines():
    digest,name=line.split()
    path=(artifacts/name).resolve()
    if not path.is_relative_to(artifacts) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
        raise SystemExit('Artifact checksum mismatch')
target=site/'public/benchmarks/robotics-control'
if target.exists(): raise SystemExit('Refusing to overwrite existing published artifacts')
path=site/'index.html'
original=path.read_bytes()
text=original.decode('utf-8')
marker='    <section class="outcomes wrap section" aria-labelledby="outcome-title">'
if text.count(marker)!=1 or 'id="robotics-benchmark"' in text:
    raise SystemExit('Homepage changed or benchmark already integrated')
newline='\r\n' if '\r\n' in text else '\n'
fragment=(artifacts/'homepage-section.html').read_text(encoding='utf-8').rstrip().replace('\n',newline)
updated=text.replace(marker,fragment+newline+newline+marker)
patch=''.join(difflib.unified_diff(text.splitlines(keepends=True),updated.splitlines(keepends=True),
             fromfile='a/index.html',tofile='b/index.html'))
css_path=site/'public/landing.css'
css_original=css_path.read_bytes()
css=css_original.decode('utf-8')
if '.robotics-benchmark>' in css: raise SystemExit('Benchmark styles already integrated')
css_updated=css.rstrip()+'\n'+(artifacts/'homepage.css').read_text(encoding='utf-8')
patch+=''.join(difflib.unified_diff(css.splitlines(keepends=True),css_updated.splitlines(keepends=True),
             fromfile='a/public/landing.css',tofile='b/public/landing.css'))
a.patch.parent.mkdir(parents=True,exist_ok=True)
a.patch.write_bytes(patch.encode('utf-8'))
if a.apply:
    if path.read_bytes()!=original or css_path.read_bytes()!=css_original: raise SystemExit('Homepage changed while preparing')
    target.mkdir(parents=True)
    for name in ('index.html','results.json','evidence.zip','development-history.zip','SHA256SUMS.txt','REPRODUCE.md','SPEC.md'):
        shutil.copyfile(artifacts/name,target/name)
    path.write_bytes(updated.encode('utf-8'))
    css_path.write_bytes(css_updated.encode('utf-8'))
print(json.dumps({'applied':a.apply,'homepage':str(path),'artifacts':str(target),'patch':str(a.patch),
                  'deployment':'not performed','release_dependency':'matching OmniSim 9.0.0 source/runtime'},indent=2))
