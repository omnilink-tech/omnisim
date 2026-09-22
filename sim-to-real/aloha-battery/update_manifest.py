# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Verify the study's archive hashes; --write rebuilds the manifest after edits."""
import argparse
from datetime import date
import hashlib
import json
from pathlib import Path
import subprocess

ARCHIVE=Path(__file__).resolve().parent
ROOT=ARCHIVE.parents[1]
MANIFEST=ARCHIVE/'manifest.json'


def normalized(data,canonical):
    try:
        data.decode('utf-8')
        text=b'\0' not in data
    except UnicodeDecodeError:
        text=False
    return (data.replace(b'\r\n',b'\n'),'CRLF to LF') if canonical and text else (data,'none')


def entry(path,canonical=False):
    raw=path.read_bytes();data,mode=normalized(raw,canonical)
    return {'path':path.relative_to(ROOT).as_posix(),'bytes':len(raw),
            'sha256':hashlib.sha256(data).hexdigest(),'normalization':mode}


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--write',action='store_true');args=ap.parse_args()
    if args.write:
        paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','--',
                                      'projects/robots/trossen/aloha'],cwd=ROOT,text=True).splitlines()
        paths+=['src/omnisim/physics/omnisim_newton_runtime.py','tests/test_newton_multiccd.py',
                'docs/reference/environment-variables.md']
        files=sorted(p for p in ARCHIVE.rglob('*') if p.is_file() and p!=MANIFEST and '__pycache__' not in p.parts)
        manifest={'created':'2026-09-20','updated':date.today().isoformat(),'hash_algorithm':'SHA-256',
                  'placement_checkpoint':'4d8ffea56','archive_files':[entry(p) for p in files],
                  'canonical_implementation_files':[entry(ROOT/p,True) for p in sorted(set(paths))]}
        MANIFEST.write_text(json.dumps(manifest,indent=2))
    manifest=json.loads(MANIFEST.read_text());rows=manifest['archive_files']+manifest['canonical_implementation_files']
    for row in rows:
        data=(ROOT/row['path']).read_bytes()
        if row.get('normalization')=='CRLF to LF':data=data.replace(b'\r\n',b'\n')
        if hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('Hash mismatch: '+row['path'])
    print(f'Verified {len(rows)} archive and implementation hashes')


if __name__=='__main__':main()
