"""Read the measured profile's prompt-affecting fields, without account IDs."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import requests

p=argparse.ArgumentParser()
p.add_argument('--key-file', type=Path, required=True)
p.add_argument('--profile', required=True)
p.add_argument('--out', type=Path, required=True)
a=p.parse_args()
if a.out.exists(): raise SystemExit('Output already exists')
r=requests.get('https://www.omnilink-agents.com/api/agent-profiles',
               headers={'Authorization':'Bearer '+a.key_file.read_text().strip()},timeout=30)
r.raise_for_status()
matches=[v for v in r.json()['profiles'] if v['name']==a.profile]
if len(matches)!=1: raise SystemExit('Expected exactly one matching profile')
profile=matches[0]
settings=profile.get('settings') or {}
keys=('availableTools','availableToolDetails','userName','agentPersona')
out={'observed_utc':datetime.now(timezone.utc).isoformat(),'profile':a.profile,
     'updated_at':profile.get('updated_at'),
     'inherited_prompt_settings':{k:settings[k] for k in keys if k in settings},
     'scope':'Read-only observation during or after the campaign, not a pre-run freeze or server-revision attestation. Account IDs and unrelated profiles are excluded.'}
a.out.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'saved':str(a.out),'inherited_fields':list(out['inherited_prompt_settings'])}))
