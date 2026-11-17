"""Unscored availability probes; never dispatch robot actions."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from omnisim.control_bench.agents import metadata, price
import requests

p = argparse.ArgumentParser()
p.add_argument('--key-file', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--model', required=True)
p.add_argument('--input-rate', type=float, required=True)
p.add_argument('--cached-rate', type=float, required=True)
p.add_argument('--output-rate', type=float, required=True)
a = p.parse_args()
if a.out.exists(): raise SystemExit('Output already exists')
body = {'agentName': 'HuskySwarm', 'engine': 'g1-engine', 'model': a.model,
        'temperature': 0, 'max_tokens': 128, 'noFallback': True, 'skipMemory': True,
        'usePromptPipeline': True,
        'systemInstructionRequest': {'mainTask': 'This is an unscored service availability check. Reply with PONG only. Do not call tools.',
                                     'availableTools': [], 'availableToolDetails': [], 'allowToolUse': False},
        'messages': [{'role': 'user', 'content': 'Reply PONG.'}]}
rates = {'input': a.input_rate, 'cached': a.cached_rate, 'output': a.output_rate}
report = {'purpose': 'Five consecutive unscored availability requests; selected model before scored outcomes; no robot actions',
          'model': a.model, 'request': body, 'rates': rates, 'rows': []}
key = a.key_file.read_text().strip()
session = requests.Session()
for i in range(5):
    start = time.monotonic()
    r = session.post('https://www.omnilink-agents.com/api/chat',
                     headers={'Authorization': 'Bearer ' + key}, json=body, timeout=75)
    data = r.json()
    meta = metadata(data.get('raw'))
    row = {'probe': i, 'utc': datetime.now(timezone.utc).isoformat(), 'http': r.status_code,
           'request_id': data.get('requestId'), 'retry_after': r.headers.get('Retry-After'),
           'response_sha256': hashlib.sha256(r.content).hexdigest(),
           'error': data.get('error'), 'model': meta.get('modelVersion'),
           'usage': meta.get('usageMetadata'), 'text': data.get('text'),
           'derived_usd': price(meta.get('usageMetadata'), rates), 'elapsed_s': time.monotonic()-start}
    report['rows'].append(row)
    a.out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(row), flush=True)
    if not r.ok or row['model'] != a.model or row['derived_usd'] is None: break
