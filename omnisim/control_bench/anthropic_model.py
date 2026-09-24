# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Direct, deliberately bounded Anthropic transport for exploratory model pilots.

This is not the deployed OmniLink chat connector. Robot bridges still require
their separate OmniKey. Never log authentication headers or credential paths.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import time

from .agents import SYSTEM, validate
from .engine import InfrastructureError
from .tasks import TOOLS

MODEL = 'claude-opus-5-5'
CONFIG = {'provider': 'anthropic-direct', 'effort': 'low', 'max_tokens': 2048,
          'max_requests': 24, 'service_tier': 'standard_only', 'retries': 0,
          'input_byte_allowance': 4096, 'input_count_multiplier': 2,
          'rates': {'input': 4., 'output': 20., 'cached': .20, 'cache_write': 5.}}
COMPARISON_CONFIG = {**CONFIG, 'max_requests': 450, 'comparison': True}
CACHED_CONFIG = {**COMPARISON_CONFIG, 'cache_policy': 'system-and-history-5m',
                 'warmup': 'one-common-prefix-per-surface-zero-output'}


def price(usage, rates, allow_cache_writes=False):
    if not isinstance(usage, dict): return None
    values = [usage.get('input_tokens'), usage.get('output_tokens'),
              usage.get('cache_read_input_tokens', 0), usage.get('cache_creation_input_tokens', 0)]
    if any(not isinstance(n, int) or isinstance(n, bool) or n < 0 for n in values): return None
    if values[3] and not allow_cache_writes: return None
    creation=usage.get('cache_creation', {})
    if not isinstance(creation,dict): return None
    five=creation.get('ephemeral_5m_input_tokens',0)
    hour=creation.get('ephemeral_1h_input_tokens',0)
    if any(not isinstance(n,int) or isinstance(n,bool) or n<0 for n in (five,hour)):
        return None
    if hour or five!=values[3]: return None
    return (values[0]*rates['input'] + values[1]*rates['output'] + values[2]*rates['cached']
            + values[3]*rates['cache_write'])/1e6


def reservation(body, counted, rates):
    if not isinstance(counted, int) or isinstance(counted, bool) or counted < 0:
        raise InfrastructureError('Invalid provider token count')
    # Count API is an estimate. Reserve the larger of twice that estimate and
    # one token per serialized UTF-8 byte, plus 4096 tokens for hidden overhead.
    bound = max(len(json.dumps(body, ensure_ascii=False).encode('utf-8')), 2*counted) + 4096
    cached=bool(body.get('cache_control')) or isinstance(body.get('system'),list)
    input_rate=max(rates['input'],rates['cache_write']) if cached else rates['input']
    return bound, (bound*input_rate + body['max_tokens']*rates['output'])/1e6


class AnthropicModel:
    def __init__(self, key, model, cap, rates, journal, config=None):
        import requests
        self.config = dict(CONFIG if config is None else config)
        if self.config not in (CONFIG, COMPARISON_CONFIG, CACHED_CONFIG):
            raise ValueError('Unsupported Anthropic configuration')
        limit = 5 if self.config.get('comparison') else 1
        if model != MODEL or rates != CONFIG['rates'] or not 0 < cap <= limit:
            raise ValueError(f'Only frozen Opus 5.5 rates and a budget <= ${limit} are supported')
        if not key.startswith('sk-ant-') or any(c.isspace() for c in key):
            raise ValueError('Invalid Anthropic key format')
        self.key, self.model, self.cap, self.rates = key, model, cap, rates
        self.records = []; self.spent = 0.; self.blocked = False
        self.session = requests.Session()  # requests defaults to zero HTTP retries
        self.journal = Path(journal)
        self.journal.touch(exist_ok=False)

    def save(self, event):
        with self.journal.open('a', encoding='utf-8') as f:
            f.write(json.dumps(event, ensure_ascii=False)+'\n')
            f.flush(); os.fsync(f.fileno())

    def body(self, messages, surface):
        body = {'model': self.model, 'max_tokens': CONFIG['max_tokens'],
                'output_config': {'effort': 'low'}, 'service_tier': 'standard_only',
                'system': SYSTEM+'\nTools: '+json.dumps(TOOLS[surface]), 'messages': messages}
        if self.config.get('cache_policy'):
            control={'type':'ephemeral','ttl':'5m'}
            body['system']=[{'type':'text','text':body['system'],'cache_control':dict(control)}]
            body['cache_control']=dict(control)
        return body

    def compile(self, messages, surface):
        record=self.generate(self.body(messages,surface))
        if record['stop_reason'] != 'end_turn':
            raise ValueError('Anthropic response did not finish; '+str(record['stop_reason']))
        text=record['text'].strip()
        if text.startswith('```'): text=text.split('\n',1)[-1].rsplit('```',1)[0]
        return validate(json.loads(text),surface)

    def warmup(self, surfaces):
        if self.config!=CACHED_CONFIG: raise ValueError('Warm-up is only for cached comparisons')
        for surface in sorted(set(surfaces)):
            body=self.body([{'role':'user','content':'Initialize the shared benchmark prompt cache only.'}],surface)
            body.pop('cache_control')  # cache only the shared system prefix here
            body['max_tokens']=0
            record=self.generate(body,purpose='common_cache_warmup:'+surface)
            if record['usage']['output_tokens']!=0 or record['stop_reason']!='max_tokens':
                self.blocked=True
                raise InfrastructureError('Unexpected cache warm-up output')
            if not (record['usage'].get('cache_read_input_tokens',0)+record['usage'].get('cache_creation_input_tokens',0)):
                self.blocked=True
                raise InfrastructureError('Provider did not cache the common prefix; stopped before scored requests')

    def generate(self, body, purpose='scored'):
        if self.blocked or len(self.records) >= self.config['max_requests']:
            self.blocked = True
            raise InfrastructureError('Anthropic run halted; no more requests')
        record = {'request': body, 'request_sha256': hashlib.sha256(
            json.dumps(body, sort_keys=True).encode()).hexdigest(),
            'usage': None, 'model': None, 'derived_usd': None, 'text': '', 'purpose':purpose}
        headers = {'x-api-key': self.key, 'anthropic-version': '2023-06-01'}
        started = time.monotonic()
        sent = False
        try:
            count_body = {k: body[k] for k in ('model', 'system', 'messages')}
            r = self.session.post('https://api.anthropic.com/v1/messages/count_tokens',
                headers=headers, json=count_body, timeout=30, allow_redirects=False)
            if not r.ok: raise InfrastructureError(f'Anthropic count HTTP {r.status_code}; no generation sent')
            counted = r.json().get('input_tokens')
            bound, reserve = reservation(body, counted, self.rates)
            if self.spent + reserve > self.cap:
                raise InfrastructureError('Insufficient pilot budget for conservative request reservation')
            record.update(counted_input_tokens=counted, reserved_input_tokens=bound, reserved_usd=reserve)
            self.records.append(record)
            self.save({'event': 'reserved', 'index': len(self.records)-1, 'record': record})
            sent = True
            r = self.session.post('https://api.anthropic.com/v1/messages', headers=headers,
                json=body, timeout=75, allow_redirects=False)
            record.update(http=r.status_code, response_sha256=hashlib.sha256(r.content).hexdigest())
            data = r.json()
            if not r.ok:
                record['error_type'] = data.get('error', {}).get('type')
                raise InfrastructureError(f'Anthropic HTTP {r.status_code}; no retry')
            record.update(model=data.get('model'), usage=data.get('usage'),
                          stop_reason=data.get('stop_reason'), response=data)
            record['derived_usd'] = price(record['usage'], self.rates,allow_cache_writes=bool(self.config.get('cache_policy')))
            if record['derived_usd'] is None:
                raise InfrastructureError('Unknown Anthropic usage; stopped spending')
            self.spent += record['derived_usd']
            if record['derived_usd'] > reserve or record['usage']['output_tokens'] > body['max_tokens']:
                raise InfrastructureError('Provider usage exceeded reservation or output limit')
            if record['model'] != self.model:
                raise InfrastructureError('Returned Anthropic model differs from frozen model')
            if record['usage'].get('service_tier') not in (None, 'standard'):
                raise InfrastructureError('Unexpected Anthropic service tier')
            record['text'] = ''.join(b.get('text', '') for b in data.get('content', []) if b.get('type') == 'text')
            return record
        except InfrastructureError:
            self.blocked = True
            raise
        except (ValueError, KeyError, TypeError):
            if record['derived_usd'] is None: self.blocked = True
            raise
        except Exception as exc:
            self.blocked = True
            record['error_type'] = type(exc).__name__
            raise InfrastructureError('Anthropic transport failed; stopped spending without retry') from None
        finally:
            record['elapsed_s'] = time.monotonic()-started
            self.save({'event': 'finished' if sent else 'not_sent',
                       'index': len(self.records)-1 if sent else None, 'record': record})
