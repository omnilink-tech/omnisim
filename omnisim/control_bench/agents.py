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
"""Explicit model-loop implementations and deterministic-router ablations."""
from __future__ import annotations

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time
from typing import TypedDict

from omnisim.paths import REPO_ROOT
from .engine import InfrastructureError, failed
from .tasks import READS, TOOLS, action_valid, position

ARMS = ('omnilink_parser','plain','langgraph','lobster','langgraph_parser','lobster_parser')
OVERLOAD_MESSAGE = 'This model is currently experiencing high demand. Spikes in demand are usually temporary. Please try again later.'
ERROR_COST_BASIS = 'google_5xx_no_token_charge_assumption_v1'
BILLING_POLICY = 'https://ai.google.dev/gemini-api/docs/billing#am-i-charged-for-failed-requests'
SYSTEM = '''You control a robot using measured state and a bounded tool surface.
Return JSON only: {"actions":[{"tool":name,"args":{...}}],
"status":"complete"|"continue"|"clarify"|"failed", "message":string,
"reported_pose":null|array}. Mobile poses are [x,y,yaw] in WORLD metres/radians;
arm poses are six joint angles in radians. Report pose only from observed state.
Batch independent sequential actions in one response. Stop a batch after an
unsuccessful action, read its result, and replan; observations/results arrive
in the conversation. Never repeat an action that already succeeded. The tools
DO NOT retry: you own recovery. A temporary_unavailable with moved=false can
be retried once; a second failure must be reported and stopped. Never schedule
a second copy speculatively. Quoted/cancelled orders, hypothetical questions,
and unspecified distances/angles do not authorize motion: answer or clarify.
Use only the supplied tools and arguments. No shell, teleport, or reset of the
world. Arm reset_to_home is a real physical joint motion. Pick uses the shared
simulator's assisted grasp primitive; judge its measured result.
Finish within six planning rounds and 24 actions. Do not claim success from
accepted alone. A finished requested operation may return status complete in
the same batch; measured outcomes still decide success, not your narration.'''


def metadata(value):
    out={}
    if isinstance(value,dict):
        for k,v in value.items():
            if k in ('usageMetadata','modelVersion'): out[k]=v
            elif isinstance(v,(list,dict)): out.update(metadata(v))
    elif isinstance(value,list):
        for v in value: out.update(metadata(v))
    return out


def price(usage, rates):
    if not isinstance(usage,dict) or not all(k in usage for k in ('promptTokenCount','totalTokenCount','candidatesTokenCount')):
        return None
    p=usage['promptTokenCount']; c=usage.get('cachedContentTokenCount',0)
    o=usage['candidatesTokenCount']+usage.get('thoughtsTokenCount',0)
    if any(not isinstance(x,int) or isinstance(x,bool) or x<0 for x in (p,c,o)) or c>p: return None
    return ((p-c)*rates['input']+c*rates['cached']+o*rates['output'])/1e6


def is_unbilled_overload(record):
    """Narrow published-policy estimate, not measured usage or an invoice."""
    return (record.get('http')==503 and record.get('usage') is None
            and record.get('model') is None and not record.get('text')
            and record.get('cost_basis')==ERROR_COST_BASIS
            and record.get('billing_policy')==BILLING_POLICY
            and record.get('provider_error')=={'code':503,'status':'UNAVAILABLE','message':OVERLOAD_MESSAGE})


def request_price(record, rates):
    if is_unbilled_overload(record): return 0.0
    return price(record.get('usage'),rates)


class ProviderOverload(InfrastructureError):
    pass


class Model:
    def __init__(self,key,model,profile,cap,rates):
        import requests
        self.key,self.model,self.profile,self.cap,self.rates=key,model,profile,cap,rates
        self.records=[]; self.spent=0.; self.blocked=False
        self.session=requests.Session()

    def compile(self,messages,surface):
        # Retry only an explicitly identified provider rejection, once. The
        # failed request remains in records and wall time includes the wait.
        for attempt in range(2):
            try: return self._compile_once(messages,surface)
            except ProviderOverload:
                if attempt: raise
                raw=self.records[-1].get('service_error',{}).get('retry_after')
                try: delay=max(60,float(raw))
                except (TypeError,ValueError): delay=60
                if not 60<=delay<=120: raise
                self.records[-1]['retry_wait_s']=delay
                time.sleep(delay)

    def _compile_once(self,messages,surface):
        if self.blocked or self.spent+.10>self.cap:
            raise InfrastructureError('Model spend cap reached or prior usage unknown')
        body={'agentName':self.profile,'engine':'g1-engine','model':self.model,
              'temperature':0,'max_tokens':2048,'noFallback':True,'skipMemory':True,
              'usePromptPipeline':True,
              'systemInstructionRequest':{'mainTask':SYSTEM+'\nTools: '+json.dumps(TOOLS[surface]),
                  'availableTools':[],'availableToolDetails':[],'allowToolUse':False},
              'messages':messages}
        record={'request':body,
                'request_sha256':hashlib.sha256(json.dumps(body,sort_keys=True).encode()).hexdigest(),
                'derived_usd':None,'model':None,'usage':None}
        self.records.append(record)
        start=time.monotonic()
        try:
            r=self.session.post('https://www.omnilink-agents.com/api/chat',
                 headers={'Authorization':'Bearer '+self.key},json=body,timeout=75)
            record['http']=r.status_code
            data=r.json()
            record['response_sha256']=hashlib.sha256(r.content).hexdigest()
            if not r.ok:
                # Retain structured failure diagnostics without copying arbitrary
                # upstream messages (which can contain provider/account details).
                error=data.get('error')
                upstream=data.get('upstream') or {}
                record['service_error']={'code':error.get('code') if isinstance(error,dict) else data.get('code'),
                    'upstream_status':upstream.get('status') if isinstance(upstream,dict) else None,
                    'retry_after':r.headers.get('Retry-After')}
            record.update(metadata(data.get('raw')))
            record['model']=record.pop('modelVersion',None)
            record['usage']=record.pop('usageMetadata',None)
            record['derived_usd']=price(record['usage'],self.rates)
            record['request_id']=data.get('requestId')
            record['text']=data.get('text','')
            if r.status_code==503 and record['usage'] is None and record['model'] is None and not record['text']:
                error=data.get('error')
                if isinstance(error,str):
                    try: error=json.loads(error).get('error')
                    except (ValueError,AttributeError): error=None
                if error=={'code':503,'status':'UNAVAILABLE','message':OVERLOAD_MESSAGE}:
                    record.update(provider_error=error,cost_basis=ERROR_COST_BASIS,billing_policy=BILLING_POLICY)
                    record['derived_usd']=request_price(record,self.rates)
                    raise ProviderOverload('Google provider overload; bounded retry exhausted or unavailable')
            if record['derived_usd'] is None:
                self.blocked=True
                raise InfrastructureError('Provider usage unknown; stopped spending')
            self.spent+=record['derived_usd']
            if not r.ok: raise InfrastructureError(f'Model HTTP {r.status_code}')
            if record['model']!=self.model:
                self.blocked=True; raise InfrastructureError('Returned model differs from frozen model')
            text=record['text'].strip()
            if text.startswith('```'):
                text=text.split('\n',1)[-1].rsplit('```',1)[0]
            return validate(json.loads(text),surface)
        except InfrastructureError: raise
        except (ValueError,KeyError,TypeError):
            if record['derived_usd'] is None: self.blocked=True
            raise
        except Exception as exc:
            self.blocked=True
            record['error_type']=type(exc).__name__
            raise InfrastructureError('Model transport failed; stopped spending') from exc
        finally:
            record['elapsed_s']=time.monotonic()-start


def validate(plan,surface):
    if not isinstance(plan,dict) or not isinstance(plan.get('actions'),list) or len(plan['actions'])>16:
        raise ValueError('Malformed action batch')
    if plan.get('status') not in ('complete','continue','clarify','failed') or not isinstance(plan.get('message',''),str):
        raise ValueError('Explicit completion status required')
    for action in plan['actions']: action_valid(action,surface)
    return plan


def parse(prompt, state, surface):
    sys.path.insert(0,str(REPO_ROOT/'packages/omnisim-bridges/src'))
    from omnisim_bridges import route, interpret, gate
    result=route.parser_first_plan(prompt,surface)
    if result is None: return None
    if result.intent==getattr(interpret,'CONDITIONAL','conditional'):
        result=route.resolve_conditional(result,position(state,surface)) if surface=='mobile' else None
        if result is None: return None
    actions=[]
    for frame in result.frames:
        name,args=gate.normalise(frame.tool,frame.args)
        if name=='stop': name='stop_robot'
        action={'tool':name,'args':dict(args)}
        try: action_valid(action,surface)
        except ValueError: return None
        actions.append(action)
    ask=getattr(result,'ask','') or ''
    return {'actions':actions,'status':'clarify' if ask else 'complete','message':ask,'reported_pose':None}


class State(TypedDict):
    messages: list
    round: int
    plan: dict
    done: bool
    budget_exhausted: bool


class Runtime:
    def __init__(self, model, arm):
        if arm not in ARMS: raise ValueError('Unknown implementation')
        self.model,self.arm=model,arm
        self.use_parser=arm.endswith('_parser')
        self.kind='plain' if arm=='omnilink_parser' else arm.removesuffix('_parser')
        self.obs=None; self.parser_hits=0
        self.graph=None; self.worker=None; self.server=None
        if self.kind=='langgraph':
            from langgraph.graph import StateGraph,START,END
            graph=StateGraph(State)
            graph.add_node('plan',self.plan); graph.add_node('execute',self.execute)
            graph.add_edge(START,'plan'); graph.add_edge('plan','execute')
            graph.add_conditional_edges('execute',lambda s:'end' if s['done'] else 'plan',{'end':END,'plan':'plan'})
            self.graph=graph.compile()
        if self.kind=='lobster': self._start_lobster()

    def plan(self,state):
        plan=None
        if state['round']==0 and self.use_parser:
            plan=parse(self.obs.prompt,self.obs.initial,self.obs.engine.surface)
            self.parser_hits+=plan is not None
        if plan is None:
            try:
                plan=self.model.compile(state['messages'],self.obs.engine.surface)
            except ValueError as exc:
                # Every implementation gets the same bounded protocol repair.
                # This consumes a planning round and all request cost remains.
                records=getattr(self.model,'records',[])
                plan={'actions':[],'status':'continue','message':'',
                      '_protocol_error':type(exc).__name__+': '+str(exc),
                      '_invalid_text':records[-1].get('text','') if records else ''}
        return {**state,'round':state['round']+1,'plan':plan}

    def execute(self,state):
        plan=state['plan']; feedback=[]; unsuccessful=False
        for action in plan['actions']:
            result=self.obs.dispatch(action)
            feedback.append({'action':action,'result':result})
            if failed(result):
                unsuccessful=True
                # The common actuator safeguard stops partial motion; it does
                # not decide whether/how to retry on the agent's behalf.
                feedback.append({'action':{'tool':'stop_robot','args':{}},
                                 'result':self.obs.dispatch({'tool':'stop_robot','args':{}})})
                break
        complete=not unsuccessful and plan['status']!='continue'
        done=complete or state['round']>=6
        feedback_message={'instruction':self.obs.prompt,'tool_results':feedback,
             'measured_state':self.obs.engine.state(objects=True),
             'response_contract':'Return the required JSON control object, including actions, status, message and reported_pose. Even a final answer must be JSON. Do not repeat successful actions.'}
        if '_protocol_error' in plan: feedback_message['protocol_error']=plan['_protocol_error']
        messages=[*state['messages'],{'role':'assistant','content':plan.get('_invalid_text',json.dumps(plan))},
                  {'role':'user','content':json.dumps(feedback_message)}]
        return {**state,'messages':messages,'done':done,'budget_exhausted':not complete and state['round']>=6}

    def run(self,obs,history):
        self.obs=obs
        state={'messages':[*history,{'role':'user','content':json.dumps({'instruction':obs.prompt,'measured_state':obs.initial})}],
               'round':0,'plan':{},'done':False,'budget_exhausted':False}
        if self.kind=='langgraph': state=self.graph.invoke(state,{'recursion_limit':20})
        elif self.kind=='lobster':
            self.callback_error=None
            self.worker.stdin.write(json.dumps({'state':state,'url':self.callback_url,'token':self.secret})+'\n'); self.worker.stdin.flush()
            line=self.worker.stdout.readline()
            if not line: raise InfrastructureError('Lobster worker exited')
            reply=json.loads(line)
            if not reply.get('ok'):
                if self.callback_error is not None: raise self.callback_error
                raise InfrastructureError('Lobster callback failed: '+str(reply.get('error')))
            state=reply['state']
        else:
            while not state['done']: state=self.execute(self.plan(state))
        return state['plan'],state['messages'],state['budget_exhausted']

    def _start_lobster(self):
        owner=self; self.secret=secrets.token_hex(24)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args): pass
            def do_POST(self):
                if self.headers.get('Authorization')!=owner.secret: self.send_error(403); return
                try:
                    size=int(self.headers.get('Content-Length',0))
                    if size>2_000_000: raise ValueError('Oversized state')
                    state=json.loads(self.rfile.read(size))
                    out={'/plan':owner.plan,'/execute':owner.execute}[self.path](state)
                    status=200
                except Exception as exc:
                    owner.callback_error=exc
                    out={'error':type(exc).__name__+': '+str(exc)}; status=500
                raw=json.dumps(out).encode()
                self.send_response(status); self.send_header('Content-Length',str(len(raw))); self.end_headers(); self.wfile.write(raw)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.callback_url=f'http://127.0.0.1:{self.server.server_port}'
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        # Same pinned dependency as the earlier campaign; caller can install
        # it elsewhere and pass its module location through the lock config.
        vendor=REPO_ROOT/'tests/benchmarks/harness_comparison/vendor/lobster/dist/src/sdk/index.js'
        self.worker=subprocess.Popen(['node',str(Path(__file__).with_name('lobster.mjs')),str(vendor)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8',
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))

    def close(self):
        if self.worker:
            self.worker.stdin.close()
            try: self.worker.wait(timeout=3)
            except subprocess.TimeoutExpired: self.worker.terminate(); self.worker.wait(timeout=3)
        if self.server: self.server.shutdown(); self.server.server_close()
