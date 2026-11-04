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

"""Refusals must remain paired with their calls on the NEXT operator turn.

The current round's wire messages used to be complete while persistent history
lost gate-refused / superseded results. Mixed bridge/gate refusals then caused
Google to reject every later turn with a function-call/response-count HTTP 400.
"""
import copy

import pytest

from omnisim_bridges.relay import DispatchHandle, OmniLinkRelay
from omnisim_bridges.tool import Tool


def assert_complete_exchanges(messages):
    """A provider-facing protocol oracle, independent of relay internals."""
    pending=[]
    for message in messages:
        if message['role']=='tool':
            assert pending, 'orphan tool response'
            assert message['tool_call_id']==pending.pop(0), 'missing or reordered tool response'
        else:
            assert not pending, 'assistant tool request has unanswered calls'
            pending=[call['id'] for call in message.get('tool_calls',[])]
    assert not pending, 'incomplete tool exchange at end of payload'


@pytest.mark.parametrize('case',['gate_only','mixed_bridge_gate','superseded'])
def test_followup_after_refused_tools_has_complete_history_and_can_act(
        monkeypatch,tmp_path,case):
    monkeypatch.setenv('OMNILINK_INTENT_STATE_DIR',str(tmp_path))
    monkeypatch.setenv('OMNILINK_PRESENCE','0')
    monkeypatch.setenv('OMNILINK_EDGE','0')
    monkeypatch.setenv('OMNILINK_PROFILE_SYNC','0')
    dispatched=[]
    relay=OmniLinkRelay(
        omni_key='olink_offline_test_not_a_credential',agent_name='HistoryProbe',
        main_task='test',usage_enabled=False,memory_enabled=False,surface='mobile',
        tools=[
            Tool('drive_forward','drive',{'type':'object','properties':{'distance':{'type':'number'}}},
                 lambda args: dispatched.append(args) or {'accepted':True}),
            Tool('get_robot_state','read',{'type':'object','properties':{}},
                 lambda args:{'accepted':False,'reason':'test bridge refusal'},physical=False),
        ])
    def call(name,id,args):return {'id':id,'name':name,'arguments':args}
    initial=[call('drive_forward','refused-1',{'distance':1})]
    if case=='gate_only':initial.append(call('drive_forward','refused-2',{'distance':1}))
    if case=='mixed_bridge_gate':initial.insert(0,call('get_robot_state','bridge-refused',{}))
    script=[{'text':'proposed','toolCalls':initial},{'text':'No movement.'},
            {'text':'proposed','toolCalls':[call('drive_forward','useful',{'distance':1})]},
            {'text':'Moved.'}]
    requests=[]
    def model(messages):
        assert_complete_exchanges(messages)
        requests.append(copy.deepcopy(messages))
        return script.pop(0)
    relay._post_chat=model
    events=[]
    try:
        handle=DispatchHandle()
        if case=='superseded':handle.motion_superseded.set()
        utterance='Drive forward 1 metre.' if case=='superseded' else 'The manual says "drive forward 1 metre".'
        relay._dispatch_one(utterance,lambda kind,data:events.append((kind,data)),handle)
        relay._release_history(handle)
        assert dispatched==[], 'refused/superseded proposals must not actuate'
        followup=DispatchHandle()
        relay._dispatch_one('Drive forward 1 metre.',lambda kind,data:events.append((kind,data)),followup)
        relay._release_history(followup)
        assert dispatched==[{'distance':1}], 'a protected turn must not poison subsequent useful work'
        assert len(requests)==4 and not script
        assert_complete_exchanges(relay.history)
        # This is the formerly broken NEXT-turn payload, not only the local
        # retry round where responses were already complete before the fix.
        answered=[m['tool_call_id'] for m in requests[2] if m['role']=='tool']
        assert answered==[c['id'] for c in initial]
    finally:
        relay.close()
