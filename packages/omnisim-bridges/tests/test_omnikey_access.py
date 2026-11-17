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

"""Run the production entry-point functions without starting the simulator."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from omnisim_bridges import route as _route_mod
from omnisim_bridges.access import connection_error, chat_config, reject_window_prompt

ROOT = Path(__file__).resolve().parents[3]
CONTROLLERS = ROOT / 'projects/samples/demos/controllers'


def load_function(path, name, namespace):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), fn], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace[name]


@pytest.mark.parametrize('kind', ['mobile','arm','quadruped'])
@pytest.mark.parametrize('configured', [False, True])
def test_disconnected_http_prompt_refuses_before_any_robot_side_effect(monkeypatch, kind, configured):
    monkeypatch.setenv('OMNI_KEY', 'configured-but-unavailable' if configured else '')
    bridge = Mock()
    handler = SimpleNamespace(path='/prompt', _json=lambda code, body: (code, body))
    fn = load_function(CONTROLLERS/f'omnilink_{kind}_bridge/omnilink_{kind}_bridge.py', '_route_post',
                       {'relay': None, 'bridge': bridge, 'connection_error': connection_error})
    status, body = fn(handler, {'text':'drive forward 1 meter'})
    assert status == (503 if configured else 401)
    assert body['actions'] == [] and body['ok'] is False
    assert body['error'] == ('omnilink_unavailable' if configured else 'omnikey_required')
    assert bridge.mock_calls == []


@pytest.mark.parametrize('kind', ['mobile','arm','quadruped'])
def test_disconnected_window_prompt_is_refused(monkeypatch, kind):
    """No relay, no chat. The `router` these handlers used to take is gone
    (2026-09-22): there is no second tier to fall through to at all."""
    monkeypatch.delenv('OMNI_KEY', raising=False)
    bridge = Mock()
    fn = load_function(CONTROLLERS/f'omnilink_{kind}_bridge/omnilink_{kind}_bridge.py',
                       'handle_wwi' if kind=='quadruped' else 'handle_wwi_message',
                       {'reject_window_prompt': reject_window_prompt})
    fn(bridge, None, 'prompt:drive forward 1 meter')
    bridge.act_stop.assert_not_called()
    bridge.note_external_command.assert_not_called()
    assert bridge.queue_window.call_args_list[0].args[0].startswith('error:')


@pytest.mark.parametrize('kind', ['mobile','arm','quadruped'])
def test_stop_button_remains_available_without_omnikey(kind):
    bridge = Mock()
    bridge.act_stop.return_value = {}
    fn = load_function(CONTROLLERS/f'omnilink_{kind}_bridge/omnilink_{kind}_bridge.py',
                       'handle_wwi' if kind=='quadruped' else 'handle_wwi_message', {})
    fn(bridge, None, 'stop')
    bridge.act_stop.assert_called_once()


def test_drone_prompt_is_not_a_keyless_fallback(monkeypatch):
    monkeypatch.delenv('OMNI_KEY', raising=False)
    state = SimpleNamespace(omnilink_relay=None)
    handler = SimpleNamespace(path='/prompt', _read_json=lambda:{'text':'takeoff'}, _json=lambda code,body:(code,body))
    fn = load_function(CONTROLLERS/'mavic_omnilink_bridge/mavic_omnilink_bridge.py','_route_post',
        {'state':state,'connection_error':connection_error,'nonempty_string':lambda v,n:v,'require_field':lambda d,n:d[n]})
    status, body = fn(handler)
    assert status==401 and body['actions']==[]


@pytest.mark.parametrize('kind', ['mobile','arm','quadruped'])
def test_a_configured_prompt_the_parser_declines_still_reaches_the_relay(monkeypatch, kind):
    """WHAT THE PARSER DECLINES, THE MODEL GETS -- even wide open.

    This used to assert that a CONFIGURED bridge dispatched every window
    prompt to the relay, which was true only because the parser had no
    live call site at all. Since 2026-09-22 it is wired (see
    test_parser_first_wiring.py), so the invariant worth pinning here is
    the other half of the sentence every guide prints: a challenge to the
    robot's own account must reach a model or nothing -- never a regex
    with a motor behind it -- and `OMNISIM_BRIDGE_PARSER_FIRST=all` does
    not change that, because CONVERSATION is not in any acting set.
    """
    monkeypatch.setenv('OMNISIM_BRIDGE_PARSER_FIRST','all')
    bridge, relay = Mock(), Mock()
    path=CONTROLLERS/f'omnilink_{kind}_bridge/omnilink_{kind}_bridge.py'
    inner=load_function(path,'_parser_first_window',{
        'shared_parser_window': _route_mod.parser_first_window,
        '_shared_parser_window': _route_mod.parser_first_window,
        '_tx_begin': lambda *a, **k: None, '_tx_end': lambda *a, **k: None})
    ns={'reject_window_prompt': reject_window_prompt,'_tx_window_cb':lambda *a:a[-1],
        '_autoread_window_cb':lambda *a:a[-1], '_on_relay_event': lambda *a:None,
        '_parser_first_window': lambda *a, **k: inner(*a, spawn=lambda f: f())}
    fn=load_function(path,'handle_wwi' if kind=='quadruped' else 'handle_wwi_message',ns)
    fn(bridge, relay, 'prompt:you never stopped, I was watching - admit it')
    relay.dispatch_async.assert_called_once()


@pytest.mark.parametrize('kind', ['mobile', 'arm', 'quadruped'])
def test_parser_first_default_does_not_make_chat_work_without_a_key(monkeypatch, kind):
    """PARSER-FIRST IS NOT AN ACCESS DECISION.

    Since 2026-09-22 an unset OMNISIM_BRIDGE_PARSER_FIRST means the
    deterministic parser ANSWERS a confident command instead of paying for
    a model (route._parser_first_set). The parser needs no OmniKey and no
    network, so the obvious way to get that flip wrong is to let it answer
    a keyless `/prompt` -- which is exactly the keyless chat the 2026-09-22
    access policy forbids.

    The access check sits in FRONT of it: no relay, no parse. This pins
    that ordering under the new default, on a sentence the parser would
    certainly have handled, and proves the parser never even counted the
    turn.
    """
    from omnisim_bridges import route as _route
    monkeypatch.delenv('OMNISIM_BRIDGE_PARSER_FIRST', raising=False)
    monkeypatch.delenv('OMNI_KEY', raising=False)
    assert _route._parser_first_set() == _route._SHORT_CIRCUIT_DEFAULT, \
        'this test is meaningless unless the parser would have acted'
    turns_before = _route._STATS['turns']

    bridge = Mock()
    handler = SimpleNamespace(path='/prompt', _json=lambda code, body: (code, body))
    fn = load_function(CONTROLLERS / f'omnilink_{kind}_bridge/omnilink_{kind}_bridge.py',
                       '_route_post',
                       {'relay': None, 'bridge': bridge, 'connection_error': connection_error})
    status, body = fn(handler, {'text': 'drive forward 1 meter'})
    assert status == 401 and body['error'] == 'omnikey_required'
    assert body['actions'] == [] and body['ok'] is False
    assert bridge.mock_calls == [], 'nothing may actuate without a key'
    assert _route._STATS['turns'] == turns_before, \
        'the parser must not even see a keyless utterance'


def test_chat_ui_requires_explicit_connection():
    assert chat_config(None)['chat_enabled'] is False
    assert chat_config(object())['chat_enabled'] is True


@pytest.mark.parametrize('clsname', ['OmniLinkRelay','OllamaRelay'])
@pytest.mark.parametrize('key', ['', '   '])
def test_every_relay_requires_a_nonempty_key(clsname, key):
    from omnisim_bridges import relay
    with pytest.raises(ValueError, match='OmniKey'):
        getattr(relay,clsname)(omni_key=key,agent_name='test',main_task='test',tools=[])


@pytest.mark.parametrize('kind', ['mobile', 'arm'])
def test_hardware_examples_do_not_parse_prompts_without_relay(monkeypatch, kind):
    monkeypatch.delenv('OMNI_KEY', raising=False)
    bridge = Mock()
    fn = load_function(ROOT/f'agents/bridges/{kind}_bridge_stub.py', 'act_prompt', {})
    result = fn(bridge, 'drive forward 1 meter')
    assert result['error'] == 'omnikey_required'
    assert bridge.mock_calls == []


COURIER = ROOT/'projects/robots/omnisim/omnitug500/controllers/omnitug500_courier/omnitug500_courier.py'


def test_courier_window_no_fallback_and_stop_still_works(monkeypatch):
    monkeypatch.delenv('OMNI_KEY', raising=False)
    bridge = Mock()
    fn = load_function(COURIER, 'handle_wwi', {})
    fn(bridge, None, 'prompt:deliver package')
    bridge.act_stop.assert_not_called()
    assert bridge.queue_window.call_args_list[0].args[0].startswith('error:')
    fn(bridge, None, 'stop')
    bridge.act_stop.assert_called_once()


def test_courier_http_no_fallback(monkeypatch):
    monkeypatch.delenv('OMNI_KEY', raising=False)
    bridge = Mock()
    fn = load_function(COURIER, 'do_POST', {'bridge': bridge, 'relay': None})
    handler = SimpleNamespace(path='/prompt', _body=lambda: {'text':'deliver package'}, _json=lambda code, body: (code,body))
    status, body = fn(handler)
    assert status == 401 and body['actions'] == []
    assert bridge.mock_calls == []


def test_the_courier_keyword_router_is_gone():
    """A FIFTH ladder, and the most dangerous one to leave lying around.

    `courier_intent.CourierIntent` advertised itself as "the no-OMNI_KEY
    fallback ... so the demo works the moment you open the world". The
    runtime never called it -- `/prompt` answered 401, `/tool` 503, the
    window path refused -- but this controller does not import `gate` at
    all, so anyone re-wiring it would have built a path that was keyless
    AND ungated. Deleted 2026-09-22 with the other four.
    """
    assert not (COURIER.parent / 'courier_intent.py').exists()
    src = COURIER.read_text(encoding='utf-8')
    assert 'CourierIntent(' not in src
    assert 'from courier_intent import' not in src
