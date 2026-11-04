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

"""The CNC OmniLink bridge against the virtual GRBL: tools, machine checks, gate, parser path.

Engine-free and model-free: the virtual GRBL ticks on a thread (ideal plant), the bridge
talks to it over TCP exactly as it would to a board, and the model is never called.
"""
import json
from pathlib import Path
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

SERVICE = Path(__file__).resolve().parents[1] / "projects/manufacturing/cnc"
MACHINES = SERVICE.parent / "machines"
sys.path.insert(0, str(SERVICE))

import cnc_bridge  # noqa: E402
import grbl  # noqa: E402
import machine  # noqa: E402
import server  # noqa: E402
import tool_table as tools  # noqa: E402

LONGMILL = MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl"
TOOLS = MACHINES / "longmill_mk2_30x30/longmill_mk2_tools.json"


class Machine:
    """A virtual GRBL (ideal plant, power-up mid-rail) ticking in real time, plus a bridge."""

    def __init__(self, stock_thickness=12.0):
        self.ctl = grbl.GrblController(machine.load(LONGMILL))
        self.ctl.tools = tools.load(TOOLS)
        self.ctl.tool = self.ctl.tools[1]
        self.srv = server.StreamServer(self.ctl, port=0)
        gport = self.srv.start()
        self.stop_clock = threading.Event()
        threading.Thread(target=self._clock, daemon=True).start()
        self.bridge = cnc_bridge.start(f"127.0.0.1:{gport}", str(LONGMILL), port=0 or self._free_port(),
                                       stock_thickness_mm=stock_thickness, zero_above_stock_mm=30.0,
                                       tools_path=str(TOOLS))
        self.url = f"http://127.0.0.1:{self.port}"

    def _free_port(self):
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        self.port = s.getsockname()[1]
        s.close()
        return self.port

    def _clock(self):
        last = time.monotonic()
        while not self.stop_clock.is_set():
            time.sleep(0.002)
            now = time.monotonic()
            # Run the machine 4x faster than the wall clock so jobs finish quickly.
            self.ctl.tick(4 * (now - last))
            last = now

    def post(self, path, body=None):
        req = urllib.request.Request(self.url + path, data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def tool(self, name, utterance=None, **args):
        body = {"tool": name, **args}       # /tool takes the arguments at the top level
        if utterance is not None:
            body["utterance"] = utterance
        return self.post("/tool", body)

    def close(self):
        self.bridge.link.close()
        self.stop_clock.set()
        self.srv.stop()


@pytest.fixture(scope="module")
def cnc():
    m = Machine()
    yield m
    m.close()


def result(resp):
    code, env = resp
    return code, (env.get("result") if isinstance(env, dict) else env) or env


def test_state_and_work_origin_on_the_stock_top(cnc):
    code, st = result(cnc.tool("get_robot_state"))
    assert code == 200 and st["mode"] == "Idle"
    assert st["work_pos_mm"] == [0.0, 0.0, 30.0]          # zeroed over the stock, 30 mm above its top


def test_jog_moves_and_reports_where_it_arrived(cnc):
    code, r = result(cnc.tool("jog", x_mm=10.0, feed_mm_min=3000))
    assert code == 200 and r["accepted"] and r["settled"]
    assert r["work_pos_mm"][0] == pytest.approx(10.0)
    cnc.tool("jog", x_mm=-10.0, feed_mm_min=3000)


def test_machine_checks_refuse_with_a_named_rule(cnc):
    _, r = result(cnc.tool("jog", x_mm=500.0))
    assert r["accepted"] is False and r["rule"] == "jog_limit"
    _, r = result(cnc.tool("move_to", x_mm=-600.0))
    assert r["accepted"] is False and r["rule"] == "travel"
    _, r = result(cnc.tool("move_to", z_mm=-1.0))
    assert r["accepted"] is False and r["rule"] == "plunge"
    _, r = result(cnc.tool("cut_pocket", shape="circle", width_mm=40, depth_mm=20))
    assert r["accepted"] is False and r["rule"] == "cut_depth"     # 12 mm stock: 11.5 mm at most
    _, r = result(cnc.tool("run_gcode", program="G21 G90\nG1 X10 Z-1 F500\n"))
    assert r["accepted"] is False and r["rule"] == "spindle_off"
    _, r = result(cnc.tool("run_gcode", program="G38.2 Z-5 F10\n"))
    assert r["accepted"] is False and r["rule"] == "preflight"


def test_the_gate_refuses_a_negated_order_before_the_machine_sees_it(cnc):
    code, env = cnc.tool("cut_pocket", utterance="don't cut a 40 mm circle 2 mm deep",
                         shape="circle", width_mm=40, depth_mm=2)
    assert code == 400 and env["error"] == "refused_by_gate"
    code, env = cnc.post("/tool", {"tool": "explode", "args": {}})
    assert code == 503 and env["error"] == "tool_not_registered"


def test_prompt_needs_an_omnikey(cnc):
    saved, cnc.bridge.relay = cnc.bridge.relay, None
    try:
        code, env = cnc.post("/prompt", {"text": "cut a circle"})
        assert code == 401
    finally:
        cnc.bridge.relay = saved


def test_a_pocket_job_runs_pauses_resumes_and_finishes(cnc):
    class FakeModel:
        """Commands (pause / resume / stop) are the parser's; only questions may reach the model."""
        asked = []

        def dispatch_sync(self, text, timeout_s=90.0):
            assert "?" in text, f"a command reached the model: {text!r}"
            self.asked.append(text)
            return {"via": "relay", "response": cnc.bridge.describe_state(), "actions": []}
    cnc.bridge.relay = FakeModel()
    code, r = result(cnc.tool("cut_pocket", shape="circle", width_mm=30, depth_mm=1.0, feed_mm_min=1500))
    assert code == 200 and r["accepted"] and r["running"] and r["estimated_s"] > 0
    time.sleep(1.0)
    code, env = cnc.post("/prompt", {"text": "pause"})
    assert code == 200 and env.get("via") == "parser"
    assert "standing still" in env["response"], env["response"]     # measured rest, not "unconfirmed"
    time.sleep(1.0)
    held = cnc.bridge.link.status()["state"]
    assert held.startswith("Hold"), held
    assert cnc.bridge.link.job.state == "running"              # paused, not cancelled
    code, env = cnc.post("/prompt", {"text": "what's your status?"})
    assert code == 200 and "Hold" in env["response"] and env["via"] == "relay"
    cnc.post("/prompt", {"text": "resume"})
    cnc.bridge.link.wait_idle(240)
    time.sleep(0.5)
    st = cnc.bridge.get_state()
    assert st["last_job"]["state"] == "done", st["last_job"]
    twin = cnc.bridge.link.twin()
    assert abs(twin["belief_mpos"][0] - twin["measured_mpos"][0]) < 1e-6


def test_stop_cancels_the_job_and_keeps_the_position(cnc):
    cnc.tool("cut_pocket", shape="rectangle", width_mm=40, height_mm=20, depth_mm=1.0, feed_mm_min=1500)
    time.sleep(1.0)
    code, env = cnc.post("/prompt", {"text": "stop"})
    assert code == 200 and env.get("via") == "parser"
    st = cnc.bridge.link.status()
    assert st["state"] == "Idle" and st["spindle"] == 0
    assert cnc.bridge.link.job.state == "cancelled"
    assert not any("ALARM:3" in e.get("text", "") for e in cnc.bridge.link.events)   # reset at rest


def test_the_platform_profile_can_be_built_from_the_bridge(cnc):
    """profile_extras() merges `capabilities` as a dict; a list broke relay start-up (2026-11-01)."""
    from omnisim_bridges.bridge_base import profile_extras
    extras = profile_extras(cnc.bridge, http_port=8765, surface="cnc")
    assert "prompt" in extras["actions"] and "cut_pocket" in extras["actions"]
    assert extras["surface"] == "cnc"


def test_tool_change_loads_the_tool_and_its_length_offset(cnc):
    """GRBL 1.1 has no M6: the sender stops, the tool goes in, G43.1 sets its length."""
    _, r = result(cnc.tool("change_tool", number=9))
    assert r["accepted"] is False and r["rule"] == "unknown_tool"
    _, r = result(cnc.tool("run_gcode", program="T9 M6\n"))
    assert r["accepted"] is False and r["rule"] == "preflight"          # T9 is not in the table
    _, r = result(cnc.tool("change_tool", number=2))
    assert r["accepted"], r
    cnc.bridge.link.wait_idle(60)
    time.sleep(0.5)
    assert cnc.bridge.link.job.state == "done" and cnc.bridge.link.job.tool_changes == 1
    assert cnc.ctl.tool.number == 2 and cnc.ctl.interp.tlo == pytest.approx(4.0)
    assert cnc.ctl.spindle[0] == "off"
    st = cnc.bridge.get_state()
    assert st["tool"]["number"] == 2 and st["tool"]["shape"] == "ball"
    # A pocket needs a flat end mill; the ball nose in the spindle is refused, by name.
    _, r = result(cnc.tool("cut_pocket", shape="circle", width_mm=30, depth_mm=1.0))
    assert r["accepted"] is False and r["rule"] == "flat_tool_needed"
    _, r = result(cnc.tool("change_tool", number=1))
    assert r["accepted"], r
    cnc.bridge.link.wait_idle(60)
    time.sleep(0.5)
    assert cnc.ctl.tool.number == 1 and cnc.ctl.interp.tlo == pytest.approx(0.0)


def test_a_dense_program_streams_with_character_counting(cnc):
    """3D finishing is thousands of short moves: the sender keeps GRBL's buffer full."""
    lines = ["G21 G90", "M3 S12000", "G0 Z1"] + [f"G1 X{0.2 * k:.3f} Y{(k % 2) * 0.2:.3f} F3000" for k in range(400)]
    program = "\n".join(lines + ["G0 Z5", "M5", "G0 X0 Y0"]) + "\n"
    t0 = time.time()
    job = cnc.bridge.link.stream(program)
    while job.state == "running" and time.time() - t0 < 60:
        time.sleep(0.05)
    assert job.state == "done" and job.acked == len(program.splitlines())
    # 406 lines in far fewer round trips than lines: the buffer held several at once.
    assert time.time() - t0 < 30


def test_each_client_gets_the_replies_to_its_own_lines(cnc):
    """The bridge and a sender share the twin: neither may read the other's `ok`."""
    from grbl_link import GrblLink
    other = GrblLink("127.0.0.1", cnc.srv.port)
    try:
        before = len(cnc.bridge.link._pending)
        for _ in range(20):
            assert other.command("G4 P0") == "ok"
            assert other.query("$TWIN", "[TWIN:")              # its own report reaches it
        time.sleep(0.3)
        assert len(cnc.bridge.link._pending) == before         # none of those oks went to the bridge
    finally:
        other.close()
