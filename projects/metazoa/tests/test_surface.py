"""mz.surface: routing, the sim-thread queue (409 / timeout / deferred
read-back) and the thin HTTP handler.  Engine-free."""
import http.client
import json
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from mz import surface as SF  # noqa: E402


# ---------------------------------------------------------------- routing

@pytest.fixture
def router():
    return SF.Router(n_patches=5, arena=18.0)


def test_capabilities_is_an_immediate_reply(router):
    r = router.route("GET", "/capabilities")
    assert r.kind == "reply" and r.code == 200
    assert r.body["ok"] and r.body["service"] == SF.SERVICE
    assert set(r.body["verbs"]) == set(SF.VERBS)
    assert r.body["config"]["n_patches"] == 5


def test_healthz_and_census(router):
    assert router.route("GET", "/healthz").body == {"ok": True}
    assert router.route("GET", "/census?since=3").kind == "census"
    assert router.route("POST", "/census", b"{}").kind == "census"


def test_unknown_verb_is_404(router):
    r = router.route("GET", "/nope")
    assert r.code == 404 and r.body["error"] == "unknown_verb"
    assert "/light" in r.body["valid"]


def test_wrong_method_is_405(router):
    r = router.route("GET", "/light")
    assert r.code == 405 and r.body["error"] == "method_not_allowed"
    assert r.body["allowed"] == ["POST"]
    assert router.route("POST", "/capabilities").code == 405


def test_bad_json_is_400(router):
    r = router.route("POST", "/light", b"{not json")
    assert r.code == 400 and r.body["error"] == "bad_json"
    r = router.route("POST", "/light", b"[1, 2]")            # not an object
    assert r.code == 400 and r.body["error"] == "bad_json"


def test_missing_field_is_400_and_names_it(router):
    r = router.route("POST", "/light", b'{"k": 1, "x": 0.5}')
    assert r.code == 400 and r.body["error"] == "missing_args"
    assert r.body["missing"] == ["y"] and r.body["need"] == ["k", "x", "y"]
    r = router.route("POST", "/recruit", b"")
    assert r.body["missing"] == ["organism", "cell"]


def test_bad_type_is_400(router):
    r = router.route("POST", "/light", {"k": "one", "x": 0, "y": 0})
    assert r.code == 400 and r.body["error"] == "bad_type" and r.body["field"] == "k"
    r = router.route("POST", "/light", {"k": 1.5, "x": 0, "y": 0})
    assert r.body["error"] == "bad_type"
    r = router.route("POST", "/dim", {"factor": True})
    assert r.body["error"] == "bad_type"
    r = router.route("POST", "/dim", {"factor": "nan"})
    assert r.body["error"] == "bad_type"


def test_light_valid_becomes_a_command_with_coerced_args(router):
    r = router.route("POST", "/light?x=1", b'{"k": 2, "x": 1, "y": -2.5, "extra": 9}')
    assert r.kind == "command" and r.verb == "light"
    assert r.args == {"k": 2, "x": 1.0, "y": -2.5}     # extra keys dropped, types coerced
    assert isinstance(r.args["x"], float)


def test_light_patch_index_out_of_range_is_400(router):
    r = router.route("POST", "/light", {"k": 5, "x": 0, "y": 0})
    assert r.code == 400 and r.body["error"] == "bad_value" and r.body["field"] == "k"
    assert router.route("POST", "/light", {"k": -1, "x": 0, "y": 0}).code == 400
    assert router.route("POST", "/light", {"k": 4, "x": 0, "y": 0}).kind == "command"


def test_light_absurd_position_is_400_but_out_of_arena_is_accepted(router):
    assert router.route("POST", "/light", {"k": 0, "x": 12, "y": 0}).kind == "command"
    r = router.route("POST", "/light", {"k": 0, "x": 1e6, "y": 0})
    assert r.code == 400 and r.body["error"] == "bad_value"


def test_split_and_recruit(router):
    r = router.route("POST", "/split", {"organism": 3})
    assert r.kind == "command" and r.args == {"organism": "3"}
    r = router.route("POST", "/recruit", {"organism": "L0_e2", "cell": 7})
    assert r.args == {"organism": "L0_e2", "cell": 7}
    r = router.route("POST", "/recruit", {"organism": "L0_e2", "cell": -1})
    assert r.code == 400 and r.body["error"] == "bad_value"
    r = router.route("POST", "/split", {"organism": [1]})
    assert r.code == 400 and r.body["error"] == "bad_type"


def test_dim_range(router):
    assert router.route("POST", "/dim", {"factor": 0}).args == {"factor": 0.0}
    assert router.route("POST", "/dim", {"factor": 2.5}).kind == "command"
    assert router.route("POST", "/dim", {"factor": -0.1}).code == 400
    assert router.route("POST", "/dim", {"factor": SF.DIM_MAX + 1}).code == 400


def test_every_error_envelope_has_ok_false(router):
    for method, path, body in [("GET", "/x", None), ("GET", "/dim", None),
                               ("POST", "/dim", b"{"), ("POST", "/dim", b"{}"),
                               ("POST", "/dim", b'{"factor": "a"}'),
                               ("POST", "/dim", b'{"factor": -1}')]:
        r = router.route(method, path, body)
        assert r.kind == "reply" and r.body["ok"] is False and "error" in r.body


# ------------------------------------------------------------------ queue

def _submit_in_thread(q, verb, args, timeout=5.0):
    box = {}

    def run():
        box["res"] = q.submit(verb, args, timeout)
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t, box


def test_submit_then_drain_returns_the_executors_result():
    q = SF.SimThreadQueue()
    t, box = _submit_in_thread(q, "dim", {"factor": 0.5})
    deadline = time.time() + 5
    n = 0
    while n == 0 and time.time() < deadline:
        n = q.drain(lambda c, tick: (200, {"ok": True, "verb": c.verb, "tick": tick}), tick=7)
        time.sleep(0.005)
    t.join(5)
    assert box["res"] == (200, {"ok": True, "verb": "dim", "tick": 7})
    assert q.stats["executed"] == 1 and not q.busy


def test_second_command_while_one_is_in_flight_is_409():
    q = SF.SimThreadQueue()
    t, box = _submit_in_thread(q, "split", {"organism": "a"})
    deadline = time.time() + 5
    while not q.busy and time.time() < deadline:
        time.sleep(0.005)
    code, body = q.submit("dim", {"factor": 1})
    assert code == 409 and body["error"] == "busy"
    assert q.stats["rejected_busy"] == 1
    q.drain(lambda c, tick: (200, {"ok": True}), tick=1)
    t.join(5)
    assert box["res"][0] == 200
    assert q.submit("dim", {"factor": 1}, timeout=0.01)[0] == 504   # free again


def test_timeout_is_504_and_the_command_is_abandoned():
    q = SF.SimThreadQueue()
    code, body = q.submit("dim", {"factor": 1}, timeout=0.05)
    assert code == 504 and body["error"] == "timeout"
    ran = []
    n = q.drain(lambda c, tick: ran.append(c) or (200, {}), tick=0)
    assert n == 0 and ran == []                  # never executed later
    assert q.stats["timed_out"] == 1 and q.stats["executed"] == 0


def test_deferred_completion_fires_only_at_due_tick():
    q = SF.SimThreadQueue()
    measured = {"pos": [0, 0, 0]}

    def executor(c, tick):
        measured["pos"] = [1.0, 2.0, 0.005]        # "stepped and read back"
        return q.defer(c, tick + 3, lambda: (200, {"ok": True, "position_measured": measured["pos"]}))

    t, box = _submit_in_thread(q, "light", {"k": 0, "x": 1, "y": 2})
    deadline = time.time() + 5
    while q.stats["executed"] == 0 and time.time() < deadline:
        q.drain(executor, tick=10)
        time.sleep(0.005)
    assert q.pending == 1 and "res" not in box
    q.drain(executor, tick=12)
    assert q.pending == 1 and "res" not in box
    q.drain(executor, tick=13)
    t.join(5)
    assert q.pending == 0
    assert box["res"] == (200, {"ok": True, "position_measured": [1.0, 2.0, 0.005]})
    assert q.stats["deferred"] == 1 and q.stats["completed"] == 1


def test_executor_exception_becomes_500_not_a_dead_sim_thread():
    q = SF.SimThreadQueue()
    t, box = _submit_in_thread(q, "split", {"organism": "x"})

    def boom(c, tick):
        raise RuntimeError("no such organism")
    deadline = time.time() + 5
    while q.stats["executed"] == 0 and time.time() < deadline:
        q.drain(boom, tick=0)
        time.sleep(0.005)
    t.join(5)
    assert box["res"][0] == 500 and "no such organism" in box["res"][1]["detail"]


def test_executor_none_without_defer_is_500():
    q = SF.SimThreadQueue()
    t, box = _submit_in_thread(q, "split", {"organism": "x"})
    deadline = time.time() + 5
    while q.stats["executed"] == 0 and time.time() < deadline:
        q.drain(lambda c, tick: None, tick=0)
        time.sleep(0.005)
    t.join(5)
    assert box["res"][0] == 500 and box["res"][1]["error"] == "no_result"


def test_drain_is_bounded_per_tick():
    q = SF.SimThreadQueue()
    for i in range(5):
        q._q.put(SF.Cmd("dim", {"factor": i}))       # bypass the busy lock on purpose
    assert q.drain(lambda c, tick: (200, {}), tick=0, max_n=2) == 2
    assert q.drain(lambda c, tick: (200, {}), tick=1, max_n=8) == 3


def test_census_box_publishes_copies():
    box = SF.CensusBox()
    assert box.get() == {"ready": False}
    snap = {"cells": [1, 2]}
    box.publish(snap)
    got = box.get()
    assert got["ready"] is True and got["cells"] == [1, 2]
    got["cells"].append(3)
    snap["cells"].append(4)
    assert box.get()["cells"] == [1, 2]         # neither side can mutate the snapshot
    assert "ready" not in snap


# ---------------------------------------------------------------- handler

@pytest.fixture
def server():
    router = SF.Router(n_patches=5, arena=18.0)
    q = SF.SimThreadQueue()
    census = SF.CensusBox()
    census.publish({"organisms": [], "cells": [], "tick": 0})
    gate = threading.Event()
    gate.set()
    state = {"tick": 0}

    def executor(c, tick):
        gate.wait(5)                               # tests clear it to hold a command
        if c.verb == "light":
            return q.defer(c, tick + 1, lambda: (200, {"ok": True, "k": c.args["k"],
                                                       "position_measured": [c.args["x"], c.args["y"], 0.005]}))
        return 200, {"ok": True, "verb": c.verb}

    stop = threading.Event()

    def sim_thread():
        while not stop.is_set():
            q.drain(executor, state["tick"])
            state["tick"] += 1
            time.sleep(0.002)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), SF.make_handler(router, q, census.get, timeout=2.0))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=sim_thread, daemon=True).start()
    yield {"port": srv.server_address[1], "gate": gate, "queue": q}
    stop.set()
    srv.shutdown()
    srv.server_close()


def _call(port, method, path, body=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    data = json.dumps(body).encode() if body is not None else None
    conn.request(method, path, body=data, headers={"Content-Type": "application/json"} if data else {})
    resp = conn.getresponse()
    out = json.loads(resp.read())
    conn.close()
    return resp.status, out, resp.version


def test_http_end_to_end(server):
    p = server["port"]
    st, body, ver = _call(p, "GET", "/healthz")
    assert st == 200 and body == {"ok": True} and ver == 11        # HTTP/1.1
    st, body, _ = _call(p, "GET", "/capabilities")
    assert st == 200 and "/light" in body["verbs"]
    st, body, _ = _call(p, "GET", "/census")
    assert st == 200 and body["ready"] and body["tick"] == 0
    st, body, _ = _call(p, "POST", "/light", {"k": 1, "x": 2.0, "y": 3.0})
    assert st == 200 and body["position_measured"] == [2.0, 3.0, 0.005]
    st, body, _ = _call(p, "POST", "/light", {"k": 9, "x": 2.0, "y": 3.0})
    assert st == 400 and body["error"] == "bad_value"
    st, body, _ = _call(p, "POST", "/nope", {})
    assert st == 404
    st, body, _ = _call(p, "GET", "/split")
    assert st == 405


def test_http_409_while_a_command_is_in_flight(server):
    p, gate, q = server["port"], server["gate"], server["queue"]
    gate.clear()
    box = {}

    def slow():
        box["res"] = _call(p, "POST", "/split", {"organism": "L0"})
    t = threading.Thread(target=slow, daemon=True)
    t.start()
    deadline = time.time() + 5
    while not q.busy and time.time() < deadline:
        time.sleep(0.005)
    st, body, _ = _call(p, "POST", "/dim", {"factor": 1})
    assert st == 409 and body["error"] == "busy"
    gate.set()
    t.join(5)
    assert box["res"][0] == 200 and box["res"][1]["verb"] == "split"
