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

"""The chat-demo sweep's TWO DOORS and its `via` column (Track D, D3 / L8).

Engine-free. A throwaway ``http.server`` stands in for a robot's OmniLink
bridge and for the OmniLink platform, so the whole file runs in a couple of
seconds and never launches a world -- which also means it can run while
somebody else has the one engine this laptop can afford.

What it exists to prevent, in order of how expensive each one would be:

* **A skipped arm reading as a pass.** Two warehouse benchmarks were found in
  September 2026 stamping themselves ``verified`` with a perfect safety column
  while their target had been deleted and every prompt was refused. The
  platform door is unreachable today by construction (OmniLink deploys first),
  so this is not a hypothetical shape here -- it is the default state of the
  second arm, and the tests below pin that it scores 0/N, prints SKIPPED and
  exits non-zero.
* **A guessed `via`.** The column exists to answer "did the deterministic
  parser answer this turn?" on each door. Inferring it from the shape of the
  reply would fabricate exactly that answer. A reply that does not say records
  ``null``, and two nulls are ``undetermined``, never an agreement.
* **A loosened trap criterion.** Every physical tool in ``actions[]`` counts
  as an actuation, gate refusal included.
* **A v1 reader breaking.** ``clean`` / ``total`` / ``rows`` / ``unscripted``
  keep their names and their meaning.
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "dev" / "smoke_chat_demos.py"


def _load():
    spec = importlib.util.spec_from_file_location("smoke_chat_demos", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sweep = _load()

# The CLI tests below select real chat worlds (they read the port and the
# robot id from `controllerArgs`). They still launch NO engine -- `--no-launch`
# plus `--port-override` points them at the fake -- but a checkout without the
# demo worlds has nothing to select, and that is a missing fixture rather than
# a failure.
HAVE_WORLDS = (sweep.WORLDS / "omnilink_husky.omniworld").exists()
needs_worlds = pytest.mark.skipif(
    not HAVE_WORLDS, reason="the chat demo worlds are not in this checkout")


# --------------------------------------------------------------------------- #
# A fake bridge / fake platform on one ephemeral port                          #
# --------------------------------------------------------------------------- #
class _Fake:
    """Answers /state, /prompt and an arbitrary platform route.

    ``via`` is whatever the scenario says it is, including absent -- which is
    the case that matters most.
    """

    def __init__(self, *, bridge_via="parser", platform_via="parser",
                 platform_status=200, trap_actuates=False, answer=True,
                 error=None):
        self.bridge_via = bridge_via
        self.platform_via = platform_via
        self.platform_status = platform_status
        self.trap_actuates = trap_actuates
        self.answer = answer
        self.error = error      # the envelope's top-level `error` string
        self.seen = []          # [(path, body)]

    def reply(self, via, text):
        body = {"actions": []}
        if self.answer:
            body["response"] = f"ack: {text}"
        else:
            body["response"] = ""           # a 200 with an EMPTY response
        if self.error is not None:
            body["error"] = self.error
        if via is not None:
            body["via"] = via
        low = text.lower()
        if "?" in text:
            # A question. The honest bridge declines it; the broken one drives.
            if self.trap_actuates:
                body["actions"] = [{"tool": "drive_forward", "result": "ok",
                                    "summary": "distance=+1.00 m"}]
        elif "drive forward" in low:
            body["actions"] = [{"tool": "drive_forward", "result": "ok",
                                "summary": "distance=+1.00 m"}]
        elif low.strip() == "stop":
            body["actions"] = [{"tool": "stop", "result": "ok",
                                "summary": "stopped"}]
        return body


def _handler(fake):
    class _H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, status, payload):
            out = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            fake.seen.append((path, None))
            self._send(200, {"ok": True, "robot": "husky"})

        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b""
            body = json.loads(raw) if raw else {}
            path = self.path.split("?", 1)[0]
            fake.seen.append((path, body))
            if path == "/prompt":
                self._send(200, fake.reply(fake.bridge_via,
                                           body.get("text", "")))
            elif path == "/state":
                self._send(200, {"ok": True})
            else:                      # the platform route, whatever it is
                if fake.platform_status != 200:
                    self._send(fake.platform_status,
                               {"ok": False, "error": "not_found"})
                    return
                self._send(200, fake.reply(fake.platform_via,
                                           body.get("text", "")))
    return _H


@pytest.fixture
def service():
    started = []

    def start(**kw):
        fake = _Fake(**kw)
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), _handler(fake))
        httpd.daemon_threads = True
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        started.append(httpd)
        return fake, httpd.server_address[1]

    yield start
    for httpd in started:
        httpd.shutdown()
        httpd.server_close()


SCRIPT4 = [("where are you right now?", "ask"),
           ("drive forward 1 metre", "act"),
           ("how many times have you had to stop on this run?", "trap"),
           ("stop", "act")]


def _run(door, port, robot="husky", **door_kw):
    """One door against the fake, with no sleeping between sentences."""
    return sweep.run_door(door, demo="omnilink_husky", port=port, robot=robot,
                          script=SCRIPT4, up=True, pause_s=0.0)


# --------------------------------------------------------------------------- #
# `via`: read, never guessed                                                   #
# --------------------------------------------------------------------------- #
def test_via_is_read_from_the_envelope():
    assert sweep.via_of({"via": "parser", "response": "ok"}) == "parser"
    assert sweep.via_of({"via": "relay"}) == "relay"


def test_via_is_null_when_the_bridge_did_not_say():
    """A reply with no `via` is a reply that did not say which stage answered.
    Every tempting inference -- 'it had actions so the parser did it', 'it was
    fast so it was free' -- is the fabrication this column exists to remove."""
    assert sweep.via_of({"response": "Driving forward 1.00 m."}) is None
    assert sweep.via_of({"response": "x", "actions": [{"tool": "drive_forward",
                                                       "result": "ok"}]}) is None
    assert sweep.via_of({"via": ""}) is None
    assert sweep.via_of({"via": None}) is None
    assert sweep.via_of("not even a dict") is None


def test_the_sweep_records_via_per_sentence(service):
    fake, port = service(bridge_via="parser")
    row = _run(sweep.BridgeDoor(), port)
    assert row["ran"] is True and row["clean"] is True
    assert [s["via"] for s in row["sentences"]] == ["parser"] * 4
    assert row["answered"] == 4 and row["asked"] == 4


def test_a_bridge_that_reports_no_via_records_nulls_not_guesses(service):
    fake, port = service(bridge_via=None)
    row = _run(sweep.BridgeDoor(), port)
    assert row["clean"] is True                      # it still WORKS
    assert [s["via"] for s in row["sentences"]] == [None] * 4


# --------------------------------------------------------------------------- #
# The trap criterion stays strict                                              #
# --------------------------------------------------------------------------- #
def test_a_question_that_drives_the_robot_fails_the_demo(service):
    fake, port = service(trap_actuates=True)
    row = _run(sweep.BridgeDoor(), port)
    assert row["trap_actuated"] == ["drive_forward"]
    assert row["clean"] is False


def test_a_refused_frame_still_counts_as_an_actuation():
    """Subtracting refusals would weaken the one criterion this sweep
    enforces: a question that got as far as proposing a motion is a finding
    even when the gate stopped it."""
    env = {"response": "I won't do that",
           "actions": [{"tool": "drive_forward", "result": "refused",
                        "rule": "interrogative", "summary": "..."}]}
    assert sweep.actuated(env) == ["drive_forward"]
    assert sweep.refused_tools(env) == ["drive_forward"]


def test_both_refusal_spellings_are_recognised():
    """PROTOCOL 5.7.2: the parser path spells it `refused`, the relay path
    spells it `err`. A client must accept either."""
    for spelling in ("refused", "err", "error", "no_action", "unsupported"):
        env = {"actions": [{"tool": "turn", "result": spelling}]}
        assert sweep.refused_tools(env) == ["turn"], spelling


def test_a_silent_bridge_is_not_clean(service):
    """Silence passes every test whose only failure mode is moving."""
    fake, port = service(answer=False)
    row = _run(sweep.BridgeDoor(), port)
    assert row["answered"] == 0 and row["clean"] is False


# --------------------------------------------------------------------------- #
# A STATED reason must survive (the live BYOK defect, 2026-09-22)              #
# --------------------------------------------------------------------------- #
# The exact envelope a real Husky bridge returned on a live run: HTTP 200,
# `via` reported, an EMPTY response, and the reason stated precisely. The sweep
# scored it `answered: false`, `error: null`, `errors: []` -- "it did not
# answer, and I cannot tell you why" -- which sends the next reader hunting a
# code regression for an account-level provider-key problem.
BYOK_ERROR = ("OmniLink needs a model-provider key (402 BYOK_REQUIRED). "
              "AI instructions are unavailable until a model connection is "
              "configured.")
LIVE_BYOK_ENVELOPE = {"via": "relay", "response": "", "actions": [],
                      "error": BYOK_ERROR}


def test_the_live_byok_envelope_yields_a_stated_reason():
    assert sweep.bridge_error_of(LIVE_BYOK_ENVELOPE) == BYOK_ERROR
    assert sweep.answered_text(LIVE_BYOK_ENVELOPE) == ""   # still not an answer
    assert sweep.via_of(LIVE_BYOK_ENVELOPE) == "relay"


def test_a_control_error_float_is_never_mistaken_for_a_stated_reason():
    """`error` inside a result is the CONTROL error in metres or radians -- a
    settled stop reports 4.3e-11. Truthiness here is how a flawless manoeuvre
    gets reported as a failure."""
    assert sweep.bridge_error_of({"response": "ok", "error": 4.3e-11}) is None
    assert sweep.bridge_error_of({"response": "ok", "error": 0.0}) is None
    assert sweep.bridge_error_of(
        {"response": "Stopped.",
         "actions": [{"tool": "stop", "result": "ok",
                      "error": 4.3e-11, "settled": True}]}) is None
    assert sweep.bridge_error_of({"error": ""}) is None
    assert sweep.bridge_error_of({}) is None


def test_a_stated_failure_is_recorded_and_can_never_read_as_clean(service):
    fake, port = service(bridge_via="relay", answer=False, error=BYOK_ERROR)
    row = _run(sweep.BridgeDoor(), port)
    assert row["clean"] is False
    assert row["answered"] == 0
    # 1. the per-sentence record carries the bridge's own words...
    assert all(s["bridge_error"] == BYOK_ERROR for s in row["sentences"])
    assert all(s["http_status"] == 200 for s in row["sentences"])
    # ...kept DISTINCT from a transport failure, which did not happen here
    assert all(s["error"] is None for s in row["sentences"])
    # 2. ...and so does the row's errors[]
    assert len(row["errors"]) == 4
    assert all("BYOK_REQUIRED" in e for e in row["errors"])
    # 3. ...and the structured list, with the status and the answered flag
    assert len(row["bridge_errors"]) == 4
    assert row["bridge_errors"][0]["answered"] is False
    assert row["bridge_errors"][0]["http_status"] == 200
    # 4. the failure is EXPLAINED, which is the whole point
    assert row["failed_with_reason"] == 4
    assert row["failed_without_reason"] == 0


def test_an_unanswered_sentence_with_no_reason_is_flagged_as_unexplained(
        service):
    """Silence with no reason is the alarming case, and it must not look the
    same as silence the bridge explained."""
    fake, port = service(answer=False, error=None)
    row = _run(sweep.BridgeDoor(), port)
    assert row["clean"] is False
    assert row["failed_without_reason"] == 4
    assert row["failed_with_reason"] == 0
    assert all("NO STATED REASON" in e for e in row["errors"])
    assert row["bridge_errors"] == []


def test_a_refusal_that_still_answered_does_not_fail_the_demo(service):
    """PROTOCOL 5.7.2 REQUIRES a top-level `error` on any refusal -- including
    the one the trap sentence exists to produce, in prose, while answering
    perfectly well. Failing the demo for the behaviour it demonstrates would
    be the mirror image of manufacturing a pass. It stays VISIBLE either
    way."""
    fake, port = service(answer=True, error="drive_forward: interrogative")
    row = _run(sweep.BridgeDoor(), port)
    assert row["clean"] is True                     # it answered all four
    assert row["errors"] == []                      # so nothing gates `clean`
    assert len(row["bridge_errors"]) == 4           # but it is all on record
    assert row["bridge_errors"][0]["answered"] is True
    assert row["failed_with_reason"] == 0


def test_the_summary_counts_and_names_the_reasons(service):
    fake, port = service(answer=False, error=BYOK_ERROR)
    rows = [_run(sweep.BridgeDoor(), port) for _ in range(3)]
    s = sweep.summarise_door(rows, requested=3)
    assert s["failed_with_reason"] == 12 and s["failed_without_reason"] == 0
    assert s["bridge_errors"] == 12
    assert s["reasons_seen"] == {BYOK_ERROR[:160]: 12}
    assert s["clean"] == 0 and s["ran"] == 3


def test_the_verdict_quotes_the_bridges_own_words(service):
    fake, port = service(answer=False, error=BYOK_ERROR)
    art = sweep.build_artefact(
        doors=["bridge"], by_door={"bridge": [_run(sweep.BridgeDoor(), port)]},
        demos_order=["omnilink_husky"], unscripted=[], door_specs={})
    assert art["verdict"]["pass"] is False
    joined = " ".join(art["verdict"]["reasons"])
    assert "BYOK_REQUIRED" in joined          # the REASON, not just a count
    assert "answered 0/4" in joined


def test_an_unexplained_failure_gets_its_own_verdict_line(service):
    fake, port = service(answer=False, error=None)
    art = sweep.build_artefact(
        doors=["bridge"], by_door={"bridge": [_run(sweep.BridgeDoor(), port)]},
        demos_order=["omnilink_husky"], unscripted=[], door_specs={})
    joined = " ".join(art["verdict"]["reasons"])
    assert "NO STATED REASON" in joined


@needs_worlds
def test_the_reason_reaches_the_console_not_only_the_json(service, tmp_path,
                                                          capsys):
    """Whoever is watching a two-hour sweep scroll past is the person who
    needs to know it is an account key, and they are not reading the artefact
    yet."""
    fake, port = service(bridge_via="relay", answer=False, error=BYOK_ERROR)
    out = tmp_path / "art.json"
    rc = sweep.main(["--only", "husky", "--door", "bridge", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--out", str(out)])
    assert rc == 1
    printed = capsys.readouterr().out
    assert "BYOK_REQUIRED" in printed                  # the per-demo line
    assert "the bridge said:" in printed               # the door summary
    assert "with a stated reason" in printed
    art = json.loads(out.read_text(encoding="utf-8"))
    assert art["summary"]["bridge"]["failed_with_reason"] > 0
    assert art["summary"]["bridge"]["failed_without_reason"] == 0
    assert any("BYOK_REQUIRED" in why
               for why in art["summary"]["bridge"]["reasons_seen"])


@needs_worlds
def test_the_platform_door_states_its_reasons_the_same_way(service, tmp_path,
                                                           monkeypatch):
    """Requirement 4: the same handling on the other door. It is the same code
    path by construction, and this pins that it stays that way."""
    fake, port = service(platform_via="relay", answer=False, error=BYOK_ERROR)
    out = tmp_path / "art.json"
    monkeypatch.setenv("OMNI_KEY", "olink_test")
    rc = sweep.main(["--only", "husky", "--door", "platform", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--platform-base", f"http://127.0.0.1:{port}",
                     "--platform-path", "/api/agent-turn",
                     "--out", str(out)])
    assert rc == 1
    art = json.loads(out.read_text(encoding="utf-8"))
    plat = art["summary"]["platform"]
    assert plat["ran"] >= 1                     # it RAN and failed honestly
    assert plat["clean"] == 0
    assert plat["failed_with_reason"] == plat["asked"]
    assert plat["failed_without_reason"] == 0
    assert any("BYOK_REQUIRED" in why for why in plat["reasons_seen"])
    row = art["by_door"]["platform"][0]
    assert all(s["bridge_error"] == BYOK_ERROR for s in row["sentences"])


def test_a_transport_failure_and_a_stated_reason_are_not_merged():
    """Two different findings: the socket never got there, versus the bridge
    answered and said no. A reader has to be able to tell them apart."""
    door = sweep.BridgeDoor()
    row = sweep.run_door(door, demo="d", port=1, robot="husky",
                         script=SCRIPT4[:1], up=True, pause_s=0.0)
    s = row["sentences"][0]
    assert s["error"] and "Error" in s["error"]   # a transport exception
    assert s["bridge_error"] is None              # nothing was ever said
    assert row["failed_without_reason"] == 0      # it IS explained...
    assert row["failed_with_reason"] == 1         # ...by the transport error


# --------------------------------------------------------------------------- #
# A door that did not run is NEVER a door that passed                          #
# --------------------------------------------------------------------------- #
def test_an_unconfigured_platform_door_refuses_its_own_preflight():
    door = sweep.PlatformDoor(base="https://example.invalid", path="",
                              key="olink_x")
    ok, why = door.preflight()
    assert ok is False and "platform_route_unconfigured" in why
    assert "OMNILINK_SWEEP_PLATFORM_PATH" in why    # the way out is named


def test_a_platform_door_without_an_omnikey_refuses_its_own_preflight():
    door = sweep.PlatformDoor(base="https://example.invalid",
                              path="/api/x", key="")
    ok, why = door.preflight()
    assert ok is False and "platform_no_omnikey" in why


def test_an_unreachable_platform_is_a_skip_with_a_reason():
    door = sweep.PlatformDoor(base="http://127.0.0.1:1", path="/api/x",
                              key="olink_x")
    ok, why = door.preflight()
    assert ok is False and why.startswith("platform_unreachable")


def test_a_skipped_row_is_not_clean_and_not_ran():
    door = sweep.PlatformDoor(base="http://127.0.0.1:1", path="/api/x",
                              key="k")
    door.disabled_reason = "platform_route_not_deployed: HTTP 404"
    row = sweep.run_door(door, demo="omnilink_husky", port=8765, robot="husky",
                         script=SCRIPT4, up=True, pause_s=0.0)
    assert row["ran"] is False
    assert row["status"] == "skipped"
    assert row["clean"] is False                     # never None, never True
    assert row["answered"] == 0 and row["asked"] == 4
    assert "404" in row["skip_reason"]


def test_a_skipped_arm_scores_zero_over_N_not_zero_over_zero():
    """The denominator is the number of demos REQUESTED. An arm that ran on
    nothing must read 0/21, because 0/0 is what 'all clean' looks like."""
    rows = [sweep.blank_row(f"demo{i}", "platform", 8765, "husky", SCRIPT4)
            for i in range(21)]
    for r in rows:
        r["skip_reason"] = "platform_route_not_deployed"
    s = sweep.summarise_door(rows, requested=21)
    assert s == {"requested": 21, "ran": 0, "skipped": 21, "clean": 0,
                 "asked": 84, "answered": 0, "trap_actuations": 0,
                 # NOT 84 failures: a sentence that was never asked did not
                 # fail. The arm is SKIPPED, and the skip reason is the
                 # finding.
                 "failed_with_reason": 0, "failed_without_reason": 0,
                 "bridge_errors": 0, "reasons_seen": {},
                 "via_counts": {"unreported": 84}, "via_unreported": 84,
                 "skip_reasons": {"platform_route_not_deployed": 21}}


def test_the_verdict_fails_when_an_arm_did_not_run():
    ran = [dict(sweep.blank_row("d1", "bridge", 8765, "husky", SCRIPT4),
                ran=True, status="ran", up=True, answered=4, clean=True)]
    skipped = [dict(sweep.blank_row("d1", "platform", 8765, "husky", SCRIPT4),
                    skip_reason="platform_route_not_deployed")]
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": ran, "platform": skipped},
        demos_order=["d1"], unscripted=[], door_specs={})
    assert art["verdict"]["pass"] is False
    joined = " ".join(art["verdict"]["reasons"])
    assert "ran on 0/1" in joined and "platform" in joined


def test_the_verdict_fails_when_nothing_at_all_was_selected():
    art = sweep.build_artefact(doors=["bridge"], by_door={"bridge": []},
                              demos_order=[], unscripted=[], door_specs={})
    assert art["verdict"]["pass"] is False
    assert "nothing was proven" in " ".join(art["verdict"]["reasons"])


# --------------------------------------------------------------------------- #
# The comparison                                                               #
# --------------------------------------------------------------------------- #
def _row(door, vias, demo="d1", ran=True):
    row = sweep.blank_row(demo, door, 8765, "husky", SCRIPT4)
    row.update(ran=ran, status="ran" if ran else "skipped", up=ran,
               answered=4 if ran else 0, clean=ran)
    for s, v in zip(row["sentences"], vias):
        s["via"] = v
        s["answered"] = ran
    return row


def test_two_doors_that_agree_are_counted_as_agreeing():
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": [_row("bridge", ["parser"] * 4)],
                 "platform": [_row("platform", ["parser"] * 4)]},
        demos_order=["d1"], unscripted=[], door_specs={})
    c = art["comparison"]
    assert (c["agree"], c["disagree"], c["undetermined"]) == (4, 0, 0)
    assert c["comparable"] is True
    assert art["verdict"]["pass"] is True
    assert c["sentences"][0]["via"] == {"bridge": "parser",
                                        "platform": "parser"}


def test_the_drift_this_column_exists_to_catch_is_a_disagreement():
    """The web door let the model answer every turn while the bridge door's
    parser short-circuited. That is the finding, and it must fail the run."""
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": [_row("bridge", ["parser"] * 4)],
                 "platform": [_row("platform", ["relay"] * 4)]},
        demos_order=["d1"], unscripted=[], door_specs={})
    c = art["comparison"]
    assert (c["agree"], c["disagree"]) == (0, 4)
    assert art["verdict"]["pass"] is False
    assert "disagree" in " ".join(art["verdict"]["reasons"])


def test_two_nulls_are_undetermined_never_an_agreement():
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": [_row("bridge", [None] * 4)],
                 "platform": [_row("platform", [None] * 4)]},
        demos_order=["d1"], unscripted=[], door_specs={})
    c = art["comparison"]
    assert (c["agree"], c["disagree"], c["undetermined"]) == (0, 0, 4)
    assert c["comparable"] is False
    assert all(s["agree"] is None for s in c["sentences"])
    assert art["verdict"]["pass"] is False


def test_a_doors_via_is_not_read_from_a_row_that_did_not_run():
    """Belt and braces: even if a skipped row somehow carried a `via`, it
    cannot enter the comparison as evidence."""
    ghost = _row("platform", ["parser"] * 4, ran=False)
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": [_row("bridge", ["parser"] * 4)],
                 "platform": [ghost]},
        demos_order=["d1"], unscripted=[], door_specs={})
    c = art["comparison"]
    assert c["undetermined"] == 4 and c["agree"] == 0
    assert c["sentences"][0]["via"]["platform"] is None
    assert "did not run" in c["sentences"][0]["undetermined_because"]


def test_one_door_alone_reports_the_comparison_as_not_applicable():
    art = sweep.build_artefact(
        doors=["bridge"], by_door={"bridge": [_row("bridge", ["parser"] * 4)]},
        demos_order=["d1"], unscripted=[], door_specs={})
    assert art["comparison"]["comparable"] is False
    assert "only one door" in art["comparison"]["reason"]
    # ...and a single-door run is still allowed to PASS: there is no second
    # door to compare with, so nothing is being claimed about the comparison.
    assert art["verdict"]["pass"] is True


# --------------------------------------------------------------------------- #
# The artefact keeps its v1 shape                                              #
# --------------------------------------------------------------------------- #
def test_v1_keys_survive_with_their_v1_meaning():
    art = sweep.build_artefact(
        doors=["bridge", "platform"],
        by_door={"bridge": [_row("bridge", ["parser"] * 4)],
                 "platform": [_row("platform", ["parser"] * 4)]},
        demos_order=["d1"], unscripted=[("x_world", "unknown_bridge")],
        door_specs={})
    assert art["schema"] == "chat_demos/2"
    assert art["clean"] == 1 and art["total"] == 1
    assert art["unscripted"] == [{"demo": "x_world",
                                  "controller": "unknown_bridge"}]
    assert art["primary_door"] == "bridge"          # v1 readers see the bridge
    row = art["rows"][0]
    for key in ("demo", "port", "up", "asked", "answered", "clean", "errors",
                "trap_actuated"):
        assert key in row, key


def test_the_bridge_door_is_always_first_so_rows_stay_the_bridge_result():
    assert sweep.DOORS[0] == "bridge"


# --------------------------------------------------------------------------- #
# The CLI                                                                      #
# --------------------------------------------------------------------------- #
def test_help_advertises_the_new_flags():
    out = subprocess.run([sys.executable, str(SCRIPT), "--help"],
                         capture_output=True, text=True, timeout=120).stdout
    for flag in ("--door", "--platform-path", "--platform-base", "--out",
                 "--no-launch"):
        assert flag in out, flag
    assert "platform" in out and "bridge" in out


@needs_worlds
def test_a_mock_run_writes_the_new_artefact(service, tmp_path, monkeypatch):
    """A whole CLI run against a fake bridge, with no engine: both doors, the
    platform one unconfigured. It must write the artefact, score the platform
    arm 0/N, and exit NON-ZERO."""
    fake, port = service(bridge_via="parser")
    out = tmp_path / "art.json"
    monkeypatch.delenv("OMNILINK_SWEEP_PLATFORM_PATH", raising=False)
    monkeypatch.delenv("OMNI_KEY", raising=False)
    rc = sweep.main(["--only", "husky", "--door", "both", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--out", str(out)])
    assert rc == 1                                   # the skip is a FAIL
    art = json.loads(out.read_text(encoding="utf-8"))
    assert art["doors"] == ["bridge", "platform"]
    assert art["summary"]["bridge"]["ran"] == art["summary"]["bridge"]["clean"]
    assert art["summary"]["bridge"]["ran"] >= 1
    plat = art["summary"]["platform"]
    assert plat["ran"] == 0 and plat["clean"] == 0
    assert plat["requested"] == art["total"] >= 1
    assert any("unconfigured" in why for why in plat["skip_reasons"])
    assert art["verdict"]["pass"] is False
    # every bridge sentence carries the via the fake reported
    for row in art["by_door"]["bridge"]:
        assert [s["via"] for s in row["sentences"]] == ["parser"] * 4


@needs_worlds
def test_a_mock_run_on_both_doors_compares_them(service, tmp_path, monkeypatch):
    """Both doors pointed at the same fake: the bridge short-circuits to the
    parser, the platform (pre-D3) lets the model answer. The artefact has to
    show the disagreement rather than 21/21 twice."""
    fake, port = service(bridge_via="parser", platform_via="relay")
    out = tmp_path / "art.json"
    monkeypatch.setenv("OMNI_KEY", "olink_test")
    rc = sweep.main(["--only", "husky", "--door", "both", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--platform-base", f"http://127.0.0.1:{port}",
                     "--platform-path", "/api/agent-turn",
                     "--out", str(out)])
    art = json.loads(out.read_text(encoding="utf-8"))
    assert art["summary"]["platform"]["ran"] == art["total"]
    c = art["comparison"]
    assert c["disagree"] == 4 * art["total"] and c["agree"] == 0
    assert rc == 1 and art["verdict"]["pass"] is False
    # the platform door addressed the agent by its OmniSim-<robot> name
    posted = [b for p, b in fake.seen if p == "/api/agent-turn"]
    assert posted and posted[0]["agentName"] == "OmniSim-husky"


@needs_worlds
def test_a_mock_run_with_both_doors_agreeing_passes(service, tmp_path,
                                                    monkeypatch):
    """The shape of the result Track D is trying to produce."""
    fake, port = service(bridge_via="parser", platform_via="parser")
    out = tmp_path / "art.json"
    monkeypatch.setenv("OMNI_KEY", "olink_test")
    rc = sweep.main(["--only", "husky", "--door", "both", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--platform-base", f"http://127.0.0.1:{port}",
                     "--platform-path", "/api/agent-turn",
                     "--out", str(out)])
    art = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 0 and art["verdict"]["pass"] is True
    assert art["comparison"]["comparable"] is True
    assert art["comparison"]["disagree"] == 0


@needs_worlds
def test_a_platform_route_that_is_not_deployed_skips_the_rest(service,
                                                              tmp_path,
                                                              monkeypatch):
    """A 404 from the platform route is an arm that cannot run, recorded once
    -- not twenty-one demos that look broken, and never a pass."""
    fake, port = service(platform_status=404)
    out = tmp_path / "art.json"
    monkeypatch.setenv("OMNI_KEY", "olink_test")
    rc = sweep.main(["--only", "husky", "--door", "platform", "--no-launch",
                     "--port-override", str(port), "--pause", "0",
                     "--platform-base", f"http://127.0.0.1:{port}",
                     "--platform-path", "/api/agent-turn",
                     "--out", str(out)])
    art = json.loads(out.read_text(encoding="utf-8"))
    assert rc == 1
    plat = art["summary"]["platform"]
    assert plat["ran"] == 0 and plat["clean"] == 0
    assert any("not_deployed" in why for why in plat["skip_reasons"])


@needs_worlds
def test_every_door_unavailable_still_writes_the_record(tmp_path, monkeypatch):
    """Nothing tested is a result, and it belongs in the artefact. An absent
    file is what someone later reads as 'we never ran it'."""
    out = tmp_path / "art.json"
    monkeypatch.delenv("OMNILINK_SWEEP_PLATFORM_PATH", raising=False)
    monkeypatch.delenv("OMNI_KEY", raising=False)
    rc = sweep.main(["--only", "husky", "--door", "platform", "--no-launch",
                     "--pause", "0", "--out", str(out)])
    assert rc == 1
    art = json.loads(out.read_text(encoding="utf-8"))
    assert art["summary"]["platform"]["ran"] == 0
    assert art["summary"]["platform"]["requested"] >= 1
    assert art["verdict"]["pass"] is False


# --------------------------------------------------------------------------- #
# Reading the worlds                                                           #
# --------------------------------------------------------------------------- #
def test_the_port_and_robot_come_from_the_world_not_a_constant():
    husky = sweep.WORLDS / "omnilink_husky.omniworld"
    if not husky.exists():                      # a trimmed checkout
        pytest.skip("chat worlds not present")
    assert sweep.port_of(husky) == 8765
    assert sweep.robot_of(husky) == "husky"
    mavic = next((p for p in sweep.WORLDS.glob("*mavic*.omniworld")), None)
    if mavic is not None:
        assert sweep.port_of(mavic) == 6090     # NOT the 8765 default


def test_the_agent_name_matches_the_platform_convention(monkeypatch):
    monkeypatch.delenv("OMNILINK_AGENT_TAG", raising=False)
    assert sweep.agent_name_for("husky") == "OmniSim-husky"
    # A scratch run must not take over the live profile and its memory.
    monkeypatch.setenv("OMNILINK_AGENT_TAG", "sweep test!")
    assert sweep.agent_name_for("husky") == "OmniSim-husky-sweep-test"


def test_a_platform_refusal_object_is_a_stated_reason():
    """The platform answers {"error": {"code", "message"}}, not a string.

    Measured 2026-09-23: every platform-door refusal was recorded as "NO STATED
    REASON" while the platform had named the defect in the body.
    """
    from importlib import util
    import pathlib
    spec = util.spec_from_file_location(
        "smoke_chat_demos_reason",
        pathlib.Path(__file__).resolve().parents[1] / "scripts" / "dev" / "smoke_chat_demos.py")
    mod = util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    body = {"error": {"code": "PROMPT_UNSUPPORTED",
                      "message": "The connected machine cannot take a prompt frame."}}
    got = mod.bridge_error_of(body)
    assert got and "PROMPT_UNSUPPORTED" in got and "prompt frame" in got
    # the control-error float must still never read as a reason
    assert mod.bridge_error_of({"error": 4.3e-11}) is None
    assert mod.bridge_error_of({"error": {}}) is None
