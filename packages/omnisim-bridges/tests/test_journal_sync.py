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

"""The action journal has to follow the agent, not the machine.

The journal removed a 26% fabrication rate, and every one of those
fabrications was about the agent's own past actions. Until now it lived in
one machine's temp dir: a different OmniSim instance, a different machine
or a cleaned temp dir put the robot straight back to answering "what did
you do before the restart?" from narrative memory.

So it rides inside the memory write, as a bounded section in the same
notes-tier shape `_merge_memory` already lifts out, and `_restore_history`
splits it back into the ActionJournal BEFORE the first turn.

These tests are the round trip and its three bounds: it must survive, it
must not duplicate, and it must not grow the memory row without limit.
"""

from __future__ import annotations

import json
import threading
import time

import pytest

from omnisim_bridges.action_journal import ActionJournal
from omnisim_bridges.relay import (
    JOURNAL_MARKER,
    OmniLinkRelay,
    _build_notes,
    _is_journal,
    _journal_entry,
    _journal_rows,
    _merge_memory,
    _restore_history,
)
from omnisim_bridges.tool import Tool


def U(text: str):
    return {"role": "user", "parts": [{"text": text}]}


def M(text: str):
    return {"role": "model", "parts": [{"text": text}]}


@pytest.fixture(autouse=True)
def isolated_state(tmp_path, monkeypatch):
    """Never touch the real journal files. A test that clobbered the live
    demo's journal would be the same class of accident as a scratch run
    taking over a production agent name."""
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    return tmp_path


def journal_with(entries, owner="RoundTrip"):
    j = ActionJournal(owner=owner)
    for tool, ok, summary in entries:
        j.record(tool, {"distance": 1.0}, ok=ok, summary=summary)
    return j


# ── The round trip ───────────────────────────────────────────────────

def test_the_journal_survives_a_write_and_a_restore(tmp_path) -> None:
    """Export, write to a memory blob, restore into a FRESH journal: the
    entries match. This is the whole feature in one test."""
    source = journal_with([
        ("drive_forward", True, "drive, commanded=1.0, achieved=0.997"),
        ("turn", True, "turn, commanded=1.57, achieved=1.56"),
        ("stop_robot", True, "halted_at=1"),
    ], owner="Before")
    rows = source.export_entries()
    assert len(rows) == 3

    blob = [U("drive forward"), M("moving"), _journal_entry(rows)]

    restored = ActionJournal(owner="After", persist=False)
    history = _restore_history(blob, 40, journal=restored)

    page = restored.listing(limit=10)
    assert [e["tool"] for e in page["entries"]] == [
        "drive_forward", "turn", "stop_robot"]
    assert [e["summary"] for e in page["entries"]] == [
        r["summary"] for r in rows]
    assert all(e["before_restart"] for e in page["entries"]), (
        "restored entries did not happen in THIS process and must say so")
    assert page["includes_before_restart"] is True
    # ...and the conversation is untouched by the passenger.
    assert [m["content"] for m in history] == ["drive forward", "moving"]


def test_the_section_never_reaches_the_model_as_conversation(tmp_path) -> None:
    """The model reads the journal through `get_action_history`. Feeding it
    the same rows again as a user turn would cost tokens every round to say
    the same thing worse."""
    rows = journal_with([("drive_forward", True, "ok")]).export_entries()
    history = _restore_history([U("hello"), _journal_entry(rows)], 40,
                               journal=ActionJournal(owner="X", persist=False))
    assert not any(JOURNAL_MARKER in (m.get("content") or "") for m in history)


def test_restore_without_a_journal_still_strips_the_section() -> None:
    """A caller that does not want the journal (an older bridge, a probe)
    must not end up with a JSON blob in its transcript."""
    rows = journal_with([("turn", True, "ok")]).export_entries()
    history = _restore_history([U("hi"), _journal_entry(rows), M("hello")], 40)
    assert [m["content"] for m in history] == ["hi", "hello"]


def test_a_second_instance_on_the_same_key_restores_the_same_journal() -> None:
    """The point of using the memory row: the record follows the KEY, not
    the temp dir. Both restores see the same three actions."""
    rows = journal_with([("drive_forward", True, "a"), ("turn", True, "b"),
                         ("stop_robot", True, "c")]).export_entries()
    blob = [_journal_entry(rows)]
    first = ActionJournal(owner="inst1", persist=False)
    second = ActionJournal(owner="inst2", persist=False)
    _restore_history(blob, 40, journal=first)
    _restore_history(blob, 40, journal=second)
    assert ([e["tool"] for e in first.listing(limit=9)["entries"]]
            == [e["tool"] for e in second.listing(limit=9)["entries"]]
            == ["drive_forward", "turn", "stop_robot"])


# ── Merge, not replace ───────────────────────────────────────────────

def test_import_is_a_merge_and_keeps_what_this_machine_did() -> None:
    incoming = journal_with([("drive_forward", True, "from the platform")],
                            owner="Remote").export_entries()
    local = ActionJournal(owner="Local", persist=False)
    local.record("close_gripper", {}, ok=True, summary="mine")
    local.import_entries(incoming)
    tools = [e["tool"] for e in local.listing(limit=9)["entries"]]
    assert "close_gripper" in tools and "drive_forward" in tools


def test_importing_twice_does_not_duplicate() -> None:
    """A bridge restarts, restores, persists, restores again. Without an
    identity the journal would double on every cycle until the cap ate the
    oldest real entries."""
    rows = journal_with([("turn", True, "a"), ("stop_robot", True, "b")],
                        owner="Src").export_entries()
    j = ActionJournal(owner="Dst", persist=False)
    assert j.import_entries(rows) == 2
    assert j.import_entries(rows) == 0
    assert len(j.listing(limit=50)["entries"]) == 2


def test_import_survives_junk() -> None:
    """The blob crosses a network and a database. Nothing in it is trusted."""
    j = ActionJournal(owner="Junk", persist=False)
    assert j.import_entries("not a list") == 0
    assert j.import_entries([None, 7, {}, {"no_tool": 1}]) == 0
    assert j.import_entries([{"tool": "turn", "n": "x", "t": None}]) == 1
    assert len(j) == 1


def test_a_corrupt_section_is_ignored_not_raised() -> None:
    broken = {"role": "user", "parts": [{"text": JOURNAL_MARKER + "\n{{{"}]}
    assert _journal_rows(broken) == []
    j = ActionJournal(owner="Broken", persist=False)
    assert _restore_history([broken, U("hi")], 40, journal=j) == [
        {"role": "user", "content": "hi"}]


# ── Bounds ───────────────────────────────────────────────────────────

def test_the_section_is_bounded_in_entries_and_in_characters() -> None:
    """A long-lived robot must not grow the memory row without limit: the
    row is read and rewritten on every single turn."""
    j = ActionJournal(owner="Long", persist=False)
    for i in range(400):
        j.record("drive_forward", {"distance": i, "note": "x" * 100},
                 ok=True, summary="y" * 200)
    rows = j.export_entries()
    assert len(rows) <= 50
    text = json.dumps(rows)
    assert len(text) <= 3400, f"section grew to {len(text)} chars"
    entry = _journal_entry(rows)
    assert _is_journal(entry)


def test_an_empty_journal_writes_no_section() -> None:
    assert _journal_entry([]) is None
    assert ActionJournal(owner="Empty", persist=False).export_entries() == []


def test_the_newest_entries_are_the_ones_that_travel() -> None:
    j = ActionJournal(owner="Order", persist=False)
    for i in range(80):
        j.record("turn", {}, ok=True, summary=f"turn {i}")
    rows = j.export_entries()
    assert rows[-1]["summary"] == "turn 79"
    assert rows[0]["summary"] != "turn 0", "the oldest must be the ones cut"


# ── Living beside the notes tier in _merge_memory ────────────────────

def test_the_section_is_lifted_out_of_the_merge_like_the_notes() -> None:
    """It must never compete with a turn for a slot, and it must never be
    deduped against one."""
    rows = journal_with([("turn", True, "a")]).export_entries()
    merged = _merge_memory([U("old")], [U("new"), _journal_entry(rows)], 100)
    journals = [e for e in merged if _is_journal(e)]
    assert len(journals) == 1
    assert _journal_rows(journals[0]) == rows


def test_the_section_is_never_folded_into_the_standing_notes() -> None:
    """Eviction folds a dropped USER turn's text into the notes. Folding a
    3 kB JSON blob in there would poison the one tier whose entire value is
    that it holds the operator's own words."""
    rows = journal_with([("turn", True, "a")]).export_entries()
    mine = [U(f"turn {i}") for i in range(30)] + [_journal_entry(rows)]
    merged = _merge_memory([U(f"old {i}") for i in range(30)], mine, 10)
    notes = [e for e in merged
             if e.get("parts") and "STANDING NOTES" in e["parts"][0]["text"]]
    assert notes, "the notes tier should have caught the evicted turns"
    assert JOURNAL_MARKER not in notes[0]["parts"][0]["text"]


def test_ours_wins_over_the_stored_copy() -> None:
    """This process already merged whatever the platform held, so its
    journal is the superset; the stored one may be an older machine's."""
    old = journal_with([("turn", True, "stale")], owner="Old").export_entries()
    new = journal_with([("turn", True, "stale"), ("stop_robot", True, "fresh")],
                       owner="New").export_entries()
    merged = _merge_memory([_journal_entry(old), U("x")],
                           [U("y"), _journal_entry(new)], 100)
    carried = _journal_rows(next(e for e in merged if _is_journal(e)))
    assert [r["summary"] for r in carried] == ["stale", "fresh"]


def test_the_stored_section_survives_a_bridge_that_has_no_journal() -> None:
    """An older bridge writing memory must not DELETE the section."""
    rows = journal_with([("turn", True, "a")]).export_entries()
    merged = _merge_memory([_journal_entry(rows), U("old")], [U("new")], 100)
    assert any(_is_journal(e) for e in merged)


def test_notes_and_journal_coexist() -> None:
    rows = journal_with([("turn", True, "a")]).export_entries()
    notes = _build_notes(["* TROLLEY_E is damaged"])
    merged = _merge_memory([notes, U("old")], [U("new"), _journal_entry(rows)], 100)
    assert sum(1 for e in merged if _is_journal(e)) == 1
    assert any("TROLLEY_E" in (e.get("parts") or [{}])[0].get("text", "")
               for e in merged)


# ── The wiring: the relay actually writes it ─────────────────────────

class FakeClient:
    def __init__(self):
        self.memory = []
        self.written = threading.Event()

    def get_memory(self, agent):
        return list(self.memory)

    def set_memory(self, agent, blob):
        self.memory = list(blob)
        self.written.set()


def test_the_relay_persists_the_section_and_restores_it(monkeypatch) -> None:
    """End to end through the real write path: three tool calls, one memory
    write, and a fresh relay that reads them back. Without this the two
    halves could each be correct and never meet."""
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="JournalWiring", main_task="test",
        tools=[Tool("drive_forward", "move",
                    {"type": "object", "properties": {}},
                    lambda a: {"commanded": 1.0, "achieved": 0.99,
                               "error": -0.01})],
        usage_enabled=False, memory_enabled=True, surface="mobile")
    client = FakeClient()
    relay._client = client
    try:
        for i in range(3):
            relay.journal.record("drive_forward", {"distance": 1.0}, ok=True,
                                 summary=f"leg {i}")
        relay.history.append({"role": "user", "content": "drive forward"})
        relay._persist_memory_async()
        assert client.written.wait(timeout=5), "memory write never ran"
        # Give the worker a beat to finish assigning.
        for _ in range(50):
            if any(_is_journal(e) for e in client.memory):
                break
            time.sleep(0.02)
        section = next(e for e in client.memory if _is_journal(e))
        assert [r["summary"] for r in _journal_rows(section)] == [
            "leg 0", "leg 1", "leg 2"]
    finally:
        relay.close()

    fresh = ActionJournal(owner="AfterRestart", persist=False)
    _restore_history(client.memory, 40, journal=fresh)
    assert [e["summary"] for e in fresh.listing(limit=9)["entries"]] == [
        "leg 0", "leg 1", "leg 2"]


def test_a_journal_export_failure_does_not_stop_the_memory_write(
        monkeypatch) -> None:
    """Losing the journal section is a degraded record. Losing the write is
    the whole conversation."""
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="JournalBoom", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=True)
    client = FakeClient()
    relay._client = client

    def _boom(*a, **k):
        raise RuntimeError("export exploded")

    relay.journal.export_entries = _boom      # type: ignore[assignment]
    try:
        relay.history.append({"role": "user", "content": "still recorded"})
        relay._persist_memory_async()
        assert client.written.wait(timeout=5)
        texts = [" ".join(p["text"] for p in e["parts"]) for e in client.memory]
        assert "still recorded" in texts
    finally:
        relay.close()


# ── The sync must not need a model turn ──────────────────────────────
#
# D7 shipped the journal as a passenger on the memory write, and the memory
# write has exactly two triggers, both of which require a COMPLETED model
# turn: the `remember_this` tool, and the tail of `_dispatch_one`. Two whole
# classes of session therefore never synced at all:
#
#   * a session answered entirely by the deterministic parser, which
#     short-circuits in route.py and never enters the dispatch loop -- so
#     the better the parser works, the less the journal persists;
#   * a session whose every model turn fails. Measured live on 2026-09-22:
#     every turn on the account answered `402 BYOK_REQUIRED`, and the error
#     branch of `_dispatch_one` returns BEFORE the persist call.
#
# The local file still held the record in both cases, so "survives a restart
# on this machine" was true. "Follows the agent to another machine" was not.
#
# Note what fills the journal in those sessions, because it is why these
# tests seed it directly. The recording sites are all in the relay's dispatch
# loop, so a strictly parser-only session records nothing NEW -- but its
# journal is not empty: the boot restore fills it from the local file AND
# from the platform row, and that UNION is exactly what has to travel on. A
# machine whose disk holds entries the platform never saw is the whole
# failure, and under the shipped code that union sat on one disk until some
# later model turn happened to complete.


class CountingClient:
    """A memory client that counts, and can fail on demand.

    `written` fires on the first SUCCESSFUL write; `attempts` counts every
    call including the ones that raise, so a test can tell "never tried"
    from "tried and failed".
    """

    def __init__(self, memory=None):
        self.memory = list(memory or [])
        self.writes = 0
        self.attempts = 0
        self.reads = 0
        self.read_fails = False
        self.write_fails = False
        self.written = threading.Event()

    def get_memory(self, agent):
        self.reads += 1
        if self.read_fails:
            raise RuntimeError("get_memory exploded")
        return list(self.memory)

    def set_memory(self, agent, blob):
        self.attempts += 1
        if self.write_fails:
            raise RuntimeError("set_memory exploded")
        self.memory = list(blob)
        self.writes += 1
        self.written.set()


def make_relay(name, **kw):
    """A relay with one real tool, and a journal under the tmp state dir.

    The CONSTRUCTOR still reads platform memory over the network (it is the
    boot restore, and it fails closed), so every test swaps `_client` for a
    fake straight afterwards; nothing this file asserts depends on that
    first read succeeding.
    """
    kw.setdefault("usage_enabled", False)
    kw.setdefault("memory_enabled", True)
    return OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name=name, main_task="test",
        tools=[Tool("drive_forward", "move",
                    {"type": "object", "properties": {}},
                    lambda a: {"commanded": 1.0, "achieved": 0.99})],
        surface="mobile", **kw)


def wait_for_section(client, timeout=10.0):
    """The journal section on the platform, or an assertion failure."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        section = next((e for e in client.memory if _is_journal(e)), None)
        if section is not None:
            return section
        time.sleep(0.02)
    raise AssertionError("the journal never reached the memory row")


def settle(seconds=0.6):
    """Let the fire-and-forget write threads finish before counting."""
    time.sleep(seconds)


def test_a_parser_only_session_still_syncs_the_journal() -> None:
    """No model turn ever runs. The journal must still travel.

    This is the cheap path the whole parser-first architecture exists to
    take, so tying the sync to the dispatch loop makes the feature fail in
    proportion to how well the parser works.
    """
    relay = make_relay("ParserOnly")
    client = CountingClient()
    relay._client = client
    try:
        relay._journal_sync_interval_s = 0.05
        for i in range(3):
            relay.journal.record("drive_forward", {"distance": 1.0}, ok=True,
                                 summary=f"leg {i}")
        section = wait_for_section(client)
        assert [r["summary"] for r in _journal_rows(section)] == [
            "leg 0", "leg 1", "leg 2"]
    finally:
        relay.close()


def test_a_session_whose_model_turns_all_fail_still_syncs() -> None:
    """The 402 case, verbatim from the live run.

    The previous session on this machine left two entries in the local
    file. Every model turn in this one answers `402 BYOK_REQUIRED`, which
    returns before the persist call -- so under the shipped code the record
    stays on this disk forever.
    """
    seed = ActionJournal(owner="ByokRobot")
    seed.record("drive_forward", {"distance": 2.0}, ok=True, summary="leg A")
    seed.record("stop_robot", {}, ok=True, summary="halted")

    relay = make_relay("ByokRobot")
    client = CountingClient()
    relay._client = client
    relay._post_chat = lambda messages: {"ok": False,
                                         "error": "402 BYOK_REQUIRED"}
    try:
        relay._journal_sync_interval_s = 0.05
        out = relay.dispatch_sync("drive forward 2 m", timeout_s=15)
        assert "BYOK_REQUIRED" in (out.get("error") or ""), out
        section = wait_for_section(client)
        assert [r["summary"] for r in _journal_rows(section)] == [
            "leg A", "halted"]
    finally:
        relay.close()


def test_an_unchanged_journal_does_not_write_again() -> None:
    """A memory write is an HTTP round trip. An idle robot must cost none."""
    relay = make_relay("Idempotent")
    client = CountingClient()
    relay._client = client
    try:
        relay.journal.record("drive_forward", {}, ok=True, summary="a")
        relay._sync_journal_if_dirty()
        assert client.written.wait(timeout=10)
        settle()
        assert client.writes == 1
        for _ in range(3):
            relay._sync_journal_if_dirty()
        settle()
        assert client.writes == 1, "an unchanged journal must cost nothing"
        # ...and a new entry makes it dirty again.
        relay.journal.record("stop_robot", {}, ok=True, summary="b")
        relay._sync_journal_if_dirty()
        deadline = time.time() + 10
        while client.writes < 2 and time.time() < deadline:
            time.sleep(0.02)
        assert client.writes == 2
        rows = _journal_rows(wait_for_section(client))
        assert [r["summary"] for r in rows] == ["a", "b"]
    finally:
        relay.close()


def test_close_flushes_the_journal() -> None:
    """A clean world shutdown must not drop the last actions of a session.

    The beat is set far enough out that only the close can have written.
    """
    relay = make_relay("Closing")
    client = CountingClient()
    relay._client = client
    relay._journal_sync_interval_s = 3600.0
    relay.journal.record("stop_robot", {}, ok=True, summary="the last thing")
    relay.close()
    assert client.writes == 1, "close did not flush the journal"
    section = next(e for e in client.memory if _is_journal(e))
    assert [r["summary"] for r in _journal_rows(section)] == ["the last thing"]


def test_close_does_not_write_an_unchanged_journal() -> None:
    relay = make_relay("CleanClose")
    client = CountingClient()
    relay._client = client
    relay.journal.record("turn", {}, ok=True, summary="a")
    relay._sync_journal_if_dirty()
    assert client.written.wait(timeout=10)
    settle()
    relay.close()
    settle()
    assert client.writes == 1


def test_the_sync_rides_a_thread_that_already_exists() -> None:
    """The relay owns a presence/poll thread. It must not grow a third."""
    before = {t.name for t in threading.enumerate() if t.is_alive()}
    relay = make_relay("OneThread")
    try:
        new = {t.name for t in threading.enumerate() if t.is_alive()} - before
        assert not any("journal" in n for n in new), new
        # One long-lived background LOOP, and it is the one that was already
        # there. (`omnilink-mem-write` and `omnilink-pypi-check` are one-shot
        # workers that finish and go away.)
        loops = sorted(n for n in new if n.startswith("omnilink-")
                       and n not in ("omnilink-mem-write",
                                     "omnilink-pypi-check"))
        assert loops in ([], ["omnilink-presence"]), loops
    finally:
        relay.close()


def test_a_failed_read_never_overwrites_the_stored_conversation() -> None:
    """The write path falls back to "write my own view" when the read
    fails. On a journal-only sync "my own view" is the section ALONE --
    which would delete every stored turn of every earlier session."""
    client = CountingClient([U("an earlier session"), M("and its reply")])
    relay = make_relay("NoClobber")
    relay._client = client
    try:
        relay.journal.record("drive_forward", {}, ok=True, summary="a")
        client.read_fails = True
        relay._sync_journal_if_dirty()
        settle()
        assert client.writes == 0, "a blind write would have clobbered the row"
        assert client.memory == [U("an earlier session"), M("and its reply")]
        # ...and it is still dirty, so the next beat retries.
        client.read_fails = False
        relay._sync_journal_if_dirty()
        assert client.written.wait(timeout=10)
        rows = _journal_rows(wait_for_section(client))
        assert [r["summary"] for r in rows] == ["a"]
        texts = [" ".join(p["text"] for p in e["parts"])
                 for e in client.memory if not _is_journal(e)]
        assert "an earlier session" in texts
    finally:
        relay.close()


def test_memory_disabled_never_syncs() -> None:
    """No key means no relay and nothing to sync; memory off is the same
    bargain one level down. Neither may produce an error."""
    relay = make_relay("NoMemory", memory_enabled=False)
    client = CountingClient()
    relay._client = client
    try:
        relay.journal.record("drive_forward", {}, ok=True, summary="a")
        relay._sync_journal_if_dirty()
        settle()
        assert client.attempts == 0
    finally:
        relay.close()
    settle()
    assert client.attempts == 0


def test_a_failing_sync_is_non_fatal_and_does_not_spam(capsys) -> None:
    """Telemetry must never take the robot down, and a flaky link must not
    fill the log -- the same discipline as the presence beat."""
    relay = make_relay("Flaky")
    client = CountingClient()
    client.write_fails = True
    relay._client = client
    try:
        relay.journal.record("drive_forward", {}, ok=True, summary="a")
        for _ in range(4):
            relay._sync_journal_if_dirty()
            settle(0.2)
        assert client.attempts >= 2, "a failed sync must be retried"
        out = capsys.readouterr().out
        assert out.count("memory persist failed") <= 1, out
        # The robot is still answering.
        page = relay.journal.listing(limit=5)
        assert page["entries"][0]["tool"] == "drive_forward"
    finally:
        relay.close()
