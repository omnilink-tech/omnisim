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

"""Memory must not be destroyed at the source of truth.

The bridge used to persist with ``set_memory(agent, history[-40:])`` -- a
whole-blob overwrite of the platform's copy with just the context window. Two
measured consequences:

* Turns did not merely fall out of context, they were DELETED. Facts seeded
  early in a session were gone from the platform blob, not just un-recalled.
* Work from another surface was destroyed. The server appends to its own
  ceiling; a bridge then wrote its 40 over the top, so a long headless
  conversation vanished the next time a robot persisted.

These tests pin the properties that stop both.
"""

from __future__ import annotations

from omnisim_bridges.relay import _entry_key, _merge_memory, _to_memory_format


def U(text: str):
    return {"role": "user", "parts": [{"text": text}]}


def M(text: str):
    return {"role": "model", "parts": [{"text": text}]}


def texts(entries):
    return [" ".join(p.get("text", "") for p in e.get("parts", [])) for e in entries]


def test_stored_history_survives_a_small_window_write() -> None:
    """The regression, stated directly: a two-message window must not wipe
    everything the platform already holds."""
    stored = [U("my badge is 8813"), M("noted"),
              U("pallet TROLLEY_E is damaged"), M("understood")]
    merged = _merge_memory(stored, [U("drive forward"), M("moving")], 100)
    got = texts(merged)
    assert "my badge is 8813" in got
    assert "pallet TROLLEY_E is damaged" in got
    assert got[-1] == "moving", "the new turn should land at the end"
    assert len(merged) == 6


def test_another_surfaces_turns_are_not_destroyed() -> None:
    stored = [U("headless api turn"), M("headless reply")]
    merged = _merge_memory(stored, [U("sim turn"), M("sim reply")], 100)
    assert "headless api turn" in texts(merged)


def test_overlapping_window_does_not_duplicate() -> None:
    """The common case: our window overlaps what is stored. Re-sending the
    same turns must be a no-op, not a doubling of history on every write."""
    stored = [U("a"), M("b"), U("c"), M("d")]
    mine = [U("c"), M("d"), U("e")]          # two overlap, one is new
    merged = _merge_memory(stored, mine, 100)
    assert texts(merged) == ["a", "b", "c", "d", "e"]


def test_cap_trims_oldest_and_keeps_what_just_happened() -> None:
    stored = [U(f"turn{i}") for i in range(120)]
    merged = _merge_memory(stored, [U("just happened")], 100)
    assert len(merged) == 100
    assert texts(merged)[-1] == "just happened"
    assert "turn0" not in texts(merged), "the cap must bite on ancient history"


def test_merge_is_idempotent() -> None:
    """Persisting twice with no new turns must not change anything -- the
    write happens after every prompt, so a non-idempotent merge would grow
    the blob without bound."""
    stored = [U("a"), M("b")]
    once = _merge_memory(stored, [U("a"), M("b")], 100)
    twice = _merge_memory(once, [U("a"), M("b")], 100)
    assert texts(once) == texts(twice) == ["a", "b"]


def test_tolerates_junk_from_either_side() -> None:
    """A malformed stored blob must not take out the write path."""
    assert _merge_memory(None, [U("x")], 100) == [U("x")]
    assert texts(_merge_memory(["not a dict", U("a")], [U("b")], 100)) == ["a", "b"]
    assert _merge_memory([U("a")], None, 100) == [U("a")]


def test_role_is_part_of_identity() -> None:
    """Same words from the operator and the robot are different turns."""
    assert _entry_key(U("stop")) != _entry_key(M("stop"))
    merged = _merge_memory([U("stop")], [M("stop")], 100)
    assert len(merged) == 2


def test_tool_scaffolding_is_not_persisted() -> None:
    """Only human-readable turns are stored; tool traffic stays transient."""
    history = [
        {"role": "user", "content": "drive forward"},
        {"role": "assistant", "content": "", "tool_calls": [{"id": "1"}]},
        {"role": "tool", "tool_call_id": "1", "content": "{}"},
        {"role": "assistant", "content": "moving"},
    ]
    conv = _to_memory_format(history)
    assert texts(conv) == ["drive forward", "moving"]
    assert all(e["role"] in ("user", "model") for e in conv)


# ── The standing-notes tier ──────────────────────────────────────────
#
# A window is still a window: past the cap something must go. What was wrong
# was that the OPERATOR'S STATEMENTS went with it, so an agent told a standing
# fact an hour ago simply no longer knew it. Evicted user turns are now folded
# into one pinned entry.

from omnisim_bridges.relay import _is_notes, _notes_lines


def test_operator_facts_survive_eviction() -> None:
    limit = 20
    blob = _merge_memory([], [U("my badge is 8813"), M("noted"),
                              U("TROLLEY_E is damaged"), M("understood")], limit)
    for i in range(60):                       # bury them far past the cap
        blob = _merge_memory(blob, [U(f"chatter {i}"), M("ok")], limit)

    assert _is_notes(blob[0]), "notes must be pinned at the front"
    notes = _notes_lines(blob[0])
    assert any("8813" in ln for ln in notes)
    assert any("TROLLEY_E" in ln for ln in notes)


def test_head_is_preserved_against_later_chatter() -> None:
    """Pure FIFO would let small talk evict the setup facts, which is the one
    thing this tier exists to prevent."""
    limit = 20
    blob = _merge_memory([], [U("my badge is 8813")], limit)
    for i in range(200):
        blob = _merge_memory(blob, [U(f"chatter {i}"), M("ok")], limit)
    assert any("8813" in ln for ln in _notes_lines(blob[0]))


def test_model_replies_are_not_folded_into_notes() -> None:
    """Only the operator's words. Re-injecting the robot's own past claims is
    how a stale assertion about the world becomes durable context."""
    limit = 8
    blob = _merge_memory([], [U("a fact"), M("I have parked four carts")], limit)
    for i in range(20):
        blob = _merge_memory(blob, [U(f"t{i}"), M("parked four carts")], limit)
    notes = _notes_lines(blob[0])
    assert not any("parked four carts" in ln for ln in notes)


def test_notes_never_duplicate_or_grow_without_bound() -> None:
    limit = 10
    blob = _merge_memory([], [U("fact one")], limit)
    for i in range(300):
        blob = _merge_memory(blob, [U(f"x{i}"), M("ok")], limit)
    notes = _notes_lines(blob[0])
    assert len(notes) == len(set(notes)), "a repeated line would grow forever"
    assert len(notes) <= 40
    assert len(blob) <= limit, "the pinned entry must fit inside the cap"


def test_notes_survive_a_round_trip_without_reprocessing() -> None:
    """Re-merging a blob that already has notes must not nest or drop them."""
    limit = 10
    blob = _merge_memory([], [U("keep me")], limit)
    for i in range(30):
        blob = _merge_memory(blob, [U(f"y{i}"), M("ok")], limit)
    once = _notes_lines(blob[0])
    again = _merge_memory(blob, [], limit)
    assert _is_notes(again[0])
    assert _notes_lines(again[0]) == once


def test_restore_keeps_the_notes_inside_a_smaller_window() -> None:
    """The blob is deliberately larger than the context window, so the pinned
    entry sits outside a naive ``[-limit:]`` slice. Getting this wrong makes
    the whole tier look implemented and do nothing: facts are persisted
    faithfully and then dropped on the way back in.
    """
    from omnisim_bridges.relay import _restore_history

    persist, window = 100, 40
    blob = _merge_memory([], [U("my badge is 8813"), M("noted that")], persist)
    for i in range(90):                       # distinct replies -> real growth
        blob = _merge_memory(blob, [U(f"c{i}"), M(f"reply {i}")], persist)

    assert len(blob) == persist and _is_notes(blob[0])

    history = _restore_history(blob, window)
    assert len(history) == window
    assert "STANDING NOTES" in history[0]["content"]
    assert any("8813" in m["content"] for m in history), "fact lost on restore"
    assert history[-1]["content"] == "reply 89", "newest turn lost on restore"


def test_restore_without_notes_is_unchanged() -> None:
    """A blob from before this change (no pinned entry) must restore exactly
    as it always did."""
    from omnisim_bridges.relay import _restore_history

    blob = [U(f"t{i}") for i in range(60)]
    history = _restore_history(blob, 40)
    assert len(history) == 40
    assert history[-1]["content"] == "t59"
    assert history[0]["content"] == "t20"


def test_persist_limit_stays_under_the_server_compaction_trigger() -> None:
    """The two writers must not undo each other.

    The server compacts (and PERSISTS the compacted form) once a conversation
    passes `maxConversationEntries - 20` = 80. A bridge writing 100 therefore
    guarantees the next server turn rewrites the blob down to ~21 entries,
    which the bridge then re-expands -- an endless correction loop in which
    the server's recap is thrown away every time. Staying at or under the
    trigger means neither side has to fix the other.
    """
    from omnisim_bridges.relay import PERSIST_LIMIT, HISTORY_LIMIT

    SERVER_COMPACTION_CEILING = 80
    assert PERSIST_LIMIT <= SERVER_COMPACTION_CEILING, (
        f"PERSIST_LIMIT={PERSIST_LIMIT} exceeds the server's compaction "
        f"trigger ({SERVER_COMPACTION_CEILING}); every write would be "
        "rewritten server-side on the next turn"
    )
    assert PERSIST_LIMIT > HISTORY_LIMIT, (
        "storing no more than we send defeats the point -- that was the "
        "original bug"
    )


# ── Operator-designated durable facts ────────────────────────────────
#
# Measured on the heuristic tier alone: 60 standing facts separated by 600
# turns of chatter left 5 retained. That is the ceiling of the approach --
# it keeps the operator's words but cannot tell "TROLLEY_E is damaged" from
# "status check 4", so noise wins on volume. The missing capability is
# SELECTION, so the agent marks a statement durable when one is made.

from omnisim_bridges.relay import PIN_PREFIX, PINNED_MAX


def test_pinned_facts_survive_sustained_chatter() -> None:
    facts = [f"standing fact {i}" for i in range(20)]
    blob, pins = [], []
    for i, f in enumerate(facts):
        pins.append(f)
        blob = _merge_memory(blob, [U(f), M(f"ack {i}")], 80, pins)
        for j in range(30):
            blob = _merge_memory(blob, [U(f"noise {i}-{j}"), M(f"r{i}-{j}")], 80, pins)

    notes = _notes_lines(blob[0])
    for f in facts:
        assert PIN_PREFIX + f in notes, f"pinned fact lost: {f}"
    assert len(blob) <= 80


def test_pins_are_deduped_across_repeated_persists() -> None:
    """Persist runs after every prompt and passes the same pin list each
    time; duplicating them would grow the notes without bound."""
    blob = []
    for _ in range(25):
        blob = _merge_memory(blob, [U("chat"), M("ok")], 80, ["one fact"])
    notes = _notes_lines(blob[0])
    assert notes.count(PIN_PREFIX + "one fact") == 1


def test_pins_come_first_so_a_char_cut_reaches_only_chatter() -> None:
    blob = _merge_memory([], [U("x")], 20, ["critical fact"])
    for i in range(200):
        blob = _merge_memory(blob, [U(f"n{i}"), M("ok")], 20, ["critical fact"])
    notes = _notes_lines(blob[0])
    assert notes[0] == PIN_PREFIX + "critical fact"


def test_pin_count_is_bounded() -> None:
    pins = [f"fact {i}" for i in range(PINNED_MAX + 15)]
    blob = _merge_memory([], [U("x")], 80, pins)
    pinned = [ln for ln in _notes_lines(blob[0]) if ln.startswith(PIN_PREFIX)]
    assert len(pinned) <= PINNED_MAX
