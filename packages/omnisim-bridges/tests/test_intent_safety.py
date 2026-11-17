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

"""Three ways the intent layer could quietly betray an operator.

All three were found by an adversarial audit of the shipped build, and all
three are cases where the SYSTEM produced the falsehood -- not the model
guessing. That makes them worse than a hallucination: the words were faithful
and the mechanism did something else.
"""

from __future__ import annotations

import os
import tempfile

import pytest

from omnisim_bridges.intents import IntentStore, _minutes_to_ttl

RULES = {"no_park_row": "stay out of the cart park row"}


@pytest.fixture()
def store(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.delenv("OMNILINK_AGENT_TAG", raising=False)
    return IntentStore("tug_a", task_noun="delivery", task_plural="deliveries",
                       rules=RULES, log=lambda m: None)


def _fresh(tmp_path):
    return IntentStore("tug_a", task_noun="delivery", task_plural="deliveries",
                       rules=RULES, log=lambda m: None)


# ── 1. a restored commitment must be explicable ──────────────────────

def test_restored_note_reaches_state_not_just_listing(tmp_path, monkeypatch):
    """state() enumerates keys, so anything added to listing() is invisible
    here unless added twice -- and get_robot_state (the tool the model is told
    to always call) plus the Foreman's ask_robot both read /state. A robot that
    came back holding an order nobody gave it this session could not say why.
    """
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.delenv("OMNILINK_AGENT_TAG", raising=False)
    first = _fresh(tmp_path)
    first.schedule("pause", "after_n_deliveries", count=2, words="two more then stop")

    revived = _fresh(tmp_path)
    assert "restored_from_previous_run" in revived.listing()
    assert "restored_from_previous_run" in revived.state()


# ── 2. cancelling must name what it destroyed ────────────────────────

def test_cancel_refuses_an_id_it_does_not_hold(store) -> None:
    """The measured failure: the operator asked to cancel something that never
    existed, the model guessed an id, and the guess hit a LIVE promise. The
    whole reply was "Cancelled."
    """
    store.schedule("pause", "after_n_deliveries", count=2, words="stop after two more")
    out = store.cancel("intent-99")
    assert out["accepted"] is False
    assert "intent-1" in out["say"], "must say what it actually holds"
    assert len(store.listing()["pending_intents"]) == 1, "live promise survived"


def test_bare_cancel_with_several_pending_refuses_rather_than_dropping_all(store) -> None:
    store.schedule("pause", "after_n_deliveries", count=2, words="a")
    store.schedule("notify", "on_next_pickup", message="x", words="b")
    out = store.cancel(None)
    assert out["accepted"] is False
    assert len(store.listing()["pending_intents"]) == 2, "nothing may be dropped"


def test_successful_cancel_quotes_the_operator_back(store) -> None:
    """So a wrong guess is corrected in the next breath rather than discovered
    when the robot fails to stop."""
    rec = store.schedule("pause", "after_n_deliveries", count=2,
                         words="stop after two more deliveries")
    out = store.cancel(rec["id"])
    assert out["accepted"] is True
    assert "stop after two more deliveries" in out["say"]
    assert store.listing()["pending_intents"] == []


def test_bare_cancel_with_exactly_one_pending_still_works(store) -> None:
    """The refusal must not make the ordinary case annoying."""
    store.schedule("pause", "after_n_deliveries", count=2, words="a")
    assert store.cancel(None)["accepted"] is True


# ── 3. a promised duration must be the enforced duration ─────────────

@pytest.mark.parametrize("given,expected", [
    (10, 600.0),
    (0.5, 60.0),          # floored
    (100000, 12 * 3600),  # capped
    (None, None),
    (0, None),
    ("junk", None),
])
def test_minutes_to_ttl(given, expected) -> None:
    assert _minutes_to_ttl(given) == expected


def test_stated_duration_is_the_enforced_duration(store) -> None:
    """Measured: asked for ten minutes, the robot said ten minutes, and the
    store applied its thirty-minute default. The sentence was faithful; the
    mechanism was not."""
    store.set_constraint("no_park_row", words="ten minutes",
                         ttl_s=_minutes_to_ttl(10))
    remaining = store.listing()["constraints"][0]["expires_in_s"]
    assert 550 < remaining <= 600, f"expected ~600s, got {remaining}"
