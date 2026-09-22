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

"""The edge connector runs BESIDE the bridge, one per machine.

A standing order the platform fires with no browser open dispatches its
tool calls down a websocket held by `omnilink.edge_connector`. That
connector has always existed and no bridge started it, so the order came
back "edge not connected" unless the operator was separately running a
second process by hand.

Three properties are pinned here, and all three are engine-free:

  * ONE PER MACHINE, claimed by a heartbeat lock file. The platform
    registry is `userId -> newest socket`, so a second connector REPLACES
    the first rather than joining it.
  * TOOLS-ONLY. `EDGE_CAPABILITIES` is pinned to `["tool"]` before the
    loop starts: an OmniKey is required for every OmniLink AI turn, and a
    connector advertising `chat` would be offering a way around that.
  * IT NEVER BLOCKS OR RAISES. Every refusal is a reported state.

`os.kill(pid, 0)` is deliberately not used anywhere in this design: on
Windows it TERMINATES the process rather than probing it.
"""

from __future__ import annotations

import json
import os
import time
import types

import pytest

from omnisim_bridges import relay as relay_mod


@pytest.fixture(autouse=True)
def isolated_lock(tmp_path, monkeypatch):
    """Per-test state dir AND a clean module global.

    ⚠️ Every test that starts a connector uses its OWN agent name. The
    worker thread releases the slot when `main()` returns, and a daemon
    thread from the previous test can still be on its way there -- if it
    carried the same name it would release THIS test's claim, and the two
    would race over one lock file. That is a test artefact, not a product
    race (`claim_edge_lock` refuses a second claim in one process), but it
    is a flaky one, so it is designed out rather than slept through.
    """
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setattr(relay_mod, "_edge_lock_owner", None, raising=False)
    yield tmp_path
    monkeypatch.setattr(relay_mod, "_edge_lock_owner", None, raising=False)
    try:
        os.remove(relay_mod.edge_lock_path())
    except OSError:
        pass


@pytest.fixture()
def fake_connector(monkeypatch):
    """Stand in for omnilink.edge_connector: records, never connects."""
    import omnilink

    started = {"count": 0, "capabilities": None}

    def _main():
        started["count"] += 1
        started["capabilities"] = list(module.EDGE_CAPABILITIES)
        return 0

    module = types.ModuleType("omnilink.edge_connector")
    module.EDGE_CAPABILITIES = ["chat", "tool"]
    module.main = _main
    monkeypatch.setitem(__import__("sys").modules,
                        "omnilink.edge_connector", module)
    monkeypatch.setattr(omnilink, "edge_connector", module, raising=False)
    return module, started


def wait_for(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


# ── The hatch and the preconditions ──────────────────────────────────

def test_the_hatch_opts_out(monkeypatch, isolated_lock) -> None:
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    monkeypatch.setenv("OMNI_KEY", "olink_live")
    status = relay_mod.start_edge_connector("A", "olink_live")
    assert status["state"] == "off"
    assert not os.path.exists(relay_mod.edge_lock_path()), (
        "an opted-out bridge must not claim the machine's slot")


def test_the_hatch_is_value_parsed(monkeypatch) -> None:
    """`=0` is off, and an UNSET variable means on. A presence-gated hatch
    would arm on `=0`."""
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    assert relay_mod._env_flag("OMNILINK_EDGE", True) is True
    for raw in ("0", "false", "no", "off"):
        monkeypatch.setenv("OMNILINK_EDGE", raw)
        assert relay_mod._env_flag("OMNILINK_EDGE", True) is False


def test_no_key_means_no_socket(monkeypatch) -> None:
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.delenv("OMNI_KEY", raising=False)
    status = relay_mod.start_edge_connector("A", "olink_whatever")
    assert status["state"] == "no_key"


def test_a_relay_built_with_a_different_key_does_not_connect(
        monkeypatch) -> None:
    """`edge_connector.main()` authenticates with the PROCESS's OMNI_KEY, so
    a relay holding another key would open a socket speaking for a
    different account. It is also what keeps offline tests, which pass a
    placeholder key, from opening a real one."""
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_real")
    status = relay_mod.start_edge_connector("A", "olink_test_placeholder")
    assert status["state"] == "no_key"
    assert "process OMNI_KEY" in status["detail"]


# ── One per machine ──────────────────────────────────────────────────

def test_the_first_claim_wins_and_the_second_is_refused(isolated_lock) -> None:
    assert relay_mod.claim_edge_lock("first") is True
    record = relay_mod.read_edge_lock()
    assert record["agent"] == "first" and record["pid"] == os.getpid()
    # A second relay IN THIS PROCESS (three bridges in one demo) must not
    # start a second connector either.
    assert relay_mod.claim_edge_lock("second") is False


def test_another_live_process_holds_the_slot(isolated_lock, monkeypatch,
                                             fake_connector) -> None:
    lock = relay_mod.edge_lock_path()
    os.makedirs(os.path.dirname(lock), exist_ok=True)
    with open(lock, "w", encoding="utf-8") as fh:
        json.dump({"pid": os.getpid() + 12345, "agent": "OmniSim-other",
                   "started_at": time.time(), "beat": time.time()}, fh)
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_live")

    status = relay_mod.start_edge_connector("edge-held-probe", "olink_live")

    assert status["state"] == "held_elsewhere"
    assert "OmniSim-other" in status["detail"]
    _, started = fake_connector
    assert started["count"] == 0


def test_a_stale_lock_is_taken_over(isolated_lock) -> None:
    """A killed engine leaves its lock behind. Without a staleness rule the
    machine would never run a connector again."""
    lock = relay_mod.edge_lock_path()
    os.makedirs(os.path.dirname(lock), exist_ok=True)
    dead = time.time() - relay_mod.EDGE_LOCK_STALE_S - 10
    with open(lock, "w", encoding="utf-8") as fh:
        json.dump({"pid": os.getpid() + 999, "agent": "ghost",
                   "started_at": dead, "beat": dead}, fh)
    assert relay_mod.claim_edge_lock("live") is True
    assert relay_mod.read_edge_lock()["agent"] == "live"


def test_liveness_is_the_heartbeat_not_the_pid(isolated_lock) -> None:
    fresh = {"pid": 1, "agent": "x", "beat": time.time()}
    stale = {"pid": 1, "agent": "x",
             "beat": time.time() - relay_mod.EDGE_LOCK_STALE_S - 1}
    assert relay_mod.edge_lock_is_live(fresh) is True
    assert relay_mod.edge_lock_is_live(stale) is False
    assert relay_mod.edge_lock_is_live(None) is False
    assert relay_mod.edge_lock_is_live({"pid": 1}) is False


def test_the_beat_refreshes_the_claim(isolated_lock) -> None:
    assert relay_mod.claim_edge_lock("beater") is True
    first = relay_mod.read_edge_lock()
    time.sleep(0.02)
    relay_mod.beat_edge_lock("beater")
    second = relay_mod.read_edge_lock()
    assert second["beat"] > first["beat"]
    assert second["started_at"] == first["started_at"], (
        "the beat must not restate when the connector started")


def test_a_non_owner_cannot_beat_or_release(isolated_lock) -> None:
    relay_mod.claim_edge_lock("owner")
    relay_mod.release_edge_lock("impostor")
    assert relay_mod.read_edge_lock()["agent"] == "owner"


def test_release_frees_the_slot_for_the_next_bridge(isolated_lock) -> None:
    assert relay_mod.claim_edge_lock("first") is True
    relay_mod.release_edge_lock("first")
    assert relay_mod.read_edge_lock() is None
    assert relay_mod.claim_edge_lock("second") is True


def test_a_corrupt_lock_file_does_not_wedge_the_machine(isolated_lock) -> None:
    lock = relay_mod.edge_lock_path()
    os.makedirs(os.path.dirname(lock), exist_ok=True)
    with open(lock, "w", encoding="utf-8") as fh:
        fh.write("}{ not json")
    assert relay_mod.read_edge_lock() is None
    assert relay_mod.claim_edge_lock("live") is True


# ── Tools-only, and started ──────────────────────────────────────────

def test_the_connector_starts_tools_only(monkeypatch, isolated_lock,
                                         fake_connector) -> None:
    module, started = fake_connector
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_live")

    status = relay_mod.start_edge_connector("edge-toolsonly-probe", "olink_live")

    assert status["state"] == "running"
    assert wait_for(lambda: started["count"] == 1), "the run loop never started"
    assert started["capabilities"] == ["tool"], (
        'the connector must not advertise "chat": an OmniKey is required for '
        "every AI turn and a local-inference offer would route around it")
    assert "tool" in module.EDGE_CAPABILITIES, (
        "dropping \"tool\" makes the server default-DENY every dispatch")


def test_the_worker_releases_the_slot_when_it_exits(monkeypatch, isolated_lock,
                                                    fake_connector) -> None:
    _, started = fake_connector
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_live")
    relay_mod.start_edge_connector("edge-release-probe", "olink_live")
    assert wait_for(lambda: started["count"] == 1)
    assert wait_for(lambda: relay_mod.read_edge_lock() is None), (
        "a connector that stopped must hand the slot back")


def test_a_connector_that_explodes_is_reported_not_raised(
        monkeypatch, isolated_lock) -> None:
    import omnilink
    module = types.ModuleType("omnilink.edge_connector")

    def _boom():
        raise RuntimeError("socket refused")

    module.EDGE_CAPABILITIES = ["chat", "tool"]
    module.main = _boom
    monkeypatch.setitem(__import__("sys").modules,
                        "omnilink.edge_connector", module)
    monkeypatch.setattr(omnilink, "edge_connector", module, raising=False)
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_live")

    status = relay_mod.start_edge_connector("edge-boom-probe", "olink_live")
    assert status["state"] == "running"       # the thread owns the failure
    assert wait_for(lambda: relay_mod.read_edge_lock() is None)


def test_a_missing_sdk_is_a_state_not_a_crash(monkeypatch,
                                              isolated_lock) -> None:
    import builtins
    real_import = builtins.__import__

    def _no_omnilink(name, *a, **k):
        if name.startswith("omnilink"):
            raise ImportError("no omnilink here")
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", _no_omnilink)
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)
    monkeypatch.setenv("OMNI_KEY", "olink_live")
    status = relay_mod.start_edge_connector("A", "olink_live")
    assert status["state"] == "unavailable"


# ── Doctor reads the same file ───────────────────────────────────────

def test_doctor_and_the_relay_agree_on_the_lock_path(isolated_lock) -> None:
    """The path convention is written twice on purpose -- doctor must answer
    on a clone where the bridges package does not import at all -- so the
    two copies are pinned against each other here. If one moves, this
    fails."""
    doctor = pytest.importorskip("omnisim.doctor")
    assert str(doctor._edge_lock_path()) == relay_mod.edge_lock_path()
    assert doctor.EDGE_LOCK_STALE_S == relay_mod.EDGE_LOCK_STALE_S


def test_doctor_reports_the_slot(isolated_lock, monkeypatch) -> None:
    doctor = pytest.importorskip("omnisim.doctor")
    monkeypatch.delenv("OMNILINK_EDGE", raising=False)

    assert doctor._edge_status()["state"] == "idle"

    relay_mod.claim_edge_lock("OmniSim-husky")
    connected = doctor._edge_status()
    assert connected["state"] == "connected"
    assert "OmniSim-husky" in connected["detail"]

    lock = relay_mod.edge_lock_path()
    with open(lock, "r", encoding="utf-8") as fh:
        record = json.load(fh)
    record["beat"] = time.time() - relay_mod.EDGE_LOCK_STALE_S - 5
    with open(lock, "w", encoding="utf-8") as fh:
        json.dump(record, fh)
    assert doctor._edge_status()["state"] == "stale"

    monkeypatch.setenv("OMNILINK_EDGE", "0")
    assert doctor._edge_status()["state"] == "off"


# ── The relay wires it in ────────────────────────────────────────────

def test_the_relay_records_its_edge_status(monkeypatch, isolated_lock) -> None:
    from omnisim_bridges.relay import OmniLinkRelay
    from omnisim_bridges.tool import Tool

    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="EdgeProbe", main_task="test",
        tools=[Tool("stop_robot", "halt", {"type": "object", "properties": {}},
                    lambda a: {"halted_at": 1})],
        usage_enabled=False, memory_enabled=False)
    try:
        assert relay.edge_status["state"] == "off"
    finally:
        relay.close()
