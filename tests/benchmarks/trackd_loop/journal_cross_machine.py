#!/usr/bin/env python3
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

"""Does the action journal actually cross a machine? Against the REAL platform.

This is D7's live verification. No engine is needed and none is launched: the
journal's crossing is the relay talking to OmniLink's memory API, and memory
needs no model provider -- so this is measurable even while every model turn on
this account answers 402.

"Another machine" is simulated honestly: two relays, each with its OWN state
directory, so the second cannot read the first's local journal file. The only
channel between them is the platform. If the second relay restores the first's
entries, they came from OmniLink memory and nowhere else.

Side effects, stated: writes one memory row under a scratch agent name, then
clears it. No profile is created (profile_sync is not called here), so the
account's 25-agent profile limit is not touched.
"""
from __future__ import annotations

import os
import pathlib
import shutil
import sys
import tempfile
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3]
                       / "packages" / "omnisim-bridges" / "src"))

# Machine A and machine B get disjoint state dirs. Set BEFORE import, because
# action_journal reads OMNILINK_INTENT_STATE_DIR when it builds its path.
ROOT = tempfile.mkdtemp(prefix="trackd_journal_")
DIR_A = os.path.join(ROOT, "machineA")
DIR_B = os.path.join(ROOT, "machineB")
os.makedirs(DIR_A)
os.makedirs(DIR_B)

AGENT = "OmniSim-trackd-journalprobe"
os.environ["OMNILINK_EDGE"] = "0"          # no edge socket for a probe
os.environ["OMNILINK_PRESENCE"] = "0"      # no heartbeat row for a probe
os.environ["OMNILINK_EVENT_WAKE"] = "0"
os.environ["OMNILINK_USAGE"] = "0"

from omnisim_bridges import relay as R            # noqa: E402
from omnilink.client import OmniLinkClient        # noqa: E402

KEY = os.environ["OMNI_KEY"]


def make_relay(state_dir: str) -> "R.OmniLinkRelay":
    os.environ["OMNILINK_INTENT_STATE_DIR"] = state_dir
    return R.OmniLinkRelay(omni_key=KEY, agent_name=AGENT,
                           main_task="journal crossing probe", tools=[],
                           memory_enabled=True)


def history(relay) -> list:
    tool = relay.tools.get("get_action_history")
    out = tool.dispatch({"limit": 50}) if tool else {}
    return out.get("entries") or out.get("actions") or out.get("history") or []


def main() -> int:
    client = OmniLinkClient(omni_key=KEY)
    try:
        client.clear_memory(AGENT)                 # start from nothing
    except Exception:
        pass

    # ── machine A: act, then sync ─────────────────────────────────────
    a = make_relay(DIR_A)
    marks = [f"probe-{int(time.time())}-{i}" for i in range(3)]
    for i, m in enumerate(marks):
        a.journal.record("drive_forward", {"distance": 1.0 + i, "mark": m},
                         ok=True, summary=f"distance={1.0 + i} mark={m}",
                         result={"achieved": 1.0 + i, "settled": True})
    print(f"[A] recorded {len(marks)} entries; journal revision "
          f"{a.journal.revision()}; local file under {DIR_A}")
    # The production path: the dirty check the presence-thread beat runs.
    synced = a._sync_journal_if_dirty()
    print(f"[A] _sync_journal_if_dirty() -> {synced!r}")
    a.close()                                       # flushes if still dirty
    time.sleep(3)                                   # the async write lands

    stored = client.get_memory(AGENT) or []
    blob = repr(stored)
    on_platform = [m for m in marks if m in blob]
    print(f"[platform] memory rows: {len(stored)}; marks present: "
          f"{len(on_platform)}/{len(marks)}")

    # ── machine B: empty local state, same key, same agent ────────────
    files_b = os.listdir(DIR_B)
    print(f"[B] local state dir before boot: {files_b or 'EMPTY'}")
    b = make_relay(DIR_B)
    rows = history(b)
    seen = [m for m in marks if m in repr(rows)]
    print(f"[B] get_action_history returned {len(rows)} rows; "
          f"marks restored: {len(seen)}/{len(marks)}")
    b.close()

    # ── clean up the scratch memory row ───────────────────────────────
    try:
        client.clear_memory(AGENT)
        print("[cleanup] scratch memory row cleared")
    except Exception as e:
        print(f"[cleanup] could not clear: {e}")
    shutil.rmtree(ROOT, ignore_errors=True)

    ok = len(on_platform) == len(marks) and len(seen) == len(marks)
    print()
    print("=== VERDICT:", "CROSSES MACHINES" if ok else "DOES NOT CROSS",
          f"(platform {len(on_platform)}/{len(marks)}, restored {len(seen)}/{len(marks)})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
