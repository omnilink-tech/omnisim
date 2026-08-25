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

"""Offline natural-language intent router for the warehouse courier.

This is the no-OMNI_KEY fallback: a small, dependency-free regex router that
turns common operator phrasings into CourierBridge actions, so the demo works
the moment you open the world. The OmniLink LLM path (courier_tools.py) is the
upgrade — it handles open-ended, multi-step, reasoning-heavy requests. Both
drive the SAME bridge action surface.

Examples it understands:
  "go to bay B"                          -> goto bay-b
  "pick up the red package"              -> pick at bay-a (red is staged there)
  "deliver it to dock 2"                 -> deliver to dock-2
  "take the package from bay B to dock 2"-> route: pick bay-b, deliver dock-2
  "collect from bay A and bay C, then drop both at dock 3"
                                         -> route: pick bay-a, pick bay-c, deliver dock-3
  "return to the charging dock"          -> goto home
  "stop" / "reset" / "where are you" / "what bays are there"
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple

_NUM_WORD = {"one": 1, "two": 2, "three": 3, "1": 1, "2": 2, "3": 3}


class CourierIntent:
    def __init__(self, bridge) -> None:
        self.bridge = bridge
        self.alias: Dict[str, str] = {}      # alias text -> station name
        self.kind: Dict[str, str] = {s["name"]: s["kind"]
                                     for s in bridge.stations.values()}
        for s in bridge.stations.values():
            name = s["name"]
            self._add(name, name)
            self._add(name.replace("-", " "), name)
            self._add(name.replace("-", ""), name)
            if s.get("label"):
                self._add(s["label"].lower(), name)
            if s["kind"] == "pickup":
                short = name.split("-", 1)[-1]            # "a".."f"
                self._add(f"bay {short}", name)
                cn = s.get("color_name")
                if cn:
                    self._add(cn, name)
                    self._add(f"{cn} package", name)
                    self._add(f"{cn} box", name)
                    self._add(f"{cn} parcel", name)
                    self._add(f"{cn} one", name)
            elif s["kind"] == "dropoff":
                n = name.split("-", 1)[-1]
                for w, v in _NUM_WORD.items():
                    if v == int(n):
                        self._add(f"dock {w}", name)
            elif s["kind"] == "home":
                for a in ("home", "charging", "charger", "charging dock",
                          "base", "the base", "start"):
                    self._add(a, name)
        # longest aliases first so "bay a" wins over "a"
        self._aliases = sorted(self.alias.keys(), key=len, reverse=True)

    def _add(self, alias: str, name: str) -> None:
        a = alias.strip().lower()
        if a and a not in self.alias:
            self.alias[a] = name

    # ── matching ──────────────────────────────────────────────────
    def _find(self, s: str, kinds: Tuple[str, ...]) -> List[str]:
        """Stations (of the given kinds) mentioned in s, in order of first
        appearance, de-duplicated."""
        hits: List[Tuple[int, str]] = []
        claimed = [False] * len(s)
        for a in self._aliases:
            name = self.alias[a]
            if self.kind[name] not in kinds:
                continue
            for m in re.finditer(r"\b" + re.escape(a) + r"\b", s):
                i, j = m.start(), m.end()
                if any(claimed[i:j]):
                    continue
                for k in range(i, j):
                    claimed[k] = True
                hits.append((i, name))
        seen, out = set(), []
        for _, name in sorted(hits):
            if name not in seen:
                seen.add(name)
                out.append(name)
        return out

    # ── dispatch ──────────────────────────────────────────────────
    def dispatch(self, text: str) -> dict:
        s = (text or "").strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        if re.search(r"\b(stop|halt|freeze|abort|cancel)\b", s):
            self.bridge.act_stop()
            return self._r("Stopping and clearing the queue.", "stop", "ok", "halted")
        if re.search(r"\b(reset|start over|reset the demo)\b", s):
            self.bridge.act_reset()
            return self._r("Resetting the rover and packages to their start.",
                           "reset", "ok", "reset")
        if re.search(r"\b(status|state|where are you|where is|pose|telemetry|what.* carrying|carrying)\b", s):
            return self._status()
        if re.search(r"\b(list|what bays|which bays|stations|help|what can you|commands)\b", s):
            return self._list()
        if re.search(r"\b(go home|return.*(base|charg|home)|recharge|charge)\b", s):
            r = self.bridge.act_goto("home")
            return self._from_result(r, "Heading to the charging dock.", "goto_station")

        pick_verb = bool(re.search(r"\b(pick|grab|load|collect|fetch|get)\b", s))
        deliver_verb = bool(re.search(r"\b(deliver|drop|unload|place|put|bring|take|move|transport|carry)\b", s))
        go_verb = bool(re.search(r"\b(go|drive|goto|navigate|head|move)\b", s))

        bays = self._find(s, ("pickup",))
        docks = self._find(s, ("dropoff", "home"))

        # full courier run: some bays AND a dock mentioned
        if bays and docks and (deliver_verb or pick_verb or go_verb):
            dock = docks[-1]
            steps = [{"action": "pick", "station": b} for b in bays]
            steps.append({"action": "deliver", "station": dock})
            r = self.bridge.act_run_route(steps)
            if "error" in r:
                return self._r(r["error"], "run_route", "err", r["error"])
            summary = " + ".join(f"pick {b}" for b in bays) + f" -> {dock}"
            return self._r(
                f"On it: {summary}. I'll route through the aisles, load "
                f"{'the package' if len(bays) == 1 else f'{len(bays)} packages'}, "
                f"and deliver to {self._label(dock)}.",
                "run_route", "ok", summary)

        # pick one or more bays (no dock yet)
        if pick_verb and bays:
            if len(bays) == 1:
                r = self.bridge.act_pick(bays[0])
                return self._from_result(
                    r, f"Going to {self._label(bays[0])} to load the package.",
                    "pick_package")
            steps = [{"action": "pick", "station": b} for b in bays]
            r = self.bridge.act_run_route(steps)
            return self._from_result(
                r, f"Collecting from {', '.join(self._label(b) for b in bays)}.",
                "run_route")

        # deliver to a dock (whatever is on the deck)
        if deliver_verb and docks:
            r = self.bridge.act_deliver(docks[0])
            return self._from_result(
                r, f"Delivering to {self._label(docks[0])}.", "deliver_package")

        # plain navigation to one station
        target = (bays + docks)
        if target:
            r = self.bridge.act_goto(target[0])
            return self._from_result(
                r, f"Driving to {self._label(target[0])}.", "goto_station")

        return self._r(
            "I didn't catch a station. Try: \"take the package from bay B to "
            "dock 2\", \"pick up the red package\", \"deliver to dock 3\", "
            "\"go to bay E\", \"status\", or \"list stations\".", None, None, None)

    # ── helpers ───────────────────────────────────────────────────
    def _label(self, name: str) -> str:
        st = self.bridge.stations.get(name, {})
        return st.get("label", name)

    def _status(self) -> dict:
        st = self.bridge.get_state()
        carry = ", ".join(st["carrying"]) if st["carrying"] else "nothing"
        loc = st["at_station"] or f"({st['x']:+.1f}, {st['y']:+.1f})"
        msg = (f"At {loc}, mode={st['mode']}, carrying {carry}, "
               f"{st['queue']} step(s) queued. Last: {st['last_event']}.")
        return self._r(msg, "get_courier_state", "ok", st["mode"])

    def _list(self) -> dict:
        bays = self.bridge.station_names("pickup")
        docks = self.bridge.station_names("dropoff")
        lines = ["Pickup bays: " + ", ".join(
            f"{self._label(b)}" + (f" ({self.bridge.stations[b].get('color_name')})"
                                   if self.bridge.stations[b].get("color_name") else "")
            for b in bays)]
        lines.append("Docks: " + ", ".join(self._label(d) for d in docks))
        lines.append("Say e.g. \"take the package from bay B to dock 2\".")
        return self._r("\n".join(lines), "list_stations", "ok",
                       f"{len(bays)} bays / {len(docks)} docks")

    def _from_result(self, r: dict, ok_msg: str, tool: str) -> dict:
        if "error" in r:
            return self._r(r["error"], tool, "err", r["error"])
        eta = r.get("eta_s")
        msg = ok_msg + (f" (~{eta:.0f}s)" if eta else "")
        return self._r(msg, tool, "ok", r.get("op", "ok"))

    @staticmethod
    def _r(agent: str, tool: Optional[str], status: Optional[str],
           summary: Optional[str]) -> dict:
        tools = [(tool, status, summary)] if tool else []
        return {"agent": agent, "tools": tools}
