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

"""match_director — referee + scorekeeper supervisor for BattleBox matches.

Owns three things the per-bot controllers can't:

  1. The clock. Hard 3-minute timer (configurable) — when it expires
     we stop the match and call a decision based on the scorecard.

  2. KO + OOTA + match-end detection. Watches every fighter's position
     and velocity each step. A fighter is OUT when:
       * its xy speed has been < KO_SPEED_M_S for KO_IMMOBILE_S seconds
         (knockout — "no controlled movement")
       * its z falls below the arena floor (out of the arena via pit)
       * its xy is outside the arena footprint by > MARGIN (driven over
         the wall — counts as OOTA)

  3. The judges' scorecard. Three categories, classic BattleBots
     judging weights:
       * damage (4 pts max)    — read from damage system HTTP API if
         present; otherwise estimated from impact events
       * aggression (3 pts max)  — forward velocity toward opponent
       * control (3 pts max)     — fraction of time held in opponent's
         half of the arena

The director writes its broadcast state into its OWN customData every
step so per-bot brains can poll us for `match_over` and `eliminated`
without needing an external bus. The final scorecard is written to the
path in customData['output'] (default: battlebox_match.json).

customData schema (set on the DEF DIRECTOR Robot in the world file):
  {
    "timer_s":       <float>,      # match length, default 180
    "mode":          "duel"|"rumble", default inferred
    "red":           <name>,       # duel: name of red fighter
    "blue":          <name>,       # duel: name of blue fighter
    "fighters":      [<name>, ...] # rumble: explicit list
    "arena_size":    <float>,      # square edge, default 8
    "oota_margin":   <float>,      # default 0.15
    "ko_immobile_s": <float>,      # default 10
    "ko_speed":      <float>,      # default 0.15
    "output":        <path>,
  }

Broadcast state (mirrored back into customData every step so brains
and the broadcast_director can read it):
  {
    ... original config ...,
    "t_s":           <float>,
    "match_over":    <bool>,
    "winner":        <name> | "draw" | null,
    "win_reason":    "ko"|"oota"|"decision"|"timeout",
    "eliminated":    [<name>, ...],
    "scorecard":     {<name>: {"damage": .., "aggression": .., "control": ..,
                               "total": ..}},
    "impact_events": <int>,
  }
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import defaultdict

from omnisim import Supervisor


DEFAULTS = {
    "timer_s": 180.0,
    "mode": "duel",
    "arena_size": 8.0,
    "oota_margin": 0.15,
    "ko_immobile_s": 10.0,
    "ko_speed": 0.15,
    "output": "battlebox_match.json",
    "score_weights": {"damage": 4.0, "aggression": 3.0, "control": 3.0},
    "impact_dv": 0.8,  # min own-Δv per step to count as a dealt-impact
}


def _xy_mag(v) -> float:
    return math.hypot(v[0], v[1])


def _parse_custom_data(raw: str) -> dict:
    cfg = dict(DEFAULTS)
    if not raw:
        return cfg
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        sys.stderr.write(f"[match_director] customData JSON invalid ({exc}); defaults\n")
        return cfg
    if not isinstance(parsed, dict):
        return cfg
    cfg.update(parsed)
    return cfg


def _scan_named_children(supervisor: Supervisor) -> dict:
    """{name -> node} for all root children that have a name field."""
    out = {}
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            try:
                node = children.getMFNode(i)
                nf = node.getField("name")
                if nf is None:
                    continue
                nm = nf.getSFString()
                if nm:
                    out[nm] = node
            except Exception:
                continue
    except Exception:
        pass
    return out


def _resolve_fighters(cfg: dict, scene: dict) -> list[str]:
    """Resolve the explicit fighter list from config."""
    if "fighters" in cfg and isinstance(cfg["fighters"], list):
        return [f for f in cfg["fighters"] if f in scene]
    fighters = []
    if "red" in cfg and cfg["red"] in scene:
        fighters.append(cfg["red"])
    if "blue" in cfg and cfg["blue"] in scene:
        fighters.append(cfg["blue"])
    return fighters


def _fetch_damage_api() -> dict | None:
    """Best-effort: pull /robot/damage from the local harness HTTP API.
    Returns None if the API isn't reachable so we fall back to estimated
    damage from impact-event count."""
    try:
        import urllib.request as _u
        with _u.urlopen("http://127.0.0.1:8001/robot/damage", timeout=0.1) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception:
        return None


def main() -> int:
    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())
    dt = basic_step_ms / 1000.0

    self_node = supervisor.getSelf()
    self_custom = self_node.getField("customData") if self_node else None
    raw_cfg = self_custom.getSFString() if self_custom else ""
    cfg = _parse_custom_data(raw_cfg)

    scene = _scan_named_children(supervisor)
    fighters = _resolve_fighters(cfg, scene)
    if len(fighters) < 2:
        sys.stderr.write(
            f"[match_director] need >=2 fighters; have {fighters}; idling\n"
        )
        # Still run so the world can be opened standalone for editing.
        while supervisor.step(basic_step_ms) != -1:
            pass
        return 0

    sys.stderr.write(
        f"[match_director] fighters={fighters} timer={cfg['timer_s']}s "
        f"arena={cfg['arena_size']}m mode={cfg['mode']}\n"
    )

    # Resolve fighter nodes once (we don't expect deletion mid-match).
    fnodes = {f: scene[f] for f in fighters}

    half = float(cfg["arena_size"]) * 0.5
    margin = float(cfg["oota_margin"])
    ko_immobile_s = float(cfg["ko_immobile_s"])
    ko_speed = float(cfg["ko_speed"])
    impact_dv = float(cfg["impact_dv"])
    weights = cfg["score_weights"]

    # Per-fighter state.
    slow_started: dict[str, float | None] = {f: None for f in fighters}
    eliminated: list[str] = []
    elim_reason: dict[str, str] = {}

    prev_speed: dict[str, float] = {f: 0.0 for f in fighters}
    impact_events: dict[str, int] = defaultdict(int)
    aggression_accum: dict[str, float] = defaultdict(float)
    control_accum: dict[str, float] = defaultdict(float)

    sim_t = 0.0
    match_over = False
    winner: str | None = None
    win_reason: str | None = None
    end_writeout_done = False

    def write_broadcast() -> None:
        if self_custom is None:
            return
        st = dict(cfg)
        st.update({
            "t_s": round(sim_t, 2),
            "match_over": match_over,
            "winner": winner,
            "win_reason": win_reason,
            "eliminated": list(eliminated),
            "impact_events": dict(impact_events),
        })
        if match_over:
            st["scorecard"] = scorecard()
        try:
            self_custom.setSFString(json.dumps(st))
        except Exception:
            pass

    def scorecard() -> dict:
        # Damage component: prefer live HTTP value if available.
        damage_api = _fetch_damage_api()
        max_impact = max(impact_events.values()) if impact_events else 1
        max_agg = max(aggression_accum.values()) if aggression_accum else 1.0
        max_ctrl = max(control_accum.values()) if control_accum else 1.0
        scores: dict[str, dict] = {}
        for f in fighters:
            if damage_api and isinstance(damage_api, dict) and f in damage_api:
                # Damage API returns 0..1 fractional HP loss
                dmg = float(damage_api[f].get("damage_dealt", 0.0))
                dmg_n = min(1.0, dmg)
            else:
                dmg_n = (impact_events.get(f, 0) / max_impact) if max_impact else 0.0
            agg_n = aggression_accum.get(f, 0.0) / max_agg if max_agg else 0.0
            ctrl_n = control_accum.get(f, 0.0) / max_ctrl if max_ctrl else 0.0
            d = dmg_n * weights["damage"]
            a = agg_n * weights["aggression"]
            c = ctrl_n * weights["control"]
            scores[f] = {
                "damage": round(d, 2),
                "aggression": round(a, 2),
                "control": round(c, 2),
                "total": round(d + a + c, 2),
            }
        return scores

    def declare_winner(reason: str, w: str | None) -> None:
        nonlocal match_over, winner, win_reason
        match_over = True
        winner = w
        win_reason = reason
        sys.stderr.write(
            f"[match_director] MATCH OVER at t={sim_t:.2f}s reason={reason} winner={w}\n"
        )

    while supervisor.step(basic_step_ms) != -1:
        sim_t += dt

        if not match_over:
            # Per-fighter sample.
            positions: dict[str, list] = {}
            speeds: dict[str, float] = {}
            for f in fighters:
                if f in eliminated:
                    continue
                node = fnodes[f]
                try:
                    pos = node.getPosition()
                    vel = node.getVelocity()
                except Exception:
                    continue
                # Physics-blow-up guard: NaN positions mean the backend
                # lost the body. Treat as instant elimination (cause:
                # nan) so we don't accumulate scores on dead state and
                # the match ends instead of ticking the full timer.
                if (math.isnan(pos[0]) or math.isnan(pos[1])
                        or math.isnan(pos[2])):
                    eliminated.append(f)
                    elim_reason[f] = "nan"
                    sys.stderr.write(
                        f"[match_director] {f} NaN-out at t={sim_t:.2f}s "
                        "(physics blow-up)\n"
                    )
                    continue

                positions[f] = pos
                speed = _xy_mag(vel)
                speeds[f] = speed

                # KO timer: slow for ko_immobile_s = OUT.
                if speed < ko_speed:
                    if slow_started[f] is None:
                        slow_started[f] = sim_t
                    elif (sim_t - slow_started[f]) >= ko_immobile_s:
                        eliminated.append(f)
                        elim_reason[f] = "ko"
                        sys.stderr.write(
                            f"[match_director] {f} KO at t={sim_t:.2f}s\n"
                        )
                        slow_started[f] = None
                        continue
                else:
                    slow_started[f] = None

                # OOTA: outside arena footprint OR below floor.
                if (abs(pos[0]) > half + margin or
                        abs(pos[1]) > half + margin or
                        pos[2] < -0.5):
                    eliminated.append(f)
                    elim_reason[f] = "oota"
                    sys.stderr.write(
                        f"[match_director] {f} OOTA at t={sim_t:.2f}s pos={pos}\n"
                    )
                    continue

                # Aggression: forward velocity component toward opponent.
                # For each other fighter, dot(vel_xy, unit_to_opp). Sum +ve.
                for g in fighters:
                    if g == f or g in eliminated or g not in fnodes:
                        continue
                    try:
                        gp = fnodes[g].getPosition()
                        ox = gp[0] - pos[0]
                        oy = gp[1] - pos[1]
                        d = math.hypot(ox, oy)
                        if d > 0.01:
                            ux, uy = ox / d, oy / d
                            dot = vel[0] * ux + vel[1] * uy
                            if dot > 0:
                                aggression_accum[f] += dot * dt
                    except Exception:
                        pass

                # Control: in opponent's half (x sign relative to opp).
                # Rumble: in the inner third of the arena.
                if cfg["mode"] == "rumble":
                    if math.hypot(pos[0], pos[1]) < half * 0.5:
                        control_accum[f] += dt
                else:
                    # Duel: red owns -x side, blue owns +x.
                    if f == cfg.get("red") and pos[0] > 0:
                        control_accum[f] += dt
                    elif f == cfg.get("blue") and pos[0] < 0:
                        control_accum[f] += dt

                # Impact event: own Δv per step above threshold.
                dv = abs(speed - prev_speed.get(f, 0.0))
                if dv > impact_dv:
                    impact_events[f] += 1
                prev_speed[f] = speed

            # Win conditions.
            survivors = [f for f in fighters if f not in eliminated]
            if len(survivors) == 1:
                last_reason = elim_reason.get(eliminated[-1], "ko") if eliminated else "ko"
                declare_winner(last_reason, survivors[0])
            elif len(survivors) == 0:
                declare_winner("ko", "draw")
            elif sim_t >= cfg["timer_s"]:
                sc = scorecard()
                best = max(survivors, key=lambda f: sc[f]["total"])
                # Draw if best tied with someone else within 0.1.
                others = [f for f in survivors if f != best and abs(sc[f]["total"] - sc[best]["total"]) < 0.1]
                if others:
                    declare_winner("decision", "draw")
                else:
                    declare_winner("decision" if cfg["timer_s"] - sim_t < 1.0 else "timeout", best)

        # Always mirror state back into our customData.
        write_broadcast()

        # On match end, write the final scorecard file once.
        if match_over and not end_writeout_done:
            out_path = cfg.get("output", DEFAULTS["output"])
            try:
                with open(out_path, "w", encoding="utf-8") as fh:
                    json.dump({
                        "match_t_s": round(sim_t, 2),
                        "winner": winner,
                        "win_reason": win_reason,
                        "eliminated": eliminated,
                        "elim_reason": elim_reason,
                        "scorecard": scorecard(),
                        "impact_events": dict(impact_events),
                        "config": {k: cfg[k] for k in cfg
                                   if k not in ("score_weights",)},
                    }, fh, indent=2)
                sys.stderr.write(f"[match_director] wrote {out_path}\n")
            except Exception as exc:
                sys.stderr.write(f"[match_director] writeout failed: {exc}\n")
            end_writeout_done = True

    return 0


if __name__ == "__main__":
    sys.exit(main())
