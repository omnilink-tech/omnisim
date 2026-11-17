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

"""combat_realism_report -- score a robot_combat match on collision-realism numbers.

Reads one or more JSONL traces written by battlebot_damage_director when it runs
under OMNISIM_DAMAGE_TRACE=<path>, and reports per trace:

  * bot-on-bot PENETRATION DEPTH (max / p95 / mean, metres) -- the number a
    viewer reads as "the bar went INTO the chassis";
  * HIT EVENTS (clusters of contact ticks), the first-hit time, contact time;
  * per event: the struck bot's peak speed change (m/s), the spinner's angular
    speed before and after, and the share of stored spin energy it gave up;
  * per bot: top speed, launch height above its rest height, peak tumble rate;
  * damage bookkeeping: HP deductions and, with --scorecard, the parts broken;
  * any non-finite value (a NaN anywhere is an automatic FAIL).

Two traces side by side (--labels before after) print a before/after table.

Usage:
  python scripts/dev/combat_realism_report.py TRACE [TRACE2 ...]
      [--labels L1 L2 ...] [--scorecard S1 S2 ...] [--bar-inertia I1 I2 ...]
      [--json OUT.json] [--md OUT.md]
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

EVENT_GAP_S = 0.15      # contact ticks closer than this belong to one hit event
PRE_WINDOW_S = 0.30     # look-back for the spinner's pre-hit angular speed
POST_WINDOW_S = 0.30    # look-ahead for its post-hit minimum


def _load(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _norm(v) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def _finite(x) -> bool:
    if isinstance(x, bool):
        return True
    if isinstance(x, (int, float)):
        return math.isfinite(x)
    if isinstance(x, dict):
        return all(_finite(v) for v in x.values())
    if isinstance(x, (list, tuple)):
        return all(_finite(v) for v in x)
    return True


def _pct(values, q) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return s[k]


def analyse(rows: list[dict], bar_inertia: float | None = None) -> dict:
    if not rows:
        return {"error": "empty trace"}
    ts = [r["t"] for r in rows]
    bots = sorted({n for r in rows for n in r.get("f", {})})
    nan = not all(_finite(r) for r in rows)

    contact_rows = [r for r in rows if r.get("c")]
    events: list[dict] = []
    for r in contact_rows:
        if events and (r["t"] - events[-1]["end"]) <= EVENT_GAP_S:
            events[-1]["end"] = r["t"]
            events[-1]["rows"].append(r)
        else:
            events.append({"start": r["t"], "end": r["t"], "rows": [r]})

    # Penetration depth, bot-on-bot only (the director gates on the pair match).
    depths_all: list[float] = []
    depths_kind = {"chassis": [], "weapon": [], "wheel": []}
    for r in contact_rows:
        for fd in r["f"].values():
            for pid, pd in fd.get("parts", {}).items():
                d = float(pd.get("d", 0.0))
                if d <= 0.0:
                    continue
                depths_all.append(d)
                kind = ("chassis" if pid == "chassis"
                        else "wheel" if pid.endswith("wheel") else "weapon")
                depths_kind[kind].append(d)

    per_bot = {}
    for b in bots:
        series = [(r["t"], r["f"][b]) for r in rows if b in r["f"]]
        if not series:
            continue
        speeds = [math.hypot(fd["v"][0], fd["v"][1]) for _, fd in series]
        zs = [fd["p"][2] for _, fd in series]
        settle = [fd["p"][2] for t, fd in series if 0.5 <= t <= 2.0]
        rest_z = statistics.median(settle) if settle else min(zs)
        tumble = max(_norm(fd["w"]) for _, fd in series)
        ww = [fd["ww"] for _, fd in series if "ww" in fd]
        per_bot[b] = {
            "top_speed_mps": round(max(speeds), 3),
            "rest_z_m": round(rest_z, 4),
            "min_z_m": round(min(zs), 4),
            "launch_m": round(max(zs) - rest_z, 4),
            "peak_tumble_radps": round(tumble, 3),
            "weapon_peak_radps": round(max(ww), 2) if ww else None,
        }

    ev_out = []
    for ev in events:
        t0, t1 = ev["start"], ev["end"]
        summary = {"t": round(t0, 3), "dur_s": round(t1 - t0, 3), "bots": {}}
        parts_hit = {b: set() for b in bots}
        for r in ev["rows"]:
            for b, fd in r["f"].items():
                parts_hit[b].update(fd.get("parts", {}).keys())
        for b in bots:
            window = [(r["t"], r["f"][b]) for r in rows
                      if b in r["f"] and (t0 - 0.10) <= r["t"] <= (t1 + 0.20)]
            vecs = [fd["v"] for _, fd in window]
            max_dv = 0.0
            for i in range(len(vecs)):
                for j in range(i + 1, len(vecs)):
                    d = _norm([vecs[i][k] - vecs[j][k] for k in range(3)])
                    if d > max_dv:
                        max_dv = d
            pre = [fd["ww"] for t, fd in
                   ((r["t"], r["f"][b]) for r in rows if b in r["f"])
                   if "ww" in fd and (t0 - PRE_WINDOW_S) <= t <= t0]
            post = [fd["ww"] for t, fd in
                    ((r["t"], r["f"][b]) for r in rows if b in r["f"])
                    if "ww" in fd and t0 <= t <= (t1 + POST_WINDOW_S)]
            zs = [fd["p"][2] for _, fd in window]
            entry = {
                "peak_dv_mps": round(max_dv, 3),
                "parts": sorted(parts_hit[b]),
                "peak_z_m": round(max(zs), 4) if zs else None,
            }
            if pre and post:
                w0, w1 = max(pre), min(post)
                entry["spin_before_radps"] = round(w0, 2)
                entry["spin_after_radps"] = round(w1, 2)
                if w0 > 1.0:
                    entry["spin_energy_given_up"] = round(
                        max(0.0, (w0 * w0 - w1 * w1) / (w0 * w0)), 3)
                    if bar_inertia:
                        entry["spin_energy_given_up_J"] = round(
                            0.5 * bar_inertia * max(0.0, w0 * w0 - w1 * w1), 1)
            summary["bots"][b] = entry
        ev_out.append(summary)

    hits = [h for r in rows for h in r.get("hits", [])]
    hits_by_bot = {}
    for h in hits:
        hb = hits_by_bot.setdefault(h["bot"], {"count": 0, "J": 0.0, "parts": set()})
        hb["count"] += 1
        hb["J"] += float(h["J"]) - float(h["thr"])
        hb["parts"].add(h["part"])
    for hb in hits_by_bot.values():
        hb["J"] = round(hb["J"], 2)
        hb["parts"] = sorted(hb["parts"])

    dts = sorted({round(b - a, 5) for a, b in zip(ts, ts[1:]) if b > a})
    dt = dts[0] if dts else None
    struck = max(
        ((e["bots"][b]["peak_dv_mps"], b, e["t"]) for e in ev_out for b in e["bots"]),
        default=(0.0, None, None))
    biggest_spin = max(
        ((e["bots"][b].get("spin_energy_given_up", 0.0), b, e["t"])
         for e in ev_out for b in e["bots"]), default=(0.0, None, None))

    return {
        "rows": len(rows),
        "duration_s": round(ts[-1], 3),
        "tick_s": dt,
        "nan": nan,
        "bots": bots,
        "contact_ticks": len(contact_rows),
        "contact_time_s": round(len(contact_rows) * (dt or 0.0), 3),
        "hit_events": len(ev_out),
        "first_hit_s": ev_out[0]["t"] if ev_out else None,
        "penetration_m": {
            "max": round(max(depths_all), 5) if depths_all else 0.0,
            "p95": round(_pct(depths_all, 0.95), 5) if depths_all else 0.0,
            "mean": round(statistics.fmean(depths_all), 5) if depths_all else 0.0,
            "samples": len(depths_all),
            "max_chassis": round(max(depths_kind["chassis"]), 5) if depths_kind["chassis"] else 0.0,
            "max_weapon": round(max(depths_kind["weapon"]), 5) if depths_kind["weapon"] else 0.0,
            "max_wheel": round(max(depths_kind["wheel"]), 5) if depths_kind["wheel"] else 0.0,
        },
        "biggest_struck_dv": {"mps": struck[0], "bot": struck[1], "t": struck[2]},
        "biggest_spin_energy_given_up": {
            "fraction": biggest_spin[0], "bot": biggest_spin[1], "t": biggest_spin[2]},
        "per_bot": per_bot,
        "events": ev_out,
        "hp_deductions": len(hits),
        "hits_by_bot": hits_by_bot,
    }


def _scorecard(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    d = json.loads(path.read_text(encoding="utf-8"))
    return {
        "winner": d.get("winner"),
        "win_reason": d.get("win_reason"),
        "match_t_s": d.get("match_t_s"),
        "broken": {f["name"]: f.get("broken_parts", []) for f in d.get("fighters", [])},
    }


def _fmt(x) -> str:
    if x is None:
        return "-"
    if isinstance(x, bool):
        return "yes" if x else "no"
    if isinstance(x, float):
        return f"{x:.4g}"
    return str(x)


def markdown(results: list[tuple[str, dict, dict]]) -> str:
    labels = [lab for lab, _, _ in results]
    lines = ["| metric | " + " | ".join(labels) + " |",
             "|---|" + "---|" * len(labels)]

    def row(name, getter):
        cells = []
        for _, a, s in results:
            try:
                cells.append(_fmt(getter(a, s)))
            except Exception:
                cells.append("-")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")

    row("trace duration (s)", lambda a, s: a["duration_s"])
    row("non-finite values", lambda a, s: a["nan"])
    row("hit events", lambda a, s: a["hit_events"])
    row("first hit (s)", lambda a, s: a["first_hit_s"])
    row("time in bot-on-bot contact (s)", lambda a, s: a["contact_time_s"])
    row("penetration max (mm)", lambda a, s: a["penetration_m"]["max"] * 1000)
    row("penetration p95 (mm)", lambda a, s: a["penetration_m"]["p95"] * 1000)
    row("penetration mean (mm)", lambda a, s: a["penetration_m"]["mean"] * 1000)
    row("chassis penetration max (mm)", lambda a, s: a["penetration_m"]["max_chassis"] * 1000)
    row("weapon penetration max (mm)", lambda a, s: a["penetration_m"]["max_weapon"] * 1000)
    row("biggest struck dv (m/s)", lambda a, s: a["biggest_struck_dv"]["mps"])
    row("  struck bot", lambda a, s: a["biggest_struck_dv"]["bot"])
    row("biggest spin energy given up in one hit", lambda a, s: a["biggest_spin_energy_given_up"]["fraction"])
    bots = sorted({b for _, a, _ in results for b in a.get("bots", [])})
    for b in bots:
        row(f"{b}: top speed (m/s)", lambda a, s, b=b: a["per_bot"][b]["top_speed_mps"])
        row(f"{b}: launch above rest (m)", lambda a, s, b=b: a["per_bot"][b]["launch_m"])
        row(f"{b}: peak tumble (rad/s)", lambda a, s, b=b: a["per_bot"][b]["peak_tumble_radps"])
        row(f"{b}: weapon peak (rad/s)", lambda a, s, b=b: a["per_bot"][b]["weapon_peak_radps"])
        row(f"{b}: HP deductions", lambda a, s, b=b: a["hits_by_bot"].get(b, {}).get("count", 0))
        row(f"{b}: parts broken", lambda a, s, b=b: ",".join(s.get("broken", {}).get(b, [])) or "none")
    row("winner", lambda a, s: s.get("winner"))
    row("win reason", lambda a, s: s.get("win_reason"))
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("traces", nargs="+", type=Path)
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--scorecard", nargs="*", type=Path, default=None)
    ap.add_argument("--bar-inertia", nargs="*", type=float, default=None,
                    help="spinner bar inertia (kg m^2) per trace, for energy in joules")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--md", type=Path, default=None)
    args = ap.parse_args(argv)

    labels = args.labels or [p.stem for p in args.traces]
    if len(labels) != len(args.traces):
        ap.error("--labels must match the number of traces")
    scorecards = list(args.scorecard or [])
    scorecards += [None] * (len(args.traces) - len(scorecards))
    inertias = list(args.bar_inertia or [])
    inertias += [None] * (len(args.traces) - len(inertias))

    results = []
    for path, lab, sc, inertia in zip(args.traces, labels, scorecards, inertias):
        if not path.exists():
            print(f"missing trace: {path}", file=sys.stderr)
            return 2
        results.append((lab, analyse(_load(path), inertia), _scorecard(sc)))

    md = markdown(results)
    print(md)
    for lab, a, _ in results:
        print()
        print(f"## {lab}: hit events")
        for e in a["events"]:
            print(json.dumps(e))
    if args.md:
        args.md.write_text(md + "\n", encoding="utf-8")
    if args.json:
        args.json.write_text(json.dumps(
            {lab: {"analysis": a, "scorecard": s} for lab, a, s in results},
            indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
