"""Local, dependency-free evidence reports from saved measurements."""
from __future__ import annotations

import html
import json
from pathlib import Path


def chart(cases):
    width, height = 900, 370
    left, top, w, h = 75, 35, 790, 260
    colors = ["#187b60", "#c83c45", "#d58424", "#236bb5", "#7554ad"]
    series = []
    for name, directory, verdict in cases:
        rows = [json.loads(x) for x in (directory / "trace.jsonl").read_text().splitlines()]
        req = next((r for r in rows if r["stop_requested"]), None)
        if req:
            coord = "along_m" if json.loads((directory / "spec.json").read_text()).get("schema_version",1) >= 2 else "x_m"
            series.append((name, [(r["t_s"] - req["t_s"], r[coord] - req[coord]) for r in rows if r["stop_requested"]], verdict))
    ymax = max([.35] + [p[1] * 1.2 for _, points, _ in series for p in points])
    xmax = max([2.5] + [p[0] for _, points, _ in series for p in points])
    def xy(t, d):
        return left + t / xmax * w, top + h - d / ymax * h
    content = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
               '<rect width="100%" height="100%" fill="#fff"/>',
               '<g font-family="Arial,sans-serif" font-size="13" fill="#233243">',
               '<text x="75" y="22" font-size="17">Measured forward travel after STOP request</text>']
    for i in range(6):
        d = ymax * i / 5
        x, y = xy(0, d)
        content += [f'<path d="M{x},{y}h{w}" stroke="#e2e8ee"/>', f'<text x="15" y="{y+5:.1f}">{d:.2f} m</text>']
    for t in [0, .5, 1, 1.5, 2, 2.5]:
        x, y = xy(t, 0)
        content.append(f'<text x="{x-8:.1f}" y="{y+20:.1f}">{t:g}</text>')
    thresholds = {json.loads((d / "spec.json").read_text())["max_stop_distance_m"] for _, d, _ in cases}
    for threshold in thresholds:
        x, y = xy(0, threshold)
        content += [f'<path d="M{x},{y}h{w}" stroke="#333" stroke-dasharray="6 5"/>',
                    f'<text x="{left+w-155}" y="{y-7:.1f}">Limit {threshold:.2f} m</text>']
    for i, (name, points, verdict) in enumerate(series):
        c = colors[i % len(colors)]
        points_text = " ".join(f"{xy(t,d)[0]:.2f},{xy(t,d)[1]:.2f}" for t, d in points)
        content += [f'<polyline points="{points_text}" fill="none" stroke="{c}" stroke-width="2.3"/>',
                    f'<text x="{75+(i%3)*260}" y="{340+(i//3)*18}" fill="{c}">{html.escape(name)}: {verdict["status"]}</text>']
    content += ['<text x="340" y="328">Seconds since stop request (engine clock)</text>', '</g></svg>']
    return "\n".join(content)


def write_report(directory: Path, cases, title="Husky braking assessment"):
    (directory / "braking.svg").write_text(chart(cases), encoding="utf-8")
    rows = []
    for name, path, verdict in cases:
        m = verdict.get("metrics", {})
        rel = path.relative_to(directory).as_posix() if path != directory else "."
        def value(key):
            v = m.get(key)
            return "unavailable" if v is None else f"{v:.4f}"
        rows.append(f'<tr><td><a href="{rel}/verdict.json">{html.escape(name)}</a></td><td>{verdict["status"]}</td>'
                    f'<td>{value("initial_speed_m_s")}</td><td>{value("max_forward_excursion_m")}</td>'
                    f'<td>{value("stop_time_s")}</td><td>{value("observed_command_delay_s")}</td>'
                    f'<td>{html.escape(", ".join(verdict["reasons"]))}</td></tr>')
    document = f'''<!doctype html><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>body{{font:16px system-ui;margin:40px auto;max-width:1100px;color:#233243;background:#f7f9fb}}table{{border-collapse:collapse;background:white;width:100%}}td,th{{padding:12px;border-bottom:1px solid #ddd;text-align:left}}img{{max-width:100%}}a{{color:#236bb5}}</style>
<h1>{html.escape(title)}</h1><p>Measured in OmniSim using the configured robot model and a dedicated deterministic wheel controller.
Simulation only. Each spec records robot, surface, incline, mounted payload and any controller delay;
these findings do not establish vendor hardware or firmware performance.
The requirement is frozen in each spec.json. Verdicts are recomputable from the saved per-step traces.</p>
<img src="braking.svg" alt="Measured forward travel after stop request">
<table><tr><th>Case</th><th>Verdict</th><th>Initial m/s</th><th>Forward m</th><th>Stop s</th><th>Command delay s</th><th>Reasons</th></tr>{''.join(rows)}</table>
<p>Cases link to verdict.json. Each case also contains spec.json, manifest.json, commands.jsonl, trace.jsonl,
completion.json and the engine log/Newton sidecar. Missing observations produce INCONCLUSIVE, never PASS.
Compare cases with identical requirements and speed; confirmation at another speed is a separate condition.</p>'''
    (directory / "report.html").write_text(document, encoding="utf-8")
