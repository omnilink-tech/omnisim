"""Bounded contact-physics sweeps and sampled failure-boundary refinement."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from itertools import product
import html
import json
from pathlib import Path
import shutil

from .registry import ROOT, profiles
from .runner import run, rerun, regrade, save
from .schema import BrakingSpec


def boundaries(cases):
    groups = {}
    for case in cases:
        if case["kind"] == "confirmation":
            continue
        s = case["spec"]
        groups.setdefault((s["robot"],s["slope_deg"],s["payload_kg"]),[]).append(case)
    result = []
    for key, items in sorted(groups.items()):
        items.sort(key=lambda x:x["spec"]["ground_mu"])
        for lo,hi in zip(items,items[1:]):
            if {lo["status"],hi["status"]} == {"PASS","FAIL"}:
                result.append({"robot":key[0],"slope_deg":key[1],"payload_kg":key[2],
                    "low_mu":lo["spec"]["ground_mu"],"high_mu":hi["spec"]["ground_mu"],
                    "low_case":lo["case"],"high_case":hi["case"],
                    "low_status":lo["status"],"high_status":hi["status"],
                    "both_braking_qualified":all(c["metrics"].get("braking_qualified",False) for c in (lo,hi)),
                    "failure_reasons":next(c["reasons"] for c in (lo,hi) if c["status"] == "FAIL")})
    return result


def report(directory, state):
    cells = []
    colors = {"PASS":"#daf1e4","FAIL":"#f8dadd","ERROR":"#ffe0a3","INCONCLUSIVE":"#eee"}
    for c in state["cases"]:
        s, m = c["spec"], c["metrics"]
        distance = m.get("max_forward_excursion_m")
        cells.append('<tr style="background:%s"><td><a href="%s/verdict.json">%s</a></td><td>%s</td><td>%g</td><td>%g</td><td>%g</td><td>%s</td><td>%s</td><td>%s</td></tr>' % (
            colors.get(c["status"],"white"),c["case"],c["case"],s["robot"],s["slope_deg"],s["ground_mu"],s["payload_kg"],
            c["status"],"—" if distance is None else f"{distance:.4f}",html.escape(", ".join(c["reasons"]))))
    lines = []
    for b in state["boundaries"]:
        lines.append(f'<li>{b["robot"]}, {b["slope_deg"]:g}° downhill, {b["payload_kg"]:g} kg: '
                     f'μ [{b["low_mu"]:.4f}, {b["high_mu"]:.4f}], {b["low_status"]} → {b["high_status"]}; '
                     f'{"qualified braking" if b["both_braking_qualified"] else "includes approach/hold failure"}. '
                     f'{html.escape(", ".join(b["failure_reasons"]))}</li>')
    content = f'''<!doctype html><meta charset="utf-8"><title>OmniSim mobility failure boundaries</title>
<style>body{{font:15px system-ui;margin:35px auto;max-width:1200px;color:#233243}}table{{border-collapse:collapse;width:100%}}td,th{{padding:9px;border:1px solid #ddd;text-align:left}}img{{max-width:100%}}</style>
<h1>Mobility assessment: measured failure boundaries</h1>
<p>Immediate braking throughout: no injected command delay. Positive slope means downhill.
μ is the imposed tyre/surface contact coefficient, verified on both sides in the compiled model.
Payload is physically mounted mass with a fixed joint, not loose cargo.
These are simulated models with a deterministic test controller, not vendor firmware or hardware certification.</p>
<p>All attempts are listed. A failure to acquire/hold speed is distinct from a braking-distance failure.
Boundaries bracket sampled transitions at fixed speed, slope and payload; untested regions and nonmonotonic behavior are not certified.</p>
<h2>Sampled transition brackets</h2><ul>{''.join(lines) or '<li>No pass/fail transition bracket in the sampled range.</li>'}</ul>
<h2>Cases</h2><table><tr><th>Case</th><th>Robot</th><th>Downhill °</th><th>μ</th><th>Payload kg</th><th>Outcome</th><th>Forward m after stop</th><th>Reason</th></tr>{''.join(cells)}</table>
<p><a href="campaign.json">Full campaign and confirmation checks</a>. Every case retains spec, adopted physics model, trace, sources and machine fingerprint.</p>'''
    (directory/"report.html").write_text(content,encoding="utf-8")


def sweep(directory, robots, slopes, frictions, payloads, refine=0, confirm=0, workers=1, max_episodes=120):
    directory = directory.resolve()
    available = profiles()
    names = list(available) if robots == ["all"] else robots
    if not names or any(n not in available for n in names):
        raise ValueError("UNSUPPORTED robot for physical-wheel braking; inspect assess robots")
    if not 1 <= workers <= 4 or refine < 0 or confirm < 0:
        raise ValueError("workers must be 1..4; refine/confirm budgets must be nonnegative")
    count = len(names)*len(slopes)*len(frictions)*len(payloads)
    if not count or count + refine + 2*confirm > max_episodes:
        raise ValueError("Sweep exceeds episode budget or has an empty axis")
    specs = []
    for robot, slope, mu, payload in product(names,slopes,frictions,payloads):
        profile = available[robot]
        speed = min(.6, profile.config["wheel_radius_m"]*profile.config["max_wheel_speed_radps"]*.7)
        # Values are experimental fixture choices, not payload ratings.
        height = .35 if robot == "husky" else max(.12,profile.config["wheel_radius_m"]*2.3)
        specs.append(BrakingSpec(schema_version=2,robot=robot,target_speed_m_s=speed,
                                slope_deg=slope,ground_mu=mu,payload_kg=payload,payload_height_m=height))
    directory.mkdir(parents=True,exist_ok=False)
    # Freeze implementation sources for later provenance without rewriting old runs.
    for base in (ROOT/"omnisim/assessment", ROOT/"projects/assessments/controllers"):
        for source in base.rglob("*.py"):
            target = directory/"source"/source.relative_to(ROOT)
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(source,target)
    state = {"schema_version":2,"complete":False,"kind":"structured_discovery_not_field_failure_rate",
             "command_delay_s":0,"axes":{"robots":names,"slopes_deg":slopes,"ground_mu":frictions,"payload_kg":payloads},
             "workers":workers,"max_episodes":max_episodes,"cases":[],"boundaries":[],"confirmations":[]}
    save(directory/"campaign.json",state)
    def execute(item):
        index,spec,kind,parent,capture = item
        name=f"case-{index:03d}"
        path=directory/name
        verdict=rerun(directory/parent,path,capture) if parent else run(spec,path,capture)
        return {"case":name,"spec":spec.to_dict(),"kind":kind,"parent":parent,
                "status":verdict["status"],"reasons":verdict["reasons"],"metrics":verdict["metrics"],
                "offline_regrade_matches":regrade(path)==verdict}
    def record(case):
        state["cases"].append(case)
        state["boundaries"]=boundaries(state["cases"])
        save(directory/"campaign.json",state)
        print(json.dumps({k:case[k] for k in ("case","status","spec","reasons","metrics")}),flush=True)
    jobs=[(i,s,"grid",None,False) for i,s in enumerate(specs)]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for case in pool.map(execute,jobs):
            record(case)
    # Only refine observed qualified braking transitions. This reports local
    # sample brackets; it makes no monotonicity or global-safety assumption.
    for _ in range(refine):
        options=[b for b in state["boundaries"] if b["both_braking_qualified"]]
        if not options:
            break
        b=max(options,key=lambda b:b["high_mu"]-b["low_mu"])
        lo=next(c for c in state["cases"] if c["case"]==b["low_case"])
        spec=replace(BrakingSpec.from_dict(lo["spec"]),ground_mu=(b["low_mu"]+b["high_mu"])/2)
        record(execute((len(state["cases"]),spec,"refinement",None,False)))
    chosen=next((b for b in state["boundaries"] if b["both_braking_qualified"]),None)
    if chosen:
        for parent in (chosen["low_case"],chosen["high_case"]):
            original=next(c for c in state["cases"] if c["case"]==parent)
            for repeat in range(confirm):
                spec=BrakingSpec.from_dict(original["spec"])
                case=execute((len(state["cases"]),spec,"confirmation",parent,repeat==0))
                record(case)
                state["confirmations"].append({"original":parent,"repeat":case["case"],
                    "same_outcome":case["status"]==original["status"],
                    "distance_difference_m":None if case["metrics"].get("max_forward_excursion_m") is None else
                    abs(case["metrics"]["max_forward_excursion_m"]-original["metrics"]["max_forward_excursion_m"])})
    state["complete"]=True
    state["all_evidence_valid"]=all(c["status"] in ("PASS","FAIL") and c["offline_regrade_matches"] for c in state["cases"])
    save(directory/"campaign.json",state)
    report(directory,state)
    return state
