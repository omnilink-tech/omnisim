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

"""Build and accept a frozen, offline Windows robot assessment delivery.

Explicit file selection keeps credentials, other robots and developer tools out
of the shipment. The integrity manifest is a checksum inventory, not a signature.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import html
import importlib.metadata
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import struct
import subprocess
import sys
import zipfile

from .registry import ROOT
from .runner import digest, read_json, regrade, rerun, runtime_hashes, save, source_hashes
from .schema import BrakingSpec
from .package_profiles import VERSION, SLOPES, FRICTIONS, DEFINITIONS, get_package_profile, render_document

MAX_ROOT_CHARS = 100  # Keeps all shipped paths below the engine's Windows path limit.


def validated_campaign(directory, current_sources=True, robot=None):
    """Reject incomplete, edited, wrong-scope or mismatched-source shipments."""
    state = read_json(directory / "campaign.json")
    if not state or not state.get("complete") or not state.get("all_evidence_valid"):
        raise ValueError("Delivery needs a complete campaign with valid evidence")
    names=state.get("axes",{}).get("robots",[])
    if len(names) != 1 or (robot is not None and names != [robot]):
        raise ValueError("Package requires one matching robot; mixed-robot campaigns cannot be delivered")
    profile=get_package_profile(names[0])
    if state.get("axes") != {"robots": [profile.robot], "slopes_deg": list(SLOPES),
                             "ground_mu": list(FRICTIONS), "payload_kg": list(profile.payloads)}:
        raise ValueError("Campaign axes differ from the selected robot package scope")
    if "package_profile" in state and state["package_profile"] != profile.to_dict():
        raise ValueError("Recorded package profile differs from the current definition")
    cases = state.get("cases", [])
    if Counter(c["kind"] for c in cases) != {"grid": 48, "refinement": 12, "confirmation": 4}:
        raise ValueError("Delivery requires 48 grid + 12 refinement + 4 confirmation episodes")
    names = [c["case"] for c in cases]
    if len(set(names)) != len(names) or any(Path(n).name != n or n in (".", "..") for n in names):
        raise ValueError("Invalid or duplicate case names")
    grid = set()
    first = read_json(directory / names[0] / "manifest.json")
    current = source_hashes(profile.robot) if current_sources else None
    for c in cases:
        path = directory / c["case"]
        spec = BrakingSpec.from_dict(c["spec"])
        expected = profile.spec(slope_deg=spec.slope_deg,ground_mu=spec.ground_mu,payload_kg=spec.payload_kg)
        if spec != expected or spec.brake_delay_s != 0:
            raise ValueError(f"Case {c['case']} differs from frozen task requirements")
        if spec.slope_deg not in SLOPES or spec.payload_kg not in profile.payloads or not min(FRICTIONS)<=spec.ground_mu<=max(FRICTIONS):
            raise ValueError("Case lies outside the selected package domain")
        if c["kind"] == "grid":
            grid.add((spec.slope_deg, spec.ground_mu, spec.payload_kg))
        if read_json(path / "spec.json") != c["spec"]:
            raise ValueError(f"Case {c['case']} spec disagrees with campaign")
        verdict = regrade(path)
        if verdict != read_json(path / "verdict.json") or any(
                verdict[k] != c[k] for k in ("status", "metrics", "reasons")):
            raise ValueError(f"Case {c['case']} does not match its offline regrade")
        if verdict["status"] not in ("PASS", "FAIL"):
            raise ValueError(f"Invalid evidence in {c['case']}")
        m = read_json(path / "manifest.json")
        if not m.get("artifacts_sha256") or any(m.get(k) != first.get(k) for k in
                ("sources_sha256", "binary_sha256", "runtime_sha256")):
            raise ValueError("Campaign provenance is missing or inconsistent")
        if current_sources and m["sources_sha256"] != current:
            raise ValueError("Campaign sources differ from delivery implementation; run a fresh campaign")
    if grid != {(s, f, p) for s in SLOPES for f in FRICTIONS for p in profile.payloads}:
        raise ValueError("Grid conditions are missing or duplicated")
    from .campaign import boundaries
    if state["boundaries"] != boundaries(cases):
        raise ValueError("Recorded boundaries disagree with cases")
    confirmations = state.get("confirmations", [])
    repeats = {c["case"]: c for c in cases if c["kind"] == "confirmation"}
    if len(confirmations) != 4 or {c["repeat"] for c in confirmations} != set(repeats):
        raise ValueError("Confirmation relationships missing")
    by_name = {c["case"]: c for c in cases}
    parents=Counter(c["original"] for c in confirmations)
    if len(parents)!=2 or sorted(parents.values()) != [2,2] or {by_name.get(n,{}).get("status") for n in parents}!={"PASS","FAIL"}:
        raise ValueError("Acceptance requires two repeats each of a passing and failing pair")
    if not any(b["both_braking_qualified"] and {b["low_case"],b["high_case"]}==set(parents) for b in state["boundaries"]):
        raise ValueError("Confirmed parents do not form a qualified braking transition")
    for check in confirmations:
        repeat = repeats[check["repeat"]]
        parent = by_name.get(check["original"])
        if not parent or repeat["parent"] != parent["case"] or repeat["spec"] != parent["spec"]:
            raise ValueError("Confirmation does not reproduce its recorded parent")
        distance = abs(repeat["metrics"]["max_forward_excursion_m"] - parent["metrics"]["max_forward_excursion_m"])
        if repeat["status"] != parent["status"] or distance > .005 or not check["same_outcome"]:
            raise ValueError("Confirmation exceeds outcome / 5 mm distance tolerance")
    return state, first


def campaign_profile(state):
    names=state.get("axes",{}).get("robots",[])
    if len(names)!=1: raise ValueError("Expected a single robot package")
    return get_package_profile(names[0])


def acceptance_pair(state):
    parents={c["original"] for c in state.get("confirmations",[])}
    return next((b for b in state["boundaries"] if b["both_braking_qualified"]
                 and (not parents or {b["low_case"],b["high_case"]}==parents)),None)


def pe_imports(path):
    """Read normal and delay-load PE imports without an external developer tool."""
    data = path.read_bytes()
    if data[:2] != b"MZ":
        return set()
    pe = struct.unpack_from("<I", data, 0x3c)[0]
    if data[pe:pe+4] != b"PE\0\0":
        raise ValueError(f"Invalid PE file: {path}")
    sections = struct.unpack_from("<H", data, pe+6)[0]
    opt_size = struct.unpack_from("<H", data, pe+20)[0]
    opt = pe+24
    plus = struct.unpack_from("<H", data, opt)[0] == 0x20b
    directories = opt + (112 if plus else 96)
    table = opt + opt_size
    ranges = [struct.unpack_from("<IIII", data, table+i*40+8) for i in range(sections)]
    def offset(rva):
        for virtual_size, virtual, raw_size, raw in ranges:
            if virtual <= rva < virtual + max(virtual_size, raw_size):
                return raw+rva-virtual
        raise ValueError(f"Unmapped PE RVA in {path}")
    def string(rva):
        pos=offset(rva)
        return data[pos:data.index(b"\0",pos)].decode("ascii").lower()
    result=set()
    for index, stride, name_at in ((1,20,12),(13,32,4)):
        rva, size=struct.unpack_from("<II",data,directories+index*8)
        if not rva: continue
        pos=offset(rva)
        for start in range(pos,pos+size,stride):
            row=data[start:start+stride]
            if not any(row): break
            # Modern PE delay-load descriptors use RVAs (attribute bit 0).
            if index == 13 and not (struct.unpack_from("<I",row)[0] & 1):
                raise ValueError(f"Unsupported old delay import: {path}")
            name=struct.unpack_from("<I",row,name_at)[0]
            if name: result.add(string(name))
    return result


def runtime_files():
    """Exact runtime capsule: embedded Python, controller API, native closure."""
    bindir=ROOT/"msys64/mingw64/bin"
    files=set()
    def tree(relative):
        files.update(p for p in (ROOT/relative).rglob("*")
                     if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc")
    for base in ("msys64/mingw64/bin/newton-runtime", "lib/controller",
                 "resources/nodes", "resources/images", "resources/icons", "resources/wren",
                 "resources/projects", "projects/default/controllers/sun_marker"):
        tree(base)
    files.update(ROOT/p for p in ("projects/objects/backgrounds/protos/OmniSimSky.proto",
                 "projects/objects/lights/protos/OmniSimSun.proto",
                 "projects/objects/lights/protos/OmniSimSunMarker.proto"))
    # Fonts with documented redistribution grants only; omit the shareware fonts.
    for p in (ROOT/"resources/fonts").glob("*"):
        if p.is_file() and not p.name.lower().startswith("code200"):
            files.add(p)
    for p in (ROOT/"resources").glob("*"):
        if p.is_file(): files.add(p)
    for base in ("platforms", "imageformats", "tls"):
        tree(f"msys64/mingw64/bin/{base}")
    files.add(bindir/"omnisim-bin.exe")
    # Windows activation-context assemblies are not PE DLL imports. Omitting
    # their manifests yields WinError 14001 before the engine executes a line.
    files.update(bindir.glob("*.manifest"))
    files.update(bindir.glob("*.exe.config"))
    files.update(bindir.glob("*._pth"))  # Embedded CPython's relative import paths.
    files.update(ROOT/p for p in ("projects/default/worlds/sounds/rotational_motor.wav",
                                 "projects/default/worlds/sounds/LICENSE.txt"))
    for name in ("wgpu_native.dll", "libopenal-1.dll"):
        if (bindir/name).exists(): files.add(bindir/name)
    lookup={p.name.lower():p for p in bindir.glob("*.dll")}
    pending=[p for p in files if p.suffix.lower() in (".exe", ".dll", ".pyd")]
    visited=set()
    while pending:
        p=pending.pop()
        if p in visited: continue
        visited.add(p)
        for name in pe_imports(p):
            if name.startswith("xinput") and (Path(os.environ.get("SystemRoot", "C:/Windows"))/"System32"/name).is_file():
                continue  # Use Windows' own system component; do not redistribute a local copy.
            target=lookup.get(name)
            if target and target not in files:
                files.add(target); pending.append(target)
    return files


def inventory(directory):
    paths=[p for p in sorted(directory.rglob("*"))
           if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"
           and p.relative_to(directory).as_posix() != "delivery-manifest.json"]
    with ThreadPoolExecutor(max_workers=8) as pool:
        return dict(pool.map(lambda p:(p.relative_to(directory).as_posix(),digest(p)),paths))


def cuda_redistributables():
    """The current Windows engine imports these even for CPU physics.

    Only CUDA Toolkit Attachment A runtime/NVRTC redistributables are selected;
    never copy a driver or arbitrary DLLs from the build machine's PATH.
    """
    required={n for n in pe_imports(ROOT/"msys64/mingw64/bin/omnisim-bin.exe")
              if n.startswith(("cudart64_", "nvrtc64_"))}
    if not required: return {}
    candidates=[Path(os.environ["CUDA_PATH"])/"bin"] if os.environ.get("CUDA_PATH") else []
    candidates += [Path(p) for p in os.environ.get("PATH", "").split(os.pathsep) if p]
    selected=next((p for p in candidates if all((p/n).is_file() for n in required)
                   and (p.parent/"EULA.txt").is_file()),None)
    if selected is None:
        raise ValueError("Packaging needs the build's CUDA runtime/NVRTC redistributables and EULA.txt")
    files=[selected/n for n in required]+list(selected.glob("nvrtc-builtins64_*.dll"))
    if not any(p.name.startswith("nvrtc-builtins") for p in files):
        raise ValueError("CUDA NVRTC builtins missing from build toolkit")
    result={f"msys64/mingw64/bin/{p.name}":p for p in files}
    result["licenses/CUDA-EULA.txt"]=selected.parent/"EULA.txt"
    result["licenses/CUDA-version.json"]=selected.parent/"version.json"
    return result


def verify_delivery(directory):
    manifest=read_json(directory/"delivery-manifest.json")
    if not manifest:
        raise ValueError("Missing or unsupported delivery manifest")
    profile=get_package_profile(manifest.get("robot"))
    if manifest.get("sku") != profile.sku:
        raise ValueError("Delivery robot and SKU disagree")
    checks=[]
    for name, expected in manifest["files_sha256"].items():
        rel=PurePosixPath(name)
        if rel.is_absolute() or ".." in rel.parts or ":" in name or "\\" in name:
            raise ValueError("Invalid manifest path")
        p=directory.joinpath(*rel.parts)
        checks.append((name,p,expected))
    def changed(item):
        name,p,expected=item
        return name if not p.is_file() or digest(p) != expected else None
    with ThreadPoolExecutor(max_workers=8) as pool:
        errors=[name for name in pool.map(changed,checks) if name]
    if errors: raise ValueError("Missing or modified delivery files: " + ", ".join(errors[:10]))
    state,_=validated_campaign(directory/"evidence",robot=profile.robot)
    if campaign_profile(state).robot != profile.robot:
        raise ValueError("Delivered evidence belongs to another robot")
    if manifest.get("package_profile") is not None and manifest["package_profile"] != profile.to_dict():
        raise ValueError("Delivery profile changed")
    return {"verified":True,"files_checked":len(manifest["files_sha256"]),
            "episodes_regraded":len(state["cases"]),"scope":profile.sku,"robot":profile.robot,"version":VERSION}


def finding_text(state):
    by_name={c["case"]:c for c in state["cases"]}
    bracket=next((b for b in state["boundaries"] if b["both_braking_qualified"]),None)
    if bracket is None: return "No qualified braking transition bracket was observed in this campaign."
    parts=[]
    for name in (bracket["low_case"],bracket["high_case"]):
        c=by_name[name]
        distance=c["metrics"].get("max_forward_excursion_m")
        parts.append(f'μ {c["spec"]["ground_mu"]:.6g}: {c["status"]}, '
                     + ("no measured post-STOP excursion" if distance is None else f"{distance:.6g} m forward excursion"))
    return (f'At {bracket["slope_deg"]:g}° downhill and {bracket["payload_kg"]:g} kg mounted payload: '
            + "; ".join(parts) + ".")


def write_customer_report(directory, state):
    profile=campaign_profile(state)
    spec=profile.spec()
    counts=Counter(c["status"] for c in state["cases"])
    grid=[c for c in state["cases"] if c["kind"] == "grid"]
    failures=[c for c in grid if c["status"] == "FAIL"]
    qualified=sum(c["metrics"].get("braking_qualified",False) for c in failures)
    rows=[]
    for c in state["cases"]:
        s,m=c["spec"],c["metrics"]
        distance=m.get("max_forward_excursion_m")
        rows.append(f'<tr><td><a href="evidence/{c["case"]}/report.html">{c["case"]}</a></td>'
                    f'<td>{c["kind"]}</td><td>{s["slope_deg"]:g}</td><td>{s["ground_mu"]:g}</td>'
                    f'<td>{s["payload_kg"]:g}</td><td class="{c["status"]}">{c["status"]}</td>'
                    f'<td>{"—" if distance is None else f"{distance:.4f}"}</td>'
                    f'<td>{html.escape(", ".join(c["reasons"]))}</td></tr>')
    fp=read_json(directory/"evidence"/state["cases"][0]["case"]/"fingerprint.json")
    save(directory/"results-summary.json",{"sku":profile.sku,"robot":profile.robot,"package_profile":profile.to_dict(),"counts":dict(counts),
         "grid_counts":dict(Counter(c["status"] for c in grid)),
         "grid_qualified_braking_failures":qualified,
         "grid_hold_approach_failures":len(failures)-qualified,"boundaries":state["boundaries"],
         "confirmations":state["confirmations"],"machine_fingerprint":fp})
    text=f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>{profile.label} braking assessment — OmniSim</title>
<style>body{{font:16px system-ui;line-height:1.6;color:#182d40;max-width:1200px;margin:auto;padding:30px}}
h1{{font-size:38px;line-height:1.2}}.box{{background:#eef3f7;padding:20px;border-radius:10px}}
table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{padding:9px;text-align:left;border-bottom:1px solid #d8e1e8}}
.PASS{{color:#12603e;font-weight:700}}.FAIL{{color:#a32d35;font-weight:700}}code{{background:#eef3f7;padding:3px}}a{{color:#07599b}}
@media print{{body{{padding:0;font-size:11px}}h1{{font-size:25px}}tr{{break-inside:avoid}}}}</style></head><body>
<p>OMNISIM · {profile.sku} · {VERSION}</p><h1>{profile.label} braking assessment</h1>
<div class="box"><strong>{counts['PASS']} passing / {counts['FAIL']} failing episodes · 64 total · immediate braking</strong>
<p>These findings assess an OmniSim reference wheel controller and {profile.label} model, not {profile.manufacturer} production firmware or measured hardware.</p></div>
<h2>Decision supported</h2><p>Identify tested conditions that meet the braking requirement and conditions needing investigation or a revised operating configuration. Use the evidence to prioritize controller and hardware tests. A simulation pass does not establish a physical operating limit.</p>
<h2>Scope and acceptance requirement</h2><p>One {profile.label} model and reference controller; target speed {profile.speed:.6g} m/s; downhill slopes 0/5/10/20°; contact μ 0.10/0.20/0.40/0.80; rigid payload {'/'.join(f'{v:g}' for v in profile.payloads)} kg centered {profile.height:.6g} m above the chassis-frame origin. The payload is a 0.30 × 0.25 × 0.20 m box with its own inertia, fixed to the chassis.</p>
<p>After a qualified approach (at least 0.30 m travel and 0.30 s in the speed band): forward excursion ≤{spec.max_stop_distance_m:.6g} m after STOP; speed ≤0.025 m/s within 1.5 s, sustained for ≥0.4 s and the remaining observation; rollback ≤0.05 m. Observe at least 2.5 s after STOP. Delay is zero. The distance limit scales with target speed squared, corresponding to a nominal minimum deceleration of 0.72 m/s², capped at 0.25 m. Requirements and payloads are experimental, not vendor specifications or rated capacities.</p>
<h2>Coverage and findings</h2><p>48 grid conditions, 12 local refinement episodes and four confirmations of one passing/failing pair. On the grid: {sum(c['status']=='PASS' for c in grid)} passed, {qualified} failed after a qualified braking approach, and {len(failures)-qualified} failed the hold/approach task. Every attempt is retained. Confirmations do not repeat the whole grid.</p>
<p>{finding_text(state)} A change in tested operating condition is not a verified controller repair. A failing case still moving at the observation end has no measured eventual stopping distance. The reports use maximum forward excursion within the recorded observation.</p>
<p>Payload-dependent results are retained in the case table. A maximum payload or a continuous safe region was not established. <a href="evidence/report.html">Sampled transition brackets and campaign details</a>.</p>
<h2>Evidence quality</h2><p>16 ms samples, eight physics substeps, Newton with CPU MuJoCo. Checks inspect compiled solver mass, tyre/floor friction and incline orientation; missing or modified evidence is an error. Held/approach failures are distinguished from braking failures. The included <a href="results-summary.json">machine-readable summary and original machine fingerprint</a> attribute these measurements to the delivery machine; this is not a throughput comparison. Source, engine, controller and runtime hashes are preserved.</p>
<h2>Reproduce and use</h2><p>Run <code>assessment.cmd verify</code> offline, then <code>assessment.cmd accept --out runs/acceptance-01</code> for four fresh failing/passing confirmations. An expected robot FAIL can be a successful reproduction. See <a href="QUICKSTART.md">Quickstart</a> for one-case reruns and the full 64-episode suite.</p>
<h2>Limitations and next engineering action</h2><p>This is a finite simulation campaign with assumed contact and rigid-body parameters. It does not test the OEM's production controller, loose cargo, rough terrain, perception, thermal effects, fatigue or fracture; it is not hardware validation or safety certification. Review low-traction hold/braking behavior, then integrate the OEM controller and correlate selected cases against independently measured hardware before setting physical limits.</p>
<p><a href="SCOPE.md">US$100 delivery scope and terms</a> · <a href="THIRD_PARTY_NOTICES.md">Notices</a> · <a href="delivery-manifest.json">Checksum inventory</a></p>
<h2>All episodes</h2><table><thead><tr><th>Case</th><th>Type</th><th>Slope °</th><th>μ</th><th>Load kg</th><th>Outcome</th><th>Forward m after STOP</th><th>Reason</th></tr></thead><tbody>{''.join(rows)}</tbody></table>
</body></html>'''
    (directory/"REPORT.html").write_text(text,encoding="utf-8")


def build_delivery(campaign, output, robot=None):
    if sys.platform != "win32": raise ValueError("The bundled delivery is Windows x64 only")
    state,first=validated_campaign(campaign.resolve(),robot=robot)
    profile=campaign_profile(state)
    binary=ROOT/"msys64/mingw64/bin/omnisim-bin.exe"
    if digest(binary) != first["binary_sha256"] or runtime_hashes(binary) != first["runtime_sha256"]:
        raise ValueError("Installed engine/controller/runtime differ from the campaign")
    output=output.resolve()
    if output.exists(): raise ValueError("Delivery output already exists")
    output.mkdir(parents=True)
    stage=output/f"{profile.slug}-{VERSION}-windows"
    stage.mkdir()
    files=runtime_files()
    files.update(ROOT/p for p in first["sources_sha256"])
    files.update(ROOT/p for p in ("omnisim/__init__.py","omnisim/paths.py",
                 "projects/policies/common/env_fingerprint.py", "LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"))
    files.update(ROOT/p for p in profile.notices)
    files.update(ROOT/f"projects/samples/demos/worlds/chat/omnilink_{name}.omniworld" for name in DEFINITIONS)
    # The shared registry reads these small world stanzas; other robot assets are
    # deliberately absent. The customer entry point exposes only this robot.
    for p in sorted(files):
        target=stage/p.relative_to(ROOT)
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(p,target)
    for relative,p in cuda_redistributables().items():
        target=stage/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(p,target)
    import psutil
    pkg=Path(psutil.__file__).parent
    site=stage/"msys64/mingw64/bin/newton-runtime/site-packages"
    shutil.copytree(pkg,site/"psutil",dirs_exist_ok=True,ignore=shutil.ignore_patterns("__pycache__","*.pyc","tests"))
    dist=importlib.metadata.distribution("psutil")
    for f in dist.files or []:
        if ".dist-info" in str(f):
            dest=site/str(f); dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(dist.locate_file(f),dest)
    shutil.copytree(campaign,stage/"evidence",ignore=shutil.ignore_patterns("__pycache__","*.pyc"))
    for name in ("SCOPE.md","QUICKSTART.md","CUSTOMER-ACCEPTANCE.md","RUNTIME-NOTICES.md"):
        text=(ROOT/"projects/assessments/packages/templates"/name).read_text(encoding="utf-8")
        (stage/name).write_text(render_document(text,profile),encoding="utf-8")
    save(stage/"package-profile.json",profile.to_dict())
    (stage/"customer.py").write_text(
        'import sys\nfrom pathlib import Path\nsys.path.insert(0,str(Path(__file__).resolve().parent))\n'
        'from omnisim.assessment.delivery import customer_main\nraise SystemExit(customer_main())\n',encoding="utf-8")
    (stage/"assessment.cmd").write_text('@echo off\nsetlocal\n'
        'set "OMNISIM_HOME=%~dp0"\nset "OMNISIM_BINARY=%~dp0msys64\\mingw64\\bin\\omnisim-bin.exe"\n'
        'set "PYTHONHOME="\nset "PYTHONPATH="\n'
        '"%~dp0msys64\\mingw64\\bin\\newton-runtime\\python.exe" -I "%~dp0customer.py" %*\n'
        'exit /b %errorlevel%\n',encoding="utf-8")
    write_customer_report(stage,state)
    save(stage/"delivery-manifest.json",{"sku":profile.sku,"robot":profile.robot,"package_profile":profile.to_dict(),"version":VERSION,"price_usd":100,
         "created_utc":datetime.now(timezone.utc).isoformat(),"platform":"Windows x64",
         "controller":"OmniSim reference driver; no OEM production firmware",
         "files_sha256":inventory(stage)})
    verify_delivery(stage)
    archive=output/f"{stage.name}.zip"
    with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(stage.rglob("*")):
            if p.is_file(): z.write(p,Path(stage.name)/p.relative_to(stage))
    checksum=digest(archive)
    (output/f"{archive.name}.sha256").write_text(f"{checksum}  {archive.name}\n",encoding="ascii")
    return {"archive":str(archive),"sha256":checksum,"bytes":archive.stat().st_size,"stage":str(stage)}


def customer_main(argv=None):
    parser=argparse.ArgumentParser(description="OmniSim robot US$100 braking assessment")
    sub=parser.add_subparsers(dest="command",required=True)
    sub.add_parser("verify",help="Check delivery hashes and regrade every saved episode; no engine")
    p=sub.add_parser("accept",help="Four fresh confirmations of an included failing/passing pair")
    p.add_argument("--out",type=Path,required=True)
    p=sub.add_parser("rerun",help="Fresh reproduction of a selected case")
    p.add_argument("case",help="Included case id, such as case-013")
    p.add_argument("--out",type=Path,required=True)
    p=sub.add_parser("suite",help="Repeat the defined grid and bounded search; outputs may differ across machines")
    p.add_argument("--out",type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        if len(str(ROOT)) > MAX_ROOT_CHARS:
            raise ValueError("Extract into a shorter local folder (full package path at most 100 characters), "
                             "for example a RobotKit folder under your user profile; avoid nested AppData paths")
        check=verify_delivery(ROOT)
        if args.command == "verify": print(json.dumps(check,indent=2)); return 0
        if args.out.resolve().is_relative_to(ROOT/"evidence"):
            raise ValueError("New runs must not overwrite delivered evidence")
        if args.command == "suite":
            from .package_profiles import qualify
            profile=get_package_profile(check["robot"])
            state=qualify(profile.robot,args.out,workers=1)
            return 0 if state["all_evidence_valid"] else 2
        state=read_json(ROOT/"evidence/campaign.json")
        if args.command == "rerun":
            if args.case not in {c["case"] for c in state["cases"]}:
                raise ValueError("Unknown included case id")
            verdict=rerun(ROOT/"evidence"/args.case,args.out)
            print(json.dumps(verdict,indent=2))
            return 0 if verdict["status"] == "PASS" else 1 if verdict["status"] == "FAIL" else 2
        chosen=acceptance_pair(state)
        if chosen is None: raise ValueError("No qualified passing/failing pair for acceptance")
        args.out.mkdir(parents=True,exist_ok=False)
        results=[]
        for name in (chosen["low_case"],chosen["high_case"]):
            original=read_json(ROOT/"evidence"/name/"verdict.json")
            for repeat in range(2):
                dest=args.out/f"{name}-repeat-{repeat+1}"
                v=rerun(ROOT/"evidence"/name,dest)
                a=v["metrics"].get("max_forward_excursion_m")
                b=original["metrics"].get("max_forward_excursion_m")
                difference=abs(a-b) if a is not None and b is not None else None
                results.append({"case":name,"repeat":repeat+1,"expected":original["status"],
                    "observed":v["status"],"distance_difference_m":difference,
                    "accepted":v["status"]==original["status"] and difference is not None and difference<=.005
                               and regrade(dest)==v})
                save(args.out/"acceptance.json",{"complete":False,"results":results})
        verified=all(r["accepted"] for r in results)
        save(args.out/"acceptance.json",{"complete":True,"verified":verified,
             "delivery_check":check,"distance_tolerance_m":.005,"results":results})
        print(json.dumps({"verified":verified,"results":results},indent=2))
        return 0 if verified else 2
    except (ValueError,OSError,KeyError) as exc:
        print(f"Assessment error: {exc}",file=sys.stderr)
        return 2
