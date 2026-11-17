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

"""Human + JSON rendering of a conformance result, scrub-by-default.

Everything that leaves this module has already been run through ``scrub`` so a
result is safe to print or paste into an issue (no usernames / home dirs).
"""

from __future__ import annotations

import json
from pathlib import Path

from ..paths import REPO_ROOT
from .scrub import scrub


def format_fingerprint(fp: dict) -> str:
    lines = [
        "config fingerprint",
        f"  id            {fp.get('fingerprint_id')}",
        f"  physics       {fp.get('physics', '?')}   resolved={fp.get('resolved_backend')}",
        f"  newton_rt     {'present' if fp.get('newton_runtime_present') else 'ABSENT'}"
        f"   binary={fp.get('omnisim_binary') or 'NOT FOUND'}",
        f"  render        {fp.get('render_backend')}   gpu={fp.get('gpu') or 'none'}",
        f"  build         {fp.get('build') or '?'}   os={fp.get('os')}"
        f"   py={fp.get('python')}   cores={fp.get('cpu_core_count')}",
        f"  pillow        {fp.get('pillow_version') or 'absent'}"
        f"   warp={fp.get('warp_version') or '-'}   mujoco={fp.get('mujoco_version') or '-'}",
    ]
    knobs = fp.get("knobs") or {}
    if knobs:
        lines.append("  knobs         " + " ".join(f"{k}={v}" for k, v in knobs.items()))
    for w in (fp.get("warnings") or []):
        lines.append(f"  warn          {w}")
    return "\n".join(lines)


def format_human(result: dict) -> str:
    result = scrub(result)
    fp = result["fingerprint"]
    out = [f"OmniSim install conformance - {result['status']}", "", format_fingerprint(fp)]
    observed = result.get("observed_backend")
    if observed and observed not in ("unknown", fp.get("resolved_backend")):
        out.append(f"  observed      physics that drove this run: {observed}")
    if not result.get("calibrated"):
        out.append("  note          bands are UNCALIBRATED - SOFT misses report "
                   "as DRIFT only, never FAIL")
    elif result.get("calibration"):
        c = result["calibration"]
        out.append(f"  calibrated    within-host runs={c.get('runs')} "
                   f"commit={c.get('commit')} fp={c.get('fingerprint_id')}")
    out += ["", "demos"]
    for d in result["demos"]:
        out.append(f"  [{d['status']:5}] {d['id']:26} {d.get('subsystem', '')}")
        if d.get("reason"):
            out.append(f"           reason: {d['reason']}")
        if d.get("note"):
            out.append(f"           note: {d['note']}")
        for m in d.get("metrics", []):
            if m["status"] != "PASS":
                val = m.get("value")
                shown = f" = {val}" if m["status"] == "INFO" and val is not None else ""
                out.append(f"           - {m['name']}: {m['status']}{shown} ({m.get('detail', '')})")
    adv = result.get("advisories", [])
    if adv:
        out += ["", "advisories"]
        for a in adv:
            out.append(f"  ! {a['code']} ({a['severity']}): {a['message']}")
            out.append(f"    fix: {a['fix']}")
    out += ["", f"verdict: {result['status']}   (logs: {result.get('report_dir')})"]
    return "\n".join(out)


def to_json(result: dict) -> str:
    return json.dumps(scrub(result), indent=2, default=str)


def fingerprint_json(fp: dict) -> str:
    return json.dumps(scrub(fp), indent=2, default=str)


def format_markdown(result: dict) -> str:
    result = scrub(result)
    return (
        f"# OmniSim install conformance — {result['status']}\n\n"
        f"```\n{format_human(result)}\n```\n"
    )


def write_bundle(result: dict, dest=None) -> dict:
    """Write a scrubbed {json, md} report bundle. Phase-2 down-payment."""
    d = Path(dest) if dest else REPO_ROOT / "install-reports"
    d.mkdir(parents=True, exist_ok=True)
    fp = result.get("fingerprint", {})
    stem = (f"install-check-{fp.get('build') or 'nobuild'}-"
            f"{fp.get('fingerprint_id', 'nofp')}-{result['status'].replace('-', '').lower()}")
    jpath = d / (stem + ".json")
    mpath = d / (stem + ".md")
    jpath.write_text(to_json(result), encoding="utf-8")
    mpath.write_text(format_markdown(result), encoding="utf-8")
    return {"json": str(jpath), "md": str(mpath)}
