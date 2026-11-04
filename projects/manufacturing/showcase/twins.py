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

"""The machine-twin stills: a SparkRover part's real job streamed to its twin over GRBL, and
photographed part-way through.

    python projects/manufacturing/showcase/twins.py laser --job deck_upper --at 0.55
    python projects/manufacturing/showcase/twins.py cnc --job shoulder_brackets --at 0.5
    python projects/manufacturing/showcase/twins.py laser --survey      # framing shots, no job

The world is the one spark_rover/make.py builds for that job (make.build_world: the LongMill
MK2 twin with its laser head or router, the job's stock), plus the showcase photographer. The
program is make.py's job text, streamed exactly as make.py's run_job does (grbl_link, character
counting). When the twin has acknowledged `--at` of the program's lines the photographer is
triggered; the machine is still running when the frame is taken. One difference from make.py's
measured runs, for the laser only: a sheet longer than 99 mm runs here as ONE sheet, which the
engine coarsens to its 1984-cell cap (0.09 mm on the 178 mm deck, where make.py cuts 0.05 mm
windows). It changes how finely the kerf is resolved, not the job.
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import showcase_lib as lib  # noqa: E402

ROOT = lib.ROOT
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"


def _free_port(base: int) -> int:
    port = base
    while True:
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", port))
                return port
            except OSError:
                port += 7


def job_world(name: str, port_base: int = 2471):
    """(world text re-pointed into showcase/worlds, the make.Job, its GRBL port, stock box mm)."""
    sys.path.insert(0, str(PRODUCT))
    import make  # noqa: E402   (spark_rover/make.py: the jobs and their worlds)
    jobs = {j.name: j for j in make.all_jobs(dedupe=False)}
    job = jobs[name]
    port = _free_port(port_base)
    out = lib.SCRATCH / "make"
    path = make.build_world(job, out, port)
    try:
        text = lib.read_world(path)
    finally:
        path.unlink(missing_ok=True)
    m = re.search(r"DEF STOCK MillingStock \{ translation (\S+) (\S+) (\S+)\s+size (\S+) (\S+) (\S+)", text)
    stock = [float(v) for v in m.groups()]
    return text, job, port, stock


SPOILBOARD_MESH = "longmill_mk2_30x30/cad/meshes/frame_24.stl"   # the 1321 x 1219 x 20 mm wasteboard


def look(process: str, text: str) -> str:
    """Two look-only changes for the stills (README.md says so). The LongMill pack's wasteboard
    (frame_24, an MDF orange that renders pale beige) is drawn dark, so plywood and cut-through
    kerfs read against it. The aluminium blank's skin (.58 .60 .63, roughness .35: dark plastic on
    camera) is drawn as mill-finish plate, .60 .63 .68 with its machined core .84 .86 .89, roughness .2. The cut and the job are unchanged."""
    def board(m):
        return m.group(0).replace(m.group(1), "baseColor 0.075 0.06 0.05") if SPOILBOARD_MESH in m.group(0) else m.group(0)
    text = re.sub(r'(baseColor [0-9.]+ [0-9.]+ [0-9.]+)[^}]*\} geometry Mesh \{ url "[^"]*"', board, text)
    # The machine worlds keep the sun marker at its default (0 0 8): the sun straight overhead,
    # every vertical face unlit. The stills move it to the camera's side (front right, 40 deg up),
    # which lights the faces the camera sees and throws the shadows behind the work.
    text = text.replace("DEF SUN_MARKER OmniSimSunMarker { }", "DEF SUN_MARKER OmniSimSunMarker { translation 2.4 -3.2 3.4 }", 1)
    if process == "cnc":
        text = text.replace("coreColor .86 .87 .89 skinColor .58 .6 .63 roughness .35",
                            "coreColor .84 .86 .89 skinColor .60 .63 .68 roughness .2")
    return text


def streamer(port: int, program: str, at: float, trigger: Path, log: list):
    """The `during` hook: connect to the twin's GRBL port, stream the job, trigger at `at`."""
    sys.path.insert(0, str(ROOT / "projects/manufacturing/cnc"))
    from grbl_link import GrblError, GrblLink  # noqa: E402

    def during(proc, run):
        link = None
        t0 = time.time()
        while link is None:
            if proc.poll() is not None:
                raise RuntimeError(f"the engine exited before the twin listened ({run})")
            try:
                link = GrblLink("127.0.0.1", port, connect_timeout_s=5.0)
            except (OSError, GrblError):
                time.sleep(1.0)
                if time.time() - t0 > 180:
                    raise RuntimeError("the twin never listened")
        j = link.stream(program)
        t1 = time.time()
        last = 0.0
        while j.state == "running" and j.acked < at * j.total:
            if proc.poll() is not None:
                raise RuntimeError("the engine exited during the job")
            if time.time() - last > 60:
                last = time.time()
                print(f"[twins] {j.acked}/{j.total} lines acknowledged ({j.acked / max(1, j.total):.0%}), "
                      f"{time.time() - t1:.0f} s", flush=True)
            time.sleep(0.5)
        st = link.status(max_age_s=1.0)
        log.append({"lines_acked": j.acked, "lines_total": j.total, "fraction": round(j.acked / max(1, j.total), 3),
                    "stream_wall_s": round(time.time() - t1, 1), "status": st, "state": j.state})
        trigger.write_text(json.dumps(log[-1], default=str), encoding="utf-8")

        def keep_streaming():          # the machine keeps cutting while the photographer works
            try:
                while j.state == "running" and proc.poll() is None:
                    time.sleep(0.5)
            finally:
                try:
                    link.close()
                except Exception:
                    pass
        threading.Thread(target=keep_streaming, daemon=True).start()
    return during


def shots_around(stock, views, at_s=0.0, trigger=None):
    """Shots aimed at the stock: views are (name, dx, dy, dz, tx, ty, tz) in m from the stock's centre."""
    x0, y0, z0, sx, sy, sz = stock
    c = (x0 + sx / 2, y0 + sy / 2, z0 + sz)
    out = []
    for name, dx, dy, dz, tx, ty, tz in views:
        s = {"name": name, "position": [c[0] + dx, c[1] + dy, c[2] + dz],
             "target": [c[0] + tx, c[1] + ty, c[2] + tz], "at_s": at_s}
        if trigger:
            s["trigger"] = str(trigger)
            s["timeout_wall_s"] = 7200
        out.append(s)
    return out


PRINTING = ROOT / "projects/manufacturing/printing"
SV08 = ROOT / "projects/manufacturing/machines/sovol_sv08"


def _rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def printer_job(part: str, layer_frac: float):
    """Slice one SparkRover printed part alone (release/models, as it lies on the bed) with the
    repo's CuraEngine wrapper, cut the G-code after `layer_frac` of its layers, and return
    (truncated G-code, layers kept, layers total, the twin's planned time to the cut in s)."""
    import re
    import subprocess
    d = lib.SCRATCH / "print"
    d.mkdir(parents=True, exist_ok=True)
    full = d / f"{part}.gcode"
    if not full.exists():
        subprocess.run([sys.executable, str(PRINTING / "cura_slice.py"), str(PRODUCT / f"release/models/{part}.stl"),
                        "--out", str(full), "--printer", str(SV08 / "sovol_sv08.json")], check=True, cwd=ROOT,
                       capture_output=True)
    text = full.read_text(encoding="utf-8")
    starts = [m.start() for m in re.finditer(r"^;LAYER:\d+", text, re.M)]
    keep = max(1, int(round(len(starts) * layer_frac)))
    cut = d / f"{part}_first{keep}.gcode"
    cut.write_text(text[:starts[keep]] if keep < len(starts) else text, encoding="utf-8", newline="\n")
    pf = subprocess.run([sys.executable, str(PRINTING / "preflight.py"), str(cut)], capture_output=True, text=True,
                        cwd=ROOT).stdout
    m = re.search(r"time_s:\s*([\d.]+)", pf)
    return cut, keep, len(starts), float(m.group(1)) if m else None


def cmd_printer(a) -> int:
    part = a.part
    cut, keep, total, t_cut = printer_job(part, a.layers)
    src = lib.WORLDS / ".gen_printer_src.omniworld"
    lib.WORLDS.mkdir(parents=True, exist_ok=True)
    import subprocess
    subprocess.run([sys.executable, str(PRINTING / "world.py"),      # the default printer is the SV08
                    "--gcode", _rel(cut), "--pack", _rel(SV08 / "cad"), "--trace", _rel(lib.SCRATCH / "print/trace.jsonl"),
                    "--out", str(src)], check=True, cwd=ROOT, capture_output=True)
    text = src.read_text(encoding="utf-8")
    src.unlink(missing_ok=True)
    m = re.search(r"DEF PRINT PrintedPart \{ translation (\S+) (\S+) (\S+)", text)
    px, py, pz = (float(v) for v in m.groups())
    c = (px + 0.175, py + 0.182, pz)                     # the part is sliced centred on the bed
    at = round(t_cut * a.at, 1) if t_cut else 600.0
    views = json.loads(Path(a.views or HERE / "views/printer.json").read_text(encoding="utf-8"))
    shots = [{"name": n, "position": [c[0] + dx, c[1] + dy, c[2] + dz], "target": [c[0] + tx, c[1] + ty, c[2] + tz],
              "at_s": at if not a.survey else 1.0} for n, dx, dy, dz, tx, ty, tz in views]
    name = "twin_printer" + a.tag + ("_survey" if a.survey else "")
    run = lib.shoot(name, text, shots, 3200, 2000, fov=0.7, far=a.far, timeout_s=6 * 3600)
    (run / "progress.json").write_text(json.dumps({"part": part, "layers_kept": keep, "layers_total": total,
                                                   "planned_s_to_cut": t_cut, "shot_at_s": at,
                                                   "gcode": _rel(cut)}, indent=1), encoding="utf-8")
    publish_frames(run, shots, a.review or a.survey)
    print(json.dumps({"part": part, "layers": f"{keep}/{total}", "t_cut": t_cut, "at": at, "run": str(run)}))
    return 0


# The printer's shot has the frame's front-right upright between the camera and the bed's right
# edge; the published still leaves that strip out (raw-frame pixels, 3200 x 2000).
CROP = {"twin_printer": (0, 0, 2600, 2000)}


def publish_frames(run: Path, shots, review: bool) -> None:
    """A view named twin_* is the published still (docs/images/manufacturing/<name>.png); the rest,
    and everything with --review, go to the review folder."""
    for s in shots:
        f = run / "frames" / f"{s['name']}.png"
        if not f.exists():
            continue
        if s["name"].startswith("twin_") and not review:
            lib.publish(f, lib.OUT / f"{s['name']}.png", crop=CROP.get(s["name"]))
        else:
            lib.publish(f, lib.SCRATCH / "review" / f"{run.name}_{s['name']}.png")


def main(argv=None) -> int:
    if len(sys.argv if argv is None else argv) > 1 and (sys.argv[1:] if argv is None else argv)[0] == "printer":
        p = argparse.ArgumentParser(description="the SV08 twin part-way through a SparkRover printed part")
        p.add_argument("process")
        p.add_argument("--part", default="shell_lower_front")
        p.add_argument("--layers", type=float, default=0.4, help="the fraction of the part's layers in the job")
        p.add_argument("--at", type=float, default=0.97, help="the shot, as a fraction of the planned time to the cut")
        p.add_argument("--views", default=None)
        p.add_argument("--survey", action="store_true")
        p.add_argument("--tag", default="")
        p.add_argument("--far", type=float, default=3.0)
        p.add_argument("--review", action="store_true")
        return cmd_printer(p.parse_args(argv))
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("process", choices=("laser", "cnc"))
    p.add_argument("--job", default=None)
    p.add_argument("--at", type=float, default=0.55, help="the fraction of the program's lines acknowledged at the shot")
    p.add_argument("--survey", action="store_true")
    p.add_argument("--views", default=None, help="a JSON file of views (name, dx, dy, dz, tx, ty, tz)")
    p.add_argument("--tag", default="")
    p.add_argument("--far", type=float, default=0.0, help="the camera's far plane (it sizes the shadow map)")
    p.add_argument("--review", action="store_true", help="write every frame to the review folder only")
    p.add_argument("--port-base", type=int, default=0, help="the GRBL port to start from (two runs at once need different ones)")
    a = p.parse_args(argv)
    job_name = a.job or {"laser": "deck_upper", "cnc": "shoulder_brackets"}[a.process]
    text, job, port, stock = job_world(job_name, a.port_base or {"laser": 2471, "cnc": 2491}[a.process])
    text = look(a.process, text)
    name = f"twin_{a.process}{a.tag}"
    views = json.loads(Path(a.views or HERE / f"views/{a.process}.json").read_text(encoding="utf-8"))
    far = a.far or 2.0          # the camera's shadow map is fitted to max(2, 0.75 far) m: 2 mm texels
    if a.survey:
        shots = shots_around(stock, views, at_s=1.0)
        run = lib.shoot(name + "_survey", text, shots, 2400, 1500, fov=0.7, far=far, timeout_s=600)
    else:
        trig = lib.trigger_file(name)
        shots = shots_around(stock, views, trigger=trig)
        log = []
        during = streamer(port, job.program, a.at, trig, log)
        run = lib.shoot(name, text, shots, 3200, 2000, fov=0.7, far=far, timeout_s=4 * 3600, during=during)
        (run / "progress.json").write_text(json.dumps({"job": job_name, "process": a.process, "progress": log},
                                                      indent=1, default=str), encoding="utf-8")
    print(json.dumps({"stock": stock, "port": port, "run": str(run)}))
    publish_frames(run, shots, a.review or a.survey)
    return 0


if __name__ == "__main__":
    sys.exit(main())
