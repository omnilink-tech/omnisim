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

"""OmniSim skill library -- the manifest-driven runner + pipeline front door.

A *skill* is a named, composable robot behaviour (walk, turn-in-place, carry-box,
stand, climb-stairs, ...). This CLI is the single entry point for the skill pipeline:

    design ghost  ->  validate  ->  preview  ->  train  ->  run/verify  ->  sequence

It reads the versioned manifests (``<class>/<skill>/skill.json``, ``sequences/*.json``,
``profiles/*.json``), assembles the deploy env bundle, and delegates to the EXISTING
generic stack (``projects/policies/training/run_walk_rl.sh`` for RL shadowing skills,
the ``g1_ghost`` hologram controller for previews, the deterministic PowerShell
launcher for the balance/arm overlays). It reimplements none of the engine -- it
factors the per-demo env bundle out of the hand-written shell scripts into data.

Commands:
    list                       list skills + sequences
    show    <name>             full manifest + ghost summary + (sequence) assembled env
    validate                   validate every manifest + round-trip every ghost lut
    ghost   <lut|--all>        describe/validate ghost luts (-> ghost_lut.py)
    preview <skill>            launch the ghost hologram preview (design -> show -> agree)
    train   <skill>            assemble + launch the recipe trainer (reconstructed env; --go to run)
    run     <skill>            deploy a single skill solo
    sequence <name>            deploy a BATON sequence (box_delivery, turn_solo, ...)
    verify-demos               PROVE the assembled env == the reproduced shell scripts
    index                      regenerate registry.json from filesystem discovery

Add ``--dry-run`` to any launch command to print the command instead of running it.

Back-compat: ``skill_lib.py --list`` and ``skill_lib.py <name> [--throw --gui --duration N]``
keep the old run_skill.py behaviour (deterministic overlays via the PowerShell launcher).
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shlex
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manifest as M          # noqa: E402
from ghost_lut import GhostLut  # noqa: E402

REPO = M.REPO_ROOT
SKILLS_DIR = M.SKILLS_DIR

# The BATON mode file is a RUNTIME scratch artefact (the harness_rig <-> policy
# handshake), never a tracked asset -- so it is resolved against THIS clone's repo
# root, not a maintainer-specific absolute path. Same default as the reproduced demo
# scripts ("${BATON_MODE_FILE:-$ROOT/_scratch/foot_redesign/boxmode.txt}"), and the
# environment override is honoured identically. posix-style so the value survives the
# round trip through bash on Windows as well as native Python on any OS.
MODE_FILE = (os.environ.get("BATON_MODE_FILE")
             or (REPO / "_scratch" / "foot_redesign" / "boxmode.txt").as_posix())

# shell-var expansions used by the reproduced demo scripts (for verify-demos) and by
# the sequence manifests (which write "$MODEF" instead of an absolute path).
_SCRIPT_VARS = {
    "$R": "projects/policies/training/runs",
    "$G": "projects/policies/controllers/g1_ghost",
    "$MODEF": MODE_FILE,
}


def _expand_vars(value: str) -> str:
    """Expand the shell-style placeholders ($R / $G / $MODEF) in a manifest env value."""
    if not isinstance(value, str) or "$" not in value:
        return value
    for var, sub in _SCRIPT_VARS.items():
        value = value.replace(var, sub)
    return value


def _assemble_env(s: M.SequenceManifest, reg: M.Registry) -> dict[str, str]:
    """``M.assemble_deploy_env`` + placeholder expansion (the single entry point)."""
    return {k: _expand_vars(v) for k, v in M.assemble_deploy_env(s, reg).items()}


# --------------------------------------------------------------------------- #
#  list / show
# --------------------------------------------------------------------------- #
def cmd_list(reg: M.Registry) -> int:
    print(f"{'SKILL':<16} {'CLASS':<9} {'KIND':<14} {'METHOD':<20} {'STATUS':<12} {'MOTION':<9} ROBOTS")
    print("-" * 100)
    for name, m in reg.skills.items():
        print(f"{name:<16} {m.klass:<9} {m.kind:<14} {m.method:<20} {m.status:<12} "
              f"{m.motion_class:<9} {','.join(m.robots)}")
    print("\nSEQUENCES (BATON demos):")
    print(f"{'SEQUENCE':<16} {'STATUS':<12} PRIMARY + SKILLS  ->  reproduces")
    print("-" * 100)
    for name, s in reg.sequences.items():
        chain = " + ".join([s.primary] + s.skills)
        print(f"{name:<16} {s.status:<12} {chain}")
        if s.reproduces:
            print(f"{'':<29}-> {s.reproduces}")
    print("\nRun:  python skill_lib.py sequence <name>        # a BATON demo")
    print("      python skill_lib.py run <skill>             # one skill solo")
    print("      python skill_lib.py preview <skill>         # the ghost hologram")
    return 0


def cmd_show(reg: M.Registry, name: str) -> int:
    if name in reg.skills:
        m = reg.skills[name]
        print(f"# SKILL  {m.name}  --  {m.title}")
        print(json.dumps(m.raw, indent=2))
        lut = m.ghost_lut
        if lut and (REPO / lut).exists():
            print("\n# ghost:")
            print("  ", GhostLut.load(REPO / lut).describe())
        return 0
    if name in reg.sequences:
        s = reg.sequences[name]
        print(f"# SEQUENCE  {s.name}  --  {s.title}")
        print(json.dumps(s.raw, indent=2))
        print("\n# assembled deploy env (KEY=VALUE bundle for run_walk_rl.sh):")
        env = _assemble_env(s, reg)
        for k in sorted(env):
            print(f"  {k}={env[k]}")
        return 0
    print(f"[skill] unknown skill/sequence '{name}'", file=sys.stderr)
    return 2


# --------------------------------------------------------------------------- #
#  validate
# --------------------------------------------------------------------------- #
def cmd_validate(reg: M.Registry) -> int:
    n_err = 0
    print("== skills ==")
    for name, m in reg.skills.items():
        issues = m.validate()
        errs = [i for i in issues if not i.startswith("warn:")]
        tag = "ERR " if errs else ("warn" if issues else "ok  ")
        print(f"[{tag}] {name}")
        for i in issues:
            print("      ", i)
        n_err += len(errs)
    print("== sequences ==")
    for name, s in reg.sequences.items():
        issues = s.validate(reg)
        errs = [i for i in issues if not i.startswith("warn:")]
        tag = "ERR " if errs else ("warn" if issues else "ok  ")
        print(f"[{tag}] {name}")
        for i in issues:
            print("      ", i)
        n_err += len(errs)
    print("== ghost luts ==")
    d = REPO / "projects" / "policies" / "controllers" / "g1_ghost"
    luts = sorted(d.glob("ghost_*_lut.json"))
    g_err = 0
    for p in luts:
        try:
            issues = GhostLut.load(p).validate()
        except Exception as e:  # noqa: BLE001
            print(f"[ERR ] {p.name}: {e}")
            g_err += 1
            continue
        errs = [i for i in issues if i.level == "error"]
        if errs:
            print(f"[ERR ] {p.name}")
            for i in errs:
                print("      ", i)
            g_err += len(errs)
    print(f"  {len(luts)} ghost luts checked, {g_err} errors")
    n_err += g_err
    print("-" * 60)
    print(f"validate: {n_err} error(s)")
    return 1 if n_err else 0


# --------------------------------------------------------------------------- #
#  launch helpers
# --------------------------------------------------------------------------- #
def _run_or_print(argv: list[str], dry: bool, cwd: Path = REPO) -> int:
    printable = " ".join(shlex.quote(a) for a in argv)
    print(f"\n$ {printable}\n")
    if dry:
        print("[dry-run] not executed. Drop --dry-run to launch.")
        return 0
    return subprocess.call(argv, cwd=str(cwd))


def _bash_run_walk(env: dict[str, str], tag: str, duration: int, gui: str, dry: bool,
                   pre: list[list[str]] | None = None) -> int:
    argv = M.render_run_argv(env, tag, duration, gui)
    if pre and not dry:
        for cmd in pre:
            subprocess.call(cmd, cwd=str(REPO))
    return _run_or_print(argv, dry)


# --------------------------------------------------------------------------- #
#  preview / train / run / sequence
# --------------------------------------------------------------------------- #
def cmd_preview(reg: M.Registry, name: str, duration: int, dry: bool) -> int:
    if name not in reg.skills:
        print(f"[skill] unknown skill '{name}'", file=sys.stderr)
        return 2
    m = reg.skills[name]
    if m.kind != "rl":
        print(f"[skill] '{name}' is {m.kind}; preview is for shadowing (ghost) skills.")
        return 2
    world = m.ghost.get("preview_world") or "projects/policies/worlds/g1_ghost_preview.wbt"
    lut = m.ghost_lut
    argv = [
        "bash", "-lc",
        f'G1_GHOST_LUT="{lut}" python -u scripts/dev/headless_runner.py '
        f'"{world}" --gui --realtime --duration {duration}',
    ]
    print(f"# preview ghost for skill '{name}': {lut}")
    print(f"#   world={world}  (design -> show owner -> agree, BEFORE training)")
    return _run_or_print(argv, dry)


def cmd_train(reg: M.Registry, name: str, iters: int, gui: str, dry: bool) -> int:
    if name not in reg.skills:
        print(f"[skill] unknown skill '{name}'", file=sys.stderr)
        return 2
    m = reg.skills[name]
    if m.kind != "rl":
        print(f"[skill] '{name}' is {m.kind}; nothing to train (deterministic).")
        return 2
    tr = m.train
    if tr.get("status") == "reconstructed":
        print("!! WARNING: this skill's training env is RECONSTRUCTED from notes/commits, not a\n"
              "   verified frozen bundle. Confirm the recipe against the doc/memory before trusting a\n"
              "   re-train (see the manifest's train.notes).")
    # a frozen bundle (via `freeze`) reproduces the champion; else the reconstructed one
    if tr.get("status") == "frozen" and tr.get("frozen_env"):
        env = M._stringify_env(tr["frozen_env"])
        env.setdefault("OMNISIM_INENGINE_PYMOD", M.RECIPE_TRAIN_PYMOD)
    else:
        env = {"OMNISIM_INENGINE_PYMOD": M.RECIPE_TRAIN_PYMOD}
        env.update(M._stringify_env(tr.get("recipe_env", {})))
    world = tr.get("train_world", "")
    if world:
        env["WALK_WORLD"] = world
    # freeze support (#3): capture the EXACT launch env so `freeze <skill>` can promote it
    if not dry:
        lock = REPO / "projects/policies/training/runs" / f"{name}_train.env.json"
        lock.parent.mkdir(parents=True, exist_ok=True)
        lock.write_text(json.dumps(env, indent=2) + "\n", encoding="utf-8")
        print(f"[train] wrote env lock -> {lock.relative_to(REPO)}  (promote with: skill_lib.py freeze {name})")
    argv = ["bash", "projects/policies/training/run_walk_rl.sh", str(iters), f"{name}_train", "train", gui] \
        + [f"{k}={env[k]}" for k in sorted(env)]
    return _run_or_print(argv, dry)


def cmd_run(reg: M.Registry, name: str, duration: int, gui: str, dry: bool,
            throw: bool = False) -> int:
    if name not in reg.skills:
        print(f"[skill] unknown skill '{name}'", file=sys.stderr)
        return 2
    m = reg.skills[name]
    mode = m.deploy_run
    if mode == "powershell":
        return _run_deterministic(m, duration, gui, throw, dry)
    if mode == "world":
        world = m.deploy.get("world") or m.deploy.get("deploy_world")
        if not world:
            print(f"[skill] '{name}' deploy.run=world but no deploy.world set", file=sys.stderr)
            return 2
        gflag = "--gui --realtime " if gui == "gui" else ""
        argv = ["bash", "-lc",
                f'python -u scripts/dev/headless_runner.py "{world}" {gflag}--duration {duration}']
        print(f"# run skill '{name}' ({m.method}) by launching its world (controller baked in):")
        print(f"#   world={world}")
        return _run_or_print(argv, dry)
    # recipe (in-engine Shadowing)
    env = M.assemble_solo_env(m)
    return _bash_run_walk(env, f"{name}_solo", duration, gui, dry)


# --------------------------------------------------------------------------- #
#  adapt / blendable (#1)  |  handover (#2)  |  freeze (#3)
# --------------------------------------------------------------------------- #
def cmd_adapt(reg: M.Registry, name: str, to_nb: int, out: str | None) -> int:
    import adapter
    if name not in reg.skills:
        print(f"[skill] unknown skill '{name}'", file=sys.stderr)
        return 2
    m = reg.skills[name]
    src = GhostLut.load(REPO / m.ghost_lut)
    resampled = adapter.resample_lut(src.raw, to_nb)
    print(f"# adapt '{name}' ghost {src.nb} -> {to_nb} bins (class={src.motion_class})")
    if out:
        Path(out).write_text(json.dumps(resampled), encoding="utf-8")
        chk = GhostLut(Path(out), resampled)
        errs = [i for i in chk.validate() if i.level == "error"]
        print(f"# wrote {out}  ({len(errs)} schema errors)")
    else:
        print("# (no --out: not written; pass --out <path> to emit the blend-compatible lut)")
    return 0


def cmd_blendable(reg: M.Registry, a: str, b: str) -> int:
    import adapter
    if a not in reg.skills or b not in reg.skills:
        print("[skill] both names must be skills", file=sys.stderr)
        return 2
    ma, mb = reg.skills[a], reg.skills[b]
    la, lb = GhostLut.load(REPO / ma.ghost_lut), GhostLut.load(REPO / mb.ghost_lut)
    rep = adapter.blend_report(la, lb, ma.policy.get("obs", {}).get("dim"),
                               mb.policy.get("obs", {}).get("dim"))
    print(f"# blend {a} <-> {b}")
    print(f"  blendable      : {rep['blendable']}")
    print(f"  cadence_match  : {rep['cadence_match']}  (nb {la.nb} vs {lb.nb})")
    print(f"  obs_match      : {rep['obs_match']}  (dim {ma.policy.get('obs', {}).get('dim')} vs {mb.policy.get('obs', {}).get('dim')})")
    for need in rep["needs"]:
        print(f"  needs          : {need}")
    return 0


def cmd_handover(reg: M.Registry, name: str) -> int:
    if name not in reg.sequences:
        print(f"[skill] unknown sequence '{name}'", file=sys.stderr)
        return 2
    plan = M.resolve_handover(reg.sequences[name], reg)
    print(f"# handover plan for '{name}'")
    print(f"  modes: {' -> '.join(plan['modes'])}")
    for e in plan["edges"]:
        print(f"  {e['edge']:<16} {e['decision']:<8} {e['reason']}")
    print(f"  => BATON_COLD_HIDDEN = {'1 (derived)' if plan['global_cold'] else '0 (not needed)'}")
    return 0


def cmd_freeze(reg: M.Registry, name: str, from_env: str, checkpoint: str | None) -> int:
    if name not in reg.skills:
        print(f"[skill] unknown skill '{name}'", file=sys.stderr)
        return 2
    m = reg.skills[name]
    envp = Path(from_env) if from_env else (REPO / "projects/policies/training/runs" / f"{name}_train.env.json")
    if not envp.exists():
        print(f"[skill] no train env lock at {envp}. Run `train {name}` first (writes it at launch), "
              f"or pass --from <env.json>.", file=sys.stderr)
        return 2
    env = json.loads(envp.read_text(encoding="utf-8"))
    M.freeze_train_bundle(m.path, env, checkpoint)
    print(f"# froze {len(env)} env keys into {m.path.name} (train.status -> frozen)"
          + (f", policy.checkpoint -> {checkpoint}" if checkpoint else ""))
    return 0


def _run_deterministic(m: M.SkillManifest, duration: int, gui: str, throw: bool, dry: bool) -> int:
    launcher = m.raw.get("launcher")
    if not launcher:
        print(f"[skill] deterministic skill '{m.name}' has no launcher.", file=sys.stderr)
        return 2
    flags = list(m.raw.get("launch_flags", []))
    flags += ["-Duration", str(duration)]
    if throw:
        flags += ["-Throw"]
    if gui == "gui":
        flags += ["-Gui"]
    argv = ["powershell", "-NoProfile", "-File", str(REPO / launcher), *flags]
    if platform.system() != "Windows" and not dry:
        print("[skill] this deterministic skill uses a PowerShell launcher; run on Windows. "
              "Command printed below.")
        dry = True
    return _run_or_print(argv, dry)


def cmd_sequence(reg: M.Registry, name: str, duration: int | None, gui: str | None, dry: bool) -> int:
    if name not in reg.sequences:
        print(f"[skill] unknown sequence '{name}'. Known: {', '.join(reg.sequences)}", file=sys.stderr)
        return 2
    s = reg.sequences[name]
    env = _assemble_env(s, reg)
    dur = duration if duration is not None else s.duration_default
    g = gui if gui is not None else s.gui_default
    pre: list[list[str]] = []
    if s.init_mode_file and env.get("BATON_MODE_FILE"):
        mf = env["BATON_MODE_FILE"]
        pre.append(["bash", "-lc", f'mkdir -p "$(dirname {shlex.quote(mf)})" && echo parked > {shlex.quote(mf)}'])
    return _bash_run_walk(env, name, dur, g, dry, pre=pre)


# --------------------------------------------------------------------------- #
#  verify-demos  (the equivalence proof)
# --------------------------------------------------------------------------- #
def _parse_demo_script(path: Path) -> dict[str, str]:
    """Extract the KEY=VALUE env bundle a demo script passes to run_walk_rl.sh."""
    text = path.read_text(encoding="utf-8").replace("\\\n", " ")
    idx = text.find("run_walk_rl.sh")
    if idx < 0:
        raise ValueError(f"{path}: no run_walk_rl.sh invocation found")
    tail = text[idx + len("run_walk_rl.sh"):]
    # stop at the end of that logical command (first newline that is not continued)
    tail = tail.split("\n")[0]
    tokens = shlex.split(tail)
    # skip the 4 positional args: <dur> <tag> deploy <gui>
    # find 'deploy' then take everything after the token following it
    if "deploy" in tokens:
        di = tokens.index("deploy")
        rest = tokens[di + 2:]   # skip 'deploy' and the gui token after it
    else:
        rest = tokens[4:]
    env: dict[str, str] = {}
    for tok in rest:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        env[k] = _expand_vars(v)
    return env


def cmd_verify_demos(reg: M.Registry) -> int:
    n_bad = 0
    for name, s in reg.sequences.items():
        if not s.reproduces:
            continue
        script = REPO / s.reproduces
        if not script.exists():
            print(f"[verify] {name}: reproduced script missing: {s.reproduces}")
            n_bad += 1
            continue
        assembled = _assemble_env(s, reg)
        scripted = _parse_demo_script(script)
        # BATON_MODE_FILE resolves to the same clone-relative scratch path on both sides
        # (manifest "$MODEF" and the script's $MODEF expand via _SCRIPT_VARS); normalise nothing.
        only_a = {k: assembled[k] for k in assembled if k not in scripted}
        only_s = {k: scripted[k] for k in scripted if k not in assembled}
        diff = {k: (assembled[k], scripted[k]) for k in assembled
                if k in scripted and assembled[k] != scripted[k]}
        if not only_a and not only_s and not diff:
            print(f"[verify] {name:<16} MATCH  ({len(assembled)} env keys)  == {s.reproduces}")
        else:
            n_bad += 1
            print(f"[verify] {name:<16} MISMATCH  == {s.reproduces}")
            for k, (a, b) in diff.items():
                print(f"          differ  {k}:  manifest={a!r}  script={b!r}")
            for k, v in only_a.items():
                print(f"          only-in-manifest  {k}={v!r}")
            for k, v in only_s.items():
                print(f"          only-in-script    {k}={v!r}")
    print("-" * 60)
    print(f"verify-demos: {n_bad} mismatch(es)")
    return 1 if n_bad else 0


# --------------------------------------------------------------------------- #
#  index (regenerate registry.json)
# --------------------------------------------------------------------------- #
def cmd_index(reg: M.Registry) -> int:
    out = {
        "_note": "Regenerated by `python skill_lib.py index` from filesystem discovery.",
        "version": 2,
        "skills": [
            {"name": m.name, "class": m.klass, "kind": m.kind, "method": m.method,
             "status": m.status, "motion_class": m.motion_class, "robots": m.robots,
             "manifest": str(m.path.relative_to(SKILLS_DIR)).replace("\\", "/")}
            for m in reg.skills.values()
        ],
        "sequences": [
            {"name": s.name, "status": s.status, "primary": s.primary, "skills": s.skills,
             "manifest": str(s.path.relative_to(SKILLS_DIR)).replace("\\", "/"),
             "reproduces": s.reproduces}
            for s in reg.sequences.values()
        ],
    }
    (SKILLS_DIR / "registry.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {SKILLS_DIR / 'registry.json'} ({len(reg.skills)} skills, {len(reg.sequences)} sequences)")
    return 0


# --------------------------------------------------------------------------- #
#  CLI dispatch (supports subcommands + legacy positional form)
# --------------------------------------------------------------------------- #
_SUBCOMMANDS = {"list", "show", "validate", "ghost", "preview", "train", "run",
                "sequence", "verify-demos", "index", "adapt", "blendable", "handover", "freeze"}


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    reg = M.Registry.discover()

    # legacy: --list, or bare '<name>' -> run/sequence
    if argv and argv[0] == "--list":
        return cmd_list(reg)
    if argv and argv[0] not in _SUBCOMMANDS and not argv[0].startswith("-"):
        # bare name: sequence if known, else run
        name = argv[0]
        rest = argv[1:]
        if name in reg.sequences:
            argv = ["sequence", *argv]
        elif name in reg.skills:
            argv = ["run", *argv]
        else:
            print(f"[skill] unknown '{name}'. Try: python skill_lib.py list", file=sys.stderr)
            return 2

    ap = argparse.ArgumentParser(prog="skill_lib.py", description="OmniSim skill library runner.")
    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("list")
    sp = sub.add_parser("show"); sp.add_argument("name")
    sub.add_parser("validate")
    gp = sub.add_parser("ghost"); gp.add_argument("lut", nargs="?"); gp.add_argument("--all", action="store_true"); gp.add_argument("--strict", action="store_true")
    pp = sub.add_parser("preview"); pp.add_argument("name"); pp.add_argument("--duration", type=int, default=30); pp.add_argument("--dry-run", action="store_true")
    tp = sub.add_parser("train"); tp.add_argument("name"); tp.add_argument("--iters", type=int, default=300); tp.add_argument("--gui", default="headless"); tp.add_argument("--dry-run", action="store_true")
    rp = sub.add_parser("run"); rp.add_argument("name"); rp.add_argument("--duration", type=int, default=60); rp.add_argument("--gui", default="gui"); rp.add_argument("--throw", action="store_true"); rp.add_argument("--dry-run", action="store_true")
    qp = sub.add_parser("sequence"); qp.add_argument("name"); qp.add_argument("--duration", type=int, default=None); qp.add_argument("--gui", default=None); qp.add_argument("--dry-run", action="store_true")
    sub.add_parser("verify-demos")
    sub.add_parser("index")
    adp = sub.add_parser("adapt"); adp.add_argument("name"); adp.add_argument("--to-nb", type=int, required=True); adp.add_argument("--out", default=None)
    blp = sub.add_parser("blendable"); blp.add_argument("a"); blp.add_argument("b")
    hop = sub.add_parser("handover"); hop.add_argument("name")
    frp = sub.add_parser("freeze"); frp.add_argument("name"); frp.add_argument("--from", dest="from_env", default=None); frp.add_argument("--checkpoint", default=None)

    args = ap.parse_args(argv)
    if args.cmd in (None, "list"):
        return cmd_list(reg)
    if args.cmd == "show":
        return cmd_show(reg, args.name)
    if args.cmd == "validate":
        return cmd_validate(reg)
    if args.cmd == "ghost":
        import ghost_lut
        return ghost_lut.main(([args.lut] if args.lut else []) + (["--all"] if args.all else []) + (["--strict"] if args.strict else []))
    if args.cmd == "preview":
        return cmd_preview(reg, args.name, args.duration, args.dry_run)
    if args.cmd == "train":
        return cmd_train(reg, args.name, args.iters, args.gui, args.dry_run)
    if args.cmd == "run":
        return cmd_run(reg, args.name, args.duration, args.gui, args.dry_run, args.throw)
    if args.cmd == "sequence":
        return cmd_sequence(reg, args.name, args.duration, args.gui, args.dry_run)
    if args.cmd == "verify-demos":
        return cmd_verify_demos(reg)
    if args.cmd == "index":
        return cmd_index(reg)
    if args.cmd == "adapt":
        return cmd_adapt(reg, args.name, args.to_nb, args.out)
    if args.cmd == "blendable":
        return cmd_blendable(reg, args.a, args.b)
    if args.cmd == "handover":
        return cmd_handover(reg, args.name)
    if args.cmd == "freeze":
        return cmd_freeze(reg, args.name, args.from_env, args.checkpoint)
    return cmd_list(reg)


if __name__ == "__main__":
    sys.exit(main())
