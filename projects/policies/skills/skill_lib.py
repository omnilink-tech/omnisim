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
    validate                   validate the versioned policy contract
    ghost   <lut|--all>        describe/validate ghost luts (-> ghost_lut.py)
    preview <skill>            launch the ghost hologram preview (design -> show -> agree)
    train   <skill>            assemble + launch the recipe trainer (reconstructed env; --go to run)
    run     <skill>            deploy a single skill solo
    sequence <name>            deploy a BATON sequence (box_delivery, turn_solo, ...)
    verify-demos               PROVE the assembled env == the reproduced shell scripts
    audit                      run every static release gate in one command
    benchmark ...              run/score the policy-block acceptance suite
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
import re
import shlex
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manifest as M          # noqa: E402
import skill_verbs as SV      # noqa: E402
import ghost_lut as GL  # noqa: E402
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
    "$G": "projects/policies/ghosts/g1",
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
    print("\nRun:  python -m omnisim policy sequence <name>   # a BATON demo")
    print("      python -m omnisim policy run <skill>        # one skill solo")
    print("      python -m omnisim policy preview <skill>    # the ghost hologram")
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
def _contract_luts(reg: M.Registry) -> list[Path]:
    """Ghosts in the release contract: versioned files plus every manifest reference.

    ``ghost --all`` intentionally audits experiments too. ``validate`` is the reproducible
    release gate and must not turn red because a developer has an untracked, half-built lut in
    the shared ghost store. In source archives without git metadata, every discovered lut is
    treated as versioned.
    """
    paths: set[Path] = set()
    try:
        run = subprocess.run(
            ["git", "ls-files", "--", "projects/policies/ghosts"], cwd=REPO,
            capture_output=True, text=True, check=False)
        if run.returncode == 0 and run.stdout.strip():
            paths.update(REPO / line for line in run.stdout.splitlines()
                         if line.strip().endswith(".json"))
        else:
            paths.update(GL.discover())
    except OSError:
        paths.update(GL.discover())
    for skill in reg.skills.values():
        if skill.ghost_lut:
            paths.add(REPO / skill.ghost_lut)
    return sorted(p for p in paths if p.exists())


def cmd_validate(reg: M.Registry) -> int:
    n_err = 0
    print("== registry discovery ==")
    if reg.issues:
        for issue in reg.issues:
            print(f"[ERR ] {issue}")
        n_err += len(reg.issues)
    else:
        print(f"[ok  ] {len(reg.skills)} unique skills, {len(reg.sequences)} unique sequences")
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
    # Release contract = every versioned lut across every robot + every manifest reference.
    # Use `ghost --all` for the deliberately broader audit of untracked experiments.
    luts = _contract_luts(reg)
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
    by_robot: dict[str, int] = {}
    for p in luts:
        r = GL.robot_of_path(p)
        by_robot[r] = by_robot.get(r, 0) + 1
    cover = ", ".join(f"{r}:{n}" for r, n in sorted(by_robot.items()))
    print(f"  {len(luts)} contract ghost luts checked, {g_err} errors  [{cover}]")
    n_err += g_err
    print("-" * 60)
    print(f"validate: {n_err} error(s)")
    return 1 if n_err else 0


# --------------------------------------------------------------------------- #
#  launch helpers
# --------------------------------------------------------------------------- #
def _bash_exe() -> str:
    """A bash that can run this repo's launchers.

    On Windows a bare "bash" resolves through PATH and often lands on
    System32\\bash.exe -- WSL. That bash sees /mnt/o/... paths, a Linux
    python3.10, and no Windows interpreter: the launcher then dies with
    'Permission denied: /mnt/o/omnisim/omnisim' (found live 2026-07-17).
    Prefer Git's bash explicitly; OMNISIM_BASH overrides.
    """
    exe = os.environ.get("OMNISIM_BASH")
    if exe:
        return exe
    if os.name == "nt":
        for cand in (r"C:\Program Files\Git\usr\bin\bash.exe",
                     r"C:\Program Files\Git\bin\bash.exe"):
            if Path(cand).exists():
                return cand
    return "bash"


def _resolve_bash_argv(argv: list[str]) -> list[str]:
    """Resolve a portable ``bash ...`` argv to the supported host shell."""
    if argv and argv[0] == "bash":
        bash = _bash_exe()
        # Git Bash launched non-interactively from a restricted Windows PATH
        # does not reliably provision /usr/bin, so launchers then lose dirname,
        # mkdir, date, sed, and tail.  ``bash -l script args...`` restores Git's
        # runtime and preserves every KEY=VALUE bundle argument without a
        # Windows command-line quoting trampoline.
        if os.name == "nt" and (len(argv) < 2 or argv[1] not in ("-c", "-lc")):
            argv = [bash, "-l", *argv[1:]]
        else:
            argv = [bash, *argv[1:]]
    return argv


def _run_or_print(argv: list[str], dry: bool, cwd: Path = REPO) -> int:
    argv = _resolve_bash_argv(argv)
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
            rc = _run_or_print(cmd, False)
            if rc:
                return rc
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
    world = m.ghost.get("preview_world") or "projects/policies/worlds/g1_ghost_preview.omniworld"
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
    env = M.assemble_solo_env(m, reg)
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

    # A `world` sequence (the quadruped BATON demo) deploys through its own launcher, which
    # opens the .wbt whose controller is baked in and reads the deploy env from the ENVIRONMENT
    # -- there is no recipe pymod to hook. The env is the same assembled bundle either way;
    # only the transport differs (exported vs KEY=VAL args).
    if s.deploy_kind == "world":
        argv = _resolve_bash_argv(M.render_run_argv(env, name, dur, g, seq=s))
        if dry:
            exports = " ".join(f"{k}={shlex.quote(env[k])}" for k in sorted(env))
            print(f"{exports} \\\n  " + " ".join(shlex.quote(a) for a in argv))
            return 0
        run_env = {**os.environ, **env, "OMNISIM_HOME": str(REPO),
                   "OMNISIM_PYTHON": sys.executable}
        # The engine launches Python controllers by resolving ``python`` from PATH. Codex and
        # packaged runtimes often invoke this CLI by absolute interpreter path without putting
        # that directory on PATH, so the shell launcher can run while every controller fails.
        py_dir = str(Path(sys.executable).resolve().parent)
        run_env["PATH"] = py_dir + os.pathsep + run_env.get("PATH", "")
        return subprocess.call(argv, cwd=REPO, env=run_env)

    pre: list[list[str]] = []
    if s.init_mode_file and env.get("BATON_MODE_FILE"):
        mf = env["BATON_MODE_FILE"]
        pre.append(["bash", "-lc", f'mkdir -p "$(dirname {shlex.quote(mf)})" && echo parked > {shlex.quote(mf)}'])
    return _bash_run_walk(env, name, dur, g, dry, pre=pre)


# --------------------------------------------------------------------------- #
#  verify-demos  (the equivalence proof)
# --------------------------------------------------------------------------- #
    # keys a launcher sets that are RUNTIME paths, not part of the deploy contract
_RUNTIME_KEYS = {"OMNISIM_HOME", "GO2_DEPLOY_LOG", "OMNISIM_DEPLOY_LOG", "RES_LOG",
                 "OMNISIM_LOG_PATH", "WARP_CACHE_PATH"}


def _parse_world_launcher(path: Path) -> dict[str, str]:
    """Extract the env a `world` launcher EXPORTS (the quadruped/controller deploy shape).

    The recipe demos pass their env as positional KEY=VAL args to run_walk_rl.sh; a world
    launcher instead `export`s it and then opens the .wbt. Same contract, different transport --
    so verify-demos has to read both, or the manifest layer can only ever verify the G1.

    This resolves the launcher's own shell variables (ROOT/GHOST/STAND/POLICY/SCHEDULE/...) and
    normalises $ROOT-absolute paths back to repo-relative, which is the form the manifests use.
    """
    text = path.read_text(encoding="utf-8").replace("\\\n", " ")
    vars_: dict[str, str] = {"ROOT": ""}
    env: dict[str, str] = {}

    def expand(v: str) -> str:
        v = v.strip().strip('"').strip("'")
        for _ in range(4):                       # resolve nested refs, cheaply
            for name, val in vars_.items():
                v = v.replace(f"${{{name}}}", val).replace(f"${name}", val)
            v = re.sub(r"\$\{([A-Z_0-9]+):-([^}]*)\}", r"\2", v)   # ${X:-default} -> default
        return v.lstrip("/")                      # $ROOT/projects/... -> projects/...

    for line in text.splitlines():
        line = line.strip()
        if line.startswith("#"):
            continue
        # KEY="v"  |  KEY='v'  |  KEY=v      -- with an optional trailing  # comment
        rhs = r"""(?:"([^"]*)"|'([^']*)'|([^\s#]*))\s*(?:#.*)?$"""
        m = re.match(r'^export\s+([A-Z_0-9]+)=' + rhs, line)
        if m:
            k = m.group(1)
            v = expand(next(g for g in m.groups()[1:] if g is not None))
            if k not in _RUNTIME_KEYS:
                env[k] = v
            vars_[k] = v
            continue
        m = re.match(r'^([A-Z_0-9]+)=' + rhs, line)          # a plain local assignment
        if m:
            raw = next(g for g in m.groups()[1:] if g is not None)
            if "$(" not in raw:
                vars_[m.group(1)] = expand(raw)
    return env


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
        # a `world` sequence's launcher EXPORTS its env; a `recipe` one passes KEY=VAL args
        scripted = (_parse_world_launcher(script) if s.deploy_kind == "world"
                    else _parse_demo_script(script))
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
    n_bad += _verify_solo_envs(reg)
    print("-" * 60)
    print(f"verify-demos: {n_bad} mismatch(es)")
    return 1 if n_bad else 0


def _verify_solo_envs(reg: M.Registry) -> int:
    """Every RL Shadowing skill's SOLO launch must be deployable: policy + ghost + a LIVE corridor.

    ⛔ THE HOLE THIS CLOSES (2026-07-13). verify-demos proved the manifests reproduce the demo
    scripts -- but only for SEQUENCES. Solo runs were never checked, and they were broken:

      * solo-swap skills (turn, climb) emitted the BATON swap keys ALONE -- no RES_POLICY, no
        GHOST_LUT_JSON, no GHOST_RESIDUAL,
      * cyclic carry/stand declared no GHOST_RESIDUAL at all,

    and GHOST_RESIDUAL DEFAULTS TO 0.0, which the trainer reads as `if gres > 0:` -> the entire
    corridor block is skipped. So `skill_lib run <skill>` was starting a Shadowing checkpoint with
    NO ghost to shadow and its corridor switched OFF -- fully out of distribution. It did not error;
    it just collapsed on the floor and looked like a broken policy. A guarantee that only covers the
    path that works is not a guarantee.
    """
    print("\n[verify] SOLO launch envs (policy + ghost + live corridor):")
    n_bad = 0
    for name, m in sorted(reg.skills.items()):
        if m.kind != "rl" or m.method != "shadowing" or m.deploy_run != "recipe":
            continue
        try:
            env = M.assemble_solo_env(m, reg)
        except Exception as e:
            n_bad += 1
            print(f"[verify] {name:<16} BROKEN  cannot assemble solo env: {e}")
            continue
        missing = [k for k in ("RES_POLICY", "GHOST_LUT_JSON", "GHOST_RESIDUAL") if not env.get(k)]
        gres = float(env.get("GHOST_RESIDUAL", 0) or 0)
        if missing:
            n_bad += 1
            print(f"[verify] {name:<16} BROKEN  solo launch missing {', '.join(missing)}"
                  f"  -> deploys with no ghost and/or corridors OFF")
        elif gres <= 0:
            n_bad += 1
            print(f"[verify] {name:<16} BROKEN  GHOST_RESIDUAL={gres} -> `if gres > 0` is False, "
                  f"the corridor block never runs")
        else:
            print(f"[verify] {name:<16} ok      corridor={gres:.3f}  ghost={os.path.basename(env['GHOST_LUT_JSON'])}")
    return n_bad


# --------------------------------------------------------------------------- #
#  index (regenerate registry.json)
# --------------------------------------------------------------------------- #
def _registry_payload(reg: M.Registry) -> dict:
    return {
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


def cmd_index(reg: M.Registry) -> int:
    out = _registry_payload(reg)
    (SKILLS_DIR / "registry.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {SKILLS_DIR / 'registry.json'} ({len(reg.skills)} skills, {len(reg.sequences)} sequences)")
    return 0


def cmd_benchmark(argv: list[str]) -> int:
    bench_dir = REPO / "projects" / "policies" / "benchmarks"
    sys.path.insert(0, str(bench_dir))
    import policy_bench  # noqa: PLC0415
    return policy_bench.main(argv)


def cmd_audit(reg: M.Registry) -> int:
    """One fail-closed release gate for the complete policy subsystem."""
    failures = 0
    print("# OmniSim policy-block audit\n")
    failures += int(cmd_validate(reg) != 0)
    print("\n== reproduced demo parity ==")
    failures += int(cmd_verify_demos(reg) != 0)

    print("\n== generated registry ==")
    expected = _registry_payload(reg)
    registry_path = SKILLS_DIR / "registry.json"
    try:
        current = json.loads(registry_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        print(f"[ERR ] cannot read {registry_path}: {exc}")
        failures += 1
    else:
        if current == expected:
            print(f"[ok  ] registry.json is current ({len(reg.skills)} skills, "
                  f"{len(reg.sequences)} sequences)")
        else:
            print("[ERR ] registry.json is stale; run: python skill_lib.py index")
            failures += 1

    print("\n== benchmark specification ==")
    bench_dir = REPO / "projects" / "policies" / "benchmarks"
    sys.path.insert(0, str(bench_dir))
    import policy_bench  # noqa: PLC0415
    bench_issues = policy_bench.validate_suite(registry=reg)
    for issue in bench_issues:
        print(f"[ERR ] {issue}")
    if not bench_issues:
        cases = policy_bench.cases_by_name(policy_bench.load_suite())
        verified = sum(c.get("status") == "verified" for c in cases.values())
        print(f"[ok  ] {len(cases)} thresholded benchmark case(s), {verified} with versioned PASS evidence")
    failures += int(bool(bench_issues))

    print("\n" + "=" * 60)
    print(f"policy-block audit: {'PASS' if not failures else 'FAIL'} ({failures} failed gate(s))")
    return 1 if failures else 0


# --------------------------------------------------------------------------- #
#  CLI dispatch (supports subcommands + legacy positional form)
# --------------------------------------------------------------------------- #
_SUBCOMMANDS = set(SV.names())


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
    # The verb table lives in skill_verbs.py so that `python -m omnisim policy`
    # -- the documented front door, which delegates here -- registers the SAME
    # subcommands instead of keeping a second hand-copied list.
    SV.register(sub)

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
    if args.cmd == "audit":
        return cmd_audit(reg)
    if args.cmd == "benchmark":
        return cmd_benchmark(args.benchmark_args)
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
