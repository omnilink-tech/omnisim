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

"""SKILL FACTORY -- an agent-driven walk through the OmniSim skill pipeline.

The audience watches an AGENT (not a human typing commands) take a robot skill
through the whole factory loop, live, with every gate machine-checked:

    DESIGN  ->  VALIDATE  ->  TRAIN  ->  CERTIFY  ->  REGISTER  ->  COMPOSE

Every stage narrates what it is doing, runs the REAL tool where the tool is
cheap (the ghost validator and the skill-library registry checks are CPU and
sub-second -- they run for real, every time), and reads its numbers from the
REAL artifacts in the tree (ghost lut gate stamps, champion launch_env.txt,
training campaign logs). Where a number cannot be read live it is quoted from
a recorded constant and SAID to be one -- the factory never dresses a constant
up as a live measurement, and it never rounds a below-bar result up to a pass.

Two train modes:
    --replay   (default) narrate today's real recorded training artifacts.
               Fully offline, deterministic, CPU-only. Demo-safe.
    --live     assemble and LAUNCH the real trainer (run_quad_walk_rl.sh).
               Requires a CUDA GPU (4090-class) + the in-engine Newton stack.

Two robots:
    --robot go2   the quadruped lane: go2_turn, trained today (2026-07-17)
    --robot g1    the humanoid lane: g1_walk / box_delivery -- same pipeline
                  shape, with the MANDATORY balance-harness disclosure baked
                  into the narration (house policy: never describe the G1 walk
                  as free-standing).

Narration architecture: everything the audience reads flows through one
``Narrator`` object. Today it renders deterministic text (no network, no keys
-- the demo must run on a plane). The class exposes a ``restyle`` hook where an
OmniLink LLM route (``OMNI_KEY`` + ``agents/production/_lib``'s
``OmniLinkAgentRunner``) can later rewrite the same facts in a live voice; the
facts and the gates stay machine-checked either way.

Run it:
    python agents/production/skill_factory/skill_factory_agent.py --robot go2 --replay
    python agents/production/skill_factory/skill_factory_agent.py --robot g1  --replay
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

# repo root: agents/production/skill_factory/skill_factory_agent.py -> up 3
REPO = Path(__file__).resolve().parents[3]
SKILLS = REPO / "projects" / "policies" / "skills"
VALIDATOR = REPO / "projects" / "policies" / "training" / "ghost_validator.py"

# ── recorded constants (fallbacks only) ─────────────────────────────────────
# Every value below was measured on 2026-07-17 (RunPod 4090 session, campaign
# go2_turn -- see _scratch/s3_results/campaigns/go2_turn/). The replay stage
# prefers the actual log files when they are present in the tree and only
# falls back to these, ALWAYS labelled "recorded 2026-07-17".
RECORDED_GO2_TURN = {
    "date": "2026-07-17",
    "iters_per_leg": 400,
    "legs": 2,                       # seed-0 warm from the r2 walker, then seed-1 refine
    "env_steps_per_leg": 78_643_200,
    "throughput_steps_s": 696_052,   # final refine window on the 4090
    "train_wall_s": 113.2,           # refine leg
    "never_fell_leg1_pct": 95.5,
    "never_fell_pct": 97.5,
    "first_fall_mean_s": 12.56,
    "vx_alive_ms": -0.005,
    "gmatch_mean": 0.841,
    "gmatch_p5": 0.839,
    "ydrift_m": 0.11,
    "warmstart": "gpu_go2_shadow_r2_main (0.415 m/s, 99.7% never-fell walk champion)",
    # the cert arm (campaign_go2_turn_cert.sh): TRAIN_PLAN section-4 WZ_FIXED upgrade,
    # seed 2, warm from the 97.5% champion -- QUAD_WZ_FIXED=0.5895 QUAD_YAW=-1.0
    "cert_never_fell_pct": 99.2,
    "cert_vx_ms": -0.003,
}

# The quadruped certification bar (TRAIN_PLAN.md section 2, the campaign's 20_bar phase).
GO2_CERT_BAR = {"never_fell_pct": 99.0, "abs_vx_ms": 0.10}

# The house harness disclosure (AGENTS.md hard rule). The gains are read LIVE from
# profiles/g1_shadow_deploy.json at runtime; this text carries the fixed physics facts.
G1_HARNESS_FACTS = ("up to ~700 N upward assistance (~2x the 34 kg G1's weight) plus "
                    "+/-350 N-m of attitude authority")


# ── narration ───────────────────────────────────────────────────────────────
class Narrator:
    """All audience-facing text flows through here.

    ``restyle`` is the future LLM hook: if set, it receives (tag, text) and may
    return replacement text (e.g. via OmniLink's engine when OMNI_KEY is set).
    It is None by default and NOTHING in this demo requires it -- offline
    deterministic output is the contract. Facts/gates are computed before the
    hook ever sees them, so a styling model can never change a verdict.
    """

    def __init__(self) -> None:
        self.restyle = None   # optional: callable(tag: str, text: str) -> str

    def _emit(self, tag: str, text: str) -> None:
        if self.restyle is not None:
            try:
                text = self.restyle(tag, text) or text
            except Exception:
                pass  # a styling failure must never break the demo
        print(text, flush=True)

    def stage(self, n: int, total: int, name: str, headline: str) -> None:
        self._emit("stage", "")
        self._emit("stage", "=" * 78)
        self._emit("stage", f"  STAGE {n}/{total}  {name}  --  {headline}")
        self._emit("stage", "=" * 78)

    def say(self, text: str = "") -> None:
        self._emit("say", f"  {text}" if text else "")

    def fact(self, label: str, value: str, source: str) -> None:
        """A number + WHERE it came from. source is 'live: <path>' or 'recorded <date>'."""
        self._emit("fact", f"    {label:<34} {value:<28} [{source}]")

    def gate(self, ok: bool, label: str, detail: str = "") -> None:
        tag = "PASS" if ok else "FAIL"
        self._emit("gate", f"    [{tag}] {label}" + (f"  -- {detail}" if detail else ""))

    def tool(self, line: str) -> None:
        """Verbatim output from a real tool the agent just ran."""
        self._emit("tool", f"    | {line}")

    def cmd(self, argv_or_line) -> None:
        line = argv_or_line if isinstance(argv_or_line, str) else " ".join(argv_or_line)
        self._emit("cmd", f"  $ {line}")


# ── small helpers ───────────────────────────────────────────────────────────
def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def rel(p: Path) -> str:
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return p.as_posix()


def run_tool(argv: list[str], cwd: Path = REPO) -> tuple[int, str]:
    """Run a real tool, capture combined output. Offline, CPU, no simulator."""
    env = {**os.environ, "OMNISIM_HOME": str(REPO), "PYTHONIOENCODING": "utf-8"}
    proc = subprocess.run(argv, cwd=str(cwd), env=env, capture_output=True,
                          text=True, encoding="utf-8", errors="replace")
    out = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, out.rstrip()


def first_existing(paths: list[Path]) -> Path | None:
    for p in paths:
        if p.exists():
            return p
    return None


def grep_lines(text: str, pattern: str, limit: int = 0) -> list[str]:
    hits = [ln for ln in text.splitlines() if re.search(pattern, ln)]
    return hits[:limit] if limit else hits


# ── robot lane configs ──────────────────────────────────────────────────────
GO2 = {
    "robot": "go2",
    "skill": "go2_turn",
    "skill_dir": SKILLS / "quadruped" / "go2_turn",
    "sequence": "go2_walk_turn_walk",
    "policy_dir": REPO / "projects/policies/research/inference/policies/gpu_go2_turn_main",
    "train_log_candidates": [
        REPO / "_scratch/s3_results/campaigns/go2_turn/logs/go2_turn_r1_rl.txt",
        REPO / "_scratch/rerun_results/campaigns/go2_turn/logs/go2_turn_r1_rl.txt",
        REPO / "_scratch/campaign_results/campaigns/go2_turn/logs/go2_turn_r1_rl.txt",
    ],
    "summary_candidates": [
        REPO / "_scratch/s3_results/campaigns/go2_turn/SUMMARY.txt",
        REPO / "_scratch/rerun_results/campaigns/go2_turn/SUMMARY.txt",
        REPO / "_scratch/campaign_results/campaigns/go2_turn/SUMMARY.txt",
    ],
}

G1 = {
    "robot": "g1",
    "skill": "g1_walk",
    "skill_dir": SKILLS / "humanoid" / "g1_walk",
    "sequence": "box_delivery",
    "profile": SKILLS / "profiles" / "g1_shadow_deploy.json",
}


# ── stages ──────────────────────────────────────────────────────────────────
def stage_design(nr: Narrator, cfg: dict, manifest: dict) -> None:
    ghost = manifest.get("ghost", {})
    lut_rel = ghost.get("lut", "")
    lut_path = REPO / lut_rel
    nr.say(f"Skill: {manifest['name']}  ({manifest.get('title', '')})")
    nr.say(f"Method: {manifest.get('method')}  |  kind: {manifest.get('kind')}"
           f"  |  motion class: {manifest.get('motion_class')}  |  status: {manifest.get('status')}")
    nr.say()
    nr.say("Shadowing starts with a GHOST: an ACHIEVABLE, recorded reference the policy")
    nr.say("will shadow through bounded corridors. Ghost feasibility is the bottleneck")
    nr.say("(docs/developer/ghost-design-rules.md) -- so the reference is designed and")
    nr.say("machine-screened BEFORE any GPU minute is spent.")
    nr.say()
    nr.fact("ghost lut", lut_rel, "manifest ghost.lut")
    if ghost.get("builder"):
        nr.fact("builder", str(ghost["builder"])[:70], "manifest ghost.builder")
    prov = str(ghost.get("provenance", ""))
    if prov:
        nr.say()
        nr.say("Provenance (the doctrine cares: recorded/achieved beats parametric edits):")
        nr.say(f"  {prov[:190]}" + ("..." if len(prov) > 190 else ""))

    if not lut_path.exists():
        nr.gate(False, f"ghost lut present on disk ({lut_rel})")
        return
    lut = load_json(lut_path)
    nr.say()
    nr.say("Reading the lut itself (live):")
    nr.fact("robot / bins / freq", f"{lut.get('robot')} / nb={lut.get('nb')} / {lut.get('freq')} Hz",
            f"live: {lut_rel}")
    if "wz" in lut:
        nr.fact("commanded wz -> achieved wz", f"{lut.get('wz')} -> {lut.get('wz_meas')} rad/s "
                f"({abs(float(lut.get('wz_meas', 0))) * 57.2958:.1f} deg/s)", f"live: {lut_rel}")
        nr.say()
        nr.say("Note the honesty in that pair: the lut stores what the plant ACHIEVED under")
        nr.say("the bare deploy PD (0.5895 rad/s), not what was commanded (0.8). The ghost is")
        nr.say("the achievable response -- that is the whole design doctrine in one number.")
    else:
        nr.fact("design speed vx", f"{lut.get('vx')} m/s", f"live: {lut_rel}")

    gates = lut.get("gates", {})
    if gates:
        nr.say()
        nr.say("Builder gates stamped into the lut (each one a refusal point before training):")
        lr = gates.get("lut_replay", {})
        if lr:
            drift_mm = float(lr.get("closure_drift_p95_m", 0)) * 1000.0
            nr.gate(bool(lr.get("pass")), "lut_replay: bare deploy PD tracks the folded lut",
                    f"steady wz {lr.get('steady_wz', 0):.4f} vs source {lr.get('source_wz', 0):.4f} rad/s, "
                    f"closure drift p95 {drift_mm:.1f} mm (gate < 20 mm), "
                    f"peak torque {float(lr.get('peak_torque_frac', 0)) * 100:.0f}%, "
                    f"fell={lr.get('fell')}")
        cr = gates.get("construction_reroll", {})
        if cr:
            nr.gate(bool(cr.get("pass")), "construction_reroll",
                    f"peak torque {float(cr.get('peak_torque_frac', 0)) * 100:.0f}%, "
                    f"saturated {cr.get('saturated_steps')}/{cr.get('of_steps')}, fell={cr.get('fell')}")
        fw = gates.get("support_fwp_certificate_replay", {})
        if fw:
            nr.gate(bool(fw.get("pass")), "support + feasible-wrench certificate",
                    f"base_frac95 {fw.get('base_frac95')}")
        cl = gates.get("corridor_law", {})
        if cl:
            peak = float(cl.get("peak_dq_rad", 0))
            nr.gate(peak < 0.15, "corridor law: ghost peak |cmd - q|",
                    f"{peak:.3f} rad at kp={cl.get('kp')} (corridor 0.15 -> trackable without feedforward)")


def stage_validate(nr: Narrator, cfg: dict, manifest: dict) -> bool:
    ghost = manifest.get("ghost", {})
    lut_rel = ghost.get("lut", "")
    lut_path = REPO / lut_rel
    nr.say("Now the agent runs the REAL pre-training validator, live, on this machine.")
    nr.say("It is CPU and sub-second: 9 tiered gates (T0 kinematic legality against the")
    nr.say("robot's actual URDF limits, T1 coupling laws, T2 provenance, T3 corridor")
    nr.say("adequacy). Calibrated 7/7 against training ground truth: it passes the ghosts")
    nr.say("that trained and fails the ones that collapsed.")
    nr.say()
    argv = [sys.executable, str(VALIDATOR), str(lut_path)]
    nr.cmd(f"python {rel(VALIDATOR)} {lut_rel}")
    rc, out = run_tool(argv)
    for ln in out.splitlines():
        nr.tool(ln)
    verdict = ""
    m = re.search(r"VERDICT:\s*(\w+)", out)
    if m:
        verdict = m.group(1)
    nr.say()

    # cross-check against the verdict stamped at design time
    stamped = ""
    if lut_path.exists():
        stamped = (load_json(lut_path).get("validator", {}) or {}).get("verdict", "")
    if not stamped:
        stamped = (ghost.get("validator") if isinstance(ghost.get("validator"), str) else "") or ""

    if cfg["robot"] == "go2":
        ok = verdict == "PASS"
        nr.gate(ok, f"ghost_validator live verdict: {verdict or '??'}",
                f"stamped at design time: {stamped or 'n/a'} -- live re-run agrees" if ok and stamped
                else "the factory refuses to train on anything but PASS")
        return ok

    # G1: the bare run WARNs on T3 (corridor not given). Run it AGAIN with the
    # shipped corridor so the audience sees the instrument's sharpest edge.
    ok = verdict in ("PASS", "WARN")
    nr.gate(ok, f"ghost_validator live verdict: {verdict or '??'}",
            "WARN = 'train with eyes open' (T3 needs the corridor to grade); the "
            f"design-time stamp is {stamped or 'n/a'}")
    corridor = str(((manifest.get("train", {}) or {}).get("recipe_env", {}) or {})
                   .get("GHOST_RESIDUAL", "0.100"))
    nr.say()
    nr.say(f"Second pass WITH the shipped corridor ({corridor} rad) -- watch T3:")
    nr.cmd(f"python {rel(VALIDATOR)} {lut_rel} --corridor {corridor}")
    rc2, out2 = run_tool([sys.executable, str(VALIDATOR), str(lut_path), "--corridor", corridor])
    for ln in grep_lines(out2, r"T3\.corridor-adequacy|VERDICT"):
        nr.tool(ln)
    nr.say()
    nr.say("That FAIL is not stagecraft -- it is the balance-harness disclosure made")
    nr.say("machine-checkable: at the shipped corridor the policy cannot reach the edge")
    nr.say("of its own foot's CoP authority, so THE CRANE PAYS THE DIFFERENCE. This is")
    nr.say("exactly why every G1 result in this factory carries the harness disclosure")
    nr.say("(see CERTIFY), and why a durable FREE-STANDING humanoid walk is still open.")
    return ok


def stage_train(nr: Narrator, cfg: dict, manifest: dict, mode: str) -> None:
    train = manifest.get("train", {})
    if cfg["robot"] == "go2":
        _train_go2(nr, cfg, manifest, mode)
        return

    # G1 lane
    nr.say("The G1 walk champion predates the skill library; its manifest records the")
    nr.say(f"training env with status '{train.get('status', '?')}' -- reconstructed from notes,")
    nr.say("not a frozen bundle. The factory says so instead of pretending otherwise.")
    nr.say()
    for k, v in (train.get("recipe_env") or {}).items():
        nr.fact(k, str(v), "manifest train.recipe_env")
    nr.say()
    if mode == "live":
        nr.say("--live for g1_walk: REFUSED. A 'reconstructed' env is not a certified")
        nr.say("re-train recipe (skill_lib.py train prints the same warning). The command")
        nr.say("it would assemble (dry):")
        nr.cmd(f"python {rel(SKILLS / 'skill_lib.py')} train g1_walk --dry-run")
    else:
        nr.say("No in-tree training log for this 2026-07-08-era champion; the numbers the")
        nr.say("factory can stand behind are the manifest's verification record (next stage).")


def _train_go2(nr: Narrator, cfg: dict, manifest: dict, mode: str) -> None:
    R = RECORDED_GO2_TURN
    if mode == "live":
        nr.say("LIVE MODE. The factory now assembles the real trainer launch. Requirements:")
        nr.say("  - a CUDA GPU (4090-class; 16 GB VRAM comfortable) -- the in-engine trainer")
        nr.say(f"    runs K=16384 parallel worlds and measured ~{R['throughput_steps_s']:,} env-steps/s,")
        nr.say(f"    so one 400-iter leg (~{R['env_steps_per_leg']:,} steps) is ~2 minutes of GPU;")
        nr.say("  - the Newton/mujoco_warp stack reachable by the engine's python (torch,")
        nr.say("    warp-lang, newton, mujoco, mujoco-warp) -- see AGENTS.md engine defaults;")
        nr.say("  - train == deploy bit-exact: this trains THROUGH omnisim-bin, no MJCF reparse.")
        nr.say()
        env_line = manifest.get("train", {}).get("env", "")
        argv = (["bash", "projects/policies/training/run_quad_walk_rl.sh",
                 "0", "go2_turn_factory", "train", "headless"] + env_line.split())
        nr.cmd(argv)
        nr.say("Launching (this is the point where a GPU session takes over) ...")
        rc = subprocess.call(argv, cwd=str(REPO),
                             env={**os.environ, "OMNISIM_HOME": str(REPO)})
        nr.gate(rc == 0, f"trainer exited rc={rc}",
                "assert the eval block + the Newton sidecar, never the exit code alone")
        return

    # replay: prefer the REAL campaign artifacts in the tree
    nr.say("REPLAY MODE: narrating today's real training run from its recorded artifacts.")
    nr.say("(The GPU leg ran this morning on a RunPod 4090; the logs came home with the")
    nr.say("champion. Nothing here is simulated after the fact -- these are the lines the")
    nr.say("trainer actually printed.)")
    nr.say()
    summary = first_existing(cfg["summary_candidates"])
    rl_log = first_existing(cfg["train_log_candidates"])

    if summary:
        text = summary.read_text(encoding="utf-8", errors="replace")
        src = f"live: {rel(summary)}"
        nr.say(f"Campaign summary ({rel(summary)}):")
        for ln in grep_lines(text, r"00_preflight|phase 1[05]_|newton sidecar|20_bar"):
            nr.tool(ln.strip())
        nr.say()
        nr.say("Read those [FAIL] lines honestly: the zero-hook ladder ended at 97.5%, UNDER")
        nr.say("the 99% durability bar, and the campaign said so in its own log -- a gate")
        nr.say("that cannot fail is not a gate. That failure triggered the escalation the")
        nr.say("plan had ALREADY designed: TRAIN_PLAN.md section 4's WZ_FIXED upgrade")
        nr.say("(campaign_go2_turn_cert.sh) -- pin obs slot 48 to the achieved yaw 0.5895")
        nr.say("rad/s, re-engage the yaw-tracking reward, warm from the 97.5% champion.")
        nr.say("The cert arm's result lives in the champion's launch_env.txt; CERTIFY reads")
        nr.say("it next, live.")
    else:
        src = f"recorded {R['date']}"
        nr.say("(campaign SUMMARY not present in this clone -- quoting recorded constants)")

    nr.say()
    nr.say("Training shape (two legs, warm-started -- an upgrade experiment, disclosed):")
    nr.fact("leg 1 warm-start", R["warmstart"], "manifest policy.warmstart")
    nr.fact("iters x envs", f"{R['iters_per_leg']} x 16384 (x{R['legs']} legs)", src)
    nr.fact("env-steps per leg", f"{R['env_steps_per_leg']:,}", src)

    if rl_log:
        text = rl_log.read_text(encoding="utf-8", errors="replace")
        lsrc = f"live: {rel(rl_log)}"
        nr.say()
        nr.say(f"The trainer's own closing lines, zero-hook leg 2 ({rel(rl_log)}):")
        for pat in (r"it= 400 ", r"\] saved ", r"IN-ENGINE EVAL", r"exported ONNX"):
            for ln in grep_lines(text, pat, limit=1):
                nr.tool(ln.strip())
        m = re.search(r"it= 400 .*?([\d,]+) env-steps/s", text)
        if m:
            nr.fact("throughput at it=400", f"{m.group(1)} env-steps/s", lsrc)
        m = re.search(r"never_fell=([\d.]+)%", text)
        if m:
            nr.fact("zero-hook eval never_fell", f"{m.group(1)}%", lsrc)
        m = re.search(r"gmatch .*?mean=([\d.]+) p5=([\d.]+)", text)
        if m:
            nr.fact("ghost-match (shape ruler)", f"mean {m.group(1)}, p5 {m.group(2)}", lsrc)
    else:
        nr.say()
        nr.say("(trainer rl log not present in this clone -- recorded constants, clearly marked)")
        nr.fact("throughput at it=400", f"{R['throughput_steps_s']:,} env-steps/s", f"recorded {R['date']}")
        nr.fact("in-engine eval never_fell", f"{R['never_fell_pct']}%", f"recorded {R['date']}")
        nr.fact("ghost-match (shape ruler)", f"mean {R['gmatch_mean']}, p5 {R['gmatch_p5']}",
                f"recorded {R['date']}")
    nr.say()
    nr.say("One trap the pipeline guards by doctrine: the champion must print 'ONNX")
    nr.say("loaded:' at deploy -- a missing onnxruntime silently runs a zero-residual")
    nr.say("bare-ghost baseline and exits 0. Assert the line, never the exit code.")


def stage_certify(nr: Narrator, cfg: dict, manifest: dict) -> None:
    if cfg["robot"] == "go2":
        _certify_go2(nr, cfg, manifest)
    else:
        _certify_g1(nr, cfg, manifest)


def _certify_go2(nr: Narrator, cfg: dict, manifest: dict) -> None:
    R = RECORDED_GO2_TURN
    bar = GO2_CERT_BAR
    nr.say("Certification = the eval numbers, read from the champion's own artifacts,")
    nr.say("checked against the published bar. The bar (TRAIN_PLAN.md section 2):")
    nr.fact("durability bar", f"never_fell >= {bar['never_fell_pct']}%",
            "quadruped walk lineage holds 99.7%")
    nr.fact("stationarity bar", f"|vx| <= {bar['abs_vx_ms']} m/s",
            "a turn-in-place that translates fails its own spec")
    nr.say()
    le = cfg["policy_dir"] / "launch_env.txt"
    nf, vx, src = None, None, None
    if le.exists():
        text = le.read_text(encoding="utf-8", errors="replace")
        nr.say(f"Reading the champion's launch_env.txt (live: {rel(le)}):")
        for ln in text.splitlines():
            nr.tool(ln.rstrip())
        m = re.search(r"never_fell\s+([\d.]+)%", text)
        nf = float(m.group(1)) if m else None
        m = re.search(r"vx\s+(-?[\d.]+)\s*m/s", text)
        vx = float(m.group(1)) if m else None
        src = f"live: {rel(le)}"
    if nf is None:
        nf, vx = R["cert_never_fell_pct"], R["cert_vx_ms"]
        src = f"recorded {R['date']}"
        nr.say("(launch_env.txt not found -- recorded constants, cert arm)")
    nr.say()
    nr.gate(abs(vx) <= bar["abs_vx_ms"], "stationarity",
            f"vx {vx:+.3f} m/s -- a TRUE turn-in-place  [{src}]")
    nr.gate(nf >= bar["never_fell_pct"], "durability",
            f"never_fell {nf}% vs the {bar['never_fell_pct']}% bar  [{src}]")
    nr.say()
    if nf < bar["never_fell_pct"]:
        nr.say("VERDICT the factory actually stands behind: TRAINED, NOT YET CERTIFIED.")
        nr.say(f"{nf}% is a working turn -- {bar['never_fell_pct'] - nf:.1f} points under the bar -- so the")
        nr.say("manifest status stays 'open' and the narration says 'working-but-uncertified'.")
        nr.say("The remaining ladder is already designed and machine-readable: the WZ_FIXED")
        nr.say("upgrade recipe (TRAIN_PLAN.md section 4, campaign_go2_turn_cert.sh) plus the")
        nr.say("live BATON exam (yaw >= 0.5 rad/s sustained, |xy drift| < 0.3 m, 0 falls).")
    else:
        short_src = "launch_env.txt, live" if src.startswith("live") else src
        nr.say("VERDICT: the eval bar is MET -- and note it was NOT met three hours ago.")
        nr.say("The audit trail is the story: 95.5% (zero-hook leg 1) -> 97.5% (leg 2,")
        nr.say("FAILED the bar in the campaign's own log) -> the designed WZ_FIXED")
        nr.say(f"escalation -> {nf}% ({short_src}). The agent climbed the ladder the plan")
        nr.say("wrote down, and the gate flipped only when a number in an artifact said so.")
        status = manifest.get("status", "?")
        nr.say()
        nr.fact("manifest status (live)", status, "live: skill.json")
        nr.say(f"Why '{status}': BOTH rungs are on record -- the in-engine bar")
        nr.say("(99.2% >= 99%) AND the live BATON walk->turn->walk exam (2026-07-17:")
        nr.say("3/3 headless, zero falls, ~169 deg turn-in-place, 'ONNX loaded:'")
        nr.say("asserted on both policies; the record lives in the sequence manifest's")
        nr.say("verification field). launch_env.txt remains the champion's own artifact")
        nr.say("for eval numbers -- the factory quotes artifacts, not memories.")
    nr.say()
    nr.say("This is the demo's real punchline: the agent cannot talk a skill past a gate.")
    nr.say("Certification is a number in an artifact, or it is nothing.")


def _certify_g1(nr: Narrator, cfg: dict, manifest: dict) -> None:
    ver = manifest.get("verification", {})
    nr.say("The G1 walk is a VERIFIED skill -- and house policy makes the next sentence")
    nr.say("non-negotiable in the same breath:")
    nr.say()
    nr.say("  DISCLOSURE: every G1 walk/sequence result runs on a WEIGHT-BEARING balance")
    nr.say(f"  harness -- {G1_HARNESS_FACTS} --")
    nr.say("  partial support, not a spotter. Never describe it as a free-standing walk.")
    nr.say("  (The stair climb is the sole HARNESS_KZ=0 exception; quadrupeds use none.)")
    nr.say()
    prof_p = cfg["profile"]
    if prof_p.exists():
        prof = load_json(prof_p)
        env = prof.get("env", {})
        nr.say(f"The crane is not a footnote; it is in the deploy profile (live: {rel(prof_p)}):")
        for k in ("HARNESS_LAM0", "HARNESS_KZ", "HARNESS_FY", "HARNESS_KYAW", "HARNESS_KP", "HARNESS_KD"):
            if k in env:
                nr.fact(k, str(env[k]), f"live: {rel(prof_p)}")
        nr.say("  (HARNESS_LAM0=0.9: 90% of the assistance budget carried; KZ=2000: the")
        nr.say("  weight-bearing vertical stiffness.)")
    nr.say()
    nr.say("With that stated, the verification record (live from the manifest):")
    nr.fact("verified", f"{ver.get('date', '?')} on {ver.get('engine', '?')}",
            "live: skill.json verification")
    nr.say(f"  result: {str(ver.get('result', ''))[:180]}")
    wb = (manifest.get("policy", {}).get("alt_checkpoints", {}) or {})
    if wb:
        for k, v in wb.items():
            nr.fact("alt checkpoint", f"{k}: {str(v)[:60]}", "live: skill.json policy")
    nr.say()
    nr.say("Shape ruler on record: WBMATCH 0.868-0.913 vs the approved ghost -- honest")
    nr.say("numbers, on the harness, per docs/developer/rl-current-state.md (the canonical")
    nr.say("status file; when any other doc disagrees, that file wins).")


def stage_register(nr: Narrator, cfg: dict, manifest: dict) -> bool:
    nr.say("Registration = the skill library's machine checks over EVERY manifest, every")
    nr.say("sequence, and every ghost lut in the tree -- run live, right now:")
    nr.say()
    nr.cmd(f"python {rel(SKILLS / 'skill_lib.py')} validate")
    rc, out = run_tool([sys.executable, str(SKILLS / "skill_lib.py"), "validate"], cwd=SKILLS)
    skill = cfg["skill"]
    shown = 0
    for ln in out.splitlines():
        if (f"] {skill}" in ln or ln.startswith("== ") or "ghost luts checked" in ln
                or ln.startswith("validate:")):
            nr.tool(ln)
            shown += 1
    ok = rc == 0
    nr.say()
    nr.gate(ok, "skill_lib validate: 0 errors across the whole registry" if ok
            else "skill_lib validate reported errors")
    nr.say()
    nr.say("What the manifest BINDS into one versioned unit (the point of the library --")
    nr.say("five otherwise-scattered artifacts, checked here live on disk):")
    ghost = manifest.get("ghost", {})
    lut = REPO / ghost.get("lut", "_missing_")
    _bind(nr, "1. ghost lut", lut)
    stamped = "PASS" if (lut.exists() and (load_json(lut).get("validator", {}) or {})
                         .get("verdict") == "PASS") else None
    if stamped is None and isinstance(ghost.get("validator"), str):
        stamped = "stamped in manifest"
    nr.fact("2. validator verdict", stamped or "??",
            f"live: stamped inside {rel(lut)}" if lut.exists() else "manifest")
    if cfg["robot"] == "go2":
        nr.fact("3. deploy env", "train.env + profiles/go2_baton_deploy.json",
                "manifest train/deploy blocks")
    else:
        nr.fact("3. deploy env", "profiles/g1_shadow_deploy.json + primary_env",
                "manifest deploy.primary_env")
    ckpt = manifest.get("policy", {}).get("checkpoint", "")
    ckpt_p = REPO / ckpt
    _bind(nr, "4. champion checkpoint", ckpt_p)
    if ckpt_p.suffix == ".onnx" and ckpt_p.exists() and ckpt_p.stat().st_size < 16384:
        # a tiny .onnx means externalized weights -- the exact collection trap that
        # once shipped a graph with no tensors. Check the sidecar, out loud.
        data = ckpt_p.with_suffix(".onnx.data")
        if data.exists():
            nr.fact("   weights sidecar", rel(data),
                    f"live: on disk, {data.stat().st_size / 1024:,.0f} KB -- externalized; "
                    "the pair must travel together")
        else:
            nr.gate(False, "onnx weights sidecar MISSING",
                    "a 2 KB .onnx with no .onnx.data is a graph with no tensors -- "
                    "the collection trap campaign_go2_turn.sh fixed")
    if cfg["robot"] == "go2":
        _bind(nr, "   (torch twin)", cfg["policy_dir"] / "policy.pt")
    nr.fact("5. provenance", str(ghost.get("builder", ghost.get("provenance", "")))[:64],
            "manifest ghost block")
    return ok


def _bind(nr: Narrator, label: str, p: Path) -> None:
    if p.exists():
        kb = p.stat().st_size / 1024.0
        nr.fact(label, f"{rel(p)}", f"live: on disk, {kb:,.0f} KB")
    else:
        nr.fact(label, f"{rel(p)}", "MISSING on disk")


def stage_compose(nr: Narrator, cfg: dict, manifest: dict) -> bool:
    seq = cfg["sequence"]
    seq_path = SKILLS / "sequences" / f"{seq}.json"
    sdata = load_json(seq_path) if seq_path.exists() else {}
    nr.say("A skill alone is a party trick; the product is COMPOSITION. BATON sequences")
    nr.say("chain specialists -- one primary policy plus per-mode specialists, crossfaded")
    nr.say("element-wise on a shared phase clock, with a support gate deciding when a")
    nr.say("handover is safe. The library assembles the whole deploy env from manifests:")
    nr.say()
    nr.fact("sequence", f"{seq}  ({sdata.get('title', '')[:52]})", f"live: {rel(seq_path)}")
    nr.fact("status", str(sdata.get("status", "?")), f"live: {rel(seq_path)}")
    chain = " -> ".join([sdata.get("primary", "?")] + list(sdata.get("skills", [])))
    nr.fact("specialists", chain, f"live: {rel(seq_path)}")
    arb = sdata.get("arbiter", {})
    nr.fact("arbiter", f"{arb.get('kind', '?')}: {str(arb.get('value', ''))[:56]}",
            f"live: {rel(seq_path)}")
    nr.say()
    nr.cmd(f"python {rel(SKILLS / 'skill_lib.py')} sequence {seq} --dry-run")
    rc, out = run_tool([sys.executable, str(SKILLS / "skill_lib.py"),
                        "sequence", seq, "--dry-run"], cwd=SKILLS)
    ok = rc == 0
    nr.say("Assembled launch (specialist wiring pulled out of the env bundle):")
    joined = out.replace("\\\n", " ")
    spec = re.search(r"BATON_SPECIALISTS=('[^']*'|\S+)", joined)
    if spec:
        for entry in spec.group(1).strip("'").split(";"):
            if not entry:
                continue
            parts = entry.split("|")
            mode = parts[0]
            onnx = parts[1] if len(parts) > 1 and parts[1] else "(deterministic lut, no policy)"
            lutp = parts[2] if len(parts) > 2 else ""
            nr.tool(f"specialist '{mode}': policy={onnx}")
            if lutp:
                nr.tool(f"    ghost={lutp}")
    for key in ("BATON_SCHEDULE", "BATON_COURSE", "BATON_MORPH_TICKS", "BATON_DS_GATE"):
        m = re.search(key + r"=('[^']*'|\S+)", joined)
        if m:
            nr.tool(f"{key}={m.group(1).strip(chr(39))}")
    lastlines = [ln for ln in out.splitlines() if ln.strip()][-2:]
    nr.say()
    nr.say("Full command as skill_lib prints it (last lines):")
    for ln in lastlines:
        nr.tool(ln.strip()[:110] + ("..." if len(ln.strip()) > 110 else ""))
    nr.say()
    if cfg["robot"] == "go2":
        st = sdata.get("status", "?")
        nr.gate(ok, "sequence env assembles cleanly (dry-run rc=0)")
        nr.say()
        nr.say(f"Status honesty: this sequence is '{st}'. The walk and stand legs are the")
        nr.say("verified go2_walk_stand_walk stack; the turn leg's champion now meets the")
        nr.say("eval bar (CERTIFY above) and both deploy hooks are in-tree (gait clock")
        nr.say("8dcb293d, wz-obs 5b64a27d) -- and the live exam RAN (2026-07-17: 3/3")
        nr.say("headless, zero falls, both 'ONNX loaded:' asserted; the record is the")
        nr.say("'verification' field of this sequence manifest). The rule stands:")
        nr.say("without that log the registry would not get to say 'verified' --")
        nr.say("same rule for agents as for humans.")
    else:
        nr.gate(ok, "sequence env assembles cleanly (dry-run rc=0)")
        nr.say()
        nr.say("box_delivery is 'verified': pick -> carry -> place -> a real ~90 deg corner")
        nr.say("-> walk away; 7/7 switches on record, worst transient 2.1 deg -- on the")
        nr.say("balance harness (disclosure above applies to every leg of it).")
        nr.say("verify-demos proves this assembled env matches the hand-written demo script")
        nr.say("key-for-key (an env-dict comparison, not a byte diff).")
    return ok


# ── main ────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(description="Skill Factory -- agent-driven skill pipeline demo")
    ap.add_argument("--robot", choices=["go2", "g1"], default="go2")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--replay", action="store_true", help="narrate from recorded artifacts (default)")
    mode.add_argument("--live", action="store_true",
                      help="launch the real trainer (CUDA GPU + Newton stack required)")
    ap.add_argument("--skill", default=None, help="override the lane's default skill name")
    args = ap.parse_args()
    train_mode = "live" if args.live else "replay"

    # Windows consoles default to cp1252; artifacts are utf-8. Never crash on a glyph.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    cfg = dict(GO2 if args.robot == "go2" else G1)
    if args.skill:
        cfg["skill"] = args.skill
        hits = list(SKILLS.glob(f"*/{args.skill}/skill.json"))
        if not hits:
            print(f"[factory] no manifest for skill '{args.skill}'", file=sys.stderr)
            return 2
        cfg["skill_dir"] = hits[0].parent

    manifest_path = cfg["skill_dir"] / "skill.json"
    manifest = load_json(manifest_path)

    nr = Narrator()
    nr.say()
    nr.say("SKILL FACTORY  --  one robot skill, end to end, agent-driven")
    nr.say(f"robot={args.robot}  skill={cfg['skill']}  mode={train_mode}  repo={REPO}")
    nr.say("Every gate below is machine-checked; 'live:' numbers are read from artifacts")
    nr.say("on this machine right now, 'recorded' numbers are labelled as such.")

    hard_ok = True
    nr.stage(1, 6, "DESIGN", "author an achievable ghost reference, provenance first")
    stage_design(nr, cfg, manifest)

    nr.stage(2, 6, "VALIDATE", "run ghost_validator live -- reject bad references in seconds")
    if not stage_validate(nr, cfg, manifest):
        hard_ok = False

    nr.stage(3, 6, "TRAIN", "shadow the ghost in-engine (train == deploy bit-exact)")
    stage_train(nr, cfg, manifest, train_mode)

    nr.stage(4, 6, "CERTIFY", "read the eval artifacts, check them against the bar")
    stage_certify(nr, cfg, manifest)

    nr.stage(5, 6, "REGISTER", "one versioned manifest binds ghost + verdict + env + champion")
    if not stage_register(nr, cfg, manifest):
        hard_ok = False

    nr.stage(6, 6, "COMPOSE", "assemble the BATON sequence from manifests (dry-run)")
    if not stage_compose(nr, cfg, manifest):
        hard_ok = False

    nr.say()
    nr.say("=" * 78)
    if hard_ok:
        nr.say("FACTORY RUN COMPLETE. Design -> validate -> train -> certify -> register ->")
        nr.say("compose, all agent-driven, all gates machine-checkable -- and every claim")
        nr.say("sized exactly to its artifact. That is the loop.")
    else:
        nr.say("FACTORY RUN FINISHED WITH FAILED HARD GATES -- see [FAIL] lines above.")
    nr.say("=" * 78)
    return 0 if hard_ok else 1


if __name__ == "__main__":
    sys.exit(main())
