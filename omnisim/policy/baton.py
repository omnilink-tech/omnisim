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

"""BATON -- OmniSim's public, morphology-free policy-switching library.

BATON hands a robot from one trained specialist to the next inside a single deploy run
(walk -> turn -> carry -> stand) by MORPHING the reference tables every downstream consumer
reads, and CROSSFADING the specialists' actions -- never snapping either. Every specialist is
evaluated every tick, so the incoming policy's recurrent state is warm when it takes over.

    THE WHOLE OF BATON'S ROBOT KNOWLEDGE IS THREE THINGS, AND ALL THREE ARE INJECTED:
      1. WHICH reference channels exist        -> host.channels
      2. WHEN a specialist may let go          -> gate(phase)      (the support model)
      3. HOW a policy is evaluated             -> host.act(...)    (torch? onnx? nothing?)
    Everything else here is morphology-free arithmetic.

⛔ WHAT THIS FILE IS NOT ANY MORE. Until 2026-07-13 BATON was ~170 lines living INSIDE
`g1_walk_recipe.py`'s deploy function, written out TWICE (course arbiter + schedule arbiter),
plus a third fossil copy in the `_legacy` recipe. The copies had drifted, and the drift was a
live bug: BATON_COLD_HIDDEN was implemented only in the schedule copy, so every box demo set
it and it never fired. Extracting it to one arbiter fixed that -- but the module still reached
straight into `world._wr_glut` / `_wr_armlut` / `_wr_elblut` / `_wr_refatt` / `_wr_h` and blended
a FIXED tuple of channels that included `arm` and `elb`. That is humanoid ANATOMY welded into a
sequencing algorithm: a quadruped has no elbow channel to blend, and its deploy has no `world`
at all (it runs as an ONNX controller in a separate process, with no torch).

So the host is now an INTERFACE (`BatonHost`), and BATON is a library any robot can import:

    class BatonHost(Protocol):
        channels: tuple[str, ...]        # the reference channels THIS robot blends
        def phase(self) -> float         # the gait clock the support gate reads
        def write_tables(self, eff: dict) -> None    # push the blended references home
        def act(self, sp: Specialist, obs) -> ndarray   # evaluate ONE specialist (host owns the runtime)
        def reset_hidden(self, sp: Specialist) -> None  # cold handover
        def log(self, msg: str) -> None

A specialist with `policy=None` is a DETERMINISTIC HOLD: zero residual, i.e. track its ghost
exactly. That is a legitimate quadruped `stand` -- a quad is statically stable on its nominal
stance and needs no learned policy (and no crane) to hold it. A biped is not, which is why the
G1's stand is a trained specialist. The library does not care either way.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any

try:
    import numpy as np
except ModuleNotFoundError:  # catalogue/graph tooling must import without a policy runtime
    class _MissingNumpy:
        def __getattr__(self, name):
            raise RuntimeError(
                "BATON action/table evaluation requires numpy in the controller interpreter")

    np = _MissingNumpy()


# Explicitly preserve the historical module surface through the compatibility
# ``from omnisim.policy.baton import *`` shim, including the two tested helpers.
__all__ = [
    "BatonState", "Specialist", "act", "always_ok", "biped_double_support",
    "gate_for", "lut_tables", "mode_at", "resolve_artifact", "setup", "step",
    "_blend", "_env_stem",
]


# ── the morphology seam #1: when may a specialist LET GO? ────────────────────
def biped_double_support(phase: float, tol: float = 0.15) -> bool:
    """True near the gait-family double-support phase (BOTH feet planted).

    0.65*2pi is the gait family's double-support phase; the modulo-pi fold accepts either
    half-cycle (left-lead or right-lead). Hand a biped over anywhere else and you hand it over
    on one foot, mid-swing.
    """
    ds = 0.65 * 2.0 * math.pi
    ph = (phase - ds) % math.pi
    return min(ph, math.pi - ph) <= tol


def always_ok(phase: float, tol: float = 0.15) -> bool:
    """No forbidden hand-over instant.

    ⛔⛔ DO NOT USE THIS FOR A TROTTING QUADRUPED. It was this module's quadruped default on the
    reasoning that "a statically stable body has a support polygon at all times" -- and a LIVE RUN
    REFUTED IT (2026-07-13). Handing a Go2 over at `walk:12` (a switch that lands mid-swing)
    froze the reference to a stance hold while a diagonal pair was airborne: the robot tripped,
    flipped onto its back at t=12.83 s, and then kept "walking" while inverted -- still scoring
    gmatch 0.92 against the ghost, because a pose metric cannot see that you are upside down.
    Schedules that happened to switch at `walk:10` and `walk:8` survived. That is phase
    dependence, i.e. exactly the thing a support gate exists to prevent.

    A trot with duty > 0.5 HAS four-foot support windows (twice per cycle) -- see
    go2_trot_gait's own docstring. The host must gate on them (the Go2 host asks the gait model
    for the per-leg swing weights and hands over only when all four feet are planted).

    This is left here for a host that genuinely has no forbidden instant (a static pose, a
    wheeled base). If you are handing over a LEGGED gait, supply a real gate.
    """
    return True


def gate_for(robot_class: str):
    """Return the only safe class-level default, and fail closed otherwise.

    Humanoids in this gait family share a calibrated double-support phase. A quadruped support
    window depends on its gait model (trot, pace, crawl, duty factor), so morphology alone cannot
    select a safe gate. Quadruped hosts must inject their gait-specific gate into :func:`step`.
    Returning ``always_ok`` here used to turn an unknown support model into permission to switch;
    the measured Go2 mid-swing flip documented above is why that fallback is now forbidden.
    """
    if robot_class == "humanoid":
        return biped_double_support
    raise ValueError(
        f"BATON has no safe default support gate for {robot_class!r}; "
        "the host must inject a gait-specific gate")


# ── the state ────────────────────────────────────────────────────────────────
@dataclass
class Specialist:
    name: str
    tables: dict                      # its reference channels (host-declared keys) + "vx"
    policy: Any = None                # OPAQUE to BATON. None = deterministic hold (zero residual).
    hidden: Any = None                # OPAQUE recurrent state (LSTM h/c, or None)
    primary: bool = False             # the primary policy: its action is passed in, not re-run


@dataclass
class BatonState:
    reg: dict[str, Specialist]
    active: str
    target: str
    src: dict                         # the morph SOURCE tables (re-snapshotted at each switch)
    morph: int = 30
    u: float = 1.0                    # morph progress 0..1 (1 = fully on `target`)
    switches: int = 0
    out: str | None = None            # the OUTGOING specialist, for the action crossfade
    sched: list = field(default_factory=list)
    period: float = 0.0


# ── construction ─────────────────────────────────────────────────────────────
def resolve_artifact(path: str) -> str:
    """Anchor a repo-relative artifact path at the install root.

    BATON_SPECIALISTS carries repo-relative checkpoint/lut paths: that is the
    portable form, and it is what the skill manifests and the hand-written demo
    scripts both emit. But a deploy controller's cwd is its OWN directory, so
    opening one verbatim raises FileNotFoundError — and under the engine on
    Windows that traceback goes to a sink nobody reads, surfacing only as
    "controller exited with status 1". Absolute paths are returned untouched, so
    callers that already resolved are unaffected.
    """
    if not path or os.path.isabs(path):
        return path
    root = os.environ.get("OMNISIM_HOME") or os.environ.get("WEBOTS_HOME")
    if root:
        anchored = os.path.join(root, path)
        if os.path.exists(anchored):
            return anchored
    return path


def lut_tables(gd: dict, vx: float, keymap: dict) -> dict:
    """Utility for hosts: pull a ghost lut's channels into a table dict. Absent channels -> None.

    `keymap` is the HOST's channel -> lut-key mapping. It is not the library's business which
    channels a body has: the humanoid maps `elb -> elbow_lut`, and a quadruped has no elbow at
    all. (This map used to live here, hardcoded with `arm_lut`/`elbow_lut` in it -- the last
    piece of humanoid anatomy inside the arbiter.)
    """
    out = {ch: (np.asarray(gd[key], np.float32) if key in gd else None)
           for ch, key in keymap.items()}
    out["vx"] = vx
    return out


def setup(host, primary_tables: dict, load_policy, geti) -> BatonState | None:
    """Build the specialist registry from the BATON_* env. None if BATON is not armed.

    BATON_SPECIALISTS = "name|ckpt|lut[|vx];..."   (pipe/semicolon: Windows paths carry colons)
    BATON_POLICY_B / BATON_LUT_B                   = the legacy two-specialist ("stand") spelling
    A specialist whose ckpt is "-" or "" is a DETERMINISTIC HOLD (policy=None, zero residual).

    `load_policy(ckpt) -> (policy, hidden)` is the host's business: torch checkpoint, ONNX
    session, or nothing at all. BATON never imports a runtime.
    """
    bp, bl = os.environ.get("BATON_POLICY_B"), os.environ.get("BATON_LUT_B")
    specs = os.environ.get("BATON_SPECIALISTS", "")
    if not ((bp and bl and os.path.exists(bp) and os.path.exists(bl)) or specs):
        return None
    reg = {"walk": Specialist("walk", primary_tables, policy=None, hidden=None, primary=True)}
    if not specs and bp and bl:
        specs = "stand|%s|%s|0" % (bp, bl)
    for entry in specs.split(";"):
        if not entry.strip():
            continue
        pp = entry.split("|")
        nm, ck, lu = pp[0].strip(), resolve_artifact(pp[1].strip()), resolve_artifact(pp[2].strip())
        vx = float(pp[3]) if len(pp) > 3 else 0.0
        gd = json.loads(open(lu).read())
        pol, hid = (None, None) if ck in ("", "-") else load_policy(ck)
        reg[nm] = Specialist(nm, host.tables_from_lut(gd, vx), policy=pol, hidden=hid)
    sched = []
    for seg in os.environ.get("BATON_SCHEDULE", "walk:8,stand:6").split(","):
        nm, s = seg.split(":")
        sched.append((nm.strip(), float(s)))
    st = BatonState(reg=reg, active=sched[0][0], target=sched[0][0],
                    src=dict(reg[sched[0][0]].tables), sched=sched,
                    period=sum(s for _, s in sched),
                    morph=max(1, geti("BATON_MORPH_TICKS", 30)))
    host.log("BATON armed: specialists=%s sched=%s morph=%d ticks (warm handover; holds=%s)"
             % (sorted(reg), os.environ.get("BATON_SCHEDULE", "walk:8,stand:6"), st.morph,
                [s.name for s in reg.values() if s.policy is None and not s.primary]))
    return st


# ── the arbiter: ONE copy, for every robot ───────────────────────────────────
def _blend(src: dict, dst: dict, u: float, k: str):
    a, b = src.get(k), dst.get(k)
    if a is None or b is None:
        return a if b is None else b
    return (1.0 - u) * a + u * b


def _env_stem(name: str) -> str:
    """Map a registry name to its opt-in BATON_<NAME>_* environment stem."""
    return "".join(c.upper() if c.isalnum() else "_" for c in name)


def step(host, st: BatonState, desired: str, tt: int, *, gate, geti,
         write_tables: bool = True) -> None:
    """One tick: gate -> switch -> morph -> blend the references and push them to the host.

    `write_tables=False` while a solo-swap specialist owns the context (e.g. the G1's turner
    runs a different cadence and obs width; writing the cyclic tables under it puts the obs
    reference block out of range -> NaN).
    """
    if st is None:
        return
    des = desired if desired in st.reg else "walk"

    if des != st.target:
        gate_ok = True
        # Only leaving a WALK needs a support window, and only once the previous morph finished.
        if st.target == "walk" and geti("BATON_DS_GATE", 1) and st.u >= 1.0:
            gate_ok = gate(host.phase())
        if gate_ok:
            dst = st.reg[st.target].tables
            keys = tuple(host.channels) + ("vx",)
            st.src = {k: _blend(st.src, dst, st.u, k) if k != "vx" else
                      (1.0 - st.u) * st.src["vx"] + st.u * dst["vx"] for k in keys}
            st.out = st.target if st.u >= 1.0 else (st.out or st.target)
            st.target = des
            # Some specialists need a gentler entry than the sequence-wide default.  Reverse
            # gaits are the motivating case: changing both travel direction and load state in
            # 30 ticks can inject a large transient even when each specialist is stable alone.
            # This is strictly opt-in, so existing BATON sequences retain bit-for-bit timing.
            st.morph = max(1, geti("BATON_%s_MORPH_TICKS" % _env_stem(des),
                                  geti("BATON_MORPH_TICKS", 30)))
            st.u = 0.0
            st.switches += 1
            if geti("BATON_COLD_HIDDEN", 0):
                host.reset_hidden(st.reg[des])      # enter COLD: drop the inherited memory
            host.log("BATON switch #%d (-> %s) at t=%d phase=%.2f morph=%d"
                     % (st.switches, des, tt, host.phase() % (2 * math.pi), st.morph))
            mf = os.environ.get("BATON_MODE_FILE")
            if mf:                       # visual props (the carry box) follow the arbiter's mode
                try:
                    open(mf, "w").write(des)
                except OSError:
                    pass

    st.u = min(1.0, st.u + 1.0 / float(st.morph))
    if st.u >= 1.0:
        st.active = st.target
    if not write_tables:
        return
    dst = st.reg[st.target].tables
    eff = {k: _blend(st.src, dst, st.u, k) for k in host.channels}
    eff["vx"] = (1.0 - st.u) * st.src["vx"] + st.u * dst["vx"]
    host.write_tables(eff)


def mode_at(st: BatonState, elapsed_s: float) -> str:
    """The scheduled mode at time t, from BATON_SCHEDULE ("walk:8,stand:6,walk:10").

    Morphology-free: it is arithmetic on a list of (name, seconds). The last segment HOLDS
    (the schedule does not loop) unless BATON_SCHEDULE_LOOP=1.
    """
    if not st.sched:
        return st.target
    t = float(elapsed_s)
    if os.environ.get("BATON_SCHEDULE_LOOP", "0").strip() == "1" and st.period > 0:
        t = t % st.period
    acc = 0.0
    for nm, dur in st.sched:
        acc += dur
        if t < acc:
            return nm
    return st.sched[-1][0]


def act(host, st: BatonState, obs, primary_action):
    """The action crossfade: every specialist runs on the SAME obs (all recurrent states stay
    warm), and the APPLIED action fades outgoing -> incoming with the same morph u.

    A `policy=None` specialist contributes a ZERO residual -- it tracks its ghost exactly. On a
    statically-stable body that is a perfectly good `stand`, and it needs no training.
    """
    if st is None:
        return primary_action
    acts = {}
    for nm, sp in st.reg.items():
        if sp.primary:
            acts[nm] = primary_action
        elif sp.policy is None:
            acts[nm] = np.zeros_like(primary_action)    # deterministic hold: track the ghost
        else:
            acts[nm] = host.act(sp, obs)
        # Policies produce residuals around their reference motion.  Per-specialist scaling is
        # useful at difficult live handovers without weakening the ghost itself or retraining a
        # policy that is already stable in isolated evaluation.  Default 1.0 preserves every
        # existing sequence.
        scale = float(os.environ.get("BATON_%s_ACTION_SCALE" % _env_stem(nm), "1.0"))
        acts[nm] = scale * acts[nm]
    out = st.out or st.target
    return (1.0 - st.u) * acts.get(out, primary_action) + st.u * acts.get(st.target, primary_action)
