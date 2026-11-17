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

"""LearnManager -- runs the skill-learning pipeline for the arm bridge.

The bridge spawns

    python <repo>/projects/policies/skills/factory/learn_runner.py \
        --skill <recipe> --events jsonl [--param k=v ...]

and reads stdout line-by-line as JSON events (the fixed contract):

    {"ev":"stage","stage":"design|validate|train|certify",
     "state":"running|pass|fail","detail":"<short line>","pct":<0-100>}
    {"ev":"log","line":"<raw log line>"}
    {"ev":"done","ok":true|false,"verb":"toss",
     "exec":"<abs path to learned_skill.json>","summary":"<one line>"}

On done(ok) the learned_skill.json is loaded and handed to `on_skill` so
the bridge can register the new verb. This class is deliberately
independent of the Webots Robot loop: it owns only a subprocess, a
reader thread, an ordered event store, and a condition variable that
SSE clients (and tests) can wait on. The bridge glues it to chat via
the `on_chat` callback and to motion via `on_skill`.

Only ONE learn runs at a time -- `start()` politely refuses a second.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# controllers/omnilink_arm_bridge -> controllers -> demos -> samples ->
# projects -> <repo root>
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", "..", ".."))
_FACTORY_DIR = os.path.join(_REPO_ROOT, "projects", "policies", "skills", "factory")
DEFAULT_RUNNER = os.path.join(_FACTORY_DIR, "learn_runner.py")

# Stages in pipeline order (used for HUD ordering + pct fallback).
STAGES = ("design", "validate", "train", "certify")

# Built-in recipe registry: phrase keywords -> skill-learning recipe.
# The factory may ship more recipes on disk (recipes/*.json with
# {"recipe"|"name", "verb", "keywords"[...], "description"}); those are
# merged in at construction when the directory exists.
# Ships EMPTY on purpose: recipes are robot-specific and live with the factory
# (recipes/*.json next to learn_runner.py), so the generic bridge carries no
# arm-specific names. A clone without the factory simply has nothing to learn
# yet -- the chat says so honestly.
DEFAULT_RECIPES: Dict[str, Dict[str, Any]] = {}


def discover_recipes(factory_dir: str = _FACTORY_DIR) -> Dict[str, Dict[str, Any]]:
    """Merge DEFAULT_RECIPES with any recipes/*.json the factory ships.

    Read-only, best-effort: a missing or malformed factory dir just means
    the built-in registry is used.
    """
    recipes = {k: dict(v) for k, v in DEFAULT_RECIPES.items()}
    rdir = os.path.join(factory_dir, "recipes")
    try:
        names = sorted(os.listdir(rdir))
    except OSError:
        return recipes
    for fn in names:
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(rdir, fn), "r", encoding="utf-8") as f:
                d = json.load(f)
            rid = d.get("recipe") or d.get("name") or os.path.splitext(fn)[0]
            verb = d.get("verb")
            if not rid or not verb:
                continue
            recipes[str(rid)] = {
                "verb": str(verb),
                "keywords": [str(k).lower() for k in (d.get("keywords") or [verb])],
                "description": str(d.get("description") or verb),
            }
        except Exception:
            continue
    return recipes


def load_learned_skill(path: str) -> Dict[str, Any]:
    """Load + validate a learned_skill.json. Raises ValueError on a bad file.

    Shape: {"verb": str, "kind": "joint_traj", "dt": s, "traj": [[q...]...],
            "gripper_events": [[t, "open"|"close"], ...], "meta": {...}}
    """
    with open(path, "r", encoding="utf-8") as f:
        sk = json.load(f)
    if not isinstance(sk, dict):
        raise ValueError("learned skill is not a JSON object")
    verb = sk.get("verb")
    if not verb or not re.fullmatch(r"[a-z][a-z0-9_]*", str(verb)):
        raise ValueError(f"bad verb {verb!r} (want lowercase identifier)")
    if sk.get("kind") != "joint_traj":
        raise ValueError(f"unsupported skill kind {sk.get('kind')!r}")
    dt = sk.get("dt")
    if not isinstance(dt, (int, float)) or dt <= 0:
        raise ValueError(f"bad dt {dt!r}")
    traj = sk.get("traj")
    if (not isinstance(traj, list) or not traj
            or not all(isinstance(row, list) and row
                       and all(isinstance(v, (int, float)) for v in row)
                       for row in traj)):
        raise ValueError("bad traj (want non-empty list of joint rows)")
    gev = sk.get("gripper_events") or []
    clean_gev: List[Tuple[float, str]] = []
    for e in gev:
        try:
            t, op = float(e[0]), str(e[1])
        except Exception:
            continue
        if op in ("open", "close"):
            clean_gev.append((t, op))
    sk["gripper_events"] = sorted(clean_gev)
    sk.setdefault("meta", {})
    return sk


class LearnManager:
    """Owns the learn_runner subprocess + the ordered event stream.

    Callbacks (all invoked from the reader thread -- they must be
    thread-safe; the bridge's queue_window is):

        on_chat(kind, text)   kind in {"agent", "tool_ok", "tool_err", "system"}
        on_skill(verb, skill) called once per successful learn, with the
                              validated learned_skill dict
    """

    def __init__(self,
                 runner_path: Optional[str] = None,
                 python_exe: Optional[str] = None,
                 on_chat: Optional[Callable[[str, str], None]] = None,
                 on_skill: Optional[Callable[[str, Dict[str, Any]], None]] = None,
                 recipes: Optional[Dict[str, Dict[str, Any]]] = None) -> None:
        self.runner_path = runner_path or DEFAULT_RUNNER
        self.python_exe = python_exe or sys.executable or "python"
        self.on_chat = on_chat or (lambda kind, text: None)
        self.on_skill = on_skill or (lambda verb, skill: None)
        self.recipes = recipes if recipes is not None else discover_recipes()

        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self.events: List[Dict[str, Any]] = []   # every event, seq-stamped
        self._running = False
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self.active_recipe: Optional[str] = None
        self.last_verb: Optional[str] = None
        self.last_result: Optional[Dict[str, Any]] = None

    # ── recipe helpers ────────────────────────────────────────────

    def resolve_recipe(self, text: str) -> Optional[str]:
        """Map free text ('learn to toss the cube into the bin') to a recipe."""
        s = (text or "").lower()
        # Exact recipe id wins ("learn <recipe-id>").
        for rid in self.recipes:
            if rid.lower() in s:
                return rid
        for rid, r in self.recipes.items():
            for kw in r.get("keywords", []):
                if re.search(r"\b" + re.escape(kw) + r"\b", s):
                    return rid
        return None

    def learnable_summary(self) -> str:
        """Honest one-liner listing what can be learned."""
        if not self.recipes:
            return "no learnable recipes are installed"
        bits = [f"'{r['verb']}' ({rid}: {r.get('description', r['verb'])})"
                for rid, r in sorted(self.recipes.items())]
        return "I can currently learn: " + "; ".join(bits)

    # ── event store ───────────────────────────────────────────────

    def _record(self, evt: Dict[str, Any]) -> Dict[str, Any]:
        with self._cond:
            evt = dict(evt)
            evt["seq"] = len(self.events)
            self.events.append(evt)
            self._cond.notify_all()
        return evt

    def events_since(self, seq: int) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self.events[max(0, int(seq)):])

    def wait_for_events(self, seq: int, timeout: float = 15.0) -> List[Dict[str, Any]]:
        """Events with .seq >= seq; blocks up to `timeout` if none yet.

        Returns [] on timeout (the SSE loop turns that into a heartbeat)."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while len(self.events) <= seq:
                left = deadline - time.monotonic()
                if left <= 0:
                    return []
                self._cond.wait(left)
            return list(self.events[max(0, int(seq)):])

    # ── lifecycle ─────────────────────────────────────────────────

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._running

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "busy": self._running,
                "active_recipe": self.active_recipe,
                "last_verb": self.last_verb,
                "last_result": self.last_result,
                "events": len(self.events),
                "recipes": {rid: {"verb": r["verb"],
                                  "description": r.get("description", "")}
                            for rid, r in self.recipes.items()},
            }

    def start(self, recipe: str,
              params: Optional[Dict[str, Any]] = None) -> Tuple[bool, str]:
        """Spawn the learn runner. Returns (ok, human message)."""
        if recipe not in self.recipes:
            return False, (f"I don't know a recipe called '{recipe}'. "
                           + self.learnable_summary() + ".")
        if not os.path.isfile(self.runner_path):
            return False, ("The skill-learning runner is not installed "
                           f"({self.runner_path} not found), so I can't "
                           "learn anything yet.")
        with self._lock:
            if self._running:
                return False, (f"I'm already learning '{self.active_recipe}' — "
                               "one skill at a time. Ask me again when it "
                               "finishes (watch this chat or /hud).")
            self._running = True
            self.active_recipe = recipe
        cmd = [self.python_exe, self.runner_path,
               "--skill", recipe, "--events", "jsonl"]
        for k, v in (params or {}).items():
            cmd += ["--param", f"{k}={v}"]
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=_REPO_ROOT if os.path.isdir(_REPO_ROOT) else None,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,   # non-JSON lines become log events
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1)
        except Exception as e:
            with self._lock:
                self._running = False
                self.active_recipe = None
            return False, f"Could not start the skill-learning runner: {e!r}"
        with self._lock:
            self._proc = proc
        self._record({"ev": "start", "recipe": recipe,
                      "params": dict(params or {}), "at": time.time()})
        t = threading.Thread(target=self._reader, args=(proc, recipe),
                             name="omnilink-learn", daemon=True)
        with self._lock:
            self._thread = t
        t.start()
        verb = self.recipes[recipe]["verb"]
        return True, (f"Starting skill learning: learning '{verb}' "
                      f"(recipe {recipe}). I'll report each stage here — "
                      "design, validate, train, certify.")

    def shutdown(self) -> None:
        """Kill the child cleanly (world shutdown / controller teardown)."""
        with self._lock:
            proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass
            try:
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)

    # ── reader thread ─────────────────────────────────────────────

    def _reader(self, proc: subprocess.Popen, recipe: str) -> None:
        saw_done = False
        try:
            for raw in proc.stdout:  # type: ignore[union-attr]
                line = raw.strip()
                if not line:
                    continue
                try:
                    evt = json.loads(line)
                    if not isinstance(evt, dict) or "ev" not in evt:
                        raise ValueError
                except Exception:
                    self._record({"ev": "log", "line": line})
                    continue
                self._record(evt)
                ev = evt.get("ev")
                if ev == "stage":
                    self._relay_stage(evt)
                elif ev == "done":
                    saw_done = True
                    self._handle_done(evt, recipe)
                # "log" events are stored for SSE/HUD but not chatted.
        except Exception as e:
            self._record({"ev": "log", "line": f"[learn] reader error: {e!r}"})
        code = None
        try:
            code = proc.wait(timeout=10.0)
        except Exception:
            pass
        if not saw_done:
            msg = (f"The skill-learning runner exited (code {code}) without "
                   "finishing. Nothing was learned.")
            self._record({"ev": "done", "ok": False, "verb": None,
                          "exec": None, "summary": msg})
            self.on_chat("agent", msg)
            with self._lock:
                self.last_result = {"ok": False, "summary": msg}
        self._record({"ev": "exit", "code": code})
        with self._lock:
            self._running = False
            self.active_recipe = None

    def _relay_stage(self, evt: Dict[str, Any]) -> None:
        stage = str(evt.get("stage", "?"))
        state = str(evt.get("state", "?"))
        detail = str(evt.get("detail", "") or "")
        pct = evt.get("pct")
        pct_s = f" ({pct:.0f}%)" if isinstance(pct, (int, float)) else ""
        if state == "running":
            self.on_chat("tool_ok", f"learn.{stage}:running — {detail}{pct_s}")
        elif state == "pass":
            self.on_chat("tool_ok", f"learn.{stage}:PASS — {detail}{pct_s}")
        elif state == "fail":
            self.on_chat("tool_err", f"learn.{stage}:FAIL — {detail}{pct_s}")
        else:
            self.on_chat("tool_ok", f"learn.{stage}:{state} — {detail}{pct_s}")

    def _handle_done(self, evt: Dict[str, Any], recipe: str) -> None:
        ok = bool(evt.get("ok"))
        summary = str(evt.get("summary", "") or "")
        with self._lock:
            self.last_result = {"ok": ok, "summary": summary,
                                "verb": evt.get("verb"), "recipe": recipe}
        if not ok:
            self.on_chat("agent",
                         "Learning did not certify. " + (summary or "No summary."))
            return
        path = evt.get("exec") or ""
        try:
            skill = load_learned_skill(path)
        except Exception as e:
            self.on_chat("agent",
                         f"The pipeline certified but I couldn't load the "
                         f"learned skill ({path}): {e}. The verb is NOT "
                         "registered.")
            return
        verb = str(skill["verb"])
        with self._lock:
            self.last_verb = verb
        try:
            self.on_skill(verb, skill)
        except Exception as e:
            self.on_chat("agent",
                         f"Learned '{verb}' but registering it failed: {e!r}")
            return
        dur = float(skill["dt"]) * max(0, len(skill["traj"]) - 1)
        props = skill.get("meta", {}).get("props") \
            or skill.get("meta", {}).get("prop_hints")
        prop_s = ""
        if props:
            if isinstance(props, str):
                props = [props]
            prop_s = (" The motion expects: "
                      + ", ".join(str(p) for p in props) + ".")
        self.on_chat("agent",
                     f"Done — I learned '{verb}'. {summary} "
                     f"Say \"{verb} it\" (or just \"{verb}\") and I'll run the "
                     f"{dur:.1f} s motion.{prop_s}")


# ── SSE plumbing (used by the bridge's GET /factory/events) ──────────

def write_sse_stream(wfile, manager: LearnManager,
                     heartbeat_s: float = 15.0,
                     max_loops: Optional[int] = None) -> None:
    """Replay every recorded event, then stream new ones as SSE frames.

    Each frame: `id: <seq>` + `data: <json>`. Idle gaps emit `: heartbeat`
    comment frames so proxies/browsers keep the connection alive. Returns
    when the client disconnects (write raises). `max_loops` bounds the
    wait iterations for tests; None streams forever.
    """
    seq = 0
    loops = 0
    while True:
        evts = manager.wait_for_events(seq, timeout=heartbeat_s)
        if evts:
            out = []
            for e in evts:
                seq = int(e["seq"]) + 1
                out.append("id: %d\ndata: %s\n\n"
                           % (e["seq"], json.dumps(e, default=str)))
            wfile.write("".join(out).encode("utf-8"))
        else:
            wfile.write(b": heartbeat\n\n")
        wfile.flush()
        loops += 1
        if max_loops is not None and loops >= max_loops:
            return


# ── HUD page ─────────────────────────────────────────────────────────

def get_hud_html() -> str:
    """The HUD page: `_hud_page.HTML` when the sibling module ships,
    else the built-in minimal placeholder below."""
    try:
        import _hud_page  # sibling module, same dir (bridge puts it on sys.path)
        return _hud_page.HTML
    except Exception:
        pass
    try:  # package-style fallback (if this dir ever becomes a package)
        from . import _hud_page as _hp  # type: ignore
        return _hp.HTML
    except Exception:
        return PLACEHOLDER_HUD_HTML


PLACEHOLDER_HUD_HTML = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Skill learning — Learn HUD</title>
<style>
body{margin:0;background:#0b0b0c;color:#f3eddc;font-family:-apple-system,"Segoe UI",Arial,sans-serif;padding:24px;}
h1{font-size:18px;letter-spacing:.06em;text-transform:uppercase;color:#b8b1a0;}
.stage{display:flex;align-items:center;gap:10px;padding:10px 14px;margin:8px 0;border:1px solid #232327;border-radius:12px;background:#131316;}
.stage .nm{width:90px;font-weight:600;text-transform:capitalize;}
.stage .st{font-size:12px;color:#8a8472;}
.stage.running{border-color:#d6a72d;} .stage.running .st{color:#fbe283;}
.stage.pass{border-color:rgba(109,209,120,.5);} .stage.pass .st{color:#6dd178;}
.stage.fail{border-color:rgba(255,138,130,.5);} .stage.fail .st{color:#ff8a82;}
#bar{height:8px;border-radius:999px;background:#1b1b1e;overflow:hidden;margin:16px 0;}
#fill{height:100%;width:0%;background:linear-gradient(90deg,#d6a72d,#fbe283);transition:width .4s;}
#done{margin-top:14px;padding:12px 14px;border-radius:12px;display:none;}
#done.ok{display:block;background:rgba(109,209,120,.08);border:1px solid rgba(109,209,120,.35);color:#6dd178;}
#done.bad{display:block;background:rgba(255,138,130,.08);border:1px solid rgba(255,138,130,.35);color:#ff8a82;}
#log{margin-top:16px;font:11px/1.5 Menlo,Consolas,monospace;color:#8a8472;white-space:pre-wrap;max-height:220px;overflow-y:auto;border-top:1px solid #1b1b1e;padding-top:8px;}
</style></head><body>
<h1>Skill learning — live learn</h1>
<div id="bar"><div id="fill"></div></div>
<div id="stages"></div>
<div id="done"></div>
<div id="log"></div>
<script>
const ORDER=["design","validate","train","certify"], rows={};
const stagesEl=document.getElementById('stages');
ORDER.forEach(n=>{const d=document.createElement('div');d.className='stage';
  d.innerHTML='<span class="nm">'+n+'</span><span class="st">waiting</span>';
  rows[n]=d;stagesEl.appendChild(d);});
function logln(t){const l=document.getElementById('log');
  l.textContent+=t+'\n';l.scrollTop=l.scrollHeight;}
const es=new EventSource('/factory/events');
es.onmessage=(m)=>{let e;try{e=JSON.parse(m.data);}catch(err){return;}
  if(e.ev==='stage'){const r=rows[e.stage];if(r){r.className='stage '+e.state;
      r.querySelector('.st').textContent=e.state+(e.detail?(' — '+e.detail):'');}
    if(typeof e.pct==='number')document.getElementById('fill').style.width=e.pct+'%';}
  else if(e.ev==='done'){const d=document.getElementById('done');
    d.className=e.ok?'ok':'bad';
    d.textContent=e.ok?('Certified — new verb "'+(e.verb||'?')+'" is live. '+(e.summary||''))
                     :('Failed. '+(e.summary||''));
    if(e.ok)document.getElementById('fill').style.width='100%';}
  else if(e.ev==='log'){logln(e.line||'');}
  else if(e.ev==='start'){logln('learn started: '+(e.recipe||''));}};
</script></body></html>
"""
