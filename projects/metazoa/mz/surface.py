#!/usr/bin/env python3
"""Agent surface for Metazoa: PURE request routing + the sim-thread queue.

Nothing in this module imports the engine.  It is the part of the reef's
HTTP bridge that can be unit-tested without a simulator, lifted from the
threading contract of `projects/alife/controllers/terrarium_life`:

THREADING CONTRACT
------------------
The supervisor API is not thread-safe and every call is an IPC round trip.
The HTTP thread NEVER touches the supervisor: it validates the request
(`Router`), queues a command (`SimThreadQueue.submit`) and blocks on an
Event; the sim thread drains the queue once per tick (`drain`), executes,
and -- for anything that moves a body -- completes the command only after
the engine has stepped and the new pose has been READ BACK (`defer` +
`complete_due`), so the result is MEASURED, never the argument echoed.  A
command arriving while one is in flight is rejected with 409.  Reads
(`/census`) come from an immutable snapshot republished by the sim thread
(`CensusBox`).

Verbs (DESIGN.md "Agent surface"):
  GET  /capabilities
  GET  /census
  POST /light   {k, x, y}          move light patch k to (x, y)
  POST /split   {organism}         force a division
  POST /recruit {organism, cell}   ask an organism to recruit a free cell
  POST /dim     {factor}           scale the light charge rate
  GET  /healthz

Error envelopes: 400 `bad_json` / `missing_args` / `bad_type` / `bad_value`,
404 `unknown_verb`, 405 `method_not_allowed` (a known verb with the wrong
method -- more useful than a bare 404), 409 `busy`, 504 `timeout`.  Every
envelope carries `ok: false` and `error: <code>`.
"""
import copy
import json
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler

SERVICE = "omnisim-metazoa"
VERBS = {
    # path: (methods, required args -> type, description)
    "/capabilities": (("GET",), {}, "this document"),
    "/census": (("GET", "POST"), {}, "organisms, cells, genomes, charge, lineage (snapshot)"),
    "/light": (("POST",), {"k": int, "x": float, "y": float},
               "move light patch k to (x, y); result is the MEASURED patch position"),
    "/split": (("POST",), {"organism": str}, "divide an organism at its midpoint"),
    "/recruit": (("POST",), {"organism": str, "cell": int},
                 "have an organism approach and dock a free cell"),
    "/dim": (("POST",), {"factor": float}, "scale the light charge rate (0..%g)"),
    "/healthz": (("GET",), {}, "liveness; touches nothing"),
}
COMMAND_VERBS = ("light", "split", "recruit", "dim")
DIM_MAX = 10.0
VERIFY_TICKS_DEFAULT = 3


class Route:
    """The router's answer.  `kind` is one of:
      "reply"    -> send (code, body) now (capabilities, healthz, errors)
      "census"   -> send the current census snapshot
      "command"  -> marshal `verb` + `args` to the sim thread
    """
    __slots__ = ("kind", "code", "body", "verb", "args")

    def __init__(self, kind, code=200, body=None, verb=None, args=None):
        self.kind, self.code, self.body, self.verb, self.args = kind, code, body, verb, args

    def __repr__(self):
        return "Route(%s, %s, verb=%s, args=%s)" % (self.kind, self.code, self.verb, self.args)


def _err(code, error, **extra):
    body = {"ok": False, "error": error}
    body.update(extra)
    return Route("reply", code, body)


def _coerce(value, typ):
    """Strict-ish coercion: numbers for float/int (no bools, no NaN/inf),
    int must be integral, str must be str or int (organism ids may be
    either).  Returns (ok, coerced)."""
    if typ is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False, None
        v = float(value)
        return math.isfinite(v), v
    if typ is int:
        if isinstance(value, bool):
            return False, None
        if isinstance(value, int):
            return True, int(value)
        if isinstance(value, float) and math.isfinite(value) and value == int(value):
            return True, int(value)
        return False, None
    if typ is str:
        if isinstance(value, bool):
            return False, None
        if isinstance(value, (str, int)):
            return True, str(value)
        return False, None
    return True, value


class Router:
    """Pure: (method, path, body) -> Route.  Knows the reef's shape
    (`n_patches`, `arena`) only to validate ranges; it never touches state."""

    def __init__(self, n_patches=5, arena=18.0, dim_max=DIM_MAX):
        self.n_patches = int(n_patches)
        self.arena = float(arena)
        self.dim_max = float(dim_max)

    # -- helpers ----------------------------------------------------------
    @staticmethod
    def split_path(path):
        return (path or "/").split("?")[0] or "/"

    @staticmethod
    def parse_body(raw):
        """bytes/str/dict/None -> (ok, dict).  An empty body is `{}`."""
        if raw is None:
            return True, {}
        if isinstance(raw, dict):
            return True, dict(raw)
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", "replace")
        if not raw.strip():
            return True, {}
        try:
            obj = json.loads(raw)
        except ValueError:
            return False, None
        if not isinstance(obj, dict):
            return False, None
        return True, obj

    def capabilities(self, config=None, verify_ticks=VERIFY_TICKS_DEFAULT):
        verbs = {}
        for path, (methods, need, desc) in VERBS.items():
            if path == "/dim":
                desc = desc % self.dim_max
            verbs[path] = {"methods": list(methods),
                           "args": {k: t.__name__ for k, t in need.items()},
                           "description": desc}
        return {
            "ok": True,
            "service": SERVICE,
            "protocol": "OmniSim Wire Protocol",
            "verbs": verbs,
            "frames": {"position": "world ENU metres", "yaw": "rad about +z"},
            "measured_fields": ["position_measured", "charge", "members", "alive"],
            "errors": {"400": ["bad_json", "missing_args", "bad_type", "bad_value"],
                       "404": ["unknown_verb"], "405": ["method_not_allowed"],
                       "409": ["busy", "<verb-specific refusal>"], "504": ["timeout"]},
            "notes": [
                "Every result reports MEASURED state read back from the simulator "
                "after the engine stepped; arguments are never echoed as outcomes.",
                "Cell count is conserved: nothing is spawned or deleted at runtime. "
                "/split and /recruit change organisation, not matter.",
                "Commands complete only after the change has been stepped and read "
                "back (%d ticks). One command in flight at a time; a second is 409."
                % verify_ticks,
                "Census positions are the poses read this tick; a parked cell "
                "reports pos null, never a stale number.",
            ],
            "config": dict(config or {}, n_patches=self.n_patches, arena=self.arena,
                           dim_max=self.dim_max),
        }

    # -- routing ------------------------------------------------------------
    def route(self, method, path, body=None):
        method = (method or "GET").upper()
        path = self.split_path(path)
        if path not in VERBS:
            return _err(404, "unknown_verb", path=path, valid=sorted(VERBS))
        methods, need, _desc = VERBS[path]
        if method not in methods:
            return _err(405, "method_not_allowed", path=path, allowed=list(methods))
        if path == "/healthz":
            return Route("reply", 200, {"ok": True})
        if path == "/capabilities":
            return Route("reply", 200, self.capabilities())
        if path == "/census":
            return Route("census", 200)

        ok, args = self.parse_body(body)
        if not ok:
            return _err(400, "bad_json", path=path)
        missing = [k for k in need if k not in args]
        if missing:
            return _err(400, "missing_args", path=path, need=sorted(need), missing=missing)
        cmd = {}
        for k, typ in need.items():
            good, v = _coerce(args[k], typ)
            if not good:
                return _err(400, "bad_type", path=path, field=k, expected=typ.__name__,
                            got=type(args[k]).__name__)
            cmd[k] = v
        verb = path[1:]
        err = self._check_ranges(verb, cmd)
        if err is not None:
            return err
        return Route("command", 200, None, verb, cmd)

    def _check_ranges(self, verb, cmd):
        if verb == "light":
            if not 0 <= cmd["k"] < self.n_patches:
                return _err(400, "bad_value", field="k", got=cmd["k"],
                            valid="0..%d" % (self.n_patches - 1))
            h = self.arena / 2.0
            # Out-of-arena targets are accepted and CLAMPED by the executor
            # (the result reports `clamped`); only absurd values are refused.
            if abs(cmd["x"]) > 10 * h or abs(cmd["y"]) > 10 * h:
                return _err(400, "bad_value", field="x,y", got=[cmd["x"], cmd["y"]],
                            valid="within +/-%g" % (10 * h))
        elif verb == "dim":
            if cmd["factor"] < 0.0 or cmd["factor"] > self.dim_max:
                return _err(400, "bad_value", field="factor", got=cmd["factor"],
                            valid="0..%g" % self.dim_max)
        elif verb == "recruit":
            if cmd["cell"] < 0:
                return _err(400, "bad_value", field="cell", got=cmd["cell"], valid=">= 0")
        return None


# --------------------------------------------------------------------------
# sim-thread marshalling
# --------------------------------------------------------------------------

class Cmd:
    __slots__ = ("verb", "args", "done", "result", "due_tick", "finish", "abandoned")

    def __init__(self, verb, args):
        self.verb, self.args = verb, dict(args or {})
        self.done = threading.Event()
        self.result = None
        self.due_tick = None
        self.finish = None
        self.abandoned = False

    def __repr__(self):
        return "Cmd(%s, %s)" % (self.verb, self.args)


class SimThreadQueue:
    """Marshal commands from HTTP threads onto the sim thread.

    HTTP side:  `submit(verb, args, timeout)` -> (code, body).  409 while
                another command is in flight, 504 when the sim thread does
                not complete it in time (the command is then ABANDONED and
                the sim thread skips it -- unlike alife, where a timed-out
                command still executed later with nobody listening).
    Sim side:   `drain(executor, tick, max_n)` once per tick.  `executor(cmd,
                tick)` returns (code, body) for an immediate result, or None
                after arming a deferred completion with `defer(cmd, due_tick,
                finish)`; `complete_due(tick)` fires those whose tick has
                come.  Exceptions never escape the sim thread: they become a
                500 envelope for the caller.
    """

    def __init__(self):
        self._q = queue.Queue()
        self._busy = threading.Lock()
        self._pending = []
        self.stats = {"submitted": 0, "rejected_busy": 0, "timed_out": 0,
                      "executed": 0, "deferred": 0, "completed": 0}

    # -- HTTP thread --------------------------------------------------------
    def submit(self, verb, args, timeout=10.0):
        if not self._busy.acquire(blocking=False):
            self.stats["rejected_busy"] += 1
            return 409, {"ok": False, "error": "busy",
                         "detail": "another command is in flight; retry"}
        try:
            self.stats["submitted"] += 1
            c = Cmd(verb, args)
            self._q.put(c)
            if not c.done.wait(timeout):
                c.abandoned = True
                self.stats["timed_out"] += 1
                return 504, {"ok": False, "error": "timeout",
                             "detail": "sim thread did not complete within %.1fs "
                                       "(is the simulation paused?)" % timeout}
            return c.result
        finally:
            self._busy.release()

    @property
    def busy(self):
        if self._busy.acquire(blocking=False):
            self._busy.release()
            return False
        return True

    # -- sim thread ---------------------------------------------------------
    @staticmethod
    def defer(cmd, due_tick, finish):
        """Arm a read-back completion: `finish()` -> (code, body) runs on the
        sim thread at `due_tick`, after the engine has stepped."""
        cmd.due_tick, cmd.finish = int(due_tick), finish
        return None

    def drain(self, executor, tick, max_n=8):
        """Run up to `max_n` queued commands (bounded, so one burst cannot
        stall the sim), then fire due deferred completions.  Returns the
        number of commands executed this call."""
        n = 0
        for _ in range(max_n):
            try:
                c = self._q.get_nowait()
            except queue.Empty:
                break
            if c.abandoned:
                continue
            n += 1
            self.stats["executed"] += 1
            try:
                res = executor(c, tick)
            except Exception as exc:                 # noqa: BLE001  never kill the sim thread
                res = (500, {"ok": False, "error": "exception", "detail": repr(exc)})
            if res is None and c.finish is not None:
                self.stats["deferred"] += 1
                self._pending.append(c)
            else:
                if res is None:
                    res = (500, {"ok": False, "error": "no_result",
                                 "detail": "executor returned None without defer()"})
                c.result = res
                c.done.set()
        self.complete_due(tick)
        return n

    def complete_due(self, tick):
        if not self._pending:
            return 0
        still, fired = [], 0
        for c in self._pending:
            if c.abandoned:
                continue
            if tick >= c.due_tick:
                try:
                    c.result = c.finish()
                except Exception as exc:             # noqa: BLE001
                    c.result = (500, {"ok": False, "error": "exception", "detail": repr(exc)})
                c.done.set()
                fired += 1
                self.stats["completed"] += 1
            else:
                still.append(c)
        self._pending[:] = still
        return fired

    @property
    def pending(self):
        return len(self._pending)


class CensusBox:
    """The immutable read snapshot the sim thread republishes; HTTP threads
    only ever copy it."""

    def __init__(self):
        self._lock = threading.Lock()
        self._snap = {"ready": False}

    def publish(self, snap):
        snap = copy.deepcopy(snap)          # the sim thread keeps mutating its own
        snap["ready"] = True
        with self._lock:
            self._snap = snap

    def get(self):
        with self._lock:
            return copy.deepcopy(self._snap)


# --------------------------------------------------------------------------
# the HTTP handler (thin; everything above is what it calls)
# --------------------------------------------------------------------------

def make_handler(router, sim_queue, census_getter, timeout=10.0, capabilities=None):
    """Build the BaseHTTPRequestHandler subclass for `ThreadingHTTPServer`.
    `census_getter()` -> dict (a `CensusBox.get`); `capabilities` optionally
    overrides the router's default document (e.g. with the live config)."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"          # keep-alive

        def log_message(self, *a):             # keep the sim log readable
            pass

        def _send(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _dispatch(self, method, raw):
            r = router.route(method, self.path, raw)
            if r.kind == "census":
                return self._send(200, census_getter())
            if r.kind == "reply":
                if capabilities is not None and r.code == 200 and \
                        router.split_path(self.path) == "/capabilities":
                    return self._send(200, capabilities() if callable(capabilities)
                                      else capabilities)
                return self._send(r.code, r.body)
            code, body = sim_queue.submit(r.verb, r.args, timeout)
            return self._send(code, body)

        def do_GET(self):
            return self._dispatch("GET", None)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(n) if n else b""
            return self._dispatch("POST", raw)

    return Handler
