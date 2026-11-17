"""Does the deterministic gate catch the MODEL's false actuations?

This is the architecture question stated as an experiment. If the model
interprets and a deterministic gate vets what it emits, the combination is
only worth building if the gate actually stops the model's mistakes.

Re-queries just the 8 cases the model got wrong, keeping the ARGS this
time, and runs each emitted frame through gate.check() exactly as the
deterministic path does at route.py:285.
"""
import json, math, pathlib, sys
sys.path.insert(0, "O:/omnisim/tests/benchmarks/interpbench")
sys.path.insert(0, "O:/omnisim/tests/benchmarks/langsoak")
sys.path.insert(0, "O:/omnisim/packages/omnisim-bridges/src")

import importlib.util as _ilu
_spec = _ilu.spec_from_file_location(
    "interpbench_run", "O:/omnisim/tests/benchmarks/interpbench/run.py")
ib = _ilu.module_from_spec(_spec); _spec.loader.exec_module(ib)
from corpus import CORPUS
from omnisim_bridges.interpret import Frame
from omnisim_bridges import gate

BAD = ["neg-a4", "quote-a1", "quote-a2", "retract-a1",
       "bounds-a1", "bounds-a2", "bounds-a3", "time-a2"]
by = {u.uid: u for u in CORPUS}

system, decls = ib.load_bridge_payload()
tok = ib.token()
rows, tin, tout = [], 0, 0
for uid in BAD:
    u = by[uid]
    resp = ib.call(tok, system, decls, u.text)
    um = resp.get("usageMetadata") or {}
    tin += int(um.get("promptTokenCount", 0)); tout += int(um.get("candidatesTokenCount", 0))
    calls = ib.tools_from(resp)
    frames = [Frame(tool=n, args=a, span=(0, 0), source="llm", rule="llm")
              for n, a in calls]
    rej = gate.check(u.text, frames)
    rows.append({"uid": uid, "text": u.text, "calls": calls,
                 "rejected": [(r.tool, r.rule, r.detail) for r in rej]})

print("=" * 70)
print("MODEL emits -> DETERMINISTIC GATE vets")
print("=" * 70)
caught = 0
for r in rows:
    ok = bool(r["rejected"])
    caught += ok
    print(f"\n{r['uid']}  {r['text'][:52]!r}")
    print(f"   model emitted : {[(n, a) for n, a in r['calls']]}")
    if ok:
        for t, rule, d in r["rejected"]:
            print(f"   GATE REFUSED  : {t} -- {rule}: {d}")
    else:
        print(f"   gate allowed  : NOT CAUGHT")
print()
print(f"gate caught {caught} of {len(rows)} model false actuations")
print(f"cost of this check: ${(tin*0.5 + tout*3.0)/1e6:.4f}")
pathlib.Path("O:/omnisim/tests/benchmarks/interpbench/gate_vs_model.json").write_text(
    json.dumps({"rows": rows, "caught": caught, "of": len(rows)}, indent=2, default=str),
    encoding="utf-8")
