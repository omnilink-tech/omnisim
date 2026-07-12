# scripts/harness/ — agent-facing validation harness

Long-running HTTP service on `127.0.0.1:6789` that wraps a headless OmniSim subprocess and injects a generic supervisor controller into whatever world it loads. Lets a coding agent author and iterate on `.wbt` files in a tight loop — load → screenshot → inspect scene tree → adjust camera → check exposure → hot-reload — without ever launching the desktop GUI.

| File | Purpose |
|---|---|
| [`omnisim_harness.py`](omnisim_harness.py) | The HTTP service. Run directly or via `python scripts/dev/omnisim_dev.py harness`. |
| [`diagnostic_codes.py`](diagnostic_codes.py) | Free-text-stderr → structured-code mapper used by `/world/load` and `/world/diagnostics`. Anchored in real `WbLog::error` / `WbLog::warning` call sites; unmatched lines pass through as `code: "UNKNOWN"`. |

The supervisor controller that the harness injects lives at [`projects/default/controllers/harness_supervisor/`](../../projects/default/controllers/harness_supervisor/). Tests live at [`tests/harness/`](../../tests/harness/).

For the entry-level overview (startup, common loop, endpoint cheatsheet) see [`AGENTS.md` §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness).
