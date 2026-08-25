# Developer Docs Archive

Plans and notes whose work is substantially done, or that have been superseded by sibling docs. Kept here (rather than deleted) so the "why did we do it this way" record stays one click away from contributors who want to read the original plan alongside the resulting commits.

The live code is the authoritative record. Anything in here may be out of date.

> ## ⚠️ ODE IS GONE — treat every physics-backend plan in this directory as history
>
> `bdc02139` (2026-08-08) deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283
> lines). Several plans in here are written around a **two-backend** engine and describe
> `physicsBackend "ode"` as a supported opt-out, a "legacy fallback", or a correctness
> oracle. All of that was true when written and none of it is true now: Newton/MuJoCo is
> the only backend, and a Solid pinned to `"ode"` is not simulated at all.
>
> **Do not cite a doc in this directory as current behaviour, and do not implement a
> rollback step from one.** The live sources are
> [../rl-current-state.md](../rl-current-state.md) for RL, the
> [reference docs](../../reference/) for node fields, and [AGENTS.md](../../../AGENTS.md)
> for what the engine does today.
