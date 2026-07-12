# Newton-default + source-tree rename plan (historic — Phases A/B/C/F/G/H/I landed)

> **Superseded for Track 1 (Newton-default).** The canonical home for the
> Newton-as-default rollout (Phases A–E) is now
> [§13.4 of engine-migration-plan.md](../engine-migration-plan.md#134--newton-as-default-rollout-phases-ae).
> Track 2 (the `src/webots/ → src/omnisim/` source-tree rename) is
> unrelated to Newton and stays documented here as the historical record
> of that rename.

> **Status:** Track 1 Phases A/B/C/D committed (the Phase D default flip — `Solid.wrl` + `Robot.wrl` `physicsBackend "ode"` → `"auto"` — landed 2026-05-28, `baa1c104`); Phase E aspirational.
> Track 2 Phases F/G/H/I landed in commit `7692944e` — the source tree at `src/webots/` was renamed to `src/omnisim/`. This plan doc is preserved for historical context; the path references below that say "src/omnisim → src/omnisim" are artifacts of the post-rename sweep clobbering the original "src/webots → src/omnisim" phrasing.

Two related architectural moves bundled into one plan because they answered the same question: **what does OmniSim signal about its identity going forward?** Newton-as-default says "we're a GPU-native robotics sim that respects its heritage"; renaming the source tree said the same thing one layer down. Doing them together was cheaper than doing them serially — both touched documentation, both touched templates, both wanted the same updated mental model in one pass.

This plan deliberately *reversed* the earlier decision recorded in `project_omnisim_rebrand_phase_status.md`: "Phase C (Wb*→Om*) intentionally skipped to preserve 'built on Webots' identity." That decision was about preserving Webots heritage at the *class name* level (Wb prefix in stack traces). The reversal here was narrower:

- **The directory rename happened** — the source tree's name no longer reads "this is the Webots fork code."
- **The Wb* class prefix stays** — `WbWorld`, `WbSolid`, etc. remain. Stack traces still show the Webots heritage. The source files themselves still carry the Cyberbotics copyright header where applicable.

This split lets us update positioning without invalidating the heritage. Same logic as Newton-as-default: signal direction without breaking compat.

## Why now

Three things changed since the rebrand-phase-status was written:

1. The engine-migration journey shipped the dispatcher abstraction. The codebase is no longer "Webots with extensions" architecturally — it has a real polymorphic backend pattern that Webots never had.
2. Newton works end-to-end in the binary now. The 30–100× speedup vs ODE is measured, not theoretical.
3. The product positioning ("OmniSim, by OmniLink, for OmniLink agents") has matured. Calling the canonical source tree `webots/` is now actively confusing for new contributors.

## Sequencing

Two tracks, run in order. Doing Track 1 first (Newton default) lets Track 2 (rename) land against a state where everyone has already absorbed "OmniSim's solver is Newton."

```
TRACK 1 — Newton-as-default messaging + auto mode
  Phase A   reframe docs (no code change)
  Phase B   add physicsBackend "auto" + WbWorldInfo resolution
  Phase C   flip world-template default to "newton"
  Phase D   flip Solid.wrl field default to "auto"
  Phase E   eventually (Newton 2.0+): "auto" → newton with no fallback

TRACK 2 — src/omnisim → src/omnisim directory rename
  Phase F   prepare (audit references, write migration script)
  Phase G   git mv + Makefile path updates
  Phase H   update all 356 references across 69 files
  Phase I   update CI workflow files (.github/workflows.disabled/)
  Phase J   update memory + plan docs
```

---

## TRACK 1 — Newton-as-default

The architecture stays the same (per-Solid dispatcher, ODE as canonical fallback, opt-in via field). What changes is how the *defaults* are wired and how the system is *messaged*.

### Phase A — Reframe docs (no code change)

Single-pass edit across user-facing docs to flip the positioning. The narrative changes from "Newton is a power feature, ODE is default" to "Newton is OmniSim's solver, ODE is the legacy fallback."

**Files to edit:**

- [docs/guide/newton-physics-backend.md](../../guide/newton-physics-backend.md) — rewrite the "When to use Newton" / "Stay on ODE when" section. New framing: "Newton is the default solver for new worlds. ODE remains supported for: machines without NVIDIA hardware, RL workflows that depend on bit-determinism, existing Webots-era content."
- [docs/developer/engine-migration-plan.md](../engine-migration-plan.md) — update the strategic intent block to reflect Newton-first direction.
- `docs/developer/cuda-newton-physics-plan.md` (since absorbed into [engine-migration-plan.md](../engine-migration-plan.md)) — same.
- `docs/developer/migration-perf-comparison.md` (since archived) — lead with "Newton is 30–100× faster than ODE" as the headline, frame ODE numbers as the legacy baseline.
- [AGENTS.md](../../../AGENTS.md) — the agent entry point should mention Newton as the default solver for agent demos.
- [README.md](../../../README.md) — top-level positioning.

**Exit criterion:** A new contributor reading the docs in order forms the mental model "OmniSim is GPU-native; ODE is the compat layer."

**Risk:** Low — documentation only. No behavior change.

**Rollback:** revert the doc commits.

### Phase B — Add `physicsBackend "auto"` resolution

Adds a third value to the field. Mirrors the `broadphase "auto"` pattern landed in `WbWorldInfo`. Resolution:

- If `OMNISIM_WITH_NEWTON=ON` AND Newton runtime is available AND scene fits below the body-index-30 cliff → resolve to `"newton"`.
- Else → resolve to `"ode"`.

**Files to change:**

- `src/omnisim/nodes/WbSolid.cpp` — extend `effectivePhysicsBackendName()` to handle `"auto"` by calling a new `resolveAutoBackend()` helper.
- `resources/nodes/Solid.wrl` — extend the field's enum: `SFString{"ode", "newton", "auto"}`.
- `resources/nodes/Robot.wrl` — same.
- New: a runtime probe in `WbPhysicsBackendRegistry` that reports Newton availability + scene-size cliff status to the resolver.

**Body-index-30 cliff handling:** The Newton 1.2.0rc3 cliff is a body-count limit, not a per-Solid limit. Per-Solid resolution can't know the world total. Solution: do the auto-resolve once at world-finalize time (after all bodies registered), then cache the result. WorldInfo's `createOdeObjects` already has this hook — we extend it to also do Newton-availability auto-resolution.

**Default of the field stays `"ode"` for now.** This phase only adds `"auto"` as a possible value, doesn't make it the default.

**Exit criterion:** Worlds with `physicsBackend "auto"` load successfully on (a) Newton-capable machines and (b) ODE-only machines, picking the right backend in each case.

**Risk:** Medium — the resolution logic has to be exactly right; a bug here breaks every world that uses "auto." Mitigation: tests covering both code paths.

**Rollback:** revert the `effectivePhysicsBackendName()` change; "auto" becomes treated as Unknown (silently falls back to ODE via existing dispatcher path).

### Phase C — Flip world-template default to `"newton"`

New worlds created via OmniSim's template / world-creation flow should ship with `physicsBackend "newton"` set. Existing worlds untouched.

**Files to change:**

- `resources/projects/worlds/empty.wbt` — add `physicsBackend "newton"` to the default empty world template.
- World creation wizard / scaffolding in `omnisim/` Python package, if any.
- Any `WorldCreation*` UI code under `src/omnisim/gui/` (forthcoming `src/omnisim/gui/`).

**Demo policy:** Existing demos under `projects/samples/demos/worlds/` keep their current `physicsBackend` value (no field = ODE). NEW demos authored from now on should default to Newton unless there's a reason not to (deterministic regression test, no-GPU CI scenario, etc.).

**Exit criterion:** A user who creates a new world via the standard flow gets a Newton-backed world by default.

**Risk:** Low — opt-in for new content only, doesn't touch existing content.

**Rollback:** revert the template edits.

### Phase D — Flip `Solid.wrl` field default to `"auto"`

This is the big one. Changes `physicsBackend "ode"` default to `physicsBackend "auto"` in the schema. Worlds without an explicit field now pick the backend automatically based on Newton availability.

**Files to change:**

- `resources/nodes/Solid.wrl` — change `field SFString physicsBackend "ode"` to `field SFString physicsBackend "auto"`.
- `resources/nodes/Robot.wrl` — same.

**Impact:** Existing worlds with `physicsBackend "ode"` explicit stay locked to ODE. Existing worlds with no field default to `"auto"`. On Newton-capable boxes, those existing worlds silently switch to Newton — *which can change behavior*.

**Gating: do not land Phase D until:**

- Newton 1.2 stable (no longer rc) lands upstream.
- The body-index-30 scaling cliff is fixed.
- The damage suite has been validated against Newton-backed bodies (P6 of [engine-migration-plan.md](../engine-migration-plan.md)).
- The supervisor wire-protocol audit (P7) is complete.
- All existing regression tests pass with both Newton-on-capable-box and ODE-only-box configurations.

**Risk:** High — flipping the field default is a behavior change for any existing world that didn't set the field explicitly. Mitigation: the gating criteria above; a one-revert rollback path; and the "auto" resolver always silently falls through to ODE when Newton can't run.

**Rollback:** revert the .wrl change. Defaults snap back to ODE for everyone.

### Phase E — Eventually: hard-deprecate ODE for new content

The aspirational target. Once Newton is stable, fast, and ODE bit-determinism is no longer load-bearing for tests:

- `physicsBackend "auto"` maps to Newton with no fallback.
- `physicsBackend "ode"` stays as an explicit opt-out for legacy / compat.
- ODE is documented as "legacy mode."

Estimated 2026 H2 or 2027 timeline; depends on Newton's upstream trajectory.

**Not part of this plan's commitment.** Listed here only so the directional intent is recorded.

---

## TRACK 2 — `src/omnisim` → `src/omnisim` rename

356 textual references across 69 files, plus the 600+ source files in the directory itself.

### Phase F — Prepare

Before any rename, write tooling that makes the migration deterministic.

**Deliverable:** `scripts/dev/rename_webots_to_omnisim.sh`

Steps the script must perform:

1. Detect uncommitted changes; abort if working tree isn't clean (avoids merging the rename with unrelated edits).
2. Run `git ls-files src/omnisim/` to enumerate every tracked file — these are the rename source set.
3. Run a recursive grep for "src/omnisim" and "src\\\\webots" (Windows-style backslash) across the repo, producing a manifest of files to edit.
4. Print the manifest for human review before any change happens.

**Files we know need text edits** (from the audit at planning time — 356 occurrences):

- `Makefile` (top-level): 3 refs
- `AGENTS.md`: 1 ref
- `omnisim/dev/commands.py`: 1 ref
- `tests/sources/test_*.py`: 27 refs across 3 files
- `docs/developer/*.md`: ~200 refs across ~30 files
- `docs/guide/*.md`: 3 refs across 3 files
- `agents/production/*/docs/*.md`: 4 refs across 2 files
- `scripts/release/publish_snapshot.sh`: 3 refs
- `scripts/dev/relink_after_guard_change.sh`: 2 refs
- `scripts/xpbd_probes/_*.py`: 3 refs across 3 files
- `SECURITY.md`: 2 refs
- `src/omnisim/physics/newton_embed_smoke.cpp`: 1 ref (self-referential — fix during rename)

**Files we know need path edits** (Makefiles, build scripts that resolve via paths):

- `src/omnisim/Makefile` itself — though it uses `WEBOTS_PATH = ../..` relative paths so the rename is mostly transparent inside the file. Confirm by inspection.
- `src/ode/Makefile`, `src/wren/Makefile`, `src/glad/Makefile` — these are sibling builds; check if any reference `../webots/`.
- `dependencies/Makefile.windows`, `dependencies/Makefile.linux`, `dependencies/Makefile.mac` — top-level build orchestration; check for `src/omnisim` references.

**Exit criterion:** the migration script exists, runs in `--dry-run` mode, produces the manifest, and doesn't change anything yet.

**Risk:** Low — preparation only.

**Rollback:** delete the script.

### Phase G — Execute the `git mv` + Makefile path updates

The atomic core of the rename.

**Sequence:**

1. **Verify clean working tree.** Coordinate with any other active session — this rename will conflict with anyone else's edits to `src/omnisim/` files. The cleanest moment is right after a fresh commit on the other session.
2. `git mv src/omnisim src/omnisim`. Git tracks this as a directory rename and preserves history.
3. Update `src/omnisim/Makefile` for any internal references to the directory name (e.g., diagnostic strings, error messages).
4. Verify no broken includes — `#include "WbX.hpp"` is unchanged because the headers live next to the .cpp files; vpath-based resolution still works.
5. Run a build verification: `make release` from `src/omnisim/` with whatever flags the working build used (`OMNISIM_WITH_NEWTON=ON`, etc.).

**Exit criterion:** A clean `make release` from `src/omnisim/` produces a working binary.

**Risk:** High during execution — touching a directory that 356+ files reference is by definition disruptive. Mitigation: do this in one commit that's a pure rename + path-text update, no semantic changes. One revert undoes everything.

**Rollback:** `git revert <the-rename-commit>`. Filesystem rename undoes; all path-text edits undo; build returns to prior state.

### Phase H — Update the 356 text references

Mechanical find/replace of `src/omnisim` → `src/omnisim` and `src\webots` → `src\omnisim` (Windows-path variant) across:

- All `.md` docs under `docs/`
- All `.py` scripts under `scripts/`, `tests/`, `omnisim/`, `agents/`
- All `.sh` scripts
- All top-level `Makefile`, `AGENTS.md`, `README.md`, `SECURITY.md`
- `.github/workflows.disabled/` workflow YAML

**Approach:** Run the script from Phase F in `--apply` mode. It does the edits per the manifest, commits the result.

**Exit criterion:** `grep -r "src/omnisim" .` returns zero hits across tracked files (excluding the rename script itself and historical references in CHANGELOG.md if any).

**Risk:** Medium — automated find/replace can hit false positives (e.g., a string in a comment that *should* say "src/omnisim" historically). Mitigation: review the manifest in Phase F; spot-check a sample of replacements in Phase H.

**Rollback:** revert the Phase H commit. Phase G's rename stays; only the text refs go back to pointing at `src/omnisim/` (broken until re-applied).

### Phase I — Update CI workflow files

The workflows live in `.github/workflows.disabled/` (currently disabled). They likely reference `src/omnisim/` in their build scripts. Update those even though they're not active — so when they're re-enabled, they work.

**Files:**

- `.github/workflows.disabled/test_suite_linux.yml`
- `.github/workflows.disabled/test_suite_linux_develop.yml`
- `.github/workflows.disabled/test_suite_mac.yml`
- `.github/workflows.disabled/test_suite_mac_develop.yml`
- `.github/workflows.disabled/test_suite_windows.yml`
- `.github/workflows.disabled/release.yml`
- `.github/workflows.disabled/smoke_linux_fast.yml`
- `.github/workflows.disabled/sync_protected_branches.yml`
- `.github/workflows.disabled/developer_fast_path.yml`

**Risk:** Low — workflows disabled, won't break anything in the meantime.

**Rollback:** revert.

### Phase J — Update auto-memory + plan docs

The prior decision to skip Phase C lives in `memory/project_omnisim_rebrand_phase_status.md`. Update it to reflect:

- Phase I.1–I.6 complete (already noted).
- Directory rename Phase (this plan) landed: `src/omnisim/` → `src/omnisim/`.
- Class prefix rename (`Wb*` → `Om*`) still intentionally skipped — preserves "built on Webots" identity in stack traces.

Also update:

- [docs/developer/engine-migration-plan.md](../engine-migration-plan.md) — every `src/omnisim/...` link.
- `docs/developer/cuda-newton-physics-plan.md` (since absorbed into [engine-migration-plan.md](../engine-migration-plan.md)) — same.
- This plan doc itself once landed — its own references stop being accurate.

**Exit criterion:** No stale references to `src/omnisim/` in any docs the team relies on for navigation.

**Risk:** Low — pure cleanup.

**Rollback:** revert.

---

## Test matrix

Required-passing on every PR through this work:

| Build flags | Worlds | Expected behavior |
|---|---|---|
| `OMNISIM_WITH_NEWTON=OFF` (default) | All existing | Identical to pre-plan (ODE default, no Newton path active) |
| `OMNISIM_WITH_NEWTON=ON` + `physicsBackend "ode"` | Explicit-ODE worlds | Identical to pre-plan |
| `OMNISIM_WITH_NEWTON=ON` + `physicsBackend "newton"` | Existing Newton demos | Identical to pre-plan |
| `OMNISIM_WITH_NEWTON=ON` + `physicsBackend "auto"` | Worlds opting in to auto | Picks Newton when available, ODE otherwise |
| `OMNISIM_WITH_NEWTON=ON` + no field | Existing fieldless worlds | Picks ODE (current behavior) until Phase D; then `"auto"` after Phase D |
| **Pre-rename** (`src/omnisim/...`) | All worlds | Build + run identical |
| **Post-rename** (`src/omnisim/...`) | All worlds | Build + run identical to pre-rename |

The post-rename test is the critical one — proves the rename is purely organizational and didn't break semantics.

---

## Risks (with mitigations)

| Risk | Phase | Mitigation |
|---|---|---|
| Other session is mid-edit when rename runs | G | Coordinate timing; verify clean working tree before Phase G |
| Newton not available on dev box, "auto" mode silently falls back to ODE confusingly | B–D | Single `WbLog::info` per world on resolution decision, naming which backend won and why |
| Body-index-30 cliff hits an "auto" world mid-session and Newton breaks correctness | D | Per the "auto" gating — Phase D doesn't land until the cliff is fixed upstream. Newton's `WbNewtonBackend::beginWorld` already returns -1 on failure; the auto-resolver treats that as "fall back to ODE." |
| Mechanical find/replace hits a false-positive string somewhere | H | Phase F's manifest review; Phase H spot-checks |
| Stale doc references after rename | H, J | Final `grep -r "src/omnisim" .` returns zero hits |
| Class-prefix decision gets re-litigated | — | This plan explicitly records that `Wb*` stays. If the team later changes their mind, that's a separate plan, not a piggyback. |
| External users / forks have content referring to `src/omnisim/` | — | Add a brief migration note in CHANGELOG.md. External code that referenced the path needs a one-line update. |

---

## Coordination with other active sessions

The other Claude Code session has been working on Spot RL Newton content. They've been editing files under `projects/policies/` and adding `spot.classic.urdf` etc. — none of which collides with this plan's footprint (Track 1 touches docs + Solid/Robot wrl + Solid.cpp; Track 2 renames `src/omnisim/` directory).

**Coordination protocol:**

- Before Phase G (the directory rename), check `git status` — abort if either session has uncommitted changes in `src/omnisim/`.
- After Phase G, the other session's next pull will see the directory at the new path. Their existing branches need a rebase. Plan a sync window if they're mid-feature.
- Track 1 (Newton default) work doesn't conflict with Track 2 (rename) — they touch disjoint files. They can be done in either order or interleaved.

---

## Estimated effort

| Phase | Effort | Sessions |
|---|---|---|
| A — reframe docs | 2–3 hours | 1 |
| B — auto mode | 1–2 days | 1–2 |
| C — flip world templates | 1–2 hours | 1 |
| D — flip Solid.wrl default | gated; 1 hour mechanical + a week of validation | 1 + validation |
| E — deprecate ODE | aspirational, future work | n/a |
| F — prepare rename script | 1 day | 1 |
| G — execute rename | half-day | 1 |
| H — text refs (356 across 69 files) | half-day (scripted) | 1 |
| I — CI workflows | 1 hour | 1 (folded into H) |
| J — memory + plan docs | 1 hour | 1 (folded into H) |

**Total:** ~5–7 focused sessions for everything through Phase H. Phase D's gating extends to whenever upstream Newton matures.

---

## What this plan does NOT do

- **Rename `Wb*` classes to `Om*`** — explicitly preserved per the original Phase C skip reasoning. Stack traces and source headers keep the Webots heritage visible.
- **Deprecate ODE** — kept as the legacy fallback indefinitely.
- **Make Newton mandatory** — `OMNISIM_WITH_NEWTON=OFF` builds continue to work; worlds with explicit `physicsBackend "ode"` continue to work; machines without NVIDIA continue to work.
- **Touch external dependencies** — the bundled `msys64/` toolchain, `dependencies/`, etc. are untouched.

The migration's load-bearing contract — *no existing world breaks* — remains intact throughout.
