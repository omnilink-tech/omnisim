# Workflows — disabled

These CI workflows are inherited from upstream Webots and have not been
validated against the OmniSim source tree. GitHub only treats files inside
`.github/workflows/` as active — anything here is dormant text.

To re-enable one, move it back and confirm it passes on a real run:

```bash
git mv .github/workflows.disabled/<name>.yml .github/workflows/<name>.yml
```

## What is actually here

This list was wrong until 2026-08-27: it advertised `release.yml`,
`tests_doc.yml`, `test_suite_*.yml` and `sync_protected_branches.yml`, none of
which exist in this directory (`release.yml` was activated long ago and lives in
`.github/workflows/`). Four files are here:

- `smoke_linux_fast.yml` — **superseded, do not re-enable as written.** Replaced
  by the active [`linux-build.yml`](../workflows/linux-build.yml). Kept only as
  a record of the upstream shape. It carries four defects that would make it
  fail or, worse, pass dishonestly:
  1. It requests `actions/setup-python` **3.11**, while the engine embeds and
     links the system **3.10**. That is the "two interpreters" trap — it does not
     fail loudly, it makes ONNX deploys silently run a zero-residual baseline.
  2. It installs `linux_optional_compilation_dependencies.sh`, which pulls
     `python3.7-dev`…`python3.10-dev` from deadsnakes — the wrong dependency set,
     and broken on newer Ubuntu.
  3. It never installs the Newton physics runtime. Newton is the only backend, so
     the smoke suite would run with no physics at all.
  4. It never installs wgpu-native, so it cannot catch a broken renderer — and
     since the WREN deletion there is no second renderer to fall back to.
- `developer_fast_path.yml` — asserts a small set of dev-doc files exist.
- `tests_sources.yml`, `tests_sources_with_latest_cppcheck.yml` — source-tree
  linters.

## Why this directory was expensive

Having no Linux CI was not a neutral gap. Two Linux-breaking defects shipped and
survived for weeks because nothing on any machine compiled this tree on Linux:

- `src/omnisim/gui/OmWgpuView.cpp` did not compile at all — a bare
  `#if QT_CONFIG(wayland)` is `1/0` to the preprocessor, and the file is in the
  unconditional source list.
- The vendored `libQt6Gui` could not link: fontconfig and xkbcommon dev packages
  were missing from every copy of the dependency list, including
  `scripts/install/linux_bootstrap.sh` — the recipe the README hands to Linux
  users.

Both are fixed, and [`linux-build.yml`](../workflows/linux-build.yml) now guards
against their return by driving `linux_bootstrap.sh` itself, so a green run is
evidence about the documented user path rather than about a private CI recipe.
