# Workflows — disabled

These CI workflows are inherited from upstream Webots and have not yet been
validated against the OmniSim source tree. GitHub only treats files inside
`.github/workflows/` as active workflows — anything in this directory is
dormant text.

To re-enable a workflow, move its file back:

```bash
git mv .github/workflows.disabled/release.yml .github/workflows/release.yml
```

Then validate it builds end-to-end on a clean runner before relying on it.

## What's here

- `release.yml` — Linux/macOS/Windows build matrix that fires on `v*` tags
  and produces a GitHub Release with `.tar.bz2` / `.deb` / `.dmg` / `.exe`
  artifacts. Will need CUDA toolkit installation and signing-cert
  configuration before it works on the OmniSim tree.
- `developer_fast_path.yml` — fast-path PR check that asserts a small set
  of dev-doc files exist.
- `smoke_linux_fast.yml` — smoke-test runner.
- `tests_doc.yml`, `tests_sources.yml`, `tests_sources_with_latest_cppcheck.yml` —
  doc + source-tree linters.
- `test_suite_*.yml` — full per-platform test suites (long-running).
- `sync_protected_branches.yml` — branch-protection sync helper.

When OmniSim is ready to validate any of these, move them back one at a
time and confirm they pass on a real run before re-enabling the next.
