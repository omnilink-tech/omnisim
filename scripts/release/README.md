# Release scripts

Everything under `scripts/release/` exists to cut an OmniSim release: bump
the version, generate the changelog section, publish the snapshot to the
public `omnilink-tech/omnisim` remote, and create the GitHub Release page.
The release *model* — why publishing is one squashed snapshot per version,
the pre-release checklist, the deny-list, recovering from a botched
release — is [docs/developer/release-model.md](../../docs/developer/release-model.md);
this file is the index of what lives here. Every script carries its full
usage in its own header (`--help` for the Python ones).

## Scripts

### `bump_version.py`

One release label, five copies, one commit. The version is stored in five
files that nothing links — `omnisim/__init__.py` (`__version__`),
`src/omnisim/core/OmApplicationInfo.cpp` (`omniSimVersionString`),
`docker/Dockerfile.train` (`ARG OMNISIM_TAG`),
`.github/workflows/train-image.yml` (the dispatch default and the env
fallback) and `scripts/packaging/omnisim_version.txt` (the installer's
tag). `publish_snapshot.sh` rewrites them a file-group at a time and
commits each group separately, which is why a release used to be four
`version: bump …` commits, and why the strings drifted before it learned
to rewrite all of them (`f29cdae88`). This script reads all five (they
must agree, or it prints every site's value and refuses), rewrites only
the version characters in place — line endings and surrounding text are
preserved byte-for-byte, and prose that merely *mentions* an old tag is
left alone — runs `tests/test_version_consistency.py`, and makes one commit.

```bash
python scripts/release/bump_version.py 8.2.1 --dry-run   # the diff; writes nothing
python scripts/release/bump_version.py 8.2.1             # rewrite + run the test; no commit
python scripts/release/bump_version.py 8.2.1 --commit    # … and ONE commit:
                                                         #   version: bump to v8.2.1 (five sites)
```

`--commit` stages exactly those five paths and refuses if any of them
already has uncommitted changes. A red consistency test leaves the files
rewritten for inspection and commits nothing. Exit codes: 0 done or
nothing to do, 1 refused, 2 usage. The test suite asserts that a
`--dry-run` on the *current* version reports `no change`, and that a
dry-run to the next patch version diffs every site without writing.

### `publish_snapshot.sh`

The publisher: `scripts/release/publish_snapshot.sh <version> [--from <ref>] [--push]`.
Builds a single squashed snapshot of this repository in a throwaway
worktree, applies `publish_deny.txt` and `public_redactions.txt`,
generates a changelog section if `CHANGELOG.md` has none for the version,
diffs against `public/main`, and — only with `--push` — pushes the commit
and the tag, records the pair in `.last_published`, and creates the GitHub
Release page when `GITHUB_TOKEN` is set. Without `--push` it is a full
dry-run that prints the file list and the diff. It calls `bump_version.py` for the
version sites (one commit, a no-op when they already match); the four
per-file auto-bump blocks it carried until 2026-09-02 are gone.
Full reference, required environment and recovery procedures:
[docs/developer/release-model.md → The Publishing Script](../../docs/developer/release-model.md#the-publishing-script).

### `generate_changelog_section.py`

Called by `publish_snapshot.sh` when `CHANGELOG.md` has no entry for the
version being cut. Emits a Keep-a-Changelog-style `## [vX.Y.Z] — YYYY-MM-DD`
section to stdout from the git history since the previous tag, categorised
by path and stripped of internal codename prefixes. It records *what*
changed; edit the section for narrative before publishing.

### `backfill_release_pages.py`

Creates GitHub Release pages for tags that were pushed before the publisher
learned to call the Releases API (v1.0.0 – v1.0.6). Reads the per-version
section out of `CHANGELOG.md`; idempotent, skips versions that already
have a page. Needs `GITHUB_TOKEN`.

## Data files

- **`publish_deny.txt`** — the publish deny-list: paths and globs stripped
  from every public snapshot. The enforcement mechanism for
  "private-tree-only"; review it before each release
  ([syntax and semantics](../../docs/developer/release-model.md#the-publish-deny-list)).
- **`public_redactions.txt`** — `private-token<TAB>public-replacement` pairs
  applied to the snapshot worktree only (account handles, workstation
  usernames). Kept here rather than inline in the publisher because the
  publisher itself ships publicly.
- **`.last_published`** — one `vX.Y.Z<TAB><private sha>` line per published
  release: which private commit each public tag was cut from. Appended by
  `publish_snapshot.sh --push`; not hand-edited.
