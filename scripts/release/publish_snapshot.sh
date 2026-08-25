#!/usr/bin/env bash
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

#
# publish_snapshot.sh — publish a single squashed snapshot of this repository
# to the public omnilink-tech/omnisim remote, as one commit authored by a single
# publishing identity. Implements the "Option B" snapshot-per-version release
# model (see docs/developer/release-model.md).
#
# Usage:
#   scripts/release/publish_snapshot.sh <version> [--from <ref>] [--push]
#
# Arguments:
#   <version>        Semver tag for this release, e.g. v1.0.0 or v1.2.3-rc1.
#   --from <ref>     Source ref in this (private) repo whose tree to publish.
#                    Defaults to HEAD.
#   --push           Actually push to the public remote. Without this flag the
#                    script runs end-to-end as a dry-run and prints the diff
#                    that would land on public/main.
#   --force-orphan   DESTRUCTIVE. Publish <version> as the ONLY version in the
#                    public repository: build the snapshot with NO PARENT, then
#                    force-push it over main and delete every OTHER release tag
#                    (and its GitHub Release, when GITHUB_TOKEN is set).
#                    The public repo ends up with one commit and one release.
#
#                    Every earlier release becomes permanently unobtainable --
#                    there is no undo, and a fork or clone someone already has
#                    is the only remaining copy. Use it when an old release
#                    carries something that must stop being distributed, which
#                    is the case it was written for.
#
#                    ONLY tags matching vMAJOR.MINOR.PATCH[-prerelease] are
#                    considered. Tags that are not releases are NEVER touched:
#                    deps-windows-v1 / deps-linux-v1 / deps-mac-v1 carry the
#                    build's dependency archives as GitHub Release assets
#                    (dependencies/Makefile.* fetches from them), so deleting
#                    them would break `make` on every platform for everyone.
#                    That is not a hypothetical -- they are the majority of the
#                    tags on the remote today.
#
#                    Without --push this only PRINTS what it would destroy.
#
# Required environment:
#   PUBLIC_REMOTE    git URL of the public repo (e.g. git@github.com:omnilink-tech/omnisim.git).
#   PUBLISH_EMAIL    email recorded as author/committer of the public commit.
#
# Optional environment:
#   PUBLISH_NAME     author/committer name. Defaults to "omnilink".
#   GITHUB_TOKEN     GitHub Personal Access Token with `repo` scope (or fine-
#                    grained equivalent with "Contents: Read and write" on the
#                    public repo). When set, the script POSTs to GitHub's API
#                    after the tag push to create a Release page with the
#                    extracted CHANGELOG.md notes — this is what makes the
#                    "Watch → Releases only" subscription fire and what
#                    populates the repo's Releases sidebar. Without a token,
#                    only the tag is pushed (no Release page) and the script
#                    prints a one-line note.
#
# Behavior:
#   * Verifies <version> matches v<MAJOR>.<MINOR>.<PATCH>[-prerelease].
#   * Verifies the tag does not already exist on the public remote.
#   * If CHANGELOG.md has no [<version>] section, auto-generates one from
#     git history since the last published private SHA (and commits it).
#   * If omniSimVersionString in src/omnisim/core/OmApplicationInfo.cpp is
#     stale, rewrites it to match <version> (and commits the bump). This
#     is what the binary's About dialog and the auto-update notifier read,
#     so without this step the public snapshot's binary would lie about
#     its own version.
#   * Materialises the tree from <ref>, applies the publish deny-list at
#     scripts/release/publish_deny.txt (paths listed there are removed
#     from the snapshot — they stay private), then commits the filtered
#     tree on top of public/main (or as the orphan first commit).
#   * Tags that commit with <version>.
#   * On --push, fast-forwards public/main and pushes the tag.
#
# This script is intentionally git-plumbing heavy. The only places it
# modifies private state are (a) the auto-CHANGELOG commit and (b) the
# omniSimVersion bump commit. Both are additive — never rewrites. All
# snapshot-tree work happens inside a throwaway worktree under
# .build_tmp/release-publish/.

set -euo pipefail

# Derive paths from the script's own location so this works regardless of
# where the repo is checked out.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKTREE_DIR="$REPO_ROOT/.build_tmp/release-publish"

err() { printf 'error: %s\n' "$*" >&2; exit 1; }
log() { printf '==> %s\n' "$*" >&2; }

# ---- argument parsing -------------------------------------------------------
VERSION=""
FROM_REF="HEAD"
PUSH=0
FORCE_ORPHAN=0

while (($#)); do
    case "$1" in
        --from)
            [[ $# -ge 2 ]] || err "--from requires an argument"
            FROM_REF="$2"
            shift 2
            ;;
        --push)
            PUSH=1
            shift
            ;;
        --force-orphan)
            FORCE_ORPHAN=1
            shift
            ;;
        -h|--help)
            sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//' | sed '$d'
            exit 0
            ;;
        v*)
            [[ -z "$VERSION" ]] || err "version specified twice ($VERSION, $1)"
            VERSION="$1"
            shift
            ;;
        *)
            err "unknown argument: $1"
            ;;
    esac
done

[[ -n "$VERSION" ]] || err "missing required <version> argument (e.g. v1.0.0)"
[[ "$VERSION" =~ ^v[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] \
    || err "version '$VERSION' is not vMAJOR.MINOR.PATCH[-prerelease]"

[[ -n "${PUBLIC_REMOTE:-}" ]] || err "PUBLIC_REMOTE env var is required"
[[ -n "${PUBLISH_EMAIL:-}" ]] || err "PUBLISH_EMAIL env var is required"
PUBLISH_NAME="${PUBLISH_NAME:-OmniLink}"

# Extract owner/repo from PUBLIC_REMOTE. Supports git@host:owner/repo.git and
# https://host/owner/repo.git. Derived once, here, because two later steps need
# it -- the Release-page POST and (under --force-orphan) the Release DELETEs --
# and the second of those runs BEFORE the block this used to be computed in.
PUBLIC_OWNER_REPO="$(printf '%s' "$PUBLIC_REMOTE" \
    | sed -E 's#^git@[^:]+:##; s#^https?://[^/]+/##; s#\.git$##')"

# ---- auto-generate CHANGELOG.md section if missing -------------------------
# Fully-automatic release notes: if CHANGELOG.md has no `## [$VERSION]`
# section, generate one from git history since the last published private
# SHA (tracked in scripts/release/.last_published, which is deny-listed so
# it never lands on public). The auto-generated section is committed to
# private so future runs and audits can see exactly what shipped. If the
# section already exists (because the user pre-authored it), we use it as
# is — pre-authoring always wins.
#
# This step is the only place publish_snapshot.sh modifies private state.
# It is additive: a new commit appended to HEAD, never a rewrite. With
# --from set to anything other than HEAD, we refuse to auto-generate
# because the snapshot would be built from a tree that doesn't contain
# the new section.
cd "$REPO_ROOT"

CHANGELOG_PRIVATE="$REPO_ROOT/CHANGELOG.md"
SENTINEL="$SCRIPT_DIR/.last_published"
GENERATOR="$SCRIPT_DIR/generate_changelog_section.py"

section_exists() {
    [[ -f "$CHANGELOG_PRIVATE" ]] \
        && grep -q "^## \[$VERSION\]" "$CHANGELOG_PRIVATE"
}

if section_exists; then
    log "release notes : found [$VERSION] section in CHANGELOG.md (pre-authored)"
else
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "no [$VERSION] section in CHANGELOG.md and --from $FROM_REF is set;
       auto-generation only modifies HEAD. Either pre-author the section
       or rerun with --from HEAD."

    [[ -f "$CHANGELOG_PRIVATE" ]] \
        || err "CHANGELOG.md is missing — auto-generation needs the file to
       exist with at least a header before it can prepend new sections."

    [[ -f "$GENERATOR" ]] \
        || err "generator script missing: $GENERATOR"

    SINCE_REF=""
    if [[ -f "$SENTINEL" ]]; then
        # Sentinel format: one "vX.Y.Z<TAB><sha>" line per published release,
        # most recent last. Take the last line's SHA.
        SINCE_REF="$(awk -F'\t' 'NF>=2 {sha=$2} END{print sha}' "$SENTINEL")"
        if [[ -n "$SINCE_REF" ]] \
            && ! git rev-parse --verify "$SINCE_REF^{commit}" >/dev/null 2>&1; then
            log "  WARNING: sentinel SHA $SINCE_REF is not in this repo;"
            log "           the generator will fall back to full history"
            SINCE_REF=""
        fi
    fi

    if [[ -n "$SINCE_REF" ]]; then
        log "auto-notes    : generating [$VERSION] from $SINCE_REF..HEAD"
    else
        log "auto-notes    : no last-published sentinel — first auto-run."
        log "                refusing to generate against the full history"
        log "                (would produce an enormous section). Author the"
        log "                [$VERSION] section in CHANGELOG.md by hand for"
        log "                this release; future releases will track the"
        log "                sentinel automatically."
        err "first-run bootstrap requires a hand-authored [$VERSION] section"
    fi

    SECTION_FILE="$(mktemp)"
    if [[ -n "$SINCE_REF" ]]; then
        python "$GENERATOR" --version "$VERSION" --since "$SINCE_REF" \
            --out "$SECTION_FILE" \
            || { rm -f "$SECTION_FILE"; err "changelog generator failed"; }
    fi

    # Prepend the generated section above the first existing `## [v` heading,
    # preserving the file's preamble (header text + intro paragraphs). Insert
    # a horizontal rule between the new section and the next one.
    UPDATED="$(mktemp)"
    awk -v section_file="$SECTION_FILE" '
        /^## \[v/ && !inserted {
            while ((getline line < section_file) > 0) print line
            close(section_file)
            print ""
            print "---"
            print ""
            inserted=1
        }
        { print }
        END {
            if (!inserted) {
                print ""
                while ((getline line < section_file) > 0) print line
                close(section_file)
            }
        }
    ' "$CHANGELOG_PRIVATE" > "$UPDATED"
    mv "$UPDATED" "$CHANGELOG_PRIVATE"
    rm -f "$SECTION_FILE"

    git add "$CHANGELOG_PRIVATE"
    git commit -m "docs: release notes for $VERSION" >/dev/null \
        || err "failed to commit auto-generated CHANGELOG.md update
       (check pre-commit hooks and signing config; the file is staged)"
    log "  committed CHANGELOG.md update to private"
    log "  review the generated section, edit if needed, then rerun."
    log "  the [$VERSION] section will be reused as-is on rerun."

    if [[ $PUSH -ne 1 ]]; then
        log "auto-notes    : stopping here so you can review the section."
        log "                rerun with --push to publish (or edit CHANGELOG.md"
        log "                first and amend the docs commit if you want)."
        exit 0
    fi
fi

# ---- bump omniSimVersion if stale ------------------------------------------
# The binary's About dialog and the auto-update notifier both read
# omniSimVersionString from src/omnisim/core/OmApplicationInfo.cpp. Before
# this block existed the bump was manual and drifted across releases
# (we shipped v1.0.10 and v2.0.0 with the previous string still baked in).
# Bumping here, before SOURCE_TREE is resolved, guarantees the public
# snapshot's binary reports the released version. The bump is a no-op
# if the string already matches, so reruns don't pile up duplicate
# commits. Like the CHANGELOG step, this only runs against HEAD —
# bumping an unrelated ref would land the change in a tree the snapshot
# never sees.
VERSION_FILE="$REPO_ROOT/src/omnisim/core/OmApplicationInfo.cpp"
TARGET_VERSION="${VERSION#v}"

if [[ ! -f "$VERSION_FILE" ]]; then
    log "WARNING: $VERSION_FILE not found — skipping omniSimVersion bump"
elif grep -q "static const QString omniSimVersionString = \"$TARGET_VERSION\";" \
        "$VERSION_FILE"; then
    log "version-bump  : omniSimVersionString already $TARGET_VERSION (no-op)"
else
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "omniSimVersionString in OmApplicationInfo.cpp is stale and
       --from $FROM_REF is set; auto-bump only modifies HEAD. Either
       bump the string manually or rerun with --from HEAD."

    # Portable across BSD/GNU sed: write to temp file then mv, instead
    # of sed -i which has different syntax on macOS vs Linux.
    TMP_VERSION_FILE="$(mktemp)"
    sed -E "s/(static const QString omniSimVersionString = )\"[^\"]+\";/\1\"$TARGET_VERSION\";/" \
        "$VERSION_FILE" > "$TMP_VERSION_FILE"
    if ! grep -q "static const QString omniSimVersionString = \"$TARGET_VERSION\";" \
            "$TMP_VERSION_FILE"; then
        rm -f "$TMP_VERSION_FILE"
        err "sed rewrite of OmApplicationInfo.cpp produced unexpected output —
       the omniSimVersion() function signature may have changed upstream"
    fi
    mv "$TMP_VERSION_FILE" "$VERSION_FILE"

    git add "$VERSION_FILE"
    git commit -m "version: bump omniSimVersion to $TARGET_VERSION" >/dev/null \
        || err "failed to commit omniSimVersion bump
       (check pre-commit hooks and signing config; the file is staged)"
    log "version-bump  : committed omniSimVersion -> $TARGET_VERSION"
fi

# The Python CLI carries its own copy, and `omnisim doctor` -- the first
# command AGENTS.md tells an agent to run -- prints it. Bumping only the C++
# string left doctor announcing the PREVIOUS release; keep the two together.
PY_VERSION_FILE="$REPO_ROOT/omnisim/__init__.py"
if [[ ! -f "$PY_VERSION_FILE" ]]; then
    log "WARNING: $PY_VERSION_FILE not found — skipping __version__ bump"
elif grep -q "^__version__ = \"$TARGET_VERSION\"$" "$PY_VERSION_FILE"; then
    log "version-bump  : omnisim.__version__ already $TARGET_VERSION (no-op)"
else
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "omnisim/__init__.py __version__ is stale and --from $FROM_REF
       is set; auto-bump only modifies HEAD."
    TMP_PY_VERSION_FILE="$(mktemp)"
    sed -E "s/^(__version__ = )\"[^\"]+\"$/\1\"$TARGET_VERSION\"/" \
        "$PY_VERSION_FILE" > "$TMP_PY_VERSION_FILE"
    if ! grep -q "^__version__ = \"$TARGET_VERSION\"$" "$TMP_PY_VERSION_FILE"; then
        rm -f "$TMP_PY_VERSION_FILE"
        err "sed rewrite of omnisim/__init__.py produced unexpected output"
    fi
    mv "$TMP_PY_VERSION_FILE" "$PY_VERSION_FILE"
    git add "$PY_VERSION_FILE"
    git commit -m "version: bump omnisim.__version__ to $TARGET_VERSION" >/dev/null \
        || err "failed to commit omnisim.__version__ bump"
    log "version-bump  : committed omnisim.__version__ -> $TARGET_VERSION"
fi

# Distribution packages previously kept the inherited Webots release label in
# scripts/packaging/omnisim_version.txt, so an OmniSim v8 tag would produce an
# `omnisim-R2025a_setup.exe`. Keep package metadata and asset names on the same
# SemVer as the binary and Python CLI. The leading `v` is intentional: the
# Linux packager removes its first character when forming a Debian version.
PACKAGE_VERSION_FILE="$REPO_ROOT/scripts/packaging/omnisim_version.txt"
if [[ ! -f "$PACKAGE_VERSION_FILE" ]]; then
    log "WARNING: $PACKAGE_VERSION_FILE not found — skipping package version bump"
elif grep -qx "$VERSION" "$PACKAGE_VERSION_FILE"; then
    log "version-bump  : package version already $VERSION (no-op)"
else
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "scripts/packaging/omnisim_version.txt is stale and --from
       $FROM_REF is set; auto-bump only modifies HEAD."
    printf '%s\n' "$VERSION" > "$PACKAGE_VERSION_FILE"
    git add "$PACKAGE_VERSION_FILE"
    git commit -m "version: bump distribution package to $VERSION" >/dev/null \
        || err "failed to commit distribution package version bump"
    log "version-bump  : committed package version -> $VERSION"
fi

# The GPU training image pins the release it builds FROM, in two files that
# must agree. Left unbumped they point at the PREVIOUS release -- and if that
# release was never published (v5.3.0 was not), a triggered build clones a ref
# the public remote does not have and fails. train-image.yml also triggers on
# a push touching these paths, so the bump and the build stay in step.
for TAG_FILE in "$REPO_ROOT/docker/Dockerfile.train" \
                "$REPO_ROOT/.github/workflows/train-image.yml"; do
    [[ -f "$TAG_FILE" ]] || { log "WARNING: $TAG_FILE not found — skipping tag bump"; continue; }
    if ! grep -qE "v[0-9]+\.[0-9]+\.[0-9]+" "$TAG_FILE"; then
        log "version-bump  : no release tag in $(basename "$TAG_FILE") (no-op)"
        continue
    fi
    # Any release tag in the file that is not already $VERSION?
    if [[ -z "$(grep -oE "v[0-9]+\.[0-9]+\.[0-9]+" "$TAG_FILE" | grep -vx "$VERSION" || true)" ]]; then
        log "version-bump  : $(basename "$TAG_FILE") already $VERSION (no-op)"
        continue
    fi
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "$TAG_FILE pins a stale release tag and --from $FROM_REF is set;
       auto-bump only modifies HEAD."
    TMP_TAG_FILE="$(mktemp)"
    sed -E "s/v[0-9]+\.[0-9]+\.[0-9]+/$VERSION/g" "$TAG_FILE" > "$TMP_TAG_FILE"
    mv "$TMP_TAG_FILE" "$TAG_FILE"
    git add "$TAG_FILE"
done
if ! git diff --cached --quiet -- "$REPO_ROOT/docker/Dockerfile.train" \
        "$REPO_ROOT/.github/workflows/train-image.yml" 2>/dev/null; then
    git commit -m "version: bump training-image tag to $VERSION" >/dev/null \
        || err "failed to commit training-image tag bump"
    log "version-bump  : committed training-image tag -> $VERSION"
fi

# ---- resolve source tree ----------------------------------------------------
SOURCE_SHA="$(git rev-parse --verify "$FROM_REF^{commit}" 2>/dev/null)" \
    || err "could not resolve source ref '$FROM_REF' in private repo"
SOURCE_TREE="$(git rev-parse --verify "$FROM_REF^{tree}")"

DENY_LIST="$SCRIPT_DIR/publish_deny.txt"
REDACTIONS_FILE="$SCRIPT_DIR/public_redactions.txt"

log "version       : $VERSION"
log "source ref    : $FROM_REF ($SOURCE_SHA)"
log "source tree   : $SOURCE_TREE"
log "public remote : $PUBLIC_REMOTE"
log "author        : $PUBLISH_NAME <$PUBLISH_EMAIL>"
log "deny-list     : $([[ -f "$DENY_LIST" ]] && echo "$DENY_LIST" || echo "(none)")"
log "mode          : $([[ $PUSH -eq 1 ]] && echo PUSH || echo dry-run)"

# ---- prepare worktree -------------------------------------------------------
# A throwaway worktree pinned to a fresh anonymous branch. We never check out
# any private history into it; we use it only to fetch from the public remote
# and produce one new commit with `git commit-tree`.
if [[ -d "$WORKTREE_DIR" ]]; then
    log "removing stale worktree at $WORKTREE_DIR"
    git worktree remove --force "$WORKTREE_DIR" 2>/dev/null || rm -rf "$WORKTREE_DIR"
fi

# Use a detached HEAD so we never accidentally affect a real branch.
git worktree add --detach "$WORKTREE_DIR" "$SOURCE_SHA" >/dev/null
trap 'git worktree remove --force "$WORKTREE_DIR" 2>/dev/null || true' EXIT

cd "$WORKTREE_DIR"

# ---- fetch public state -----------------------------------------------------
git remote remove public 2>/dev/null || true
git remote add public "$PUBLIC_REMOTE"

log "fetching public remote (this may prompt for credentials)…"
PUBLIC_HAS_MAIN=1
if ! git fetch --no-tags public main 2>/dev/null; then
    log "public/main not found — treating this as the first release"
    PUBLIC_HAS_MAIN=0
fi

# Verify the tag does not already exist on the public remote.
#
# Under --force-orphan this is a warning rather than an error: republishing the
# SAME version as the sole release is exactly what that mode is for (e.g. the
# first attempt shipped something that had to be withdrawn). The tag is deleted
# and recreated below. Every other path still refuses, because silently moving
# a published tag is how a release stops being reproducible.
if git ls-remote --tags --exit-code public "refs/tags/$VERSION" >/dev/null 2>&1; then
    if [[ $FORCE_ORPHAN -eq 1 ]]; then
        log "WARNING: tag $VERSION already exists on $PUBLIC_REMOTE — --force-orphan will REPLACE it"
    else
        err "tag $VERSION already exists on $PUBLIC_REMOTE — pick a new version"
    fi
fi

# ---- --force-orphan preflight ----------------------------------------------
# Work out, and SAY OUT LOUD, exactly what is about to stop existing. This runs
# in dry-run too, which is the point: the destructive list must be readable
# before anyone types --push.
DOOMED_TAGS=()
if [[ $FORCE_ORPHAN -eq 1 ]]; then
    while read -r _sha _ref; do
        [[ -n "${_ref:-}" ]] || continue
        _tag="${_ref#refs/tags/}"
        [[ "$_tag" == *'^{}'* ]] && continue
        # Releases only. Anything not matching the release pattern -- notably the
        # deps-*-v1 tags that carry the build's dependency archives -- is left alone.
        [[ "$_tag" =~ ^v[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] || continue
        [[ "$_tag" == "$VERSION" ]] && continue
        DOOMED_TAGS+=("$_tag")
    done < <(git ls-remote --tags public 2>/dev/null || true)

    log ""
    log "=============================================================="
    log " --force-orphan: $VERSION will become the ONLY version"
    log "=============================================================="
    log "  commit history : DISCARDED (snapshot committed with no parent)"
    log "  main           : FORCE-PUSHED over $(git rev-parse --short FETCH_HEAD 2>/dev/null || echo 'public/main')"
    if ((${#DOOMED_TAGS[@]})); then
        log "  release tags   : DELETING ${#DOOMED_TAGS[@]} — ${DOOMED_TAGS[*]}"
        log "                   (+ their GitHub Releases, if GITHUB_TOKEN is set)"
    else
        log "  release tags   : none to delete"
    fi
    # Report what is deliberately being SPARED, because a silent exclusion is
    # indistinguishable from a bug the day someone audits this.
    _spared="$(git ls-remote --tags public 2>/dev/null \
        | sed 's|.*refs/tags/||' | grep -v '\^{}' \
        | grep -Ev '^v[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$' | tr '\n' ' ' || true)"
    [[ -n "${_spared// /}" ]] && log "  KEPT (not releases): ${_spared}"
    log "  This is not reversible. An existing clone is the only other copy."
    log "=============================================================="
    log ""
fi

# ---- apply publish deny-list ------------------------------------------------
# The deny-list (scripts/release/publish_deny.txt) names paths that live in
# the private tree but must not appear in the public snapshot — internal bug
# trackers, contributor-only docs, etc. We rm --cached from the worktree's
# index (the working files in the throwaway worktree are touched but the
# private repo is untouched), then re-snapshot the index with `git
# write-tree` to get the filtered tree SHA. Commit-tree uses that filtered
# tree, not the source tree.
SNAPSHOT_TREE="$SOURCE_TREE"
if [[ -f "$DENY_LIST" ]]; then
    log "applying deny-list…"
    DENIED_TOTAL=0
    DENIED_MISSING=0
    # Every file the deny-list removes, recorded so we can afterwards check
    # whether anything that DOES ship still points at one of them.
    DENIED_MANIFEST="$(mktemp)"
    trap 'rm -f "$DENIED_MANIFEST"' EXIT
    while IFS= read -r entry || [[ -n "$entry" ]]; do
        # Strip whitespace.
        entry="${entry#"${entry%%[![:space:]]*}"}"
        entry="${entry%"${entry##*[![:space:]]}"}"
        # Skip blank lines and comments.
        [[ -z "$entry" || "${entry:0:1}" == "#" ]] && continue
        # Use a sentinel to tell whether anything actually got removed.
        before="$(git ls-files -- "$entry" | wc -l)"
        if [[ "$before" -gt 0 ]]; then
            git ls-files -- "$entry" >> "$DENIED_MANIFEST"
            git rm -rf --cached --ignore-unmatch -- "$entry" >/dev/null
            log "  excluded: $entry ($before file(s))"
            DENIED_TOTAL=$((DENIED_TOTAL + before))
        else
            log "  WARNING: deny-list entry matched no tracked files: $entry"
            DENIED_MISSING=$((DENIED_MISSING + 1))
        fi
    done < "$DENY_LIST"
    log "  total files excluded: $DENIED_TOTAL"
    if [[ "$DENIED_MISSING" -gt 0 ]]; then
        log "  ($DENIED_MISSING entries did not match — typo or already moved?)"
    fi
    # ---- dangling-reference check -------------------------------------------
    # The deny-list removes whole files, but it cannot rewrite the files that
    # SURVIVE. A world that loads a held URDF, a catalogue row linking a held
    # demo, a guide pointing at a held world -- each parses fine here and only
    # breaks in the reader's hands, which is the worst place to find it. This
    # cost us a publicly-catalogued flagship demo that could never load on
    # public. So: grep the surviving tree for the basenames we just removed.
    #
    # Match on the last TWO path segments ("<dir>/<file.ext>"), not the bare
    # basename. A path-shaped reference is what actually breaks -- a world
    # url, a markdown link, a registry entry -- whereas a bare name in prose is
    # usually just prose. This is the difference between ~50 hits worth reading
    # and ~135 that nobody will.
    log "checking for references to excluded files…"
    PATTERNS="$DENIED_MANIFEST.patterns"
    REMOVED_REFS="$DENIED_MANIFEST.removed-refs"
    SURVIVING_REFS="$DENIED_MANIFEST.surviving-refs"
    # awk does the segment split in one process; spawning `basename` per path
    # costs ~10k process launches here and takes minutes under MSYS.
    awk -F/ 'NF>=2{print $(NF-1)"/"$NF} NF==1{print $NF}' "$DENIED_MANIFEST" \
      | sort -u > "$REMOVED_REFS"
    # A reference only dangles if NOTHING that ships still matches it -- else
    # it still resolves and the hit is a path collision, not a break (a held
    # agent's docs/OVERVIEW.md must not indict the shipped one).
    git ls-files \
      | awk -F/ 'NF>=2{print $(NF-1)"/"$NF} NF==1{print $NF}' \
      | sort -u > "$SURVIVING_REFS"
    comm -23 "$REMOVED_REFS" "$SURVIVING_REFS" > "$PATTERNS"
    rm -f "$REMOVED_REFS" "$SURVIVING_REFS"
    DANGLING=0
    if [[ -s "$PATTERNS" ]]; then
        # -F fixed strings, -f patterns-from-file, -n line numbers, -I skip binaries.
        # --cached searches the index, which after the rm pass IS the shipping set,
        # so there is no path list to pass and nothing to overflow argv.
        DANGLING_HITS="$(git grep -n -I -F -f "$PATTERNS" --cached -- . 2>/dev/null || true)"
        if [[ -n "$DANGLING_HITS" ]]; then
            DANGLING="$(printf '%s\n' "$DANGLING_HITS" | wc -l | tr -d ' ')"
            printf '%s\n' "$DANGLING_HITS" | head -40 | while IFS= read -r h; do
                log "  DANGLING: ${h:0:160}"
            done
            [[ "$DANGLING" -gt 40 ]] && log "  … and $((DANGLING - 40)) more"
        fi
    fi
    rm -f "$PATTERNS"
    if [[ "$DANGLING" -gt 0 ]]; then
        log "  $DANGLING dangling reference(s): the snapshot would ship files that"
        log "  point at content it does not contain. Some are prose that names a"
        log "  held path on purpose ('not in the public snapshot') -- those are"
        log "  fine. A world url, a catalogue row or a registry entry is not:"
        log "  deny the referring file too, or edit the reference out."
        # Advisory by default: the list still contains deliberate prose mentions,
        # so failing the release on it would just train everyone to set the
        # bypass. Set PUBLISH_STRICT_DANGLING=1 in CI once the list is clean.
        if [[ "${PUBLISH_STRICT_DANGLING:-0}" == "1" ]]; then
            err "dangling references to excluded files (PUBLISH_STRICT_DANGLING=1)"
        fi
    else
        log "  no dangling references."
    fi

    # Re-snapshot the filtered index into a tree object.
    SNAPSHOT_TREE="$(git write-tree)"
    log "filtered tree : $SNAPSHOT_TREE"
else
    log "no deny-list found — publishing unfiltered source tree"
fi

# ---- redact private identity tokens ----------------------------------------
# Some publishable benchmark records legitimately preserve absolute paths from
# the machine that produced them. The measurements should ship, but a local OS
# username or private account handle should not. Apply a private, deny-listed
# fixed-string table ONLY to the throwaway snapshot worktree; the source records
# remain byte-accurate. Re-scan the index after every replacement and fail
# closed if any token survives.
if [[ -f "$REDACTIONS_FILE" ]]; then
    command -v perl >/dev/null 2>&1 \
        || err "perl is required for fixed-string public-snapshot redaction"
    log "applying private identity redactions…"
    REDACTED_FILES=0
    REDACTION_RULES=0
    while IFS=$'\t' read -r private_token public_replacement _rest \
          || [[ -n "${private_token:-}" ]]; do
        private_token="${private_token%$'\r'}"
        public_replacement="${public_replacement%$'\r'}"
        [[ -z "$private_token" || "${private_token:0:1}" == "#" ]] && continue
        [[ -n "$public_replacement" ]] \
            || err "redaction rule has no public replacement"
        REDACTION_RULES=$((REDACTION_RULES + 1))
        RULE_FILES=0
        while IFS= read -r -d '' redacted_path; do
            OMNISIM_REDACT_FROM="$private_token" \
            OMNISIM_REDACT_TO="$public_replacement" \
              perl -0pi -e '
                BEGIN {
                  $from = $ENV{"OMNISIM_REDACT_FROM"};
                  $to = $ENV{"OMNISIM_REDACT_TO"};
                }
                s/\Q$from\E/$to/g;
              ' -- "$redacted_path"
            git add -- "$redacted_path"
            RULE_FILES=$((RULE_FILES + 1))
        done < <(git grep --cached -I -l -z -F -e "$private_token" -- . 2>/dev/null || true)
        if git grep --cached -I -q -F -e "$private_token" -- . 2>/dev/null; then
            err "private identity token survived public-snapshot redaction"
        fi
        REDACTED_FILES=$((REDACTED_FILES + RULE_FILES))
        log "  redacted identity token from $RULE_FILES file(s)"
    done < "$REDACTIONS_FILE"
    SNAPSHOT_TREE="$(git write-tree)"
    log "  privacy rules: $REDACTION_RULES; file rewrites: $REDACTED_FILES"
    log "privacy tree  : $SNAPSHOT_TREE"
else
    log "WARNING: no private identity redaction table found"
fi

# ---- build the snapshot commit ---------------------------------------------
PARENT_ARGS=()
if [[ $FORCE_ORPHAN -eq 1 ]]; then
    # Deliberate orphan. Note this is the SAME shape as the accident the
    # PUBLIC_HAS_MAIN=0 branch produces when a fetch fails on auth -- which is
    # why it is asked for explicitly here rather than inferred from a missing
    # parent. If you see "orphan" in a log, check which of the two it was.
    log "parent commit : (none — DELIBERATE orphan, --force-orphan)"
elif [[ $PUBLIC_HAS_MAIN -eq 1 ]]; then
    PARENT_SHA="$(git rev-parse FETCH_HEAD)"
    PARENT_ARGS=(-p "$PARENT_SHA")
    log "parent commit : $PARENT_SHA (public/main)"
else
    log "parent commit : (none — orphan first commit)"
fi

# The commit title is fixed; the body comes from the matching section in
# CHANGELOG.md. Under Option B there is exactly one commit per release on
# public, so this commit body is what GitHub Releases auto-generates the
# release notes from. CHANGELOG.md is read from the source worktree (the
# private repo's tree at $FROM_REF), not from the filtered snapshot index,
# because the deny-list operates on the index but never touches working
# files. If no section exists for $VERSION the script falls back to a
# title-only commit and warns — releasing without notes is allowed but
# loud.
COMMIT_TITLE="OmniSim $VERSION"
COMMIT_BODY=""
CHANGELOG_PATH="$WORKTREE_DIR/CHANGELOG.md"
if [[ -f "$CHANGELOG_PATH" ]]; then
    COMMIT_BODY="$(awk -v marker="## [$VERSION]" '
        index($0, marker) == 1 { in_section=1; next }
        in_section && /^## / { exit }
        in_section { lines[++n] = $0 }
        END {
            start = 1
            while (start <= n && lines[start] ~ /^[[:space:]]*$/) start++
            end = n
            while (end >= start && lines[end] ~ /^[[:space:]]*$/) end--
            # CHANGELOG.md separates versions with a "---" horizontal rule.
            # The rule belongs to the document layout, not the release note,
            # so strip it (and any blank lines above it) before emitting.
            if (end >= start && lines[end] ~ /^---+[[:space:]]*$/) {
                end--
                while (end >= start && lines[end] ~ /^[[:space:]]*$/) end--
            }
            for (i = start; i <= end; i++) print lines[i]
        }
    ' "$CHANGELOG_PATH")"
fi

# The message goes to commit-tree on STDIN, never as -m argv: release notes are
# hundreds of lines (v5.5.0 was 456) and argv has a hard OS limit -- passing it
# as arguments died with "Argument list too long" and would block any large
# release. stdin has no such ceiling.
COMMIT_MSG="$COMMIT_TITLE"
if [[ -n "$COMMIT_BODY" ]]; then
    BODY_LINES="$(printf '%s\n' "$COMMIT_BODY" | wc -l | tr -d ' ')"
    log "release notes : extracted $BODY_LINES line(s) from CHANGELOG.md"
    COMMIT_MSG="$COMMIT_MSG"$'\n\n'"$COMMIT_BODY"
else
    log "WARNING: no '## [$VERSION] — …' section found in CHANGELOG.md"
    log "         publishing with title-only commit message — add a section"
    log "         before --push if you want release notes on the GitHub Release"
fi

# ⚠ EXPORT the identity, do not prefix it. A `VAR=x cmd1 | cmd2` assignment is
# scoped to cmd1 only -- so the four GIT_* vars used to land on `printf` and
# `git commit-tree` never saw them, silently falling back to the publisher's
# local `git config user.name/user.email`. The script logged the intended
# identity while committing a different one, so PUBLISH_NAME/PUBLISH_EMAIL had
# no effect on any release up to and including v5.5.1 (both were published as
# the local git user). Exporting inside this command substitution keeps the
# assignment in the subshell and reaches both sides of the pipe.
SNAPSHOT_SHA="$(
    export GIT_AUTHOR_NAME="$PUBLISH_NAME" \
           GIT_AUTHOR_EMAIL="$PUBLISH_EMAIL" \
           GIT_COMMITTER_NAME="$PUBLISH_NAME" \
           GIT_COMMITTER_EMAIL="$PUBLISH_EMAIL"
    printf '%s\n' "$COMMIT_MSG" | git commit-tree "$SNAPSHOT_TREE" "${PARENT_ARGS[@]}"
)"
log "snapshot sha  : $SNAPSHOT_SHA"

# ---- verify the identity actually landed ------------------------------------
# Every release up to and including v5.5.1 was published under whatever git
# identity the publisher's machine happened to carry, because the four GIT_*
# assignments above were scoped to `printf` instead of exported (see the comment
# there). The script LOGGED the intended identity and COMMITTED a different one,
# and the public repo ended up listing three contributors -- two of them personal
# accounts. A logged intention is not evidence; read the identity back off the
# object that is about to be pushed.
ACTUAL_AUTHOR="$(git log -1 --format='%an <%ae>' "$SNAPSHOT_SHA")"
ACTUAL_COMMITTER="$(git log -1 --format='%cn <%ce>' "$SNAPSHOT_SHA")"
WANT_IDENT="$PUBLISH_NAME <$PUBLISH_EMAIL>"
if [[ "$ACTUAL_AUTHOR" != "$WANT_IDENT" || "$ACTUAL_COMMITTER" != "$WANT_IDENT" ]]; then
    err "identity check failed -- refusing to publish.
       wanted author+committer: $WANT_IDENT
       commit author          : $ACTUAL_AUTHOR
       commit committer       : $ACTUAL_COMMITTER"
fi
log "identity ok   : $WANT_IDENT (author + committer, read back from $SNAPSHOT_SHA)"

# Create the tag locally so dry-run can show what would be pushed.
git tag -f "$VERSION" "$SNAPSHOT_SHA" >/dev/null

# ---- show the diff ----------------------------------------------------------
# `--root` makes diff-tree treat the empty tree as the parent for orphan
# commits, so the very first release shows the full file list instead of
# nothing. For non-orphan releases the flag is harmless.
#
# We can't pipe directly into `head -50` under `set -o pipefail` — when head
# closes its stdin git diff-tree dies with SIGPIPE, which counts as a failed
# command and tears the script down before it can print the total. Capture
# the full list once into a temp file and slice it from there.
log "files in this snapshot:"
DIFF_LIST="$(mktemp)"
trap 'rm -f "$DIFF_LIST"; git worktree remove --force "$WORKTREE_DIR" 2>/dev/null || true' EXIT
git diff-tree --no-commit-id --name-status --root -r "$SNAPSHOT_SHA" > "$DIFF_LIST"
head -50 "$DIFF_LIST"
TOTAL_FILES="$(wc -l < "$DIFF_LIST")"
log "(total: $TOTAL_FILES files)"

# Under --force-orphan the snapshot has no parent, so PARENT_SHA was never
# set -- but public/main still EXISTS and is exactly what is about to be
# overwritten, so the diff against it is the most useful thing to print.
# Resolve it from FETCH_HEAD rather than reusing the (unset) parent.
if [[ $PUBLIC_HAS_MAIN -eq 1 ]]; then
    if [[ $FORCE_ORPHAN -eq 1 ]]; then
        log "diff vs public/main (which --force-orphan will REPLACE outright):"
    else
        log "diff vs public/main:"
    fi
    _diff_base="${PARENT_SHA:-$(git rev-parse FETCH_HEAD)}"
    git --no-pager diff --stat "$_diff_base" "$SNAPSHOT_SHA" | tail -30
fi

# ---- push or stop -----------------------------------------------------------
if [[ $PUSH -ne 1 ]]; then
    log "dry-run complete. Re-run with --push to publish."
    exit 0
fi

# Bypass the local pre-push smoke hook (.githooks/pre-push). That hook is
# a dev-workflow gate: it runs the smoke world set before pushing dev
# commits to origin, and it ships OMNISIM_SKIP_PUSH_CHECK=1 as its
# documented escape hatch. The release path is exactly what that hatch is
# for — we push a curated, pre-validated snapshot from the throwaway
# worktree under .build_tmp/release-publish/, which has no built binary,
# so the smoke run cannot execute here anyway. Release-tree validation is
# a separate preflight step (see docs/developer/release-model.md).
# TAG FIRST, THEN THE BRANCH. Workflows on the public repo trigger on the
# push to main (.github/workflows/train-image.yml rebuilds the GPU training
# image whenever its recipe changes) and build FROM the release tag. If the
# branch landed first, that run could start against a tag the remote does not
# have yet and fail on checkout. A tag pointing at a not-yet-referenced commit
# is valid git; the reverse race is not recoverable without a re-run.
if [[ $FORCE_ORPHAN -eq 1 ]]; then
    # Replacing the tag: delete it remotely first, since a plain push cannot
    # move an existing tag and --force on a tag is easy to fire by accident
    # elsewhere.
    if git ls-remote --tags --exit-code public "refs/tags/$VERSION" >/dev/null 2>&1; then
        log "deleting existing remote tag $VERSION…"
        OMNISIM_SKIP_PUSH_CHECK=1 git push public ":refs/tags/$VERSION"
    fi
fi

log "pushing tag $VERSION…"
OMNISIM_SKIP_PUSH_CHECK=1 git push public "refs/tags/$VERSION"
log "pushing $SNAPSHOT_SHA to $PUBLIC_REMOTE main…"
if [[ $FORCE_ORPHAN -eq 1 ]]; then
    # An orphan has no ancestor in common with public/main, so this is a
    # non-fast-forward by construction and --force is not optional.
    OMNISIM_SKIP_PUSH_CHECK=1 git push --force public "$SNAPSHOT_SHA:refs/heads/main"
else
    OMNISIM_SKIP_PUSH_CHECK=1 git push public "$SNAPSHOT_SHA:refs/heads/main"
fi
log "published $VERSION."

# ---- --force-orphan: retire every other release ----------------------------
# Deleting a tag does NOT delete its GitHub Release: the Release survives,
# orphaned, still listed on the repo's Releases page and still serving its
# assets. So the Release goes first, then the tag.
if [[ $FORCE_ORPHAN -eq 1 ]] && ((${#DOOMED_TAGS[@]})); then
    for _tag in "${DOOMED_TAGS[@]}"; do
        if [[ -n "${GITHUB_TOKEN:-}" ]]; then
            # urllib, not curl -- same reason as the Release-creation call
            # below: curl's Windows Schannel backend fails on CRL lookups
            # against this host.
            log "deleting GitHub Release for $_tag..."
            GITHUB_TOKEN="$GITHUB_TOKEN" OWNER_REPO="$PUBLIC_OWNER_REPO" TAG="$_tag" \
            python -c "
import json, os, sys, urllib.request, urllib.error
base = f\"https://api.github.com/repos/{os.environ['OWNER_REPO']}/releases\"
hdrs = {
    'Authorization': f\"Bearer {os.environ['GITHUB_TOKEN']}\",
    'Accept': 'application/vnd.github+json',
    'X-GitHub-Api-Version': '2022-11-28',
}
tag = os.environ['TAG']
def call(url, method):
    req = urllib.request.Request(url, method=method, headers=hdrs)
    with urllib.request.urlopen(req, timeout=30) as resp:
        raw = resp.read().decode('utf-8')
        return json.loads(raw) if raw.strip() else {}
try:
    rel = call(f'{base}/tags/{tag}', 'GET')
except urllib.error.HTTPError as e:
    if e.code == 404:
        print(f'no Release page for {tag} (tag only)')
        sys.exit(0)
    sys.stderr.write(f'HTTP {e.code} looking up {tag}\n'); sys.exit(2)
rid = rel.get('id')
if not rid:
    print(f'no Release id for {tag}'); sys.exit(0)
try:
    call(f'{base}/{rid}', 'DELETE')
    print(f'deleted Release {rid} for {tag}')
except urllib.error.HTTPError as e:
    sys.stderr.write(f'HTTP {e.code} deleting Release {rid}\n'); sys.exit(2)
" || log "WARNING: could not delete the Release for $_tag -- remove it by hand at https://github.com/$PUBLIC_OWNER_REPO/releases"
        else
            log "NOTE: no GITHUB_TOKEN -- the Release page for $_tag will OUTLIVE its deleted tag; delete it by hand"
        fi
        log "deleting remote tag $_tag..."
        OMNISIM_SKIP_PUSH_CHECK=1 git push public ":refs/tags/$_tag" \
            || log "WARNING: could not delete tag $_tag -- remove it by hand"
    done
    log "retired ${#DOOMED_TAGS[@]} earlier release(s): ${DOOMED_TAGS[*]}"
fi

# ---- record the published private SHA --------------------------------------
# After a successful push, append the private commit that produced this
# release to scripts/release/.last_published. The next auto-generation
# run reads this to know where to start the diff. The file is in the
# publish deny-list so it never lands on public.
cd "$REPO_ROOT"
PUBLISHED_PRIVATE_SHA="$(git rev-parse --verify HEAD)"
{
    [[ -f "$SENTINEL" ]] && cat "$SENTINEL"
    printf '%s\t%s\n' "$VERSION" "$PUBLISHED_PRIVATE_SHA"
} > "$SENTINEL.tmp"
mv "$SENTINEL.tmp" "$SENTINEL"
log "recorded sentinel: $VERSION $PUBLISHED_PRIVATE_SHA"

git add "$SENTINEL"
if ! git diff --cached --quiet -- "$SENTINEL"; then
    git commit -m "release: record $VERSION published private SHA" >/dev/null \
        || log "WARNING: sentinel update did not commit cleanly — commit by hand"
fi

# ---- create GitHub Release page (if token is provided) ---------------------
# After the tag is pushed, post a Release page to GitHub so:
#   - The repo home-page sidebar shows the latest release.
#   - Users with "Watch → Releases only" get email notifications.
#   - The /releases page populates with version history + notes.
# Token: $GITHUB_TOKEN. Personal Access Token with `repo` scope, or a fine-
# grained token with "Contents: Read and write" on the public repo. Without
# a token, this step is a no-op with a one-line note.
#
# The API call uses Python's urllib instead of curl. On Windows, curl's
# Schannel TLS backend can fail with CRYPT_E_NO_REVOCATION_CHECK when its
# CRL distribution-point lookup times out — even when the cert chain is
# perfectly valid otherwise. Python's TLS stack does not rely on Schannel
# CRL lookups in the same way and consistently succeeds on the same hosts
# where curl fails. Keeping the API call out of curl entirely sidesteps
# the whole class of Windows TLS-backend quirks.
if [[ -n "${GITHUB_TOKEN:-}" ]]; then
    # Any tag with a hyphen-suffix (v1.0.0-rc1, v2.0.0-beta) is a prerelease.
    PRERELEASE_FLAG="false"
    [[ "$VERSION" == *-* ]] && PRERELEASE_FLAG="true"

    log "creating GitHub Release page for $VERSION on $PUBLIC_OWNER_REPO..."
    set +e
    REL_URL="$(
      GITHUB_TOKEN="$GITHUB_TOKEN" \
      OWNER_REPO="$PUBLIC_OWNER_REPO" \
      VERSION="$VERSION" \
      COMMIT_BODY="${COMMIT_BODY:-}" \
      PRERELEASE_FLAG="$PRERELEASE_FLAG" \
      python -c "
import json, os, sys, urllib.request, urllib.error
payload = {
    'tag_name':   os.environ['VERSION'],
    'name':       'OmniSim ' + os.environ['VERSION'],
    'body':       os.environ.get('COMMIT_BODY', ''),
    'draft':      False,
    'prerelease': os.environ['PRERELEASE_FLAG'] == 'true',
}
url = f\"https://api.github.com/repos/{os.environ['OWNER_REPO']}/releases\"
req = urllib.request.Request(
    url,
    data=json.dumps(payload).encode('utf-8'),
    method='POST',
    headers={
        'Authorization': f\"Bearer {os.environ['GITHUB_TOKEN']}\",
        'Accept': 'application/vnd.github+json',
        'X-GitHub-Api-Version': '2022-11-28',
        'Content-Type': 'application/json',
    },
)
try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        body = json.loads(resp.read().decode('utf-8') or 'null') or {}
        print(body.get('html_url', '(no url returned)'))
except urllib.error.HTTPError as e:
    try:
        err = json.loads(e.read().decode('utf-8') or 'null') or {}
        msg = err.get('message', str(e))
    except Exception:
        msg = str(e)
    sys.stderr.write(f'HTTP {e.code}: {msg}\n')
    sys.exit(2)
except urllib.error.URLError as e:
    sys.stderr.write(f'URL error: {e}\n')
    sys.exit(2)
"
    )"
    PY_RC=$?
    set -e
    if [[ $PY_RC -eq 0 && -n "$REL_URL" ]]; then
        log "Release page  : $REL_URL"
    else
        log "WARNING: GitHub Release creation failed (Python rc=$PY_RC; see stderr above)"
        log "  the tag $VERSION is pushed; create the Release page manually at"
        log "  https://github.com/$PUBLIC_OWNER_REPO/releases/new?tag=$VERSION"
        log "  or rerun the backfill helper:"
        log "    python scripts/release/backfill_release_pages.py \\"
        log "        --owner-repo $PUBLIC_OWNER_REPO $VERSION"
    fi
else
    log "GITHUB_TOKEN not set — skipping GitHub Release page creation."
    log "  (the tag $VERSION is pushed but no Release page is created;"
    log "   set GITHUB_TOKEN to enable. See docs/developer/release-model.md.)"
fi
