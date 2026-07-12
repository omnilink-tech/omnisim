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
#
# Required environment:
#   PUBLIC_REMOTE    git URL of the public repo (e.g. git@github.com:omnilink-tech/omnisim.git).
#   PUBLISH_EMAIL    email recorded as author/committer of the public commit.
#
# Optional environment:
#   PUBLISH_NAME     author/committer name. Defaults to "OmniLink".
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
#   * If omniSimVersionString in src/omnisim/core/WbApplicationInfo.cpp is
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
# omniSimVersionString from src/omnisim/core/WbApplicationInfo.cpp. Before
# this block existed the bump was manual and drifted across releases
# (we shipped v1.0.10 and v2.0.0 with the previous string still baked in).
# Bumping here, before SOURCE_TREE is resolved, guarantees the public
# snapshot's binary reports the released version. The bump is a no-op
# if the string already matches, so reruns don't pile up duplicate
# commits. Like the CHANGELOG step, this only runs against HEAD —
# bumping an unrelated ref would land the change in a tree the snapshot
# never sees.
VERSION_FILE="$REPO_ROOT/src/omnisim/core/WbApplicationInfo.cpp"
TARGET_VERSION="${VERSION#v}"

if [[ ! -f "$VERSION_FILE" ]]; then
    log "WARNING: $VERSION_FILE not found — skipping omniSimVersion bump"
elif grep -q "static const QString omniSimVersionString = \"$TARGET_VERSION\";" \
        "$VERSION_FILE"; then
    log "version-bump  : omniSimVersionString already $TARGET_VERSION (no-op)"
else
    [[ "$FROM_REF" == "HEAD" ]] \
        || err "omniSimVersionString in WbApplicationInfo.cpp is stale and
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
        err "sed rewrite of WbApplicationInfo.cpp produced unexpected output —
       the omniSimVersion() function signature may have changed upstream"
    fi
    mv "$TMP_VERSION_FILE" "$VERSION_FILE"

    git add "$VERSION_FILE"
    git commit -m "version: bump omniSimVersion to $TARGET_VERSION" >/dev/null \
        || err "failed to commit omniSimVersion bump
       (check pre-commit hooks and signing config; the file is staged)"
    log "version-bump  : committed omniSimVersion -> $TARGET_VERSION"
fi

# ---- resolve source tree ----------------------------------------------------
SOURCE_SHA="$(git rev-parse --verify "$FROM_REF^{commit}" 2>/dev/null)" \
    || err "could not resolve source ref '$FROM_REF' in private repo"
SOURCE_TREE="$(git rev-parse --verify "$FROM_REF^{tree}")"

DENY_LIST="$SCRIPT_DIR/publish_deny.txt"

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
if git ls-remote --tags --exit-code public "refs/tags/$VERSION" >/dev/null 2>&1; then
    err "tag $VERSION already exists on $PUBLIC_REMOTE — pick a new version"
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
    while IFS= read -r entry || [[ -n "$entry" ]]; do
        # Strip whitespace.
        entry="${entry#"${entry%%[![:space:]]*}"}"
        entry="${entry%"${entry##*[![:space:]]}"}"
        # Skip blank lines and comments.
        [[ -z "$entry" || "${entry:0:1}" == "#" ]] && continue
        # Use a sentinel to tell whether anything actually got removed.
        before="$(git ls-files -- "$entry" | wc -l)"
        if [[ "$before" -gt 0 ]]; then
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
    # Re-snapshot the filtered index into a tree object.
    SNAPSHOT_TREE="$(git write-tree)"
    log "filtered tree : $SNAPSHOT_TREE"
else
    log "no deny-list found — publishing unfiltered source tree"
fi

# ---- build the snapshot commit ---------------------------------------------
PARENT_ARGS=()
if [[ $PUBLIC_HAS_MAIN -eq 1 ]]; then
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

COMMIT_MSG_ARGS=(-m "$COMMIT_TITLE")
if [[ -n "$COMMIT_BODY" ]]; then
    BODY_LINES="$(printf '%s\n' "$COMMIT_BODY" | wc -l | tr -d ' ')"
    log "release notes : extracted $BODY_LINES line(s) from CHANGELOG.md"
    COMMIT_MSG_ARGS+=(-m "$COMMIT_BODY")
else
    log "WARNING: no '## [$VERSION] — …' section found in CHANGELOG.md"
    log "         publishing with title-only commit message — add a section"
    log "         before --push if you want release notes on the GitHub Release"
fi

SNAPSHOT_SHA="$(
    GIT_AUTHOR_NAME="$PUBLISH_NAME" \
    GIT_AUTHOR_EMAIL="$PUBLISH_EMAIL" \
    GIT_COMMITTER_NAME="$PUBLISH_NAME" \
    GIT_COMMITTER_EMAIL="$PUBLISH_EMAIL" \
    git commit-tree "$SNAPSHOT_TREE" "${PARENT_ARGS[@]}" "${COMMIT_MSG_ARGS[@]}"
)"
log "snapshot sha  : $SNAPSHOT_SHA"

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

if [[ $PUBLIC_HAS_MAIN -eq 1 ]]; then
    log "diff vs public/main:"
    git --no-pager diff --stat "$PARENT_SHA" "$SNAPSHOT_SHA" | tail -30
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
log "pushing $SNAPSHOT_SHA to $PUBLIC_REMOTE main…"
OMNISIM_SKIP_PUSH_CHECK=1 git push public "$SNAPSHOT_SHA:refs/heads/main"
log "pushing tag $VERSION…"
OMNISIM_SKIP_PUSH_CHECK=1 git push public "refs/tags/$VERSION"
log "published $VERSION."

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
    # Extract owner/repo from PUBLIC_REMOTE. Supports git@host:owner/repo.git
    # and https://host/owner/repo.git URL forms.
    PUBLIC_OWNER_REPO="$(printf '%s' "$PUBLIC_REMOTE" \
        | sed -E 's#^git@[^:]+:##; s#^https?://[^/]+/##; s#\.git$##')"

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
