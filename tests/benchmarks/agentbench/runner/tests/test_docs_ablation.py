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

"""The docs-ablation cell's read deny-list (SPEC 6.3; plan Phase R item 6).

The load-bearing property, tested here rather than promised: a read-denied
path is **indistinguishable from a nonexistent one** through the file tools --
byte-identical error text (modulo the path itself), absent from listings, and
absent from listing *counts* -- so the ablated agent cannot infer what is
being hidden from the shape of the refusal. And the empty deny-list is a
no-op: same behaviour, same ``tools_sha256``, same ``manifest_sha256`` as the
baseline, so a baseline row's identity is untouched by this feature existing.
"""

from __future__ import annotations

import os

import pytest

from agentbench.runner.config import RunnerConfig
from agentbench.runner.isolation import (READ_DENY_PRESETS, Sandbox,
                                         parse_read_deny)
from agentbench.runner.tools import get_tool_set


def call(tool_set, name, args, timeout=30.0):
    return tool_set.get(name).handler(args, timeout)


@pytest.fixture()
def fake_repo(tmp_path):
    """A miniature repo so the deny mechanics are tested against known files
    rather than whatever the real tree happens to contain."""
    repo = tmp_path / "repo"
    (repo / "docs" / "developer" / "deep").mkdir(parents=True)
    (repo / "docs" / "guide").mkdir(parents=True)
    (repo / "README.md").write_text("readme\n", encoding="utf-8")
    (repo / "manual.md").write_text("the manual\n", encoding="utf-8")
    (repo / "docs" / "developer" / "secret.md").write_text("s\n",
                                                           encoding="utf-8")
    (repo / "docs" / "developer" / "deep" / "nested.md").write_text(
        "n\n", encoding="utf-8")
    (repo / "docs" / "guide" / "intro.md").write_text("i\n", encoding="utf-8")
    return repo


def make_sandbox(tmp_path, repo, read_deny):
    return Sandbox.create(tmp_path / "run", tmp_path / "run" / "scratch",
                          repo=repo, ports=False, read_deny=read_deny)


@pytest.fixture()
def ablated(tmp_path, fake_repo):
    sb = make_sandbox(tmp_path, fake_repo, "manual.md:docs/developer")
    return sb, get_tool_set("shell", sb)


# -- (a) unreadable, and indistinguishable from nonexistent -----------------

def test_denied_file_is_unreadable(ablated, fake_repo):
    _, tools = ablated
    out = call(tools, "read_file", {"path": str(fake_repo / "manual.md")})
    assert out.is_error
    assert "the manual" not in out.text


def test_denied_read_error_is_byte_identical_to_nonexistent(ablated,
                                                            fake_repo):
    """The agent must not be able to tell "hidden" from "missing"."""
    _, tools = ablated
    denied = call(tools, "read_file", {"path": str(fake_repo / "manual.md")})
    missing = call(tools, "read_file", {"path": str(fake_repo / "ghost.md")})
    assert denied.is_error and missing.is_error
    # Same template, differing only by the path the caller supplied.
    d = denied.text.replace(str(fake_repo / "manual.md"), "<P>")
    m = missing.text.replace(str(fake_repo / "ghost.md"), "<P>")
    assert d == m == "error: not a file: <P>"
    # No side-channel through the structured result either.
    assert denied.data == missing.data


def test_denied_dir_error_is_byte_identical_to_nonexistent(ablated,
                                                           fake_repo):
    _, tools = ablated
    denied = call(tools, "list_dir",
                  {"path": str(fake_repo / "docs" / "developer")})
    missing = call(tools, "list_dir",
                   {"path": str(fake_repo / "docs" / "nowhere")})
    assert denied.is_error and missing.is_error
    d = denied.text.replace(str(fake_repo / "docs" / "developer"), "<P>")
    m = missing.text.replace(str(fake_repo / "docs" / "nowhere"), "<P>")
    assert d == m == "error: not a directory: <P>"


def test_denied_paths_are_invisible_in_listings_and_counts(ablated,
                                                           fake_repo):
    _, tools = ablated
    flat = call(tools, "list_dir", {"path": str(fake_repo),
                                    "max_entries": 1000})
    names = {e["name"] for e in flat.data["entries"]}
    assert "README.md" in names
    assert "manual.md" not in names
    # The count must not leak existence: total_seen == what was shown.
    assert flat.data["total_seen"] == len(flat.data["entries"])


def test_hidden_cwd_reads_as_nonexistent(ablated, fake_repo):
    _, tools = ablated
    denied = call(tools, "run_shell",
                  {"command": "echo hi",
                   "cwd": str(fake_repo / "docs" / "developer")})
    missing = call(tools, "run_shell",
                   {"command": "echo hi",
                    "cwd": str(fake_repo / "docs" / "nowhere")})
    assert denied.is_error and missing.is_error
    d = denied.text.replace(str(fake_repo / "docs" / "developer"), "<P>")
    m = missing.text.replace(str(fake_repo / "docs" / "nowhere"), "<P>")
    assert d == m


# -- (b) siblings stay readable ---------------------------------------------

def test_sibling_non_denied_paths_still_readable(ablated, fake_repo):
    _, tools = ablated
    ok = call(tools, "read_file", {"path": str(fake_repo / "README.md")})
    assert not ok.is_error and "readme" in ok.text
    ok = call(tools, "read_file",
              {"path": str(fake_repo / "docs" / "guide" / "intro.md")})
    assert not ok.is_error
    docs = call(tools, "list_dir", {"path": str(fake_repo / "docs"),
                                    "max_entries": 1000})
    names = {e["name"] for e in docs.data["entries"]}
    assert "guide" in names
    assert "developer" not in names


# -- (c) a directory deny covers everything under it ------------------------

def test_directory_deny_covers_nested_files(ablated, fake_repo):
    _, tools = ablated
    for rel in (("docs", "developer", "secret.md"),
                ("docs", "developer", "deep", "nested.md")):
        out = call(tools, "read_file", {"path": str(fake_repo.joinpath(*rel))})
        assert out.is_error and out.text.startswith("error: not a file:")
    deep = call(tools, "list_dir", {"path": str(fake_repo),
                                    "recursive": True, "max_entries": 1000})
    names = {e["name"].replace("\\", "/") for e in deep.data["entries"]}
    assert "docs/guide/intro.md" in names
    assert not any("developer" in n for n in names)
    assert deep.data["total_seen"] == len(deep.data["entries"])


# -- (d) the empty deny-list is a no-op -------------------------------------

def test_empty_deny_changes_nothing(tmp_path, fake_repo, monkeypatch):
    monkeypatch.delenv("AGENTBENCH_READ_DENY", raising=False)
    default = Sandbox.create(tmp_path / "a", tmp_path / "a" / "s",
                             repo=fake_repo, ports=False)          # env unset
    explicit = make_sandbox(tmp_path / "b", fake_repo, "")         # explicit ""
    ts_default = get_tool_set("shell", default)
    ts_explicit = get_tool_set("shell", explicit)
    # Identical hashes: the feature existing must not move a baseline row's
    # identity (tools_sha256 is the cross-sim fairness hash; manifest_sha256
    # is the row's condition identity).
    assert ts_default.tools_sha256 == ts_explicit.tools_sha256
    assert ts_default.manifest_sha256 == ts_explicit.manifest_sha256
    assert "read_deny" not in ts_default.env_policy
    assert default.read_deny == () and explicit.read_deny == ()
    # ...and identical behaviour: everything is readable.
    ok = call(ts_explicit, "read_file", {"path": str(fake_repo / "manual.md")})
    assert not ok.is_error and "the manual" in ok.text


def test_nonempty_deny_changes_manifest_hash_but_not_tools_hash(tmp_path,
                                                                fake_repo):
    """The attribution property: an ablation row is mechanically identifiable
    by its manifest_sha256, while the model-facing tool definitions -- the
    cross-simulator fairness hash -- stay byte-identical."""
    base = make_sandbox(tmp_path / "a", fake_repo, "")
    abl = make_sandbox(tmp_path / "b", fake_repo, "docs_ablation")
    ts_base = get_tool_set("shell", base)
    ts_abl = get_tool_set("shell", abl)
    assert ts_base.tools_sha256 == ts_abl.tools_sha256
    assert ts_base.manifest_sha256 != ts_abl.manifest_sha256
    assert ts_abl.env_policy["read_deny"]["roots"] == list(
        READ_DENY_PRESETS["docs_ablation"])


# -- (e) traversal and aliasing cannot escape the deny ----------------------

def test_dotdot_traversal_cannot_reach_a_denied_path(ablated, fake_repo):
    _, tools = ablated
    out = call(tools, "read_file",
               {"path": str(fake_repo / "docs" / "developer" / ".."
                            / "developer" / "secret.md")})
    assert out.is_error and out.text.startswith("error: not a file:")
    out = call(tools, "read_file",
               {"path": str(fake_repo / "docs" / "guide" / ".."
                            / "developer" / "secret.md")})
    assert out.is_error and out.text.startswith("error: not a file:")


def test_relative_traversal_from_scratch_cannot_reach_a_denied_path(ablated,
                                                                    fake_repo):
    sandbox, tools = ablated
    rel = os.path.relpath(str(fake_repo / "manual.md"),
                          str(sandbox.scratch_dir))
    out = call(tools, "read_file", {"path": rel})
    assert out.is_error and out.text.startswith("error: not a file:")


def test_symlink_cannot_alias_a_denied_path_back_into_view(ablated,
                                                           fake_repo):
    sandbox, tools = ablated
    link = sandbox.scratch_dir / "innocent.md"
    try:
        os.symlink(str(fake_repo / "manual.md"), str(link))
    except (OSError, NotImplementedError):
        pytest.skip("cannot create symlinks on this host (needs privilege "
                    "on Windows)")
    out = call(tools, "read_file", {"path": str(link)})
    assert out.is_error and out.text.startswith("error: not a file:")
    # And a symlinked directory does not resurface hidden children.
    dlink = sandbox.scratch_dir / "ddir"
    os.symlink(str(fake_repo / "docs" / "developer"), str(dlink),
               target_is_directory=True)
    out = call(tools, "read_file", {"path": str(dlink / "secret.md")})
    assert out.is_error and out.text.startswith("error: not a file:")


# -- parsing and presets ----------------------------------------------------

def test_parse_read_deny_preset_and_paths():
    assert parse_read_deny("") == ()
    assert parse_read_deny(None) == ()
    assert (parse_read_deny("docs_ablation")
            == READ_DENY_PRESETS["docs_ablation"]
            == ("AGENTS.md", "CLAUDE.md", "docs/developer"))
    # Hyphen/case-insensitive preset name; mixing preset + explicit path.
    assert parse_read_deny("Docs-Ablation") == parse_read_deny("docs_ablation")
    assert parse_read_deny("docs_ablation;projects/foo") == (
        "AGENTS.md", "CLAUDE.md", "docs/developer", "projects/foo")
    # Both separators; backslashes normalized; duplicates collapsed.
    assert parse_read_deny("a/b:c;a\\b") == ("a/b", "c")


def test_parse_read_deny_rejects_malformed_entries_loudly():
    """A silently dropped deny entry would run the wrong cell while
    attributing its rows to the ablation -- so it must raise, not warn."""
    for bad in ("/abs/path", "..", ".", "a/../..", "../up"):
        with pytest.raises(ValueError):
            parse_read_deny(bad)


# -- the env var drives config and sandbox consistently ---------------------

def test_env_var_reaches_config_sandbox_and_artifacts(tmp_path, fake_repo,
                                                      monkeypatch):
    monkeypatch.setenv("AGENTBENCH_READ_DENY", "docs_ablation")
    cfg = RunnerConfig.from_env()
    sb = Sandbox.create(tmp_path / "run", tmp_path / "run" / "scratch",
                        repo=fake_repo, ports=False)   # no explicit arg: env
    want = READ_DENY_PRESETS["docs_ablation"]
    assert cfg.read_deny == want                # -> runner_config trace event
    assert sb.read_deny == want                 # -> enforced by the guard
    assert cfg.as_dict()["read_deny"] == want
    assert sb.as_dict()["read_deny"] == list(want)   # -> runner_result.json
    # The agent's own environment must NOT carry the deny list (AGENTBENCH_*
    # is stripped) -- the list's absence from env is part of invisibility.
    assert "AGENTBENCH_READ_DENY" not in sb.env()


def test_leak_detector_flags_mentions_of_hidden_roots(ablated, fake_repo):
    """run_shell cannot be confined; a mention of an ablated absolute path in
    a command or result must at least be greppable as leak_suspect."""
    sandbox, _ = ablated
    assert sandbox.guard.flags("cat " + str(fake_repo / "manual.md"))
    assert sandbox.guard.flags(
        str(fake_repo / "docs" / "developer" / "secret.md"))
    assert not sandbox.guard.flags("echo hello")
    assert not sandbox.guard.flags(str(fake_repo / "README.md"))


# -- the real tree, the real preset -----------------------------------------

def test_real_repo_docs_ablation_hides_the_manual_and_only_the_manual(
        tmp_path):
    """Against the actual checkout: the preset hides AGENTS.md, CLAUDE.md and
    docs/developer/, and deliberately KEEPS DEMOS.md and the harness README
    readable (the rationale is in runner/config.py)."""
    from agentbench.common.paths import REPO
    sb = make_sandbox(tmp_path, REPO, "docs_ablation")
    tools = get_tool_set("shell", sb)
    for rel in ("AGENTS.md", "CLAUDE.md", "docs/developer/rl-current-state.md"):
        out = call(tools, "read_file", {"path": str(REPO / rel)})
        assert out.is_error and out.text.startswith("error: not a file:"), rel
    for rel in ("README.md", "DEMOS.md", "scripts/harness/README.md",
                "docs/WORLD_RECIPE.md"):
        out = call(tools, "read_file", {"path": str(REPO / rel)})
        assert not out.is_error, "%s must stay readable in the ablation" % rel
    root = call(tools, "list_dir", {"path": str(REPO), "max_entries": 10000})
    names = {e["name"] for e in root.data["entries"]}
    assert "AGENTS.md" not in names and "CLAUDE.md" not in names
    assert "README.md" in names
    docs = call(tools, "list_dir", {"path": str(REPO / "docs"),
                                    "max_entries": 10000})
    dnames = {e["name"] for e in docs.data["entries"]}
    assert "developer" not in dnames
