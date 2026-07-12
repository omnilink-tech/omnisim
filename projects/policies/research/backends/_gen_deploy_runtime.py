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

"""One-shot generator: writes g1_deploy_runtime.py from WbNewtonBackend.cpp.

Run from the repo root:  python projects/policies/research/backends/_gen_deploy_runtime.py

It copies the embedded kNewtonRuntimeSource Python block VERBATIM, prepends a
module docstring, and appends the (non-verbatim) sync-guard helpers. The sync
guard re-reads the .cpp block at runtime and asserts it still equals the
verbatim body, so the extract can never silently drift from the deploy.

This generator is kept in-repo so the extract can be regenerated deterministically
if the deploy runtime is intentionally updated (regenerate, then commit both).
"""
from pathlib import Path

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
CPP = REPO / "src" / "omnisim" / "physics" / "WbNewtonBackend.cpp"
OUT = REPO / "projects" / "rl" / "backends" / "g1_deploy_runtime.py"

START = 'kNewtonRuntimeSource = R"PY('
END = '\n)PY";'

DOCSTRING = (
    '"""VERBATIM extract of kNewtonRuntimeSource from WbNewtonBackend.cpp '
    "-- the deploy's physics runtime. Keep in sync.\n"
    "\n"
    "This module is a byte-for-byte copy of the embedded Python source the\n"
    "OmniSim native backend executes at deploy time (the ``class World`` that\n"
    "builds + steps the Newton model). It is extracted here so the golden-\n"
    "parity test can build the LITERAL deploy model in-process and diff its\n"
    "compiled MuJoCo model against the trainer's. ``check_in_sync()`` (and the\n"
    'paired pytest) re-reads the R"PY(...)PY" block out of the .cpp and asserts\n'
    "it still equals the body of this module, so the copy can never silently\n"
    "drift from the deploy. Regenerate with _gen_deploy_runtime.py.\n"
    '"""\n'
)

SYNC = r'''

# ===========================================================================
# Sync guard -- NOT part of the verbatim deploy source. These helpers re-read
# the embedded Python block out of WbNewtonBackend.cpp and assert it still
# matches the verbatim body above, so this extract can never drift from the
# deploy without the test going red.
# ===========================================================================
from pathlib import Path as _Path  # noqa: E402

_CPP_PATH = (next(_p for _p in _Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
             / "src" / "omnisim" / "physics" / "WbNewtonBackend.cpp")
_START_MARK = 'kNewtonRuntimeSource = R"PY('
_END_MARK = '\n)PY";'
_GUARD_MARK = "# " + "=" * 75 + "\n# Sync guard"


def _normalize(text):
    """Strip trailing whitespace per line + leading/trailing blank lines so the
    comparison is insensitive to editor whitespace policy and to the single
    newline that separates the docstring from the verbatim body."""
    lines = [ln.rstrip() for ln in text.split("\n")]
    while lines and lines[0] == "":
        lines.pop(0)
    while lines and lines[-1] == "":
        lines.pop()
    return "\n".join(lines)


def deploy_runtime_source_from_cpp(cpp_path=None):
    """Return the verbatim Python block embedded in WbNewtonBackend.cpp."""
    p = _Path(cpp_path) if cpp_path is not None else _CPP_PATH
    cpp = p.read_text(encoding="utf-8")
    i = cpp.index(_START_MARK) + len(_START_MARK)
    assert cpp[i] == "\n", 'expected a newline right after R"PY('
    i += 1
    j = cpp.index(_END_MARK, i)
    return cpp[i:j]


def _this_module_verbatim_body():
    """The verbatim deploy block stored in THIS module: everything between the
    end of the module docstring and the sync-guard delimiter below."""
    src = _Path(__file__).read_text(encoding="utf-8")
    # End of the module docstring: the close of the SECOND triple-quote.
    first = src.index('"""')
    body_start = src.index('"""', first + 3) + 3
    body_end = src.index(_GUARD_MARK)
    return src[body_start:body_end]


def check_in_sync(cpp_path=None):
    """Assert this module's verbatim body == the .cpp embedded block (normalized
    for trailing whitespace). Raises AssertionError with a hint on drift."""
    cpp_block = _normalize(deploy_runtime_source_from_cpp(cpp_path))
    mod_block = _normalize(_this_module_verbatim_body())
    if cpp_block != mod_block:
        cl = cpp_block.split("\n")
        ml = mod_block.split("\n")
        first = next((k for k in range(min(len(cl), len(ml)))
                      if cl[k] != ml[k]), min(len(cl), len(ml)))
        msg = ("g1_deploy_runtime.py has DRIFTED from "
               "WbNewtonBackend.cpp's kNewtonRuntimeSource.\n"
               "  cpp lines=%d module lines=%d\n"
               "  first differing line index=%d\n" % (len(cl), len(ml), first))
        if first < len(cl):
            msg += "  cpp[%d]=%r\n" % (first, cl[first])
        if first < len(ml):
            msg += "  mod[%d]=%r\n" % (first, ml[first])
        raise AssertionError(msg)
    return True


if __name__ == "__main__":
    check_in_sync()
    print("g1_deploy_runtime.py IS IN SYNC with WbNewtonBackend.cpp "
          "kNewtonRuntimeSource.")
'''


def main():
    cpp = CPP.read_text(encoding="utf-8")
    i = cpp.index(START) + len(START)
    assert cpp[i] == "\n", 'expected a newline right after R"PY('
    i += 1
    j = cpp.index(END, i)
    block = cpp[i:j]
    out = DOCSTRING + block
    if not out.endswith("\n"):
        out += "\n"
    out = out + SYNC
    OUT.write_text(out, encoding="utf-8")
    print("wrote", OUT, "lines:", out.count("\n") + 1,
          "| verbatim block lines:", block.count("\n") + 1)


# ===========================================================================
# REVERSE direction (Phase 2 of the train<->deploy unification): make the
# shared .py the CANONICAL source and (re)generate the C++ string literal(s)
# from it. DORMANT by default -- the forward direction above is still canonical
# today. Running --write-cpp is part of the (rebuild-gated) Layer-C promotion
# (see docs/developer/train-deploy-unification.md). Both modes are guarded by a
# byte-exact re-extraction check so they can never silently corrupt the .cpp.
# ===========================================================================
SMOKE_CPP = REPO / "src" / "omnisim" / "physics" / "newton_embed_smoke.cpp"
_GUARD_MARK = "# " + "=" * 75 + "\n# Sync guard"


def _canonical_py_block():
    """The verbatim deploy block as stored in g1_deploy_runtime.py -- exactly
    the string the forward generator embedded between the .cpp R"PY( ... )PY"
    markers (the block, with the docstring + sync-guard stripped)."""
    src = OUT.read_text(encoding="utf-8")
    first = src.index('"""')
    body_start = src.index('"""', first + 3) + 3
    body_end = src.index(_GUARD_MARK)
    # forward gen wrote: DOCSTRING + block + ('\n') + SYNC, and SYNC begins
    # '\n\n' + _GUARD_MARK. The captured .cpp block (cpp[i:j], j at '\n)PY";')
    # never ends in a newline, so stripping the trailing newlines recovers it.
    return src[body_start:body_end].rstrip("\n")


def _splice_into_cpp(cpp_text: str, block: str) -> str:
    i = cpp_text.index(START) + len(START)
    assert cpp_text[i] == "\n", 'expected a newline right after R"PY('
    i += 1
    j = cpp_text.index(END, i)
    return cpp_text[:i] + block + cpp_text[j:]


def _forward_block(cpp_text: str) -> str:
    i = cpp_text.index(START) + len(START) + 1   # past R"PY( and its newline
    j = cpp_text.index(END, i)
    return cpp_text[i:j]


def check_reverse():
    """Prove the reverse splice is SEMANTICALLY exact: embedding the canonical
    .py block into the .cpp(s) and re-extracting reproduces the .py block
    byte-for-byte (no corruption). Verifiable NOW (no rebuild needed).

    Note: this is a round-trip guarantee, not a literal byte no-op against the
    pre-existing .cpp -- the current .cpp may carry cosmetic whitespace (e.g. a
    trailing blank line before )PY";) that the .py mirror normalized away; the
    flip would normalize it. That is reported as informational, not a failure.
    """
    block = _canonical_py_block()
    ok = True
    for path in (CPP, SMOKE_CPP):
        if not path.exists():
            print("  [skip] missing", path)
            continue
        txt = path.read_text(encoding="utf-8")
        if START not in txt:
            print("  [skip] no kNewtonRuntimeSource marker in", path.name)
            continue
        new = _splice_into_cpp(txt, block)
        roundtrips = _forward_block(new) == block      # the real guarantee
        cosmetic = (new != txt)                        # whitespace-only delta
        ok = ok and roundtrips
        tag = "byte-identical" if not cosmetic else "cosmetic whitespace delta"
        print(f"  {path.name}: round-trip exact = {roundtrips}  ({tag})")
    if not ok:
        raise AssertionError(
            "reverse splice does NOT round-trip -- block recovery drifted from "
            "the canonical .py. Run forward gen / fix sync first.")
    print("reverse round-trip OK (re-extraction reproduces the canonical .py "
          "block byte-for-byte).")
    return True


def write_cpp():
    """FLIP the source of truth: regenerate the C++ literal(s) from the canonical
    .py. Writes only after a per-file re-extraction confirms byte-exactness.
    DORMANT -- intended for the rebuild-gated Layer-C promotion only."""
    block = _canonical_py_block()
    for path in (CPP, SMOKE_CPP):
        if not path.exists() or START not in path.read_text(encoding="utf-8"):
            print("  [skip]", path.name)
            continue
        txt = path.read_text(encoding="utf-8")
        new = _splice_into_cpp(txt, block)
        if _forward_block(new) != block:   # safety: must round-trip
            raise AssertionError(f"refusing to write {path.name}: re-extraction "
                                 f"would not reproduce the canonical .py block.")
        path.write_text(new, encoding="utf-8")
        print("  wrote", path.name, "from canonical g1_deploy_runtime.py")
    print("Layer-C flipped to .py-canonical. A NATIVE REBUILD is required for "
          "the change to take effect in the binary.")


if __name__ == "__main__":
    import sys
    if "--check-reverse" in sys.argv:
        check_reverse()
    elif "--write-cpp" in sys.argv:
        write_cpp()
    else:
        main()
