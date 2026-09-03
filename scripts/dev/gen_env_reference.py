#!/usr/bin/env python3
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
"""Generate docs/reference/environment-variables.md from the tracked source tree.

An agent cannot enumerate the engine's knobs by reading AGENTS.md: a 2026-09-02
audit found 283 distinct ``OMNISIM_*`` variables read across the tree, 92 of them
documented nowhere, and 44 read presence-only (so ``=0`` ARMS them -- the
``OMNISIM_REQUIRE_NEWTON`` trap class). This script derives the reference page
from the code, so it cannot drift:

    python scripts/dev/gen_env_reference.py            # rewrite the page in place
    python scripts/dev/gen_env_reference.py --check    # exit 1 if the committed page is stale
    python scripts/dev/gen_env_reference.py --stats    # totals, top readers, inconsistent reads
    python scripts/dev/gen_env_reference.py --out F    # write elsewhere (the docs test does this)

What counts as a READ (per language, per occurrence of the literal name):

  C/C++/Python   the literal is the argument of an env-bearing call or subscript
                 (``getenv``, ``qgetenv``, ``qEnvironmentVariable*``, ``std::getenv``,
                 ``os.environ[...]``, ``os.environ.get``, ``os.getenv``, ``env.get``,
                 a ``QProcessEnvironment::value/contains``, or a helper whose name
                 contains ``env``), or the left side of ``in os.environ`` / ``in env``.
  sh / yml       ``$X``, ``${X...}``, ``${{ env.X }}``; Makefile ``$(X)`` / ``ifdef X`` /
                 ``X ?=`` (a ``$(X)`` is NOT a read in a file that assigns X
                 unconditionally -- that is a make variable, not the environment);
                 PowerShell ``$env:X``; batch ``%X%`` / ``if defined X``.

Writes are excluded: ``putenv``/``qputenv``/``setenv``, ``env[X] = ``, ``.pop``/
``.setdefault``/``del``, ``export X=``, ``set X=``, ``$env:X =``, a YAML ``X:`` env
entry, an inline ``X=value cmd`` prefix. Comments, docstrings and plain string
literals (help text, log messages, dict keys) are neither.

The read KIND is a heuristic on the read line plus, when the result is assigned,
the next four lines that use the assigned name:

  presence   only the existence of the variable is tested (``IsSet``/``IsEmpty``,
             ``if getenv(...)``, ``"X" in os.environ``, ``[ -n "$X" ]``) -- so ``=0``
             ARMS it; UNSET it to disarm.
  value      the string is compared or lowered (``== "0"``, ``.trimmed().toLower()``,
             ``.lower() in (...)``) -- ``0``/``false``/``off`` mean OFF.
  int        parsed as a number (``IntValue``, ``.toInt()``, ``int(...)``).
  string     used as a path, name or opaque value.
  mixed      presence at some read sites and value/int/string at others -- the
             inconsistent class: ``=0`` arms one site and disarms another.
"""
from __future__ import annotations

import argparse
import io
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir))
OUTPUT_REL = "docs/reference/environment-variables.md"
SELF_REL = "scripts/dev/gen_env_reference.py"
TEST_REL = "docs/tests/test_env_reference.py"

SOURCE_EXTS = {".cpp", ".hpp", ".c", ".h", ".py", ".sh", ".ps1", ".bat", ".yml", ".mk"}
C_LIKE = {".cpp", ".hpp", ".c", ".h"}
# Dev-only trees that never ship, plus frozen artefacts: scanning them would list
# reads of copies (social/ carries whole snapshots of scripts/) rather than sources.
EXCLUDED_PREFIXES = ("social/", "cloud/", "_scratch/", "distribution/", ".github/workflows.disabled/")
EXCLUDED_RE = re.compile(r"^tests/benchmarks/.*/results?/")
DOC_ROOTS = ("AGENTS.md", "README.md", "PROTOCOL.md", "CHANGELOG.md")

# Order matters twice: a variable is filed under the FIRST area (top to bottom)
# that reads it, and `harness` must precede `scripts` because it nests inside it.
AREAS = (
    ("engine", "Engine (`src/omnisim`)", ("src/omnisim/",)),
    ("controller", "Controller library (`src/controller`, `lib/controller`, `include/controller`)",
     ("src/controller/", "lib/controller/", "include/controller/")),
    ("cli", "Python CLI (`omnisim/`)", ("omnisim/",)),
    ("harness", "Harness (`scripts/harness`)", ("scripts/harness/",)),
    ("scripts", "Scripts (`scripts/`)", ("scripts/",)),
    ("policies", "Policies (`projects/policies`)", ("projects/policies/",)),
    ("packages", "Packages (`packages/`)", ("packages/",)),
    ("other", "Everything else (tests, samples, resources, CI workflows)", ()),
)

VAR_RE = re.compile(r"\bOMNISIM_[A-Z0-9_]+\b")
MAX_SITES = 5
MAX_DESC = 160
KIND_ORDER = ("value", "int", "presence", "string")  # precedence when one line reads a name twice


# --------------------------------------------------------------------------- files

def tracked_files(root: str) -> list[str]:
    out = subprocess.run(["git", "ls-files", "-z"], cwd=root, capture_output=True, check=True).stdout
    return sorted(p.decode("utf-8", "replace") for p in out.split(b"\0") if p)


def lang_of(path: str) -> str | None:
    base = path.rsplit("/", 1)[-1]
    if base == "Makefile" or base.startswith("Makefile."):
        return "make"
    ext = os.path.splitext(base)[1].lower()
    if ext not in SOURCE_EXTS:
        return None
    if ext in C_LIKE:
        return "c"
    if ext == ".mk":
        return "make"
    return ext[1:]  # py, sh, ps1, bat, yml


def is_scanned(path: str) -> bool:
    if path in (OUTPUT_REL, SELF_REL, TEST_REL):
        return False
    if path.startswith(EXCLUDED_PREFIXES) or EXCLUDED_RE.match(path):
        return False
    return lang_of(path) is not None


def area_of(path: str) -> str:
    for key, _title, prefixes in AREAS:
        if any(path.startswith(p) for p in prefixes):
            return key
    return "other"


def read_text(root: str, rel: str) -> str:
    with open(os.path.join(root, rel), "rb") as f:
        return f.read().decode("utf-8", "replace")


# ------------------------------------------------------------------ comment stripping

def strip_comments(lines: list[str], lang: str) -> tuple[list[str], list[bool]]:
    """Return (code_lines, is_comment_line). Heuristic, line-based."""
    code: list[str] = []
    is_comment: list[bool] = []
    in_block = False  # C /* */, PowerShell <# #>, Python triple-quoted string
    open_tok, close_tok = ("/*", "*/") if lang == "c" else ("<#", "#>") if lang == "ps1" else (None, None)
    for raw in lines:
        line = raw.rstrip("\n")
        stripped = line.strip()
        comment_line = in_block
        prev_comment = bool(is_comment[-1]) if is_comment else False
        if lang == "py":
            # Docstring lines are marked 2 (not True): they are harvestable as a
            # description only when they NAME the variable, never as the nearest block.
            n = stripped.count('"""') + stripped.count("'''")
            if in_block:
                comment_line = 2
                if n % 2 == 1:
                    in_block = False
                line = ""
            elif n % 2 == 1:
                in_block = True
                idx = min(i for i in (line.find('"""'), line.find("'''")) if i >= 0)
                line = line[:idx]
                comment_line = 2 if stripped.startswith(('"""', "'''")) else False
            elif n and stripped.startswith(('"""', "'''")):
                comment_line = 2
                line = ""
        elif open_tok:
            work = line
            out = ""
            while work:
                if in_block:
                    j = work.find(close_tok)
                    if j < 0:
                        work = ""
                        break
                    work = work[j + len(close_tok):]
                    in_block = False
                    continue
                i = work.find(open_tok)
                if i < 0:
                    out += work
                    break
                out += work[:i]
                work = work[i + len(open_tok):]
                in_block = True
            line = out
            if lang == "c":
                # A leading `*` continues a block comment only inside one or right after a
                # comment line; `*ptr = x;` is code.
                star = stripped.startswith("*") and (comment_line or (prev_comment and (
                    stripped == "*" or stripped.startswith(("* ", "*/", "*\t")))))
                comment_line = comment_line or stripped.startswith(("//", "/*")) or star
        if lang == "c":
            i = line.find("//")
            while i > 0 and line[i - 1] == ":":  # http://
                i = line.find("//", i + 2)
            if i >= 0:
                line = line[:i]
        elif lang == "bat":
            low = stripped.lower()
            if low.startswith("::") or low == "rem" or low.startswith("rem "):
                comment_line = True
                line = ""
        else:  # py sh ps1 yml make
            m = re.search(r"(^|\s)#", line)
            if m and not (lang == "sh" and line[: m.end()].endswith("${#")):
                line = line[: m.start()]
            comment_line = comment_line or stripped.startswith("#")
        code.append(line)
        is_comment.append(comment_line)
    return code, is_comment


# ------------------------------------------------------------------ read detection

WRITE_CALLEE_RE = re.compile(r"^(q?put|q?unset|set)_?env|^(insert|remove|pop|popitem|setdefault|update|unset|set|discard|delete)$")
ENV_CALL_RE = re.compile(r"([\w.:>-]*env[\w.:>-]*(?:\(\))?(?:\.\w+)?)\s*\(\s*$", re.I)
ENV_SUBSCRIPT_RE = re.compile(r"[\w.:>-]*env[\w.:>-]*\s*\[\s*$", re.I)
DEL_RE = re.compile(r"\bdel\s+[\w.]*env[\w.]*\s*\[\s*$", re.I)
IN_ENV_RE = re.compile(r"""^["']\s+(?:not\s+)?in\s+[\w.]*env\w*""", re.I)
SUBSCRIPT_WRITE_RE = re.compile(r"""^["']\s*\]\s*=(?!=)""")
ASSIGN_RE = re.compile(r"^\s*(?:const\s+|static\s+|auto\s+|global\s+)*(?:[\w:<>*&]+\s+)*?[*&]?(\w+)\s*(?:\[\d*\])?\s*[:+]?=\s*[^=]")

PRESENCE_LINE_RES = [
    re.compile(r"\.isEmpty\(\)|\.isNull\(\)"),
    re.compile(r"(!=|==)\s*(NULL|nullptr)\b"),
    re.compile(r"\bif\s*\(\s*!?\s*(?:std::)?(?:q?getenv|secure_getenv)\s*\("),
    re.compile(r"!\s*(?:std::)?(?:q?getenv|secure_getenv|os\.getenv|os\.environ\.get|environ\.get)\s*\("),
    re.compile(r"""getenv\s*\(\s*["']\w+["']\s*\)\s*(&&|\|\||\?)"""),
    re.compile(r"""\bif\s+(?:not\s+)?(?:os\.)?(?:environ\.get|getenv|env\.get)\(\s*["']\w+["'][^)]*\)\s*(:|\band\b|\bor\b)"""),
    re.compile(r"\bbool\(\s*(?:os\.)?(?:environ|getenv|env)"),
    re.compile(r"\bis\s+(?:not\s+)?None\b"),
    re.compile(r"\bin\s+(?:os\.)?environ\b"),
]
INT_LINE_RES = [
    re.compile(r"IntValue\(|\.to(Int|UInt|Double|Float|LongLong|ULongLong)\(|\b(atoi|atof|atol|atoll|strto[a-z]+|sscanf)\(|\bstd::sto[a-z]+\(|\b(int|float)\("),
]
VALUE_LINE_RES = [
    re.compile(r"\.to(Lower|Upper)\(\)|\.(lower|upper|casefold)\(\)"),
    re.compile(r"""\s(==|!=)\s*["'](?!\\0')"""),  # `v[0] != '\0'` is a presence test, not a compare
    re.compile(r"""\b(compare|startsWith|startswith|endswith)\(\s*["']"""),
    re.compile(r"""\bin\s*[(\[{]\s*["']"""),
    re.compile(r"\btruthy\b|TRUTHY|_TRUE_VALUES|FALSE_VALUES|\bis_true\(|\bis_false\("),
]
NAME_PRESENCE_TMPL = (
    r"\b@N@\s*(!=|==)\s*(NULL|nullptr)\b",
    r"\b@N@\s*\[\s*0\s*\]",
    r"\bif\s*\(\s*!?\s*@N@\s*\)",
    r"!?\s*@N@\.(isEmpty|isNull)\(\)",
    r"\bif\s+(not\s+)?@N@\s*:",
    r"\b@N@\s+is\s+(not\s+)?None\b",
    r"\bbool\(\s*@N@\s*\)",
    r"\bnot\s+@N@\b",
)


def same_line_kind(line: str) -> str | None:
    if any(r.search(line) for r in VALUE_LINE_RES):
        return "value"
    if any(r.search(line) for r in INT_LINE_RES):
        return "int"
    if any(r.search(line) for r in PRESENCE_LINE_RES):
        return "presence"
    return None


def following_kind(name: str, window: list[str]) -> str | None:
    """Kind from the lines after an assignment, scoped to the assigned name."""
    if not name:
        return None
    name_re = re.compile(r"\b%s\b" % re.escape(name))
    uses = [ln for ln in window if name_re.search(ln)]
    if not uses:
        return None
    if any(r.search(ln) for ln in uses for r in VALUE_LINE_RES):
        return "value"
    if any(r.search(ln) for ln in uses for r in INT_LINE_RES):
        return "int"
    presence_res = [re.compile(t.replace("@N@", re.escape(name))) for t in NAME_PRESENCE_TMPL]
    total = sum(len(name_re.findall(ln)) for ln in uses)
    tested = 0
    for ln in uses:
        for r in presence_res:
            tested += len(r.findall(ln))
    if tested and tested >= total:
        return "presence"
    return None


def kind_c_py(callee: str, line: str, window: list[str], name: str = "") -> str:
    seg = re.split(r"[.:>-]+", callee)[-1].lower() if callee else ""
    if seg.endswith(("isset", "isempty", "contains", "is_set", "isdefined", "defined", "has")) or seg.startswith("has_"):
        # `IsSet("X") && IntValue("X") > 0`: the presence test only guards a parse of the same name.
        k = same_line_kind(line)
        if k in ("value", "int") and name and line.count(name) >= 2:
            return k
        return "presence"
    if seg.endswith("intvalue") or seg in ("int", "float") or re.search(r"(^|_)(int|num|float|number)(_|$)", seg):
        return "int"
    if re.search(r"bool|flag|truthy|enabled|switch", seg):
        return "value"
    k = same_line_kind(line)
    if k:
        return k
    m = ASSIGN_RE.match(line)
    k = following_kind(m.group(1) if m else "", window)
    return k or "string"


def classify_c_py(code: str, m: re.Match, window: list[str], prev_line: str = "") -> str | None:
    before, after = code[: m.start()], code[m.end():]
    if not (before.endswith(('"', "'")) and after.startswith(('"', "'"))):
        return None
    tail = before[:-1].rstrip()
    if not tail.strip() and prev_line.rstrip().endswith(("(", ",")):
        # `helper(\n    "OMNISIM_X", ...)`: the call opened on the previous line.
        tail = prev_line.rstrip()
        code = prev_line.rstrip() + " " + code.lstrip()
    if DEL_RE.search(tail):
        return None
    if ENV_SUBSCRIPT_RE.search(tail):
        if SUBSCRIPT_WRITE_RE.match(after):
            return None
        return kind_c_py("", code, window, m.group(0))
    if IN_ENV_RE.match(after):
        return "presence"
    call = ENV_CALL_RE.search(tail)
    if call:
        callee = call.group(1)
        seg = re.split(r"[.:>-]+", callee)[-1].lower()
        if WRITE_CALLEE_RE.search(seg):
            return None
        kind = kind_c_py(callee, code, window, m.group(0))
        if kind == "presence" and writes_name(m.group(0), window[:2]):
            return None  # `if not env.get(X): env[X] = default` sets a default; it reads nothing
        return kind
    helper = HELPER_CALL_RE.search(tail)
    if helper and helper.group(1) in ENV_HELPERS:
        # `_contact_value("OMNISIM_NEWTON_GROUND_MU", ...)`: a project helper that reads
        # the environment through its parameter (found by find_env_helpers()).
        kind = kind_c_py(helper.group(1), code, window, m.group(0))
        return ENV_HELPERS[helper.group(1)] if kind == "string" else kind
    return None


HELPER_CALL_RE = re.compile(r"(\w+)\s*\(\s*$")
HELPER_DEF_RE = re.compile(r"^\s*def\s+(\w+)\s*\(")
NONLITERAL_ENV_READ_RE = re.compile(r"""(?:os\.environ\.get|os\.getenv|environ\.get|\bgetenv|\benv\.get|environ\s*\[)\s*\(?\s*(?!["'])[A-Za-z_]""")
GENERIC_HELPER_NAMES = {"get", "value", "main", "run", "setup", "read", "load", "parse", "getenv", "environ", "wrapper", "inner"}
ENV_HELPERS: dict[str, str] = {}


def find_env_helpers(root: str, files: list[str]) -> dict[str, str]:
    """Python functions whose body reads the environment through a PARAMETER, with the kind
    the body parses it as -- so a call passing a literal name counts as a read of that name."""
    helpers: dict[str, str] = {}
    for rel in files:
        if not is_scanned(rel) or lang_of(rel) != "py":
            continue
        text = read_text(root, rel)
        if "environ" not in text and "getenv" not in text:
            continue
        lines, _ = strip_comments(text.splitlines(), "py")
        for i, ln in enumerate(lines):
            m = HELPER_DEF_RE.match(ln)
            if not m or m.group(1) in GENERIC_HELPER_NAMES or m.group(1).startswith("__"):
                continue
            indent = len(ln) - len(ln.lstrip())
            for j in range(i + 1, min(len(lines), i + 40)):
                body = lines[j]
                if body.strip() and len(body) - len(body.lstrip()) <= indent:
                    break  # the function ended
                if NONLITERAL_ENV_READ_RE.search(body):
                    helpers[m.group(1)] = kind_c_py("", body, lines[j + 1: j + 5], "")
                    break
    return helpers


def writes_name(name: str, window: list[str]) -> bool:
    esc = re.escape(name)
    pat = re.compile(r"""\[\s*["']%s["']\s*\]\s*=(?!=)|\b(setdefault|q?putenv|setenv)\(\s*["']%s["']""" % (esc, esc))
    return any(pat.search(ln) for ln in window)


SH_WRITE_BEFORE_RE = re.compile(r"(^|[;&|(]|\bthen\b|\bdo\b|\belse\b)\s*(export\s+|local\s+|readonly\s+|declare\s+(-\w+\s+)?|unset\s+)?$")
SH_PRESENCE_TMPL = (r"-[nzv]\s+\"?\$?\{?@N@\b", r"\$\{@N@:\+", r"-v\s+@N@\b")
SH_VALUE_TMPL = (r"\$\{?@N@(:-[^}]*)?\}?\"?\s*(==?|!=)\s*\"?\S", r"case\s+\"?\$\{?@N@\b", r"\$@N@\s+in\s+")
SH_INT_TMPL = (r"\$\{?@N@\}?\"?\s*-(eq|ne|gt|lt|ge|le)\b", r"\$\(\(\s*[^)]*\$\{?@N@\b")


def _any(tmpls: tuple, name: str, line: str) -> bool:
    n = re.escape(name)
    return any(re.search(t.replace("@N@", n), line) for t in tmpls)


def classify_shell_family(lang: str, code: str, m: re.Match, name: str, make_assigned: set[str]) -> str | None:
    before, after = code[: m.start()], code[m.end():]
    esc = re.escape(name)
    if lang == "make":
        if re.search(r"^\s*(export\s+|override\s+)?$", before) and re.match(r"\s*(:=|::=|=|\+=)", after):
            return None
        if re.search(r"^\s*(export\s+)?$", before) and re.match(r"\s*\?=", after):
            return None  # a make variable with a default: a build contract, not an OmniSim knob
        if re.search(r"\bifn?def\s+$", before):
            return "presence"
        if before.endswith(("$(", "${")) and after[:1] in (")", "}"):
            if name in make_assigned:
                return None
            if re.search(r"\bifn?eq\s*\(\s*(\$\(strip\s+)?\$[({]%s[)}]\)?\s*,\s*\)" % esc, code):
                return "presence"
            if re.search(r"\$\(if\s+\$[({]%s[)}]" % esc, code):
                return "presence"
            if re.search(r"\bifn?eq\s*\(\s*(\$\(strip\s+)?\$[({]%s[)}]\)?\s*,\s*\S" % esc, code):
                return "value"
            return "string"
        return None
    if lang == "ps1":
        if re.search(r"\$\{?env:$", before, re.I):
            if re.match(r"\}?\s*=(?!=)", after):
                return None
            if re.search(r"-not\s+\$\{?env:%s\b|!\s*\$\{?env:%s\b|if\s*\(\s*\$\{?env:%s\s*\)|IsNullOrEmpty|-(eq|ne)\s+\$null" % (esc, esc, esc), code, re.I):
                return "presence"
            if re.search(r"\[(int|double|float|long)\]\s*\$\{?env:%s\b" % esc, code, re.I):
                return "int"
            if re.search(r"\$\{?env:%s\}?\s*-(eq|ne|like|in|match|notin)\b" % esc, code, re.I):
                return "value"
            return "string"
        if re.search(r"Test-Path\s+(-Path\s+)?[\"']?env:$", before, re.I):
            return "presence"
        return None
    if lang == "bat":
        if before.endswith("%") and after.startswith("%"):
            if re.search(r"if\s+(/i\s+)?(not\s+)?\"?%%%s%%\"?\s*==\s*\"\"" % esc, code, re.I):
                return "presence"
            if re.search(r"if\s+(/i\s+)?(not\s+)?\"?%%%s%%\"?\s*(==|equ|neq|gtr|lss)\s*\"?\S" % esc, code, re.I):
                return "value"
            return "string"
        if re.search(r"\bdefined\s+$", before, re.I):
            return "presence"
        return None
    # sh / yml
    if lang == "yml" and re.match(r"^\s*$", before) and re.match(r":(\s|$)", after):
        return None  # a YAML env: entry
    if lang == "yml" and re.search(r"\benv\.$", before):
        if re.search(r"env\.%s\s*(==|!=)\s*['\"]" % esc, code):
            return "value"
        return "string"
    if SH_WRITE_BEFORE_RE.search(before) and after.startswith("="):
        return None
    if re.search(r"\bunset\s+$", before):
        return None
    if before.endswith(("$", "${", "${!")):
        if _any(SH_PRESENCE_TMPL, name, code):
            return "presence"
        if _any(SH_INT_TMPL, name, code):
            return "int"
        if _any(SH_VALUE_TMPL, name, code):
            return "value"
        return "string"
    if re.search(r"-v\s+$", before):
        return "presence"
    return None


# ------------------------------------------------------------------ descriptions

COMMENT_MARK_RE = re.compile(r"^\s*(?:///?|/\*+|\*+/?|#+>?|<#|rem\b|::|;+|\"\"\"|''')\s?", re.I)
LICENSE_RE = re.compile(r"Licensed under|Copyright|SPDX-License|-\*- coding|^#!|\bnoqa\b|\bpylint\b|\btype:\s|\bNOLINT\b|\bpragma\b", re.I)


def clean_comment_line(raw: str) -> str:
    text = COMMENT_MARK_RE.sub("", raw.strip())
    text = re.sub(r"(\*/|\"\"\"|''')\s*$", "", text).strip()
    if not re.search(r"[A-Za-z0-9]", text):
        return ""
    return text


def trailing_comment(raw: str, lang: str) -> str:
    if lang == "c":
        i = raw.find("//")
        while i > 0 and raw[i - 1] == ":":
            i = raw.find("//", i + 2)
        return raw[i + 2:].strip() if i >= 0 else ""
    if lang == "bat":
        return ""
    m = re.search(r"\s#\s?(.*)$", raw)
    return m.group(1).strip() if m else ""


def harvest_description(raw_lines: list[str], is_comment: list[bool], idx: int, lang: str, name: str) -> tuple[str, int]:
    """Return (description, quality): 2 when the comment is inline or names the variable,
    1 when it is merely the nearest block above, 0 when there is none."""
    inline = trailing_comment(raw_lines[idx], lang)
    if inline and not LICENSE_RE.search(inline) and re.search(r"[A-Za-z]{3}", inline):
        return finish_description(inline), 2

    def block_ending_at(k: int) -> list[str]:
        start = k
        while start - 1 >= 0 and is_comment[start - 1]:
            start -= 1
        return [clean_comment_line(raw_lines[j]) for j in range(start, k + 1)]

    def block_text(block: list[str]) -> str:
        text = " ".join(t for t in block if t)
        if not text or LICENSE_RE.search(text):
            return ""
        return text

    # 1. A comment mentioning the variable within 15 lines above.
    for k in range(idx - 1, max(-1, idx - 16), -1):
        if is_comment[k] and name in raw_lines[k]:
            end = k
            while end + 1 < idx and is_comment[end + 1]:
                end += 1
            text = block_text(block_ending_at(end))
            if text:
                return finish_description(text), 2
            break
    # 2. The nearest comment block within 8 lines above (at most 3 code lines between).
    code_lines = 0
    for k in range(idx - 1, max(-1, idx - 9), -1):
        if is_comment[k] == 1:
            text = block_text(block_ending_at(k))
            return (finish_description(text), 1) if text else ("", 0)
        if raw_lines[k].strip():
            code_lines += 1
            if code_lines > 3:
                break
    return "", 0


def finish_description(text: str) -> str:
    text = re.sub(r"\s+", " ", text).strip()
    text = text.replace("|", "\\|").replace("<", "&lt;").replace(">", "&gt;")
    if text.count("`") % 2:
        text = text.replace("`", "")
    if len(text) > MAX_DESC:
        cut = text[: MAX_DESC - 3]
        if " " in cut:
            cut = cut[: cut.rfind(" ")]
        text = cut.rstrip(" ,;:") + "..."
    return text


# ------------------------------------------------------------------ scan

class Site:
    __slots__ = ("path", "line", "kind", "description", "quality")

    def __init__(self, path: str, line: int, kind: str, description: str, quality: int):
        self.path, self.line, self.kind, self.description, self.quality = path, line, kind, description, quality


def scan_file(root: str, rel: str) -> list[tuple[str, Site]]:
    text = read_text(root, rel)
    if "OMNISIM_" not in text:
        return []
    lang = lang_of(rel)
    raw_lines = text.splitlines()
    code, is_comment = strip_comments(raw_lines, lang)
    make_assigned: set[str] = set()
    if lang == "make":
        for ln in code:
            m = re.match(r"^\s*(?:export\s+|override\s+)?(OMNISIM_[A-Z0-9_]+)\s*(?::=|::=|=|\+=|\?=)", ln)
            if m:
                make_assigned.add(m.group(1))
    found: list[tuple[str, Site]] = []
    for i, line in enumerate(code):
        if "OMNISIM_" not in line:
            continue
        window = code[i + 1: i + 5]
        for m in VAR_RE.finditer(line):
            name = m.group(0)
            if lang in ("c", "py"):
                kind = classify_c_py(line, m, window, code[i - 1] if i else "")
            else:
                kind = classify_shell_family(lang, line, m, name, make_assigned)
            if kind is None:
                continue
            desc, quality = harvest_description(raw_lines, is_comment, i, lang, name)
            found.append((name, Site(rel, i + 1, kind, desc, quality)))
    return found


AREA_RANK = {key: i for i, (key, _t, _p) in enumerate(AREAS)}


def site_order(s: "Site") -> tuple:
    """Engine sites first, then the other layers in AREAS order; path and line within a layer."""
    return (AREA_RANK[area_of(s.path)], s.path, s.line)


def documented_names(root: str, files: list[str]) -> set[str]:
    names: set[str] = set()
    for rel in files:
        if rel == OUTPUT_REL:
            continue
        if rel in DOC_ROOTS or (rel.startswith("docs/") and rel.endswith(".md")):
            names.update(VAR_RE.findall(read_text(root, rel)))
    return names


def scan(root: str = REPO_ROOT) -> dict:
    files = tracked_files(root)
    ENV_HELPERS.clear()
    ENV_HELPERS.update(find_env_helpers(root, files))
    sites: dict[str, list[Site]] = defaultdict(list)
    for rel in files:
        if not is_scanned(rel) or not os.path.isfile(os.path.join(root, rel)):
            continue
        for name, site in scan_file(root, rel):
            sites[name].append(site)
    documented = documented_names(root, files)
    variables = {}
    for name in sorted(sites):
        ss = sorted(sites[name], key=site_order)
        for s in ss:
            if s.kind != "presence":
                continue
            # `if "X" in os.environ: n = int(os.environ["X"])` -- a presence test within
            # three lines of a read that uses the value is a guard, not a gate.
            near = [o for o in ss if o is not s and o.path == s.path and abs(o.line - s.line) <= 3 and o.kind != "presence"]
            if near:
                s.kind = near[0].kind
        # One site per line: a line reading the name twice (`IsSet(X) ? parse(X) : ...`) is one read.
        by_line: dict[tuple[str, int], Site] = {}
        for s in ss:
            key = (s.path, s.line)
            if key not in by_line or KIND_ORDER.index(s.kind) < KIND_ORDER.index(by_line[key].kind):
                by_line[key] = s
        ss = sorted(by_line.values(), key=site_order)
        # The kind is decided by the sites in the area the variable is filed under; a
        # test asserting on an env dict, or a shell pass-through, must not outvote the engine.
        area = next(k for k, _t, _p in AREAS if any(area_of(s.path) == k for s in ss))
        voters = [s for s in ss if area_of(s.path) == area]
        kinds = Counter(s.kind for s in voters)
        if "presence" in kinds and len(kinds) > 1:
            kind = "mixed (%s)" % "+".join(k for k in KIND_ORDER if k in kinds)
        elif "value" in kinds:
            kind = "value"
        elif "int" in kinds:
            kind = "int"
        elif "presence" in kinds:
            kind = "presence"
        else:
            kind = "string"
        # Best description: a comment that names the variable beats the merely-nearest one,
        # the filed layer's sites beat the others, and earlier sites win ties.
        ranked = sorted(enumerate(ss), key=lambda iv: (-(iv[1].quality), 0 if iv[1] in voters else 1, iv[0]))
        desc = next((s.description for _i, s in ranked if s.description), "")
        variables[name] = {
            "sites": ss, "kinds": kinds, "kind": kind, "description": desc,
            "area": area, "documented": name in documented,
        }
    return variables


# ------------------------------------------------------------------ render

def render(variables: dict) -> str:
    out = io.StringIO()
    w = out.write
    total = len(variables)
    presence = sum(1 for v in variables.values() if v["kind"] == "presence")
    mixed = sum(1 for v in variables.values() if v["kind"].startswith("mixed"))
    undocumented = sorted(n for n, v in variables.items() if not v["documented"])
    w("# Environment Variables\n\n")
    w("<!-- GENERATED FILE: do not edit by hand. Regenerate with the command below. -->\n\n")
    w("**This page is GENERATED** from the tracked source tree by\n")
    w("`python scripts/dev/gen_env_reference.py`. Regenerate it after adding, removing or\n")
    w("re-parsing any `OMNISIM_*` read; `docs/tests/test_env_reference.py` fails whenever the\n")
    w("committed page differs from what the generator produces, so the page cannot drift from\n")
    w("the code. Read sites are `file:line` in the tracked tree (`git ls-files`, source\n")
    w("extensions only; `social/`, `cloud/`, `_scratch/`, `distribution/` and benchmark result\n")
    w("directories excluded); a variable is filed under the lowest layer that reads it, so one\n")
    w("read anywhere in the engine files it under **Engine** even when scripts read it too, and\n")
    w("its **Kind** is decided by the read sites in that layer (a test asserting on an env dict\n")
    w("or a shell pass-through is listed, but does not outvote the engine).\n\n")
    w("**Summary:** %d variables, %d presence-gated, %d mixed, %d documented nowhere before this page.\n\n"
      % (total, presence, mixed, len(undocumented)))
    w("**Kind** is a heuristic read of the code around each read site:\n\n")
    w("- `presence` -- only the existence of the variable is tested (`qEnvironmentVariableIsSet`,\n")
    w("  `if getenv(...)`, `\"X\" in os.environ`, `[ -n \"$X\" ]`). **`=0` ARMS it** -- the\n")
    w("  `OMNISIM_REQUIRE_NEWTON` trap; UNSET it to disarm.\n")
    w("- `value` -- the string is compared or lowered (`== \"0\"`, `.trimmed().toLower()`,\n")
    w("  `.lower() in (...)`); by convention `0` / `false` / `off` mean OFF and anything else ON.\n")
    w("- `int` -- parsed as a number.\n")
    w("- `string` -- used as a path, name, URL or opaque value.\n")
    w("- `mixed (...)` -- presence at some read sites and value/int/string at others: the\n")
    w("  inconsistent class, where `=0` arms one site and disarms another.\n\n")
    w("**Documented** means the name appears in `AGENTS.md`, `README.md`, `PROTOCOL.md`,\n")
    w("`CHANGELOG.md` or any `docs/**/*.md` other than this page; a variable named nowhere else\n")
    w("is marked *(undocumented elsewhere)*. **Description** is the nearest source comment above\n")
    w("(or trailing) the first read site, clipped to %d characters; an empty cell means the code\n" % MAX_DESC)
    w("carries no comment there.\n\n")
    for key, title, _prefixes in AREAS:
        rows = [(n, v) for n, v in variables.items() if v["area"] == key]
        if not rows:
            continue
        w("## %s\n\n" % title)
        w("%d variable%s.\n\n" % (len(rows), "" if len(rows) == 1 else "s"))
        w("| Variable | Kind | Read at | Description |\n")
        w("|---|---|---|---|\n")
        for name, v in rows:
            sites = v["sites"]
            shown = ", ".join("`%s:%d`" % (s.path, s.line) for s in sites[:MAX_SITES])
            if len(sites) > MAX_SITES:
                shown += " (+%d more)" % (len(sites) - MAX_SITES)
            flag = "" if v["documented"] else " *(undocumented elsewhere)*"
            w("| `%s`%s | %s | %s | %s |\n" % (name, flag, v["kind"], shown, v["description"]))
        w("\n")
    w("## Not documented anywhere else\n\n")
    if undocumented:
        w("%d variables whose only documentation is this page:\n\n" % len(undocumented))
        w(", ".join("`%s`" % n for n in undocumented) + "\n")
    else:
        w("None: every variable above is also named in a hand-written document.\n")
    return out.getvalue()


def stats(variables: dict) -> str:
    lines = []
    total = len(variables)
    by_kind = Counter(v["kind"].split(" ")[0] for v in variables.values())
    undocumented = [n for n, v in variables.items() if not v["documented"]]
    no_desc = [n for n, v in variables.items() if not v["description"]]
    lines.append("variables: %d" % total)
    lines.append("by kind: %s" % ", ".join("%s %d" % (k, by_kind[k]) for k in ("presence", "value", "int", "string", "mixed") if by_kind[k]))
    lines.append("undocumented elsewhere: %d" % len(undocumented))
    lines.append("without a harvested description: %d" % len(no_desc))
    lines.append("read sites: %d" % sum(len(v["sites"]) for v in variables.values()))
    lines.append("by area: %s" % ", ".join("%s %d" % (k, sum(1 for v in variables.values() if v["area"] == k)) for k, _t, _p in AREAS))
    audit_dirs = ("src/omnisim/", "src/controller/", "lib/controller/python/", "omnisim/", "scripts/", "projects/policies/", "packages/")
    in_audit = {n: v for n, v in variables.items() if any(s.path.startswith(audit_dirs) for s in v["sites"])}
    lines.append("2026-09-02 audit scope (%s): %d variables, %d presence-gated, %d undocumented elsewhere"
                 % (", ".join(audit_dirs), len(in_audit),
                    sum(1 for v in in_audit.values() if v["kind"] == "presence"),
                    sum(1 for v in in_audit.values() if not v["documented"])))
    lines.append("top readers:")
    for name, v in sorted(variables.items(), key=lambda kv: (-len(kv[1]["sites"]), kv[0]))[:10]:
        lines.append("  %-40s %3d sites  %s" % (name, len(v["sites"]), v["kind"]))
    mixed = [(n, v) for n, v in variables.items() if v["kind"].startswith("mixed")]
    lines.append("mixed (presence at some sites, parsed at others): %d" % len(mixed))
    for name, v in mixed:
        lines.append("  %-40s %s" % (name, dict(v["kinds"])))
    return "\n".join(lines)


def generate(root: str = REPO_ROOT) -> str:
    return render(scan(root))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, OUTPUT_REL), help="output path (default: the reference page)")
    ap.add_argument("--check", action="store_true", help="exit 1 if the committed page differs from a fresh generation")
    ap.add_argument("--stats", action="store_true", help="print totals, top readers and inconsistent reads to stdout")
    args = ap.parse_args(argv)
    variables = scan(REPO_ROOT)
    text = render(variables)
    if args.stats:
        print(stats(variables))
    if args.check:
        target = os.path.join(REPO_ROOT, OUTPUT_REL)
        current = read_text(REPO_ROOT, OUTPUT_REL).replace("\r\n", "\n") if os.path.isfile(target) else None
        if current != text:
            print("%s is stale; run: python %s" % (OUTPUT_REL, SELF_REL), file=sys.stderr)
            return 1
        print("%s is up to date." % OUTPUT_REL)
        return 0
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    if not args.stats:
        print("wrote %s (%d variables)" % (os.path.relpath(args.out, REPO_ROOT), len(variables)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
