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

"""Shared curated-knowledge search for production agents.

The single implementation behind every agent's ``tools/knowledge.py`` shim.
Scans a knowledge folder, chunks each file on heading or blank-line
boundaries, scores chunks by keyword hits (plus a proximity bonus when
multiple terms land close together), and returns the top N matches with file,
line range, and text. The only per-agent difference was the folder path, so
``search_knowledge`` takes ``kb_root`` as a parameter; each shim resolves its
own folder from ``__file__`` and passes it in.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

_TEXT_EXTS = {".md", ".markdown", ".txt", ".rst", ".csv"}
_JSON_EXTS = {".json"}
_PDF_EXTS = {".pdf"}


def _extract_pdf_text(path: Path) -> Optional[str]:
    try:
        from pypdf import PdfReader  # type: ignore
    except Exception:
        return None
    try:
        reader = PdfReader(str(path))
        parts: List[str] = []
        for page in reader.pages:
            try:
                parts.append(page.extract_text() or "")
            except Exception:
                continue
        return "\n\n".join(parts)
    except Exception:
        return None


def _load_file(path: Path) -> Optional[str]:
    ext = path.suffix.lower()
    try:
        if ext in _TEXT_EXTS:
            return path.read_text(encoding="utf-8", errors="replace")
        if ext in _JSON_EXTS:
            raw = path.read_text(encoding="utf-8", errors="replace")
            try:
                return json.dumps(json.loads(raw), indent=2, ensure_ascii=False)
            except Exception:
                return raw
        if ext in _PDF_EXTS:
            return _extract_pdf_text(path)
    except Exception:
        return None
    return None


def _chunk_text(text: str, file_label: str) -> List[Dict[str, Any]]:
    chunks: List[Dict[str, Any]] = []
    if not text:
        return chunks

    lines = text.splitlines()
    has_headings = any(re.match(r"^#{1,6}\s+\S", ln) for ln in lines)

    if has_headings:
        current_heading = ""
        buf: List[str] = []
        buf_start_line = 1
        for idx, ln in enumerate(lines, 1):
            if re.match(r"^#{1,6}\s+\S", ln):
                if buf:
                    chunks.append({
                        "file": file_label,
                        "heading": current_heading or "(preface)",
                        "start_line": buf_start_line,
                        "end_line": idx - 1,
                        "text": "\n".join(buf).strip(),
                    })
                current_heading = ln.lstrip("#").strip()
                buf = []
                buf_start_line = idx + 1
            else:
                buf.append(ln)
        if buf:
            chunks.append({
                "file": file_label,
                "heading": current_heading or "(preface)",
                "start_line": buf_start_line,
                "end_line": len(lines),
                "text": "\n".join(buf).strip(),
            })
    else:
        paragraphs = re.split(r"\n\s*\n", text)
        line_cursor = 1
        for para_idx, para in enumerate(paragraphs):
            para_lines = para.splitlines()
            if any(s.strip() for s in para_lines):
                chunks.append({
                    "file": file_label,
                    "heading": f"paragraph {para_idx + 1}",
                    "start_line": line_cursor,
                    "end_line": line_cursor + len(para_lines) - 1,
                    "text": para.strip(),
                })
            line_cursor += len(para_lines) + 1

    return [c for c in chunks if c["text"]]


def _score_chunk(chunk_text_lower: str, terms: List[str]) -> int:
    if not terms:
        return 1
    score = 0
    for t in terms:
        if not t:
            continue
        hits = chunk_text_lower.count(t)
        if hits:
            score += hits * 3
    if len(terms) > 1:
        positions: Dict[str, List[int]] = {}
        for t in terms:
            positions[t] = [m.start() for m in re.finditer(re.escape(t), chunk_text_lower)]
        close_pairs = 0
        for i, ti in enumerate(terms):
            for tj in terms[i + 1:]:
                for pi in positions.get(ti, []):
                    for pj in positions.get(tj, []):
                        if abs(pi - pj) <= 80:
                            close_pairs += 1
                            break
                    else:
                        continue
                    break
        score += close_pairs * 4
    return score


def search_knowledge(
    kb_root: Path,
    query: str = "",
    max_results: int = 8,
    max_snippet_chars: int = 800,
    **_: Any,
) -> Dict[str, Any]:
    kb_root = Path(kb_root)
    if not kb_root.exists():
        return {
            "status": "ok",
            "hits": [],
            "note": f"knowledge folder not found at {kb_root}",
        }

    terms = [t for t in re.split(r"\s+", (query or "").strip().lower()) if len(t) >= 2]
    all_chunks: List[Dict[str, Any]] = []
    files_scanned = 0
    skipped_by_type: List[str] = []

    for path in sorted(kb_root.rglob("*")):
        if not path.is_file():
            continue
        if path.name.lower() == "readme.md":
            continue
        ext = path.suffix.lower()
        if ext not in _TEXT_EXTS | _JSON_EXTS | _PDF_EXTS:
            skipped_by_type.append(path.name)
            continue
        text = _load_file(path)
        if text is None:
            if ext in _PDF_EXTS:
                skipped_by_type.append(f"{path.name} (pypdf not installed)")
            else:
                skipped_by_type.append(path.name)
            continue
        files_scanned += 1
        try:
            rel = str(path.relative_to(kb_root)).replace("\\", "/")
        except ValueError:
            rel = path.name
        all_chunks.extend(_chunk_text(text, rel))

    if not terms:
        sample = all_chunks[: max(1, int(max_results))]
        return {
            "status": "ok",
            "query": query,
            "files_scanned": files_scanned,
            "total_chunks": len(all_chunks),
            "skipped": skipped_by_type[:10],
            "hits": [
                {
                    "file": c["file"],
                    "heading": c["heading"],
                    "start_line": c["start_line"],
                    "end_line": c["end_line"],
                    "text": c["text"][:max_snippet_chars],
                    "score": 1,
                }
                for c in sample
            ],
            "note": "Empty query — returned a sample. Pass `query` for scored results.",
        }

    scored: List[Dict[str, Any]] = []
    for c in all_chunks:
        haystack = f"{c['file']}\n{c['heading']}\n{c['text']}".lower()
        s = _score_chunk(haystack, terms)
        if s <= 0:
            continue
        heading_lower = f"{c['file']} {c['heading']}".lower()
        heading_hits = sum(heading_lower.count(t) for t in terms if t)
        s += heading_hits * 5
        scored.append({**c, "score": s})

    scored.sort(key=lambda x: (-x["score"], x["file"], x["start_line"]))
    top = scored[: max(1, int(max_results))]

    return {
        "status": "ok",
        "query": query,
        "files_scanned": files_scanned,
        "total_chunks": len(all_chunks),
        "hits": [
            {
                "file": c["file"],
                "heading": c["heading"],
                "start_line": c["start_line"],
                "end_line": c["end_line"],
                "text": c["text"][:max_snippet_chars],
                "score": c["score"],
            }
            for c in top
        ],
        "truncated": len(scored) > len(top),
        "skipped": skipped_by_type[:10] if skipped_by_type else None,
    }
