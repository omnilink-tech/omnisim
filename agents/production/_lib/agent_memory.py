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

"""Shared local long-term memory store for production agents.

This is the single implementation behind every agent's
``tools/local_memory.py`` shim. Same hybrid retrieval (vector cosine via
Ollama + BM25 lexical, fused via RRF) and same storage shape (markdown file
per memory + SQLite index) that every agent used to carry as its own ~625-line
copy. The only thing that differs per agent is the storage root, so this is a
class parameterized by ``memory_root``; each agent's shim instantiates one
``LocalMemoryStore`` pointed at its ``long_term_memory/`` folder and exposes the
tool impls + ``search_local_memory_for_recall`` from the instance.

Layout (per agent, unchanged):

    long_term_memory/
      2026-04-26-some-memory.md
      _index.sqlite                 # (id, title, body, tags, mtime, embedding)
"""

from __future__ import annotations

import json
import math
import re
import sqlite3
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_MAX_TAGS = 8
_DEFAULT_TOP_K = 5

# ── Embedding provider ────────────────────────────────────────────────

_OLLAMA_URL = "http://127.0.0.1:11434/api/embeddings"
_OLLAMA_MODEL = "nomic-embed-text"


def _try_embed_ollama(text: str, timeout: float = 10.0) -> Optional[List[float]]:
    body = json.dumps({"model": _OLLAMA_MODEL, "prompt": text}).encode()
    req = urllib.request.Request(_OLLAMA_URL, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            payload = json.loads(r.read().decode("utf-8", errors="replace") or "{}")
    except (urllib.error.URLError, TimeoutError, OSError):
        return None
    except Exception:
        return None
    vec = payload.get("embedding")
    if isinstance(vec, list) and vec and all(isinstance(x, (int, float)) for x in vec):
        return [float(x) for x in vec]
    return None


def _cosine(a: List[float], b: List[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    dot = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(y * y for y in b))
    if na == 0 or nb == 0:
        return 0.0
    return dot / (na * nb)


# ── BM25 + RRF ────────────────────────────────────────────────────────

_TOKEN_RE = re.compile(r"[a-z0-9]+")
_BM25_K1 = 1.2
_BM25_B = 0.75
_RRF_K = 60
_RRF_VECTOR_WEIGHT = 2.0
_RRF_BM25_WEIGHT = 1.0
_BM25_CONTRIB_MIN_TOP_SCORE = 2.5


def _tokens(s: str) -> List[str]:
    return _TOKEN_RE.findall(s.lower())


def _bm25_corpus_stats(docs: List[List[str]]) -> Dict[str, Any]:
    df: Dict[str, int] = {}
    for doc in docs:
        for term in set(doc):
            df[term] = df.get(term, 0) + 1
    total_len = sum(len(d) for d in docs)
    avgdl = total_len / max(1, len(docs))
    return {"df": df, "avgdl": avgdl, "n": len(docs)}


def _bm25_score(q_terms: List[str], doc: List[str], stats: Dict[str, Any]) -> float:
    if not q_terms or not doc:
        return 0.0
    df: Dict[str, int] = stats["df"]
    n: int = stats["n"]
    avgdl: float = stats["avgdl"] or 1.0
    doc_len = len(doc)
    doc_tf: Dict[str, int] = {}
    for t in doc:
        doc_tf[t] = doc_tf.get(t, 0) + 1
    score = 0.0
    for t in q_terms:
        tf = doc_tf.get(t, 0)
        if tf == 0:
            continue
        idf = math.log((n - df.get(t, 0) + 0.5) / (df.get(t, 0) + 0.5) + 1.0)
        norm = 1.0 - _BM25_B + _BM25_B * (doc_len / avgdl)
        score += idf * (tf * (_BM25_K1 + 1)) / (tf + _BM25_K1 * norm)
    return score


def _rrf_fuse(
    ranked_lists: List[Tuple[List[str], float]],
    k: int = _RRF_K,
) -> Dict[str, float]:
    scores: Dict[str, float] = {}
    for lst, weight in ranked_lists:
        for rank, doc_id in enumerate(lst, start=1):
            scores[doc_id] = scores.get(doc_id, 0.0) + weight / (k + rank)
    return scores


# ── File I/O ──────────────────────────────────────────────────────────

def _slugify(text: str, max_len: int = 48) -> str:
    s = re.sub(r"[^a-zA-Z0-9]+", "-", text or "").strip("-").lower()
    return (s[:max_len] or "memory").strip("-") or "memory"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _parse_frontmatter(raw: str) -> Tuple[Dict[str, Any], str]:
    if not raw.startswith("---\n"):
        return {}, raw
    end = raw.find("\n---\n", 4)
    if end == -1:
        return {}, raw
    head = raw[4:end]
    body = raw[end + 5:]
    meta: Dict[str, Any] = {}
    for line in head.splitlines():
        line = line.rstrip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        k = k.strip()
        v = v.strip()
        if v.startswith("[") and v.endswith("]"):
            inner = v[1:-1]
            meta[k] = [x.strip().strip('"').strip("'") for x in inner.split(",") if x.strip()]
        else:
            meta[k] = v.strip('"').strip("'")
    return meta, body.lstrip("\n")


def _render_frontmatter(meta: Dict[str, Any]) -> str:
    lines = ["---"]
    for k in ("id", "title", "tags", "created_at", "updated_at"):
        if k not in meta:
            continue
        v = meta[k]
        if isinstance(v, list):
            lines.append(f"{k}: [{', '.join(str(x) for x in v)}]")
        else:
            lines.append(f"{k}: {v}")
    lines.append("---")
    lines.append("")
    return "\n".join(lines)


def _write_file(path: Path, meta: Dict[str, Any], body: str) -> None:
    content = _render_frontmatter(meta) + body.strip() + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(content, encoding="utf-8")
    tmp.replace(path)


# ── SQLite ────────────────────────────────────────────────────────────

_SCHEMA = """
CREATE TABLE IF NOT EXISTS memories (
    id          TEXT PRIMARY KEY,
    title       TEXT NOT NULL,
    body        TEXT NOT NULL,
    tags        TEXT,
    path        TEXT NOT NULL,
    created_at  TEXT NOT NULL,
    updated_at  TEXT NOT NULL,
    embedding   BLOB,
    embed_dim   INTEGER
);
CREATE INDEX IF NOT EXISTS memories_updated_at_idx ON memories(updated_at DESC);
"""


def _vec_to_blob(vec: Optional[List[float]]) -> Optional[bytes]:
    if not vec:
        return None
    return (",".join(f"{x:.6f}" for x in vec)).encode("ascii")


def _blob_to_vec(blob: Optional[bytes]) -> Optional[List[float]]:
    if not blob:
        return None
    try:
        return [float(x) for x in blob.decode("ascii").split(",") if x]
    except Exception:
        return None


# ── Store ─────────────────────────────────────────────────────────────

class LocalMemoryStore:
    """Per-agent local long-term memory backed by ``memory_root``.

    One instance per agent; the agent's ``tools/local_memory.py`` shim
    binds the tool impls to this instance's methods so the public tool
    surface (save/search/list/forget + ``search_local_memory_for_recall``)
    is identical to the old per-agent module.
    """

    def __init__(self, memory_root: Path) -> None:
        self.memory_root = Path(memory_root)
        self.index_path = self.memory_root / "_index.sqlite"
        self._embed_dim: Optional[int] = None

    # ── Embedding ──────────────────────────────────────────────────
    def _embed(self, text: str) -> Optional[List[float]]:
        if not text.strip():
            return None
        vec = _try_embed_ollama(text)
        if vec is None:
            return None
        if self._embed_dim is None:
            self._embed_dim = len(vec)
        return vec

    # ── Storage ────────────────────────────────────────────────────
    def _ensure_dirs(self) -> None:
        self.memory_root.mkdir(parents=True, exist_ok=True)

    def _conn(self) -> sqlite3.Connection:
        self._ensure_dirs()
        c = sqlite3.connect(str(self.index_path))
        c.executescript(_SCHEMA)
        return c

    # ── Tool impls ─────────────────────────────────────────────────
    def save(
        self,
        title: str = "",
        body: str = "",
        tags: Any = None,
        **_: Any,
    ) -> Dict[str, Any]:
        title = (title or "").strip()
        body = (body or "").strip()
        if not title:
            return {"error": "title is required"}
        if not body:
            return {"error": "body is required"}

        tag_list: List[str] = []
        if isinstance(tags, list):
            tag_list = [str(t).strip() for t in tags if str(t).strip()][:_MAX_TAGS]
        elif isinstance(tags, str):
            tag_list = [t.strip() for t in tags.split(",") if t.strip()][:_MAX_TAGS]

        mem_id = f"mem_{int(time.time() * 1000)}_{uuid.uuid4().hex[:6]}"
        now = _now_iso()
        date_slug = datetime.now(timezone.utc).strftime("%Y-%m-%d")
        filename = f"{date_slug}-{_slugify(title)}.md"
        path = self.memory_root / filename

        meta = {
            "id": mem_id,
            "title": title,
            "tags": tag_list,
            "created_at": now,
            "updated_at": now,
        }
        _write_file(path, meta, body)

        vec = self._embed(f"{title}\n\n{body}")
        vec_blob = _vec_to_blob(vec)

        with self._conn() as c:
            c.execute(
                "INSERT INTO memories(id,title,body,tags,path,created_at,updated_at,embedding,embed_dim) "
                "VALUES (?,?,?,?,?,?,?,?,?)",
                (
                    mem_id, title, body, json.dumps(tag_list), str(path),
                    now, now, vec_blob, len(vec) if vec else None,
                ),
            )

        return {
            "status": "ok",
            "id": mem_id,
            "path": str(path.relative_to(self.memory_root.parent)),
            "embedded": vec is not None,
            "embedding_provider": "ollama" if vec is not None else None,
        }

    def search(
        self,
        query: str = "",
        k: int = _DEFAULT_TOP_K,
        tags: Any = None,
        mode: str = "hybrid",
        **_: Any,
    ) -> Dict[str, Any]:
        query = (query or "").strip()
        if not query:
            return {"error": "query is required"}

        try:
            k_int = max(1, int(k))
        except (TypeError, ValueError):
            k_int = _DEFAULT_TOP_K

        filter_tags: List[str] = []
        if isinstance(tags, list):
            filter_tags = [str(t).strip().lower() for t in tags if str(t).strip()]
        elif isinstance(tags, str):
            filter_tags = [t.strip().lower() for t in tags.split(",") if t.strip()]

        mode = (mode or "hybrid").lower()
        if mode not in ("hybrid", "vector", "bm25"):
            mode = "hybrid"

        q_vec = self._embed(query)
        q_terms = _tokens(query)

        with self._conn() as c:
            rows = c.execute(
                "SELECT id,title,body,tags,path,created_at,updated_at,embedding FROM memories"
            ).fetchall()

        items: List[Dict[str, Any]] = []
        for rid, title, body, tags_json, path, created_at, updated_at, embed_blob in rows:
            row_tags = json.loads(tags_json or "[]") if isinstance(tags_json, str) else []
            if filter_tags and not any(t.lower() in [x.lower() for x in row_tags] for t in filter_tags):
                continue
            doc_tokens = _tokens(f"{title} {body}")
            items.append({
                "id": rid, "title": title, "body": body, "tags": row_tags,
                "path": path, "updated_at": updated_at,
                "embedding": _blob_to_vec(embed_blob),
                "doc_tokens": doc_tokens,
            })

        if not items:
            return {
                "status": "ok", "query": query, "count": 0, "hits": [],
                "embedding_provider": "ollama" if q_vec is not None else None,
                "total_indexed": len(rows), "mode": mode,
            }

        vector_scored: List[Tuple[str, float]] = []
        if q_vec is not None and mode in ("hybrid", "vector"):
            for it in items:
                if it["embedding"] is not None:
                    vector_scored.append((it["id"], _cosine(q_vec, it["embedding"])))
            vector_scored.sort(key=lambda x: x[1], reverse=True)

        bm25_scored: List[Tuple[str, float]] = []
        if q_terms and mode in ("hybrid", "bm25"):
            stats = _bm25_corpus_stats([it["doc_tokens"] for it in items])
            for it in items:
                s = _bm25_score(q_terms, it["doc_tokens"], stats)
                if s > 0:
                    bm25_scored.append((it["id"], s))
            bm25_scored.sort(key=lambda x: x[1], reverse=True)

        overfetch = max(k_int * 4, 20)
        by_id = {it["id"]: it for it in items}

        if mode == "vector":
            hits_scored = [(rid, s) for rid, s in vector_scored[:overfetch]]
            used_signals = ["vector"]
        elif mode == "bm25":
            hits_scored = [(rid, s) for rid, s in bm25_scored[:overfetch]]
            used_signals = ["bm25"]
        else:
            ranked_lists: List[Tuple[List[str], float]] = []
            bm25_top_score = bm25_scored[0][1] if bm25_scored else 0.0
            bm25_contributing = bm25_top_score >= _BM25_CONTRIB_MIN_TOP_SCORE
            used_signals: List[str] = []

            if vector_scored:
                ranked_lists.append(
                    ([rid for rid, _ in vector_scored[:overfetch]], _RRF_VECTOR_WEIGHT)
                )
                used_signals.append("vector")
            if bm25_scored and bm25_contributing:
                ranked_lists.append(
                    ([rid for rid, _ in bm25_scored[:overfetch]], _RRF_BM25_WEIGHT)
                )
                used_signals.append("bm25")
            elif bm25_scored:
                used_signals.append("bm25-suppressed")

            if not ranked_lists:
                if bm25_scored:
                    ranked_lists.append(([rid for rid, _ in bm25_scored[:overfetch]], 1.0))
                    used_signals = ["bm25-fallback"]
                else:
                    return {
                        "status": "ok", "query": query, "count": 0, "hits": [],
                        "embedding_provider": None, "total_indexed": len(rows),
                        "mode": mode, "signals_used": [],
                    }

            fused = _rrf_fuse(ranked_lists, k=_RRF_K)
            hits_scored = sorted(fused.items(), key=lambda x: x[1], reverse=True)

        out_hits: List[Dict[str, Any]] = []
        for rid, score in hits_scored[:k_int]:
            it = by_id.get(rid)
            if not it:
                continue
            out_hits.append({
                "id": it["id"],
                "title": it["title"],
                "snippet": it["body"] if len(it["body"]) <= 400 else it["body"][:400] + "…",
                "tags": it["tags"],
                "path": it["path"],
                "updated_at": it["updated_at"],
                "score": round(score, 4),
                "method": "+".join(used_signals) if len(used_signals) > 1 else (used_signals[0] if used_signals else "none"),
            })

        return {
            "status": "ok",
            "query": query,
            "count": len(out_hits),
            "hits": out_hits,
            "embedding_provider": "ollama" if q_vec is not None else None,
            "total_indexed": len(rows),
            "mode": mode,
            "signals_used": used_signals,
        }

    def list_memories(self, tags: Any = None, limit: int = 50, **_: Any) -> Dict[str, Any]:
        filter_tags: List[str] = []
        if isinstance(tags, list):
            filter_tags = [str(t).strip().lower() for t in tags if str(t).strip()]
        elif isinstance(tags, str):
            filter_tags = [t.strip().lower() for t in tags.split(",") if t.strip()]

        try:
            lim = max(1, int(limit))
        except (TypeError, ValueError):
            lim = 50

        with self._conn() as c:
            rows = c.execute(
                "SELECT id,title,tags,created_at,updated_at,path FROM memories ORDER BY updated_at DESC LIMIT ?",
                (lim,),
            ).fetchall()
        out: List[Dict[str, Any]] = []
        for rid, title, tags_json, created_at, updated_at, path in rows:
            row_tags = json.loads(tags_json or "[]") if isinstance(tags_json, str) else []
            if filter_tags and not any(t.lower() in [x.lower() for x in row_tags] for t in filter_tags):
                continue
            out.append({
                "id": rid, "title": title, "tags": row_tags,
                "created_at": created_at, "updated_at": updated_at, "path": path,
            })
        return {"status": "ok", "count": len(out), "memories": out}

    def forget(self, id: str = "", **_: Any) -> Dict[str, Any]:
        mid = (id or "").strip()
        if not mid:
            return {"error": "id is required"}
        with self._conn() as c:
            row = c.execute("SELECT path FROM memories WHERE id = ?", (mid,)).fetchone()
            if not row:
                return {"error": f"no memory with id {mid}"}
            (path,) = row
            try:
                Path(path).unlink()
            except FileNotFoundError:
                pass
            c.execute("DELETE FROM memories WHERE id = ?", (mid,))
        return {"status": "ok", "id": mid, "removed_file": path}

    # ── For recall.py ──────────────────────────────────────────────
    def search_for_recall(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        result = self.search(query=query, k=limit)
        if result.get("status") != "ok":
            return []
        out: List[Dict[str, Any]] = []
        for h in result.get("hits", []) or []:
            out.append({
                "source": f"local_memory:{h.get('title', '')}",
                "id": h.get("id"),
                "content": h.get("snippet", ""),
                "tags": h.get("tags", []),
                "score": h.get("score", 0),
                "updated_at": h.get("updated_at"),
                "method": h.get("method"),
            })
        return out
