# Axis's long-term memory

Agent-writable markdown notes indexed with embeddings. The store for facts Axis wants to carry across sessions — robot calibrations, operator-authored safety thresholds, recurring failure signatures, reusable trajectory recipes.

## Layout

```
long_term_memory/
  README.md                          — this file (ignored by the index)
  2026-04-21-omniarm6_01-calibration.md
  2026-04-21-ik-max-dq-override.md
  _index.sqlite                      — SQLite index; one row per memory,
                                       holds the embedding blob
```

Each markdown file has minimal YAML-ish frontmatter:

```markdown
---
id: mem_1776679123456_abc123
title: omniarm6_01 calibration offset
tags: [robot, omniarm6_01, calibration]
created_at: 2026-04-21T12:34:56Z
updated_at: 2026-04-21T12:34:56Z
---

Full note body in markdown.
```

## Tools

| Tool | Purpose |
|---|---|
| `save_local_memory(title, body, tags?)` | Write a new memory |
| `search_local_memory(query, k=5, tags?)` | Hybrid vector + BM25 search |
| `list_local_memories(tags?, limit=50)` | Browse metadata |
| `forget_local_memory(id)` | Delete a memory (file + index row) |

`recall(query)` also pulls from this store as its `long_term` tier alongside the knowledge folder.

## What belongs here (for Axis)

- **Robot calibrations** — "omniarm6_01 home pose measured 0.02 rad off on joint2"
- **Operator-authored thresholds** — "cap `IK_MAX_DQ` at 0.05 for omniarm6_03 pending payload review"
- **Recurring failure signatures** — "target (0.6, 0.3, 0.1) consistently nears elbow singularity — route via waypoint (0.55, 0.3, 0.2)"
- **Reusable trajectory recipes** — "pick-from-bin-A → drop-on-conveyor: 7-waypoint template, avg 4.2 s"
- **Deployment topology** — "sim host robotics-lab-02 currently runs omniarm6_01 and omniarm6_02"

## What does NOT belong here

- Per-tick telemetry (use logs, not memory)
- Raw joint vectors with no associated decision
- Secrets — bridge API tokens, operator credentials
- Operator chat transcripts beyond the rationale for a specific setpoint

## Embedding provider

Default: local [Ollama](https://ollama.com/) with `nomic-embed-text`.

```bash
ollama pull nomic-embed-text
ollama serve   # if not already running
```

The tools will pick it up automatically at `http://127.0.0.1:11434`. If Ollama is unreachable, search falls back to BM25 token scoring. Ranking quality drops but nothing breaks.

## Why local?

Same reasoning as OmniLink's other first-party agents: the cloud-backed memory store hit embedding-quota rate limits that produced blank replies under load. Local storage removes that failure mode entirely: writes are disk I/O, embeddings run on the local machine, and memories are plain markdown files under version control.

The tradeoff: memories don't sync across machines. For Axis this is even more appropriate than for a general-purpose assistant — robot calibrations are deployment-specific, so the local-to-the-operator store aligns with the intended scope.
