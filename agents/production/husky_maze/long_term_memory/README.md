# long_term_memory

Agent-written, source-controlled. The Husky Maze agent persists durable notes here between sessions:

- BFS shortest paths it has computed for a given world title
- Recurring fault signatures the operator wants the agent to recognise
- Operator overrides ("for this lab, drive at speed 0.3 not 0.5")
- World-specific quirks ("husky_maze_unknown.wbt at row 7 has a tight corridor — snap before turning")

Each note is a markdown file with a YAML-ish frontmatter (`id`, `title`, `tags`, `created_at`, `updated_at`). The companion `_index.sqlite` holds the same fields plus an embedding vector (from local Ollama when available; BM25 fallback when not).

Hybrid retrieval (vector cosine + BM25, fused via RRF) means short paraphrased queries still find the right note.

The agent reads + writes this directory through the `save_local_memory`, `search_local_memory`, `list_local_memories`, and `forget_local_memory` tools, or via the unified `recall` tool that also queries the `knowledge/` folder.

These notes are committed alongside the agent code so future operators (and future agent instances) inherit the institutional memory.
