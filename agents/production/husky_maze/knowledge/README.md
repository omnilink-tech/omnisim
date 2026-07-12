# Knowledge — Husky Maze

Curated grounding docs the Husky Maze agent reads at runtime (and that anyone hand-writing a session prompt should skim).

- [husky-bridge.md](husky-bridge.md) — the HTTP contract the agent uses to drive the Husky, including the action surface, mode lifecycle, fault codes, and the world-frame convention.
- [maze-layout.md](maze-layout.md) — geometry, wall convention, start/goal, and how to read the runtime adjacency graph.

The maze graph itself is not stored here — it is parsed from `husky_maze.wbt` on demand by the `get_maze_graph` tool, so the world file remains the single source of truth.
