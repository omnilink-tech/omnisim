# Mission Captain — long_term_memory

Saved **mission patterns**: sequences of delegations that worked for a given operator-goal shape on a given world combination.

Each note's title follows the pattern:

```
<specialists involved>: <mission-shape> on <world(s)>
```

E.g. `"Husky Maze: corners-tour on Husky Maze (Corners Tour)"` or `"Husky Maze: find-and-report on Husky Maze (Visual)"`.

The body holds the leg sequence in the form the captain can lift directly:

```
1. delegate_to_agent("Husky Maze", "drive to cell (5, 3) and complete_mission")
2. delegate_to_agent("Husky Maze", "drive back to the start cell (0, 10) and complete_mission")
...
```

When a new operator request comes in, the captain calls `recall` with keywords from the request. If a pattern matches, the captain replays it instead of decomposing from scratch.

Hybrid-indexed: vector embeddings via local Ollama when available, BM25 lexical fallback, fused via Reciprocal Rank Fusion. SQLite index regenerated from markdown notes.
