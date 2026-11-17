# Long-term memory

Agent-written session notes persist here (markdown + YAML frontmatter), the same
schema as the other `agents/production/` agents. The courier agent can record a
solved route ("delivered the red package to dock 2 in N steps") so a later
session recalls the plan instead of re-deriving it.

Frontmatter:

```markdown
---
id: 2026-06-26-red-to-dock2
title: Red package -> Dock 2
tags: [courier, bay-a, dock-2]
created_at: 2026-06-26T00:00:00Z
---

run_route([{pick bay-a}, {deliver dock-2}]); verified delivered, deck empty.
```

Empty by default — notes accrue as the agent runs.
