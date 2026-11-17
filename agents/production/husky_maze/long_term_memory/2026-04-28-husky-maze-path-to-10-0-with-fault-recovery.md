---
id: mem_1777375481352_82f1ab
title: Husky Maze: Path to (10,0) with fault recovery
tags: [Husky Maze, path, fault_recovery, mission_complete]
created_at: 2026-04-28T11:24:41.352452+00:00
updated_at: 2026-04-28T11:24:41.352452+00:00
---
Destination cell: (10, 0).
Strategy: Used try_get_known_map to get the BFS shortest path. Recovered from a goto_cell_timeout by stopping the husky, getting its current position (approximately [8,1]), and then re-planning and executing the remainder of the path, which ended up being [[9, 0], [10, 0]].
Full ordered path: [[0, 10], [0, 9], [0, 8], [0, 7], [1, 7], [2, 7], [3, 7], [3, 6], [2, 6], [2, 5], [2, 4], [1, 4], [0, 4], [0, 3], [0, 2], [1, 2], [1, 1], [2, 1], [2, 0], [3, 0], [3, 1], [3, 2], [4, 2], [4, 1], [4, 0], [5, 0], [5, 1], [5, 2], [6, 2], [6, 1], [7, 1], [8, 1], [8, 2], [8, 3], [8, 4], [7, 4], [6, 4], [5, 4], [5, 5], [6, 5], [6, 6], [5, 6], [5, 7], [5, 8], [4, 8], [3, 8], [3, 9], [4, 9], [4, 10], [5, 10], [6, 10], [6, 9], [7, 9], [7, 10], [8, 10], [9, 10], [10, 10], [10, 9], [10, 8], [10, 7], [10, 6], [10, 5], [9, 5], [9, 4], [10, 4], [10, 3], [9, 3], [9, 2], [10, 2], [10, 1], [9, 1], [9, 0], [10, 0]]
