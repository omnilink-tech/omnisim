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

"""Maze utilities — parse husky_maze.omniworld walls into a cell adjacency graph.

The world file labels two wall flavors:

* ``H_*`` runs along x (size 2 0.2 1) and blocks N-S passage.
* ``V_*`` runs along y (size 0.2 2 1) and blocks E-W passage.

Cell ``(col, row)`` center is at world ``(-10 + 2*col, -10 + 2*row)``.

* An H-wall at world ``(x, y)`` with col=(x+10)/2, blocks the passage
  between cells ``(col, row_lo)`` and ``(col, row_lo+1)`` where
  ``row_lo = (y + 10 - 1)/2`` — i.e. the wall sits at the southern edge of
  the upper cell.
* A V-wall at world ``(x, y)`` with row=(y+10)/2, blocks the passage
  between cells ``(col_lo, row)`` and ``(col_lo+1, row)`` where
  ``col_lo = (x + 10 - 1)/2``.

This module is shared between the bridge tools (the agent reads the graph
through a tool) and the standalone solver.
"""

from __future__ import annotations

import re
from collections import deque
from pathlib import Path
from typing import Dict, List, Tuple

CELLS = 11
CELL_SIZE = 2.0
ORIGIN_X = -10.0
ORIGIN_Y = -10.0
START = (0, 10)
GOAL = (10, 0)


_H_PATTERN = re.compile(
    r'Wall\s*\{\s*name\s+"H_\d+"\s+translation\s+'
    r'(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+'
    r'size\s+2\s+0\.2\s+1\s*\}'
)
_V_PATTERN = re.compile(
    r'Wall\s*\{\s*name\s+"V_\d+"\s+translation\s+'
    r'(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+'
    r'size\s+0\.2\s+2\s+1\s*\}'
)


def parse_walls(world_path: Path) -> Tuple[List[Tuple[int, int, int]], List[Tuple[int, int, int]]]:
    """Return ``(h_blocks, v_blocks)``.

    ``h_blocks`` is ``[(col, row_lo, row_hi=row_lo+1), ...]`` — each entry
    means cells ``(col, row_lo)`` and ``(col, row_lo+1)`` are NOT connected.
    ``v_blocks`` is ``[(col_lo, col_hi=col_lo+1, row), ...]``.
    """
    text = world_path.read_text(encoding="utf-8")
    h_blocks: List[Tuple[int, int, int]] = []
    v_blocks: List[Tuple[int, int, int]] = []
    for m in _H_PATTERN.finditer(text):
        x, y, _ = float(m.group(1)), float(m.group(2)), float(m.group(3))
        col = round((x - ORIGIN_X) / CELL_SIZE)
        row_lo = round((y - ORIGIN_Y - 1.0) / CELL_SIZE)
        if 0 <= col < CELLS and 0 <= row_lo < CELLS - 1:
            h_blocks.append((col, row_lo, row_lo + 1))
    for m in _V_PATTERN.finditer(text):
        x, y, _ = float(m.group(1)), float(m.group(2)), float(m.group(3))
        col_lo = round((x - ORIGIN_X - 1.0) / CELL_SIZE)
        row = round((y - ORIGIN_Y) / CELL_SIZE)
        if 0 <= col_lo < CELLS - 1 and 0 <= row < CELLS:
            v_blocks.append((col_lo, col_lo + 1, row))
    return h_blocks, v_blocks


def build_adjacency(h_blocks, v_blocks) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
    """Build a 4-connected adjacency dict for every cell, minus blocked passages."""
    blocked_edges = set()
    for col, r_lo, r_hi in h_blocks:
        blocked_edges.add(((col, r_lo), (col, r_hi)))
        blocked_edges.add(((col, r_hi), (col, r_lo)))
    for c_lo, c_hi, row in v_blocks:
        blocked_edges.add(((c_lo, row), (c_hi, row)))
        blocked_edges.add(((c_hi, row), (c_lo, row)))

    adj: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}
    for col in range(CELLS):
        for row in range(CELLS):
            here = (col, row)
            neighbours: List[Tuple[int, int]] = []
            for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nc, nr = col + dc, row + dr
                if not (0 <= nc < CELLS and 0 <= nr < CELLS):
                    continue
                if (here, (nc, nr)) in blocked_edges:
                    continue
                neighbours.append((nc, nr))
            adj[here] = neighbours
    return adj


def shortest_path(adj, start=START, goal=GOAL) -> List[Tuple[int, int]]:
    """BFS shortest path. Returns [start, ..., goal] or [] if unreachable."""
    if start == goal:
        return [start]
    seen = {start: None}
    q = deque([start])
    while q:
        here = q.popleft()
        if here == goal:
            break
        for nb in adj.get(here, ()):
            if nb in seen:
                continue
            seen[nb] = here
            q.append(nb)
    if goal not in seen:
        return []
    path: List[Tuple[int, int]] = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = seen[cur]
    path.reverse()
    return path


def cell_to_world(col: int, row: int) -> Tuple[float, float]:
    return ORIGIN_X + col * CELL_SIZE, ORIGIN_Y + row * CELL_SIZE


def world_to_cell(x: float, y: float) -> Tuple[int, int]:
    col = round((x - ORIGIN_X) / CELL_SIZE)
    row = round((y - ORIGIN_Y) / CELL_SIZE)
    col = max(0, min(CELLS - 1, col))
    row = max(0, min(CELLS - 1, row))
    return col, row


def load_graph(world_path: Path):
    """Parse the world file once and return ``(adj, h_blocks, v_blocks)``."""
    h_blocks, v_blocks = parse_walls(world_path)
    adj = build_adjacency(h_blocks, v_blocks)
    return adj, h_blocks, v_blocks
