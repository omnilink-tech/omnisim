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
"""Read-only pose recording, called by an existing supervisor after each step.

No engine imports, stepping, motor commands, resets, or monkeypatches here.
See CINEMATIC_REPLAY.md for controller integration and the coordinate contract.
"""
from __future__ import annotations

import json
import math
from pathlib import Path


class PoseRecorder:
    """Write row-major world-space getPose() matrices without replacing a take."""

    def __init__(self, path: str | Path, nodes: dict):
        if not nodes or any(not isinstance(k, str) or not k or v is None
                            for k, v in nodes.items()):
            raise ValueError("Recording requires unique, nonempty names and live nodes")
        self.nodes = dict(nodes)
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", encoding="utf-8")
        self.last_time = -math.inf

    def sample(self, simulation_time: float) -> None:
        t = float(simulation_time)
        if not math.isfinite(t) or t < 0 or t <= self.last_time:
            raise ValueError("Time must increase; start a new recording after a reset")
        poses = {name: list(node.getPose()) for name, node in self.nodes.items()}
        if any(len(p) != 16 or not all(math.isfinite(v) for v in p)
               for p in poses.values()):
            raise ValueError("getPose() must return 16 finite matrix values")
        self.stream.write(json.dumps({"time": t, "poses": poses},
                                     separators=(",", ":"), allow_nan=False) + "\n")
        self.stream.flush()
        self.last_time = t

    def close(self) -> None:
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def discover_rigid_nodes(root) -> dict:
    """Find named rigid nodes, including URDF links, below one robot.

    Duplicate names fail rather than silently replacing a link. For an unusual
    PROTO or duplicate names, supply an explicit name-to-node mapping instead.
    Call separately for each robot and record separate JSONL files.
    """
    nodes, seen = {}, set()

    def visit(node):
        if node is None or node.getId() in seen:
            return
        seen.add(node.getId())
        if node.getBaseTypeName() in ("Robot", "Solid", "URDFRobot"):
            field = node.getField("name") or node.getBaseNodeField("name")
            name = field.getSFString() if field else str(node.getId())
            if name in nodes:
                raise ValueError(f"Duplicate node name {name!r}; use explicit bindings")
            nodes[name] = node
        for name in ("children", "endPoint"):
            field = node.getField(name) or node.getBaseNodeField(name)
            if field:
                if name == "endPoint":
                    visit(field.getSFNode())
                else:
                    for i in range(field.getCount()):
                        visit(field.getMFNode(i))

    visit(root)
    return nodes
