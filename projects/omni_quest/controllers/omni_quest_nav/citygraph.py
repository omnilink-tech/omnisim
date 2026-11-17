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

"""General outdoor route graph + A* planner with blocked-edge reroute.

A graph of walkable segments ("walk" edges) and road crossings ("cross" edges);
plan() runs A* from a start node to a goal node, avoiding any edge marked blocked,
and returns the COST-OPTIMAL route. The cost model is what keeps the robot from
wasting time or going where it shouldn't:

  * distance (metres) — the obvious term,
  * a per-segment TRANSFER penalty — prefer few long legs over many short ones
    (fewer turns / hand-offs = a straighter, faster path),
  * a CROSSING penalty — stepping into a road costs extra (wait for a gap, exposure
    to traffic), so the planner stays on one sidewalk unless crossing truly saves
    time. A distance-only planner crosses roads gratuitously.

A* (distance-to-goal heuristic) expands far fewer nodes than Dijkstra on a large
network, so this scales from the 5-node demo to a real OSM street graph (hundreds
of nodes) unchanged. The graph itself is general: build it by hand, from a grid
(grid_graph), or from OpenStreetMap (tools/build_osm_map.py) — the planner is the
same. This is the global half of a Nav2-style global+local stack; the stereo
follow-the-gap planner is the local half.

Edge meta carries the geometry the navigator needs to drive the segment:
  walk  : {"axis":"x"|"y", "lane": <fixed coord>}   travel along axis to node b
  cross : {"axis":"x"|"y", "lane": <fixed coord>, "road": <road centre coord>}
"""

from __future__ import annotations

import heapq
import math

SEGMENT_PENALTY_M = 1.5    # per-leg transfer cost -> prefer fewer, straighter legs
CROSS_PENALTY_M = 12.0     # crossing a road ~ this much extra distance (wait + risk)
KIND_PENALTY = {"cross": CROSS_PENALTY_M}


class CityGraph:
    def __init__(self, nodes, edges):
        # nodes: {id: (x, y)};  edges: [(a, b, kind, meta), ...]
        self.nodes = dict(nodes)
        self.edges = {}                       # frozenset({a, b}) -> (kind, meta)
        self.adj = {}                         # id -> [(nbr, key)]
        for a, b, kind, meta in edges:
            key = frozenset((a, b))
            self.edges[key] = (kind, meta)
            self.adj.setdefault(a, []).append((b, key))
            self.adj.setdefault(b, []).append((a, key))

    def cost(self, a, b):
        """Straight-line distance between two nodes (metres)."""
        return math.dist(self.nodes[a], self.nodes[b])

    def edge_cost(self, key):
        """Travel cost of an edge: distance + transfer penalty + kind penalty."""
        a, b = tuple(key)
        kind = self.edges[key][0]
        return self.cost(a, b) + SEGMENT_PENALTY_M + KIND_PENALTY.get(kind, 0.0)

    def nearest(self, xy):
        return min(self.nodes, key=lambda n: math.dist(self.nodes[n], xy))

    def plan(self, start, goal, blocked=()):
        """Cost-optimal route start->goal as leg dicts {a,b,key,kind,meta}, avoiding
        blocked edge keys; None if unreachable. A* with a distance-to-goal heuristic
        (admissible: edge_cost >= straight-line distance), so the result is optimal."""
        if start not in self.nodes or goal not in self.nodes:
            return None
        blocked = set(blocked)
        gx, gy = self.nodes[goal]

        def h(n):
            x, y = self.nodes[n]
            return math.hypot(gx - x, gy - y)

        g = {start: 0.0}
        prev = {}
        pq = [(h(start), 0.0, start)]
        while pq:
            _, gc, u = heapq.heappop(pq)
            if u == goal:
                break
            if gc > g.get(u, math.inf):
                continue
            for v, key in self.adj.get(u, ()):
                if key in blocked:
                    continue
                ng = gc + self.edge_cost(key)
                if ng < g.get(v, math.inf):
                    g[v] = ng
                    prev[v] = (u, key)
                    heapq.heappush(pq, (ng + h(v), ng, v))
        if goal not in g:
            return None
        out = []
        cur = goal
        while cur != start:
            u, key = prev[cur]
            kind, meta = self.edges[key]
            out.append({"a": u, "b": cur, "key": key, "kind": kind, "meta": meta})
            cur = u
        out.reverse()
        return out

    def route_cost(self, legs):
        return sum(self.edge_cost(leg["key"]) for leg in legs) if legs else math.inf

    def route_length(self, legs):
        return sum(self.cost(leg["a"], leg["b"]) for leg in legs) if legs else math.inf


def grid_graph(nx, ny, spacing=10.0):
    """A general nx-by-ny lattice of 4-connected walkable nodes — a stand-in for any
    derived street/sidewalk network, used to prove the planner generalises past the
    hand-built demo graph. Node ids are (i, j); positions are i,j * spacing."""
    nodes = {(i, j): (i * spacing, j * spacing) for i in range(nx) for j in range(ny)}
    edges = []
    for i in range(nx):
        for j in range(ny):
            if i + 1 < nx:
                edges.append(((i, j), (i + 1, j), "walk", {"axis": "x", "lane": j * spacing}))
            if j + 1 < ny:
                edges.append(((i, j), (i, j + 1), "walk", {"axis": "y", "lane": i * spacing}))
    return CityGraph(nodes, edges)


def build_city_graph(xs, ys, off):
    """Full sidewalk graph for a grid of roads (centrelines xs, ys; sidewalk offset
    off from each centreline). Each intersection gets four sidewalk-corner nodes at
    (x +/- off, y +/- off); WALK edges run the sidewalks over each block; CROSS edges
    step across a road at each intersection (where the robot waits for traffic). The
    result is any-to-any routable across the whole city — the general version of the
    hand-built single-street demo. Returns (nodes, edges) in the CityGraph format."""
    xs, ys = sorted(xs), sorted(ys)

    def nid(i, j, cx, cy):
        return f"i{i}j{j}{cx}{cy}"

    nodes, edges = {}, []
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for cx, sx in (("W", -1.0), ("E", 1.0)):
                for cy, sy in (("S", -1.0), ("N", 1.0)):
                    nodes[nid(i, j, cx, cy)] = (round(x + sx * off, 3),
                                                round(y + sy * off, 3))
            # crossings: step across a road at this intersection (axis = travel dir,
            # lane = the constant coord held while crossing, road = the road centre).
            edges.append((nid(i, j, "W", "N"), nid(i, j, "E", "N"), "cross",
                          {"axis": "x", "lane": round(y + off, 3), "road": x}))
            edges.append((nid(i, j, "W", "S"), nid(i, j, "E", "S"), "cross",
                          {"axis": "x", "lane": round(y - off, 3), "road": x}))
            edges.append((nid(i, j, "W", "N"), nid(i, j, "W", "S"), "cross",
                          {"axis": "y", "lane": round(x - off, 3), "road": y}))
            edges.append((nid(i, j, "E", "N"), nid(i, j, "E", "S"), "cross",
                          {"axis": "y", "lane": round(x + off, 3), "road": y}))
    for i in range(len(xs) - 1):                       # EW walks over each block
        for j, y in enumerate(ys):
            edges.append((nid(i, j, "E", "N"), nid(i + 1, j, "W", "N"), "walk",
                          {"axis": "x", "lane": round(y + off, 3)}))
            edges.append((nid(i, j, "E", "S"), nid(i + 1, j, "W", "S"), "walk",
                          {"axis": "x", "lane": round(y - off, 3)}))
    for i, x in enumerate(xs):                         # NS walks over each block
        for j in range(len(ys) - 1):
            edges.append((nid(i, j, "E", "N"), nid(i, j + 1, "E", "S"), "walk",
                          {"axis": "y", "lane": round(x + off, 3)}))
            edges.append((nid(i, j, "W", "N"), nid(i, j + 1, "W", "S"), "walk",
                          {"axis": "y", "lane": round(x - off, 3)}))
    return nodes, edges


def _dijkstra_cost(g, start, goal, blocked=()):
    """Independent Dijkstra oracle (no heuristic) to check A* optimality in the test."""
    blocked = set(blocked)
    dist = {start: 0.0}
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            return d
        if d > dist.get(u, math.inf):
            continue
        for v, key in g.adj.get(u, ()):
            if key in blocked:
                continue
            nd = d + g.edge_cost(key)
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return dist.get(goal, math.inf)


def _selftest():
    ok = True

    # 1) Bus-stop scenario (unchanged behaviour): direct N walk, reroute around when blocked.
    nodes = {"AN": (-31.0, -24.5), "BN": (31.0, -24.5), "AS": (-31.0, -37.5),
             "BS": (31.0, -37.5), "G": (40.0, -24.5)}
    edges = [("AN", "BN", "walk", {"axis": "x", "lane": -24.5}),
             ("AS", "BS", "walk", {"axis": "x", "lane": -37.5}),
             ("AN", "AS", "cross", {"axis": "y", "lane": -31.0, "road": -31.0}),
             ("BN", "BS", "cross", {"axis": "y", "lane": 31.0, "road": -31.0}),
             ("BN", "G", "walk", {"axis": "x", "lane": -24.5})]
    g = CityGraph(nodes, edges)
    direct = [leg["b"] for leg in g.plan("AN", "G")]
    around = [leg["b"] for leg in g.plan("AN", "G", blocked={frozenset(("AN", "BN"))})]
    print("direct :", ["AN"] + direct)
    print("reroute:", ["AN"] + around)
    ok &= (direct == ["BN", "G"] and around == ["AS", "BS", "BN", "G"])

    # 2) A* returns the OPTIMAL cost (matches an independent Dijkstra oracle) on a grid.
    gg = grid_graph(8, 8)
    s, t = (0, 0), (7, 7)
    legs = gg.plan(s, t)
    a_cost, d_cost = gg.route_cost(legs), _dijkstra_cost(gg, s, t)
    print(f"grid A* cost {a_cost:.2f} vs Dijkstra {d_cost:.2f} "
          f"({len(legs)} legs, {gg.route_length(legs):.1f} m)")
    ok &= abs(a_cost - d_cost) < 1e-9

    # 3) Reroute on the grid when a whole column is blocked: still optimal, still reaches.
    wall = {frozenset(((3, j), (4, j))) for j in range(7)}     # block x=3->4 except top
    detour = gg.plan(s, t, blocked=wall)
    ok &= detour is not None and gg.route_cost(detour) - _dijkstra_cost(gg, s, t, wall) < 1e-9
    print(f"grid reroute around a wall: {len(detour) if detour else 0} legs "
          f"({gg.route_length(detour):.1f} m)")

    # 4) Crossing penalty: given two equal-distance options, prefer the one that does
    #    NOT cross a road (stay on the sidewalk).
    cn = {"P": (0.0, 0.0), "Q": (10.0, 0.0), "M": (5.0, 0.0)}
    ce = [("P", "Q", "walk", {"axis": "x", "lane": 0.0}),                 # straight walk
          ("P", "M", "cross", {"axis": "x", "lane": 0.0, "road": 2.5}),   # via a crossing
          ("M", "Q", "cross", {"axis": "x", "lane": 0.0, "road": 7.5})]
    cg = CityGraph(cn, ce)
    pref = [leg["kind"] for leg in cg.plan("P", "Q")]
    print(f"prefers walk over crossing: {pref}")
    ok &= pref == ["walk"]

    # 5) Full-city graph: any-to-any optimal routing + reroute across the whole grid.
    cn, ce = build_city_graph([-93, -31, 31, 93], [-93, -31, 31, 93], 6.5)
    cg = CityGraph(cn, ce)
    s, t = "i0j0WS", "i3j3EN"
    legs = cg.plan(s, t)
    reach = sum(1 for n in cn if cg.plan(s, n) is not None)
    print(f"city graph: {len(cn)} nodes, {len(ce)} edges; "
          f"{reach}/{len(cn)} reachable from {s}")
    print(f"city route {s}->{t}: {len(legs)} legs, {cg.route_length(legs):.0f} m "
          f"(cost {cg.route_cost(legs):.0f})")
    ok &= legs is not None and reach == len(cn)
    ok &= abs(cg.route_cost(legs) - _dijkstra_cost(cg, s, t)) < 1e-9
    wlegs = [l for l in legs if l["kind"] == "walk"]
    if wlegs:
        blk = wlegs[len(wlegs) // 2]["key"]
        alt = cg.plan(s, t, {blk})
        ok &= alt is not None and abs(cg.route_cost(alt) - _dijkstra_cost(cg, s, t, {blk})) < 1e-9
        print(f"city reroute around a blocked walk: {len(alt) if alt else 0} legs, "
              f"{cg.route_length(alt):.0f} m")

    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    _selftest()
