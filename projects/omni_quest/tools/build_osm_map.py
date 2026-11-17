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

"""Fetch a REAL OpenStreetMap walking network and build a routable nav graph.

This is the MAP layer of the native outdoor stack — the open equivalent of a
Google-Maps road/sidewalk network, for ANY real place on Earth, no simulation.
We pull the walkable ways (footways, paths, crossings, quiet roads) for a bounding
box, project lat/lon to local ENU metres, and emit a graph the global planner
(citygraph) routes and re-routes on — replacing the hand-authored toy grid.

    python tools/build_osm_map.py            # demo: fetch a real area + route + reroute
    python tools/build_osm_map.py <lat> <lon> <radius_m> <out.json>

Output JSON: {"ref":[lat0,lon0], "nodes":{id:[x,y]}, "edges":[[a,b,kind,meta]...]}
where x,y are metres east/north of ref, kind is "walk" or "cross".
"""

from __future__ import annotations

import json
import math
import sys
import urllib.parse
import urllib.request
from pathlib import Path

try:                                    # AVG intercepts TLS on this machine
    import truststore
    truststore.inject_into_ssl()
except Exception:
    pass

OVERPASS = "https://overpass-api.de/api/interpreter"
# walkable ways: pedestrian infrastructure + quiet roads a robot/pedestrian may use
WALK_RE = ("footway|path|pedestrian|steps|living_street|residential|service|"
           "track|cycleway|unclassified|tertiary")
EARTH_R = 6378137.0


def fetch(bbox):
    """bbox = (south, west, north, east) in degrees -> Overpass JSON."""
    s, w, n, e = bbox
    q = (f'[out:json][timeout:60];'
         f'(way["highway"~"{WALK_RE}"]({s},{w},{n},{e}););out geom;')
    body = urllib.parse.urlencode({"data": q}).encode()
    req = urllib.request.Request(OVERPASS, data=body, headers={
        "User-Agent": "OmniQuest-nav/0.1 (research; outdoor robot navigation)",
        "Accept": "application/json",
        "Content-Type": "application/x-www-form-urlencoded"})
    return json.loads(urllib.request.urlopen(req, timeout=90).read())


def enu(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R
    return round(x, 2), round(y, 2)


def build(osm, ref):
    """Return (nodes {id:(x,y)}, edges [[a,b,kind,meta]]). Ways share OSM node ids at
    intersections, so the network is connected automatically."""
    lat0, lon0 = ref
    nodes, edges, seen = {}, [], set()
    for el in osm.get("elements", []):
        if el.get("type") != "way":
            continue
        tags = el.get("tags", {})
        hw = tags.get("highway", "")
        nds, geom = el.get("nodes", []), el.get("geometry", [])
        if len(nds) != len(geom) or len(nds) < 2:
            continue
        is_cross = (tags.get("footway") == "crossing" or "crossing" in tags
                    or hw == "crossing")
        kind = "cross" if is_cross else "walk"
        ids = []
        for nid, g in zip(nds, geom):
            nodes[nid] = enu(g["lat"], g["lon"], lat0, lon0)
            ids.append(nid)
        for a, b in zip(ids, ids[1:]):
            key = (min(a, b), max(a, b))
            if key in seen:
                continue
            seen.add(key)
            edges.append([a, b, kind, {"name": tags.get("name", ""), "hw": hw}])
    return nodes, edges


def bbox_around(lat, lon, radius_m):
    dlat = radius_m / EARTH_R * 180.0 / math.pi
    dlon = dlat / math.cos(math.radians(lat))
    return (lat - dlat, lon - dlon, lat + dlat, lon + dlon)


def _demo():
    # A real, dense walkable grid: Midtown Manhattan around Herald Square.
    lat, lon, radius = 40.7480, -73.9870, 220.0
    print(f"[osm] fetching real map @ ({lat},{lon}) r={radius:.0f} m ...")
    osm = fetch(bbox_around(lat, lon, radius))
    nodes, edges = build(osm, (lat, lon))
    walk = sum(1 for e in edges if e[2] == "walk")
    cross = sum(1 for e in edges if e[2] == "cross")
    print(f"[osm] built graph: {len(nodes)} nodes, {len(edges)} edges "
          f"({walk} walk, {cross} cross)")

    # Route + reroute on the REAL network.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                           / "controllers" / "omni_quest_nav"))
    import citygraph
    g = citygraph.CityGraph(nodes, edges)
    # pick the westmost and eastmost nodes as start/goal (a real cross-town route)
    start = min(nodes, key=lambda n: nodes[n][0])
    goal = max(nodes, key=lambda n: nodes[n][0])
    route = g.plan(start, goal)
    if not route:
        print("[osm] no route found (disconnected sample)"); return
    length = sum(g.cost(leg["a"], leg["b"]) for leg in route)
    print(f"[osm] route: {len(route)} legs, {length:.0f} m, "
          f"start={nodes[start]} -> goal={nodes[goal]}")
    # block a leg in the MIDDLE of the route (in the dense grid, where alternates
    # exist) and re-plan — the global reroute when a segment is impassable.
    mid = route[len(route) // 2]
    blk = mid["key"]
    alt = g.plan(start, goal, {blk})
    bn = (nodes[mid["a"]], nodes[mid["b"]])
    if alt and [l["key"] for l in alt] != [l["key"] for l in route]:
        alen = sum(g.cost(leg["a"], leg["b"]) for leg in alt)
        print(f"[osm] blocked a mid-route segment at {bn[0]}->{bn[1]}")
        print(f"[osm] REROUTE: {len(alt)} legs, {alen:.0f} m "
              f"(+{alen - length:+.0f} m detour) -> still reaches the destination")
    else:
        print("[osm] reroute: no alternate for that segment")


def main(argv):
    if len(argv) >= 4:
        lat, lon, radius, out = float(argv[0]), float(argv[1]), float(argv[2]), argv[3]
        osm = fetch(bbox_around(lat, lon, radius))
        nodes, edges = build(osm, (lat, lon))
        Path(out).write_text(json.dumps(
            {"ref": [lat, lon], "nodes": {str(k): v for k, v in nodes.items()},
             "edges": edges}), encoding="utf-8")
        print(f"[osm] wrote {out}: {len(nodes)} nodes, {len(edges)} edges")
    else:
        _demo()


if __name__ == "__main__":
    main(sys.argv[1:])
