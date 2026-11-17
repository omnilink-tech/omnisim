#!/usr/bin/env python3
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
"""
baseline_metrics.py - baseline for the OmniSim corridor comparison.

Emits the five metrics OmniLink asked for, per run and aggregated:
    completion rate, travel time, path length, minimum clearance, collisions

Ground rules from their reply: the waypoint list and the waypoint-acceptance
margin stay FIXED across every condition, and disturbances are applied one at a
time. Geometry, noise model and controller are taken unchanged from
simulacion_montecarlo.py so the numbers stay comparable to the existing study.

Does not overwrite mision.json.

    python baseline_metrics.py                      # all conditions, 200 runs each
    python baseline_metrics.py --runs 50 --solid-shelves
"""

import argparse
import json
import numpy as np

# -- geometry / model constants (unchanged from simulacion_montecarlo.py) -----
SHELF_W, SHELF_H, Y_MID, GAP, Z_FIJO = 3.0, 4.0, 2.0, 2.0, 1.0
E = [i * (SHELF_W + GAP) for i in range(3)]
CHECKPOINT_STEP = 0.30

MARGEN       = 0.18     # FIXED across all conditions
DT           = 0.1
K_P          = 2.0
VMAX         = 3.0
MAX_ITER     = 400      # per-waypoint budget; exhausting it = waypoint missed
SIGMA_LIDAR  = 0.01
SIGMA_MOTOR  = 0.005
DISP_INICIAL = 0.02


# -- mission -----------------------------------------------------------------
def waypoints_base(x0, nombre):
    x1, z = x0 + SHELF_W, Z_FIJO
    return [
        {"pos": [x0, 0.0,     z], "next": [x1, 0.0,     z], "msg": nombre + ": -> row1"},
        {"pos": [x1, 0.0,     z], "next": [x1, Y_MID,   z], "msg": nombre + ": up right"},
        {"pos": [x1, Y_MID,   z], "next": [x0, Y_MID,   z], "msg": nombre + ": <- row2"},
        {"pos": [x0, Y_MID,   z], "next": [x0, SHELF_H, z], "msg": nombre + ": up left"},
        {"pos": [x0, SHELF_H, z], "next": [x1, SHELF_H, z], "msg": nombre + ": -> row3"},
        {"pos": [x1, SHELF_H, z], "next": [x0, 0.0,     z], "msg": nombre + ": back to start"},
    ]


# Scan lanes. Shelves occupy x in [0,3], [5,8], [10,13] with y in [0,4]; the
# aisles are x in [3,5] and [8,10]. The drone flies a boustrophedon of four
# vertical passes at STANDOFF from the faces it scans, transiting around the
# shelf ends. A centre-aisle pass covers the two faces bounding that aisle.
STANDOFF = 0.5
LANE_A = -STANDOFF                          # E1 outer (left) face
LANE_B = E[1] - GAP / 2.0                   # aisle 1 centre: E1 right + E2 left
LANE_C = E[2] - GAP / 2.0                   # aisle 2 centre: E2 right + E3 left
LANE_D = E[2] + SHELF_W + STANDOFF          # E3 outer (right) face
Y_LO = -STANDOFF                            # transit lane below the shelf ends
Y_HI = SHELF_H + STANDOFF                   # transit lane above the shelf ends


def route_aisles():
    """Obstacle-aware route: every leg lies in free space."""
    pts = [(LANE_A, Y_LO), (LANE_A, Y_HI),
           (LANE_B, Y_HI), (LANE_B, Y_LO),
           (LANE_C, Y_LO), (LANE_C, Y_HI),
           (LANE_D, Y_HI), (LANE_D, Y_LO)]
    labels = ["scan E1 outer face", "transit over E1",
              "scan E1 right + E2 left", "transit aisle 1 -> 2",
              "scan E2 right + E3 left", "transit over E3",
              "scan E3 outer face"]
    return [{"pos": [pts[i][0], pts[i][1], Z_FIJO],
             "next": [pts[i + 1][0], pts[i + 1][1], Z_FIJO],
             "msg": labels[i]} for i in range(len(pts) - 1)]


def route_through():

    base = (
        [{"pos": [0.0, 0.0, Z_FIJO], "next": [E[0], 0.0, Z_FIJO], "msg": "corridor start"}]
        + waypoints_base(E[0], "E1")
        + [{"pos": [E[0], 0.0, Z_FIJO], "next": [E[1], 0.0, Z_FIJO], "msg": "transit E1->E2"}]
        + waypoints_base(E[1], "E2")
        + [{"pos": [E[1], 0.0, Z_FIJO], "next": [E[2], 0.0, Z_FIJO], "msg": "transit E2->E3"}]
        + waypoints_base(E[2], "E3")
    )
    return base


def densify(base):
    dense = []
    for h in base:
        pos, nxt = np.array(h["pos"][:3]), np.array(h["next"][:3])
        dist = np.linalg.norm(nxt[:2] - pos[:2])
        if dist <= CHECKPOINT_STEP:
            dense.append(h)
            continue
        n = int(np.ceil(dist / CHECKPOINT_STEP))
        pts = [pos + (i / n) * (nxt - pos) for i in range(n + 1)]
        for j in range(n):
            dense.append({"pos": pts[j].tolist(), "next": pts[j + 1].tolist(),
                          "msg": h["msg"] + " ." + str(j + 1) + "/" + str(n)})
    return dense


def build_mission(route="aisles"):
    base = route_aisles() if route == "aisles" else route_through()
    return base, densify(base)


# -- obstacles ---------------------------------------------------------------
# An AABB is (x0, y0, x1, y1).
SHELF_BOXES = [(xi, 0.0, xi + SHELF_W, SHELF_H) for xi in E]

# Blocks a scan pass mid-leg. For the aisle route this sits in aisle 1 on lane B,
# clear of both shelf faces; for the legacy route it blocks the E1->E2 transit.
BLOCKING_BOX = {"aisles": (3.70, 1.70, 4.30, 2.30),
                "through": (3.40, -0.60, 4.00, 0.60)}


def dist_to_box(p, box):
    """Euclidean distance from point to AABB; 0.0 when inside."""
    x0, y0, x1, y1 = box
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return float(np.hypot(dx, dy))


def clearance(p, boxes):
    return min((dist_to_box(p, b) for b in boxes), default=float("inf"))


# -- one run -----------------------------------------------------------------
def run_once(seed, condition, mapa, solid_shelves, gate_on="next", route="aisles"):
    """gate_on="pos"  reproduces the acceptance test in simulacion_montecarlo.py
    and cerebro_node.py: it gates on the waypoint being left while commanding
    toward the next one, which deadlocks at every leg transition.
    gate_on="next" gates on the waypoint actually being commanded."""
    rng = np.random.default_rng(seed)

    boxes = list(SHELF_BOXES) if solid_shelves else []
    sigma_lidar = SIGMA_LIDAR
    goal_shift_idx, goal_shift_vec = None, np.zeros(3)

    if condition == "blocked":
        boxes = boxes + [BLOCKING_BOX[route]]
    elif condition == "sensor_noise":
        sigma_lidar = SIGMA_LIDAR * 5.0            # 1 cm -> 5 cm
    elif condition == "goal_shift":
        goal_shift_idx = len(mapa) // 2            # one waypoint, mid-mission
        goal_shift_vec = np.array([0.40, 0.40, 0.0])

    pos = np.array(mapa[0]["pos"], dtype=float)
    pos[:2] += rng.normal(0, DISP_INICIAL, 2)

    steps = 0
    path_len = 0.0
    min_clear = float("inf")
    collision_steps = 0
    missed = 0

    for i, hito in enumerate(mapa):
        target_cmd = np.array(hito["next"], dtype=float)
        if goal_shift_idx is not None and i == goal_shift_idx:
            target_cmd = target_cmd + goal_shift_vec
        target_gate = (np.array(hito["pos"][:2]) if gate_on == "pos"
                       else target_cmd[:2].copy())

        reached = False
        for _ in range(MAX_ITER):
            perceived = pos.copy()
            perceived[:2] += rng.normal(0, sigma_lidar, 2)
            if np.linalg.norm(perceived[:2] - target_gate) <= MARGEN:
                reached = True
                break

            vel = np.clip(K_P * (target_cmd - perceived), -VMAX, VMAX)
            motor = np.zeros(3)
            motor[:2] = rng.normal(0, SIGMA_MOTOR, 2)
            prev = pos.copy()
            pos = pos + (vel + motor) * DT
            pos[2] = Z_FIJO
            steps += 1
            path_len += float(np.linalg.norm(pos[:2] - prev[:2]))

            if boxes:
                # sub-sample the step so a fast leg cannot tunnel through a box
                for t in (0.34, 0.67, 1.0):
                    q = prev[:2] + t * (pos[:2] - prev[:2])
                    c = clearance(q, boxes)
                    min_clear = min(min_clear, c)
                    if c <= 0.0:
                        collision_steps += 1
                        break

        if not reached:
            missed += 1

    return {
        "completed": missed == 0,
        "waypoints_missed": missed,
        "travel_time_s": steps * DT,
        "path_length_m": path_len,
        "min_clearance_m": min_clear if boxes else float("nan"),
        "collision_steps": collision_steps,
        "collided": collision_steps > 0,
    }


# -- aggregation -------------------------------------------------------------
def summarize(runs):
    def stat(key):
        v = np.array([r[key] for r in runs], dtype=float)
        v = v[np.isfinite(v)]
        if v.size == 0:
            return None
        return {"mean": float(v.mean()), "std": float(v.std()),
                "p95": float(np.percentile(v, 95)),
                "min": float(v.min()), "max": float(v.max())}

    n = len(runs)
    return {
        "n_runs": n,
        "completion_rate": sum(r["completed"] for r in runs) / n,
        "collision_rate": sum(r["collided"] for r in runs) / n,
        "travel_time_s": stat("travel_time_s"),
        "path_length_m": stat("path_length_m"),
        "min_clearance_m": stat("min_clearance_m"),
        "waypoints_missed": stat("waypoints_missed"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--seed", type=int, default=20260901)
    ap.add_argument("--solid-shelves", action="store_true",
                    help="treat the three shelves as collision geometry")
    ap.add_argument("--route", choices=["aisles", "through"], default="aisles",
                    help="'aisles' flies the free-space boustrophedon; "
                         "'through' is the legacy route that crosses the shelves")
    ap.add_argument("--gate", choices=["pos", "next"], default="next",
                    help="acceptance gate: 'pos' reproduces the original "
                         "deadlocking logic, 'next' is the corrected one")
    ap.add_argument("--out", default="baseline_metrics.json")
    args = ap.parse_args()

    base, mapa = build_mission(args.route)
    conditions = ["none", "blocked", "sensor_noise", "goal_shift"]

    print("waypoints: " + str(len(base)) + " base -> " + str(len(mapa)) +
          " checkpoints (step " + str(CHECKPOINT_STEP) + " m, margin " +
          str(MARGEN) + " m, FIXED)")
    print("route: " + args.route + "   shelves as obstacles: " + str(args.solid_shelves) +
          "   runs/condition: " + str(args.runs) + "\n")

    report = {"config": {"runs": args.runs, "seed": args.seed,
                         "margin_m": MARGEN, "checkpoint_step_m": CHECKPOINT_STEP,
                         "n_checkpoints": len(mapa),
                         "solid_shelves": args.solid_shelves,
                         "route": args.route, "standoff_m": STANDOFF,
                         "acceptance_gate": args.gate,
                         "sigma_lidar_m": SIGMA_LIDAR, "sigma_motor_m": SIGMA_MOTOR,
                         "dt_s": DT, "k_p": K_P},
              "conditions": {}}

    hdr = ("{:<14}{:>8}{:>9}{:>10}{:>10}{:>10}"
           .format("condition", "compl.", "collis.", "time s", "path m", "clear m"))
    print(hdr)
    print("-" * len(hdr))
    for cond in conditions:
        runs = [run_once(args.seed + k, cond, mapa, args.solid_shelves,
                         args.gate, args.route)
                for k in range(args.runs)]
        s = summarize(runs)
        report["conditions"][cond] = s
        clear = s["min_clearance_m"]
        clear_s = "{:.3f}".format(clear["min"]) if clear else "n/a"
        print("{:<14}{:>7.1f}%{:>8.1f}%{:>10.2f}{:>10.2f}{:>10}".format(
            cond,
            s["completion_rate"] * 100,
            s["collision_rate"] * 100,
            s["travel_time_s"]["mean"],
            s["path_length_m"]["mean"],
            clear_s))

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print("\nwrote " + args.out)


if __name__ == "__main__":
    main()
