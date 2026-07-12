"""Engine-level joint-limit stress test for OmniSim.

Pelts Spot with escalating impacts and per-tick asserts that joint
positions never leave the URDF [lower, upper] range and joint velocities
never exceed velocity_limit. Writes a result file and quits the
simulation with status 0 (PASS) or 1 (FAIL).

This test is intentionally independent of the RL pipeline.
"""
import os as _bootstrap_os
_bootstrap_diag = _bootstrap_os.environ.get(
    "OMNISIM_TEST_DIAG_FILE", "C:/tmp/omnisim_joint_limits_diag.txt")
try:
    import os
    os.makedirs(os.path.dirname(_bootstrap_diag) or ".", exist_ok=True)
    with open(_bootstrap_diag, "w") as _df:
        _df.write("controller entered\n")
        _df.flush()
except Exception:
    pass

import math
import sys
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from controller import Supervisor
except BaseException:
    try:
        with open(_bootstrap_diag, "a") as _df:
            _df.write("FAILED to import controller:\n")
            _df.write(traceback.format_exc())
    except Exception:
        pass
    raise

LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINTS = ("hip_x", "hip_y", "knee")

NOMINAL = {}
for leg in LEGS:
    sign = +1.0 if "left" in leg else -1.0
    NOMINAL[f"{leg}_hip_x"] = sign * 0.30
    NOMINAL[f"{leg}_hip_y"] = 0.30
    NOMINAL[f"{leg}_knee"] = -0.60

TOL_VEL = 0.5    # rad/s - small buffer for finite-diff + solver discretization
TOL_POS = 0.005  # rad   - small buffer for solver-step boundary settle


def parse_urdf_limits(urdf_path):
    out = {}
    root = ET.parse(urdf_path).getroot()
    for jn in root.iter("joint"):
        if jn.attrib.get("type") != "revolute":
            continue
        name = jn.attrib["name"]
        lim = jn.find("limit")
        if lim is None:
            continue
        out[name] = (float(lim.attrib.get("lower", "0")),
                     float(lim.attrib.get("upper", "0")),
                     float(lim.attrib.get("velocity", "0")))
    return out


def main():
    sup = Supervisor()
    step_ms = int(sup.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    urdf_path = Path(__file__).resolve().parents[5] / \
        "projects/robots/boston_dynamics/spot/urdf/spot.classic.urdf"
    limits = parse_urdf_limits(urdf_path)
    sys.stdout.write(
        f"[joint_limits_stress] loaded {len(limits)} revolute joint limits\n")
    sys.stdout.flush()

    motors = {}
    sensors = {}
    for leg in LEGS:
        for jname in JOINTS:
            jn = f"{leg}_{jname}"
            mot = sup.getDevice(f"{jn}_motor")
            if mot is None:
                sys.stdout.write(f"[joint_limits_stress] missing motor {jn}_motor\n")
                sup.simulationQuit(1)
                return 1
            motors[jn] = mot
            s = mot.getPositionSensor()
            if s is None:
                sys.stdout.write(f"[joint_limits_stress] missing sensor on {jn}\n")
                sup.simulationQuit(1)
                return 1
            s.enable(step_ms)
            sensors[jn] = s
            mot.setPosition(NOMINAL.get(jn, 0.0))

    root_children = sup.getRoot().getField("children")
    live_cubes = []
    cube_count = [0]

    def spawn_cube(x, y, z, vx, vy, vz, mass=1.0, side=0.10):
        cube_count[0] += 1
        defname = f"STRESS_CUBE_{cube_count[0]}"
        node_str = (
            f'DEF {defname} Solid {{ '
            f'translation {x:.3f} {y:.3f} {z:.3f} '
            f'children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor 1.0 0.2 0.2 metalness 0 }} '
            f'geometry Box {{ size {side} {side} {side} }} '
            f'}} ] '
            f'boundingObject Box {{ size {side} {side} {side} }} '
            f'physics Physics {{ mass {mass:.2f} }} '
            f'name "{defname}" '
            f'}}'
        )
        root_children.importMFNodeFromString(-1, node_str)
        node = sup.getFromDef(defname)
        if node is not None:
            try:
                node.setVelocity([vx, vy, vz, 0.0, 0.0, 0.0])
            except Exception:
                pass
            live_cubes.append((sup.getTime(), node))

    prev_q = {jn: None for jn in motors}
    worst_vel = {jn: 0.0 for jn in motors}
    worst_q_under = {jn: 0.0 for jn in motors}
    worst_q_over = {jn: 0.0 for jn in motors}
    violation_count = 0
    first_violation = None

    next_barrage_t = 0.0
    anvil_dropped = False
    sim_end = 11.0
    sim_t = 0.0

    while sup.step(step_ms) != -1:
        sim_t = sup.getTime()
        if sim_t >= sim_end:
            break

        if 1.5 <= sim_t < 4.5 and sim_t >= next_barrage_t:
            i = int((sim_t - 1.5) / 0.5)
            ang = i * (math.pi / 2)
            r = 0.8
            cx, cy = r * math.cos(ang), r * math.sin(ang)
            spawn_cube(cx, cy, 0.5,
                       -5.0 * math.cos(ang), -5.0 * math.sin(ang), 0.0,
                       mass=1.0, side=0.10)
            next_barrage_t = sim_t + 0.5

        if 4.5 <= sim_t < 7.0 and sim_t >= next_barrage_t:
            i = int((sim_t - 4.5) / 0.5)
            ang = i * (math.pi / 2) + 0.5
            r = 0.9
            cx, cy = r * math.cos(ang), r * math.sin(ang)
            spawn_cube(cx, cy, 0.5,
                       -10.0 * math.cos(ang), -10.0 * math.sin(ang), 0.0,
                       mass=5.0, side=0.15)
            next_barrage_t = sim_t + 0.5

        if 7.0 <= sim_t < 7.1 and not anvil_dropped:
            spawn_cube(0.0, 0.0, 2.5, 0.0, 0.0, -2.0, mass=20.0, side=0.30)
            anvil_dropped = True

        for jn, sensor in sensors.items():
            try:
                q = float(sensor.getValue())
            except Exception:
                continue
            pq = prev_q[jn]
            prev_q[jn] = q
            if pq is None:
                continue
            qd = (q - pq) / step_dt
            lo, hi, vlim = limits.get(jn, (0.0, 0.0, 0.0))
            if abs(qd) > worst_vel[jn]:
                worst_vel[jn] = abs(qd)
            if vlim > 0 and abs(qd) > vlim + TOL_VEL:
                violation_count += 1
                if first_violation is None:
                    first_violation = (
                        f"t={sim_t:.2f}s {jn} qd={qd:+.2f} rad/s "
                        f"(limit +-{vlim})")
            if lo != hi:
                if q < lo:
                    under = lo - q
                    if under > worst_q_under[jn]:
                        worst_q_under[jn] = under
                    if under > TOL_POS:
                        violation_count += 1
                        if first_violation is None:
                            first_violation = (
                                f"t={sim_t:.2f}s {jn} q={q:+.3f} < lo={lo:+.3f}")
                elif q > hi:
                    over = q - hi
                    if over > worst_q_over[jn]:
                        worst_q_over[jn] = over
                    if over > TOL_POS:
                        violation_count += 1
                        if first_violation is None:
                            first_violation = (
                                f"t={sim_t:.2f}s {jn} q={q:+.3f} > hi={hi:+.3f}")

        now = sup.getTime()
        survivors = []
        for tspawn, node in live_cubes:
            if now - tspawn > 1.5:
                try:
                    node.remove()
                except Exception:
                    pass
            else:
                survivors.append((tspawn, node))
        live_cubes[:] = survivors

    result_path = Path(os.environ.get(
        "OMNISIM_TEST_RESULT_FILE",
        "C:/tmp/omnisim_joint_limits_result.txt"))
    try:
        result_path.parent.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    verdict = "PASS" if violation_count == 0 else "FAIL"
    lines = [
        f"verdict={verdict}",
        f"violations={violation_count}",
        f"first_violation={first_violation or 'none'}",
        f"sim_time={sim_t:.3f}s",
    ]
    for jn in motors:
        lo, hi, vlim = limits.get(jn, (0.0, 0.0, 0.0))
        lines.append(
            f"  {jn}: worst_qd={worst_vel[jn]:6.2f}/{vlim:.1f}rad/s  "
            f"q_under={worst_q_under[jn]:+.4f}  "
            f"q_over={worst_q_over[jn]:+.4f}")
    try:
        result_path.write_text("\n".join(lines) + "\n")
    except Exception:
        pass
    for ln in lines:
        sys.stdout.write(f"[joint_limits_stress] {ln}\n")
    sys.stdout.flush()
    sup.simulationQuit(0 if violation_count == 0 else 1)
    return 0 if violation_count == 0 else 1


if __name__ == "__main__":
    # Mirror any uncaught exception to a file so it's diagnosable from
    # the launcher (OmniSim's --batch mode swallows controller stderr).
    diag_path = os.environ.get(
        "OMNISIM_TEST_DIAG_FILE", "C:/tmp/omnisim_joint_limits_diag.txt")
    try:
        Path(diag_path).parent.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    try:
        main()
    except BaseException:
        import traceback
        try:
            with open(diag_path, "w") as _f:
                _f.write(traceback.format_exc())
        except Exception:
            pass
        raise
