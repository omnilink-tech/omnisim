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

"""Pick & place wiring test for the Talk-to-Ari physical-grasp demo.

Runs WITHOUT Webots. Stubs `controller` with a fake Supervisor + a fake scene:
three DEF GRASP_CUBE_* rigid bodies and a robot whose tool-mount link ("flange")
reports its LIVE forward-kinematics pose, so the bridge's real grasp-anchor path
(flange pose + tool_reach along the tool axis -> weld nearest cube) is exercised.

Drives:
  * act_pick("red") -> top-down reach, grasp (weld), lift; asserts RED is held
    and rises.
  * act_place()     -> carry to the drop zone, release; asserts RED is dropped
    near the zone and nothing stays held.
  * router "pick up the green cube" / "put it down".

    python test_pick_sequence.py
"""

import math
import sys
import types

br = None  # the bridge module, imported after the controller stub is installed


# ── fake Webots scene ────────────────────────────────────────────────

class _SF:
    def __init__(self, v):
        self.v = list(v)

    def getSFVec3f(self):
        return list(self.v)

    def setSFVec3f(self, v):
        self.v = list(v)


class _Name:
    def __init__(self, s):
        self.s = s

    def getSFString(self):
        return self.s


class _CubeNode:
    def __init__(self, defname, pos):
        self._def = defname
        self._t = _SF(pos)
        self.reset_count = 0

    def getDef(self):
        return self._def

    def getField(self, n):
        return self._t if n == "translation" else None

    def getPosition(self):
        return list(self._t.v)

    def resetPhysics(self):
        self.reset_count += 1


class _FlangeNode:
    """Tool-mount link: reports the arm's live FK flange pose."""

    def __init__(self):
        self.bridge = None

    def getField(self, n):
        return _Name("flange") if n == "name" else None

    def _pose(self):
        ik = self.bridge.cfg["ik"]
        return br.forward_kinematics_pose(ik["chain"], self.bridge._read_q(),
                                          ik["tcp_offset"])

    def getPosition(self):
        p, _R = self._pose()
        return list(p)

    def getOrientation(self):
        _p, R = self._pose()
        return [R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]]


class _MF:
    def __init__(self, nodes):
        self.nodes = nodes

    def getCount(self):
        return len(self.nodes)

    def getMFNode(self, i):
        return self.nodes[i]


class _BaseNode:
    def __init__(self, links):
        self._mf = _MF(links)

    def getField(self, n):
        if n == "name":
            return _Name("omniarm6")
        if n == "children":
            return self._mf
        return None

    def getPosition(self):
        return [0.0, 0.0, 0.0]

    def getOrientation(self):
        return [1, 0, 0, 0, 1, 0, 0, 0, 1]


class _Sensor:
    def __init__(self, motor):
        self._m = motor

    def enable(self, _ts):
        pass

    def getValue(self):
        return self._m.position


class _Motor:
    def __init__(self):
        self.position = 0.0
        self._s = _Sensor(self)

    def getMaxVelocity(self):
        return 1.0

    def setVelocity(self, _v):
        pass

    def setPosition(self, q):
        self.position = q

    def getPositionSensor(self):
        return self._s


class _Supervisor:
    def __init__(self, cubes):
        self._time = 0.0
        self._motors = {}
        self._root = _BaseNode(cubes)        # root.children walked for graspables
        self.flange = _FlangeNode()
        self._self = _BaseNode([self.flange])  # getSelf subtree -> mount link

    def getBasicTimeStep(self):
        return 16.0

    def getDevice(self, name):
        if name.endswith("_motor"):
            base = name[: -len("_motor")]
            self._motors.setdefault(base, _Motor())
            return self._motors[base]
        return None

    def getSelf(self):
        return self._self

    def getFromDef(self, _n):
        return None

    def getRoot(self):
        return self._root

    def getTime(self):
        return self._time


def _install_controller():
    # The bridge imports `from omnisim import Supervisor`; `omnisim` is itself a
    # shim over `controller`. Stub both so this runs outside the simulator.
    for name in ("controller", "omnisim"):
        mod = types.ModuleType(name)
        mod.Supervisor = _Supervisor
        mod.Robot = _Supervisor
        sys.modules[name] = mod


def _run(bridge, robot, ticks):
    for _ in range(ticks):
        robot._time += 0.016
        bridge.tick(robot.getTime())


def main() -> int:
    global br
    print("[pick test] stubbing OmniSim, fake scene with 3 cubes + FK flange")
    _install_controller()
    import omnilink_arm_bridge as _br
    br = _br

    cubes = [
        _CubeNode("GRASP_CUBE_RED", [0.45, -0.12, 0.025]),
        _CubeNode("GRASP_CUBE_BLUE", [0.45, 0.0, 0.025]),
        _CubeNode("GRASP_CUBE_GREEN", [0.45, 0.12, 0.025]),
    ]
    red, blue, green = cubes
    robot = _Supervisor(cubes)
    cfg = br.get_config("omniarm6")
    bridge = br.ArmBridge(robot, cfg, "omniarm6", gripper_id="robotiq_2f85")
    assert bridge.effector is not None, "robotiq_2f85 effector should attach"
    assert bridge.gripper_anchor is robot.flange, "flange should be the grasp anchor"
    robot.flange.bridge = bridge   # wire live FK now that the bridge exists

    # ── pick the RED cube ────────────────────────────────────────────
    res = bridge.act_pick("red")
    print("  act_pick ->", {k: res[k] for k in res if k != "pos"})
    assert res.get("target") == "GRASP_CUBE_RED", f"wrong target: {res}"
    assert bridge.motion[0] == "sequence", "pick should start a sequence"

    _run(bridge, robot, 400)   # play out the whole pick sequence

    assert bridge.held_node is red, \
        f"RED cube should be welded to the tool, held={bridge.held_node}"
    red_z = red._t.getSFVec3f()[2]
    print(f"  after pick: held=RED, red z={red_z:.3f} (started 0.025)")
    assert red_z > 0.12, f"RED cube should have been lifted, z={red_z}"
    assert blue._t.getSFVec3f()[2] < 0.05 and green._t.getSFVec3f()[2] < 0.05, \
        "only the RED cube should move"

    # ── place it on the drop zone ────────────────────────────────────
    res = bridge.act_place()
    print("  act_place ->", res)
    assert bridge.motion[0] == "sequence", "place should start a sequence"

    _run(bridge, robot, 400)

    assert bridge.held_node is None, "cube should be released after place"
    rx, ry, rz = red._t.getSFVec3f()
    drop = cfg["drop_zone"]
    print(f"  after place: red at ({rx:.2f}, {ry:.2f}, {rz:.2f}); drop {drop}")
    assert math.hypot(rx - drop[0], ry - drop[1]) < 0.12, \
        f"cube should land near the drop zone, got ({rx:.2f}, {ry:.2f})"
    assert red.reset_count > 0, "release should resetPhysics so the cube falls"

    # ── intent router routes pick/place too ──────────────────────────
    out = br.IntentRouter(bridge).dispatch("pick up the green cube")
    assert out["tools"] and out["tools"][0][0] == "pick", out
    _run(bridge, robot, 400)
    assert bridge.held_node is green, "router pick should grab GREEN"
    out = br.IntentRouter(bridge).dispatch("put it down")
    assert out["tools"] and out["tools"][0][0] == "place", out
    _run(bridge, robot, 400)   # finish placing GREEN
    print("  router: 'pick up the green cube' + 'put it down' OK")

    # ── a bare "open the gripper" releases the held cube so it falls ──
    bridge.act_pick("red")
    _run(bridge, robot, 400)
    held = bridge.held_node
    assert held is not None, "should be holding a cube before open"
    rc = held.reset_count
    out = bridge.act_open_gripper()
    assert bridge.held_node is None, "open_gripper must release the held cube"
    assert out.get("released"), f"open should report a release: {out}"
    assert held.reset_count > rc, "open must resetPhysics so the cube drops"
    print("  open_gripper releases the held cube OK")

    print("[pick test] PASS")
    return 0


def _inert_gripper_cannot_claim_a_hold() -> None:
    """A gripper that resolved NO motors must not report `holding: True`.

    ⚠ THIS IS A REGRESSION TEST FOR A SHIPPED FAKE. `ROBOTIQ_2F140` named a
    motor called `finger_joint`, no URDF in this tree defines a joint by that
    name, every lookup returned None, `_apply` skipped them all -- and
    `grasp()` still answered `holding: True`. The config is fixed, but the
    CLASS of bug is the dangerous part (missing devices are deliberately
    tolerated, so any future config can re-enter it), and the sim is about to
    be compared against real hardware where a phantom grasp is not a cosmetic
    difference. Guard the behaviour, not just the one config.
    """
    import gripper_effectors as ge

    class _NoDevices:
        def getBasicTimeStep(self): return 32
        def getDevice(self, _n): return None

    g = ge.ParallelFingerGripper(_NoDevices(), {
        "model": "phantom", "kind": "parallel", "motors": ["nope_joint"],
        "open_q": [0.0], "close_q": [0.7], "max_width": 0.140})
    r = g.grasp()
    assert r.get("error") == "no_motors_resolved", \
        "an inert gripper must REFUSE the claim, got %r" % (r,)
    assert r["holding"] is False, "an inert gripper must not report holding"
    assert g.state()["holding"] is False, "state() must agree"
    assert g.state().get("actuable") is False
    assert "nope_joint" in g.state().get("missing_motors", [])
    print("  inert gripper: grasp() refuses, holding stays False OK")

    # And the repaired 2F-140 config must name joints that the reference URDF
    # actually defines -- the other half of the same failure.
    import os as _os
    import re as _re
    from _gripper_configs import ROBOTIQ_2F140, ROBOTIQ_2F140_GRIP
    urdf = _os.path.normpath(_os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)),
        "..", "..", "..", "..", "robots", "omnisim", "omniarm6",
        "omniarm6_2f140_grip.urdf"))
    if _os.path.exists(urdf):
        names = set(_re.findall(r'<joint name="([^"]+)"', open(urdf).read()))
        for cfg_name, cfg in (("ROBOTIQ_2F140", ROBOTIQ_2F140),
                              ("ROBOTIQ_2F140_GRIP", ROBOTIQ_2F140_GRIP)):
            for m in cfg["motors"]:
                assert m in names, \
                    ("%s names motor %r, which %s does not define -- this is "
                     "the finger_joint bug again"
                     % (cfg_name, m, _os.path.basename(urdf)))
        print("  2F-140 configs name joints the reference URDF defines OK")
    else:
        print("  (reference URDF not found; joint-name check skipped)")


if __name__ == "__main__":
    rc = main()
    if rc == 0:
        _inert_gripper_cannot_claim_a_hold()
        print("[gripper honesty] PASS")
    raise SystemExit(rc)
