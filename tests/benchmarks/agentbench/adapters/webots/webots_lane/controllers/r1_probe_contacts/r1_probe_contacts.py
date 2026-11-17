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

"""Bring-up probe: can upstream name a robot<->STATIC-obstacle contact?

The Webots recorder pairs contact points ACROSS QUERIES, and today it only
queries Robot nodes -- so a robot/obstacle contact has no second participant
and R1.5 can never count one. Whether that is fixable by simply querying the
obstacle too depends on something undocumented: whether
Node.getContactPoints() answers for a Solid with NO Physics node (no ODE rigid
body). This measures it.
"""
from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
robot = sup.getFromDef("ROBOT")
obstacles = [(n, sup.getFromDef(n)) for n in
             ("OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3")]
arena = sup.getFromDef("RECT_ARENA")
print("[cprobe] robot=%s obstacles=%s" % (robot is not None,
                                          [n for n, o in obstacles
                                           if o is not None]), flush=True)
step = 0
while sup.step(dt) != -1 and sup.getTime() < 8.0:
    step += 1
    if step % 10:
        continue
    rp = robot.getContactPoints(True) or []
    line = ["t=%.2f robot_pts=%d" % (sup.getTime(), len(rp))]
    if rp:
        line.append("first=(%.3f, %.3f, %.3f) node_id=%s"
                    % (rp[0].getPoint()[0], rp[0].getPoint()[1],
                       rp[0].getPoint()[2], rp[0].getNodeId()))
    for name, node in obstacles:
        if node is None:
            continue
        try:
            op = node.getContactPoints(True) or []
        except Exception as exc:                      # noqa: BLE001
            line.append("%s=EXC(%r)" % (name, exc))
            continue
        line.append("%s=%d" % (name, len(op)))
        if op:
            line.append("  %s_first=(%.3f, %.3f, %.3f)"
                        % (name, op[0].getPoint()[0], op[0].getPoint()[1],
                           op[0].getPoint()[2]))
    print("[cprobe] " + " ".join(line), flush=True)
print("[cprobe] done", flush=True)
sup.simulationQuit(0)
