# ROS 2 and OmniSim: a deliberate non-goal, and how to bridge anyway

**2026-07-10.** This doc makes an ambiguous situation explicit. OmniSim has **no
first-party ROS 2 bridge**, and that is a *decision*, not an unfinished feature. This page
states the decision, says honestly what that costs you, tells you when to use a different
simulator, and gives a working recipe for the cases where you genuinely need ROS 2 on top
of OmniSim.

## The decision

**OmniSim's agent interface is HTTP/JSON, not ROS.** The [OmniSim Wire
Protocol](../../PROTOCOL.md) — the harness (`:6789`), the per-robot bridges (`:8765`), the
capture service (`:6791`), the MCP server ([`packages/omnisim-mcp/`](../../packages/omnisim-mcp/))
— is the intentional, first-class way to drive the simulator. The whole product thesis is
*agent-native*: an LLM agent authors a world and drives a robot over a small, versioned HTTP
surface it can reason about, without a ROS graph, a middleware, or a message-type
compilation step. That is a different bet from the ROS ecosystem, made on purpose.

So we do not ship, maintain, or test a ROS 2 bridge, and the comparison docs
([simulator-comparison.md](simulator-comparison.md)) mark the ROS row as a loss, plainly.

## What that actually costs you

Be clear-eyed about the gap:

- **No `webots_ros2`.** Although OmniSim is a Webots fork, the upstream `webots_ros2`
  packages (the ROS 2 ↔ Webots bridge Cyberbotics maintains, and headlined again in Webots
  R2025a) are **not carried in this tree**. There is no `rclcpp`/`rclpy`, no `ros2_control`
  plugin, no live topic/service/action surface.
- **OmniSim is absent from the `ros2_control` simulator registry** — where Gazebo and
  MuJoCo are first-party-hosted and even Isaac Sim and Webots appear as community entries.
- **What *is* inherited is assets, not integration:** ROS-derived robot models (Husarion
  Rosbot / Rosbot XL, Robotis TurtleBot3) and sensor `xacro`/URDF descriptions (velodyne,
  sick, MPU-9250 IMU, smartmicro radar) came along with the fork. They describe robots; they
  do not connect OmniSim to a ROS graph.

## If you need ROS 2, pick the right path

### You want a ROS-native workflow → use Gazebo (or upstream Webots)
For a lab whose stack *is* ROS 2 — `ros2_control`, Nav2, MoveIt, rosbag, the whole graph —
**[Gazebo](https://gazebosim.org) is the correct tool**, and we recommend it without
reservation: its `ros2_control` integration is hosted first-party by the ros-controls
organization. If you specifically want the Webots robot/sensor models with a maintained ROS
bridge, **upstream [Webots](https://github.com/cyberbotics/webots) + `webots_ros2`** is the
supported path (OmniSim is a divergent fork, so `.wbt` compatibility is best-effort — expect
to reconcile the OmniSim-only nodes like `URDFRobot` and the `omnisim://` URL scheme).

### You want OmniSim's engine *and* a few ROS 2 topics → bridge externally
OmniSim's HTTP surface is easy to adapt into ROS 2 from *outside* the simulator, with no
engine changes. The pattern: a small ROS 2 node that polls (or drives) the OmniSim bridge
over HTTP and republishes as ROS messages. This keeps OmniSim agent-native while giving a
ROS consumer what it expects.

```python
# omnisim_ros2_shim.py — a minimal external ROS 2 ↔ OmniSim bridge.
# Runs in your ROS 2 environment (rclpy); talks to a running OmniSim per-robot
# bridge (PROTOCOL.md §robot_bridge, default 127.0.0.1:8765) over plain HTTP.
# This is a SKETCH of the pattern, not a shipped package — OmniSim does not
# depend on rclpy and does not test this path.
import json, urllib.request
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

OMNISIM = "http://127.0.0.1:8765"   # the robot bridge the world's controller starts

def _post(path, body):
    req = urllib.request.Request(OMNISIM + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())

class OmniSimShim(Node):
    def __init__(self):
        super().__init__("omnisim_shim")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        # command path: ROS topic -> OmniSim HTTP
        self.create_subscription(JointState, "joint_command", self.on_command, 10)
        # state path: poll OmniSim HTTP -> ROS topic at 50 Hz
        self.create_timer(0.02, self.publish_state)

    def publish_state(self):
        st = _post("/get_robot_state", {})          # {q: [...], tcp: [...], ...}
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = list(st.get("q", []))
        self.pub.publish(msg)

    def on_command(self, msg: JointState):
        _post("/set_joint_positions", {"q": list(msg.position)})

def main():
    rclpy.init(); rclpy.spin(OmniSimShim()); rclpy.shutdown()

if __name__ == "__main__":
    main()
```

The endpoint names above follow the per-robot-class surfaces in
[PROTOCOL.md](../../PROTOCOL.md) (arm bridge shown; mobile / quadruped / flying bridges
expose the analogous `set_*` / `get_*` calls). Because the shim lives entirely in *your*
ROS environment, it carries all the ROS dependencies and OmniSim stays dependency-free — the
same separation the [`agents/bridges/`](../../agents/bridges/) real-robot starter kit uses.

For richer perception/TF, extend the same node: republish `/robot/<def>/joints` and
`/sim/contacts` from the harness (`:6789`), or map a robot's camera bridge output to
`sensor_msgs/Image`.

## If this decision is wrong

If OmniSim's audience turns out to be ROS-first and this non-goal is blocking adoption, the
reversible path is to **port upstream `webots_ros2`** — it already targets `.wbt` worlds and
the Webots controller protocol OmniSim inherited, so it is the natural starting point rather
than a from-scratch bridge. That would be a real, multi-week workstream and a change of
product direction; this doc is the record that we chose *not* to do it, and why.

## See also

- [PROTOCOL.md](../../PROTOCOL.md) — the HTTP/JSON surface that replaces ROS here.
- [packages/omnisim-mcp/](../../packages/omnisim-mcp/) — the MCP server over that surface.
- [simulator-comparison.md](simulator-comparison.md) §4 — the ROS loss stated in context.
- [AGENTS.md §4–§5](../../AGENTS.md) — driving robots and worlds over the bridge/harness.
