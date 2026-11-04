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

"""Virtual GRBL controller driving a simulated 3-axis gantry.

The GRBL emulator (projects/manufacturing/cnc) runs on the simulator clock and
listens on a TCP port; any GRBL sender can connect. Each step it turns GRBL's
belief into a physical rail command, and the slider motors follow it as
position servos (Newton runs motorised sliders as position servos; a
velocity command left them standing). PositionSensors close the loop: limit
switches, status `Pn:` and `$TWIN` see where the carriage really is, not where
GRBL thinks it is.

With --stock, the tool carves a block of stock (stock.py) from the MEASURED
tool position, and the controller (a supervisor) pushes the changed heights
into the world's STOCK_GRID ElevationGrid.

With --laser <module json>, a laser replaces the cutter (projects/manufacturing/laser):
the module's GRBL settings are applied ($30/$31/$32, as gSender's laser toggle does),
GRBL's laser pin is integrated over every planner sub-step (laser.LaserOutput: M3/M4,
G0 off, PWM), and the power is written to the world's DEF STOCK MillingStock
(toolShape "laser"), which burns where the beam really goes.

controllerArgs: --machine <cfg> [--port 2323] [--host 127.0.0.1]
                [--s0 x,y,z] [--rails x,y,z] [--trace <jsonl>]
                [--stock ox,oy,oz,sx,sy,thickness,spacing --tool-d mm
                 --tool-tip x,y,z --cad-s x,y,z --axis-dirs x;y;z]
                [--laser <module json> --laser-material <name>]
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "projects/manufacturing/cnc"))
sys.path.insert(0, str(ROOT / "projects/manufacturing/core"))
sys.path.insert(0, str(ROOT / "projects/manufacturing/laser"))

from omnisim import Robot, Supervisor  # noqa: E402

import machine as machine_mod  # noqa: E402
from grbl import GrblController  # noqa: E402
from machine import AXES  # noqa: E402
from server import StreamServer  # noqa: E402
from stock import Stock, tool_tip  # noqa: E402
import tool_table as tools_mod  # noqa: E402
import cutting  # noqa: E402
from twin import Plant  # noqa: E402
from drives import FedForwardDrives  # noqa: E402

TRACE_PERIOD_S = 0.05
STOCK_FLUSH_S = 0.08     # push carved heights to the renderer at ~12 Hz of sim time
SKIN_REVEAL_MM = 0.15    # a cut this deep removes the painted skin (world.py SKIN_MM is 0.2)
# The slider motors are OmniSim position servos, driven like steppers by the shared drive
# model (projects/manufacturing/core/drives.py): the target leads the command by the
# servo's kd/ke = 0.05 s times its speed (3.3 mm at 4000 mm/min), plus this machine's
# lead trim and acceleration feed-forward below.
# Fed forward exactly, that lead left every axis AHEAD of its command by 0.95 ms x speed
# (48 um at 50 mm/s in Z, 39 um at 42 mm/s in X; measured 2026-11-01, every step): about
# half of the engine's 2 ms substep. The velocity feed-forward is shorter by that much.
VELOCITY_LEAD_TRIM_S = 0.00095
# What is left is inertia: the acceleration is fed forward too. MEASURED 2026-11-01 on the
# LongMill, every 8 ms step, with the target centred on the step it holds for (see the loop)
# and the lead trimmed: a 3D finishing staircase (Z at 2500 mm/min down and up, 90-degree
# turns into X rows, 600 mm/min plunges), worst error over the run:
#   coefficient (s^2, every axis)  0.9e-4   1e-4    1.1e-4   (Z 1.5e-4, X 1.25e-4)
#   Z (um)                         -5.1/6.1 -4.7/4.5 -8.6/7.6  -30.2/23.3
#   X (um)                         -5.5/5.1 -5.3/5.3 -9.4/9.7  -16.2/17.0
# Before the lead trim the best per-axis values (Z 2e-4, X/Y 1.25e-4) left 60-70 um at
# those corners, and a ball nose finishing against a wall cut 0.28 mm into its fillet.
# 1e-4 is drives.accel_feedforward's dt x kd / ke for this world (2 ms substeps): the
# 12.5 kg gantry is lighter than the 20 kg the servo-armature bound raises it to.
SERVO_ACCEL_S2 = (1.0e-4, 1.0e-4, 1.0e-4)
# The planner runs this many steps ahead of the physics, so each step's target can be
# centred on the step it holds for (see the loop). GRBL's belief leads the carriage by
# these 16 ms; status and $TWIN report the carriage where it really is.
PLAN_AHEAD_STEPS = 2


# The drawn tool (world.py draws one per tool table entry under TOOL_SPIN). At the real
# 18,000 rpm the bit turns 600 times per displayed frame and looks frozen or random, so it
# is drawn turning at this rate: "spindle on", not its speed. It turns only while the
# machine moves, when every frame is drawn anyway; standing still with the spindle on it
# shows a still, translucent blur sleeve (BLUR_n) and the window can hold its frames.
# MEASURED 2026-11-01 (RTX 3060 Laptop, LongMill 3D world, 30 fps window, machine still):
# spindle off 8.6 % GPU / 17.8 W; spindle on with the bit turning every frame 34.0 % / 32.3 W.
VISUAL_SPIN_RPS = 4.0
TOOL_SWAP_S = 0.4         # each half of the drawn tool change: old bit up, new bit down
TOOL_SWAP_RISE_M = 0.12   # world.py HIDDEN_RISE_M: a bit not in the spindle waits inside the router


class ToolView:
    """The bit you see: turning or blurred while the spindle runs, swapped visibly on a tool change."""

    def __init__(self, robot, table):
        spin = _find_def(robot, "TOOL_SPIN")
        self.rotation = spin.getField("rotation") if spin is not None else None
        self.tools, self.blurs = {}, {}
        for n, tool in table.items():
            base = -tool.stickout_mm / 1000.0
            node = _find_def(robot, f"TOOL_{n}")
            if node is not None:
                self.tools[n] = (node.getField("translation"), base)
            blur = _find_def(robot, f"BLUR_{n}")
            if blur is not None:
                self.blurs[n] = (blur.getField("translation"), base)
        self.angle = 0.0
        self.shown = 1 if 1 in self.tools else None
        self.blurred = None       # the tool whose blur sleeve is out, if any
        self.swap = None

    def change(self, number, t):
        if number not in self.tools or number == self.shown:
            return
        self.swap = {"old": self.shown, "new": number, "t0": t, "half": 0}

    def _blur(self, number):
        # One sleeve out at most; a field is written only when it changes, so a still
        # machine sends nothing and its frames hold.
        if number == self.blurred:
            return
        if self.blurred in self.blurs:
            tr, base = self.blurs[self.blurred]
            tr.setSFVec3f([0.0, 0.0, base + TOOL_SWAP_RISE_M])
        if number in self.blurs:
            tr, base = self.blurs[number]
            tr.setSFVec3f([0.0, 0.0, base])
        self.blurred = number

    def step(self, t, dt, spindle, moving):
        spinning = spindle != "off"
        self._blur(self.shown if spinning and self.swap is None else None)
        if self.rotation is not None and spinning and moving:
            self.angle = (self.angle + (-1.0 if spindle == "cw" else 1.0) * 2 * math.pi * VISUAL_SPIN_RPS * dt) % (2 * math.pi)
            self.rotation.setSFRotation([0.0, 0.0, 1.0, self.angle])
        if self.swap is None:
            return
        s = self.swap
        f = (t - s["t0"]) / TOOL_SWAP_S
        if s["half"] == 0:
            if s["old"] is not None:
                tr, base = self.tools[s["old"]]
                tr.setSFVec3f([0.0, 0.0, base + min(f, 1.0) * TOOL_SWAP_RISE_M])
            if f >= 1.0:
                self.shown, s["half"], s["t0"] = s["new"], 1, t
                f = 0.0
        if s["half"] == 1:
            tr, base = self.tools[s["new"]]
            tr.setSFVec3f([0.0, 0.0, base + max(0.0, 1.0 - f) * TOOL_SWAP_RISE_M])
            if f >= 1.0:
                self.swap = None


def _find_def(robot, name):
    """A DEF inside the machine (the router body nests it under the axes' Solids)."""
    for path in (name, f"CNC.GANTRY.CARRIAGE.Z.{name}"):
        node = robot.getFromDef(path)
        if node is not None:
            return node
    return None


def _triple(text):
    return [float(v) for v in text.split(",")] if text else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--machine", required=True)
    p.add_argument("--port", type=int, default=2323)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--s0")
    p.add_argument("--rails")
    p.add_argument("--trace", default=os.environ.get("OMNISIM_CNC_TWIN_TRACE",
                                                     str(ROOT / ".tmp/cnc-twin/trace.jsonl")))
    p.add_argument("--stock")
    p.add_argument("--tool-d", type=float, default=6.35)
    p.add_argument("--tools", help="tool table JSON (projects/manufacturing/cnc/tool_table.py); tool 1 is loaded")
    p.add_argument("--material", default="mdf", choices=sorted(cutting.MATERIALS),
                   help="the stock's material, for the cutting-load model (cutting.py)")
    p.add_argument("--tool-tip")
    p.add_argument("--cad-s")
    p.add_argument("--axis-dirs")
    p.add_argument("--bridge-port", type=int, help="start the OmniLink CNC bridge on this HTTP port")
    p.add_argument("--zero-above-stock", type=float,
                   help="tool tip height above the stock top at power-up: the bridge zeroes work there")
    p.add_argument("--laser", help="laser module JSON (projects/manufacturing/laser): a laser instead of the cutter")
    p.add_argument("--laser-material", default="plywood", help="the sheet's material (laser.py MATERIALS)")
    args = p.parse_args()

    robot = Supervisor() if (args.stock or args.laser) else Robot()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0
    machine_path = Path(args.machine)
    if not machine_path.is_absolute():
        machine_path = ROOT / machine_path
    m = machine_mod.load(machine_path)
    module = None
    if args.laser:
        import laser as laser_mod
        module = laser_mod.LaserModule.load(ROOT / args.laser if not Path(args.laser).is_absolute() else Path(args.laser))
        for key, value in module.grbl.items():
            m.apply_setting(key, value)
    rails = dict(zip(AXES, _triple(args.rails))) if args.rails else None
    s0 = _triple(args.s0) or [float((rails or {}).get(a, m.axes[a].travel)) / 2 for a in AXES]
    plant = Plant.build(m, initial_s=s0, rail_lengths=rails)
    ctl = GrblController(m, plant)
    tools_path = Path(args.tools) if args.tools else None
    if tools_path is not None and not tools_path.is_absolute():
        tools_path = ROOT / tools_path
    ctl.tools = tools_mod.load(tools_path) if tools_path else tools_mod.default_tools(args.tool_d)
    ctl.tool = ctl.tools[1]
    meter = cutting.LoadMeter(cutting.MATERIALS[args.material],
                              cutting.Spindle.from_dict(tools_mod.load_spindle(tools_path)))
    laser_out = laser_node = power_field = glow_field = None
    if module is not None:
        laser_out = laser_mod.LaserOutput(ctl, module)
        ctl.planner.on_substep = laser_out.substep
        grbl_sync = ctl.planner.on_sync

        def on_sync(block):
            if block.kind in ("spindle", "end"):
                laser_out.on_sync(block)
            grbl_sync(block)

        ctl.planner.on_sync = on_sync
        laser_node = robot.getFromDef("STOCK")
        if laser_node is None:
            raise SystemExit("[grbl_twin] --laser needs the world's DEF STOCK MillingStock (toolShape \"laser\")")
        power_field = laser_node.getField("laserPower")
        glow = robot.getFromDef("LASER_GLOW")
        if glow is not None:
            glow_field = glow.getField("children").getMFNode(0).getField("appearance").getSFNode().getField("transparency")
    server = StreamServer(ctl, args.host, args.port)
    port = server.start()

    motors, sensors = [], []
    for a in AXES:
        motor = robot.getDevice(f"{a}_motor")
        motor.setPosition(0.0)
        sensor = robot.getDevice(f"{a}_sensor")
        sensor.enable(step_ms)
        motors.append(motor)
        sensors.append(sensor)

    trace_path = Path(args.trace)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    trace = trace_path.open("w", encoding="utf-8", buffering=1)
    trace.write(json.dumps({"kind": "start", "machine": m.summary(), "s0": s0, "port": port,
                            "rails": {a: plant.rails[a].length for a in AXES}}) + "\n")
    print(f"[grbl_twin] virtual GRBL for {m.name} on {args.host}:{port}", flush=True)

    stock = grid = native = None
    if args.stock:
        ox, oy, oz, sx, sy, thick, spacing = _triple(args.stock)
        stock = Stock(sx, sy, thick, spacing)
        ctl.stock = stock
        node = robot.getFromDef("STOCK_GRID") or robot.getFromDef("STOCK.STOCK_GRID")
        if node is not None:
            # Fallback renderer: this controller carves and pushes ElevationGrid heights.
            grid = node.getField("height")
            skin_node = robot.getFromDef("STOCK_SKIN") or robot.getFromDef("STOCK.STOCK_SKIN")
            skin = skin_node.getField("height") if skin_node is not None else None
            skin_hidden = set()
        else:
            # The engine's MillingStock carves and draws the block at full resolution; this
            # controller's own coarse model only judges dry cuts (it knows the spindle state,
            # the engine does not) and $TWIN reports the engine's figures beside it.
            native = robot.getFromDef("STOCK")
            if native is None:
                raise SystemExit("[grbl_twin] --stock given but the world has neither DEF STOCK MillingStock "
                                 "nor DEF STOCK_GRID ElevationGrid")
            removed_field, deepest_field = native.getField("removedVolume"), native.getField("deepestCut")
        tip_cad, cad_s = _triple(args.tool_tip), _triple(args.cad_s)
        dirs = [[float(c) for c in d.split(",")] for d in args.axis_dirs.split(";")]
        print(f"[grbl_twin] stock {sx:g} x {sy:g} x {thick:g} mm at {spacing:g} mm, "
              f"tool T1 {ctl.tool.name or ctl.tool.shape}", flush=True)
        # The tool in the spindle. A longer tool's tip sits `stickout_mm` lower (tool_table.py).
        cutter = {"radius": ctl.tool.radius_mm, "stickout": 0.0}
        base_tip = native.getField("toolTip").getSFVec3f() if native is not None else None

        def tip_local(s):
            w = tool_tip(tip_cad, cad_s, dirs, s)
            return [w[0] - ox, w[1] - oy, w[2] - oz - cutter["stickout"]]

        def load_tool(tool):
            # The operator's hands: the engine's cutter becomes the new tool. The coarse
            # model here only judges dry cuts, so it keeps a flat cutter of that radius.
            cutter["radius"], cutter["stickout"] = tool.radius_mm, tool.stickout_mm
            if native is not None:
                native.getField("toolShape").setSFString(tool.shape)
                native.getField("toolDiameter").setSFFloat(tool.diameter_mm / 1000.0)
                native.getField("toolAngle").setSFFloat(math.radians(tool.angle_deg))
                native.getField("toolTip").setSFVec3f(
                    [base_tip[0], base_tip[1], base_tip[2] - tool.stickout_mm / 1000.0])
            if view is not None:
                view.change(tool.number, ctl.time)
            print(f"[grbl_twin] tool change: T{tool.number} {tool.name or tool.shape}", flush=True)

        view = ToolView(robot, ctl.tools)
        if not view.tools:
            view = None
        print(f"[grbl_twin] drawn tools: {sorted(view.tools) if view else 'none in the world'}", flush=True)
        ctl.on_tool_change = load_tool
        load_tool(ctl.tool)

    if args.bridge_port:
        # The OmniLink bridge talks to THIS controller's GRBL over TCP, exactly as it would
        # to a board. It has to start on a thread: the virtual GRBL only answers while the
        # loop below steps the simulation.
        import cnc_bridge

        def start_bridge():
            try:
                cnc_bridge.start(f"127.0.0.1:{port}", str(machine_path), args.bridge_port, args.tool_d,
                                 thick if stock is not None else None, args.zero_above_stock,
                                 name=f"cnc-{machine_path.stem}",
                                 tools_path=str(tools_path) if tools_path else None)
            except Exception as exc:  # the machine keeps running without its agent
                print(f"[grbl_twin] OmniLink bridge failed to start: {exc}", flush=True)

        threading.Thread(target=start_bridge, name="cnc-bridge-start", daemon=True).start()

    next_trace = 0.0
    events_written = 0
    drives = FedForwardDrives(3, VELOCITY_LEAD_TRIM_S, SERVO_ACCEL_S2)
    command = list(s0)
    last_tip = None
    dirty = set()
    next_flush = 0.0
    # planned[0..3] = the rail command at t(k-1), t(k), t(k+1), t(k+2), where t(k) is now.
    planned = [list(s0) for _ in range(PLAN_AHEAD_STEPS + 2)]
    if stock is None:
        view = None
    # The laser's energy per step, delayed to the step whose motion it belongs to: the tick
    # that produced planned[k+1] integrated the interval the physics now moves through.
    from collections import deque
    laser_queue = deque([0.0] * (PLAN_AHEAD_STEPS - 1))
    laser_w = 0.0
    lit = False
    while robot.step(step_ms) != -1:
        measured = [s0[i] + sensors[i].getValue() * 1000.0 for i in range(3)]
        if view is not None:
            view.step(ctl.time, dt, ctl.spindle[0],
                      moving=any(abs(planned[2][i] - planned[1][i]) > 1e-9 for i in range(3)))
        planned = planned[1:] + [ctl.tick(dt, measured_s=measured)]
        prev, command, nxt, after = planned
        if laser_out is not None:
            laser_queue.append(laser_out.take())
            power = laser_queue.popleft() / dt
            if power != laser_w:
                power_field.setSFFloat(power)
                laser_w = power
            if glow_field is not None and (power > 0) != lit:
                lit = power > 0
                glow_field.setSFFloat(0.15 if lit else 1.0)
        # The target holds for the whole coming step, so it is centred on that step.
        for i, target in enumerate(drives.targets(prev, command, nxt, after, measured, dt)):
            motors[i].setPosition((target - s0[i]) / 1000.0)
        if stock is not None:
            tip = tip_local(measured)
            if last_tip is not None:
                changed = stock.cut(last_tip, tip, cutter["radius"], spindle_on=ctl.spindle[0] != "off", t=ctl.time)
                if grid is not None:
                    dirty.update(changed.tolist())
            last_tip = tip
            # Cutting load from the material actually removed this step (the engine's
            # measurement, or this controller's coarse model on the grid fallback).
            removed = removed_field.getSFFloat() * 1e9 if native is not None else stock.removed_mm3
            meter.update(ctl.time, removed)
            overloads = len(meter.overloads)
            ctl.cutting = meter.report(ctl.spindle[1] if ctl.spindle[0] != "off" else 0.0,
                                       ctl.planner.current_feed_mm_min(), ctl.tool.diameter_mm,
                                       ctl.tool.flutes, ctl.time)
            if len(meter.overloads) > overloads:
                ctl._event("spindle_overload", **meter.overloads[-1], rated_w=meter.spindle.power_w)
            if native is not None and ctl.time >= next_flush:
                stock.engine = {"removed_mm3": round(removed_field.getSFFloat() * 1e9, 1),
                                "deepest_mm": round(deepest_field.getSFFloat() * 1000.0, 3)}
                next_flush = ctl.time + 0.25
            if dirty and ctl.time >= next_flush:
                flat, initial = stock.h.ravel(), stock.initial.ravel()
                for idx in sorted(dirty):
                    grid.setMFFloat(idx, float(flat[idx]) / 1000.0)
                    # Hide the painted skin once the cut is deeper than the skin is thick.
                    if skin is not None and idx not in skin_hidden and initial[idx] - flat[idx] > SKIN_REVEAL_MM:
                        skin.setMFFloat(idx, 0.0)
                        skin_hidden.add(idx)
                dirty.clear()
                next_flush = ctl.time + STOCK_FLUSH_S
        for event in ctl.events[events_written:]:
            trace.write(json.dumps({"kind": "event", **event}) + "\n")
        events_written = len(ctl.events)
        if ctl.time >= next_trace:
            next_trace = ctl.time + TRACE_PERIOD_S
            trace.write(json.dumps({
                "kind": "sample", "t": round(ctl.time, 4), "state": ctl.state,
                "belief_mpos": [round(v, 4) for v in ctl.planner.position],
                "command_s": [round(v, 4) for v in command],
                "measured_s": [round(v, 4) for v in measured],
                "feed": round(ctl.planner.current_feed_mm_min(), 1),
                "power_w": (ctl.cutting or {}).get("power_w"),
                **({"laser_w": round(laser_w, 3), "laser_j": round(laser_out.total_j, 3)} if laser_out else {}),
            }) + "\n")
    server.stop()
    trace.close()


if __name__ == "__main__":
    main()
