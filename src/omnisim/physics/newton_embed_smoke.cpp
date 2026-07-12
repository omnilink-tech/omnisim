// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
//
// Standalone smoke check for the embedded-CPython call path used by
// WbNewtonBackend. Mirrors the production constructor's import path
// AND, beyond that, exercises the helper-module flow that P3.1 wires
// into the backend: load `newton_runtime` into a private namespace,
// instantiate a World, add a ground plane + a sphere body, finalize,
// step under gravity for 15 ticks, read back the position. The same
// simulation reproduced out-of-process drops the sphere from z=1.5 to
// z≈1.17 over 0.25s -- this binary is the in-process equivalent.
//
// Build (PowerShell, output to user-temp because mingw can't write
// to the repo drive from this sandbox path):
//   $env:Path = "C:\msys64\mingw64\bin;C:\msys64\usr\bin;" + $env:Path
//   $py = "$env:LOCALAPPDATA/Programs/Python/Python312"
//   g++ -std=c++17 \
//     -isystem "$py/include" \
//     src/omnisim/physics/newton_embed_smoke.cpp \
//     -L "$py/libs" \
//     -lpython312 \
//     -o "$env:TEMP/newton_embed_smoke.exe"
//
// Run: & "$env:TEMP/newton_embed_smoke.exe"
//
// Expected (success): 15 lines of "step N body_q[z]=..." with z
// monotonically decreasing from 1.5 to ≈1.17, then `[smoke] PASS`.

#include <Python.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// Inline Python helper. Same source string the production
// WbNewtonBackend ships; if you edit one, edit the other.
//
// Lives in its own dictionary so the names don't pollute __main__,
// which keeps things clean if the host process is also doing other
// Python things later.
static const char *kNewtonRuntimeSource = R"PY(
import warp as wp
import newton

class World:
    def __init__(self):
        self.builder = newton.ModelBuilder()
        self.model = None
        self.solver = None
        self.state_a = None
        self.state_b = None
        self.joint_targets = {}
        self.control = None

    def add_ground_plane(self):
        self.builder.add_ground_plane()

    def add_body(self, mass, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        return self.builder.add_body(
            mass=float(mass),
            xform=wp.transform((float(x), float(y), float(z)),
                               (float(qx), float(qy), float(qz), float(qw))),
        )

    def add_shape_sphere(self, body_idx, radius):
        return self.builder.add_shape_sphere(int(body_idx), radius=float(radius))

    def add_shape_box(self, body_idx, hx, hy, hz):
        return self.builder.add_shape_box(int(body_idx),
                                          hx=float(hx), hy=float(hy), hz=float(hz))

    def add_shape_cylinder(self, body_idx, radius, half_height):
        return self.builder.add_shape_cylinder(int(body_idx),
                                               radius=float(radius),
                                               half_height=float(half_height))

    def add_shape_capsule(self, body_idx, radius, half_height):
        return self.builder.add_shape_capsule(int(body_idx),
                                              radius=float(radius),
                                              half_height=float(half_height))

    def add_joint_revolute(self, parent_idx, child_idx,
                           ax, ay, az,
                           parent_anchor_x, parent_anchor_y, parent_anchor_z,
                           child_anchor_x, child_anchor_y, child_anchor_z,
                           target_ke=0.0, target_kd=0.0):
        return self.builder.add_joint_revolute(
            parent=int(parent_idx),
            child=int(child_idx),
            axis=(float(ax), float(ay), float(az)),
            parent_xform=wp.transform(
                (float(parent_anchor_x), float(parent_anchor_y), float(parent_anchor_z)),
                (0.0, 0.0, 0.0, 1.0),
            ),
            child_xform=wp.transform(
                (float(child_anchor_x), float(child_anchor_y), float(child_anchor_z)),
                (0.0, 0.0, 0.0, 1.0),
            ),
            target_ke=float(target_ke),
            target_kd=float(target_kd),
        )

    def set_joint_target_vel(self, joint_idx, vel):
        self.joint_targets[int(joint_idx)] = float(vel)

    def finalize(self):
        self.model = self.builder.finalize()
        self.solver = newton.solvers.SolverXPBD(self.model, iterations=30)
        self.state_a = self.model.state()
        self.state_b = self.model.state()

    def step(self, dt):
        if self.joint_targets:
            if self.control is None:
                self.control = self.model.control()
            arr = self.control.joint_target_vel.numpy()
            for idx, v in self.joint_targets.items():
                if 0 <= idx < len(arr):
                    arr[idx] = v
            self.control.joint_target_vel.assign(arr)
        contacts = self.model.collide(self.state_a)
        self.solver.step(self.state_a, self.state_b, self.control, contacts, float(dt))
        self.state_a, self.state_b = self.state_b, self.state_a

    def body_xform(self, idx):
        # Newton body_q layout: [x, y, z, qx, qy, qz, qw] -- one row per body.
        q = self.state_a.body_q.numpy()[int(idx)]
        return (float(q[0]), float(q[1]), float(q[2]),
                float(q[3]), float(q[4]), float(q[5]), float(q[6]))
)PY";

static int dieOnPyErr(const char *step) {
  if (PyErr_Occurred()) {
    PyErr_PrintEx(0);
    std::fprintf(stderr, "[smoke] FAIL: %s raised\n", step);
    return 1;
  }
  return 0;
}

int main() {
  Py_InitializeEx(0);
  if (!Py_IsInitialized()) {
    std::fprintf(stderr, "[smoke] FAIL: Py_InitializeEx returned, Py_IsInitialized=false\n");
    return 1;
  }
  std::fprintf(stdout, "[smoke] Py_InitializeEx OK\n");

  // Bring up the helper module in a private namespace.
  PyObject *globals = PyDict_New();
  PyObject *builtins = PyEval_GetBuiltins();
  PyDict_SetItemString(globals, "__builtins__", builtins);

  PyObject *result = PyRun_String(kNewtonRuntimeSource, Py_file_input, globals, globals);
  if (result == nullptr) {
    PyErr_PrintEx(0);
    std::fprintf(stderr, "[smoke] FAIL: helper module exec\n");
    return 2;
  }
  Py_DECREF(result);
  std::fprintf(stdout, "[smoke] helper module exec OK\n");

  PyObject *worldClass = PyDict_GetItemString(globals, "World");  // borrowed
  if (worldClass == nullptr) {
    std::fprintf(stderr, "[smoke] FAIL: World class not found in helper\n");
    return 3;
  }

  PyObject *world = PyObject_CallObject(worldClass, nullptr);
  if (dieOnPyErr("World()") != 0)
    return 4;
  std::fprintf(stdout, "[smoke] World() OK\n");

  PyObject *r = PyObject_CallMethod(world, "add_ground_plane", nullptr);
  if (dieOnPyErr("add_ground_plane") != 0)
    return 5;
  Py_XDECREF(r);

  // mass=0.25 at (0, 0, 1.5)
  r = PyObject_CallMethod(world, "add_body", "(dddd)", 0.25, 0.0, 0.0, 1.5);
  if (dieOnPyErr("add_body") != 0)
    return 6;
  long bodyIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_body OK (idx=%ld)\n", bodyIdx);

  r = PyObject_CallMethod(world, "add_shape_sphere", "(ld)", bodyIdx, 0.12);
  if (dieOnPyErr("add_shape_sphere") != 0)
    return 7;
  Py_XDECREF(r);

  // Second body + revolute joint connecting them. Joint axis (0, 1, 0)
  // anchored at the parent's local origin and child's local origin
  // (so the hinge sits between the two bodies). With this setup, when
  // the assembly is dropped, body 0 falls + body 1 follows but can
  // rotate freely around the y axis. We don't add any shape to body 1
  // here -- the joint is enough to verify the FFI surface end-to-end.
  r = PyObject_CallMethod(world, "add_body", "(dddd)", 0.10, 0.30, 0.0, 1.5);
  if (dieOnPyErr("add_body (child)") != 0)
    return 12;
  long childIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_body child OK (idx=%ld)\n", childIdx);

  r = PyObject_CallMethod(world, "add_shape_sphere", "(ld)", childIdx, 0.06);
  if (dieOnPyErr("add_shape_sphere (child)") != 0)
    return 13;
  Py_XDECREF(r);

  r = PyObject_CallMethod(world, "add_joint_revolute", "(iiddddddddd)",
                          (int)bodyIdx, (int)childIdx,
                          0.0, 1.0, 0.0,    // axis
                          0.15, 0.0, 0.0,   // parent anchor (between the two)
                          -0.15, 0.0, 0.0); // child anchor
  if (dieOnPyErr("add_joint_revolute") != 0)
    return 14;
  long jointIdx = PyLong_AsLong(r);
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] add_joint_revolute OK (idx=%ld)\n", jointIdx);

  r = PyObject_CallMethod(world, "finalize", nullptr);
  if (dieOnPyErr("finalize") != 0)
    return 8;
  Py_XDECREF(r);
  std::fprintf(stdout, "[smoke] finalize OK\n");

  // Run 15 steps at dt=1/60. Read back z each time.
  const double dt = 1.0 / 60.0;
  for (int i = 0; i < 15; ++i) {
    r = PyObject_CallMethod(world, "step", "(d)", dt);
    if (dieOnPyErr("step") != 0)
      return 9;
    Py_XDECREF(r);

    r = PyObject_CallMethod(world, "body_xform", "(l)", bodyIdx);
    if (dieOnPyErr("body_xform") != 0)
      return 10;
    double x0 = 0, y0 = 0, z0 = 0, qx = 0, qy = 0, qz = 0, qw = 0;
    if (!PyArg_ParseTuple(r, "ddddddd", &x0, &y0, &z0, &qx, &qy, &qz, &qw)) {
      std::fprintf(stderr, "[smoke] FAIL: body_xform tuple parse\n");
      return 11;
    }
    Py_DECREF(r);

    r = PyObject_CallMethod(world, "body_xform", "(l)", childIdx);
    if (dieOnPyErr("body_xform child") != 0)
      return 15;
    double x1 = 0, y1 = 0, z1 = 0, qcx = 0, qcy = 0, qcz = 0, qcw = 0;
    if (!PyArg_ParseTuple(r, "ddddddd", &x1, &y1, &z1, &qcx, &qcy, &qcz, &qcw)) {
      std::fprintf(stderr, "[smoke] FAIL: child body_xform tuple parse\n");
      return 16;
    }
    Py_DECREF(r);

    // Distance between body centers should stay near 0.30 (initial gap)
    // throughout the simulation if the hinge is holding them together.
    const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::fprintf(stdout,
                 "[smoke] step %2d  parent.z=%.4f  child.z=%.4f  |child-parent|=%.4f\n",
                 i + 1, z0, z1, dist);
  }

  Py_DECREF(world);
  Py_DECREF(globals);

  std::fprintf(stdout, "[smoke] PASS\n");
  return 0;
}
