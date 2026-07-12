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

"""Throwaway in-engine probe: dump H1's engine DOF/joint layout once, so the H1 walk
driver maps gait slots -> newton dofs against REALITY (the SolverMuJoCo-compiled model
renames joints + reorders dofs vs the URDF). Loaded via OMNISIM_INENGINE_PYMOD."""
import numpy as np


def probe(world):
    if getattr(world, "_h1probe_done", False):
        return
    sol = getattr(world, "solver", None)
    mjm = getattr(sol, "mj_model", None)
    m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or m2nd is None:
        return
    world._h1probe_done = True
    import mujoco as mj
    m2nd = m2nd.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    d = mj.MjData(mjm); mj.mj_forward(mjm, d)
    # position actuator per joint (affine servo)
    act_of = {}
    for a in range(int(mjm.nu)):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            j = int(mjm.actuator_trnid[a, 0])
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of:
                act_of[j] = a
    pelvis_z = 0.0
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            pelvis_z = float(d.xpos[int(mjm.jnt_bodyid[j])][2]); break
    lines = ["H1 PROBE nq=%d nv=%d nu=%d njnt=%d pelvis_z=%.3f" %
             (int(mjm.nq), int(mjm.nv), int(mjm.nu), int(mjm.njnt), pelvis_z)]
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            continue
        nd = int(m2nd[j])
        bid = int(mjm.jnt_bodyid[j]); pos = np.array(d.xpos[bid], float)
        ax = np.array(mjm.jnt_axis[j], float); axdom = int(np.argmax(np.abs(ax)))
        nm = mj.mj_id2name(mjm, mj.mjtObj.mjOBJ_JOINT, j)
        below = pos[2] < pelvis_z - 0.02
        lines.append("nd=%2d j=%2d name=%-22s ax=%s axdom=%d pos=(%.2f,%.2f,%.2f) %s act=%s qadr=%d"
                     % (nd, j, nm, np.round(ax, 1).tolist(), axdom, pos[0], pos[1], pos[2],
                        "LEG" if below else "arm/waist", act_of.get(j, "-"), int(mjm.jnt_qposadr[j])))
    msg = "\n".join(lines)
    try:
        world._mpc_log("h1probe:\n" + msg)
    except Exception:
        pass
    import sys
    sys.stderr.write("[h1probe]\n" + msg + "\n")
    # also dump to a file for sure
    try:
        import os
        with open(os.path.join(os.environ.get("OMNISIM_HOME", "."), "_scratch", "h1_probe.txt"), "w") as f:
            f.write(msg + "\n")
    except Exception:
        pass
