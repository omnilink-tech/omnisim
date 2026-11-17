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

"""BATON hosts -- the robot-side adapters that let one arbiter drive different bodies.

A *host* answers the only three questions BATON asks (see baton.py):

    channels        which reference channels this robot has (a quad has no elbow)
    phase()         the gait clock the support gate reads
    write_tables()  where the blended references go, in THIS runtime
    act()           how to evaluate one specialist, in THIS runtime (torch? onnx? nothing?)
    reset_hidden()  cold handover, for whatever "hidden" means here

`InEngineHost` is the humanoid one: it runs inside the engine's physics tick, owns torch, and
writes the recipe's `world._wr_*` reference attributes. It is a faithful adapter -- the writes
below are byte-for-byte the ones that used to be inlined in the arbiter, which is why the G1
stays bit-identical.

The quadruped host is NOT here: it lives in the Go2's ONNX *controller*
(`research/controllers/go2_shadow_deploy/`), a separate process with no `world`, no torch and
no crane. That is the point. If BATON needed to change to accommodate it, BATON would not be a
library -- and it did not change.
"""

from __future__ import annotations

import numpy as np


class InEngineHost:
    """Humanoid, in-engine (the g1_walk_recipe deploy pymod). Owns torch; writes `world._wr_*`."""

    # the humanoid's reference channels, and where each one lives in a ghost lut.
    # `glut`/`ref` = leg track, `att` = base attitude, `arm`/`elb` = the arm ghost.
    # A quadruped declares a shorter map (no elbow); BATON blends whatever it is told.
    LUT_KEYS = {"glut": "leg_lut", "arm": "arm_lut", "elb": "elbow_lut",
                "att": "att_lut", "ref": "leg_lut"}
    channels = tuple(LUT_KEYS)

    def tables_from_lut(self, gd: dict, vx: float) -> dict:
        from projects.policies.training import baton as BATON
        return BATON.lut_tables(gd, vx, self.LUT_KEYS)

    def __init__(self, world, torch, make_net, log):
        self.world = world
        self.torch = torch
        self.make_net = make_net
        self._log = log

    # ---- the interface -----------------------------------------------------
    def phase(self) -> float:
        return float(self.world._wr_phase)

    def log(self, msg: str) -> None:
        self._log(self.world, msg)

    def load_policy(self, ckpt):
        """-> (net, hidden). The recipe's actor-critic + a fresh LSTM state."""
        net = self.make_net()
        net.load_state_dict(self.torch.load(ckpt, map_location="cpu"))
        net.eval()
        return net, net.init_hidden(1, "cpu")

    def act(self, sp, obs):
        with self.torch.no_grad():
            mu, _, sp.hidden = sp.policy.act(self.torch.from_numpy(obs[None, :]), sp.hidden)
        return np.clip(mu.numpy()[0], -1, 1)

    def reset_hidden(self, sp) -> None:
        # the PRIMARY specialist's recurrent state is the recipe's own `world._wr_h`;
        # every other specialist carries its own.
        if sp.primary:
            self.world._wr_h = self.world._wr_net.init_hidden(1, "cpu")
        elif sp.policy is not None:
            sp.hidden = sp.policy.init_hidden(1, "cpu")

    def write_tables(self, eff: dict) -> None:
        """Push the blended references where the recipe's corridor / REF_OBS / harness read them.

        Byte-for-byte the writes the arbiter used to inline -- including `_wr_ffdq = None`
        (specialist luts carry no feedforward table) and the keep-what-you-had fallback when a
        specialist does not define a channel.
        """
        w = self.world
        w._wr_glut = eff["glut"]
        w._wr_ffdq = None
        for attr, key in (("_wr_armlut", "arm"), ("_wr_elblut", "elb"),
                          ("_wr_refatt", "att"), ("_wr_reftgt", "ref")):
            v = eff.get(key)
            setattr(w, attr, v if v is not None else getattr(w, attr, None))
        w._wr_cmd = eff["vx"]

    # ---- the primary policy's own tables (the "walk" specialist) ------------
    def primary_tables(self, vx: float) -> dict:
        w = self.world

        def cp(a):
            return a.copy() if a is not None else None
        return {"glut": cp(w._wr_glut), "arm": cp(getattr(w, "_wr_armlut", None)),
                "elb": cp(getattr(w, "_wr_elblut", None)),
                "att": cp(getattr(w, "_wr_refatt", None)),
                "ref": cp(getattr(w, "_wr_reftgt", None)), "vx": vx}
