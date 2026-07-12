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

"""ref_obs.py -- reference-in-observation block for the universal motion tracker.

Phase 1 of the universal-tracker plan. Today the ghost
enters the policy IMPLICITLY (phase clock + corridor feedforward + reward), so one policy tracks one
motion. A universal tracker must condition on WHAT to track -> put the TARGET POSE in the observation.
This module is the standalone, OFFLINE-TESTABLE core of that change, kept separate so the crown-jewel
trainer (g1_walk_recipe.py) edit is tiny and gated.

Gated by REF_OBS=1 in the trainer. Default OFF -> the trainer's obs and the champion are byte-for-byte
unchanged. The reference block is APPENDED to the END of the existing obs vector, so an old checkpoint's
obs is a strict PREFIX of the new one -- which is what makes the warm-start weight surgery
(`load_ref_expand`) exact: copy the old first-layer input columns, zero the new ref columns, and
iteration-0 behaviour is identical to the loaded policy.

Design notes:
- The reference is expressed as target-minus-current DELTAS for the tracked joints at K future
  lookaheads (+ the target base roll/pitch), i.e. "where should I be a few frames from now". Deltas
  (not absolutes) keep the block zero-mean and same-scale as the existing jpos term.
- `build_ref_block` is generic over the target table width D: D=12 for the legs-only Phase-1
  regression (target = ghost_leg_t), D=23 for the Phase-2 whole-body corpus (target = a wb table).
"""
import os


def ref_params():
    """(on, K, stride) from env. K = number of future lookaheads, stride = bins between them."""
    on = os.environ.get("REF_OBS", "0") == "1"
    K = int(os.environ.get("REF_OBS_K", "3"))
    stride = int(os.environ.get("REF_OBS_STRIDE", "4"))
    return on, K, stride


def ref_block_dim(tgt_width, K, use_att):
    """Width the reference block adds to OBS_DIM."""
    return K * int(tgt_width) + (2 if use_att else 0)


def build_ref_block(ghost_tgt_t, att_t, gb, cur_pos, K, stride, NB):
    """Reference block for one obs step.

    ghost_tgt_t : (NB, D) target joint table (e.g. ghost_leg_t, or a whole-body table).
    att_t       : (NB, 2) target base [roll, pitch] table, or None.
    gb          : (Kenv,) long -- current phase bin per env.
    cur_pos     : (Kenv, D) -- current tracked-joint positions (same D/order as ghost_tgt_t).
    Returns (Kenv, K*D [+2]) = per-lookahead (target - current) deltas, then target att.
    """
    import torch
    cols = []
    for k in range(1, K + 1):
        bin_k = (gb + k * stride) % NB
        cols.append(ghost_tgt_t[bin_k] - cur_pos)          # target delta at lookahead k
    if att_t is not None:
        bin_1 = (gb + stride) % NB
        cols.append(att_t[bin_1])                          # nearest-lookahead target roll,pitch
    return torch.cat(cols, dim=1)


def load_ref_expand(net, path, device):
    """Load a checkpoint into `net`, zero-expanding any first-layer whose input grew (REF_OBS added
    columns at the END of the obs). Handles the MLP actor (`pi.0.weight`), the recurrent actor
    (`enc.weight`), and the privileged critic (`vf.0.weight`). Copies the old input columns and zeros
    the new ones so iteration-0 output == the loaded policy. Returns True if any layer was expanded.

    If REF_OBS is off (dims already match) this is a plain, exact load (returns False). Raises on a
    genuinely incompatible checkpoint (e.g. wrong ACT_DIM) instead of silently masking it.
    """
    import torch
    sd = torch.load(path, map_location=device)
    cur = net.state_dict()
    expanded = False
    for key in ("pi.0.weight", "enc.weight", "vf.0.weight"):
        if key in sd and key in cur:
            old_in = sd[key].shape[1]
            new_in = cur[key].shape[1]
            if new_in > old_in and sd[key].shape[0] == cur[key].shape[0]:
                w = cur[key].detach().clone()
                w.zero_()
                w[:, :old_in] = sd[key]
                sd[key] = w
                expanded = True
    net.load_state_dict(sd)   # non-strict not needed: every key present, only widened first layers
    return expanded
