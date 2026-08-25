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

"""Offline unit test for ref_obs.py (Phase 1 of the community-tracker plan).

No sim / no GPU needed. Proves the three things the trainer integration relies on:
  1. ref_block_dim / build_ref_block produce the right shape and values.
  2. REF_OBS OFF is a byte-identical no-op (backward compat).
  3. load_ref_expand's zero-column warm-start makes iteration-0 output IDENTICAL to the loaded
     champion -- the property that lets us fine-tune instead of retraining from scratch.

Run:  python projects/policies/training/test_ref_obs.py
"""
import os
import sys
import tempfile

import pytest

import torch
import torch.nn as nn

_here = os.path.abspath(__file__)
_repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_here))))  # .../omnisim
sys.path.insert(0, _repo)
from projects.policies.training import ref_obs


class MockAC(nn.Module):
    """Mirrors the real AC's parameter KEY NAMES (pi.0.weight, vf.0.weight[, enc.weight])."""
    def __init__(self, obs, priv, act, arch="mlp", hid=8):
        super().__init__()
        if arch in ("lstm", "gru"):
            self.enc = nn.Linear(obs, hid)
            self.pi = nn.Sequential(nn.Linear(hid, act))   # (rnn omitted; irrelevant to the test)
        else:
            self.pi = nn.Sequential(nn.Linear(obs, hid), nn.ELU(), nn.Linear(hid, act))
        self.vf = nn.Sequential(nn.Linear(obs + priv, hid), nn.ELU(), nn.Linear(hid, 1))


def test_block_dim_and_values():
    NB, D, K, stride = 8, 3, 2, 1
    ghost = torch.arange(NB * D, dtype=torch.float32).reshape(NB, D)  # distinct rows
    att = torch.arange(NB * 2, dtype=torch.float32).reshape(NB, 2) + 100.0
    gb = torch.tensor([0, 4])
    cur = torch.zeros(2, D)

    assert ref_obs.ref_block_dim(D, K, use_att=True) == K * D + 2
    assert ref_obs.ref_block_dim(D, K, use_att=False) == K * D

    blk = ref_obs.build_ref_block(ghost, att, gb, cur, K, stride, NB)
    assert blk.shape == (2, K * D + 2), blk.shape
    # env0 gb=0: lookahead1 = ghost[1]-0, lookahead2 = ghost[2]-0, att = att[1]
    assert torch.allclose(blk[0, 0:D], ghost[1])
    assert torch.allclose(blk[0, D:2 * D], ghost[2])
    assert torch.allclose(blk[0, 2 * D:], att[1])
    # env1 gb=4: lookahead1 = ghost[5], lookahead2 = ghost[6], att = att[5]; and the mod wraps
    assert torch.allclose(blk[1, 0:D], ghost[5])
    assert torch.allclose(blk[1, D:2 * D], ghost[6])
    assert torch.allclose(blk[1, 2 * D:], att[5])
    # wrap-around: gb=7, stride 1, k up to 2 -> bins 0,1
    gbw = torch.tensor([7]); curw = torch.zeros(1, D)
    blkw = ref_obs.build_ref_block(ghost, None, gbw, curw, K, stride, NB)
    assert torch.allclose(blkw[0, 0:D], ghost[0]) and torch.allclose(blkw[0, D:2 * D], ghost[1])
    print("  [ok] ref_block_dim + build_ref_block shape/values/wrap")


def _pi_forward(net, x):
    return net.pi(x) if not hasattr(net, "enc") else net.pi(torch.tanh(net.enc(x)))


# MockAC branches on arch: "lstm"/"gru" build an `enc` layer, anything else is
# a plain mlp -- and the assertions below key off that split. The parameter was
# declared but never parametrized, so pytest reported "fixture 'arch' not
# found" and this test had never actually run.
@pytest.mark.parametrize("arch", ["mlp", "lstm", "gru"])
def test_warm_start_identity(arch):
    OBS, PRIV, ACT, REF = 49, 7, 12, 38
    torch.manual_seed(0)
    small = MockAC(OBS, PRIV, ACT, arch=arch)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "champ.pt")
        torch.save(small.state_dict(), p)

        # (a) expand into a bigger-obs net: REF_OBS appended 38 cols at the END
        big = MockAC(OBS + REF, PRIV, ACT, arch=arch)
        changed = ref_obs.load_ref_expand(big, p, "cpu")
        assert changed is True, "expected expansion"
        fk = "enc.weight" if arch != "mlp" else "pi.0.weight"
        bw = dict(big.named_parameters())[fk]
        sw = small.state_dict()[fk]
        assert torch.allclose(bw[:, :OBS], sw), "old input columns not copied"
        assert torch.count_nonzero(bw[:, OBS:]) == 0, "new ref columns must be zero"
        vk = "vf.0.weight"
        assert torch.allclose(dict(big.named_parameters())[vk][:, :OBS + PRIV], small.state_dict()[vk])
        assert torch.count_nonzero(dict(big.named_parameters())[vk][:, OBS + PRIV:]) == 0

        # (b) DECISIVE: iteration-0 policy output identical on old obs + zero ref block
        x = torch.randn(5, OBS)
        x_big = torch.cat([x, torch.zeros(5, REF)], 1)
        assert torch.allclose(_pi_forward(small, x), _pi_forward(big, x_big), atol=1e-6), \
            "iter-0 output diverged from champion"

        # (c) REF_OBS off (no size change) -> plain exact load, no expansion
        same = MockAC(OBS, PRIV, ACT, arch=arch)
        changed2 = ref_obs.load_ref_expand(same, p, "cpu")
        assert changed2 is False
        assert torch.allclose(_pi_forward(same, x), _pi_forward(small, x), atol=1e-6)
    print(f"  [ok] warm-start zero-expansion is iter-0 identical  (arch={arch})")


def test_ref_params_default_off():
    for k in ("REF_OBS", "REF_OBS_K", "REF_OBS_STRIDE"):
        os.environ.pop(k, None)
    on, K, stride = ref_obs.ref_params()
    assert on is False and K == 3 and stride == 4, (on, K, stride)
    print("  [ok] REF_OBS defaults OFF (backward compatible)")


if __name__ == "__main__":
    print("ref_obs offline tests:")
    test_ref_params_default_off()
    test_block_dim_and_values()
    test_warm_start_identity("mlp")
    test_warm_start_identity("lstm")
    print("ALL PASSED")
