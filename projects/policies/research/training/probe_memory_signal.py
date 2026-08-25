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

"""Does HISTORY predict the robot's joint response better than the present?

If yes, a memory policy (frame stack / RNN) has signal to exploit for
lag-compensation; if no, recurrence is pointless for this robot.

Method: ridge-regress next-tick achieved joint position q[t+1] from
  A) present only:  [q[t], qd[t], cmd[t]]
  B) history K=4:   [q[t-3..t], qd[t-3..t], cmd[t-3..t]]
on logged deploy traces (G1_TRACE_REF csv: t, bx, phase, q[13], model[13],
cmd[13]). Report per-joint RMSE improvement of B over A, train/test split
by time (first 70% train).
"""
import sys

import numpy as np

K = 4
JN = {0: "hip_p", 3: "knee", 4: "ankle_p"}


def ridge_fit_predict(Xtr, Ytr, Xte, lam=1e-3):
    X1 = np.c_[Xtr, np.ones(len(Xtr))]
    W = np.linalg.solve(X1.T @ X1 + lam * np.eye(X1.shape[1]), X1.T @ Ytr)
    return np.c_[Xte, np.ones(len(Xte))] @ W


for path in sys.argv[1:]:
    rows = np.loadtxt(path, delimiter=",")
    t = rows[:, 0]
    q = rows[:, 3:16]
    cmd = rows[:, 29:42]
    # steady walking only
    m = t > t[0] + 6.0
    q, cmd = q[m], cmd[m]
    dt = 0.016
    qd = np.vstack([np.zeros((1, 13)), np.diff(q, axis=0) / dt])

    print(f"\n{path.split(chr(92))[-1]}  ({len(q)} ticks)")
    for j, nm in JN.items():
        feats, feats_h, targ = [], [], []
        for i in range(K, len(q) - 1):
            feats.append([q[i, j], qd[i, j], cmd[i, j]])
            h = []
            for k in range(K):
                h += [q[i - k, j], qd[i - k, j], cmd[i - k, j]]
            feats_h.append(h)
            targ.append(q[i + 1, j])
        A = np.array(feats); B = np.array(feats_h); Y = np.array(targ)
        ntr = int(0.7 * len(Y))
        pA = ridge_fit_predict(A[:ntr], Y[:ntr], A[ntr:])
        pB = ridge_fit_predict(B[:ntr], Y[:ntr], B[ntr:])
        eA = np.sqrt(np.mean((pA - Y[ntr:]) ** 2))
        eB = np.sqrt(np.mean((pB - Y[ntr:]) ** 2))
        gain = (1 - eB / eA) * 100
        print(f"  {nm:7s}: present-only RMSE {np.degrees(eA):6.3f} deg | "
              f"history-K4 {np.degrees(eB):6.3f} deg | "
              f"memory gain {gain:+5.1f}%")
