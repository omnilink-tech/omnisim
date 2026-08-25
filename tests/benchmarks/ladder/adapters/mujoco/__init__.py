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

"""The **MuJoCo / MJX column** of the capability ladder.

MuJoCo is the roster's strongest adversary above T2 and it is free and local
(``docs/developer/capability-ladder-plan.md`` §4.5, §1 adverse outcome 3). This
package is that column's foundation: the URDF path, a scripted T1 runner, and
the evidence builder that maps a run into the neutral bundle the sim-agnostic
cores consume.

Four modules, in the order a run passes through them::

    model_build.py  raw ROS URDF -> a driveable MJCF scene, by MuJoCo's own
                    documented path, with every edit recorded and cited
    runner.py       one scripted (non-agent) T1 attempt: settle, drive to the
                    commanded point, hold; writes a run directory
    recording.py    read one run directory off disk. No neutral schema here
    evidence.py     -> agentbench.graders.evidence.EvidenceBundle, plus the
                    ladder's support-contact channel

**Nothing in this package is a result.** The scripted runner exists so the
column's plumbing can be proven end to end without an agent and without a
token; a ladder cell is an *agent* run, and a scripted attempt is a
control/fixture, never a cell. ``BRINGUP.md`` records the install, the numbers
the scripted attempt actually produced, the traps, and this column's entry in
the effort-parity ledger (plan §5b rule 6).

Scope, stated once
------------------

* **T1 only.** T2 (an arm and a grip), T3 and T4 (locomotion, where MuJoCo
  Playground is the specific named risk) are *not* addressed here and no claim
  about them is made from this package.
* **CPU only.** MJX is installed on this box but ``jax`` resolves a CPU device
  only; T1 needs neither. See ``BRINGUP.md`` §1.
* **Version-pinned.** Every citation is written against MuJoCo 3.8.1. The
  compiler defaults this column depends on have changed before (``strippath``
  was once hardcoded true for URDF), so a different version is a different
  measurement.
"""
