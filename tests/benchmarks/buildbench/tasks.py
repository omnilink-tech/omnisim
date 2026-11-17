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

"""BuildBench task registry — the declarations, as code rather than prose.

Same decision, for the same reason, as ``agentbench/sims.py``: a runner, a
matrix renderer and a report cannot disagree about what was declared if there
is only one declaration.

Read ``SPEC.md`` first. The three things that matter most here:

* **A declaration is not a result** (SPEC 7.2). Every task below is
  ``DECLARED`` or ``BLOCKED``; none has a world, a grader, an oracle or a null,
  so none of them says anything about any simulator yet.
* **``verification_status`` starts at ``UNVERIFIED``** (SPEC 2) and every
  competitor claim in this file is at ``UNVERIFIED`` today -- including the ones
  that feel obvious. A claim at ``UNVERIFIED`` is somebody's belief.
* **``NOT_EXPRESSIBLE`` must name the missing capability** (SPEC 0.2 rule 2),
  and only becomes publishable once it is ``CITED`` against that simulator's own
  docs or ``MEASURED``. A single credible counter-example flips it.

``challenges`` is the field that stops this becoming marketing: it records, in
the declaration itself and before anything is run, the reasons to think our own
claim is wrong.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict

SUITE = "buildbench/v0.1"

# --- expressibility verdicts -------------------------------------------------
EXPRESSIBLE = "EXPRESSIBLE"          # the work has an honest formulation here
NOT_EXPRESSIBLE = "NOT_EXPRESSIBLE"  # it does not -- needs a named capability
PARTIAL = "PARTIAL"                  # part of the work, with the split stated
UNKNOWN = "UNKNOWN"                  # nobody has thought about it properly yet

VERDICTS = (EXPRESSIBLE, NOT_EXPRESSIBLE, PARTIAL, UNKNOWN)

# --- how well established a claim is (SPEC 2) --------------------------------
UNVERIFIED = "UNVERIFIED"  # the default: a belief, nobody checked
CITED = "CITED"            # a citation to that simulator's OWN docs/API/source
MEASURED = "MEASURED"      # a run on a named machine, with an evidence record
REFUTED = "REFUTED"        # a credible counter-example flipped it

STATUSES = (UNVERIFIED, CITED, MEASURED, REFUTED)

#: Only these license a published statement (SPEC 2 rule 1).
PUBLISHABLE_STATUSES = (CITED, MEASURED, REFUTED)

# --- task build status -------------------------------------------------------
DECLARED = "DECLARED"    # registered; no world, grader, oracle or null exists
BUILDING = "BUILDING"    # under construction; still not quotable
GATED = "GATED"          # world + grader + oracle + null all green (SPEC rule 4)
BLOCKED = "BLOCKED"      # cannot be built -- the reason is recorded, not hidden

BUILD_STATUSES = (DECLARED, BUILDING, GATED, BLOCKED)

SIMS = ("omnisim", "webots", "mujoco")


@dataclass(frozen=True)
class Citation:
    """A resolvable reference. ``where`` is a path, URL or file:line."""

    where: str
    says: str


@dataclass
class Claim:
    """One simulator's expressibility for one task."""

    sim: str
    verdict: str
    reason: str
    #: Required when verdict is NOT_EXPRESSIBLE: the SPECIFIC absent thing --
    #: a verb, a device, a node type, a service. Never "it can't really do it".
    missing_capability: str = ""
    verification_status: str = UNVERIFIED
    citations: list = field(default_factory=list)
    #: Reasons to think THIS claim is wrong, written down before anything runs.
    challenges: list = field(default_factory=list)

    @property
    def publishable(self) -> bool:
        """May this claim appear in a rendering without the word UNVERIFIED?"""
        return self.verification_status in PUBLISHABLE_STATUSES

    def as_dict(self):
        d = asdict(self)
        d["citations"] = [asdict(c) if not isinstance(c, dict) else c
                          for c in self.citations]
        d["publishable"] = self.publishable
        return d


@dataclass
class Task:
    id: str
    title: str
    prompt: str
    #: What robotics capability completing this demonstrates.
    demonstrates: str
    #: MUST stand on its own merits and MUST NOT name a simulator (SPEC 0.2
    #: rule 1). If it only makes sense once you know who cannot do it, the task
    #: is reverse-engineered from a competitor's gaps.
    why_this_is_real_work: str
    claims: list
    build_status: str = DECLARED
    #: Why it is not built, when build_status is BLOCKED. Never deleted.
    blocked_reason: str = ""
    #: SPEC 6.1 risk ids this task depends on.
    depends_on_risks: list = field(default_factory=list)
    #: What the grader would have to measure, in physical units. Design note,
    #: not an implementation -- no grader exists.
    grading_sketch: str = ""

    def claim(self, sim):
        for c in self.claims:
            if c.sim == sim:
                return c
        return None

    def as_dict(self):
        return {
            "suite": SUITE,
            "id": self.id,
            "title": self.title,
            "prompt": self.prompt,
            "demonstrates": self.demonstrates,
            "why_this_is_real_work": self.why_this_is_real_work,
            "build_status": self.build_status,
            "blocked_reason": self.blocked_reason,
            "depends_on_risks": list(self.depends_on_risks),
            "grading_sketch": self.grading_sketch,
            "claims": {c.sim: c.as_dict() for c in self.claims},
            "oracle_null_gate": {
                # SPEC rule 4: per (task, simulator), and our own arm is not
                # exempt. Nothing here is green.
                sim: "NOT RUN" for sim in SIMS
            },
        }


# ---------------------------------------------------------------------------
# The five registered tasks.
# ---------------------------------------------------------------------------

_MUJOCO_SENSOR_CHALLENGE = (
    "'MuJoCo has no Lidar/Camera device model' is probably FALSE as stated: "
    "MuJoCo ships a `rangefinder` sensor and offscreen rendering (mjr_render), "
    "and MuJoCo-based embodied-AI stacks routinely produce camera "
    "observations. The defensible narrower claim is about a packaged device "
    "model plus per-robot controller processes as simulator concepts, not "
    "about whether ray casts and pixels can be obtained at all. Until someone "
    "cites MuJoCo's own docs either way this stays UNVERIFIED and the broad "
    "phrasing is not to be repeated."
)

_OMNISIM_SENSOR_RISK = (
    "This may be wrong about US, not about them (SPEC 6.1 risk 1): it is "
    "UNVERIFIED whether Camera devices render at all in the batched "
    "mujoco_warp GPU path. If they do not, the sensor half of this task does "
    "not exist on OmniSim either and the EXPRESSIBLE verdict is wrong."
)


B1 = Task(
    id="B1_trained_locomotion_deploy",
    title="trained locomotion policy, deployed on a sensor-guided course",
    prompt=(
        "Train a locomotion policy for a legged robot, then use that trained "
        "policy to drive the robot through a navigation course whose route is "
        "not known in advance, using a planar LiDAR to find the way. Deliver "
        "the trained policy, the world, and the controller that runs them, and "
        "show the robot completing the course."
    ),
    demonstrates=(
        "the full learn-then-deploy loop inside one simulator: batched "
        "training, a policy artifact, and that same policy driving a robot "
        "that is also consuming a range sensor in closed loop"
    ),
    why_this_is_real_work=(
        "Learned locomotion is how legged robots are controlled in practice, "
        "and a policy that only works in the trainer is not a controller. "
        "Closing the loop -- train, deploy, then navigate an unknown route "
        "with a range sensor -- is the ordinary shape of a legged-robot "
        "deployment, and the step where sim-to-sim and sim-to-real gaps "
        "actually show up."
    ),
    depends_on_risks=["risk_1_sensors_under_batching"],
    grading_sketch=(
        "geometric, from a recorded run: the robot's base reaches the goal "
        "within a stated tolerance; no recorded sample lies inside an "
        "obstacle; integrated path length exceeds a floor derived from the "
        "graded layout; the policy artifact is loaded (assert the load line, "
        "never the exit code -- a missing runtime silently runs a zero-residual "
        "baseline and exits 0)."
    ),
    claims=[
        Claim(
            sim="omnisim",
            verdict=EXPRESSIBLE,
            reason=(
                "in-engine batched training exists and trains THROUGH the "
                "engine, so train == deploy bit-exact; per-robot controller "
                "processes and a LiDAR device both exist"
            ),
            verification_status=UNVERIFIED,
            challenges=[_OMNISIM_SENSOR_RISK],
        ),
        Claim(
            sim="webots",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "the deployment half is expressible -- upstream has both a "
                "LiDAR device and per-robot controller processes -- but the "
                "training half has no path: no GPU-batched parallel-env "
                "rollout"
            ),
            missing_capability=(
                "GPU-batched parallel environments / an in-engine RL rollout "
                "path"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                "PARTIAL may be the honest verdict rather than "
                "NOT_EXPRESSIBLE: an agent could train externally and deploy "
                "the resulting policy in a Webots controller, which is a real "
                "and common workflow. Whether the task means 'train in this "
                "simulator' or 'train somewhere and deploy here' is a "
                "specification question this declaration has not settled, and "
                "it changes the verdict.",
            ],
        ),
        Claim(
            sim="mujoco",
            verdict=PARTIAL,
            reason=(
                "training is a core strength; the declared gap is a packaged "
                "LiDAR/Camera device model and per-robot controller processes"
            ),
            missing_capability=(
                "a packaged range/camera device model and per-robot controller "
                "processes"
            ),
            verification_status=UNVERIFIED,
            challenges=[_MUJOCO_SENSOR_CHALLENGE],
        ),
    ],
)


B2 = Task(
    id="B2_granular_traversal",
    title="graded interaction with granular media",
    prompt=(
        "Drive a wheeled robot across a bed of loose granular material to a "
        "goal on the far side, or move a stated mass of that material into a "
        "target zone, steering from a planar LiDAR. Show the robot reaching "
        "the goal, or the material arriving in the zone."
    ),
    demonstrates=(
        "two-way interaction between a rigid robot and bulk particulate "
        "media -- wheel sinkage, traction loss, and material displaced"
    ),
    why_this_is_real_work=(
        "Planetary rovers, construction and agricultural machines spend their "
        "working lives on loose ground, and terramechanics -- sinkage, slip, "
        "and the force it takes to move a mass of material -- is the thing "
        "that decides whether such a machine works. A vehicle model on rigid "
        "flat ground says nothing about it."
    ),
    build_status=BLOCKED,
    blocked_reason=(
        "MEASURED 2026-08-11, machine 9722d23d12a3: the granular subsystem "
        "does not support a graded task, on four independent grounds. See "
        "SPEC 6.2 and evidence/2026-08-11-granular.md. (1) Particles do not "
        "simulate on the shipped binary -- 'GranularGroup is inert: CUDA is "
        "not available on this build/box', no CPU fallback. (2) Even with "
        "CUDA on, OmGranularGroup::collectColliders skips every Solid whose "
        "body() is NULL, and body() is the ODE body, deleted in bdc02139 -- so "
        "the collider list is always empty and robots and particles are "
        "mutually invisible, in BOTH directions. (3) The reverse force is "
        "written into OmSolid::addForceAtPosition, whose body is empty. (4) "
        "There is no supervisor API to read particle state, so displaced mass "
        "is unmeasurable regardless. The rigid-sphere fallback is not a "
        "shortcut either: granular_sand_demo.omniworld does simulate (294 dynamic "
        "Newton bodies, grains fall z 1.593 -> 0.048) but its grains escape a "
        "+/-1.4 m pit and roll to -80 m, and it contains no robot. NO TASK WAS "
        "AUTHORED. Kept registered at BLOCKED rather than deleted (SPEC 7.5)."
    ),
    depends_on_risks=["risk_3_granular_graded_task"],
    grading_sketch=(
        "would have been geometric: robot base crosses the bed to the goal, "
        "with wheel-hub z measured against the undisturbed bed surface to "
        "evidence sinkage; or mass of material inside the target zone, summed "
        "from particle state. BOTH channels are unmeasurable today -- there is "
        "no particle readback, and no robot-particle interaction to measure."
    ),
    claims=[
        Claim(
            sim="omnisim",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "a GranularGroup node exists and parses, but on the shipped "
                "binary it is inert, its robot-collider list is structurally "
                "empty since the ODE deletion, its reverse-force target is an "
                "empty function, and no API reads particle state"
            ),
            missing_capability=(
                "granular-to-rigid-body coupling (both directions) and any "
                "particle-state readback"
            ),
            verification_status=MEASURED,
            citations=[
                Citation(
                    where="tests/benchmarks/buildbench/evidence/2026-08-11-granular.md",
                    says="the four blockers, with the runs that establish them",
                ),
                Citation(
                    where="src/omnisim/nodes/OmSolid.cpp:4214-4222",
                    says=("'ODE removed: OmSolid::addForceAtPosition / addTorque "
                          "are now UNIMPLEMENTED -- ... the granular-group "
                          "coupling (OmGranularGroup) that call them apply "
                          "nothing.' The function body is empty."),
                ),
                Citation(
                    where="docs/benchmarks/lane4-capability-matrix.md",
                    says=("object.granular_group is PARTIAL: 'this lane has NO "
                          "way to read particle state through the supervisor "
                          "API, so the particles are simulated is UNMEASURED. "
                          "Not counted as working.'"),
                ),
                Citation(
                    where="docs/developer/granular-cuda-plan.md",
                    says=("'Newton coupling -- now a HARD BREAK on every world' "
                          "since bdc02139; 'balls no longer push the robot back "
                          "in any world, and there is no world flag that "
                          "restores it'"),
                ),
            ],
            challenges=[
                "This is MEASURED on ONE binary (built OMNISIM_WITH_CUDA=OFF) "
                "on ONE machine. Blocker 1 is a build-configuration fact and "
                "would be lifted by a CUDA build; blockers 2-4 are source "
                "facts and would not. A CUDA-ON build should still be run "
                "before the verdict is treated as permanent -- it would "
                "confirm or refute blocker 2 empirically rather than by "
                "reading collectColliders.",
                "The verdict is about the CURRENT tree. It is a repairable "
                "defect, not an architectural impossibility, and must never be "
                "quoted as though granular media are beyond the engine.",
            ],
        ),
        Claim(
            sim="webots",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "upstream models bulk particulate media only as many "
                "individual rigid bodies; there is no granular medium as such"
            ),
            missing_capability="a granular / particulate media model",
            verification_status=UNVERIFIED,
            challenges=[
                "'ODE has no granular media' is a claim about a solver, and "
                "the interesting question is whether N rigid spheres are an "
                "honest formulation of the task. Our own measurement says "
                "rigid spheres CAN be stepped in bulk (294 bodies) -- so the "
                "verdict may be PARTIAL for both of us, with the real "
                "distinction being scale and fidelity rather than "
                "expressibility.",
            ],
        ),
        Claim(
            sim="mujoco",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "declared: no CUDA particle system and no packaged range "
                "sensor for the closed-loop half"
            ),
            missing_capability="a granular / particulate media model",
            verification_status=UNVERIFIED,
            challenges=[
                _MUJOCO_SENSOR_CHALLENGE,
                "MuJoCo can step large numbers of rigid bodies and is used for "
                "deformable and composite objects; whether that reaches "
                "'granular media' is exactly the definitional question above, "
                "and it is unresolved for every arm including ours.",
            ],
        ),
    ],
)


B3 = Task(
    id="B3_robustness_distribution",
    title="a success rate with an interval, over ~1000 randomised draws",
    prompt=(
        "Measure how reliably this robot completes its task when the world is "
        "not exactly as authored. Randomise ground friction, payload mass and "
        "the robot's initial pose over about a thousand draws, and report the "
        "success rate with a confidence interval -- not a single run."
    ),
    demonstrates=(
        "domain randomisation at a scale where the deliverable is a "
        "distribution, and the simulator is the instrument that produces it"
    ),
    why_this_is_real_work=(
        "A controller that worked once tells you nothing about whether it "
        "works. Anything going near hardware is qualified on a success rate "
        "over randomised conditions, because the friction, the mass and the "
        "starting pose are never what the model said. Producing that number, "
        "with an interval, IS the engineering deliverable."
    ),
    depends_on_risks=["risk_1_sensors_under_batching"],
    grading_sketch=(
        "the deliverable is itself a measurement, so the grader checks the "
        "PROCESS as well as the number: N draws actually differed (assert the "
        "sampled parameters vary, not just the seed), each draw ran to a "
        "terminal condition, the reported rate matches a recount from the "
        "per-draw records, and the interval is computed from N rather than "
        "asserted. A single-run answer fails by construction."
    ),
    claims=[
        Claim(
            sim="omnisim",
            verdict=EXPRESSIBLE,
            reason=(
                "batched parallel environments reach the scale, and worlds are "
                "authored artifacts that can be varied per draw"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                _OMNISIM_SENSOR_RISK,
                "'reaches the scale' is unquantified here. The measured "
                "in-engine env-step rate spans ~33x across GPUs (10,228 "
                "env-steps/s at batch 256 on an RTX 3060 vs 333,036 at 4096 on "
                "a 4090), so whether 1000 draws is minutes or hours is a "
                "per-machine fact and this task must state the machine.",
            ],
        ),
        Claim(
            sim="webots",
            verdict=NOT_EXPRESSIBLE,
            reason="declared: cannot reach the scale",
            missing_capability=(
                "GPU-batched parallel environments; parallelism is N OS "
                "processes"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                "'Cannot reach the scale' is a throughput claim and nobody has "
                "measured it. 1000 draws of a short episode across K parallel "
                "processes is not obviously out of reach on a workstation, and "
                "if it merely takes longer then the honest verdict is "
                "EXPRESSIBLE-but-slower, which is a cost finding and not an "
                "expressibility one. This row should probably be a "
                "measurement, not a declaration.",
            ],
        ),
        Claim(
            sim="mujoco",
            verdict=PARTIAL,
            reason=(
                "batching is a core strength and the statistical half is "
                "straightforward; the declared gap is scenario authoring and "
                "the device model for the sensing half"
            ),
            missing_capability="scenario authoring and a packaged device model",
            verification_status=UNVERIFIED,
            challenges=[
                _MUJOCO_SENSOR_CHALLENGE,
                "If the task's sensing requirement is dropped, this is very "
                "likely EXPRESSIBLE on MuJoCo and arguably easier there than "
                "here. That would be a good result to publish and the task "
                "should not be shaped to avoid it.",
            ],
        ),
    ],
)


B4 = Task(
    id="B4_multi_robot_radio",
    title="~20 robots coordinating over a radio device model",
    prompt=(
        "Build a scene with twenty mobile robots that must each reach a "
        "different goal without colliding with one another, coordinating over "
        "a simulated radio link -- each robot broadcasts and receives "
        "messages, and there is no central controller. Show every robot at its "
        "goal and no collisions along the way."
    ),
    demonstrates=(
        "many independent robots, each with its own controller process and a "
        "message-passing device, solving a shared spatial problem"
    ),
    why_this_is_real_work=(
        "Warehouse fleets, agricultural swarms and inspection teams are "
        "decentralised: each machine runs its own controller and coordinates "
        "over a radio with latency and range limits. Whether a distributed "
        "policy actually avoids deadlock and collision at fleet scale is a "
        "question only a simulator answers before the fleet is bought."
    ),
    depends_on_risks=["risk_2_njmax_silent_overflow"],
    grading_sketch=(
        "geometric and per-robot: every robot's base within tolerance of its "
        "own goal; zero robot-robot contacts over the whole run (contacts "
        "scanned for the entire window -- a zero-length window makes "
        "collision-freedom unknowable, and 'we could not check' must never "
        "read as 'nothing was hit'); messages actually delivered, evidenced "
        "behaviourally rather than by a send counter. PLUS a mandatory "
        "instrument check: the world must declare newtonNjmax/newtonNconmax "
        "and the run must record observed peak nefc against the cap, or the "
        "result is INVALID."
    ),
    claims=[
        Claim(
            sim="omnisim",
            verdict=EXPRESSIBLE,
            reason=(
                "Emitter/Receiver devices and one controller process per robot "
                "both exist, and multi-robot worlds at this scale are routine"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                "SPEC 6.1 risk 2: newtonNjmax/newtonNconmax default to 256 and "
                "overflow SILENTLY on mujoco_warp -- the constraint vector is "
                "truncated, contact solving degrades, and the only warning is "
                "a kernel-side wp.printf that Windows discards entirely. A "
                "wheeled robot on four wheels is ~32 constraint rows, so 20 "
                "robots is far past the default. Unless the world sets the "
                "fields deliberately, this task does not measure multi-robot "
                "coordination -- it measures our own unset default.",
                "The ~9-robot threshold above is itself UNCONFIRMED on the "
                "present runtime: OmniBench lane 4b swept driven rovers, found "
                "peak nefc tracking 32*N exactly but the ALLOCATED cap "
                "tracking it too, and could not force an overflow in three "
                "attempts -- so it reports cliff_detector_validated: false and "
                "refuses to call it a pass. Do not quote lane 4b as "
                "reassurance and do not read it as refuting the threshold.",
            ],
        ),
        Claim(
            sim="webots",
            verdict=PARTIAL,
            reason=(
                "upstream has both Emitter/Receiver and per-robot controller "
                "processes; the declared limitation is throughput at scale"
            ),
            missing_capability="",
            verification_status=UNVERIFIED,
            challenges=[
                "The throughput claim names the WRONG ENGINE VERSION. Upstream "
                "Webots R2025a is not this tree's deleted ODE path, and "
                "nobody has measured upstream at 20 robots. Recorded as "
                "PARTIAL rather than NOT_EXPRESSIBLE precisely because the "
                "devices plainly exist -- and if the throughput holds, the "
                "honest verdict is EXPRESSIBLE and this is a task upstream "
                "does as well as we do or better. That would be a fine result.",
                "Our own historical measurement cuts against the assumption: "
                "same-harness on the step_cost bench while ODE still shipped, "
                "ODE was FASTER per step than Newton at every scene size "
                "measured. Any 'ODE collapses at scale' phrasing is unsupported "
                "by this tree's own numbers and must not be repeated.",
            ],
        ),
        Claim(
            sim="mujoco",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "declared: no radio device model and no independent controller "
                "processes -- one process steps one model"
            ),
            missing_capability=(
                "an inter-robot message-passing device (Emitter/Receiver) and "
                "independent per-robot controller processes"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                "A user would reasonably implement message passing in the "
                "driving Python program, which may well be an honest "
                "formulation -- the task says the robots coordinate over a "
                "radio, not that the simulator ships a radio node. If so this "
                "is EXPRESSIBLE. The narrower defensible claim is about a "
                "MODELLED link (range limits, latency, collisions) rather than "
                "about message passing as such, and the declaration has not "
                "settled which the task requires.",
            ],
        ),
    ],
)


B5 = Task(
    id="B5_procedural_generalization",
    title="train across seeded procedural worlds, grade on an unseen seed",
    prompt=(
        "Generate fifty different worlds from seeds 1 to 50, train a policy "
        "that works across all of them, and then show it working in a world "
        "generated from a seed it has never seen."
    ),
    demonstrates=(
        "deterministic seeded world generation and training across a "
        "distribution of environments, graded on held-out generalisation"
    ),
    why_this_is_real_work=(
        "A policy trained in one environment memorises that environment. "
        "Training across a generated distribution and holding out unseen "
        "instances is the standard way to tell generalisation from memorisation "
        "-- and it only works if world generation is deterministic from a seed, "
        "so the held-out instance is reproducible and the training set is "
        "exactly stated."
    ),
    depends_on_risks=["risk_1_sensors_under_batching"],
    grading_sketch=(
        "the held-out seed is the whole point, so the grader draws it ITSELF "
        "and never accepts one the agent chose: generate the world from a seed "
        "outside the training range at grade time, verify byte-identical "
        "regeneration from that seed, then measure task completion "
        "geometrically in it. A policy that only completes training seeds "
        "fails, and that gap IS the finding."
    ),
    claims=[
        Claim(
            sim="omnisim",
            verdict=EXPRESSIBLE,
            reason=(
                "seeded deterministic world generation is a shipped tool "
                "(same recipe+seed+params -> byte-identical .wbt) and in-engine "
                "batched training exists"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                _OMNISIM_SENSOR_RISK,
                "Byte-identical generation is a documented property of the "
                "generator; that it holds is UNVERIFIED here, and 'documented' "
                "is not 'measured' (SPEC 2 rule 2). It is cheap to check and "
                "should be checked before the row is quoted.",
                "Training across 50 DISTINCT worlds is not the same shape as "
                "batching 4096 copies of one world. Whether the in-engine "
                "trainer can batch heterogeneous worlds at all is not "
                "established, and if it cannot, this task is not expressible "
                "on OmniSim as written.",
            ],
        ),
        Claim(
            sim="webots",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "declared: neither the training half nor seeded deterministic "
                "world generation"
            ),
            missing_capability=(
                "a seeded deterministic procedural world generator, and a "
                "GPU-batched training path"
            ),
            verification_status=UNVERIFIED,
            challenges=[
                "A user can write a world generator in fifty lines of Python "
                "-- .wbt is text. 'The simulator does not ship one' is a very "
                "different claim from 'this cannot be expressed', and only the "
                "first is defensible. This row is the one most likely to be "
                "flipped by a single credible counter-example, which is "
                "exactly the SPEC 0.2 rule 2 trigger.",
            ],
        ),
        Claim(
            sim="mujoco",
            verdict=NOT_EXPRESSIBLE,
            reason=(
                "declared: no seeded deterministic world generation as a "
                "shipped capability"
            ),
            missing_capability="a seeded deterministic procedural world generator",
            verification_status=UNVERIFIED,
            challenges=[
                "MJCF is text and procedurally generating it is routine in "
                "MuJoCo-based RL work; domain randomisation over generated "
                "scenes is arguably more common there than here. This row "
                "should be expected to flip to EXPRESSIBLE and it is recorded "
                "in that expectation.",
            ],
        ),
    ],
)


TASKS = [B1, B2, B3, B4, B5]
BY_ID = {t.id: t for t in TASKS}


# --- risk register (SPEC 6.1), carried in code so tasks can reference it -----
RISKS = {
    "risk_1_sensors_under_batching": {
        "statement": (
            "Do sensors (camera / lidar) work in the batched mujoco_warp GPU "
            "path? Suspected: cameras do NOT render under batching."
        ),
        "threatens": ["B1_trained_locomotion_deploy",
                      "B3_robustness_distribution",
                      "B5_procedural_generalization"],
        "status": UNVERIFIED,
        "note": (
            "The highest-value check in the register, because it decides "
            "whether three of five tasks are expressible ON OMNISIM -- i.e. it "
            "is a risk to our own claims first."
        ),
    },
    "risk_2_njmax_silent_overflow": {
        "statement": (
            "newtonNjmax / newtonNconmax default to 256 and overflow SILENTLY "
            "on mujoco_warp; ~32 constraint rows per wheeled robot puts ~20 "
            "robots far past it. The task must set the per-world fields "
            "deliberately or it measures our own overflow."
        ),
        "threatens": ["B4_multi_robot_radio"],
        "status": UNVERIFIED,
        "note": (
            "CITED as a hazard, UNVERIFIED as a threshold: OmniBench lane 4b "
            "could not force an overflow on generated rovers and reports "
            "cliff_detector_validated: false. A green that cannot be made to "
            "go red is not evidence in either direction."
        ),
    },
    "risk_3_granular_graded_task": {
        "statement": (
            "B2 needs a robot to interact with granular media in a GRADED way; "
            "demos existing is a weaker claim than a scored task."
        ),
        "threatens": ["B2_granular_traversal"],
        "status": MEASURED,
        "note": (
            "RESOLVED NEGATIVELY, 2026-08-11, machine 9722d23d12a3. The "
            "subsystem does not support a graded task on four independent "
            "grounds; see SPEC 6.2 and evidence/2026-08-11-granular.md. No "
            "task was authored."
        ),
    },
}


def matrix():
    """Render the expressibility matrix as text.

    Every cell prints its verification_status, because a claim at UNVERIFIED
    may not be shown without it (SPEC 2 rule 1).
    """
    lines = ["%s -- expressibility matrix" % SUITE,
             "DECLARATIONS ONLY. No cell has an oracle, a null or a graded run.",
             ""]
    head = "%-32s %-8s %s" % ("task", "build", "  ".join(
        "%-26s" % s for s in SIMS))
    lines.append(head)
    lines.append("-" * len(head))
    for t in TASKS:
        cells = []
        for sim in SIMS:
            c = t.claim(sim)
            if c is None:
                cells.append("%-26s" % "-")
                continue
            tag = "" if c.publishable else " !UNVERIFIED"
            cells.append("%-26s" % (c.verdict + tag))
        lines.append("%-32s %-8s %s" % (t.id[:32], t.build_status, "  ".join(cells)))
    lines.append("")
    lines.append("! = UNVERIFIED: a belief, not a finding. Not publishable "
                 "without this marker (SPEC 2).")
    n_pub = sum(1 for t in TASKS for c in t.claims if c.publishable)
    lines.append("publishable claims: %d of %d"
                 % (n_pub, sum(len(t.claims) for t in TASKS)))
    return "\n".join(lines)


def as_dict():
    return {"suite": SUITE,
            "sims": list(SIMS),
            "tasks": [t.as_dict() for t in TASKS],
            "risks": RISKS}


def write_meta(root=None):
    """Mirror each declaration into tasks/<id>/{meta.json,prompt.txt}.

    The registry is the single source of truth (see the module docstring); these
    files are generated so a reader browsing the task tree sees the same thing
    the code says. ``test_declarations.py`` asserts they have not drifted.
    """
    import os
    root = root or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "tasks")
    written = []
    for t in TASKS:
        d = os.path.join(root, t.id)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "meta.json"), "w", encoding="utf-8",
                  newline="\n") as fh:
            json.dump(t.as_dict(), fh, indent=2)
            fh.write("\n")
        with open(os.path.join(d, "prompt.txt"), "w", encoding="utf-8",
                  newline="\n") as fh:
            fh.write(t.prompt.rstrip() + "\n")
        written.append(d)
    return written


if __name__ == "__main__":
    import sys
    if "--json" in sys.argv:
        print(json.dumps(as_dict(), indent=2))
    elif "--write-meta" in sys.argv:
        for d in write_meta():
            print("wrote %s" % d)
    elif "--list" in sys.argv:
        for t in TASKS:
            print("%-32s %-9s %s" % (t.id, t.build_status, t.title))
    else:
        print(matrix())
