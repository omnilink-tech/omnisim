# Archived documentation

Docs removed from the tree on 2026-09-02 because they were stale, duplicated, or the log/plan of a campaign that has finished. They misled agents that read the tree as current.

Every entry is recoverable with `git show <sha>:<path>`; nothing here is current.

Class key: **campaign log** = dated measurements or a journal of work that ended; **plan (executed)** = a plan whose work landed (the code and the current reference pages are the record); **plan (never executed)** = a plan that was not carried out; **duplicate** = the same content lives in the page named in the last column; **stale** = asserted a state of the engine that no longer holds (WREN, ODE, `.wbt` written); **upstream-unused** = inherited Webots page for a node that has no effect under Newton; **archive** = already under an archive directory.

| Path | Class | Last commit | What it was |
|---|---|---|---|
| `docs/benchmarks/friction-reconciliation-2026-08-09.md` | campaign log | `be41986f8` | dated campaign report; ODE arms unrepeatable since bdc02139 |
| `docs/benchmarks/lane2-graphed-ab-2026-08-08.md` | campaign log | `486f2afc5` | dated campaign report; ODE arms unrepeatable since bdc02139 |
| `docs/benchmarks/measurements-2026-06-14.md` | campaign log | `3d1f88514` | dated campaign report; ODE arms unrepeatable since bdc02139 |
| `docs/benchmarks/measurements-2026-07-18.md` | campaign log | `323395c95` | dated campaign report; ODE arms unrepeatable since bdc02139 |
| `docs/design/gripper_support_plan.md` | plan (executed) | `2c1e476b7` | banner "Status: ALL PHASES DONE"; grippers.md is the reference |
| `docs/developer/agent-native-api.md` | plan (executed) | `74fe6542c` | analysis+proposal 2026-07-26; banner lists what shipped since and which proposals were wrong |
| `docs/developer/agent-runtime-observability.md` | plan (executed) | `2c1e476b7` | implemented and live; /sim/events is specified in PROTOCOL s7.19 |
| `docs/developer/architectural-baseline.md` | plan (executed) | `784e9e0c1` | milestone COMPLETE 2026-06-07; banner marks clauses 1,2,4 historical; tag exists despite README claim |
| `docs/developer/archive/README.md` | archive | `187a9baab` | under docs/developer/archive/ |
| `docs/developer/archive/build-baselines.md` | archive | `0749b2537` | under docs/developer/archive/ |
| `docs/developer/archive/cuda-rigid-body-solver-plan.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/editor-and-gui-responsiveness.md` | archive | `3d1f88514` | under docs/developer/archive/ |
| `docs/developer/archive/fps-optimization-journey.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/migration-perf-comparison.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/newton-default-and-omnisim-rename-plan.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/r3-rendering-backend-evaluation.md` | archive | `74fe6542c` | under docs/developer/archive/ |
| `docs/developer/archive/rl-pipeline.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/rl-training-handoff.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/archive/spot-newton-state.md` | archive | `74fe6542c` | under docs/developer/archive/ |
| `docs/developer/archive/spot-walking-demo.md` | archive | `6f495ca01` | under docs/developer/archive/ |
| `docs/developer/asset-pipeline-and-world-quality.md` | stale | `3d1f88514` | line 39/49: "create or reuse a WREN texture ... WREN-side texture caching" -- WREN deleted 2026-08-23, no banner |
| `docs/developer/atlas-stand-rl-journey.md` | campaign log | `be41986f8` | journey for a robot removed 2026-07-17; 4/5 cited paths gone |
| `docs/developer/binary-parity-probe.md` | plan (executed) | `be41986f8` | probe built+run 2026-06-26, welded lane passes; 15 inbound |
| `docs/developer/codebase-audit-2026-07.md` | campaign log | `3b30359e0` | dated audit snapshot; 4 cited src/wren paths gone |
| `docs/developer/cold-launch-failure-2026-08-02.md` | campaign log | `3d1f88514` | dated debug session marked open; the one-in-three race was fixed 2026-08-29 (console attach, issue #3) -- doc not updated |
| `docs/developer/community-tracker-plan.md` | plan (never executed) | `00baef779` | Phase 1 spec never executed; dance campaign PARKED 2026-07-04 (ghosts/g1/dance/README) |
| `docs/developer/dispatcher-surface-signoff.md` | plan (executed) | `74fe6542c` | sign-off whose proof is void (banner 2026-08-08) |
| `docs/developer/g1-deploy-walk.md` | campaign log | `be41986f8` | superseded 2026-07-01 banner; 4/7 cited paths gone |
| `docs/developer/g1-deterministic-brain.md` | campaign log | `4e3b3ee7c` | multi-session research/coordination log |
| `docs/developer/g1-mjcf-single-model.md` | plan (never executed) | `3d1f88514` | both consumer switches designed, neither flipped (2026-06-17) |
| `docs/developer/g1-mpc-deterministic-brain-research.md` | campaign log | `056b26a8b` | self-described archival research writeup |
| `docs/developer/g1-sit-mimic-plan.md` | plan (executed) | `6f495ca01` | workstream closed and folded into research archive (banner) |
| `docs/developer/g1-sitstand-journey.md` | campaign log | `6f495ca01` | journey doc with superseded pivot |
| `docs/developer/g1-stand-engine-vs-centering-lesson.md` | campaign log | `3d1f88514` | lesson write-up |
| `docs/developer/g1-universal-tracker.md` | campaign log | `47d999fa7` | north-star objective + walking test-bed ledger |
| `docs/developer/g1-walk-rl-journey.md` | campaign log | `6f495ca01` | journal of retracted distances (banner) |
| `docs/developer/ghost-construction-improvements.md` | plan (never executed) | `be41986f8` | survey + improvement roadmap, not executed |
| `docs/developer/ghost-tracking-pipeline.md` | duplicate | `be41986f8` | banner: superseded as the how-to by shadowing.md; kept for history |
| `docs/developer/granular-cuda-plan.md` | plan (executed) | `3d1f88514` | CUDA M2 complete and demoed (2026-06-09) |
| `docs/developer/h1-walk-rl-journey.md` | campaign log | `6f495ca01` | pre-parity H1 journal; verdicts superseded (banner) |
| `docs/developer/harness-latency-2026-07-31.md` | campaign log | `cbb1a1675` | dated latency measurement on the XPBD-era engine |
| `docs/developer/humanoid-balance-gap.md` | campaign log | `be41986f8` | SUPERSEDED 2026-06-10 banner; historical analysis |
| `docs/developer/improvement-backlog.md` | plan (never executed) | `3d1f88514` | rolling backlog with "freshness" caveat; 7 ODE mentions; entries not tracked to commits |
| `docs/developer/isaac-parity-plan.md` | plan (never executed) | `9f99ec379` | W7 delivered by reversing a non-goal; W1-W6 scoped, not started (2026-08-17) |
| `docs/developer/ladder-findings-2026-08-02.md` | campaign log | `be41986f8` | dated findings, none fixed; contactProperties finding still true |
| `docs/developer/ladder-grid-2026-08-02.md` | campaign log | `cbb1a1675` | first ladder grid (invalid cell) |
| `docs/developer/locomotion-shadowing-vs-pure-rl.md` | campaign log | `be41986f8` | SUPERSEDED 2026-07-03 banner; kept as history of why hand ghosts fail |
| `docs/developer/loopbench-plan.md` | plan (never executed) | `5df03fa35` | plan, 2026-08-02. Nothing measured yet. |
| `docs/developer/migration-parallel-lanes.md` | plan (executed) | `be41986f8` | multi-session split for a migration that is complete (ODE gone 08-08, WREN gone 08-23) |
| `docs/developer/newton-robot-combat-completion-plan.md` | plan (executed) | `6f495ca01` | Complete 2026-07-09; verification strategy no longer runnable (banner) |
| `docs/developer/omglwindow-extraction-brief.md` | plan (executed) | `b2464df72` | brief for OmGlWindow extraction; src/omnisim/gui/OmGlWindow.cpp exists |
| `docs/developer/omniarm6-suction-bin-pick-journey.md` | campaign log | `3b30359e0` | journey doc |
| `docs/developer/omnilink-roadmap.md` | plan (executed) | `3f3a6a6ae` | status table as of 2026-05-27; says 13 chat demos (now 15) |
| `docs/developer/open-problem-legitimate-stair-climb.md` | campaign log | `8b387ac90` | OPEN problem statement 2026-07-11; 0 inbound |
| `docs/developer/optimization-backlog.md` | plan (executed) | `3d1f88514` | table of landed + build-gated optimisations with commit hashes |
| `docs/developer/p6-captures/README.md` | campaign log | `6f495ca01` | frozen p6_ode_*.jsonl artifacts of a deleted backend |
| `docs/developer/p6-captures/p6_newton_substep4_velsmooth5_20260529.jsonl` | campaign log | `8bdc4882e` | frozen P6 capture of the deleted ODE backend (companion of p6-captures/README.md) |
| `docs/developer/p6-captures/p6_ode_20260528.jsonl` | campaign log | `bfdbced91` | frozen P6 capture of the deleted ODE backend (companion of p6-captures/README.md) |
| `docs/developer/p6-captures/p6_ode_raw_20260529.jsonl` | campaign log | `8bdc4882e` | frozen P6 capture of the deleted ODE backend (companion of p6-captures/README.md) |
| `docs/developer/phase-two-architecture-plan.md` | plan (never executed) | `3d1f88514` | April architecture plan; guardrails overtaken (banner); phases not executed as written |
| `docs/developer/phase-two-execution-program.md` | plan (never executed) | `3d1f88514` | workstream sequencing for the phase-two plan; only WS1 partially in tree (freshness note) |
| `docs/developer/physics-contact-impulse-api.md` | plan (executed) | `0b4c8e789` | design + landing record; ODE wire path deleted (banner) |
| `docs/developer/physics-p8-statics-design.md` | plan (executed) | `be41986f8` | GOAL ACHIEVED / WHOLLY HISTORICAL banner |
| `docs/developer/physics-step-cost-optimization-plan.md` | plan (executed) | `6f495ca01` | plan of record 2026-08-08/09; tier-1 shipped (2.52x end-to-end) |
| `docs/developer/procedural-world-generation-plan.md` | plan (never executed) | `6f495ca01` | 106 KB three-tier plan; 38 of 63 cited paths do not exist (omniworld-agent-playbook.md, region-profiles, scenario-guide, audit_priors.py); tiers 2-3 unbuilt |
| `docs/developer/quad-mpc-in-engine-port.md` | plan (never executed) | `be41986f8` | specced here, not yet applied (2026-06-28) |
| `docs/developer/r3-rendering-backend-evaluation.md` | plan (executed) | `74fe6542c` | R3 design; wgpu shipped and WREN deleted |
| `docs/developer/r4-completion-checklist.md` | plan (executed) | `6f495ca01` | living tracker for a flip that happened 2026-08-19 |
| `docs/developer/r4-step3c-plan.md` | plan (executed) | `3d1f88514` | PLANNING 2026-06-04 for the viewport swap, done |
| `docs/developer/rendering-arm-checklist.md` | plan (executed) | `6f495ca01` | WREN->wgpu completion checklist; arm complete |
| `docs/developer/rl-accelerated-training.md` | stale | `be41986f8` | says "OmniSim RL pipeline supports three training backends" (sb3/mjx/mujoco_warp) and routes to archive/rl-training-handoff -- AGENTS: train IN-ENGINE, and projects/policies/research is the non-working archive |
| `docs/developer/rl-journey.md` | campaign log | `be41986f8` | narrative "current to 2026-06-14" |
| `docs/developer/rl-phase-a-validation-log.md` | campaign log | `be41986f8` | validation log, last session 2026-06-10 |
| `docs/developer/rl-two-layer-architecture.md` | duplicate | `be41986f8` | earlier name for what is now called Shadowing -- content is shadowing.md |
| `docs/developer/sensor-and-device-performance.md` | stale | `3d1f88514` | line 28 "renders through a WREN camera", line 153 cites src/omnisim/wren/OmWrenTextureOverlay.* -- deleted 2026-08-23, no banner |
| `docs/developer/shadow-iteration.md` | campaign log | `be41986f8` | two-campaign research record |
| `docs/developer/shadowing-experiments.md` | campaign log | `be41986f8` | measured results table for the paper |
| `docs/developer/shadowing-ghost-tube-exploration-research.md` | campaign log | `be41986f8` | research, not shipped |
| `docs/developer/shadowing-verification.md` | campaign log | `be41986f8` | claim-by-claim verification dated 2026-06-23 |
| `docs/developer/t2-column-2026-08-04.md` | campaign log | `82001366f` | dated column result; 0 inbound |
| `docs/developer/train-deploy-unification.md` | plan (executed) | `be41986f8` | Phase 0/1 landed, Phase 2 functional (2026-06-24) |
| `docs/developer/v6-readiness.md` | plan (executed) | `6137aa6b7` | SUPERSEDED 2026-08-08 banner: goal (a) executed |
| `docs/developer/verifiably-best-agentic-simulator-plan.md` | plan (never executed) | `b31553687` | active execution plan as of 2026-08-13; frontier reporter exists, campaign rows not |
| `docs/developer/wgpu-shadow-aura.md` | campaign log | `308f3a7fb` | FIXED 2026-08-24 bug write-up |
| `docs/developer/wren-retirement-plan.md` | plan (executed) | `74fe6542c` | audit + plan for a deletion that landed 2026-08-23 |
| `docs/guide/actuators.md` | upstream-unused | `d2b7e6d7a` | listed on the upstream Webots asset catalog (assets.md: there is no cloud catalog); table lists Track, whose propulsion is dead |
| `docs/guide/building-omnisim.md` | duplicate | `6fa8d23b1` | 7-line pointer to developer/quickstart.md + AGENTS s2 |
| `docs/guide/general-bugs.md` | stale | `cbb1a1675` | Cylinder-Cylinder ... IndexedFaceSet-Cylinder collision detection may occasionaly yield wrong contact points (ODE collider bugs) and "relies on OpenGL" -- engine is Newton/MuJoCo + wgpu |
| `docs/guide/sensors.md` | upstream-unused | `d2b7e6d7a` | listed on the upstream Webots asset catalog under the sensor keyword -- catalog not shipped (assets.md); generic list duplicates reference/menu |
| `docs/reference/changelog.md` | duplicate | `47d999fa7` | pointer to root CHANGELOG.md |
| `docs/reference/contactproperties.md` | upstream-unused | `05f0f3c66` | node parsed but not read since bdc02139 (banner); friction lives in WorldInfo.newton* fields |
| `docs/reference/damping.md` | upstream-unused | `3d1f88514` | banner: "records what the node did while ODE shipped. Read it as history" |
| `docs/reference/lua-procedural-proto.md` | upstream-unused | `0db6a18a7` | 1-line empty stub for removed Lua PROTOs (menu test allow-lists it) |
| `docs/reference/procedural-proto-nodes.md` | duplicate | `0db6a18a7` | 4-line pointer to javascript-procedural-proto.md (menu test allow-lists it) |
| `docs/reference/track.md` | upstream-unused | `0f29df9c6` | Track propulsion does not reach the physics (banner); AGENTS: Track is dead |
| `docs/reference/trackwheel.md` | upstream-unused | `0db6a18a7` | companion of the dead Track node |
| `projects/metazoa/PLAN.md` | plan (never executed) | `7fb8ef1bb` | plan v1 2026-08-28; P1 landed, P2/P3 not |
| `projects/omni_quest/docs/RESEARCH.md` | campaign log | `be41986f8` | research synthesis |
| `projects/policies/ghosts/g1/dance/README.md` | plan (never executed) | `0debcc161` | PARKED 2026-07-04 campaign luts |
| `projects/policies/research/README.md` | archive | `be41986f8` | index of the RL research archive ("non-working / superseded") |
| `projects/policies/research/controllers/g1_ghost_walk/README.md` | archive | `d509118ff` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/controllers/humanoid_static_walk/README.md` | archive | `d509118ff` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/foot_redesign/RESULTS.md` | archive | `6f495ca01` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/g1_centroidal/README.md` | archive | `b8e2be480` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/g1_mpc_pushrecovery_research.md` | archive | `c4d743660` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/g1_step_residual_rl.md` | archive | `6f495ca01` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/g1_walk_wbc_design.md` | archive | `d509118ff` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/mpc/quad_morph/RESULTS.md` | archive | `be41986f8` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/research/training/README.md` | archive | `be41986f8` | under the RL research archive (projects/policies/research/, "non-working / superseded") |
| `projects/policies/skills/humanoid/g1_arm_motion/README.md` | duplicate | `6f495ca01` | same content as docs/developer/g1-arm-motion-skill.md |
| `projects/policies/skills/quadruped/go2_turn/TRAIN_PLAN.md` | plan (executed) | `5fa2a2db4` | pod recipe for a campaign that ran (Go2 round 3) |
| `projects/samples/demos/controllers/smart_house_bridge/VERIFICATION.md` | campaign log | `0fcfe87d3` | measured verification 2026-08-19 |
| `projects/default/controllers/braitenberg/` | controller (no world names it) | `8f3b82945` | no live world, test or script names the controller |
| `projects/default/controllers/perf_window_runner/` | controller (no world names it) | `0fc15a998` | no live world, test or script names the controller |
| `projects/robots/franka_emika/panda/controllers/panda_arm_demo/` | controller (no world names it) | `8f3b82945` | no live world, test or script names the controller |
| `projects/robots/omnisim/omnidune/controllers/omnidune_driver/` | controller (no world names it) | `6bd5c41be` | no live world, test or script names the controller |
| `projects/samples/devices/controllers/hokuyo/` | controller (no world names it) | `8f3b82945` | no live world, test or script names the controller |
| `projects/samples/devices/controllers/sick/` | controller (no world names it) | `8f3b82945` | no live world, test or script names the controller |
| `projects/samples/devices/controllers/sick_point_cloud/` | controller (no world names it) | `49374843e` | no live world, test or script names the controller |
| `projects/robots/husarion/rosbot/worlds/rosbot.omniworld` | world (dead robot dir) | `6f495ca01` | rule 1 -- its robot package is unreached; nothing outside the package names it |
| `projects/robots/omnisim/omnidune/protos/OmniDune.proto` | PROTO (uninstantiated) | `60dbcf00e` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/robots/omnisim/omnidune/worlds/dune_course.omniworld` | world (dead robot dir) | `60dbcf00e` | rule 1 -- its robot package is unreached; nothing outside the package names it |
| `projects/robots/omnisim/omnidune/worlds/omnidune_showcase.omniworld` | world (dead robot dir) | `60dbcf00e` | rule 1 -- its robot package is unreached; nothing outside the package names it |
| `projects/robots/omnisim/omnidune/worlds/omnidune_test.omniworld` | world (dead robot dir) | `60dbcf00e` | rule 1 -- its robot package is unreached; nothing outside the package names it |
| `projects/robots/franka_emika/panda/` | robot description (unreached) | `f79b19a70` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/franka_emika/panda/controllers/` | robot description (unreached) | `f79b19a70` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/franka_emika/panda/meshes/` | robot description (unreached) | `47d999fa7` | 10 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/franka_emika/panda/urdf/` | robot description (unreached) | `47d999fa7` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/husarion/rosbot_description/meshes/rosbot/` | robot description (unreached) | `4738e1e30` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/omnisim/omnidune/scripts/` | robot description (unreached) | `60dbcf00e` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/omnisim/omnidune/worlds/` | robot description (unreached) | `60dbcf00e` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/clearpath/husky_description/meshes/` | robot meshes (unreferenced by any URDF/xacro) | `74fe6542c` | 17 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/clearpath/husky_description/meshes/accessories/` | robot meshes (unreferenced by any URDF/xacro) | `74fe6542c` | 6 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/clearpath/jackal_description/meshes/` | robot meshes (unreferenced by any URDF/xacro) | `74fe6542c` | 7 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/unitree/b2/meshes/` | robot meshes (unreferenced by any URDF/xacro) | `5d57831e3` | 4 files, 0.00 MB; nothing in the tree names them |
| `projects/robots/unitree/g1/urdf/` | robot meshes (unreferenced by any URDF/xacro) | `6f495ca01` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robots/unitree/h1/urdf/meshes/` | robot meshes (unreferenced by any URDF/xacro) | `09c194713` | 28 files, 0.00 MB; nothing in the tree names them |
| `projects/appearances/protos/ChequeredParquetry.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/DarkParquetry.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/Fabric.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/SlatePavement.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/SquarePavement.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/StonePavement.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/objects/street_furniture/protos/BusStopBench.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/objects/street_furniture/protos/BusStopMesh.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/objects/street_furniture/protos/PublicToiletMesh.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/samples/protogen/protos/OmniSimDemoMaterial.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/samples/protogen/protos/OmniSimDemoPipeProp.proto` | PROTO (uninstantiated) | `9484a0656` | no world, PROTO or script instantiates it (sidecar .yaml/.py/icon removed with it) |
| `projects/appearances/protos/` | PROTO textures/sidecars | `f286da72f` | 6 files, 0.00 MB; nothing in the tree names them |
| `projects/appearances/protos/icons/` | PROTO textures/sidecars | `f286da72f` | 6 files, 0.00 MB; nothing in the tree names them |
| `projects/objects/street_furniture/protos/` | PROTO textures/sidecars | `f40c72067` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/samples/demos/worlds/textures/moon/` | PROTO textures/sidecars | `0db6a18a7` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/samples/protogen/protos/` | PROTO textures/sidecars | `9484a0656` | 4 files, 0.00 MB; nothing in the tree names them |
| `projects/samples/protogen/protos/icons/` | PROTO textures/sidecars | `47d999fa7` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/_archive/samples/demos/worlds/showcase/_fleet_cam_capture.omniworld` | world (load FAIL) | `6f495ca01` | validate-worlds 2026-09-02: Cannot open URDF file 'O:/omnisim/projects/_archive/samples/demos/worlds/showcase/../../../../robots/clearpath/husky_description/urdf/husky.urdf': thi |
| `projects/_archive/samples/demos/worlds/showcase/husky_fleet_arena_mkt.omniworld` | world (load FAIL) | `6f495ca01` | validate-worlds 2026-09-02: Cannot open URDF file 'O:/omnisim/projects/_archive/samples/demos/worlds/showcase/../../../../robots/clearpath/husky_description/urdf/husky.urdf': thi |
| `projects/_archive/samples/demos/worlds/showcase/husky_fleet_arena_topdown.omniworld` | world (load FAIL) | `6f495ca01` | validate-worlds 2026-09-02: Cannot open URDF file 'O:/omnisim/projects/_archive/samples/demos/worlds/showcase/../../../../robots/clearpath/husky_description/urdf/husky.urdf': thi |
| `projects/_archive/samples/demos/worlds/showcase/husky_fleet_arena_topdown_x2.omniworld` | world (load FAIL) | `6f495ca01` | validate-worlds 2026-09-02: Cannot open URDF file 'O:/omnisim/projects/_archive/samples/demos/worlds/showcase/../../../../robots/clearpath/husky_description/urdf/husky.urdf': thi |
| `projects/_archive/` | world sidecar | `6f495ca01` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/control/` | policy artefacts (unreferenced) | `d84077a53` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/control/gait/` | policy artefacts (unreferenced) | `d84077a53` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/control/gait/tools/` | policy artefacts (unreferenced) | `be41986f8` | 4 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/demos/` | policy artefacts (unreferenced) | `6f495ca01` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/ghosts/g1/` | policy artefacts (unreferenced) | `17d0645eb` | 35 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/ghosts/g1/dance/` | policy artefacts (unreferenced) | `17d0645eb` | 17 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/` | policy artefacts (unreferenced) | `1a60ff38f` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/envs/` | policy artefacts (unreferenced) | `be41986f8` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/inference/` | policy artefacts (unreferenced) | `be41986f8` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/inference/policies/gpu_b2_stand_ke1400/` | policy artefacts (unreferenced) | `056b26a8b` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/mpc/` | policy artefacts (unreferenced) | `17d0645eb` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/policies/omniquad_cpg_zero/` | policy artefacts (unreferenced) | `be41986f8` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/policies/omniquad_omnisim_main/` | policy artefacts (unreferenced) | `be41986f8` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/policies/omniquad_walk_mjx_main/` | policy artefacts (unreferenced) | `be41986f8` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/runners/` | policy artefacts (unreferenced) | `be41986f8` | 7 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/shadowing/` | policy artefacts (unreferenced) | `74fe6542c` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/shadowing/ghosts/` | policy artefacts (unreferenced) | `74fe6542c` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/tools/` | policy artefacts (unreferenced) | `1a60ff38f` | 21 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/` | policy artefacts (unreferenced) | `1a60ff38f` | 29 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/mjcf/` | policy artefacts (unreferenced) | `be41986f8` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_sitstand_track1/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk16_arms_a3/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk17_natural_n6/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk18_human_h12/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk20_winter_w5/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk22_swing_t5/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk23_style_v3/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk24_ab_base/` | policy artefacts (unreferenced) | `056b26a8b` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk24_ab_look/` | policy artefacts (unreferenced) | `056b26a8b` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk24_ab_stack4/` | policy artefacts (unreferenced) | `056b26a8b` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk25_long_c16/` | policy artefacts (unreferenced) | `056b26a8b` | 4 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk25_long_c17/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk27_lat_c1/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/research/training/runs/gpu_g1_walk27_lat_c4/` | policy artefacts (unreferenced) | `056b26a8b` | 3 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/skills/` | policy artefacts (unreferenced) | `d84077a53` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/policies/training/` | policy artefacts (unreferenced) | `74fe6542c` | 10 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/training/ablations/` | policy artefacts (unreferenced) | `dece36da4` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/policies/training/runs/` | policy artefacts (unreferenced) | `74fe6542c` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/alife/` | residue (unreached) | `b1d12d090` | 2 files, 0.00 MB; nothing in the tree names them |
| `projects/robolife/` | residue (unreached) | `b705267b0` | 5 files, 0.00 MB; nothing in the tree names them |
| `projects/robolife/controllers/robolife_probe_dock/` | residue (unreached) | `b1d12d090` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robolife/controllers/robolife_robot/` | residue (unreached) | `b705267b0` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robolife/controllers/robolife_world/` | residue (unreached) | `b1d12d090` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robolife/rl/` | residue (unreached) | `b705267b0` | 6 files, 0.00 MB; nothing in the tree names them |
| `projects/robolife/robots/` | residue (unreached) | `416b39599` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robolife/seeds/` | residue (unreached) | `416b39599` | 1 file, 0.00 MB; nothing in the tree names them |
| `projects/robolife/tests/` | residue (unreached) | `b1d12d090` | 2 files, 0.00 MB; nothing in the tree names them |
| `resources/projects/libraries/qt_utils/icons/` | residue (unreached) | `3b30359e0` | 5 files, 0.00 MB; nothing in the tree names them |

## Media

The same cleanup deleted 208 unreferenced image, preview and image-source files under `docs/` (64.6 MB; commits `b77fc1787` and `55375159f` list every path). `git show <commit>^:<path>` recovers any of them. `docs/tests/test_images.py` now fails on an orphaned PNG or thumbnail, so the set cannot silently regrow.

Regenerate a deleted page into a scratch file, never back into the tree: `git show <sha>:<path> > /tmp/<name>.md`.
