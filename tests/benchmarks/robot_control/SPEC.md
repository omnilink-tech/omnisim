# Robotics control benchmark v1 protocol

Status: development protocol. The bundled tasks were authored while implementing this harness and include previously observed failure classes. No headline measurement is preregistered by this document alone; each campaign has an immutable lock containing its task pack and implementation hashes.

## Question and scope

Compare robotic-control integrations on identical simulator jobs. A success means the measured robot completed every operator turn with the required physical outcome and response status. The unit is an episode, not a model response or a tool's acceptance flag. The initial version covers two wheeled bases and one arm, not fleet coordination, vision-based navigation, autonomous exploration, real hardware or full hosted products.

Every implementation receives the same tool schema, measured initial state, previous conversation and tool feedback. The model can emit a whole sequential batch; it is never forced to call a model per motor action. Upon failure the executor halts the batch and issues Stop, then returns the failure to the planner. Recovery decisions belong to the agent; the executor does not retry. Six planning rounds and 24 actions per operator turn are the shared bounds. A 90-second turn deadline prevents new dispatches; an in-flight bounded HTTP request can finish later and its full elapsed time remains charged.

The engine runs in FAST mode. Reported latency is wall time for an accelerated simulation workflow, including model waiting time; it is not a physical robot's production cycle time. Robot observations retain simulation time as a separate clock. Engine startup/setup is separately recorded and excluded from task latency. This convention must accompany any published time comparison.

The shared bridge gate applies to all arms and gets the verbatim operator prompt. Shared safeguards receive no framework-specific credit. The plain and framework arms are named implementations, not representative of every configuration of those frameworks. Parser-enabled framework controls expose the value of the deterministic component directly. Those controls must not be hidden when they tie or beat OmniLink.

From pilot-02, malformed model plans receive protocol-error feedback and may be repaired within the same six-round limit. The failed response consumes a round, time and tokens. This applies equally to every implementation, and feedback repeats the operator instruction and JSON contract. Pilot-01 used fail-fast parsing; keep that result separately and do not pool the two versions.

## Fixtures and oracles

Each episode uses a fresh engine and controller session to prevent residual momentum, memory, deferred intent or manipulated-object state from leaking between arms. Setup consists of logged, unscored typed motions; operator turns then execute continuously without reset. All robot motion uses the common gated tool endpoint. Setup has no language utterance because it is fixture control, not an agent instruction.

Initial poses are measured, including nonzero translated/rotated states. Expected mobile waypoints are computed relative to that measured initial pose. Arm targets are measured joint vectors, payload identity and the part's own world position. OmniArm 6's bundled chat world uses a simulator-assisted grasp attachment; this benchmark evaluates agent use of that primitive, not friction-only grasp physics.

Mobile completion requires ordered waypoint visitation, correct final pose and total completed-segment distance/rotation within max(0.08, 15% of requested total). Pose tolerance is 0.08 metres/radians. Arm joint tolerance is 0.07 radians; object destination tolerance is 0.06 metres. The trajectory observer samples at 30 ms wall intervals and retains tool-boundary measurements. It is not sub-step continuous monitoring. Sampling errors invalidate observations instead of certifying no motion.

Non-action turns must remain within 0.03 metres/radians (arm joints: 0.03 radians), with no payload attachment change on the arm. Pose reports use structured measured coordinates/joints, not keyword or first-number heuristics. These tolerances establish a test criterion, not a safety certification.

The pick oracle requires a live holding flag, matching measured attachment identity from the completed command, and more than 0.05 metres of observed lift of that named part. Release requires an empty live holding flag plus the requested measured destination. Joint samples alone cannot establish a successful pick. Payload identity is bridge-reported; independent simulator instrumentation would strengthen this measurement.

Injected recovery failures happen before motion and explicitly say moved=false. Transient cases inject the first physical failure; persistent cases inject every physical attempt. Grading verifies the requested operation was attempted, the registered retry count, and absence of duplicated successful motion. The agent does not see the fault schedule or oracle.

## Accounting and statistical rule

Keep PASS, FAIL and ERROR in the attempt denominator. Primary cost is the sum of derived model costs for **all** attempts divided by successful episodes. Primary time uses all task elapsed time, including failed attempts, divided by successes. Unknown usage is unknown, never zero. Also publish completion rate, unwanted-motion count, model requests, robot breakdowns and raw traces. Zero successes yields undefined cost/time per success.

From campaign 04, a precisely identified Google high-demand 503 rejection gets
one bounded transport retry with the original request retained and waiting time
charged to the episode. Usage stays null. Its estimated zero token charge is an
explicit application of Google's published failed-request billing policy, not
measured usage or a checked invoice. The lock and CAMPAIGN_04.md define the exact
classification. Publish per-arm rejection counts and a $0.01/rejection sensitivity
estimate. Other missing usage still stops spending. Earlier runs are unchanged.

Pair task/robot/repeat across implementations. Resample task **families**, retaining all robot variants, both condition branches and repeats in the block, using 2,000 bootstrap draws and seed 240926. Intervals describe the registered authored task distribution on this machine and model. They are not a population-wide guarantee. Report the number of independent families; the small pilot is not powered for a leadership claim.

The relative rule requires the upper 95% bound of cost-per-success ratio below 1, upper bound of time-per-success ratio below 1, and lower bound of success difference above -3 percentage points. Every draw must have defined comparison metrics. This is an efficiency/non-inferiority rule, not proof of higher success. Per-competitor findings must not become an untested universal claim.

A leadership claim is ineligible if coverage is incomplete/duplicated, source/dependency integrity fails, evidence/lock digests differ, the run is calibration/development, observation/cost data is missing, or OmniLink exhibits any unwanted motion. Passing this criterion is not a complete product release certification. All reported results remain visible even when claims are blocked.

## Reproducibility and limitations

The lock records sources, selected robot assets, executable/controller hashes, library versions, model/profile, price assumptions and machine information. The deployed model-service revision and internal retries are not exposed by the shared transport. Requested temperature and output limit are not certified as provider-enforced. Runtime package versions and selected binary hashes do not attest every transitive binary byte; archive the engine release/runtime bundle for external reproduction.

Do not modify a frozen campaign to repair a product, grader or task. Preserve the old data, document the defect, and run a new version. A corrected secondary analysis must be labelled as such. An externally authored holdout is spent once inspected for development.
