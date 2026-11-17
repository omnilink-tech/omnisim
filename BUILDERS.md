# OmniSim Builder Challenge

Build one small, measurable thing in OmniSim and show enough evidence that
another person can reproduce it. There is no entry fee, deadline, or paid
prize. Useful entries are linked from the project and may become maintained
demos, benchmarks, or documentation with the contributor's permission.

The challenge is intentionally not a screenshot contest. A scene can look
excellent while its robot falls through the floor, its controller never ticks,
or its success detector measures the wrong thing.

## Choose one lane

### First edit

Start from any shipped demo, make one visible change, and prove the edited
world loads. This is the right lane for a first contribution and should fit in
one sitting.

Minimum evidence:

- the exact prompt or commands you used;
- `python -m omnisim doctor` output;
- the changed world or a small patch;
- a screenshot and the load-check command.

### Robot or task

Add one robot, sensor, controller behavior, or bounded task. Define success in
physical units or an observable state before implementing it.

Minimum evidence:

- model and asset provenance;
- a deterministic launch command;
- success and failure criteria;
- one positive run and one useful negative control;
- known limitations, including anything that only works visually.

### Agent benchmark

Contribute a simulator-neutral task to
[`tests/benchmarks/agentbench/`](tests/benchmarks/agentbench/) that asks whether
an agent can get a real job done from one sentence. This is the most valuable
lane for researchers: externally authored tasks reduce the benchmark owner's
task-selection bias.

Minimum evidence:

- a task prompt that does not name a preferred implementation;
- a grader expressed in observable state or physical units;
- a fake-simulator test for the harness and grader path;
- a disclosure of any assets, models, or evaluation data supplied by the
  contributor.

## How entries are reviewed

Reviewers look for reproducibility, measurement quality, honest limits, and a
small maintenance surface. Visual polish is welcome but does not replace a
behavioral check. A result that demonstrates a real engine limitation can be
as useful as a successful demo.

Use the [Builder Challenge issue form](https://github.com/omnilink-tech/omnisim/issues/new?template=builder_challenge.yml)
to enter. If you need OmniLink to do the initial porting work, use
[Request a Sim](https://github.com/omnilink-tech/omnisim/issues/new?template=request_a_sim.yml)
instead.

Contributions remain under their submitted license and must be compatible with
the repository's Apache-2.0 distribution. Do not submit private, restricted,
or ambiguously licensed robot assets.
