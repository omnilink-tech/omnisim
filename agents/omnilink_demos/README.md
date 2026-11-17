# agents/omnilink_demos/ — OmniLink demos

Showcase demos of OmniLink driving a robot in OmniSim: an operator talks in
plain language, OmniLink turns it into gated tool calls, and the robot does
the work on real simulated physics. Each demo folder holds everything behind
it except the world and robot bridge (those live in `projects/`, where
OmniSim looks for them):

- `README.md` — what it shows, how to run it, the orders that work, and what
  it does **not** show
- the generator for its world, if the world is generated
- `film/` — the scripts that record and edit its demo video
- `evidence/` — the rigs and logs behind every number the README quotes

Videos are not committed (size); each README says where the film lives.

| Demo | Robot | World | Film |
|---|---|---|---|
| [X30 plant inspection](x30_plant_inspection/) | Deep Robotics X30 quadruped, real-contact scripted trot | [`showcase/x30_plant_inspection.omniworld`](../../projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld) | draft, 2 min 56 s |

Still to gather here: the Husky and PX4 x500 OmniLink demos (worlds
[`chat/omnilink_husky.omniworld`](../../projects/samples/demos/worlds/chat/omnilink_husky.omniworld)
and [`showcase/x500_arena.omniworld`](../../projects/samples/demos/worlds/showcase/x500_arena.omniworld)).

Film style: [`scripts/cinema/OMNILINK_DEMOS.md`](../../scripts/cinema/OMNILINK_DEMOS.md).
How OmniLink drives a robot: [`docs/guide/omnilink-chat-demos.md`](../../docs/guide/omnilink-chat-demos.md).
