## Environments

This section shows some of the possible environments and objects available in OmniSim inside composed scenes.
OmniSim objects are modular and parametrizable, so other environments can be created simply, starting from these examples.
The world files for these examples are located in the "[OMNISIM\_HOME/projects/samples/demos/worlds/environments/]({{ url.github_tree }}/projects/samples/demos/worlds/environments/)" directory.

In this directory, you will find the following world files:

Each of these is an *environment* world — a hand-built backdrop with no mobile or agent robots. Demos that want a robot driving through them compose on top (copy the file and add a robot, or EXTERNPROTO its content). All three follow the canonical OmniSim lighting recipe (procedural sky + `OmniSimSun` + a draggable `SUN_MARKER`).

### [city.omniworld]({{ url.github_tree }}/projects/samples/demos/worlds/environments/city.omniworld)

**Keywords**: City, urban street block, buildings, intersection

A bright-midday urban street block: a 2-lane avenue running East–West crossed at the origin by a side street, lined with a mix of building PROTO nodes (Hotel, BigGlassTower, CommercialBuilding, Residential, CyberboticsTower). Sidewalks carry street lights, benches, a bus stop, fire hydrants, bins, a news stand, mailbox, phone booth, parking meters, street trees, and a four-way intersection with traffic lights and zebra crossings. A pocket park (lawn, stone fountain, oaks, benches) fills the NW plot. The flat asphalt surface and real sidewalks let a ground robot drive the street once one is composed in.

### [desert_ruins.omniworld]({{ url.github_tree }}/projects/samples/demos/worlds/environments/desert_ruins.omniworld)

**Keywords**: Desert, ancient ruins, golden hour, cinematic

A golden-hour desert scene built to push OmniSim's wgpu-native renderer. The backdrop is hand-built ancient architecture — a 4-tier ziggurat, an Egyptian-style pylon gate, a colossal marble colonnade (several columns collapsed), an obelisk, and a half-buried stone dome — plus haze-faded duplicates on the far horizon. The silhouettes are primitive composites (Box/Cylinder/Sphere) with mineral PBR appearances (Plaster, DryMud, Marble) rather than generic city building PROTOs.

### [forest.omniworld]({{ url.github_tree }}/projects/samples/demos/worlds/environments/forest.omniworld)

**Keywords**: Forest, woodland, deep mist, cinematic

A deep-mist morning forest, a deciduous–conifer mix with the same level of hand-built detail as the desert scene but tuned for woodland atmosphere. Designed as a backdrop for a husky / omniquad / etc. composed on top of the tree and floor content.
