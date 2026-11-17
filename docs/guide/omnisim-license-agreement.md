## License Agreement

### OmniSim Source Code

OmniSim is released under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
Apache 2.0 is an industry-friendly, non-contaminating, permissive open-source license that grants everyone the right to use, modify and redistribute the source code, free of charge, for any purpose, including commercial applications.

The full text ships in the distribution as [`LICENSE`](https://github.com/omnilink-tech/omnisim/blob/main/LICENSE).

OmniSim is a derivative work of [Webots](https://github.com/cyberbotics/webots) (Copyright 1996-2024 Cyberbotics Ltd.), which is itself Apache-2.0. Files inherited from Webots keep their original copyright headers and carry a modification notice; see [`NOTICE`](https://github.com/omnilink-tech/omnisim/blob/main/NOTICE). OmniSim is not affiliated with, endorsed by, or sponsored by Cyberbotics Ltd.

### Not Everything in the Distribution Is Apache-2.0

The Apache-2.0 grant covers OmniSim's own source code and the Webots-derived source code it builds on. It **cannot** grant rights in third-party material whose owners licensed it on other terms, and this distribution contains such material. **Before you redistribute OmniSim, read the two attribution files:**

- [`NOTICE`](https://github.com/omnilink-tech/omnisim/blob/main/NOTICE) — the short attribution file required by Apache-2.0 §4(d). Its **"Carve-Out"** section names, explicitly, every component in the tree believed to carry terms narrower than or incompatible with Apache-2.0.
- [`THIRD_PARTY_NOTICES.md`](https://github.com/omnilink-tech/omnisim/blob/main/THIRD_PARTY_NOTICES.md) — the full component-by-component inventory: robot models, vendored code, simulation assets, fonts, icons and reference data, with the license that applies to each and the path to its license text.

### Sample Simulations

The sample simulations — world files (`.wbt`), robot and object models (`.proto`, URDF), controllers, plugins and libraries — are **not uniformly Apache-2.0**.

Every PROTO file declares its own license on a `# license:` header line, and that line is authoritative for that file. Across the PROTO set as shipped, the declared licenses are a mix of:

- Apache License 2.0 (the majority, including every object model authored for OmniSim),
- Creative Commons Attribution 4.0 International (CC BY 4.0), copyright Cyberbotics Ltd.,
- Creative Commons Attribution 3.0 United States (two traffic PROTOs),
- MIT (two PROTOs),

plus a small number of files that carry no `# license:` header at all. The exact counts are tabulated in [`NOTICE`](https://github.com/omnilink-tech/omnisim/blob/main/NOTICE) and [`THIRD_PARTY_NOTICES.md`](https://github.com/omnilink-tech/omnisim/blob/main/THIRD_PARTY_NOTICES.md). **Check the header of the specific file you intend to reuse** — do not assume the license of a neighbouring file.

Some sample simulations were contributed by users under their own license agreements, mostly open-source ones. In each case the license is stated in the corresponding file or folder.

### Robot Models

Robot descriptions (URDF/MJCF) and 3D meshes that originate from third parties remain subject to their upstream license **and** to the rights of the manufacturer whose product they depict. Each such package ships the verbatim upstream license beside the geometry as a `LICENSE.upstream` file in the robot's directory — for example `projects/robots/clearpath/*/LICENSE.upstream` or `projects/robots/unitree/LICENSE.upstream`. The per-robot table in [`THIRD_PARTY_NOTICES.md`](https://github.com/omnilink-tech/omnisim/blob/main/THIRD_PARTY_NOTICES.md) lists the upstream source, the license, and the path to that text for every model in the distribution.

Redistribution here implies no endorsement by, or affiliation with, those manufacturers.

### Trademarks

The Apache License covers the source code only; it grants no rights in trademarks (Apache-2.0 §6). "OmniSim", "OmniLink", the OmniLink orb mark and the associated brand palette are trademarks of OmniLink — see the project's [`TRADEMARKS.md`](https://github.com/omnilink-tech/omnisim/blob/main/TRADEMARKS.md), which also states the rule that forks redistributing modified code must rename and rebrand. All manufacturer and product names used in the distribution are used nominatively, to identify the machine a model depicts.
