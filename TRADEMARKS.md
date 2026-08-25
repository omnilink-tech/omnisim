# OmniSim & OmniLink Trademark Policy

OmniSim is open source under the Apache License 2.0 — the **code** is free for anyone to use, modify, fork, redistribute, and ship in commercial or academic products (see [LICENSE](LICENSE) and [NOTICE](NOTICE)). The **OmniSim and OmniLink brands**, however, are not part of that grant. This document explains what we protect, what is freely permitted, and what requires written permission.

The goal is simple: **the code is yours to build on; the brand identifies OmniLink's work and the OmniLink agentic platform it is built for.** If users see "OmniSim," it should mean the simulator OmniLink maintains. If users see the dot-sphere orb, it should mean OmniLink.

---

## What is protected

The following are trademarks of **OmniLink** ("the Marks"):

- **Word marks**
  - "OmniSim"
  - "OmniLink"
  - The tagline *"by OmniLink, for OmniLink agents"* (and close variants such as *"made by OmniLink, for OmniLink"*)

- **Logos and design marks**
  - The **OmniSim particle-orb** mark — the Fibonacci-lattice dot sphere — in all color variants, and its small-size **glyph** variant
  - The **OmniSim wordmark** (the `OMNI` + `SIM` lockup with the mimosa accent on `SIM`)
  - The **OmniLink connector monogram** (the dot-and-bar hexagonal mark) in all color variants
  - Any composite mark or lockup combining the above with the "OmniSim" or "OmniLink" wordmark

- **Trade dress and visual identity**
  - The OmniSim color palette as defined in [`resources/branding/omnisim/BRAND.md`](resources/branding/omnisim/BRAND.md) (near-black / ink / mimosa), and the OmniLink palette in [`resources/branding/omnilink/BRAND.md`](resources/branding/omnilink/BRAND.md) (black / cream / mimosa)
  - The OmniSim splash screen, About box layout, and GUI icon set as shipped in this repository under [`resources/branding/omnisim/`](resources/branding/omnisim/) and [`resources/branding/omnilink/`](resources/branding/omnilink/)

All rights in the Marks are reserved by OmniLink. The Apache 2.0 license covering the code does **not** grant any right to use the Marks (Apache 2.0 §6 explicitly excludes trademark rights).

### Two separate rights: the marks, and the artwork files

This policy governs the **trademark** — using the Marks to identify a product, project, service or distribution.

The **copyright in the artwork files themselves** is a different right, and it is reserved separately, in [`resources/branding/LICENSE`](resources/branding/LICENSE) and in carve-out item 5 of [`NOTICE`](NOTICE). Everything under [`resources/branding/`](resources/branding/) — logos, orb, wordmarks, lockups, glyphs, favicons — is © 2026 OmniLink, all rights reserved, and is **excluded from the Apache-2.0 grant**.

Both are stated because §6 on its own does not reach the files. It withholds the right to use a mark; it says nothing about copying a `.svg`. Without the separate copyright reservation, a recipient of this repository could copy, alter and republish the artwork so long as they stopped short of using it *as* a mark — which is not the intent. Reserving the copyright closes that gap; this policy then defines the generous permissions that apply on top of it.

**In practice:** reproducing the assets *unmodified* to refer factually to OmniSim or OmniLink is permitted without asking (see the next section). Altering them, or shipping them as another product's identity, is not.

---

## Marks we do NOT claim: Webots and Cyberbotics

OmniSim is an independent fork of [Webots](https://github.com/cyberbotics/webots), the open-source robotics simulator by **Cyberbotics Ltd.**, which Cyberbotics released under the Apache License 2.0. We are grateful for that work and we attribute it — see [NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

To be unambiguous about the boundary:

- **"Webots" and "Cyberbotics", and the Webots ladybug logo, are trademarks of Cyberbotics Ltd.** They are **not** OmniLink marks, they are **not** part of the Marks defined above, and OmniLink claims **no rights whatsoever** in them.
- Apache 2.0 **§6 grants no trademark rights**. Cyberbotics' Apache-2.0 release of the Webots *code* therefore gave us — and gives you — **no license to Cyberbotics' marks**. We do not have one, we do not sublicense one, and nothing in this document purports to grant you one.
- OmniSim is **not affiliated with, endorsed by, or sponsored by Cyberbotics Ltd.** We use the name "Webots" in this repository **only nominatively** — to state the factual origin of the code, to point at upstream documentation and issue trackers, and to identify inherited file formats and APIs (`.wbt`, which OmniSim still reads, the PROTO system, and the `WEBOTS_HOME` build alias the Makefiles still expand). ⚠️ The legacy `webots/` **include** forwarders are no longer among these examples — all 91 were deleted on 2026-08-16, along with the `controller` Python module alias, and the C++ namespace became `omnisim`. That is descriptive use, not a claim of ownership.
- We do **not** ship the Webots wordmark or ladybug logo as OmniSim branding, and neither should you. If you fork OmniSim, the same rule applies to Cyberbotics' marks as to ours: don't use them to brand your product.

If you are looking for permission to use the Webots or Cyberbotics marks, **we cannot give it** — ask [Cyberbotics](https://cyberbotics.com).

---

## What you may do without asking

These uses are always permitted and do not require permission:

1. **Use the unmodified software** under the terms of the Apache License 2.0, including in commercial products.
2. **Refer to OmniSim accurately and descriptively** — sometimes called *nominative fair use*. Examples that are fine:
   - "This agent runs on OmniSim."
   - "Compatible with OmniSim."
   - "Built using OmniSim by OmniLink."
   - "Ported from OmniSim."
   - A blog post, paper, talk, or video that discusses or reviews OmniSim and uses the name and (unmodified) logo for that purpose.
3. **Distribute unmodified copies** of OmniSim under the name "OmniSim" with the original branding intact, provided the LICENSE and NOTICE files are preserved.
4. **Include "OmniSim" in the name of a plugin, integration, or add-on** that genuinely works with OmniSim, as long as it is clear your project is not OmniSim itself and is not endorsed by OmniLink. Examples: `omnisim-ros2-bridge`, `awesome-omnisim`, `omnisim-cookbook`.
5. **Use the Marks in academic and journalistic contexts** — citations, comparisons, screenshots in papers, reviews — without prior approval.

---

## What requires written permission from OmniLink

The following uses are **not** permitted without prior written permission:

1. **Naming a fork or modified distribution "OmniSim"**, or any name that is confusingly similar (e.g., "OmniSim Pro", "OmniSim Plus", "OmniSim Cloud", "OmniSim Enterprise"). If you fork and redistribute modified code, you **must rename** the distribution and remove the dot-sphere mark from the splash screen, About box, and GUI. The underlying code remains Apache 2.0; only the brand needs to change.
2. **Using the dot-sphere orb mark** or any OmniLink composite logo to identify your own product, service, company, or organization, or in a way that suggests endorsement, sponsorship, or affiliation with OmniLink.
3. **Registering** "OmniSim", "OmniLink", confusingly similar names, or the dot-sphere mark as a trademark, domain name, social-media handle, npm/PyPI package, or company name.
4. **Selling merchandise** (t-shirts, stickers, hardware) bearing the Marks.
5. **Using the Marks in a way that is misleading, disparaging, or implies an official OmniLink product** when the product or service is not from OmniLink.
6. **Modifying the Marks** — altering colors, proportions, adding elements, or creating derivative logos.

If you redistribute a **modified** version of OmniSim, you must:

- Rename the distribution (e.g., "AcmeSim", not "OmniSim-Acme").
- Replace the splash screen, About-box branding, and GUI icons with your own.
- Remove the *"by OmniLink, for OmniLink agents"* tagline from user-facing surfaces.
- Keep the LICENSE and NOTICE files intact (Apache 2.0 §4 requires this) and add your own NOTICE entry for your modifications.
- You may, and should, state in your README or NOTICE that your project is "based on OmniSim" or "a fork of OmniSim by OmniLink" — that is nominative fair use under the rules above.

---

## Plugins, integrations, and the OmniLink platform

We **encourage** the community to build on top of OmniSim — controllers, robots, worlds, agents, bridges, training environments, content packs, integrations with other tools. You do not need permission to publish these, and you may reference OmniSim by name. Where helpful, you may use a badge like **"Built for OmniSim"** that we may publish under [`resources/branding/omnilink/`](resources/branding/omnilink/) for this purpose.

OmniSim is purpose-built as the simulation environment for the **OmniLink agentic AI platform** at [omnilink-agents.com](https://www.omnilink-agents.com). Agents built on OmniLink that use OmniSim as their simulation backend are the canonical, first-class use case — and that connection is part of the project's identity, which is why the brand is protected.

---

## Why we do this

OmniSim is intentionally permissive on the **code** so that anyone can build, fork, ship, sell, and remix it. The **brand**, by contrast, is how users tell OmniLink's work apart from forks, derivatives, and unrelated projects. Protecting the brand protects users from confusion, protects contributors from having their work attributed elsewhere, and lets OmniLink keep the "OmniSim" name meaningful over time.

This policy is closely modeled on the trademark policies of other open-source projects with strong brand stewardship — Mozilla (Firefox), the Rust Foundation, Signal, and the Linux Foundation — adapted to OmniSim's context.

---

## Questions, permission requests, and reporting misuse

For any of the following, contact OmniLink:

- Requesting written permission for a use that this policy reserves.
- Reporting a confusing or misleading use of the Marks.
- Asking whether a specific use is okay.
- Press, partnership, and merchandise inquiries.

**Contact:** **info@omnilink-agents.com** — the project's single point of contact, monitored by the maintainers. For anything that is not confidential you can equally open an issue on the [GitHub repository](https://github.com/omnilink-tech/omnisim/issues), which is usually faster and leaves a public record others can find. General project information: [omnilink-agents.com](https://www.omnilink-agents.com).

**Registration status:** the Marks are asserted as unregistered (common-law) trademarks — use ™ rather than ®. If that changes, this line changes with it.

We try to be reasonable. If your use is genuinely descriptive, non-misleading, and consistent with the spirit of this policy, we are likely to say yes — or to tell you that no permission is needed.

---

OmniSim and OmniLink are trademarks of OmniLink. **Webots and Cyberbotics are trademarks of Cyberbotics Ltd., used here nominatively to attribute the upstream project OmniSim is forked from; OmniLink claims no rights in them and is not affiliated with or endorsed by Cyberbotics.** All other trademarks referenced in this repository are the property of their respective owners; their mention does not imply endorsement.
