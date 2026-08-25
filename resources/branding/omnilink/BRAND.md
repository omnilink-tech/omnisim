# OmniLink brand assets — the parent brand, and the GUI icon pipeline

This directory is the single source of truth for the **OmniLink** visual identity inside the OmniSim repo, and it is what the **shipped GUI/packaging icon pipeline currently renders from**. To refresh the GUI branding, edit the source files and rerun the two scripts under [Regenerating GUI icons](#regenerating-gui-icons) below.

> ### ⚠️ Scope — read this before calling anything here "the OmniSim mark"
>
> **This directory is NOT the authority for the OmniSim identity.** OmniSim has its own brand — its own particle orb, its own wordmark, its own surface palette — and it lives in [`../omnisim/BRAND.md`](../omnisim/BRAND.md). **That file wins** for anything OmniSim-branded: READMEs, docs, social, brand book, marketing, new product surfaces.
>
> Use *this* directory for two things only:
> 1. **The OmniLink parent brand** — the connector monogram, the OmniLink wordmark and lockups.
> 2. **The legacy GUI/packaging icon pipeline** — `render_omnilink_orb.py` + `build_omnilink_icons.py` still generate the app icon, taskbar icon, About-box logo, splash and packaging icons that the C++ GUI consumes.
>
> **Known gap:** because of (2), the *shipped* OmniSim app icon is today the **OmniLink** orb (220-dot procedural lattice, `#F6E932`), not the **OmniSim** orb (340-dot hand-curated SVG, `#F6E905`) defined in [`../omnisim/`](../omnisim/). The two are similar but not the same asset, and the product's primary surface therefore carries the parent brand rather than the product brand. Closing this means repointing `build_omnisim_icons` at `../omnisim/svg/orb.svg` + `glyph.svg` and regenerating `resources/icons/core/` — a deliberate visual change to the shipped binary, so it is **not** done implicitly. Until it is, this file documents what actually ships.

OmniSim is the simulator built **by OmniLink, for OmniLink agents**. The visuals reflect that: the OmniSim and OmniLink brands are both first-party and lock up together; the upstream Webots engine is acknowledged in attribution lines and license files only, and Cyberbotics' marks are never used to brand either.

## Identity at a glance

- **Primary visual — the orb.** A spherical Fibonacci lattice of matte Mimosa-yellow dots, matching the static reference on the OmniLink home page. This is the **OmniLink** orb, and it is the mark the GUI/packaging icon pipeline currently rasterizes (see the scope note above). For the **OmniSim** orb — the product's own mark — see [`../omnisim/svg/orb.svg`](../omnisim/svg/orb.svg). Source: [`orb/orb.png`](orb/orb.png) (1024×1024 RGBA, transparent background).
- **Connector monogram (secondary).** The dot-and-bar hexagonal mark from the OmniLink wordmark. Use [`svg/mark.svg`](svg/mark.svg) (mono black) or [`svg/mark_yellow.svg`](svg/mark_yellow.svg) (Mimosa) for non-icon contexts where a flat geometric mark is needed: favicons for the docs site, embedded badges, marketing lockups.
- **Wordmark lockup.** `omni / link` stacked beside the connector mark. Use [`svg/lockup_horizontal.svg`](svg/lockup_horizontal.svg) on light backgrounds, [`svg/lockup_horizontal_yellow.svg`](svg/lockup_horizontal_yellow.svg) on darker ones.

## The orb

Rendered programmatically by [`scripts/branding/render_omnilink_orb.py`](../../../scripts/branding/render_omnilink_orb.py):

- 220 surface dots distributed via the spherical-Fibonacci lattice (golden-angle method) — uniform coverage of the sphere surface
- 80 interior dots inside the 0.92-radius shell, dimmer and slightly smaller, visible through the front face for a subtle 3D read
- Each dot is a clean anti-aliased filled circle — **no bloom, no halo, no central glow**
- Front-facing dots are larger (8 px @ z=+r) and bright Mimosa (`#F6E932`); back-facing dots are smaller (2.4 px @ z=-r) and muted (`#968A26`)
- Subtle perspective scaling (4 % at the poles) gives a hint of depth without breaking the orthographic flatness of the reference

The renderer is **resolution-aware** — see `_params_for(resolution)`. At small icon sizes it drops the dot count and bumps the relative dot radius so the orb stays legible on dark taskbars. The buckets are:

| Resolution | Surface dots | Interior dots | Front dot px | Back dot px |
|---|---|---|---|---|
| ≥ 512 | 220 | 80 | proportional | proportional |
| ≥ 192 | 140 | 40 | 4.5 px floor | 1.6 px floor |
| ≥ 64 | 80 | 0 | 2.6 px floor | 1.4 px floor |
| < 64 | 46 | 0 | 1.7 px floor | 1.2 px floor |

Without this, a 1024-px lattice downsampled to 32 px would put each dot at sub-pixel scale and Lanczos averaging would dim the icon to near-invisibility. With it, the orb reads cleanly all the way down to 16 px.

## Color tokens

| Token | Hex | RGB | Use |
|---|---|---|---|
| `omnilink.black` | `#000000` | 0,0,0 | Primary text, default surface |
| `omnilink.cream` | `#EEEEE0` | 238,238,224 | Light background, paper-toned panels, splash text |
| `omnilink.mimosa` | `#F6E905` | 246,233,5 | Accent, links, highlights, the orb dots |

The Qt stylesheets ([`resources/omnisim_classic.qss`](../../omnisim_classic.qss), [`resources/omnisim_dusk.qss`](../../omnisim_dusk.qss), [`resources/omnisim_night.qss`](../../omnisim_night.qss)) and the splash screen pull these values directly. Do not introduce new accent colors without updating this file.

## Typography

- **Display / branding** — Hogira (Black, Bold, ExtraLight) — splash titles, marketing surfaces
- **UI / body** — Montserrat (Light, Regular, Medium, SemiBold, Bold) — every Qt widget label

**Montserrat is bundled in [`fonts/`](fonts/)** under the SIL Open Font License 1.1 (© 2011 The Montserrat Project Authors; licence text: [`fonts/LICENSE-Montserrat-OFL-1.1.txt`](fonts/LICENSE-Montserrat-OFL-1.1.txt)). It is freely redistributable, so it travels with the source tree.

**The Hogira display face is NOT redistributed.** No licence grant accompanies it — the font files carry no copyright, licence or vendor record in their own OpenType metadata, and no EULA was ever filed alongside them — so it is excluded from the public snapshot via `scripts/release/publish_deny.txt` and its files are absent from public distributions. It remains available in the private tree for OmniLink's own branding and video work. If you are building from a public checkout, the display face is simply not there: substitute Montserrat Bold/Black for display titles, or supply your own licensed display face. Do not add Hogira, or any other retail typeface, back into a redistributed tree without a written redistribution grant.

## File layout

```
branding/omnilink/
├── BRAND.md                              ← this file (canonical brand spec)
├── orb/
│   ├── orb.png                           1024×1024 RGBA master (the icon)
│   └── orb_512.png                       512×512 cached half-size
├── preview/
│   └── orb_preview.png                   icon-size preview sheet for PRs
├── svg/                                  secondary marks (connector + lockups)
│   ├── mark.svg                          primary connector mark, mono black
│   ├── mark_yellow.svg                   primary connector mark, Mimosa
│   ├── mark_white.svg                    primary connector mark, white-on-dark
│   ├── lockup_horizontal.svg             mark + wordmark, mono
│   ├── lockup_horizontal_yellow.svg      mark + wordmark, Mimosa
│   └── …
├── png/                                  raster fallbacks for the secondary marks
└── fonts/
    ├── Montserrat-{Light,Regular,Medium,SemiBold,Bold}.{otf,ttf}
    ├── LICENSE-Montserrat-OFL-1.1.txt    SIL OFL 1.1 (see Typography above)
    └── (the Hogira display face lives here in the private tree only —
         it is not redistributed; see Typography above)
```

## Regenerating GUI icons

The C++ GUI loads icons from `images:omnisim.png`, `icons/core/omnisim.png`, `icons/core/omnisim64x64.png`, `src/omnisim/gui/omnisim.ico`, etc. Those files are **derived** from `orb/orb.png` here.

To refresh after editing the orb renderer:

```bash
python scripts/branding/render_omnilink_orb.py     # → orb/orb.png + orb/orb_512.png
python scripts/branding/build_omnilink_icons.py    # → every GUI icon path
python scripts/branding/preview_orb.py             # → preview/orb_preview.png (for PRs)
```

The icon builder renders the orb fresh at each target icon size — never resizes the master — so the resolution-aware dot population fires per icon. Output sizes: 16, 24, 32, 48, 64, 128, 256 px PNGs plus multi-size Windows `.ico` files.

## Attribution

OmniSim is built on top of [Webots](https://github.com/cyberbotics/webots) by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics — they are not part of the OmniLink identity and must not be used to brand OmniSim. References to "Webots" in OmniSim's code, docs, or About box are upstream-engine attributions, not branding.
