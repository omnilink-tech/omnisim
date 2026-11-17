# OmniLink brand assets: the parent half of one brand

> **The brand guide for both names is [`../omnisim/BRAND.md`](../omnisim/BRAND.md)** (approved 2026-11-16). It defines the relationship ("OmniSim, by OmniLink"), both marks, both wordmarks, the two fonts and the one palette. This file only lists what lives in this folder and what is retired here. If the two ever disagree, `../omnisim/BRAND.md` is right.

OmniLink is the company and its AI agents; OmniSim is OmniLink's open-source workshop where those agents work on robots and machines.

## What is current in this folder

- **The OmniLink mark is the circuit sphere**: [`svg/mark_yellow.svg`](svg/mark_yellow.svg) (mimosa), [`svg/mark.svg`](svg/mark.svg) (black), [`svg/mark_white.svg`](svg/mark_white.svg). These are the source artwork, drawn in a 1080 square with about 25 % padding on every side. **For layouts, use the tight copy** [`svg/mark_yellow_tight.svg`](svg/mark_yellow_tight.svg), which is the same as `../omnisim/svg/omnilink_mark.svg`, so the mark prints at its real size.
- The OmniLink wordmark and lockup are generated with the OmniSim ones, so the two names are set identically. They live in `../omnisim/svg/`: `omnilink_wordmark.svg`, `omnilink_lockup.svg`, and the co-lockup `lockup_omnilink.svg` ("OmniSim, by OmniLink"). Run `python scripts/branding/render_brand_system.py --png` to regenerate them.

## Retired here (kept on disk so old renders can be rebuilt; nothing new uses them)

- **The OmniLink dot orb**: `orb/orb.png`, `orb/orb_512.png` and `scripts/branding/render_omnilink_orb.py`. It has 220 surface and 80 interior dots, #F6E932 / #968A26.
- **The stacked "omni / link" lockups**: `svg/lockup_horizontal.svg`, `svg/lockup_horizontal_yellow.svg`, `svg/lockup_alt.svg` and `png/lockup_*.png`. Use `../omnisim/svg/omnilink_lockup.svg`.
- **The OmniLink palette** black #000000 / cream #EEEEE0 / #F6E932. The one palette is mimosa #F6E905 on near-black #0A0A06, with ink #F6F4EF.
- **Montserrat and Hogira.** The two brand fonts are Sora SemiBold and Inter, bundled under `../omnisim/fonts/` with their OFL licences. Montserrat stays here under its OFL. **Hogira has no redistribution licence**: it is excluded from public snapshots by `scripts/release/publish_deny.txt` and must never be added back to a redistributed tree.

## Regenerating GUI icons

The C++ GUI loads icons from `images:omnisim.png`, `icons/core/omnisim.png`, `icons/core/omnisim64x64.png`, `src/omnisim/gui/omnisim.ico`, etc. Since 2026-11-03 those files are derived from the **OmniSim** mark, not from `orb/orb.png` here — see [`../omnisim/BRAND.md`](../omnisim/BRAND.md#app-icons).

To refresh the OmniLink orb itself:

```bash
python scripts/branding/render_omnilink_orb.py     # → orb/orb.png + orb/orb_512.png
python scripts/branding/preview_orb.py             # → preview/orb_preview.png (for PRs)
```

The icon builder renders the orb fresh at each target icon size — never resizes the master — so the resolution-aware dot population fires per icon. Output sizes: 16, 24, 32, 48, 64, 128, 256 px PNGs plus multi-size Windows `.ico` files.

## Attribution

OmniSim is built on top of [Webots](https://github.com/cyberbotics/webots) by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics — they are not part of the OmniLink identity and must not be used to brand OmniSim. References to "Webots" in OmniSim's code, docs, or About box are upstream-engine attributions, not branding.
