# OmniLink and OmniSim: the brand guide

**This file is the single brand guide for both names.** [`../omnilink/BRAND.md`](../omnilink/BRAND.md), the brand book [`OmniSim-Brand.html`](OmniSim-Brand.html) and [`../../../TRADEMARKS.md`](../../../TRADEMARKS.md) point here. If they disagree with this file, this file is right; fix them.

Approved by the owner on 2026-11-16 ("Yes, roll it out"). The goal: OmniLink's agents and OmniSim, their workshop, read as one entity. It should be simple and the same everywhere: the simulator, the website, documents and films.

## 1. What the two names are

- **OmniLink** is the company and its AI agents.
- **OmniSim** is OmniLink's open-source workshop. There, those agents design, build, test and debug robots and machines, and connect to the real one.
- **Where both appear, write "OmniSim, by OmniLink".**
- **One-line story:** "OmniLink builds agents that work on robots and machines; OmniSim is their workshop."
- **Claims:** every product claim still has to trace to [`docs/developer/positioning.md`](../../../docs/developer/positioning.md); never widen one.
  - Customer-facing pages do not describe architecture.
  - Never "first" or "only".
  - Sim-to-real covers the control surface only.
  - A "digital twin" is a model built from the robot's or machine's own description.
- **Retired lines:** "Where robot software gets debugged", "The simulator you can talk to" and "An open-world simulator for agents". The current OmniSim line is "an open-source robotics workshop for agents".

## 2. Names

| Write | Never |
|---|---|
| OmniLink | Omnilink, Omni Link, OMNILINK (outside legal all-caps blocks), omni link |
| OmniSim | Omnisim, Omni Sim, OMNISIM, Omni-Sim |
| OmniKey | Omni Key, Omnikey |

Code identifiers keep their spelling (`olink-` element prefix, `OLINK_*`, `olink_` keys, `omnisim` module). They are not prose.

## 3. Marks: two spheres, one family

| | OmniLink | OmniSim |
|---|---|---|
| Mark | **the circuit sphere**: [`svg/omnilink_mark.svg`](svg/omnilink_mark.svg) (tight frame), `omnilink_mark_ink.svg` for paper | **the particle orb**: [`svg/orb.svg`](svg/orb.svg), 250 dots on 14 rows, lit from the upper left; [`svg/glyph.svg`](svg/glyph.svg) (52 dots) at 128 px and below |
| Wordmark | [`svg/omnilink_wordmark.svg`](svg/omnilink_wordmark.svg) | [`svg/wordmark.svg`](svg/wordmark.svg) |
| Lockup (mark + name, side by side) | [`svg/omnilink_lockup.svg`](svg/omnilink_lockup.svg) | [`svg/lockup_horizontal.svg`](svg/lockup_horizontal.svg) |
| App icon | the website's favicon set | the simulator's icon set (decision of 2026-11-03), on its rounded #060604 tile |

- **Same optical size.**
  - In both lockups the sphere is 166 units across in a 240-unit-high frame, and the name's cap height is 84 units.
  - Put the two marks side by side at the same height and they read as a pair.
  - The circuit sphere's source artwork (`../omnilink/svg/mark_yellow.svg`) sits in a 1080 square with about 25 % padding on each side. Use the tight `omnilink_mark.svg` (`../omnilink/svg/mark_yellow_tight.svg` is the same file), so the mark prints at its real size.
- **Both brands together:** [`svg/lockup_omnilink.svg`](svg/lockup_omnilink.svg), reading "OmniSim, by OmniLink".
  - It is both lockups at full weight, OmniSim first and OmniLink second, with a hairline between them.
  - In a nav bar or page header, the speaker comes first. The website leads with OmniLink; the simulator and its docs lead with OmniSim.
- **Retired. Do not use on any new surface:**
  - the old OmniLink dot orb (`../omnilink/orb/`, 220 + 80 dots);
  - the stacked "OMNI / LINK" lockups (`../omnilink/svg/lockup_horizontal*.svg`, `lockup_alt.svg`, `../omnilink/png/lockup_*.png`);
  - every all-caps wordmark (OMNISIM, OMNILINK, OMNI/SIM, OMNI/LINK);
  - the ring-and-dot "OmniLink mark" the old co-lockup drew.

  The files stay in the tree so old renders can be rebuilt; nothing new points at them.
- **Don'ts, for both marks:**
  - don't rotate (the orb is lit from the upper left);
  - don't recolour (mimosa is the only accent);
  - don't squash or stretch;
  - don't put a mark on mimosa;
  - no gradient under a mark.

## 4. Wordmarks

- **Mark + name, side by side.** Both names are set identically:
  - [Sora](https://fonts.google.com/specimen/Sora) SemiBold (600), mixed case, Sora's own kerning, no added tracking;
  - "Omni" in ink #F6F4EF and the second word ("Sim", "Link") in mimosa;
  - on paper, use the `_mono_dark` versions.
- **The SVGs carry the letters as outlines.** Never retype a wordmark in a fallback font; use the SVG or a PNG rendered from it.
- **Regenerate:**
  - `python scripts/branding/outline_omnisim_wordmark.py` regenerates "OmniSim".
  - `python scripts/branding/render_brand_system.py --png` regenerates "OmniLink", the OmniLink lockup, the co-lockup, the static and web fonts, and `png/`.

## 5. Typography: two families, both OFL

| Role | Family | Files |
|---|---|---|
| Names, headings, titles, big numbers | **Sora SemiBold (600)** | `fonts/Sora.ttf` (variable), `fonts/static/Sora-SemiBold.ttf`, `fonts/web/Sora-SemiBold.woff2`; licence `fonts/OFL-Sora.txt` |
| Body, UI (website, simulator Qt interface, PDFs, film captions) | **Inter** 400 / 500 / 600 | `fonts/Inter.ttf` (variable, opsz + wght), `fonts/static/Inter-{Regular,Medium,SemiBold}.ttf`, `fonts/web/*.woff2`; licence `fonts/OFL-Inter.txt` |

- **Code and terminal text** use the platform's monospace (`ui-monospace, Consolas, "DejaVu Sans Mono", monospace`). It is not a brand face.
- **Retired:**
  - Space Grotesk;
  - JetBrains Mono as a brand face;
  - Montserrat;
  - **Hogira**. It has no redistribution licence and is kept out of public snapshots by `scripts/release/publish_deny.txt`.

  The files stay where they are, so old films can be rebuilt; nothing new loads them.
- **Fallbacks:** `Sora, Inter, ui-sans-serif, system-ui, sans-serif` for headings; `Inter, ui-sans-serif, system-ui, "Segoe UI", sans-serif` for body.

## 6. Colour: one mimosa, one near-black

| Token | Hex | Use |
|---|---|---|
| **mimosa** | **#F6E905** | The one accent: marks, the second word of each name, calls to action, links, highlights |
| **near-black** | **#0A0A06** | The one canvas: page and slide backgrounds, dark panels, the ink on mimosa fills |
| ink | #F6F4EF | Text and headings on near-black |
| ink-mute | #B4B3A4 | Secondary text on near-black |
| mimosa-dim | #D4C500 | Hover and pressed states; a yellow that has to sit on white paper |
| surface | #14140D | Cards and panels on near-black |
| sunk | #060604 | Wells, code, terminals, the app-icon tile |

- **Mimosa is #F6E905**, the value the OmniSim identity always defined.
- **Retired yellows:**
  - #F6E932, the old OmniLink orb's dots;
  - #e1d256, the website's softer gold, with #b9a73f and #f3edb3;
  - #fbbf24, amber;
  - #f5d565;
  - every other one-off yellow.
- **Retired blacks:** #000000 as a surface, #11120f, #111109 and #0c0c0d. Use near-black.
- **On white paper** (PDF documents), text is near-black or #14140E. Yellow on white is mimosa-dim, or mimosa as a fill under near-black text, never as thin text.
- **Contrast:** mimosa on near-black is about 15:1. Near-black on mimosa is the button pairing.

## 7. Applying it

- **Website** (`O:\omnilink`): the header is the OmniLink lockup (mark + "OmniLink" in Sora). The favicon is the circuit sphere on near-black. The colour and font tokens above live in `branding/colors.css` and `public/site-system.css`. The OmniSim page uses the OmniSim lockup.
- **Simulator:**
  - The app icon is the orb on its tile (`python scripts/branding/build_omnilink_icons.py`).
  - The Qt interface uses Inter. It is loaded from `resources/fonts/` by `OmGuiApplication`.
  - The splash shows the orb and the OmniSim wordmark, with "by OmniLink" set in Inter under it.
  - The About box says "OmniSim, by OmniLink".
- **Documents** (proposals, reports): page headers carry the OmniLink lockup and the OmniSim lockup at the same height, with a hairline between them. Headings are Sora, body is Inter, and the palette is the one above.
- **Films:** title and end cards use Sora for names and titles and Inter for captions, on near-black with mimosa accents. The end slate is the co-lockup. See `social/VIDEO_STYLE.md`.
- **Terminal output:** mimosa `◐ ◑ ◒` progress glyphs, mimosa-soft #FAF6A4 `✓`, ink-mute telemetry (`scripts/cinema/brand.py`).

## 8. The orb, the glyph and the app icons (engineering)

| Mark | Rows | Dots | Radius (sphere r = 100) | Hemispheres |
|---|---|---|---|---|
| Orb | 14 | 250 | 1.2–5.0 | both; far dots at 0.1–0.4 opacity |
| Glyph | 8 | 52 | 3.6–9.6 | near side plus the rim |
| Micro (16–24 px icons only) | 5 | 19 | 8–14 | as the glyph |

- **The orb is a generated lattice.** It lives in `ORB_LATTICE` / `GLYPH_LATTICE` in [`render_omnisim_orb.py`](../../../scripts/branding/render_omnisim_orb.py). Change the lattice and run `--regenerate-svg`; never hand-edit the circles.
- **App icon tile:** #060604, corners 21 % of the side, a soft mimosa vignette at the upper left, a hairline rim from 32 px up.
- **Which mark at which size:** the orb above 128 px, the glyph from 128 down to 32, the micro lattice at 24 and 16.
- **Regenerate the icons:**

```bash
python scripts/branding/render_omnisim_orb.py --regenerate-svg   # only after changing the lattice
python scripts/branding/render_omnisim_orb.py                    # orb/, glyph/, preview/, the splash orb
python scripts/branding/render_omnisim_wordmark.py               # wordmark/*.png, resources/images/splash_wordmark.png
python scripts/branding/build_omnilink_icons.py                  # every shipped app icon
python scripts/branding/render_brand_system.py --png             # the OmniLink half, co-lockup, fonts, png/
```

The `.ico` reaches `omnisim-bin.exe` through `windres`, so rebuild the engine to see it on a Windows desktop.

## 9. Files

```
branding/omnisim/                  ← the brand guide for both names (this folder)
├── BRAND.md                       ← this file (canonical)
├── OmniSim-Brand.html             ← the brand book, rendered from this file's rules
├── svg/
│   ├── orb.svg, glyph.svg, favicon.svg          OmniSim mark
│   ├── wordmark.svg (+ _mono_light, _mono_dark) "OmniSim"
│   ├── lockup_horizontal.svg                    OmniSim lockup
│   ├── lockup_stacked.svg                       orb over wordmark over "by OmniLink" (splash, posters)
│   ├── omnilink_mark.svg, omnilink_mark_ink.svg OmniLink mark, tight frame
│   ├── omnilink_wordmark.svg (+ mono)           "OmniLink"
│   ├── omnilink_lockup.svg                      OmniLink lockup
│   └── lockup_omnilink.svg                      "OmniSim, by OmniLink" co-lockup
├── png/                           ← the lockups and marks above, rendered by render_brand_system.py --png
├── orb/, glyph/, wordmark/        ← OmniSim rasters
└── fonts/
    ├── Sora.ttf, Inter.ttf        variable masters
    ├── static/                    Sora-SemiBold, Inter-Regular/Medium/SemiBold (.ttf)
    ├── web/                       the same as .woff2
    └── OFL-Sora.txt, OFL-Inter.txt
```

## Attribution

OmniSim is built on top of [Webots](https://github.com/cyberbotics/webots) by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics. They are not part of the OmniSim or OmniLink identity and must not be used to brand either.
