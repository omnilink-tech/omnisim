# OmniSim brand assets

This directory is the canonical source of truth for the **OmniSim** visual identity — distinct from the parent [OmniLink](../omnilink/BRAND.md) brand. The full identity is documented in [`OmniSim-Brand.html`](OmniSim-Brand.html); this README is the engineering crib-sheet.

> **Precedence.** For anything OmniSim-branded — READMEs, docs, social, the brand book, marketing, new product surfaces — **this directory wins** over [`../omnilink/`](../omnilink/). The OmniLink directory is the authority for the *parent* brand (the connector monogram, the OmniLink wordmark) and for the *legacy GUI/packaging icon pipeline* only. If the two disagree about "the OmniSim mark", this file is right.
>
> **⚠️ Known gap — the shipped app icon is still the OmniLink orb.** `scripts/branding/build_omnilink_icons.py` renders the *OmniLink* orb (220-dot procedural lattice, `#F6E932`) into every icon path the C++ GUI and the packagers consume — `resources/images/omnisim.png`, `resources/icons/core/omnisim.png`, `omnisim_doc.ico`, `scripts/packaging/omnisim*.png`. So the OmniSim orb defined *here* (340-dot hand-curated SVG, `#F6E905`) is **not** what the product actually displays on the taskbar, splash, or About box. Closing this is a deliberate visual change to the shipped binary — repoint the icon builder at [`svg/orb.svg`](svg/orb.svg) + [`svg/glyph.svg`](svg/glyph.svg) and regenerate — so it is not done silently. Track it before claiming the OmniSim identity is fully applied in-product.

OmniSim is the simulator built **by OmniLink, for OmniLink agents**. OmniSim has its own mark (the particle orb), its own wordmark, and its own surface palette (near-black + mimosa). It locks up *with* the OmniLink mark in co-branded contexts; it does not borrow OmniLink's connector monogram.

> **Open the brand book.** [`OmniSim-Brand.html`](OmniSim-Brand.html) is a self-contained Claude artifact bundle — open it in any browser to see every page rendered with the actual fonts, the dot lattice at full fidelity, and live in-context mocks (favicon, app icon, nav bar, terminal). This file is the authority; the SVGs and PNGs in this directory are rasterizations of it.

## Identity at a glance

- **The mark — the orb.** A particle sphere of ~340 mimosa-yellow dots on a Fibonacci lattice, depth-shaded so the front face reads brighter and bigger than the back. Lit from upper-left. The orb is the canonical OmniSim mark; the wordmark is secondary. Source: [`svg/orb.svg`](svg/orb.svg).
- **The glyph — the small-size variant.** ~110 bolder dots in the same arrangement. Use below 24 px (favicons, taskbar, scaled-down chat avatars) where the full orb dissolves. Source: [`svg/glyph.svg`](svg/glyph.svg).
- **The wordmark — `OMNI`+`SIM`.** Space Grotesk Medium, uppercase, +0.08em tracking. `OMNI` stays ink (`#F6F4EF`); `SIM` carries the mimosa accent (`#F6E905`). Source: [`svg/wordmark.svg`](svg/wordmark.svg), plus mono variants for light and mimosa surfaces.
- **The horizontal lockup.** Orb + wordmark side by side. Default for nav bars, headers, READMEs. Source: [`svg/lockup_horizontal.svg`](svg/lockup_horizontal.svg).
- **The stacked lockup.** Orb above wordmark above tagline (*— an open-world simulator for agents*). For splash screens, app icons, posters, social cards. Source: [`svg/lockup_stacked.svg`](svg/lockup_stacked.svg).
- **The OmniLink × OmniSim co-lockup.** OmniLink connector mark + `OmniLink / OmniSim` for surfaces where the parent brand and product brand both need parity (docs nav, product cards). Source: [`svg/lockup_omnilink.svg`](svg/lockup_omnilink.svg).

## The orb

The orb is a designed asset — **not** a procedurally regenerated lattice. Its exact dot positions and per-dot radii live in [`svg/orb.svg`](svg/orb.svg) and are the source of truth. The Python renderer rasterizes that SVG; it does not synthesize a new sphere.

| Layer | Dot count | Radius range | Opacity | Role |
|---|---|---|---|---|
| Back (z<0) | ~140 | 0.7–2.3 px | 0.15–0.4 | Implies the far hemisphere |
| Mid (z≈0) | ~140 | 0.9–3.0 px | 0.5–0.8 | The equator band |
| Front (z>0) | ~60 | 1.4–3.0 px | 0.7–1.0 | The lit, near face |

The asymmetric brightness gradient — upper-left quadrant denser and slightly larger — implies a single light source without rendering one. Do not "fix" this asymmetry; it is the read.

Below 24 px the full orb dissolves into noise. Switch to the glyph variant ([`svg/glyph.svg`](svg/glyph.svg)) which keeps the same visual DNA with ~70 % fewer, ~2× larger dots.

## Color tokens

| Token | Hex | RGB | Use |
|---|---|---|---|
| `omnisim.mimosa` | `#F6E905` | 246,233,5 | Accent, marks, CTAs |
| `omnisim.mimosa-dim` | `#D4C500` | 212,197,0 | Hover, pressed |
| `omnisim.mimosa-soft` | `#FAF6A4` | 250,246,164 | Captions on dark |
| `omnisim.ink` | `#F6F4EF` | 246,244,239 | Body, headings |
| `omnisim.ink-mute` | `#B4B3A4` | 180,179,164 | Secondary text |
| `omnisim.surface` | `#14140D` | 20,20,13 | Cards, panels |
| `omnisim.bg` | `#0A0A06` | 10,10,6 | Canvas |
| `omnisim.sunk` | `#060604` | 6,6,4 | Wells, code, terminals |

> **Note on mimosa.** OmniSim mimosa is `#F6E905` — the vivid yellow on the orb. Earlier code shipped a warmer amber (`#fbbf24`) in `scripts/cinema/brand.py`; that has been migrated. New surfaces must use `#F6E905`.

No gradients on the brand surface itself. Gradients live *inside* the mark (the dot-depth fade) — never under it.

## Typography

- **Display + body** — [Space Grotesk](https://fonts.google.com/specimen/Space+Grotesk), weights 300 · 400 · **500** · 600 · 700. The wordmark uses Medium (500).
- **Mono** — [JetBrains Mono](https://www.jetbrains.com/lp/mono/), weights 400 · 500. Used for labels, page numbers, terminal output, build metadata — anything that wants to read as "machine voice". For labels: uppercase, 11 px, `letter-spacing: 0.18em`.

Both families load from Google Fonts in the brand book HTML. For production surfaces where webfonts aren't available (Qt splash, packaged PDFs), fall back to system sans (`ui-sans-serif`) — the wordmark will look slightly different but the OMNI/SIM color split still does the brand work.

## File layout

```
branding/omnisim/
├── BRAND.md                          ← this file
├── OmniSim-Brand.html                ← the full brand book (canonical artifact)
├── svg/                              ← canonical vector assets
│   ├── orb.svg                       full 340-dot orb
│   ├── glyph.svg                     small-size 110-dot glyph
│   ├── wordmark.svg                  OMNI + SIM (mimosa accent on SIM)
│   ├── wordmark_mono_light.svg       OMNISIM in ink, for dark surfaces
│   ├── wordmark_mono_dark.svg        OMNISIM in near-black, for mimosa/light surfaces
│   ├── lockup_horizontal.svg         orb + wordmark, nav default
│   ├── lockup_stacked.svg            orb + wordmark + tagline, splash/poster
│   ├── lockup_omnilink.svg           OmniLink × OmniSim co-lockup
│   └── favicon.svg                   32 × 32 with rounded corners + bg
├── orb/                              ← rasterized full orb (≥48 px)
│   ├── orb.png                       1024 × 1024
│   ├── orb_512.png
│   ├── orb_256.png
│   ├── orb_128.png
│   └── orb_64.png
├── glyph/                            ← rasterized glyph (≤48 px)
│   ├── glyph_256.png                 macOS app-icon source
│   ├── glyph_48.png
│   ├── glyph_32.png
│   └── glyph_16.png
└── preview/
    └── orb_preview.png               contact sheet for PRs
```

## Regenerating the PNGs

The SVG files are hand-curated (extracted from the brand book artifact, then linted). The PNGs are derived from them:

```bash
python scripts/branding/render_omnisim_orb.py
```

This reads `svg/orb.svg` + `svg/glyph.svg`, rasterizes each at 4× supersample, downsamples with Lanczos, and writes the size variants above plus the preview sheet. No procedural regeneration — the dot pattern is whatever lives in the SVG.

## Voice — applied in product

The brand book's page 09 ("In context") shows three canonical applications. Match these tones when adding new surfaces:

- **Terminal output** uses mimosa `◐ ◑ ◒` glyphs as progress indicators, mimosa-soft `✓` for "ready", ink-mute for telemetry. See [`scripts/cinema/brand.py`](../../../scripts/cinema/brand.py) for the canonical palette.
- **Nav bars** show `OmniLink / OmniSim` — parent brand first, then a hairline separator, then the OmniSim mark + wordmark. Both halves get equal vertical weight.
- **App icons** sit the glyph on a soft radial vignette (centered at 30 %/25 %) over `#060604`, rounded 21 % of the icon side.

## Don'ts (from page 10 of the brand book)

- × Don't rotate the orb. It's lit; the upper-left brightness is a direction.
- × Don't recolor. Mimosa is the only accent.
- × Don't squash or stretch. The orb is round.
- × Don't sit the orb on mimosa. Use the dark surface.

## Relationship to OmniLink branding

OmniSim is a product of OmniLink. The parent brand's assets live in [`../omnilink/`](../omnilink/) and remain the authority for:

- The Qt GUI icons, splash screen, About-box logo — these still come out of `render_omnilink_orb.py` and `build_omnilink_icons.py` (the legacy paths the C++ GUI consumes).
- The OmniLink connector monogram (`mark.svg`, etc.) — used for OmniLink itself, never for OmniSim alone.

OmniSim's own surfaces (READMEs, social videos, brand book, marketing assets) should use the OmniSim assets in this directory. When both brands need to appear together, use the co-lockup ([`svg/lockup_omnilink.svg`](svg/lockup_omnilink.svg)).

## Attribution

OmniSim is built on top of [Webots](https://github.com/cyberbotics/webots) by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics — they are not part of the OmniSim or OmniLink identity and must not be used to brand either.
