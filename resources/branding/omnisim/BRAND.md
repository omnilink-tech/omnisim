# OmniSim brand assets

This directory is the canonical source of truth for the **OmniSim** visual identity — distinct from the parent [OmniLink](../omnilink/BRAND.md) brand. The full identity is documented in [`OmniSim-Brand.html`](OmniSim-Brand.html); this README is the engineering crib-sheet.

> **Precedence.** For anything OmniSim-branded — READMEs, docs, social, the brand book, marketing, new product surfaces — **this directory wins** over [`../omnilink/`](../omnilink/). The OmniLink directory is the authority for the *parent* brand (the connector monogram, the OmniLink wordmark) and for the legacy OmniLink orb only. If the two disagree about "the OmniSim mark", this file is right.
>
> **The shipped app icon is the OmniSim mark (2026-11-03).** `scripts/branding/build_omnilink_icons.py` renders the mark defined here, on its rounded tile, into every icon path the C++ GUI and the packagers consume (see [App icons](#app-icons)). Until that date it rendered the OmniLink orb bare on a transparent background, which dissolved into a busy desktop wallpaper. The `.exe` icons are compiled in, so a binary shows the new icon only once it is rebuilt.

OmniSim is the simulator built **by OmniLink, for OmniLink agents**. OmniSim has its own mark (the particle orb), its own wordmark, and its own surface palette (near-black + mimosa). It locks up *with* the OmniLink mark in co-branded contexts; it does not borrow OmniLink's connector monogram.

> **Open the brand book.** [`OmniSim-Brand.html`](OmniSim-Brand.html) is a self-contained Claude artifact bundle — open it in any browser to see every page rendered with the actual fonts, the dot lattice at full fidelity, and live in-context mocks (favicon, app icon, nav bar, terminal). This file is the authority; the SVGs and PNGs in this directory are rasterizations of it.

## Identity at a glance

- **The mark — the orb.** A sphere of 250 mimosa-yellow dots on 14 latitude rows, depth-shaded so the front face reads brighter and bigger than the back. Lit from upper-left. The orb is the canonical OmniSim mark; the wordmark is secondary. Source: [`svg/orb.svg`](svg/orb.svg).
- **The glyph — the small-size variant.** 52 bolder dots on 8 rows, the near side only. Use at 128 px and below (favicons, taskbar, scaled-down chat avatars) where the full orb gets busy, then dissolves. Source: [`svg/glyph.svg`](svg/glyph.svg).
- **The wordmark — `Omni`+`Sim`.** "OmniSim" in [Sora](https://fonts.google.com/specimen/Sora) SemiBold (600), mixed case, Sora's own kerning, no extra tracking. `Omni` stays ink (`#F6F4EF`); `Sim` carries the mimosa accent (`#F6E905`). Never set it in all caps: the capital S splits the name even without the colour. The SVGs carry the letters as outlines, so they need no font installed. Source: [`svg/wordmark.svg`](svg/wordmark.svg), plus mono variants for light and mimosa surfaces. Until 2026-11-03 the wordmark was `OMNISIM` in Space Grotesk Medium caps at +0.08em; that version is retired.
- **The horizontal lockup.** Orb + wordmark side by side. Default for nav bars, headers, READMEs. Source: [`svg/lockup_horizontal.svg`](svg/lockup_horizontal.svg).
- **The stacked lockup.** Orb above wordmark above tagline (*— an open-world simulator for agents*). For splash screens, app icons, posters, social cards. Source: [`svg/lockup_stacked.svg`](svg/lockup_stacked.svg).
- **The OmniLink × OmniSim co-lockup.** OmniLink connector mark + `OmniLink / OmniSim` for surfaces where the parent brand and product brand both need parity (docs nav, product cards). Source: [`svg/lockup_omnilink.svg`](svg/lockup_omnilink.svg).

## The orb

The orb is a **generated** lattice with no randomness in it, so it reads as engineered rather than as glitter: dots sit on latitude rows (alternate rows offset half a step), the sphere is tilted so the rows read as arcs, each dot's size follows its depth and its brightness follows a single light from the upper left. The SVG is the asset every surface uses; the lattice that produced it is `ORB_LATTICE` / `GLYPH_LATTICE` in [`render_omnisim_orb.py`](../../../scripts/branding/render_omnisim_orb.py). To change the mark, change the lattice and run `--regenerate-svg` (below): never hand-edit the circles, because six SVGs embed the same block.

| Mark | Rows | Dots | Radius (viewBox units, sphere r = 100) | Hemispheres |
|---|---|---|---|---|
| Orb | 14 | 250 | 1.2–5.0 | both; far dots at 0.1–0.4 opacity imply the back |
| Glyph | 8 | 52 | 3.6–9.6 | the near side plus the rim; brighter floor so the shadowed edge still closes the circle |
| Micro (16–24 px icons only, not an SVG) | 5 | 19 | 8–14 | as the glyph |

The brightness gradient — upper-left lit, lower-right in shadow — implies a single light source without rendering one. Do not "fix" this asymmetry; it is the read.

## App icons

Every app icon (Windows `.ico`, taskbar, the Windows installer's `setup.exe`, About box, packaging PNGs, the streaming viewer's favicon) is the mark on a **rounded near-black tile**: `#060604`, corners 21 % of the side, a soft mimosa vignette upper-left and a hairline rim from 32 px up. The tile is what gives the icon an edge on any wallpaper. The mark switches with size: orb above 128 px, glyph from 128 down to 32, micro lattice at 24 and 16.

The GUI splash screen draws the bare orb on black with its own glow, from `resources/images/splash_orb.png`, and the wordmark under it from `resources/images/splash_wordmark.png` (this directory is not packaged, so the splash cannot read either here).

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

- **Wordmark** — [Sora](https://fonts.google.com/specimen/Sora) SemiBold (600), only for the word "OmniSim" in the mark. Shipped as [`fonts/Sora.ttf`](fonts/Sora.ttf) (OFL 1.1); [`outline_omnisim_wordmark.py`](../../../scripts/branding/outline_omnisim_wordmark.py) writes it into every wordmark and lockup SVG as outlines.
- **Display + body** — [Space Grotesk](https://fonts.google.com/specimen/Space+Grotesk), weights 300 · 400 · 500 · 600 · 700, for headings and running text.
- **Mono** — [JetBrains Mono](https://www.jetbrains.com/lp/mono/), weights 400 · 500. Used for labels, page numbers, terminal output, build metadata — anything that wants to read as "machine voice". For labels: uppercase, 11 px, `letter-spacing: 0.18em`.

Both families load from Google Fonts in the brand book HTML. For production surfaces where webfonts aren't available (Qt splash, packaged PDFs), fall back to system sans (`ui-sans-serif`). Never re-type the wordmark in a fallback font: use the SVGs, which are outlined, or the PNGs rendered from them.

## File layout

```
branding/omnisim/
├── BRAND.md                          ← this file
├── OmniSim-Brand.html                ← the full brand book (canonical artifact)
├── svg/                              ← canonical vector assets
│   ├── orb.svg                       full 250-dot orb
│   ├── glyph.svg                     small-size 52-dot glyph
│   ├── wordmark.svg                  Omni + Sim (mimosa accent on Sim), Sora outlines
│   ├── wordmark_mono_light.svg       OmniSim in ink, for dark surfaces
│   ├── wordmark_mono_dark.svg        OmniSim in near-black, for mimosa/light surfaces
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
├── wordmark/                         ← rasterized wordmark, cropped to the letters
│   ├── wordmark.png                  Omni + Sim, 400 px cap height
│   ├── wordmark_mono_light.png
│   └── wordmark_mono_dark.png
├── fonts/
│   ├── Sora.ttf                      the wordmark face (variable, OFL)
│   └── SpaceGrotesk.ttf              display + body (variable, OFL)
└── preview/
    └── orb_preview.png               contact sheet for PRs
```

## Regenerating the wordmark

The wordmark in `wordmark*.svg` and the three lockups sits between `<!-- wordmark:begin -->` and `<!-- wordmark:end -->`. After changing the face, weight or a lockup's position, rerun:

```bash
python scripts/branding/outline_omnisim_wordmark.py
```

It shapes "OmniSim" with HarfBuzz from `fonts/Sora.ttf` and touches only that block, so it never disturbs the orb dots. Then rasterize the wordmark for the surfaces that cannot draw SVG paths (the Qt splash, the ffmpeg end slate and watermark in `scripts/cinema/brand.py`):

```bash
python scripts/branding/render_omnisim_wordmark.py   # wordmark/*.png, resources/images/splash_wordmark.png
```

## Regenerating the PNGs

The mark inside the SVG files is generated from the lattice; the PNGs are derived from the SVGs:

```bash
python scripts/branding/render_omnisim_orb.py --regenerate-svg   # only after changing the lattice
python scripts/branding/render_omnisim_orb.py                    # orb/, glyph/, preview/, the splash orb
python scripts/branding/build_omnilink_icons.py                  # every shipped app icon
```

`--regenerate-svg` rewrites the mark block in `orb.svg`, `glyph.svg`, `favicon.svg` and the three lockups. The plain run reads `svg/orb.svg` + `svg/glyph.svg`, rasterizes each at 4× supersample, downsamples with Lanczos, and writes the size variants above plus the preview sheet (the bare marks, then the app icon at every shipped size). `build_omnilink_icons.py` writes `resources/images/omnisim.png`, `resources/icons/core/omnisim*.png` and `omnisim_doc.ico`, `src/omnisim/gui/omnisim.ico`, `resources/web/streaming_viewer/omnisim_icon.png` and `scripts/packaging/omnisim*.png`. The `.ico` reaches `omnisim-bin.exe` and the launchers through `windres`, so rebuild the engine to see it on a Windows desktop.

## Voice — applied in product

The brand book's page 09 ("In context") shows three canonical applications. Match these tones when adding new surfaces:

- **Terminal output** uses mimosa `◐ ◑ ◒` glyphs as progress indicators, mimosa-soft `✓` for "ready", ink-mute for telemetry. See [`scripts/cinema/brand.py`](../../../scripts/cinema/brand.py) for the canonical palette.
- **Nav bars** show `OmniLink / OmniSim` — parent brand first, then a hairline separator, then the OmniSim mark + wordmark. Both halves get equal vertical weight.
- **App icons** sit the mark on a soft radial vignette over `#060604`, rounded 21 % of the icon side (see [App icons](#app-icons)).

## Don'ts (from page 10 of the brand book)

- × Don't rotate the orb. It's lit; the upper-left brightness is a direction.
- × Don't recolor. Mimosa is the only accent.
- × Don't squash or stretch. The orb is round.
- × Don't sit the orb on mimosa. Use the dark surface.

## Relationship to OmniLink branding

OmniSim is a product of OmniLink. The parent brand's assets live in [`../omnilink/`](../omnilink/) and remain the authority for:

- The OmniLink connector monogram (`mark.svg`, etc.) — used for OmniLink itself, never for OmniSim alone.

OmniSim's own surfaces (READMEs, social videos, brand book, marketing assets) should use the OmniSim assets in this directory. When both brands need to appear together, use the co-lockup ([`svg/lockup_omnilink.svg`](svg/lockup_omnilink.svg)).

## Attribution

OmniSim is built on top of [Webots](https://github.com/cyberbotics/webots) by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics — they are not part of the OmniSim or OmniLink identity and must not be used to brand either.
