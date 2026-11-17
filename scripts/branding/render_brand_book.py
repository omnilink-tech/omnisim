#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Render the OmniLink and OmniSim brand book from the brand's own files.

The brand guide is ``resources/branding/omnisim/BRAND.md``; it wins over this
book. The book is one self-contained HTML file with no network access: every
mark is the shipped SVG inlined from ``resources/branding/omnisim/svg/``, and the
two brand faces are the shipped WOFF2 files embedded as base64 ``@font-face``.

    python scripts/branding/render_brand_book.py            # writes OmniSim-Brand.html
    python scripts/branding/render_brand_book.py --out X.html
"""

from __future__ import annotations

import argparse
import base64
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BRAND = ROOT / "resources" / "branding" / "omnisim"
SVG = BRAND / "svg"
FONTS = BRAND / "fonts" / "web"
RETIRED_PNG = ROOT / "resources" / "branding" / "omnilink" / "png" / "lockup_horizontal_yellow.png"
DEFAULT_OUT = BRAND / "OmniSim-Brand.html"


def svg(name: str, cls: str = "", style: str = "") -> str:
    """Return a shipped SVG as inline markup (declaration and comments stripped)."""
    text = (SVG / name).read_text(encoding="utf-8")
    text = re.sub(r"<\?xml[^>]*\?>", "", text)
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    text = re.sub(r"\n\s*\n", "\n", text).strip()
    extra = ""
    if cls:
        extra += f' class="{cls}"'
    if style:
        extra += f' style="{style}"'
    return text.replace("<svg ", f"<svg{extra} ", 1)


def font_face(family: str, weight: int, file: str) -> str:
    data = base64.b64encode((FONTS / file).read_bytes()).decode("ascii")
    return (
        "@font-face{font-family:'%s';font-style:normal;font-weight:%d;font-display:block;"
        "src:url(data:font/woff2;base64,%s) format('woff2');}" % (family, weight, data)
    )


def png_data_uri(path: Path) -> str:
    return "data:image/png;base64," + base64.b64encode(path.read_bytes()).decode("ascii")


CSS = r"""
:root{
  --mimosa:#F6E905; --near-black:#0A0A06; --ink:#F6F4EF; --ink-mute:#B4B3A4;
  --mimosa-dim:#D4C500; --surface:#14140D; --sunk:#060604; --hair:#2A2A20;
  --head:Sora, Inter, ui-sans-serif, system-ui, sans-serif;
  --body:Inter, ui-sans-serif, system-ui, "Segoe UI", sans-serif;
  --mono:ui-monospace, Consolas, "DejaVu Sans Mono", monospace;
}
*{box-sizing:border-box;margin:0;padding:0}
html{background:var(--near-black);-webkit-print-color-adjust:exact;print-color-adjust:exact}
body{background:var(--near-black);color:var(--ink);font-family:var(--body);font-size:16px;line-height:1.6;
  -webkit-font-smoothing:antialiased}
svg{display:block}
code{font-family:var(--mono);font-size:.86em;background:var(--sunk);padding:.1em .4em;border-radius:4px;color:var(--ink)}
a{color:var(--mimosa);text-decoration:none}
.page{max-width:1240px;margin:0 auto;padding:112px 72px;border-top:1px solid var(--hair)}
.page:first-child{border-top:0}
.eyebrow{font-family:var(--body);font-weight:500;font-size:13px;letter-spacing:.14em;text-transform:uppercase;color:var(--ink-mute);margin-bottom:18px}
.eyebrow b{color:var(--mimosa);font-weight:600;margin-right:12px}
h1,h2,h3{font-family:var(--head);font-weight:600;color:var(--ink);letter-spacing:-.01em}
h1{font-size:64px;line-height:1.08}
h2{font-size:44px;line-height:1.12;margin-bottom:22px}
h3{font-size:20px;line-height:1.3;margin-bottom:10px}
p{max-width:68ch;color:var(--ink)}
p.lede{font-size:20px;line-height:1.55;color:var(--ink-mute);max-width:60ch;margin-bottom:48px}
.mute{color:var(--ink-mute)}
.small{font-size:13px;line-height:1.5}
.accent{color:var(--mimosa)}
ul.rules{list-style:none;max-width:72ch}
ul.rules li{position:relative;padding-left:22px;margin:8px 0}
ul.rules li::before{content:"";position:absolute;left:0;top:.68em;width:8px;height:8px;border-radius:50%;background:var(--mimosa)}
.grid{display:grid;gap:24px}
.g2{grid-template-columns:repeat(2,minmax(0,1fr))}
.g3{grid-template-columns:repeat(3,minmax(0,1fr))}
.g4{grid-template-columns:repeat(4,minmax(0,1fr))}
.g5{grid-template-columns:repeat(5,minmax(0,1fr))}
.card{background:var(--surface);border-radius:16px;padding:32px}
.stage{background:var(--surface);border-radius:16px;padding:48px;display:flex;align-items:center;justify-content:center}
.stage.sunk{background:var(--sunk)}
.stage.paper{background:var(--ink)}
.caption{margin-top:14px;font-size:13px;color:var(--ink-mute);line-height:1.5}
.caption code{background:transparent;padding:0;color:var(--ink);font-size:.95em}
.block{margin-top:72px}
.block > h3{margin-bottom:20px}
table{width:100%;border-collapse:collapse;font-size:15px}
th{font-family:var(--body);font-weight:500;font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:var(--ink-mute);text-align:left;padding:0 16px 14px 0;border-bottom:1px solid var(--hair)}
td{padding:16px 16px 16px 0;border-bottom:1px solid var(--hair);vertical-align:top}
td.write{font-family:var(--head);font-weight:600;font-size:18px;white-space:nowrap}
td.never{color:var(--ink-mute)}
.x{text-decoration:line-through;text-decoration-color:rgba(246,244,239,.45)}

/* cover */
.cover{min-height:820px;display:flex;flex-direction:column;justify-content:space-between;padding-top:96px}
.cover .co{width:100%;max-width:1000px}
.cover h1{margin-top:96px}
.cover .meta{display:flex;gap:48px;flex-wrap:wrap;margin-top:56px;font-size:14px;color:var(--ink-mute)}
.cover .meta b{display:block;color:var(--ink);font-weight:500;font-size:15px}

/* 1 relationship */
.story{font-family:var(--head);font-weight:600;font-size:36px;line-height:1.25;max-width:24ch;margin:8px 0 56px}
.story span{color:var(--mimosa)}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:24px}
.pair .card h3{font-size:26px}
.pair .card .tag{font-size:13px;color:var(--ink-mute);margin-bottom:16px}

/* 3 marks */
.markbox{width:240px;height:240px;display:flex;align-items:center;justify-content:center}
.markbox .orb{width:240px;height:240px}
.markbox .olm{width:85%;height:auto}
.marks-pair{display:flex;gap:96px;justify-content:center;align-items:center;padding:56px 48px}
.marks-pair figure{text-align:center}
.lockup{width:100%;height:auto}
.lockup.h{max-width:520px}
.retired{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:20px}
.retired .card{padding:24px;border:1px dashed #3A3A30;background:transparent}
.retired .card h3{font-size:16px;margin-bottom:6px}
.struck{position:relative;background:var(--sunk);border-radius:10px;height:110px;display:flex;align-items:center;justify-content:center;margin-bottom:16px;overflow:hidden}
.struck img{max-width:78%;max-height:70px;opacity:.55}
.struck::after{content:"";position:absolute;left:8%;right:8%;top:50%;height:2px;background:#C8C7BA;transform:rotate(-12deg)}
.struck .txt{font-family:var(--body);font-weight:600;letter-spacing:.1em;color:#6E6D62;font-size:12px;text-align:center;line-height:1.8}
.sizes{display:flex;align-items:flex-end;gap:40px}

/* 4 wordmarks */
.wm{width:100%;max-width:300px;height:auto}

/* 5 type */
.spec{background:var(--surface);border-radius:16px;padding:40px}
.spec .big{font-family:var(--head);font-weight:600;font-size:96px;line-height:1;letter-spacing:-.01em}
.spec .alpha{font-family:var(--head);font-weight:600;font-size:26px;line-height:1.45;color:var(--ink-mute);margin-top:20px;word-break:break-all}
.spec.inter .big{font-family:var(--body);font-weight:400}
.spec.inter .alpha{font-family:var(--body);font-weight:400}
.weights{display:flex;gap:40px;margin-top:24px;font-size:22px}
.weights span small{display:block;font-size:12px;color:var(--ink-mute);letter-spacing:.08em}
.hier{background:var(--surface);border-radius:16px;padding:40px}
.hier .h1s{font-family:var(--head);font-weight:600;font-size:40px;line-height:1.15}
.hier .h2s{font-family:var(--head);font-weight:600;font-size:24px;margin-top:20px}
.hier .bs{font-size:16px;color:var(--ink);margin-top:10px;max-width:60ch}
.hier .ui{display:flex;gap:12px;margin-top:22px;align-items:center}
.btn{font-family:var(--body);font-weight:600;font-size:14px;padding:10px 18px;border-radius:999px;background:var(--mimosa);color:var(--near-black);display:inline-block}
.btn.ghost{background:transparent;color:var(--ink);border:1px solid #3A3A30;font-weight:500}
.mono-line{font-family:var(--mono);font-size:14px;background:var(--sunk);border-radius:10px;padding:14px 18px;margin-top:22px;color:var(--ink-mute)}
.mono-line .ok{color:#FAF6A4}
.mono-line .y{color:var(--mimosa)}

/* 6 colour */
.swatch{border-radius:14px;height:132px;padding:16px;display:flex;align-items:flex-end;font-size:13px;font-weight:500}
.swatch.edge{box-shadow:inset 0 0 0 1px #3A3A30}
.chip{display:inline-block;width:22px;height:22px;border-radius:6px;vertical-align:middle;margin-right:12px;box-shadow:inset 0 0 0 1px rgba(246,244,239,.18)}
.retired-colours{display:flex;flex-wrap:wrap;gap:12px}
.rc{display:flex;align-items:center;gap:10px;background:var(--surface);border-radius:10px;padding:10px 14px;font-size:14px}
.rc .chip{margin:0;position:relative;overflow:hidden}
.rc .chip::after{content:"";position:absolute;left:-4px;right:-4px;top:50%;height:2px;background:#E8E6DE;transform:rotate(-45deg);box-shadow:0 0 0 1px #0A0A06}
.rc .lbl{color:var(--ink-mute)}

/* 7 applying */
.mock{border-radius:16px;overflow:hidden;background:var(--sunk);box-shadow:0 0 0 1px var(--hair)}
.site-nav{background:var(--near-black);display:flex;align-items:center;justify-content:space-between;padding:20px 32px;border-bottom:1px solid var(--hair)}
.site-nav .lk{height:34px;width:auto}
.site-nav nav{display:flex;gap:28px;align-items:center;font-size:14px;color:var(--ink-mute);font-weight:500}
.site-hero{background:var(--near-black);padding:64px 32px 72px}
.site-hero h4{font-family:var(--head);font-weight:600;font-size:40px;line-height:1.15;max-width:22ch;color:var(--ink)}
.site-hero h4 span{color:var(--mimosa)}
.site-hero p{color:var(--ink-mute);margin-top:16px;font-size:17px}
.site-hero .btns{display:flex;gap:12px;margin-top:28px}
.icons{display:flex;align-items:flex-end;justify-content:center;gap:36px;padding:48px;background:var(--surface);border-radius:16px;aspect-ratio:16/10}
.tile{background:radial-gradient(circle at 24% 20%, rgba(246,233,5,.13), rgba(246,233,5,0) 58%), #060604;
  border-radius:21%;box-shadow:inset 0 0 0 1px rgba(246,244,239,.09);display:flex;align-items:center;justify-content:center}
.tile svg{width:82%;height:82%}
.tile.t32{box-shadow:none}
.splash{background:var(--near-black);border-radius:16px;box-shadow:0 0 0 1px var(--hair);aspect-ratio:16/10;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:10px}
.splash .orb{width:150px;height:150px}
.splash .wm{width:230px;max-width:none}
.splash .by{font-family:var(--body);font-size:15px;color:var(--ink-mute);margin-top:2px}
.doc{background:var(--near-black);border-radius:16px;box-shadow:0 0 0 1px var(--hair);padding:36px 44px 48px}
.doc .hdr{display:flex;align-items:center;gap:28px;padding-bottom:22px;border-bottom:1px solid var(--hair)}
.doc .hdr svg{height:36px;width:auto}
.doc .hdr .hl{width:1px;height:32px;background:#3A3A30}
.doc .hdr .ref{margin-left:auto;font-size:12px;color:var(--ink-mute)}
.doc h4{font-family:var(--head);font-weight:600;font-size:30px;margin-top:36px;line-height:1.2}
.doc .sub{color:var(--ink-mute);margin-top:8px}
.doc .lines{margin-top:26px}
.doc .lines i{display:block;height:9px;border-radius:5px;background:#1E1E16;margin:12px 0}
.slate{background:var(--near-black);aspect-ratio:16/9;border-radius:16px;box-shadow:0 0 0 1px var(--hair);display:flex;flex-direction:column;align-items:center;justify-content:center;gap:24px;padding:28px}
.slate svg{width:88%;height:auto}
.slate .cap{font-size:16px;color:var(--ink-mute)}

/* 8 don'ts */
.dont .stage{height:220px;padding:28px;position:relative}
.dont .stage svg{width:140px;height:140px}
.dont .stage::after{content:"";position:absolute;inset:16px;border-radius:10px;
  background:linear-gradient(to top right, transparent calc(50% - 1.5px), rgba(246,244,239,.55) calc(50% - 1.5px), rgba(246,244,239,.55) calc(50% + 1.5px), transparent calc(50% + 1.5px))}
.dont h3{font-size:16px;margin-top:16px;margin-bottom:4px}

footer.page{padding-top:72px;padding-bottom:72px}
footer .row{display:flex;justify-content:space-between;align-items:center;gap:32px;flex-wrap:wrap}
footer .lk{height:40px;width:auto}

@media (max-width:900px){
  .page{padding:72px 20px}
  .g2,.g3,.g4,.g5,.pair,.retired{grid-template-columns:1fr}
  h1{font-size:44px} h2{font-size:32px}
  .marks-pair{flex-direction:column;gap:40px}
  .spec .big{font-size:64px}
}
@media print{
  @page{size:A4 landscape;margin:0}
  body{font-size:12px}
  .page{page-break-before:always;break-before:page;padding:48px 56px;border-top:0;max-width:none}
  .page:first-child{page-break-before:auto;break-before:auto}
  .card,.stage,.mock,.spec,.hier,.splash,.doc,.slate,.dont,.swatch,figure,tr{break-inside:avoid}
  .cover{min-height:auto}
}
"""


def build() -> str:
    fonts = "".join(
        [
            font_face("Sora", 600, "Sora-SemiBold.woff2"),
            font_face("Inter", 400, "Inter-Regular.woff2"),
            font_face("Inter", 500, "Inter-Medium.woff2"),
            font_face("Inter", 600, "Inter-SemiBold.woff2"),
        ]
    )

    names_rows = [
        ("OmniLink", "Omnilink, Omni Link, OMNILINK (outside legal all-caps blocks), omni link"),
        ("OmniSim", "Omnisim, Omni Sim, OMNISIM, Omni-Sim"),
        ("OmniKey", "Omni Key, Omnikey"),
    ]
    names_html = "".join(
        f'<tr><td class="write">{w}</td><td class="never">{n}</td></tr>' for w, n in names_rows
    )

    colours = [
        ("mimosa", "#F6E905", "The one accent: marks, the second word of each name, calls to action, links, highlights", "#0A0A06"),
        ("near-black", "#0A0A06", "The one canvas: page and slide backgrounds, dark panels, the ink on mimosa fills", "#F6F4EF"),
        ("ink", "#F6F4EF", "Text and headings on near-black", "#0A0A06"),
        ("ink-mute", "#B4B3A4", "Secondary text on near-black", "#0A0A06"),
        ("mimosa-dim", "#D4C500", "Hover and pressed states; a yellow that has to sit on white paper", "#0A0A06"),
        ("surface", "#14140D", "Cards and panels on near-black", "#F6F4EF"),
        ("sunk", "#060604", "Wells, code, terminals, the app-icon tile", "#F6F4EF"),
    ]
    swatches = "".join(
        f'<div><div class="swatch edge" style="background:{h};color:{fg}">{h}</div>'
        f'<div class="caption"><b style="color:var(--ink);font-weight:600">{n}</b></div></div>'
        for n, h, _u, fg in colours
    )
    colour_rows = "".join(
        f'<tr><td style="white-space:nowrap"><span class="chip" style="background:{h}"></span>'
        f'<b style="font-weight:600">{n}</b></td><td><code>{h}</code></td><td class="mute">{u}</td></tr>'
        for n, h, u, _fg in colours
    )
    retired_colours = [
        ("#F6E932", "the old OmniLink orb's dots"),
        ("#E1D256", "the website's softer gold"),
        ("#B9A73F", "softer-gold companion"),
        ("#F3EDB3", "softer-gold companion"),
        ("#FBBF24", "amber"),
        ("#F5D565", "one-off yellow"),
        ("#000000", "as a surface"),
        ("#11120F", "old black"),
        ("#111109", "old black"),
        ("#0C0C0D", "old black"),
    ]
    retired_colours_html = "".join(
        f'<div class="rc"><span class="chip" style="background:{h}"></span><code>{h}</code>'
        f'<span class="lbl">{lbl}</span></div>'
        for h, lbl in retired_colours
    )

    retired_png = png_data_uri(RETIRED_PNG)

    t = {
        "%%FONTS%%": fonts,
        "%%CSS%%": CSS,
        "%%CO_LOCKUP%%": svg("lockup_omnilink.svg", "co"),
        "%%NAMES%%": names_html,
        "%%ORB%%": svg("orb.svg", "orb"),
        "%%OL_MARK%%": svg("omnilink_mark.svg", "olm"),
        "%%GLYPH96%%": svg("glyph.svg", "", "width:96px;height:96px"),
        "%%GLYPH48%%": svg("glyph.svg", "", "width:48px;height:48px"),
        "%%FAVICON%%": svg("favicon.svg", "", "width:32px;height:32px"),
        "%%OL_MARK_INK%%": svg("omnilink_mark_ink.svg", "", "width:96px;height:96px"),
        "%%OL_LOCKUP%%": svg("omnilink_lockup.svg", "lockup h"),
        "%%OS_LOCKUP%%": svg("lockup_horizontal.svg", "lockup h"),
        "%%CO_LOCKUP2%%": svg("lockup_omnilink.svg", "lockup"),
        "%%RETIRED_PNG%%": retired_png,
        "%%WM_OS%%": svg("wordmark.svg", "wm"),
        "%%WM_OL%%": svg("omnilink_wordmark.svg", "wm"),
        "%%WM_OS_DARK%%": svg("wordmark_mono_dark.svg", "wm"),
        "%%WM_OL_DARK%%": svg("omnilink_wordmark_mono_dark.svg", "wm"),
        "%%WM_OS_LIGHT%%": svg("wordmark_mono_light.svg", "wm"),
        "%%WM_OL_LIGHT%%": svg("omnilink_wordmark_mono_light.svg", "wm"),
        "%%SWATCHES%%": swatches,
        "%%COLOUR_ROWS%%": colour_rows,
        "%%RETIRED_COLOURS%%": retired_colours_html,
        "%%NAV_LOCKUP%%": svg("omnilink_lockup.svg", "lk"),
        "%%TILE_ORB%%": svg("orb.svg"),
        "%%TILE_GLYPH%%": svg("glyph.svg"),
        "%%SPLASH_ORB%%": svg("orb.svg", "orb"),
        "%%SPLASH_WM%%": svg("wordmark.svg", "wm"),
        "%%DOC_OL%%": svg("omnilink_lockup.svg"),
        "%%DOC_OS%%": svg("lockup_horizontal.svg"),
        "%%SLATE%%": svg("lockup_omnilink.svg"),
        "%%DONT_ROT%%": svg("orb.svg", "", "transform:rotate(150deg)"),
        "%%DONT_RECOLOUR%%": svg("omnilink_mark.svg", "", "filter:hue-rotate(170deg) saturate(1.4)"),
        "%%DONT_SQUASH%%": svg("omnilink_mark.svg", "", "transform:scale(1.35,.72)"),
        "%%DONT_ON_MIMOSA%%": svg("omnilink_mark_ink.svg"),
        "%%DONT_GRADIENT%%": svg("orb.svg"),
        "%%FOOT_LOCKUP%%": svg("lockup_omnilink.svg", "lk"),
    }
    html = TEMPLATE
    for k, v in t.items():
        html = html.replace(k, v)
    leftover = re.findall(r"%%[A-Z0-9_]+%%", html)
    if leftover:
        raise SystemExit(f"unfilled placeholders: {leftover}")
    return html


TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="description" content="The brand book for OmniLink and OmniSim: one family, two spheres. Rendered from resources/branding/omnisim/BRAND.md, which wins if the two disagree.">
<meta name="generator" content="scripts/branding/render_brand_book.py">
<title>OmniLink and OmniSim: brand book</title>
<style>%%FONTS%%</style>
<style>%%CSS%%</style>
</head>
<body>

<!-- Generated by scripts/branding/render_brand_book.py from the shipped SVGs and fonts.
     The canonical guide is resources/branding/omnisim/BRAND.md; if this book disagrees, BRAND.md is right. -->

<header class="page cover">
  <div>
    <div class="eyebrow"><b>Brand book</b>Approved 2026-11-16</div>
    %%CO_LOCKUP%%
    <h1>OmniLink and OmniSim:<br>brand book</h1>
    <p class="lede" style="margin-top:20px;margin-bottom:0">One entity, two names, two spheres. Simple and the same everywhere: the simulator, the website, documents and films.</p>
  </div>
  <div class="meta">
    <div><b>Canonical guide</b><code>resources/branding/omnisim/BRAND.md</code> wins if this book disagrees</div>
    <div><b>Contents</b>1 Relationship &middot; 2 Names &middot; 3 Marks &middot; 4 Wordmarks &middot; 5 Typography &middot; 6 Colour &middot; 7 Applying it &middot; 8 Don&rsquo;ts</div>
  </div>
</header>

<!-- 1 -->
<section class="page" id="relationship">
  <div class="eyebrow"><b>01</b>The relationship</div>
  <h2>OmniSim, by OmniLink</h2>
  <p class="story">OmniLink builds agents that work on robots and machines; <span>OmniSim is their workshop.</span></p>
  <div class="pair">
    <div class="card">
      <div class="tag">The company and its AI agents</div>
      <h3>OmniLink</h3>
      <p class="mute">OmniLink is the company, and the AI agents it builds.</p>
    </div>
    <div class="card">
      <div class="tag">OmniLink&rsquo;s open-source workshop</div>
      <h3>OmniSim</h3>
      <p class="mute">There, OmniLink&rsquo;s agents design, build, test and debug robots and machines, and connect to the real one. The current line: &ldquo;an open-source robotics workshop for agents&rdquo;.</p>
    </div>
  </div>
  <div class="grid g2 block">
    <div>
      <h3>Where both names appear</h3>
      <ul class="rules">
        <li>Write &ldquo;OmniSim, by OmniLink&rdquo;.</li>
        <li>In a nav bar or page header, the speaker comes first: the website leads with OmniLink; the simulator and its docs lead with OmniSim.</li>
      </ul>
    </div>
    <div>
      <h3>Claims</h3>
      <ul class="rules">
        <li>Every product claim traces to <code>docs/developer/positioning.md</code>; never widen one.</li>
        <li>Customer-facing pages do not describe architecture.</li>
        <li>Never &ldquo;first&rdquo; or &ldquo;only&rdquo;.</li>
        <li>Sim-to-real covers the control surface only.</li>
        <li>A &ldquo;digital twin&rdquo; is a model built from the robot&rsquo;s or machine&rsquo;s own description.</li>
      </ul>
    </div>
  </div>
  <div class="block">
    <h3>Retired lines</h3>
    <p class="mute"><span class="x">Where robot software gets debugged</span> &nbsp;&middot;&nbsp; <span class="x">The simulator you can talk to</span> &nbsp;&middot;&nbsp; <span class="x">An open-world simulator for agents</span></p>
  </div>
</section>

<!-- 2 -->
<section class="page" id="names">
  <div class="eyebrow"><b>02</b>Names</div>
  <h2>Write it one way</h2>
  <p class="lede">Mixed case, one word, no space. The second word starts with a capital.</p>
  <table>
    <thead><tr><th style="width:34%">Write</th><th>Never</th></tr></thead>
    <tbody>%%NAMES%%</tbody>
  </table>
  <p class="caption" style="margin-top:22px">Code identifiers keep their spelling (<code>olink-</code> element prefix, <code>OLINK_*</code>, <code>olink_</code> keys, the <code>omnisim</code> module). They are not prose.</p>
</section>

<!-- 3 -->
<section class="page" id="marks">
  <div class="eyebrow"><b>03</b>Marks</div>
  <h2>Two spheres, one family</h2>
  <p class="lede">The circuit sphere is OmniLink. The particle orb is OmniSim. Set side by side at the same height, they read as a pair.</p>

  <div class="stage marks-pair">
    <figure>
      <div class="markbox">%%OL_MARK%%</div>
      <figcaption class="caption"><b style="color:var(--ink)">OmniLink</b> &middot; the circuit sphere<br><code>svg/omnilink_mark.svg</code> (tight frame)</figcaption>
    </figure>
    <figure>
      <div class="markbox">%%ORB%%</div>
      <figcaption class="caption"><b style="color:var(--ink)">OmniSim</b> &middot; the particle orb<br><code>svg/orb.svg</code>, 250 dots on 14 rows, lit from the upper left</figcaption>
    </figure>
  </div>
  <p class="caption">Same optical size: in both lockups the sphere is 166 units across in a 240-unit-high frame, and the name&rsquo;s cap height is 84 units. Use the tight <code>omnilink_mark.svg</code>, not the padded 1080 source artwork, so the mark prints at its real size.</p>

  <div class="grid g3 block">
    <div>
      <div class="stage" style="height:200px"><div class="sizes">%%GLYPH96%%%%GLYPH48%%%%FAVICON%%</div></div>
      <div class="caption"><b style="color:var(--ink)">Glyph</b> &middot; <code>svg/glyph.svg</code>, 52 dots. The orb above 128 px, the glyph from 128 down to 32, the micro lattice (19 dots) at 24 and 16.</div>
    </div>
    <div>
      <div class="stage paper" style="height:200px">%%OL_MARK_INK%%</div>
      <div class="caption"><b style="color:var(--ink)">On paper</b> &middot; <code>svg/omnilink_mark_ink.svg</code>, the circuit sphere in near-black.</div>
    </div>
    <div>
      <div class="card" style="height:200px;display:flex;flex-direction:column;justify-content:center">
        <p class="small mute">App icons: the website uses its favicon set (the circuit sphere on near-black); the simulator uses its icon set, the orb on a rounded #060604 tile. See <a href="#applying">Applying it</a>.</p>
      </div>
    </div>
  </div>

  <div class="block">
    <h3>Lockups: mark and name, side by side</h3>
    <div class="grid g2">
      <div><div class="stage" style="padding:56px 40px">%%OL_LOCKUP%%</div><div class="caption"><b style="color:var(--ink)">OmniLink</b> &middot; <code>svg/omnilink_lockup.svg</code></div></div>
      <div><div class="stage" style="padding:56px 40px">%%OS_LOCKUP%%</div><div class="caption"><b style="color:var(--ink)">OmniSim</b> &middot; <code>svg/lockup_horizontal.svg</code></div></div>
    </div>
  </div>

  <div class="block">
    <h3>Both brands together: the co-lockup</h3>
    <div class="stage" style="padding:64px 56px">%%CO_LOCKUP2%%</div>
    <div class="caption"><code>svg/lockup_omnilink.svg</code>, reading &ldquo;OmniSim, by OmniLink&rdquo;: both lockups at full weight, OmniSim first and OmniLink second, a hairline between them.</div>
  </div>

  <div class="block">
    <h3>Retired. Do not use on any new surface</h3>
    <div class="retired">
      <div class="card">
        <div class="struck"><img src="%%RETIRED_PNG%%" alt="Retired stacked OMNI/LINK lockup"></div>
        <h3>Stacked &ldquo;OMNI / LINK&rdquo; lockups</h3>
        <p class="small mute"><code>omnilink/svg/lockup_horizontal*.svg</code>, <code>lockup_alt.svg</code>, <code>omnilink/png/lockup_*.png</code></p>
      </div>
      <div class="card">
        <div class="struck"><div class="txt">220 + 80 DOTS</div></div>
        <h3>The old OmniLink dot orb</h3>
        <p class="small mute"><code>omnilink/orb/</code>. The circuit sphere replaces it.</p>
      </div>
      <div class="card">
        <div class="struck"><div class="txt">OMNISIM &middot; OMNILINK<br>OMNI/SIM &middot; OMNI/LINK</div></div>
        <h3>Every all-caps wordmark</h3>
        <p class="small mute">Both names are mixed case, always.</p>
      </div>
      <div class="card">
        <div class="struck"><div class="txt">RING + DOT</div></div>
        <h3>The ring-and-dot &ldquo;OmniLink mark&rdquo;</h3>
        <p class="small mute">The mark the old co-lockup drew.</p>
      </div>
    </div>
    <p class="caption">The files stay in the tree so old renders can be rebuilt; nothing new points at them.</p>
  </div>
</section>

<!-- 4 -->
<section class="page" id="wordmarks">
  <div class="eyebrow"><b>04</b>Wordmarks</div>
  <h2>Both names, set identically</h2>
  <p class="lede">Sora SemiBold (600), mixed case, Sora&rsquo;s own kerning, no added tracking. &ldquo;Omni&rdquo; in ink, the second word in mimosa.</p>
  <div class="grid g2">
    <div><div class="stage" style="height:200px">%%WM_OL%%</div><div class="caption"><code>svg/omnilink_wordmark.svg</code> &middot; on near-black</div></div>
    <div><div class="stage" style="height:200px">%%WM_OS%%</div><div class="caption"><code>svg/wordmark.svg</code> &middot; on near-black</div></div>
    <div><div class="stage paper" style="height:200px">%%WM_OL_DARK%%</div><div class="caption"><code>omnilink_wordmark_mono_dark.svg</code> &middot; on paper</div></div>
    <div><div class="stage paper" style="height:200px">%%WM_OS_DARK%%</div><div class="caption"><code>wordmark_mono_dark.svg</code> &middot; on paper</div></div>
    <div><div class="stage sunk" style="height:200px">%%WM_OL_LIGHT%%</div><div class="caption"><code>omnilink_wordmark_mono_light.svg</code> &middot; one-colour ink</div></div>
    <div><div class="stage sunk" style="height:200px">%%WM_OS_LIGHT%%</div><div class="caption"><code>wordmark_mono_light.svg</code> &middot; one-colour ink</div></div>
  </div>
  <div class="grid g2 block">
    <div>
      <h3>Outlines, not type</h3>
      <ul class="rules">
        <li>The SVGs carry the letters as outlines. Never retype a wordmark in a fallback font; use the SVG or a PNG rendered from it.</li>
        <li>On paper, use the <code>_mono_dark</code> versions.</li>
      </ul>
    </div>
    <div>
      <h3>Regenerate</h3>
      <ul class="rules">
        <li><code>python scripts/branding/outline_omnisim_wordmark.py</code> &mdash; &ldquo;OmniSim&rdquo;.</li>
        <li><code>python scripts/branding/render_brand_system.py --png</code> &mdash; &ldquo;OmniLink&rdquo;, the OmniLink lockup, the co-lockup, fonts and <code>png/</code>.</li>
      </ul>
    </div>
  </div>
</section>

<!-- 5 -->
<section class="page" id="typography">
  <div class="eyebrow"><b>05</b>Typography</div>
  <h2>Two families, both OFL</h2>
  <p class="lede">Sora SemiBold for names, headings, titles and big numbers. Inter for body and UI: the website, the simulator&rsquo;s Qt interface, PDFs and film captions.</p>
  <div class="grid g2">
    <div class="spec">
      <div class="eyebrow" style="margin-bottom:24px">Sora SemiBold 600 &middot; headings</div>
      <div class="big">Aa Gg 42</div>
      <div class="alpha">ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz 0123456789</div>
    </div>
    <div class="spec inter">
      <div class="eyebrow" style="margin-bottom:24px">Inter 400 / 500 / 600 &middot; body and UI</div>
      <div class="big">Aa Gg 42</div>
      <div class="alpha">ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz 0123456789</div>
      <div class="weights"><span style="font-weight:400">Regular<small>400</small></span><span style="font-weight:500">Medium<small>500</small></span><span style="font-weight:600">SemiBold<small>600</small></span></div>
    </div>
  </div>
  <div class="hier block" style="margin-top:24px">
    <div class="eyebrow">In use</div>
    <div class="h1s">A workshop where agents build robots</div>
    <div class="h2s">Test the gripper before you cut the part</div>
    <p class="bs">Body copy is Inter Regular at a comfortable measure. Secondary text drops to ink-mute; links and calls to action take mimosa, the one accent.</p>
    <div class="ui"><span class="btn">Get started</span><span class="btn ghost">Read the guide</span></div>
    <div class="mono-line"><span class="y">&#9680;</span> run-headless warehouse.omniworld &nbsp; <span class="ok">&#10003;</span> load check passed</div>
    <p class="caption">Code and terminal text use the platform&rsquo;s monospace (<code>ui-monospace, Consolas, "DejaVu Sans Mono", monospace</code>). It is not a brand face.</p>
  </div>
  <div class="grid g3 block">
    <div class="card"><h3>Files</h3><p class="small mute"><code>fonts/Sora.ttf</code>, <code>fonts/Inter.ttf</code> (variable masters); <code>fonts/static/</code> (.ttf); <code>fonts/web/</code> (.woff2); licences <code>OFL-Sora.txt</code>, <code>OFL-Inter.txt</code>.</p></div>
    <div class="card"><h3>Fallbacks</h3><p class="small mute">Headings: <code>Sora, Inter, ui-sans-serif, system-ui, sans-serif</code><br>Body: <code>Inter, ui-sans-serif, system-ui, "Segoe UI", sans-serif</code></p></div>
    <div class="card"><h3>Retired faces</h3><p class="small mute"><span class="x">Space Grotesk</span> &middot; <span class="x">JetBrains Mono</span> (as a brand face) &middot; <span class="x">Montserrat</span> &middot; <span class="x">Hogira</span> (no redistribution licence; kept out of public snapshots). The files stay so old films can be rebuilt; nothing new loads them.</p></div>
  </div>
</section>

<!-- 6 -->
<section class="page" id="colour">
  <div class="eyebrow"><b>06</b>Colour</div>
  <h2>One mimosa, one near-black</h2>
  <p class="lede">Mimosa is #F6E905, the value the OmniSim identity always defined. Near-black is the one canvas.</p>
  <div class="grid" style="grid-template-columns:repeat(7,minmax(0,1fr));gap:16px">%%SWATCHES%%</div>
  <div class="block">
    <table>
      <thead><tr><th style="width:20%">Token</th><th style="width:14%">Hex</th><th>Use</th></tr></thead>
      <tbody>%%COLOUR_ROWS%%</tbody>
    </table>
  </div>
  <div class="grid g2 block">
    <div>
      <h3>Contrast and paper</h3>
      <ul class="rules">
        <li>Mimosa on near-black is about 15:1. Near-black on mimosa is the button pairing.</li>
        <li>On white paper (PDF documents), text is near-black or #14140E. Yellow on white is mimosa-dim, or mimosa as a fill under near-black text, never as thin text.</li>
      </ul>
      <div style="display:flex;gap:14px;margin-top:22px;flex-wrap:wrap">
        <span class="btn">Near-black on mimosa</span>
        <span style="background:#fff;color:#0A0A06;border-radius:10px;padding:10px 16px;font-size:14px">Paper text &middot; <b style="background:var(--mimosa);padding:1px 6px;border-radius:4px;font-weight:600">mimosa fill</b> &middot; <b style="color:var(--mimosa-dim);font-weight:600">mimosa-dim</b></span>
      </div>
    </div>
    <div>
      <h3>Retired values</h3>
      <div class="retired-colours">%%RETIRED_COLOURS%%</div>
      <p class="caption">And every other one-off yellow. Use mimosa and near-black.</p>
    </div>
  </div>
</section>

<!-- 7 -->
<section class="page" id="applying">
  <div class="eyebrow"><b>07</b>Applying it</div>
  <h2>The same everywhere</h2>
  <p class="lede">The speaker leads. The website speaks as OmniLink; the simulator and its docs speak as OmniSim.</p>

  <h3>Website header</h3>
  <div class="mock">
    <div class="site-nav">
      %%NAV_LOCKUP%%
      <nav><span>Agents</span><span>OmniSim</span><span>Docs</span><span class="btn">Get an OmniKey</span></nav>
    </div>
    <div class="site-hero">
      <h4>OmniLink builds agents that work on robots and machines; <span>OmniSim is their workshop.</span></h4>
      <p>The header is the OmniLink lockup. The favicon is the circuit sphere on near-black. The OmniSim page uses the OmniSim lockup.</p>
    </div>
  </div>

  <div class="grid g2 block">
    <div>
      <h3>Simulator app icon</h3>
      <div class="icons">
        <div class="tile" style="width:168px;height:168px">%%TILE_ORB%%</div>
        <div class="tile" style="width:96px;height:96px">%%TILE_GLYPH%%</div>
        <div class="tile t32" style="width:32px;height:32px">%%TILE_GLYPH%%</div>
      </div>
      <p class="caption">The orb on a #060604 tile, corners 21 % of the side, a soft mimosa vignette at the upper left, a hairline rim from 32 px up. The orb above 128 px, the glyph below.</p>
    </div>
    <div>
      <h3>Simulator splash</h3>
      <div class="splash">
        %%SPLASH_ORB%%
        %%SPLASH_WM%%
        <div class="by">by OmniLink</div>
      </div>
      <p class="caption">The orb and the OmniSim wordmark, with &ldquo;by OmniLink&rdquo; set in Inter under it. The About box says &ldquo;OmniSim, by OmniLink&rdquo;. The Qt interface uses Inter.</p>
    </div>
  </div>

  <div class="grid g2 block">
    <div>
      <h3>Document page header</h3>
      <div class="doc">
        <div class="hdr">%%DOC_OL%%<span class="hl"></span>%%DOC_OS%%</div>
        <h4>Proposal: a robot cell for small shops</h4>
        <div class="sub">Headings in Sora, body in Inter, the palette above.</div>
        <div class="lines"><i style="width:92%"></i><i style="width:86%"></i><i style="width:90%"></i><i style="width:60%"></i></div>
      </div>
      <p class="caption">Both lockups at the same height, a hairline between them.</p>
    </div>
    <div>
      <h3>Film end slate</h3>
      <div class="slate">
        %%SLATE%%
        <div class="cap">An open-source robotics workshop for agents</div>
      </div>
      <p class="caption">The end slate is the co-lockup on near-black. Title and end cards use Sora for names and titles, Inter for captions, with mimosa accents. See <code>social/VIDEO_STYLE.md</code>.</p>
    </div>
  </div>

  <div class="card block">
    <h3>Terminal output</h3>
    <div class="mono-line" style="margin-top:8px"><span class="y">&#9680; &#9681; &#9682;</span> progress in mimosa &nbsp; <span class="ok">&#10003;</span> done in mimosa-soft #FAF6A4 &nbsp; telemetry in ink-mute</div>
    <p class="caption"><code>scripts/cinema/brand.py</code></p>
  </div>
</section>

<!-- 8 -->
<section class="page" id="donts">
  <div class="eyebrow"><b>08</b>Don&rsquo;ts</div>
  <h2>For both marks</h2>
  <p class="lede">Use the shipped files as they are. Mimosa is the one accent, and near-black is the ground.</p>
  <div class="grid g5 dont">
    <div><div class="stage">%%DONT_ROT%%</div><h3>Don&rsquo;t rotate</h3><p class="small mute">The orb is lit from the upper left.</p></div>
    <div><div class="stage">%%DONT_RECOLOUR%%</div><h3>Don&rsquo;t recolour</h3><p class="small mute">Mimosa is the one accent.</p></div>
    <div><div class="stage">%%DONT_SQUASH%%</div><h3>Don&rsquo;t squash or stretch</h3><p class="small mute">Scale proportionally.</p></div>
    <div><div class="stage" style="background:var(--mimosa)">%%DONT_ON_MIMOSA%%</div><h3>Don&rsquo;t put a mark on mimosa</h3><p class="small mute">Not even the ink version.</p></div>
    <div><div class="stage" style="background:linear-gradient(135deg,#3b2f7a,#0A0A06 60%,#7a2f2f)">%%DONT_GRADIENT%%</div><h3>No gradient under a mark</h3><p class="small mute">Flat near-black or surface.</p></div>
  </div>
  <div class="grid g2 block">
    <div class="card">
      <h3>Don&rsquo;t use retired material</h3>
      <p class="small mute">No old dot orb, no stacked OMNI / LINK lockups, no all-caps wordmarks, no ring-and-dot mark, no retired faces or yellows, and no retired lines.</p>
    </div>
    <div class="card">
      <h3>Don&rsquo;t retype a name as a logo</h3>
      <p class="small mute">The wordmarks are outlines. In running text, write the names in the body face, spelled as on the Names page.</p>
    </div>
  </div>
</section>

<footer class="page" id="attribution">
  <div class="eyebrow"><b>&mdash;</b>Attribution</div>
  <p>OmniSim is built on top of Webots by Cyberbotics Ltd., licensed under Apache 2.0. The Webots name and ladybug logo belong to Cyberbotics. They are not part of the OmniSim or OmniLink identity and must not be used to brand either.</p>
  <div class="row block" style="margin-top:56px">
    %%FOOT_LOCKUP%%
    <div class="small mute" style="text-align:right">omnilink-agents.com &middot; info@omnilink-agents.com<br>Sora and Inter are licensed under the SIL Open Font License 1.1.</div>
  </div>
</footer>

</body>
</html>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    html = build()
    args.out.write_text(html, encoding="utf-8", newline="\n")
    print(f"wrote {args.out} ({len(html.encode('utf-8')) / 1024:.0f} KiB)")


if __name__ == "__main__":
    main()
