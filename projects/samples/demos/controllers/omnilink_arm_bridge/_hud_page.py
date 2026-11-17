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

"""Skill-learning live HUD — fullscreen pipeline board for the arm bridge.

Served by the bridge at GET /hud (Content-Type: text/html; charset=utf-8).
Fully self-contained (inline CSS + JS, zero external resources — safe to show
offline) and same-origin: it opens one EventSource on GET /factory/events at
window.location.origin, so no CORS concerns.

Event contract consumed (one JSON object per SSE `data:` line):

    {"ev":"stage","stage":"design|validate|train|certify",
     "state":"running|pass|fail","detail":"...","pct":0-100}
    {"ev":"log","line":"..."}
    {"ev":"done","ok":true,"verb":"toss","summary":"..."}

The endpoint is expected to replay history on (re)connect, then stream.
The page fully rebuilds its board from a replay: on every EventSource open
it resets to the waiting state and re-applies whatever arrives, so native
auto-reconnect is safe and idempotent. A new "stage" event arriving after a
"done" resets the board for the next learn in the same session.

Keyboard: F toggles fullscreen. Honors prefers-reduced-motion (static orb,
no pulse/dash animation). Designed for a 16:9 display at 1080p, readable
from meters away.
"""

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="dark">
<title>OmniSim · Skill learning</title>
<style>
:root{
  color-scheme: dark;
  --ground:#0A0A06; --panel:#141410; --edge:#26261C;
  --ink:#F2EFE4; --muted:#9B9A8A;
  --mimosa:#F6E905; --pass:#63C77A; --fail:#E5604C;
  --disp:"Segoe UI Variable Display","Segoe UI",system-ui,sans-serif;
  --mono:"Cascadia Mono",Consolas,ui-monospace,"SF Mono",Menlo,monospace;
}
*{box-sizing:border-box;margin:0;padding:0;}
html,body{height:100%;}
body{
  background:var(--ground); color:var(--ink); font-family:var(--disp);
  overflow:hidden; -webkit-font-smoothing:antialiased; cursor:default;
}
#app{
  height:100vh; display:grid;
  grid-template-columns:340px 1fr; grid-template-rows:1fr 150px;
}

/* ---------- eyebrows / shared type ---------- */
.eyebrow{
  font-family:var(--mono); font-size:11px; letter-spacing:.22em;
  text-transform:uppercase; color:var(--muted);
}
.num{font-variant-numeric:tabular-nums; font-feature-settings:"tnum";}

/* ---------- left rail ---------- */
#rail{
  grid-row:1 / 3; border-right:1px solid var(--edge); background:var(--panel);
  display:flex; flex-direction:column; align-items:center;
  padding:44px 24px 24px; gap:8px;
}
#orb{width:250px; height:250px; flex:0 0 auto;}
#orb.pulse{animation:orbpulse 1.5s ease-out 1;}
@keyframes orbpulse{
  0%{filter:drop-shadow(0 0 0 rgba(246,233,5,0));}
  22%{filter:drop-shadow(0 0 34px rgba(246,233,5,.65));}
  100%{filter:drop-shadow(0 0 0 rgba(246,233,5,0));}
}
.brandline{margin-top:10px; color:var(--ink); letter-spacing:.3em;}
.railsep{width:56px; border-top:1px solid var(--edge); margin:30px 0 26px;}
#clock{
  font-family:var(--disp); font-weight:800; font-size:78px; line-height:1;
  letter-spacing:-.02em; margin-top:10px;
  font-variant-numeric:tabular-nums; font-feature-settings:"tnum";
}
#conn{margin-top:auto; display:flex; align-items:center; gap:8px;}
.cdot{width:8px; height:8px; border-radius:50%; background:var(--muted); flex:0 0 auto;}
.cdot.live{background:var(--pass); box-shadow:0 0 6px var(--pass);}
.cdot.retry{background:var(--fail); box-shadow:0 0 6px var(--fail);
  animation:blink 1s step-end infinite;}
@keyframes blink{50%{opacity:.25;}}

/* ---------- center stage ---------- */
#stagearea{
  grid-column:2; grid-row:1; display:flex; flex-direction:column;
  align-items:center; justify-content:center; padding:0 56px; gap:34px;
}
#statusline{font-size:13px; text-align:center; min-height:18px; transition:color .3s;}
#statusline.wait{color:var(--muted);}
#statusline.run{color:var(--mimosa);}
#statusline.ok{color:var(--pass);}
#statusline.bad{color:var(--fail);}

#thread{
  width:100%; max-width:1420px; display:grid; align-items:start;
  grid-template-columns:180px 1fr 180px 1fr 180px 1fr 180px 1fr 200px;
  grid-template-rows:auto auto;
}
.seg{
  grid-row:1; height:3px; margin-top:60px; /* disc is 122px -> center ~61 */
  background:var(--edge); border-radius:2px; transition:background .3s;
}
.seg.filled{background:var(--mimosa); box-shadow:0 0 10px rgba(246,233,5,.35);}
.seg.running{
  background-image:linear-gradient(90deg,var(--mimosa) 0 15px,transparent 15px 28px);
  background-size:28px 3px; background-repeat:repeat-x;
  animation:dash .7s linear infinite;
}
@keyframes dash{from{background-position-x:0;}to{background-position-x:28px;}}

.stage{grid-row:1; display:flex; flex-direction:column; align-items:center; gap:10px;}
.disc{
  width:122px; height:122px; border-radius:50%; border:2px solid var(--edge);
  background:var(--panel); display:grid; place-items:center;
  transition:border-color .3s, box-shadow .3s;
}
.glyph{
  font-family:var(--mono); font-size:22px; color:var(--muted);
  font-variant-numeric:tabular-nums; letter-spacing:.06em; text-align:center;
  max-width:104px; overflow-wrap:break-word;
}
.sname{font-family:var(--mono); font-size:13px; letter-spacing:.24em;
  text-transform:uppercase; color:var(--muted); margin-top:4px; transition:color .3s;}
.sstate{font-family:var(--mono); font-size:10px; letter-spacing:.2em;
  text-transform:uppercase; color:var(--muted); min-height:13px;}
.sdetail{
  font-family:var(--mono); font-size:12px; color:var(--muted); text-align:center;
  line-height:1.45; max-width:190px; min-height:36px;
  display:-webkit-box; -webkit-line-clamp:2; -webkit-box-orient:vertical; overflow:hidden;
}

.stage.running .disc{
  border-color:var(--mimosa);
  animation:discpulse 1.6s ease-in-out infinite;
}
@keyframes discpulse{
  0%,100%{box-shadow:0 0 14px rgba(246,233,5,.22);}
  50%{box-shadow:0 0 34px rgba(246,233,5,.55);}
}
.stage.running .glyph{color:var(--mimosa); font-size:34px;}
.stage.running .sname{color:var(--ink);}
.stage.running .sstate{color:var(--mimosa);}

.stage.pass .disc{border-color:var(--pass); box-shadow:0 0 16px rgba(99,199,122,.22);}
.stage.pass .glyph{color:var(--pass); font-size:44px; font-family:var(--disp); font-weight:800;}
.stage.pass .sname{color:var(--ink);}
.stage.pass .sstate{color:var(--pass);}

.stage.fail .disc{border-color:var(--fail); box-shadow:0 0 16px rgba(229,96,76,.28);}
.stage.fail .glyph{color:var(--fail); font-size:44px; font-family:var(--disp); font-weight:800;}
.stage.fail .sname{color:var(--ink);}
.stage.fail .sstate{color:var(--fail);}
.stage.fail .sdetail{color:var(--fail);}

/* fifth node: the certified verb */
#st-verb .disc{border-style:dashed; opacity:.35;}
#st-verb .sname,#st-verb .sstate{opacity:.35;}
#st-verb.lit .disc{
  border-style:solid; border-color:var(--mimosa); opacity:1;
  box-shadow:0 0 30px rgba(246,233,5,.45);
}
#st-verb.lit .sname,#st-verb.lit .sstate{opacity:1;}
#st-verb.lit .glyph{
  color:var(--mimosa); font-family:var(--disp); font-weight:800;
  font-size:26px; letter-spacing:.02em; text-transform:uppercase;
}
#st-verb.lit .sname{color:var(--mimosa);}
#st-verb.lit .sstate{color:var(--mimosa);}

/* train progress (row 2, under the TRAIN column) */
#trainprog{
  grid-row:2; grid-column:4 / 7; margin-top:26px; justify-self:center;
  width:100%; max-width:460px; display:flex; flex-direction:column; gap:9px;
}
#trainprog.hidden{visibility:hidden;}
.pbar{
  height:16px; border:1px solid var(--edge); border-radius:8px;
  background:var(--ground); overflow:hidden;
}
#pfill{
  height:100%; width:0%; background:var(--mimosa); border-radius:7px;
  box-shadow:0 0 14px rgba(246,233,5,.45); transition:width .4s ease;
}
.pmeta{display:flex; justify-content:space-between; gap:16px; align-items:baseline;}
#ppct{font-family:var(--disp); font-weight:800; font-size:26px;
  font-variant-numeric:tabular-nums; font-feature-settings:"tnum"; color:var(--mimosa);}
#pdetail{font-family:var(--mono); font-size:12px; color:var(--muted);
  text-align:right; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}

#summary{
  font-family:var(--disp); font-weight:600; font-size:22px; letter-spacing:-.01em;
  color:var(--ink); text-align:center; min-height:30px; max-width:1000px;
}

/* ---------- bottom terminal ---------- */
#term{
  grid-column:2; grid-row:2; border-top:1px solid var(--edge);
  background:var(--panel); padding:18px 44px 16px;
  display:flex; flex-direction:column; justify-content:flex-end; gap:5px;
  font-family:var(--mono); font-size:14px; color:var(--muted);
  overflow:hidden; white-space:nowrap;
}
#term .tl{overflow:hidden; text-overflow:ellipsis; flex:0 0 auto;}
#term .tl:nth-last-child(4){opacity:.3;}
#term .tl:nth-last-child(3){opacity:.5;}
#term .tl:nth-last-child(2){opacity:.75;}
#term .tl:nth-last-child(1){opacity:1; color:var(--ink);}

@media (prefers-reduced-motion: reduce){
  *{animation:none !important; transition:none !important;}
  .seg.running{background-image:none; background:var(--mimosa);}
  .stage.running .disc{box-shadow:0 0 24px rgba(246,233,5,.4);}
}
</style>
</head>
<body>
<div id="app">

  <aside id="rail">
    <canvas id="orb" width="250" height="250"></canvas>
    <div class="eyebrow brandline">OMNISIM · SKILL LEARNING</div>
    <div class="railsep"></div>
    <div class="eyebrow">ELAPSED</div>
    <div id="clock" class="num">00:00</div>
    <div id="conn">
      <span id="conndot" class="cdot"></span>
      <span id="conntext" class="eyebrow">CONNECTING…</span>
    </div>
  </aside>

  <main id="stagearea">
    <div id="statusline" class="eyebrow wait"></div>

    <div id="thread">
      <div class="stage" id="st-design">
        <div class="disc"><span class="glyph">01</span></div>
        <div class="sname">Design</div><div class="sstate">·</div>
        <div class="sdetail"></div>
      </div>
      <div class="seg" id="seg-validate"></div>
      <div class="stage" id="st-validate">
        <div class="disc"><span class="glyph">02</span></div>
        <div class="sname">Validate</div><div class="sstate">·</div>
        <div class="sdetail"></div>
      </div>
      <div class="seg" id="seg-train"></div>
      <div class="stage" id="st-train">
        <div class="disc"><span class="glyph">03</span></div>
        <div class="sname">Train</div><div class="sstate">·</div>
        <div class="sdetail"></div>
      </div>
      <div class="seg" id="seg-certify"></div>
      <div class="stage" id="st-certify">
        <div class="disc"><span class="glyph">04</span></div>
        <div class="sname">Certify</div><div class="sstate">·</div>
        <div class="sdetail"></div>
      </div>
      <div class="seg" id="seg-verb"></div>
      <div class="stage" id="st-verb">
        <div class="disc"><span class="glyph">◇</span></div>
        <div class="sname">New verb</div><div class="sstate">·</div>
        <div class="sdetail"></div>
      </div>

      <div id="trainprog" class="hidden">
        <div class="pbar"><div id="pfill"></div></div>
        <div class="pmeta"><span id="ppct">0%</span><span id="pdetail"></span></div>
      </div>
    </div>

    <div id="summary"></div>
  </main>

  <footer id="term"></footer>
</div>

<script>
'use strict';
const $ = (id) => document.getElementById(id);
const REDUCED = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const WAIT_MSG = "waiting for a learn request — type 'learn …' in the chat";
const ORDER = ['design', 'validate', 'train', 'certify'];

/* ---------------- board state ---------------- */
const S = {};           // stage name -> {state:'idle'|'running'|'pass'|'fail'}
let finished = false;   // a done event closed the current learn
let startTs = null, clockTimer = null;
let termLines = [];

function setStatus(cls, text) {
  const el = $('statusline');
  el.className = 'eyebrow ' + cls;
  el.textContent = text;
}

function startClock() {
  startTs = Date.now();
  if (clockTimer) clearInterval(clockTimer);
  clockTimer = setInterval(tickClock, 250);
  tickClock();
}
function stopClock() { if (clockTimer) { clearInterval(clockTimer); clockTimer = null; } }
function tickClock() {
  if (startTs === null) return;
  const s = Math.max(0, Math.floor((Date.now() - startTs) / 1000));
  $('clock').textContent =
    String(Math.floor(s / 60)).padStart(2, '0') + ':' + String(s % 60).padStart(2, '0');
}

function term(line) {
  if (!line) return;
  termLines.push(line);
  if (termLines.length > 4) termLines.shift();
  const el = $('term');
  el.textContent = '';
  for (const l of termLines) {
    const d = document.createElement('div');
    d.className = 'tl';
    d.textContent = '› ' + l;   /* › prefix */
    el.appendChild(d);
  }
}

function resetBoard(clearTerm) {
  finished = false;
  stopClock();
  startTs = null;
  $('clock').textContent = '00:00';
  ORDER.forEach((name, i) => {
    S[name] = { state: 'idle' };
    const st = $('st-' + name);
    st.className = 'stage';
    st.querySelector('.glyph').textContent = '0' + (i + 1);
    st.querySelector('.sstate').textContent = '·';
    st.querySelector('.sdetail').textContent = '';
  });
  const v = $('st-verb');
  v.className = 'stage';
  v.querySelector('.glyph').textContent = '◇';
  v.querySelector('.sname').textContent = 'New verb';
  v.querySelector('.sstate').textContent = '·';
  S.certified = false;
  paintSegs();
  $('trainprog').className = 'hidden';
  $('summary').textContent = '';
  setStatus('wait', WAIT_MSG);
  if (clearTerm) { termLines = []; $('term').textContent = ''; }
}

function paintSegs() {
  for (let i = 1; i < ORDER.length; i++) {
    const into = S[ORDER[i]].state, from = S[ORDER[i - 1]].state;
    const seg = $('seg-' + ORDER[i]);
    if (into === 'running') seg.className = 'seg running';
    else if (into === 'pass' || into === 'fail' || from === 'pass')
      seg.className = 'seg filled';
    else seg.className = 'seg';
  }
  $('seg-verb').className = 'seg' + (S.certified ? ' filled' : '');
}

const GLYPHS = { running: '●', pass: '✓', fail: '✗' };
const LABELS = { running: 'running', pass: 'pass', fail: 'fail' };

function onStage(d) {
  if (finished) resetBoard(false);          // a new learn begins after a done
  if (!ORDER.includes(d.stage)) return;
  if (startTs === null) startClock();

  const state = d.state;
  S[d.stage].state = state;
  const st = $('st-' + d.stage);
  st.className = 'stage ' + state;
  st.querySelector('.glyph').textContent =
    GLYPHS[state] || '0' + (ORDER.indexOf(d.stage) + 1);
  st.querySelector('.sstate').textContent = LABELS[state] || '·';
  if (d.detail !== undefined && d.detail !== null)
    st.querySelector('.sdetail').textContent = d.detail;

  // TRAIN gets the big progress bar while running
  if (d.stage === 'train') {
    if (state === 'running') {
      $('trainprog').className = '';
      const pct = Math.max(0, Math.min(100, Number(d.pct) || 0));
      $('pfill').style.width = pct + '%';
      $('ppct').textContent = pct.toFixed(0) + '%';
      $('pdetail').textContent = d.detail || '';
    } else {
      $('trainprog').className = 'hidden';
    }
  }

  if (state === 'fail') {
    stopClock();
    setStatus('bad', 'fail · ' + d.stage + (d.detail ? ' — ' + d.detail : ''));
  } else {
    setStatus('run', 'learning · ' + d.stage + ' ' + (LABELS[state] || ''));
  }
  if (d.detail) term(d.stage + ': ' + d.detail);
  paintSegs();
}

function onDone(d) {
  finished = true;
  stopClock();
  $('trainprog').className = 'hidden';
  if (d.ok) {
    S.certified = true;
    const verb = String(d.verb || 'skill');
    const v = $('st-verb');
    v.className = 'stage lit';
    v.querySelector('.glyph').textContent = verb;
    v.querySelector('.sname').textContent = 'New verb: ' + verb;
    v.querySelector('.sstate').textContent = 'certified';
    $('summary').textContent = d.summary || '';
    setStatus('ok', 'skill certified · new verb: ' + verb);
    orbPulse();
  } else {
    setStatus('bad', 'run failed' + (d.summary ? ' — ' + d.summary : ''));
    $('summary').textContent = d.summary || '';
  }
  if (d.summary) term(d.summary);
  paintSegs();
}

function handle(d) {
  if (!d || typeof d !== 'object') return;
  if (d.ev === 'stage') onStage(d);
  else if (d.ev === 'log') term(d.line);
  else if (d.ev === 'done') onDone(d);
}

/* ---------------- SSE ---------------- */
// Same origin as the bridge that served this page; native auto-reconnect.
// The endpoint replays history on connect, so every (re)open does a full
// board reset and lets the replay rebuild the state — idempotent.
const es = new EventSource('/factory/events');
es.onopen = () => {
  $('conndot').className = 'cdot live';
  $('conntext').textContent = 'SSE · LIVE';
  resetBoard(true);
};
es.onerror = () => {
  $('conndot').className = 'cdot retry';
  $('conntext').textContent = 'SSE · RECONNECTING…';
};
es.onmessage = (m) => { try { handle(JSON.parse(m.data)); } catch (e) {} };

/* ---------------- fullscreen ---------------- */
document.addEventListener('keydown', (e) => {
  if (e.key === 'f' || e.key === 'F') {
    if (document.fullscreenElement) document.exitFullscreen();
    else document.documentElement.requestFullscreen().catch(() => {});
  }
});

/* ---------------- the orb ---------------- */
(function orbInit() {
  const c = $('orb'), ctx = c.getContext('2d');
  const CSS = 250, DPR = window.devicePixelRatio || 1;
  c.width = CSS * DPR; c.height = CSS * DPR;
  ctx.scale(DPR, DPR);
  const N = 340, GA = Math.PI * (3 - Math.sqrt(5));   // golden angle
  const pts = [];
  for (let i = 0; i < N; i++) {
    const y = 1 - 2 * (i + 0.5) / N;
    const r = Math.sqrt(1 - y * y);
    const th = i * GA;
    pts.push([r * Math.cos(th), y, r * Math.sin(th)]);
  }
  const TILT = 0.35, ct = Math.cos(TILT), st = Math.sin(TILT);
  function draw(t) {
    const a = t * 0.00010;                 // ~63 s per revolution — ambient
    const ca = Math.cos(a), sa = Math.sin(a);
    const cx = CSS / 2, cy = CSS / 2, R = CSS * 0.43;
    ctx.clearRect(0, 0, CSS, CSS);
    for (const p of pts) {
      const x1 = p[0] * ca + p[2] * sa;
      const z1 = -p[0] * sa + p[2] * ca;
      const y2 = p[1] * ct - z1 * st;
      const z2 = p[1] * st + z1 * ct;      // depth toward viewer
      const px = cx + x1 * R, py = cy - y2 * R;
      const near = z2 > 0.35;
      const rad = 0.9 + (z2 + 1) * 0.75;
      ctx.beginPath();
      ctx.arc(px, py, rad, 0, 6.2832);
      if (near) ctx.fillStyle = 'rgba(246,233,5,' + (0.55 + z2 * 0.4).toFixed(3) + ')';
      else ctx.fillStyle = 'rgba(242,239,228,' + (0.10 + (z2 + 1) * 0.14).toFixed(3) + ')';
      ctx.fill();
    }
  }
  if (REDUCED) { draw(0); }
  else {
    const loop = (t) => { draw(t); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }
  c.addEventListener('animationend', () => c.classList.remove('pulse'));
})();

function orbPulse() {
  if (REDUCED) return;
  const c = $('orb');
  c.classList.remove('pulse');
  void c.offsetWidth;                       // restart the one-shot animation
  c.classList.add('pulse');
}

resetBoard(true);
</script>
</body>
</html>
"""
