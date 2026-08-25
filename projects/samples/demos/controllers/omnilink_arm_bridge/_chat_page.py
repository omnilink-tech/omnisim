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

"""Full-page browser chat for the OmniLink arm bridge.

Served by the bridge at GET /chat. It is fully self-contained (inline CSS +
JS) and same-origin: every fetch goes to the bridge that served the page
(window.location.origin), so there are no CORS or port-injection concerns —
robot A on :8765 and robot B on :8766 each open their own tab driving their
own agent.

Endpoints it uses (all on the bridge):
    GET  /chat_config      -> { robot, display_name, tagline, agent,
                                suggestions[], voice, has_ik, has_gripper }
    GET  /get_robot_state  -> live vitals (q, tcp, mode, ...)
    POST /prompt {text}     -> { response, actions:[{tool,result,summary}], error? }

Voice is browser-native (Web Speech API): speechSynthesis reads replies,
SpeechRecognition (Chrome/Edge) powers the mic. No server TTS/STT needed.
"""

CHAT_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OmniLink Console</title>
<style>
:root{
  --bg:#0b0b0c; --panel:#131316; --panel-2:#17171b; --line:#232327; --line-soft:#1b1b1e;
  --text:#f3eddc; --dim:#b8b1a0; --faint:#8a8472;
  --mimosa:#fbe283; --mimosa-deep:#d6a72d; --blue:#8eb6ff; --ok:#6dd178; --warn:#ff9e3d; --err:#ff8a82;
  --radius:16px; --radius-sm:11px;
}
*{box-sizing:border-box;}
html,body{margin:0;height:100%;}
body{
  color:var(--text); font-size:15px; line-height:1.5;
  font-family:"Inter",-apple-system,BlinkMacSystemFont,"Segoe UI","Helvetica Neue",Arial,sans-serif;
  background:
    radial-gradient(1200px 520px at 50% -10%, rgba(251,226,131,0.08), transparent 60%),
    radial-gradient(900px 600px at 110% 10%, rgba(142,182,255,0.05), transparent 60%),
    linear-gradient(180deg,#0e0e11 0%,#0a0a0b 100%);
  -webkit-font-smoothing:antialiased;
  display:flex; flex-direction:column; align-items:center; min-height:100%;
}
.wrap{width:100%; max-width:820px; min-height:100vh; display:flex; flex-direction:column; padding:18px;}

/* header */
.top{display:flex; align-items:center; justify-content:space-between; padding-bottom:14px; border-bottom:1px solid var(--line-soft);}
.brand{display:flex; align-items:center; gap:9px;}
.orb{width:16px;height:16px;border-radius:50%;
  background:radial-gradient(circle at 32% 28%,#fff6d3 0%,#fbe283 28%,#d6a72d 62%,#6e4e0d 100%);
  box-shadow:0 0 9px rgba(251,226,131,.55);}
.brand b{font-weight:600;letter-spacing:.09em;font-size:12px;text-transform:uppercase;color:var(--dim);}
.pill{display:flex;align-items:center;gap:7px;font-size:12px;color:var(--dim);
  background:rgba(255,255,255,.03);border:1px solid var(--line-soft);border-radius:999px;padding:5px 12px;}
.dot{width:9px;height:9px;border-radius:50%;background:#555;}
.dot.connected{background:var(--ok);box-shadow:0 0 6px var(--ok);}
.dot.thinking{background:var(--mimosa);box-shadow:0 0 6px var(--mimosa);animation:pulse 1s infinite;}
.dot.speaking{background:var(--warn);box-shadow:0 0 6px var(--warn);animation:pulse .6s infinite;}
.dot.error{background:var(--err);box-shadow:0 0 6px var(--err);}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}

/* hero */
.hero{display:flex;align-items:center;gap:16px;margin-top:16px;
  background:linear-gradient(160deg,var(--panel-2),var(--panel));border:1px solid var(--line);
  border-radius:var(--radius);padding:16px 18px;box-shadow:0 8px 26px rgba(0,0,0,.4);}
.avatar{flex:0 0 auto;width:58px;height:58px;border-radius:50%;display:flex;align-items:center;justify-content:center;
  font-weight:700;font-size:26px;color:#1a1206;
  background:radial-gradient(circle at 34% 26%,#fff7d6 0%,#fbe283 32%,#e0b53c 64%,#9a6e15 100%);
  box-shadow:0 0 16px rgba(251,226,131,.4),inset 0 -3px 7px rgba(110,78,13,.45),inset 0 2px 5px rgba(255,255,255,.55);}
.avatar.thinking{animation:glow 1.5s ease-in-out infinite;}
.avatar.speaking{animation:ripple 1.05s ease-out infinite;}
@keyframes glow{0%,100%{box-shadow:0 0 14px rgba(251,226,131,.35),inset 0 -3px 7px rgba(110,78,13,.45),inset 0 2px 5px rgba(255,255,255,.55)}50%{box-shadow:0 0 26px rgba(251,226,131,.85),inset 0 -3px 7px rgba(110,78,13,.45),inset 0 2px 5px rgba(255,255,255,.55)}}
@keyframes ripple{0%{box-shadow:0 0 0 0 rgba(251,226,131,.5),inset 0 -3px 7px rgba(110,78,13,.45),inset 0 2px 5px rgba(255,255,255,.55)}100%{box-shadow:0 0 0 16px rgba(251,226,131,0),inset 0 -3px 7px rgba(110,78,13,.45),inset 0 2px 5px rgba(255,255,255,.55)}}
.htext{flex:1 1 auto;min-width:0;}
.hname{font-size:21px;font-weight:600;}
.htag{font-size:12px;color:var(--dim);margin-top:2px;}
.hagent{margin-top:7px;display:inline-flex;gap:6px;align-items:center;font-size:10px;
  background:rgba(255,255,255,.03);border:1px solid var(--line-soft);border-radius:999px;padding:2px 9px;}
.hagent .k{color:var(--faint);text-transform:uppercase;letter-spacing:.07em;}
.hagent .v{color:var(--mimosa);font-family:"SF Mono",Menlo,Consolas,monospace;}
.iconbtn{flex:0 0 auto;width:38px;height:38px;border-radius:11px;border:1px solid var(--line);
  background:rgba(255,255,255,.02);color:var(--text);font-size:17px;cursor:pointer;transition:.15s;}
.iconbtn:hover{background:rgba(255,255,255,.06);}

/* vitals */
.vitals{display:flex;gap:8px;margin-top:12px;flex-wrap:wrap;}
.vital{flex:1 1 120px;background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:9px 12px;}
.vital .k{font-size:9.5px;color:var(--faint);text-transform:uppercase;letter-spacing:.07em;}
.vital .v{font-size:13px;color:var(--text);font-family:"SF Mono",Menlo,Consolas,monospace;margin-top:3px;
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}

/* chat */
.chat{flex:1 1 auto;min-height:220px;display:flex;flex-direction:column;margin-top:12px;
  background:#08080a;border:1px solid var(--line);border-radius:var(--radius);overflow:hidden;box-shadow:inset 0 2px 12px rgba(0,0,0,.4);}
.log{flex:1 1 auto;overflow-y:auto;padding:16px;display:flex;flex-direction:column;gap:10px;}
.log::-webkit-scrollbar{width:10px;}
.log::-webkit-scrollbar-thumb{background:#2a2a30;border-radius:999px;border:2px solid transparent;background-clip:content-box;}
.msg{padding:10px 13px;border-radius:15px;max-width:86%;word-wrap:break-word;white-space:pre-wrap;
  animation:in .22s ease-out;box-shadow:0 2px 7px rgba(0,0,0,.25);}
@keyframes in{from{opacity:0;transform:translateY(7px)}to{opacity:1;transform:none}}
.msg.user{align-self:flex-end;border-radius:15px 15px 5px 15px;color:#1c1505;font-weight:500;
  background:linear-gradient(160deg,#fbe283,#eccd63);}
.msg.agent{align-self:flex-start;border-radius:15px 15px 15px 5px;background:linear-gradient(160deg,#1a1a1e,#141417);border:1px solid var(--line);}
.msg.tool{align-self:flex-start;font-family:"SF Mono",Menlo,Consolas,monospace;font-size:12px;border-radius:10px;
  background:rgba(109,209,120,.06);border:1px solid rgba(109,209,120,.22);color:var(--ok);box-shadow:none;}
.msg.tool.err{color:var(--err);background:rgba(255,138,130,.07);border-color:rgba(255,138,130,.25);}
.msg.err{align-self:stretch;background:linear-gradient(160deg,#2c1414,#241010);border:1px solid rgba(255,138,130,.3);color:var(--err);}
.typing{display:inline-flex;gap:5px;align-items:center;}
.typing span{width:7px;height:7px;border-radius:50%;background:var(--dim);animation:bounce 1.25s infinite ease-in-out;}
.typing span:nth-child(2){animation-delay:.15s;} .typing span:nth-child(3){animation-delay:.3s;}
@keyframes bounce{0%,80%,100%{transform:translateY(0);opacity:.35}40%{transform:translateY(-5px);opacity:1}}

/* chips */
.chips{display:flex;flex-wrap:wrap;gap:7px;margin-top:12px;}
.chip{border:1px solid var(--line);background:rgba(255,255,255,.025);color:var(--dim);border-radius:999px;
  padding:6px 13px;font-size:12px;cursor:pointer;transition:.15s;font-family:inherit;}
.chip:hover{background:rgba(251,226,131,.08);border-color:var(--mimosa-deep);color:var(--mimosa);transform:translateY(-1px);}

/* composer */
.composer{display:flex;gap:9px;margin-top:12px;align-items:flex-end;}
textarea{flex:1 1 auto;resize:none;background:var(--panel);color:var(--text);border:1px solid var(--line);
  border-radius:var(--radius-sm);padding:12px 14px;font-family:inherit;font-size:15px;outline:none;min-height:48px;max-height:140px;transition:.15s;}
textarea:focus{border-color:var(--mimosa-deep);box-shadow:0 0 0 3px rgba(251,226,131,.12);}
.btn{padding:0 16px;height:48px;border-radius:var(--radius-sm);border:1px solid var(--line);background:rgba(255,255,255,.025);
  color:var(--text);font-family:inherit;font-size:14px;font-weight:500;cursor:pointer;transition:.15s;white-space:nowrap;}
.btn:hover{background:rgba(255,255,255,.06);}
.btn.primary{background:linear-gradient(160deg,#fbe283,#e6c354);color:#0b0b0c;border-color:var(--mimosa-deep);font-weight:600;box-shadow:0 3px 14px rgba(251,226,131,.22);}
.btn.mic{background:rgba(142,182,255,.08);border-color:rgba(142,182,255,.28);color:var(--blue);}
.btn.mic.rec{background:linear-gradient(160deg,#3a1a18,#2c1414);border-color:rgba(255,138,130,.45);color:var(--err);animation:pulse 1s infinite;}
.hidden{display:none!important;}
.foot{text-align:center;color:var(--faint);font-size:10.5px;margin-top:10px;}
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div class="brand"><span class="orb"></span><b>OmniLink</b></div>
    <div class="pill"><span id="dot" class="dot"></span><span id="statusText">connecting…</span></div>
  </div>

  <div class="hero">
    <div id="avatar" class="avatar">·</div>
    <div class="htext">
      <div id="name" class="hname">—</div>
      <div id="tag" class="htag">connecting…</div>
      <div class="hagent"><span class="k">agent</span><span id="agent" class="v">—</span></div>
    </div>
    <button id="speaker" class="iconbtn" title="Toggle voice">🔊</button>
  </div>

  <div class="vitals">
    <div class="vital"><div class="k">Mode</div><div id="vMode" class="v">—</div></div>
    <div class="vital"><div class="k">TCP (x, y, z)</div><div id="vTcp" class="v">—</div></div>
    <div class="vital"><div class="k">Joints</div><div id="vJoints" class="v">—</div></div>
  </div>

  <div class="chat"><div id="log" class="log"></div></div>
  <div id="chips" class="chips"></div>

  <div class="composer">
    <textarea id="input" rows="1" placeholder="Talk to the robot…"></textarea>
    <button id="mic" class="btn mic hidden">🎙 Talk</button>
    <button id="send" class="btn primary">Send</button>
  </div>
  <div class="foot">OmniLink Console · same-origin chat with this robot's agent</div>
</div>

<script>
const API = window.location.origin;
const $ = (id)=>document.getElementById(id);
let speak = true, recog = null, recording = false;

function status(state, label){
  const known = ['connected','thinking','speaking','error'];
  $('dot').className = 'dot ' + (known.includes(state)?state:'');
  $('statusText').textContent = label || state;
  $('avatar').classList.remove('thinking','speaking');
  if(state==='thinking') $('avatar').classList.add('thinking');
  if(state==='speaking') $('avatar').classList.add('speaking');
}
function add(kind, text, extra){
  const d = document.createElement('div');
  d.className = 'msg ' + kind + (extra?(' '+extra):'');
  d.textContent = text;
  $('log').appendChild(d);
  if(typingEl && typingEl.parentNode===$('log')) $('log').appendChild(typingEl);
  $('log').scrollTop = $('log').scrollHeight;
  return d;
}
let typingEl=null;
function showTyping(){
  if(!typingEl){ typingEl=document.createElement('div'); typingEl.className='msg agent';
    typingEl.innerHTML='<span class="typing"><span></span><span></span><span></span></span>'; }
  $('log').appendChild(typingEl); $('log').scrollTop=$('log').scrollHeight;
}
function hideTyping(){ if(typingEl&&typingEl.parentNode) typingEl.parentNode.removeChild(typingEl); }

function sayOut(text){
  if(!speak || !text || !window.speechSynthesis) return;
  try{ const u=new SpeechSynthesisUtterance(text); u.rate=1.02; u.pitch=1.05;
    status('speaking','speaking…'); u.onend=()=>status('connected','idle');
    window.speechSynthesis.cancel(); window.speechSynthesis.speak(u);
  }catch(e){}
}

async function ask(text){
  if(!text.trim()) return;
  add('user', text);
  status('thinking','thinking…'); showTyping();
  try{
    const r = await fetch(API+'/prompt', {method:'POST',headers:{'Content-Type':'application/json'},
      body: JSON.stringify({text})});
    const data = await r.json();
    hideTyping();
    (data.actions||[]).forEach(a=>{
      add('tool', '→ '+a.tool+' · '+(a.summary||a.result||''), a.result==='ok'?'':'err');
    });
    if(data.error){ add('err', data.error); status('error','error'); return; }
    const reply = data.response || '(no reply)';
    add('agent', reply); status('connected','idle'); sayOut(reply);
  }catch(e){ hideTyping(); add('err','network error: '+e); status('error','error'); }
}

// ── Skill-learning live feed (SSE) ──────────────────────────────────
// Streams learn-pipeline progress (stage PASS/FAIL, certification) into
// the chat. Optional: bridges without the learn manager 503 the endpoint
// and the feed silently stays off. Dedupe by seq so the replay-then-
// stream contract (and EventSource auto-reconnect) never double-prints.
let sseNext = 0, sseGotAny = false;
function factoryFeed(){
  if(!window.EventSource) return;
  let es;
  try{ es = new EventSource(API+'/factory/events'); }catch(e){ return; }
  es.onmessage = (m)=>{
    let e; try{ e = JSON.parse(m.data); }catch(err){ return; }
    sseGotAny = true;
    if(typeof e.seq === 'number'){
      if(e.seq < sseNext) return;   // already shown (replay/reconnect)
      sseNext = e.seq + 1;
    }
    if(e.ev === 'stage'){
      const label = e.state === 'running' ? '…' : String(e.state||'').toUpperCase();
      add('tool', '⚙ learn·'+e.stage+' · '+label
          +(e.detail?(' — '+e.detail):'')
          +(typeof e.pct==='number'?(' ('+Math.round(e.pct)+'%)'):''),
          e.state==='fail'?'err':'');
    } else if(e.ev === 'done'){
      if(e.ok){
        const line = 'New skill learned — say "'+(e.verb||'it')+'" to run it. '+(e.summary||'');
        add('agent', line); sayOut(line);
      } else {
        add('err', 'Learning failed. '+(e.summary||''));
      }
    } else if(e.ev === 'start'){
      add('tool', '⚙ learn · started — recipe '+(e.recipe||'?'));
    }
  };
  es.onerror = ()=>{
    try{ es.close(); }catch(err){}
    // Endpoint absent (no learn manager): give up. Transient drop after
    // a successful connect: retry, seq-dedupe suppresses the replay.
    if(sseGotAny) setTimeout(factoryFeed, 5000);
  };
}

function fmt(n){ return (n>=0?'+':'')+n.toFixed(2); }
async function poll(){
  try{
    const r = await fetch(API+'/get_robot_state'); const s = await r.json();
    $('vMode').textContent = s.mode || '—';
    $('vTcp').textContent = s.tcp ? ('('+s.tcp.map(v=>v.toFixed(2)).join(', ')+')') : 'n/a';
    $('vJoints').textContent = s.q ? ('['+s.q.map(fmt).join(', ')+']') : '—';
  }catch(e){}
}

async function boot(){
  status('connected','connecting…');
  try{
    const r = await fetch(API+'/chat_config'); const c = await r.json();
    const nm = c.display_name || c.robot || 'Robot';
    document.title = nm + ' — OmniLink Console';
    $('name').textContent = nm;
    $('tag').textContent = c.tagline || c.robot_class || '';
    $('agent').textContent = c.agent || '—';
    $('avatar').textContent = (nm.trim()[0]||'·').toUpperCase();
    (c.suggestions||[]).forEach(s=>{
      const b=document.createElement('button'); b.className='chip'; b.textContent=s;
      b.onclick=()=>{ ask(s); }; $('chips').appendChild(b);
    });
    if(c.greeting){ add('agent', c.greeting); }
    // mic only if the browser supports speech recognition
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    if(SR){
      $('mic').classList.remove('hidden');
      recog = new SR(); recog.lang='en-US'; recog.interimResults=false; recog.maxAlternatives=1;
      recog.onresult=(ev)=>{ const t=ev.results[0][0].transcript; stopRec(); ask(t); };
      recog.onerror=()=>stopRec(); recog.onend=()=>stopRec();
    }
    status('connected','connected');
  }catch(e){ add('err','could not load config: '+e); status('error','error'); }
  poll(); setInterval(poll, 1300);
  factoryFeed();
}

function startRec(){ if(!recog||recording) return; try{ recog.start(); recording=true;
  $('mic').classList.add('rec'); $('mic').textContent='● Listening — click to stop'; status('thinking','listening…'); }catch(e){} }
function stopRec(){ recording=false; $('mic').classList.remove('rec'); $('mic').textContent='🎙 Talk';
  try{ recog && recog.stop(); }catch(e){} if($('dot').className.indexOf('thinking')>=0) status('connected','idle'); }

$('send').onclick=()=>{ const t=$('input').value; $('input').value=''; ask(t); };
$('input').addEventListener('keydown',(e)=>{ if(e.key==='Enter'&&!e.shiftKey){ e.preventDefault(); $('send').click(); }});
$('mic').onclick=()=>{ recording?stopRec():startRec(); };
$('speaker').onclick=()=>{ speak=!speak; $('speaker').textContent=speak?'🔊':'🔇';
  if(!speak&&window.speechSynthesis) window.speechSynthesis.cancel(); };

boot();
</script>
</body>
</html>
"""
