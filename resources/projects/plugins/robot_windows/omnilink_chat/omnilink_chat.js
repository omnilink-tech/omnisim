// OmniLink robot window — chat-style prompt console with optional voice.
//
// Protocol with the controller (bridge):
//   window → controller
//     "configure"               handshake; ask bridge to send its config
//     "prompt:<text>"           operator prompt
//     "audio_in:<json>"         mic clip {audio_b64, mime_type} for STT
//     "stop"                    emergency halt
//
//   controller → window
//     "configure:<json>"        { robot, robot_class, agent, suggestions[],
//                                 home, display_name?, tagline?, voice? }
//     "status:<state>"          connected|thinking|acting|idle|error|
//                               transcribing|speaking
//     "agent:<text>"            agent natural-language message to user
//     "transcript:<text>"       what STT heard from the operator's mic clip
//     "tool:<name>:<ok|err>:<summary>"   tool call audit line
//     "audio_out:<json>"        spoken reply {audio_b64, mime_type} to play
//     "usage:<json>"            per-turn token/credit telemetry footer
//     "system:<text>"           neutral status/info line
//     "error:<text>"            error to surface in red

import RobotWindow from '/resources/web/wwi/RobotWindow.js';

const window_ = window;
window_.robotWindow = new RobotWindow();

const els = {
  statusDot: document.getElementById('ol-status-dot'),
  statusText: document.getElementById('ol-status-text'),
  avatar: document.getElementById('ol-avatar'),
  avatarInitial: document.getElementById('ol-avatar-initial'),
  robotName: document.getElementById('ol-robot-name'),
  robotTagline: document.getElementById('ol-robot-tagline'),
  agentName: document.getElementById('ol-agent-name'),
  speaker: document.getElementById('ol-speaker'),
  popout: document.getElementById('ol-popout'),
  log: document.getElementById('ol-log'),
  suggestBar: document.getElementById('ol-suggest-bar'),
  composer: document.getElementById('ol-composer'),
  input: document.getElementById('ol-input'),
  mic: document.getElementById('ol-mic'),
  send: document.getElementById('ol-send'),
  stop: document.getElementById('ol-stop'),
};

// Map any status label the bridge sends to one of the five dot visuals.
const STATE_CLASS = {
  idle: 'idle', connected: 'connected', thinking: 'thinking',
  acting: 'acting', error: 'error',
  transcribing: 'thinking', listening: 'acting', speaking: 'acting',
};
const DOT_STATES = ['idle', 'connected', 'thinking', 'acting', 'error'];
// States where the agent is "working" — drives the avatar glow + typing bubble.
const THINKING_STATES = ['thinking', 'transcribing', 'acting'];

let muted = false;          // speaker toggle for audio_out playback
let typingEl = null;        // the live "…" typing bubble, if shown

function setStatus(state, label) {
  const cls = STATE_CLASS[state] || 'connected';
  DOT_STATES.forEach((s) => els.statusDot.classList.remove('ol-dot--' + s));
  els.statusDot.classList.add('ol-dot--' + cls);
  els.statusText.textContent = label || state;

  // Avatar reacts: glow while working, ripple while speaking.
  els.avatar.classList.remove('is-thinking', 'is-speaking');
  if (state === 'speaking') els.avatar.classList.add('is-speaking');
  else if (THINKING_STATES.indexOf(state) !== -1) els.avatar.classList.add('is-thinking');

  // Typing bubble appears while the agent is working, gone otherwise.
  if (THINKING_STATES.indexOf(state) !== -1) showTyping();
  else hideTyping();
}

function showTyping() {
  if (!typingEl) {
    typingEl = document.createElement('div');
    typingEl.className = 'ol-msg ol-msg--agent';
    const dots = document.createElement('span');
    dots.className = 'ol-typing';
    dots.innerHTML = '<span></span><span></span><span></span>';
    typingEl.appendChild(dots);
  }
  els.log.appendChild(typingEl);   // appendChild moves it to the bottom
  els.log.scrollTop = els.log.scrollHeight;
}

function hideTyping() {
  if (typingEl && typingEl.parentNode) typingEl.parentNode.removeChild(typingEl);
}

function append(kind, text, extraClass) {
  const div = document.createElement('div');
  div.classList.add('ol-msg');
  div.classList.add('ol-msg--' + kind);
  if (extraClass) div.classList.add(extraClass);
  div.textContent = text;
  els.log.appendChild(div);
  // Keep the typing indicator pinned to the bottom if it's showing.
  if (typingEl && typingEl.parentNode === els.log) els.log.appendChild(typingEl);
  els.log.scrollTop = els.log.scrollHeight;
}

function send(text) {
  window_.robotWindow.send(text);
}

// ── Composer ─────────────────────────────────────────────────────────
els.composer.addEventListener('submit', (ev) => {
  ev.preventDefault();
  const text = els.input.value.trim();
  if (!text) return;
  append('user', text);
  send('prompt:' + text);
  els.input.value = '';
  setStatus('thinking', 'thinking…');
});

els.input.addEventListener('keydown', (ev) => {
  if (ev.key === 'Enter' && !ev.shiftKey) {
    ev.preventDefault();
    els.composer.requestSubmit();
  }
});

els.stop.addEventListener('click', () => {
  if (recording) stopRec();
  append('system', '[stop] emergency halt requested');
  send('stop');
});

// ── Voice in: mic capture → audio_in ─────────────────────────────────
let mediaStream = null;
let recorder = null;
let chunks = [];
let recording = false;
let recTimer = null;

function hasMicSupport() {
  return !!(navigator.mediaDevices &&
            navigator.mediaDevices.getUserMedia &&
            window.MediaRecorder);
}

function pickMime() {
  const cands = [
    'audio/webm;codecs=opus', 'audio/webm',
    'audio/ogg;codecs=opus', 'audio/ogg', 'audio/mp4',
  ];
  for (const m of cands) {
    try { if (MediaRecorder.isTypeSupported(m)) return m; } catch (e) { /* skip */ }
  }
  return '';
}

async function startRec() {
  if (recording) return;
  try {
    mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
  } catch (e) {
    append('system', '🎙 mic unavailable: ' + ((e && e.message) || e));
    return;
  }
  const mime = pickMime();
  try {
    recorder = mime ? new MediaRecorder(mediaStream, { mimeType: mime })
                    : new MediaRecorder(mediaStream);
  } catch (e) {
    append('system', '🎙 recorder error: ' + ((e && e.message) || e));
    stopTracks();
    return;
  }
  chunks = [];
  recorder.ondataavailable = (ev) => {
    if (ev.data && ev.data.size > 0) chunks.push(ev.data);
  };
  recorder.onstop = () => finishRec(recorder.mimeType || mime || 'audio/webm');
  recorder.start();
  recording = true;
  els.mic.classList.add('ol-btn--mic-rec');
  els.mic.textContent = '● Recording — click to send';
  setStatus('listening', 'listening…');
  // Safety: auto-stop after 15 s so a forgotten session doesn't run forever.
  recTimer = setTimeout(() => { if (recording) stopRec(); }, 15000);
}

function stopRec() {
  if (!recording) return;
  recording = false;
  if (recTimer) { clearTimeout(recTimer); recTimer = null; }
  try { recorder.stop(); } catch (e) { /* ignore */ }
}

function stopTracks() {
  if (mediaStream) {
    mediaStream.getTracks().forEach((t) => t.stop());
    mediaStream = null;
  }
}

function finishRec(mime) {
  stopTracks();
  els.mic.classList.remove('ol-btn--mic-rec');
  els.mic.textContent = '🎙 Talk';
  const blob = new Blob(chunks, { type: mime });
  if (blob.size === 0) {
    append('system', '🎙 nothing recorded');
    setStatus('connected', 'idle');
    return;
  }
  const reader = new FileReader();
  reader.onloadend = () => {
    const b64 = String(reader.result || '').split(',')[1] || '';
    if (!b64) { append('system', '🎙 encode failed'); return; }
    setStatus('transcribing', 'transcribing…');
    send('audio_in:' + JSON.stringify({ audio_b64: b64, mime_type: mime }));
  };
  reader.readAsDataURL(blob);
}

els.mic.addEventListener('click', () => {
  if (recording) stopRec(); else startRec();
});

// ── Voice out: play audio_out clips ──────────────────────────────────
function playAudio(b64, mime) {
  if (muted || !b64) return;
  try {
    const audio = new Audio('data:' + (mime || 'audio/mpeg') + ';base64,' + b64);
    setStatus('speaking', 'speaking…');
    audio.onended = () => setStatus('connected', 'idle');
    audio.onerror = () => setStatus('connected', 'idle');
    const p = audio.play();
    if (p && p.catch) p.catch(() => setStatus('connected', 'idle'));
  } catch (e) { /* ignore playback errors */ }
}

els.popout.addEventListener('click', () => {
  // The controller opens the full-page chat in the system browser (one
  // tab per robot/agent). We just ask it to.
  send('open_chat');
});

els.speaker.addEventListener('click', () => {
  muted = !muted;
  els.speaker.textContent = muted ? '🔇' : '🔊';
  els.speaker.setAttribute('aria-pressed', String(muted));
  els.speaker.title = muted ? "Unmute the agent's voice" : "Mute the agent's voice";
});

// ── Receive from controller ──────────────────────────────────────────
function parseJSON(payload) {
  try { return JSON.parse(payload); } catch (e) { return null; }
}

function applyConfig(cfg) {
  if (!cfg) return;
  const name = cfg.display_name || cfg.robot || 'Robot';
  els.robotName.textContent = name;
  els.robotTagline.textContent = cfg.tagline || cfg.robot_class || '';
  els.avatarInitial.textContent = (name.trim()[0] || '·').toUpperCase();
  if (cfg.display_name) {
    document.title = name + ' — OmniLink';
    els.avatar.classList.add('ol-avatar--persona');
  }
  if (cfg.agent) els.agentName.textContent = cfg.agent;

  if (Array.isArray(cfg.suggestions) && cfg.suggestions.length > 0) {
    els.suggestBar.innerHTML = '';
    cfg.suggestions.forEach((s) => {
      const chip = document.createElement('button');
      chip.type = 'button';
      chip.className = 'ol-chip';
      chip.textContent = s;
      chip.addEventListener('click', () => {
        els.input.value = s;
        els.composer.requestSubmit();
      });
      els.suggestBar.appendChild(chip);
    });
    els.suggestBar.hidden = false;
  }

  // Voice controls only when the bridge advertises them (relay/OMNI_KEY).
  const voiceOn = !!cfg.voice && hasMicSupport();
  els.mic.hidden = !voiceOn;
  els.speaker.hidden = !voiceOn;

  setStatus('connected', 'connected');
}

function renderUsage(payload) {
  const o = parseJSON(payload);
  const text = (o && o.text) ? o.text : (o ? '' : payload);
  if (text) append('usage', '✦ ' + text);
}

window_.robotWindow.receive = function(message, _robot) {
  // Webots strips backslashes in transit; messages are colon-delimited and
  // we split on the FIRST colon so JSON payloads survive intact.
  const colon = message.indexOf(':');
  const tag = colon === -1 ? message : message.slice(0, colon);
  const payload = colon === -1 ? '' : message.slice(colon + 1);

  switch (tag) {
    case 'configure':
      applyConfig(parseJSON(payload));
      break;
    case 'status':
      setStatus(payload, payload);
      break;
    case 'agent':
      hideTyping();
      append('agent', payload);
      setStatus('connected', 'idle');
      break;
    case 'transcript':
      append('user', '🎙 ' + payload);
      break;
    case 'tool': {
      // payload = "<name>:<ok|err>:<summary>"
      const parts = payload.split(':');
      const name = parts.shift() || '?';
      const result = parts.shift() || 'ok';
      const summary = parts.join(':');
      const cls = result === 'ok' ? 'ol-msg--tool-ok' : 'ol-msg--tool-err';
      append('tool', '→ ' + name + ' · ' + summary, cls);
      if (result === 'ok') setStatus('acting', 'acting…');
      else setStatus('error', 'tool error');
      break;
    }
    case 'audio_out': {
      const o = parseJSON(payload);
      if (o) playAudio(o.audio_b64, o.mime_type);
      break;
    }
    case 'usage':
      renderUsage(payload);
      break;
    case 'system':
      append('system', payload);
      break;
    case 'error':
      hideTyping();
      append('error', payload);
      setStatus('error', 'error');
      break;
    default:
      append('system', message);
  }
};

// ── Boot ──────────────────────────────────────────────────────────────
window_.onload = function() {
  document.title = 'OmniLink robot console';
  setStatus('idle', 'connecting…');
  // The controller listens for "configure" and replies with "configure:{…}".
  send('configure');
};
