// OmniLink robot window — chat-style prompt console.
//
// Protocol with the controller (bridge):
//   window → controller
//     "configure"               handshake; ask bridge to send its config
//     "prompt:<text>"           operator prompt
//     "audio_in:<json>"         { audio_b64, mime_type } -- mic capture
//     "stop"                    emergency halt
//
//   controller → window
//     "configure:<json>"        { robot, robot_class, agent, suggestions[], home }
//     "status:<state>"          one of: connected|thinking|acting|idle|error|transcribing
//     "agent:<text>"            agent natural-language message to user
//     "tool:<name>:<ok|err>:<summary>"   tool call audit line
//     "usage:<json>"            per-turn { text, tokens_per_hour, credits, ... }
//     "transcript:<text>"       STT result -- what the mic heard
//     "audio_out:<json>"        { audio_b64, mime_type } -- TTS agent voice
//     "system:<text>"           neutral status/info line
//     "error:<text>"            error to surface in red

import RobotWindow from '/resources/web/wwi/RobotWindow.js';

const window_ = window;
window_.robotWindow = new RobotWindow();

const els = {
  statusDot: document.getElementById('ol-status-dot'),
  statusText: document.getElementById('ol-status-text'),
  robotName: document.getElementById('ol-robot-name'),
  robotClass: document.getElementById('ol-robot-class'),
  agentName: document.getElementById('ol-agent-name'),
  log: document.getElementById('ol-log'),
  emptyHint: document.getElementById('ol-empty-hint'),
  suggestions: document.getElementById('ol-suggestions'),
  composer: document.getElementById('ol-composer'),
  input: document.getElementById('ol-input'),
  send: document.getElementById('ol-send'),
  mic: document.getElementById('ol-mic'),
  stop: document.getElementById('ol-stop'),
};

// ── Mic capture (hold-to-record) ─────────────────────────────────────
// Uses MediaRecorder API. OmniSim robot windows are loopback HTTPS-less,
// so getUserMedia works without permission prompts on Chromium-based
// hosts (which OmniSim embeds via QtWebEngine).
let mediaRecorder = null;
let recordChunks = [];

async function startRecording() {
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const mime = MediaRecorder.isTypeSupported('audio/webm;codecs=opus') ? 'audio/webm;codecs=opus' : 'audio/webm';
    mediaRecorder = new MediaRecorder(stream, { mimeType: mime });
    recordChunks = [];
    mediaRecorder.addEventListener('dataavailable', (e) => {
      if (e.data && e.data.size > 0) recordChunks.push(e.data);
    });
    mediaRecorder.addEventListener('stop', async () => {
      const blob = new Blob(recordChunks, { type: mime });
      stream.getTracks().forEach((t) => t.stop());
      if (blob.size === 0) return;
      const b64 = await blobToBase64(blob);
      send('audio_in:' + JSON.stringify({ audio_b64: b64, mime_type: mime.split(';')[0] }));
    });
    mediaRecorder.start();
    els.mic.classList.add('is-recording');
    els.mic.textContent = '● Rec';
    setStatus('thinking', 'recording…');
  } catch (err) {
    append('error', 'mic unavailable: ' + (err && err.message ? err.message : err));
  }
}

function stopRecording() {
  if (mediaRecorder && mediaRecorder.state === 'recording') {
    mediaRecorder.stop();
  }
  els.mic.classList.remove('is-recording');
  els.mic.textContent = 'Mic';
}

function blobToBase64(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onloadend = () => {
      // result is data:audio/webm;base64,XXXX -- strip prefix.
      const s = String(reader.result || '');
      const i = s.indexOf(',');
      resolve(i >= 0 ? s.slice(i + 1) : s);
    };
    reader.onerror = reject;
    reader.readAsDataURL(blob);
  });
}

function playAudioB64(audioB64, mime) {
  if (!audioB64) return;
  try {
    const audio = new Audio('data:' + (mime || 'audio/mpeg') + ';base64,' + audioB64);
    audio.play().catch((e) => append('system', '[audio playback blocked: ' + e.message + ']'));
  } catch (e) {
    append('system', '[audio decode failed]');
  }
}

function setStatus(state, label) {
  const states = ['idle', 'connected', 'thinking', 'acting', 'error'];
  states.forEach((s) => els.statusDot.classList.remove('ol-dot--' + s));
  els.statusDot.classList.add('ol-dot--' + state);
  els.statusText.textContent = label || state;
}

function append(kind, text, extraClass) {
  // Hide the empty-hint block as soon as we have any message.
  if (els.emptyHint && !els.emptyHint.hidden) els.emptyHint.hidden = true;

  const div = document.createElement('div');
  div.classList.add('ol-msg');
  div.classList.add('ol-msg--' + kind);
  if (extraClass) div.classList.add(extraClass);
  div.textContent = text;
  els.log.appendChild(div);
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
  append('system', '[stop] emergency halt requested');
  send('stop');
});

// Hold-to-record: press to start, release to stop. Works for both
// mouse and touch interactions.
els.mic.addEventListener('mousedown', startRecording);
els.mic.addEventListener('mouseup', stopRecording);
els.mic.addEventListener('mouseleave', () => {
  if (els.mic.classList.contains('is-recording')) stopRecording();
});
els.mic.addEventListener('touchstart', (e) => { e.preventDefault(); startRecording(); }, { passive: false });
els.mic.addEventListener('touchend',   (e) => { e.preventDefault(); stopRecording();  }, { passive: false });

// ── Receive from controller ──────────────────────────────────────────
function parseConfig(payload) {
  try {
    return JSON.parse(payload);
  } catch (e) {
    return null;
  }
}

function applyConfig(cfg) {
  if (!cfg) return;
  if (cfg.robot)       els.robotName.textContent = cfg.robot;
  if (cfg.robot_class) els.robotClass.textContent = cfg.robot_class;
  if (cfg.agent)       els.agentName.textContent = cfg.agent;
  if (Array.isArray(cfg.suggestions) && cfg.suggestions.length > 0) {
    els.suggestions.innerHTML = '';
    cfg.suggestions.forEach((s) => {
      const li = document.createElement('li');
      li.textContent = '"' + s + '"';
      li.addEventListener('click', () => {
        els.input.value = s;
        els.input.focus();
      });
      els.suggestions.appendChild(li);
    });
  }
  setStatus('connected', 'connected');
}

window_.robotWindow.receive = function(message, _robot) {
  // OmniSim strips backslashes in transit; messages are colon-delimited.
  const colon = message.indexOf(':');
  const tag = colon === -1 ? message : message.slice(0, colon);
  const payload = colon === -1 ? '' : message.slice(colon + 1);

  switch (tag) {
    case 'configure':
      applyConfig(parseConfig(payload));
      break;
    case 'status':
      setStatus(payload, payload);
      break;
    case 'agent':
      append('agent', payload);
      setStatus('connected', 'idle');
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
    case 'transcript':
      // STT result -- show it like a user-typed prompt so the chat
      // log is self-contained even when the operator used the mic.
      append('user', '🎤 ' + payload);
      break;
    case 'audio_out': {
      // Synthesized agent voice. Decode and play.
      let info = null;
      try { info = JSON.parse(payload); } catch (e) { /* tolerate */ }
      if (info && info.audio_b64) {
        playAudioB64(info.audio_b64, info.mime_type);
      }
      break;
    }
    case 'usage': {
      // Per-turn token/credit usage delta. Compact rendering: prefer
      // the bridge's pre-built one-liner if present, otherwise format
      // from the structured fields.
      let info = null;
      try { info = JSON.parse(payload); } catch (e) { /* tolerate bad JSON */ }
      let line = info && info.text ? info.text : '';
      if (!line && info) {
        const toks = (info.total_units !== undefined) ? info.total_units : '?';
        const creds = (info.credits !== undefined) ? info.credits : '?';
        line = 'usage: ' + toks + ' tokens, ' + creds + ' credits';
      }
      if (line) append('usage', line, 'ol-msg--usage');
      break;
    }
    case 'system':
      append('system', payload);
      break;
    case 'error':
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
