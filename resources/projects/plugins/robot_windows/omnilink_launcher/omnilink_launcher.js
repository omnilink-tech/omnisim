// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// OmniSim Launcher — side-panel demo gallery.
//
// Protocol with the supervisor controller (omnilink_launcher.py):
//   panel  -> controller
//     "ready"                     handshake; ask for the manifest
//     "load:<repo-rel-path>"      launch a demo
//
//   controller -> panel
//     "manifest:<json>"           demo catalogue (categories + demos)
//     "loading:<absolute-path>"   optimistic ack — Webots is loading
//     "status:<text>"             advisory status line
//     "error:<text>"              red error line

import RobotWindow from '/resources/web/wwi/RobotWindow.js';

const window_ = window;
window_.robotWindow = new RobotWindow();

const els = {
  statusDot:  document.getElementById('lr-status-dot'),
  statusText: document.getElementById('lr-status-text'),
  search:     document.getElementById('lr-search'),
  tabs:       document.getElementById('lr-tabs'),
  gallery:    document.getElementById('lr-gallery'),
};

const state = {
  manifest: null,
  activeCategory: '__all__',
  searchQuery: '',
};

// ── Status helpers ─────────────────────────────────────────────────
function setStatus(kind, label) {
  const known = ['idle', 'connected', 'loading', 'error'];
  known.forEach((k) => els.statusDot.classList.remove('lr-dot--' + k));
  els.statusDot.classList.add('lr-dot--' + kind);
  els.statusText.textContent = label;
}

// ── Send to controller ─────────────────────────────────────────────
function send(msg) {
  window_.robotWindow.send(msg);
}

// ── Render: tabs ────────────────────────────────────────────────────
function renderTabs() {
  if (!state.manifest) return;
  const cats = state.manifest.categories || [];
  const total = cats.reduce((sum, c) => sum + (c.demos?.length || 0), 0);

  els.tabs.innerHTML = '';
  const all = document.createElement('button');
  all.className = 'lr-tab' + (state.activeCategory === '__all__' ? ' is-active' : '');
  all.dataset.cat = '__all__';
  all.innerHTML = 'All <span class="lr-tab-count">' + total + '</span>';
  all.addEventListener('click', () => { state.activeCategory = '__all__'; renderTabs(); renderGallery(); });
  els.tabs.appendChild(all);

  for (const cat of cats) {
    const b = document.createElement('button');
    b.className = 'lr-tab' + (state.activeCategory === cat.id ? ' is-active' : '');
    b.dataset.cat = cat.id;
    b.innerHTML = escapeHtml(cat.label) + ' <span class="lr-tab-count">' + (cat.demos?.length || 0) + '</span>';
    b.addEventListener('click', () => { state.activeCategory = cat.id; renderTabs(); renderGallery(); });
    els.tabs.appendChild(b);
  }
}

// ── Render: gallery ─────────────────────────────────────────────────
function matchesSearch(demo, q) {
  if (!q) return true;
  const haystack = (demo.name + ' ' + (demo.blurb || '') + ' ' + (demo.world || '')).toLowerCase();
  return haystack.includes(q);
}

function renderGallery() {
  if (!state.manifest) {
    els.gallery.innerHTML = '<div class="lr-empty">Waiting for catalogue…</div>';
    return;
  }
  const cats = state.manifest.categories || [];
  const q = (state.searchQuery || '').trim().toLowerCase();
  const showAll = state.activeCategory === '__all__';
  els.gallery.innerHTML = '';

  let total = 0;
  for (const cat of cats) {
    if (!showAll && cat.id !== state.activeCategory) continue;
    const filtered = (cat.demos || []).filter((d) => matchesSearch(d, q));
    if (filtered.length === 0) continue;

    const section = document.createElement('section');
    section.className = 'lr-cat';

    if (showAll || cat.description) {
      const head = document.createElement('div');
      head.className = 'lr-cat-head';
      head.textContent = cat.label;
      section.appendChild(head);

      if (cat.description) {
        const blurb = document.createElement('div');
        blurb.className = 'lr-cat-blurb';
        blurb.textContent = cat.description;
        section.appendChild(blurb);
      }
    }

    for (const demo of filtered) {
      section.appendChild(renderCard(demo));
      total += 1;
    }
    els.gallery.appendChild(section);
  }

  if (total === 0) {
    const empty = document.createElement('div');
    empty.className = 'lr-empty';
    empty.textContent = q
      ? 'No demos match “' + (state.searchQuery) + '”.'
      : 'No demos in this category.';
    els.gallery.appendChild(empty);
  }
}

// ── Terminal-only demos ─────────────────────────────────────────────
// Some demos (the G1 / policy BATON demos) CANNOT be started by a bare
// Supervisor.worldLoad(). They need a deploy environment — the champion
// checkpoint, the harness gains, the ghost LUT, all exported as env vars by
// a shell script — before the world is opened. Loading their .wbt straight
// from this panel opens the scene with NO policy driving the robot: a G1
// standing there lifeless. That is worse than no button at all, so we must
// not offer one.
//
// The manifest already marks these demos: their blurb starts with
//   "RUN: <command>  — <description>"
// That prefix is the contract. Parse the command out of it, disable the
// launch button, and surface the command for copy/paste instead. Demos with
// no RUN: prefix are unaffected and still launch in-app as before.
function parseRunSpec(demo) {
  const m = /^RUN:\s*(.+)$/i.exec((demo.blurb || '').trim());
  if (!m) return null;
  const body = m[1];
  // demos.json separates the command from the prose with an em/en dash
  // surrounded by whitespace. Requiring whitespace on BOTH sides keeps
  // flags like `powershell -File ...` inside the command.
  const sep = body.search(/\s+[—–]\s+/);
  if (sep === -1)
    return { command: body.trim(), description: '' };
  return {
    command: body.slice(0, sep).trim(),
    description: body.slice(sep).replace(/^\s+[—–]\s+/, '').trim(),
  };
}

// Clipboard in the robot window runs in a Qt WebEngine page that is not
// always a "secure context", so navigator.clipboard can be missing. Fall
// back to execCommand, and finally to just selecting the text.
function copyText(text, node, btn) {
  const done = (ok) => {
    btn.textContent = ok ? 'Copied' : 'Press Ctrl+C';
    btn.classList.toggle('is-copied', ok);
    setTimeout(() => { btn.textContent = 'Copy'; btn.classList.remove('is-copied'); }, 1600);
  };
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).then(() => done(true), () => selectFallback(text, node, done));
    return;
  }
  selectFallback(text, node, done);
}

function selectFallback(text, node, done) {
  try {
    const ta = document.createElement('textarea');
    ta.value = text;
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.appendChild(ta);
    ta.select();
    const ok = document.execCommand('copy');
    document.body.removeChild(ta);
    if (ok) { done(true); return; }
  } catch (e) { /* fall through to selection */ }
  // Last resort: select the on-screen text so the user can hit Ctrl+C.
  try {
    const range = document.createRange();
    range.selectNodeContents(node);
    const sel = window.getSelection();
    sel.removeAllRanges();
    sel.addRange(range);
  } catch (e) { /* nothing else we can do */ }
  done(false);
}

function renderCard(demo) {
  const card = document.createElement('article');
  card.className = 'lr-card';

  const run = parseRunSpec(demo);
  if (run)
    card.classList.add('lr-card--terminal');

  const title = document.createElement('div');
  title.className = 'lr-card-title';
  title.textContent = demo.name;
  if (run) {
    const badge = document.createElement('span');
    badge.className = 'lr-badge';
    badge.textContent = 'TERMINAL';
    badge.title = 'Needs a deploy environment that a shell script sets — cannot be launched in-app.';
    title.appendChild(badge);
  }
  card.appendChild(title);

  // For a terminal-only demo show the prose only; the command gets its own
  // copyable row below, so we don't repeat the raw "RUN: ..." string here.
  const blurbText = run ? run.description : demo.blurb;
  if (blurbText) {
    const blurb = document.createElement('div');
    blurb.className = 'lr-card-blurb';
    blurb.textContent = blurbText;
    card.appendChild(blurb);
  }

  const foot = document.createElement('div');
  foot.className = 'lr-card-foot';

  const path = document.createElement('span');
  path.className = 'lr-card-path';
  path.textContent = demo.world;
  path.title = demo.world;
  foot.appendChild(path);

  const btn = document.createElement('button');
  btn.type = 'button';
  if (run) {
    // No launch path exists for this demo. Say so, and don't pretend.
    btn.className = 'lr-launch lr-launch--terminal';
    btn.textContent = 'Run from a terminal';
    btn.disabled = true;
    btn.title = 'This demo needs a deploy environment (policy checkpoint + harness gains) '
              + 'that only its launch script sets. Copy the command below and run it in a terminal.';
  } else {
    btn.className = 'lr-launch';
    btn.textContent = 'Launch';
    btn.addEventListener('click', () => {
      btn.disabled = true;
      btn.textContent = 'Loading…';
      setStatus('loading', 'loading ' + demo.name + '…');
      send('load:' + demo.world);
    });
  }
  foot.appendChild(btn);
  card.appendChild(foot);

  if (run) {
    const cmdRow = document.createElement('div');
    cmdRow.className = 'lr-cmd';

    const code = document.createElement('code');
    code.className = 'lr-cmd-text';
    code.textContent = run.command;
    code.title = run.command;
    cmdRow.appendChild(code);

    const copy = document.createElement('button');
    copy.className = 'lr-copy';
    copy.type = 'button';
    copy.textContent = 'Copy';
    copy.title = 'Copy the command to the clipboard';
    copy.addEventListener('click', () => copyText(run.command, code, copy));
    cmdRow.appendChild(copy);

    card.appendChild(cmdRow);
  }

  return card;
}

// ── Tiny HTML-escape (we mostly use textContent, but tabs build innerHTML)
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  }[c]));
}

// ── Search input ────────────────────────────────────────────────────
els.search.addEventListener('input', () => {
  state.searchQuery = els.search.value;
  renderGallery();
});

// ── Receive from controller ─────────────────────────────────────────
window_.robotWindow.receive = function(message, _robot) {
  const colon = message.indexOf(':');
  const tag = colon === -1 ? message : message.slice(0, colon);
  const payload = colon === -1 ? '' : message.slice(colon + 1);

  switch (tag) {
    case 'manifest':
      try {
        state.manifest = JSON.parse(payload);
        setStatus('connected', 'ready');
        renderTabs();
        renderGallery();
      } catch (e) {
        setStatus('error', 'manifest parse failed');
      }
      break;
    case 'loading':
      setStatus('loading', 'loading…');
      break;
    case 'status':
      setStatus('connected', payload || 'ready');
      break;
    case 'error':
      setStatus('error', payload);
      // Re-enable any disabled launch buttons so the user can retry.
      els.gallery.querySelectorAll('.lr-launch').forEach((b) => {
        b.disabled = false;
        b.textContent = 'Launch';
      });
      break;
    default:
      // Unknown tag — swallow silently.
      break;
  }
};

// ── Boot ──────────────────────────────────────────────────────────────
window_.onload = function() {
  document.title = 'OmniSim — Demo launcher';
  setStatus('idle', 'connecting…');
  send('ready');
};
