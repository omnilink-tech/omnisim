# Launch the "Talk to Ari" demo in the OmniSim GUI.
#
# Ari is an OmniArm 6 6-DOF arm you converse with as a warm, first-person
# persona. Right-click the robot in the 3D view -> Show Robot Window, then
# type or click the mic and speak. Ari chats, answers questions about
# herself, and moves the arm when you ask.
#
# Voice + persona route prompts through OmniLink to a real LLM, so they need
# an Omni Key (starts with "olink_"). Provide it one of two ways:
#   1. Set it first:   $env:OMNI_KEY = "olink_..."; .\run_omniarm6_talk.ps1
#   2. Pass it inline:  .\run_omniarm6_talk.ps1 olink_...
# Without a key the demo still runs, but as the terse offline command console
# (regex intent router) — no conversation, no voice.
#
# Env this launcher sets (override any of them before launching):
#   OMNILINK_ENGINE=g1-engine   route through Gemini — works on any Omni Key.
#                               Switch to g4-engine (Claude) for a richer
#                               persona IF your account has an Anthropic key
#                               added on the API & Keys page (else: 402
#                               BYOK_REQUIRED). g2-engine=GPT, g3-engine=Grok.
#   OMNILINK_VOICE_OUT=1        speak Ari's replies back (Chirp3-HD TTS)
#   OMNISIM_WARMUP_TOKEN        per-launch token; controller reloads once at
#                               startup to dodge the cold-first-load bug
#
# Usage:  powershell -File run_omniarm6_talk.ps1 [olink_KEY]
param([string]$OmniKey = "")

$root = (Resolve-Path "$PSScriptRoot\..\..\..\..\..").Path
$env:OMNISIM_HOME = $root

if ($OmniKey -ne "") { $env:OMNI_KEY = $OmniKey }

if ([string]::IsNullOrWhiteSpace($env:OMNI_KEY)) {
  Write-Host "[run_omniarm6_talk] No OMNI_KEY set -- launching the OFFLINE command console." -ForegroundColor Yellow
  Write-Host "[run_omniarm6_talk] For the conversational + voice Ari, set an Omni Key:" -ForegroundColor Yellow
  Write-Host '                $env:OMNI_KEY = "olink_..."; .\run_omniarm6_talk.ps1' -ForegroundColor Yellow
} else {
  Write-Host "[run_omniarm6_talk] OMNI_KEY detected -- conversational Ari with voice is ON." -ForegroundColor Green
  if ([string]::IsNullOrWhiteSpace($env:OMNILINK_ENGINE)) { $env:OMNILINK_ENGINE = "g1-engine" }
  if ([string]::IsNullOrWhiteSpace($env:OMNILINK_VOICE_OUT)) { $env:OMNILINK_VOICE_OUT = "1" }
  Write-Host "[run_omniarm6_talk] engine=$($env:OMNILINK_ENGINE)  voice_out=$($env:OMNILINK_VOICE_OUT)" -ForegroundColor Green
}

$env:OMNISIM_WARMUP_TOKEN = "gui$PID"
& "$root\msys64\mingw64\bin\omnisim-bin.exe" "$PSScriptRoot\omniarm6_talk.wbt"
