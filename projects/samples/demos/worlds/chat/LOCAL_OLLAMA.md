# Running the chat demos on a local LLM (Ollama) — zero accounts, zero cost

Every chat demo in this folder (`omnilink_ur5e.omniworld`, `omnilink_husky.omniworld`, …)
can run against a **local Ollama server** instead of the OmniLink platform.
No OmniLink account, no cloud key, no per-token cost — the model runs on
your own GPU. This is the free tier for OmniSim's talk-to-the-robot demos.

How it works: when the demo world launches and `OMNI_KEY` is **not** set,
the robot's bridge probes `http://127.0.0.1:11434`. If an Ollama server
answers, the chat panel routes through it with native tool calling and the
panel label reads `local Ollama (<model>)`. If not, the demo still works in
its offline regex mode (literal commands only). Setting `OMNI_KEY` always
takes priority and routes through the OmniLink platform instead.

## Quick start

1. **Install Ollama** — download from [ollama.com](https://ollama.com)
   (Windows / macOS / Linux) and let it run in the background.
2. **Pull a model** (one time):

   ```bash
   # Recommended — best command-following we measured (see table below):
   ollama pull qwen2.5:7b

   # Small/fast floor for 4-6 GB GPUs or CPU-only machines:
   ollama pull qwen2.5:3b
   ```

3. **Launch any chat demo** with no `OMNI_KEY` in the environment:

   ```bat
   launch.bat projects\samples\demos\worlds\chat\omnilink_ur5e.omniworld
   ```

4. Right-click the robot → *Show Robot Window* → type
   `wave hello`, `move the end effector to 0.4, 0.2, 0.5`, `go home`.

To use the recommended model when it isn't the default:

```bat
set OLLAMA_MODEL=qwen2.5:7b
launch.bat projects\samples\demos\worlds\chat\omnilink_ur5e.omniworld
```

## Which model should I use?

We benchmarked the UR5e arm demo end-to-end (prompt → LLM → tool call →
simulated physics, state verified after every command) on an RTX 3060
laptop (6 GB VRAM). Ten-prompt suite, from easy ("wave hello") to hard
(multi-waypoint, relative motion, unreachable targets, session memory):

| Model | Where it ran | Warm latency / turn | Behavior | Verdict |
|---|---|---|---|---|
| **qwen2.5:14b** | RTX 4090 (remote pod, SSH tunnel) | **1.2–3 s** | Perfect run: 10/10 hard prompts. The only model that chained a full square (5 IK waypoints in one turn), read state before relative motions, gave exact joint readouts from a real tool call, and summarized the session accurately. | ⭐⭐ **Best overall** — use it if you have ≥12 GB VRAM or a remote GPU. |
| qwen2.5:32b | RTX 4090 (remote pod, SSH tunnel) | 6–12 s (27 s first) | Zero failures, zero hallucinations; the most careful workspace reasoning (correctly refused a genuinely unreachable "+20 cm from home"). More conservative + 3–5× slower than 14b with no accuracy win on this suite. | Fine, but 14b beats it on speed at equal correctness. |
| **qwen2.5:7b** | RTX 3060 6 GB (local) | 2–6 s | Correct refusals, real `reset_to_home` + accurate summary, improvises expressive commands ("nod" → wave). Won't chain 4-waypoint shapes in one turn. | ⭐ **Best local pick** for small GPUs. |
| qwen2.5:3b (default) | RTX 3060 6 GB (local) | 1–3 s | Fine on direct commands (wave / absolute moves / joint targets / home). On hard prompts it guesses instead of reading state, and once **claimed "I'm home" without calling any tool** — a trust problem. | Floor for small GPUs. Simple commands only. |
| qwen3:4b | RTX 3060 6 GB (local) | 20 s+ … timeout | Thinking mode reasons past the bridge's 90 s turn budget (every prompt died empty); with thinking disabled it leaks its inner monologue into the reply text. | Avoid for this use case. |

Rules of thumb:

- **≥ 12 GB VRAM** → `qwen2.5:14b` — the clean sweep in our tests.
- **≥ 8 GB VRAM** → `qwen2.5:7b` runs fully on-GPU and is both fast and reliable.
- **4–6 GB VRAM** → `qwen2.5:7b` partially offloads to CPU (still usable,
  2–6 s/turn measured); `qwen2.5:3b` if you want snappier turns and only
  need direct commands.
- **Thinking/reasoning models** (qwen3, deepseek-r1, gpt-oss) are a poor
  fit for chat-to-robot latency. The bridge disables their thinking mode
  automatically; re-enable with `OMNISIM_OLLAMA_THINK=1` if you must.

## Remote Ollama: run the model on another machine or a cloud GPU

`OLLAMA_BASE_URL` can point anywhere, so a weak laptop can drive the demo
while the model runs on a desktop, a home server, or a rented pod. The
simplest secure transport is an SSH tunnel (Ollama has no auth of its own —
don't expose port 11434 to the internet):

```bash
# On the GPU box (or cloud pod): install Ollama, pull a model, leave it serving.
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:14b

# On the OmniSim machine: tunnel a local port to the remote Ollama...
ssh -N -L 11435:127.0.0.1:11434 user@gpu-box &

# ...and launch any chat demo against it:
set OLLAMA_BASE_URL=http://127.0.0.1:11435
set OLLAMA_MODEL=qwen2.5:14b
launch.bat projects\samples\demos\worlds\chat\omnilink_ur5e.omniworld
```

Measured (RunPod RTX 4090 in EU, OmniSim on a laptop, ~35 min session ≈
$0.40 at $0.69/hr): the tunnel's round-trip is negligible next to
generation — qwen2.5:14b turned in the fastest *and* most capable run of
every model tested (table above). A cheap rented GPU by the hour is a
realistic way to demo the "big model" experience without owning one.

## What works and what doesn't (measured, UR5e arm + qwen2.5:7b)

Verified against actual sim state, not the model's claims:

| Prompt | Result |
|---|---|
| "wave hello" | ✅ wave executed |
| "move the end effector to position 0.4, 0.2, 0.5" | ✅ TCP measured at the target (±2 cm IK settle) |
| "move to 0.4, 0, 0.4 and then raise it 20 cm higher" | ✅ two chained IK calls, model did the arithmetic |
| "move joint 2 to -0.9 and joint 5 to 0.6 at the same time" | ✅ one multi-joint call |
| "move the end effector to 2.0, 0.0, 3.0" (unreachable) | ✅ graceful refusal, no motion |
| "nod as if saying yes" | ✅ improvised with the closest tool (wave) |
| "go home and summarize the session" | ✅ real reset + accurate summary |
| "what are your current joint angles?" | ✅ answered correctly |
| "draw a square with 15 cm sides" | ⚠️ does one side per turn instead of chaining the loop |
| "raise the arm 20 centimeters" (standalone relative) | ⚠️ inconsistent — sometimes reads state first, sometimes guesses a target |

## Give your robot a memory: hybrid mode (free OmniLink account)

Naked local mode forgets everything on every world reload. Set `OMNI_KEY`
**while Ollama is running** and the bridge switches to **hybrid** mode —
inference stays on your GPU (free, same speed), and OmniLink adds the
layer around it:

- **Memory across sessions** — reload the world, say "do that again but
  slower", and the robot knows what "that" was (restored from platform
  short-term memory).
- **Dashboard presence** — the robot appears as an agent at
  omnilink-agents.com with its tool surface.
- **Cloud fallback** — if your local Ollama dies mid-session, the turn
  runs through your cloud engine instead of erroring.
- **Voice + usage telemetry** — STT/TTS and the usage page work.

```bat
set OMNI_KEY=olink_...
launch.bat projects\samples\demos\worlds\chat\omnilink_ur5e.omniworld
:: panel shows: local Ollama (<model>) — bridge log: "HYBRID relay ON"
```

An explicitly set `OMNILINK_ENGINE` overrides this (that means you chose
cloud inference on purpose). To also drive this robot from your phone or
the web UI while inference stays on your GPU, run the edge connector:
`pip install "omnilink[bridges]"` then `python -m omnilink.edge_connector`.

## Environment knobs

| Var | Default | Meaning |
|---|---|---|
| `OLLAMA_MODEL` | `qwen2.5:3b` | Model the bridge requests |
| `OLLAMA_BASE_URL` | `http://127.0.0.1:11434` | Ollama server address |
| `OMNISIM_OLLAMA` | `1` | Set `0` to force the offline regex router |
| `OMNISIM_OLLAMA_TIMEOUT` | `180` | Per-LLM-call timeout, seconds |
| `OMNISIM_OLLAMA_THINK` | `0` | Re-enable thinking mode on reasoning models |
| `OLLAMA_KEEP_ALIVE` | `5m` (Ollama's) | Raise (e.g. `30m`) so the model stays in VRAM between prompts |

## Troubleshooting

- **Panel says `local intent (regex)`** — the bridge didn't find Ollama at
  launch. Check `curl http://127.0.0.1:11434/api/version`, then reload the
  world (the probe runs once at controller start). A *just-started* Ollama
  can take a couple of seconds to answer; the bridge waits up to 2 s.
- **First reply takes tens of seconds** — that's the model loading into
  VRAM (one-time per idle period). Set `OLLAMA_KEEP_ALIVE=30m` when
  starting Ollama, or send a throwaway "hi" first.
- **`ollama HTTP 404: model not found`** — you set `OLLAMA_MODEL` to a
  model you haven't pulled. `ollama pull <name>` first.
- **Slow every turn** — the model doesn't fit your VRAM and is spilling to
  CPU. Drop to `qwen2.5:3b`, or check `ollama ps` for the CPU/GPU split.
