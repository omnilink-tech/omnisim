# Security Policy

## Reporting a Vulnerability

If you believe you have found a security vulnerability in OmniSim, please **do
not file a public GitHub issue**. Disclose it privately so we can fix it
before it is exploited.

Two ways to report:

- **GitHub Security Advisories** (preferred). On the OmniSim repository, open
  the **Security** tab and click *Report a vulnerability*. This creates a
  private advisory that only the maintainers and the people you invite can
  see.
- **Email**: send the report to `security@omnilink-agents.com`. PGP is not
  required; if you want to encrypt, ask in your first message and we will
  share a key.

Include in your report:

1. A clear description of the vulnerability and its impact.
2. The OmniSim commit, tag, or release where you reproduced it.
3. The platform (Windows / Linux / macOS) and any relevant build flags.
4. Step-by-step reproduction, ideally with a minimal world file or controller
   that triggers the issue.
5. Any proof-of-concept code or logs.
6. Whether you intend to disclose publicly, and on what timeline.

## What to expect

- We will acknowledge receipt within **3 business days**.
- We aim to give an initial assessment (severity, applicability, fix path)
  within **10 business days**.
- For confirmed vulnerabilities, we will work with you on a fix and a
  coordinated disclosure timeline. The default window is **90 days** from
  acknowledgement until public disclosure, but we may shorten or extend that
  by mutual agreement based on severity, the availability of mitigations,
  and whether the issue is being actively exploited.
- We will credit reporters in the security advisory unless you ask to remain
  anonymous.

## Scope

In scope:

- The OmniSim simulator binary and bundled libraries (`src/omnisim/`,
  `src/wren/`, `src/controller/`).
- Net-new OmniSim subsystems — the agent-facing harness
  (`scripts/harness/`), the CUDA compute layer
  (`src/omnisim/compute/cuda/`), the procedural world library
  (`src/python/omniworld/`), the OmniLink-side agents
  (`agents/production/`), and the bridges they talk to under
  `projects/.../controllers/`.
- The release tooling under `scripts/release/`.

Out of scope:

- Third-party components vendored into the build (ODE, Assimp, GLM, stb, OIS,
  Pico, OpenVR, Wren, MSYS2/MinGW runtime, Qt). Report those issues to the
  upstream projects directly. See [`NOTICE`](NOTICE) for the list and links.
- Issues in upstream Webots that have not been touched in this fork. These
  may still be fixed in OmniSim, but the canonical disclosure path is to
  upstream Cyberbotics.
- Issues in the OmniLink platform itself
  (`https://www.omnilink-agents.com/api/...`). Those should be reported to
  OmniLink directly through that platform's security channel, not here.

## Hardening notes for operators

OmniSim is a desktop simulator that loads `.wbt` world files, executes
controller binaries, and (when the agent-facing harness is enabled) opens
local TCP ports for IPC. A few things worth knowing:

- World files (`.wbt`, `.proto`) are **not sandboxed** — they may reference
  external assets and trigger code paths that read from disk. Treat
  unfamiliar world files the same way you would treat untrusted source code.
- Controller processes spawned by OmniSim run with the same privileges as
  the simulator. Do not run controllers from untrusted sources without
  reading them first.
- The agent-facing harness binds to `127.0.0.1` by default and is intended
  for local agent loops. Do not expose its ports beyond the loopback
  interface unless you have a specific reason to.

## License

This security policy is licensed under the same Apache License, Version 2.0
as the rest of the OmniSim project. See [`LICENSE`](LICENSE) for terms.
