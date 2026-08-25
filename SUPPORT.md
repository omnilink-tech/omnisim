# Getting help with OmniSim

OmniSim is maintained by [OmniLink](https://www.omnilink-agents.com). This page says where to ask, what to expect, and how quickly.

---

## Start here: ask your coding agent

This is not a deflection — it is genuinely the fastest path, and it is what the project is built around.

[`AGENTS.md`](AGENTS.md) at the repository root is a long, deliberately blunt operating manual written for AI coding agents (Claude Code, Codex, Cursor, and anything else that reads the [AGENTS.md standard](https://agents.md/)). It carries the build commands, the demo catalogue, the HTTP harness API, and — most usefully — a large collection of hard-won traps that are not guessable from the code: which fields are silently ignored, which devices report values they are not measuring, which defaults quietly do nothing.

So if you have cloned the repo, open it in your editor with an agent attached and ask. A great many questions are answered there in more detail than an issue thread would give you.

Run this first, whatever your question:

```bash
python -m omnisim doctor
```

It reports the truth about your clone right now — binary path, engine/libController compatibility, port status, worlds present. A surprising share of "it doesn't work" turns out to be something `doctor` names in one line.

---

## Reporting a bug

**[Open an issue](https://github.com/omnilink-tech/omnisim/issues/new/choose)** using the bug report template.

What makes a report fast to act on, in rough order of value:

- **The output of `python -m omnisim doctor`.** It identifies your machine, your binary and your runtime versions, which is otherwise the first round-trip.
- **A world file that reproduces it**, or the exact command you ran.
- **What you measured, and what you expected.** A number beats a description. "The box rests at z=0.4396 and I expected 0.18" is immediately actionable; "the physics looks wrong" needs a conversation first.
- **Your machine.** Results here vary meaningfully across GPUs and operating systems, and we never average across machines — so which box you are on is part of the bug.

⚠️ **A note that will save you time: a headless `PASS` does not mean the physics is correct.** It means the world loaded, stepped, and logged nothing bad. A body falling forever through a missing floor passes identically to a working world. If you are reporting a physics problem, add `--fail-on-runaway`, or better, give us a measured value.

## Requesting a feature, or asking a question

Open an issue and say what you are trying to do rather than what you think the fix is — the answer is often a capability that already exists under a name you would not have searched for.

**GitHub Discussions is not enabled yet.** It is being turned on; until then, questions are welcome as issues and will not be closed for being questions.

## Asking us to build a simulation for you

We will try to build a robot or scenario you describe, free, in public, and the result ships in this repository as a world anyone can run.

Use the **[Request a Sim](https://github.com/omnilink-tech/omnisim/issues/new?template=request_a_sim.yml)** template. It screens up front for the handful of things the engine genuinely cannot simulate today — closed kinematic loops, tracked propulsion, concave colliders, and a few others — because those are not effort problems and we would rather tell you in a day than in a month.

## Security

**Do not open a public issue for a security problem.** See [SECURITY.md](SECURITY.md) for the disclosure process and response times.

Worth knowing generally: `.omniworld` / `.wbt` world files and controllers are **not sandboxed**. They can execute code. Treat a world file from an untrusted source exactly as you would treat a script from an untrusted source.

## Questions about the OmniLink platform

OmniSim runs fully offline and needs no account — the demos fall back to a local model and then to an offline intent router. Questions about the platform itself (keys, billing, the agent runtime) belong at [omnilink-agents.com](https://www.omnilink-agents.com), not in this repository's issue tracker.

## Upstream Webots questions

OmniSim forked from [Webots](https://github.com/cyberbotics/webots) and inherits its world-file syntax, much of its base node set, and the controller API shape. If your question is about behaviour that is genuinely upstream's, the [Webots tag on Robotics Stack Exchange](https://robotics.stackexchange.com/questions/tagged/webots) will serve you better and faster. Those channels belong to the Webots project, not to us.

---

## What to expect

This project is maintained by a very small team. Please read this as a description rather than a service-level agreement:

- **Security reports** get priority — see [SECURITY.md](SECURITY.md) for the actual commitments.
- **Bug reports with a reproducer** are the ones that get fixed. A reproducer moves a report from a conversation to a task.
- **Everything else** is best-effort. If an issue goes quiet, a polite bump is welcome and not annoying.

If you are relying on OmniSim for something that matters and need more than best-effort, say so in the issue — commercial support is not currently offered, but knowing who depends on what genuinely changes what gets prioritised.

## Contributing a fix yourself

[CONTRIBUTING.md](CONTRIBUTING.md) has the process, the licence gates, and the policy on using AI tools — which is short, and is a bar rather than a ban.
