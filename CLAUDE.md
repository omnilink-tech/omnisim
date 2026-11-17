@AGENTS.md

<!--
This file exists so Claude Code loads the SAME project instructions that every
other coding agent reads from AGENTS.md. AGENTS.md is the single source of truth —
keep all guidance there, not here.

Why the one-line @import: Claude Code auto-loads CLAUDE.md but does NOT natively
read AGENTS.md; the `@AGENTS.md` directive above pulls it in without duplicating
content, and other tools (Codex, Cursor, Aider, …) keep reading AGENTS.md directly.
Cross-platform safe (no symlink). See https://code.claude.com/docs/en/memory
-->
