# Codebase Stats — What We Built on Top of Webots

> **One-liner for anyone who asks:** On top of the Webots base, we wrote
> **~275,000 lines of our own source code** — ~211k lines of Python (RL pipeline,
> OmniLink runtime, harness/capture/cinema, dev tooling) and ~56k lines of new
> C++ engine code (native URDF importer, Newton physics backend, wgpu renderer,
> wire protocol). We also net-added ~0.8M lines of robot/world/mesh **data** and
> deleted ~4.7M lines of Webots we didn't need — so the repo **as of the 2026-07-08
> snapshot** was **~3.1M lines leaner** than the raw import, while ~23M lines of total
> change have moved through it since the fork.
>
> ⚠ **The snapshot predates `bdc02139`** (2026-08, the ODE deletion: `src/ode/` +
> `include/ode/`, **106,283 lines**), so the "~3.1M leaner" figure is **low by ~106,283
> lines** and the deletion bucket is correspondingly low. No recomputed total is given
> here on purpose — regenerate it with the [Methodology](#methodology) commands rather
> than adding the delta by hand.

This doc exists so the "how many lines did we write?" question always has a
single, honest, reproducible answer. The headline numbers below are a **snapshot**;
the [Methodology](#methodology) section has the exact commands to regenerate them
at any later commit.

---

## Snapshot

| Field | Value |
|---|---|
| Date computed | 2026-07-08 |
| First commit (Webots import) | `0db6a18a` — *"Initial commit: OmniSim robotics simulator"* (2026-04-11) |
| HEAD at snapshot | `1799997f` (2026-07-08) |
| Commits since import | ~2,130 |
| Span | ~3 months |

The very first commit is a **full upstream Webots import** (13,013 files,
**5.77M lines**). That is the base we started from — *not* code we wrote.
Everything after it is our work.

---

## The three numbers people confuse

Asking "how many lines?" has three legitimate answers. Be clear which one you mean.

| Number | What it measures | Use when someone asks… |
|---:|---|---|
| **~14.4M** | Total lines git has *inserted* across all history (5.77M Webots import + 8.62M post-import insertions) | "How big is the git history?" |
| **~23.2M** | Total git line-changes = insertions + deletions across all history | "How much total churn?" |
| **~275k** | Actual **original source code we authored** on top of Webots (~211k Python + ~56k new C++ + ~8k scripts/shaders) | **"How much did we write?"** ← usually the real question |

The "millions of lines" figure that sometimes gets quoted is one of the **first
two** — they're dominated by the imported Webots tree plus world/mesh data and
churn, *not* by hand-written code. If you want the impressive-but-honest scale
number, "~23M lines of change since the fork" is defensible; if you want the
substance number, it's "~275k lines of original code."

---

## How much did we ADD to the original Webots code?

This is the **net difference** between the original import (`0db6a18a`) and HEAD,
broken down by file type. It splits into three very different buckets.

### Bucket 1 — Original source code we authored (~275k lines)

| Type | Lines | Notes |
|---|---:|---|
| Python (`.py`) | **~211,000** authored (834 new files); net **+197,637** | controllers, supervisors, OmniLink bridges, RL / Shadowing trainers, skill library, harness/capture/cinema, dev/build & social tooling — the heart of what we built |
| C/C++ (`.cpp` / `.hpp` / `.h` / `.c`) — **new engine files** | **~55,800** across 170 new files | native URDF importer, Newton physics backend, wgpu renderer, OmniSim wire protocol, capture service — see the C++ note below |
| Shaders / CUDA / scripts (`.frag`, `.vert`, `.cu`, `.cuh`, `.sh`, `.ps1`, `.bat`) | ~+8,200 net | |

**The C++ detail:** across all C/C++ we **added 57,434 and removed 101,822**
lines. Of the additions, ~55.8k are in **170 brand-new engine files** (our
backends, importer, protocol), plus light in-place edits to 123 existing Webots
files (+1,604 / −5,610). The remaining ~96k removed lines are **620 vendored /
unused Webots C++ files we deleted**. Net C/C++ is therefore −44k — but that
negative hides real authored work; quote the **~56k new-engine-C++** figure when
the C++ engine work matters. *(Caveat: with rename detection off, a small number
of heavily-relocated upstream files can register as new+deleted pairs, so treat
~56k as a generous upper bound on brand-new C++.)*

### Bucket 2 — Robot / world / mesh DATA (~0.8M lines net, not "code")

| Type | Net lines | What it is |
|---|---:|---|
| Meshes (`.stl`) | +568,000 | robot geometry |
| Meshes (`.dae`) | +270,000 | robot geometry |
| Config / robot defs (`.urdf`, `.yaml`, `.json`, `.xacro`, `.mjcf`) | ~+52,000 | |
| World files (`.wbt`) | **−84,000** | net *smaller* — we deleted more of Webots' bundled demo worlds than we added |

These are text in the repo but they're **data, not authored logic**. Note the
correction from earlier snapshots: worlds (`.wbt`) are now **net negative** — the
large generated worlds that once inflated this bucket are no longer committed
(they're regenerated on demand / gitignored).

### Bucket 3 — What we deleted (~4.7M lines)

We stripped out vendored libraries, translations, docs, and unused PROTO/assets
from Webots we didn't need. Overall net diff import→HEAD **as of the 2026-07-08 snapshot**:
**+1.58M added / −4.72M deleted**, so **the tree at that snapshot was ~3.14M lines *smaller*
than the raw Webots import.**

⚠ **This bucket is now low by ~106,283 lines.** `bdc02139` (2026-08) deleted the last big
vendored library — ODE (`src/ode/` + `include/ode/`) — which is not in the 2026-07-08 numbers
above. So both the −4.72M deletion figure and the ~3.14M net-smaller figure understate the
tree as it stands. Re-run the [Methodology](#methodology) commands at HEAD for a current
total; do not patch the numbers above in place.

---

## Methodology

All numbers come from `git`. `IMPORT` is the first commit (the Webots base):

```bash
IMPORT=0db6a18a   # "Initial commit: OmniSim robotics simulator"

# Size of the Webots base import (the thing we built ON, not OUR code)
git show --shortstat --format='' $IMPORT | tail -1
#   -> 13013 files changed, 5772070 insertions(+)

# Total insertions/deletions since import (post-import churn)
git log --no-merges --shortstat --format='' ${IMPORT}..HEAD \
  | awk '/files changed/{i+=$4; d+=$6} END{printf "insertions:%d deletions:%d\n", i, d}'
#   -> insertions:8617790 deletions:8783026
#   total inserted all history = 5.77M + 8.62M = ~14.4M
#   total churn all history    = 14.4M + 8.78M = ~23.2M

# NET change vs the original Webots tree, broken down by file type
#   (this is the "how much did we ADD" answer)
git diff --numstat $IMPORT HEAD 2>/dev/null \
  | awk 'NF==3 && $1!="-" {n=split($3,a,"."); ext=(n>1?a[n]:"none");
         ins[ext]+=$1; del[ext]+=$2}
         END{for(e in ins) printf "%-9d  +%-9d  -%-9d  %s\n",
             ins[e]-del[e], ins[e], del[e], e}' \
  | sort -nr | head -30

# Authored code, split by file status (A=new file, M=modified, D=deleted)
#   — this is what separates "we wrote it" from "we deleted upstream"
for F in A M D; do
  git diff --numstat --diff-filter=$F $IMPORT HEAD 2>/dev/null \
    | awk -v f=$F 'NF==3 && $1!="-"{n=split($3,a,"."); e=a[n];
        if(e=="py"||e=="cpp"||e=="hpp"||e=="h"||e=="c"){i+=$1;d+=$2;c++}}
        END{printf "%s: %d files  +%d / -%d\n", f, c, i, d}'
done
```

**Notes for re-running:**
- Where "churn" is mentioned these are **cumulative insertions** — a line
  edited N times counts N times. The per-extension **net diff** (import→HEAD) is
  the better measure of "what exists now that we added."
- Git skips exhaustive rename detection on a diff this large (harmless warning);
  renamed files may show as delete+add, slightly inflating both sides — this is
  why the C++ new-file count is a generous upper bound.
- To attribute *current-tree* lines line-by-line (most accurate but slow), use
  `git blame` per file and bucket by author/commit — not done here.

---

## TL;DR to quote

> *"Starting from the Webots engine, we've written about **275,000 lines of
> original source code** — ~211k of Python (our RL / Shadowing pipeline, OmniLink
> runtime, HTTP harness, capture & cinema, tooling) and ~56k of new C++ engine
> code (native URDF importer, Newton physics backend, wgpu renderer, wire
> protocol). We also net-removed ~3.1M lines of upstream Webots we didn't need,
> so OmniSim is leaner than the original import even as ~23M lines of change have
> moved through it since the fork three months ago."*
