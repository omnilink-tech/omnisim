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

"""Build BATON.pdf.

⛔ THE RESULTS SECTION IS DATA-DRIVEN BY CONSTRUCTION.
Every number in §5.2 is read from _scratch/baton_horizon/results.json (produced by
scripts/dev/baton_horizon_experiment.sh). Nothing is typed in by hand. If the file is
absent, the paper builds with the results section replaced by an explicit
"NOT YET RUN" block -- it does NOT ship a plausible-looking number.

That is not pedantry. The sibling Shadowing paper spent a day being corrected because
its headline attributed three deploy results to a method that did not produce them,
and because an earlier ablation was scored off a trajectory dump. A paper that cannot
print a number it does not have cannot make that mistake.

Usage:  python make_figs.py && python build_paper.py
"""
import json
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import LETTER
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
FIGS = os.path.join(HERE, "figs")
DATA = os.path.join(ROOT, "_scratch", "baton_horizon", "results.json")
OUT = os.path.join(HERE, "BATON.pdf")

ACCENT = colors.HexColor("#B85C00")
INK = colors.HexColor("#222222")
GREY = colors.HexColor("#666666")

ss = getSampleStyleSheet()
st_title = ParagraphStyle("t", parent=ss["Title"], fontName="Times-Bold", fontSize=19,
                          leading=23, textColor=INK, spaceAfter=2)
st_sub = ParagraphStyle("s", parent=ss["Normal"], fontName="Times-Italic", fontSize=11,
                        leading=14, alignment=TA_CENTER, textColor=GREY, spaceAfter=8)
st_auth = ParagraphStyle("a", parent=ss["Normal"], fontName="Times-Roman", fontSize=10.5,
                         alignment=TA_CENTER, textColor=INK)
st_affil = ParagraphStyle("af", parent=ss["Normal"], fontName="Times-Italic", fontSize=9,
                          alignment=TA_CENTER, textColor=GREY)
st_h1 = ParagraphStyle("h1", parent=ss["Heading1"], fontName="Times-Bold", fontSize=13,
                       leading=16, textColor=ACCENT, spaceBefore=12, spaceAfter=5)
st_h2 = ParagraphStyle("h2", parent=ss["Heading2"], fontName="Times-Bold", fontSize=11,
                       leading=14, textColor=INK, spaceBefore=8, spaceAfter=3)
st_body = ParagraphStyle("b", parent=ss["BodyText"], fontName="Times-Roman", fontSize=9.8,
                         leading=13.2, alignment=TA_JUSTIFY, textColor=INK, spaceAfter=5)
st_body0 = ParagraphStyle("b0", parent=st_body, firstLineIndent=0)
st_cap = ParagraphStyle("c", parent=ss["Normal"], fontName="Times-Italic", fontSize=8.2,
                        leading=10.5, alignment=TA_CENTER, textColor=GREY, spaceAfter=8)
st_cell = ParagraphStyle("tc", parent=ss["Normal"], fontName="Times-Roman", fontSize=8,
                         leading=9.8, textColor=INK)
st_cellb = ParagraphStyle("tcb", parent=st_cell, fontName="Times-Bold")

story = []
A = story.append


def P(t, s=st_body):
    return Paragraph(t, s)


def H1(n, t):
    return Paragraph(f"{n}&nbsp;&nbsp;{t}", st_h1)


def H2(n, t):
    return Paragraph(f"{n}&nbsp;&nbsp;{t}", st_h2)


def fig(name, cap, w=6.6):
    p = os.path.join(FIGS, name)
    if not os.path.exists(p):
        A(P(f"[figure missing: {name}]", st_cap))
        return
    from PIL import Image as PILImage
    iw, ih = PILImage.open(p).size
    W = w * inch
    A(Image(p, width=W, height=W * ih / iw))
    A(P(cap, st_cap))


# ── the data (or the honest absence of it) ──────────────────────────────────
RES = json.load(open(DATA)) if os.path.exists(DATA) else None


def res_summary():
    """-> (eng_rate0..n, naive_rate, eng_surv, naive_surv, n, seeds) or None."""
    if not RES:
        return None
    e = RES["arms"]["engineered"]
    v = RES["arms"]["naive"]
    n, s = RES["cycles"], RES["seeds"]
    es = sum(e["survival"]) / max(1, len(e["survival"]))
    vs = sum(v["survival"]) / max(1, len(v["survival"]))
    return e, v, es, vs, n, s


# ═══════════════════════════════════════════════════════════════════════════
# TITLE
# ═══════════════════════════════════════════════════════════════════════════
A(Paragraph("BATON: Runtime Handover Between Specialist Policies", st_title))
A(Paragraph("The switch is the engineering.", st_sub))
A(Paragraph("A. Fetouh", st_auth))
A(Paragraph("OmniSim Research", st_affil))
A(Paragraph("Technical Report &mdash; July 2026", st_affil))
A(Spacer(1, 8))

# ── Abstract ────────────────────────────────────────────────────────────────
A(Paragraph("<b>Abstract</b>", st_h2))
_r = res_summary()
if _r:
    e, v, es, vs, n, s = _r
    A(P(
        "The standard way to give one robot many skills is to train <i>one</i> policy on all of "
        "them and condition it on a command. That taxes every skill with every other skill's "
        "training distribution, and it cannot absorb a genuinely new skill without retraining. "
        "The alternative is obvious and mostly dismissed: keep <b>independently-trained "
        "specialists</b> and switch between them at runtime. It is dismissed because the switch "
        "is where it breaks. This report is about the switch. We describe <b>BATON</b>, an "
        "engineered handover between specialist policies for a humanoid: a morph blend over the "
        "command, a phase-gated entry so a cyclic specialist is only entered at a compatible gait "
        "phase, and&mdash;the part that actually matters&mdash;a <b>recurrent-state law</b>: a "
        "warm LSTM state carried out of a <i>stand</i> and into a <i>locomotion</i> policy locks "
        "the incoming policy in the stand attractor, and it marches in place. We make the "
        "handover <i>data</i>: each skill is one manifest binding its reference, its checkpoint "
        "and its deploy context, and a sequence is a list of them. "
        f"We then test whether the engineering earns its keep, on a closed-circuit task the robot "
        f"repeats {n} times (walk&rarr;turn&rarr;walk&rarr;turn, four corners, back to the start "
        f"pose), scoring each cycle as a whole: <b>the engineered handover survives "
        f"{es:.1f}/{n} cycles on average against {vs:.1f}/{n} for a naive one</b> "
        f"(warm state, no morph; {s} seeds per arm, identical specialists, identical physics). "
        "We are explicit about what this does <i>not</i> show: the third arm of the comparison, a "
        "fairly-trained monolith, is a training campaign we have not run, so <b>&lsquo;switching "
        "beats a monolith&rsquo; remains an open hypothesis</b> and we do not claim it. "
        "⚠️ All humanoid results run on a <b>weight-bearing balance harness</b> carrying up to "
        "twice the robot's weight; the footwork is the robot's own, the balance is not.", st_body0))
else:
    A(P(
        "The standard way to give one robot many skills is to train <i>one</i> policy on all of "
        "them and condition it on a command. The alternative&mdash;keep independently-trained "
        "<b>specialists</b> and switch between them&mdash;is usually dismissed, because the "
        "switch is where it breaks. This report is about the switch. We describe <b>BATON</b>, an "
        "engineered handover: a morph blend, a phase-gated entry, and a <b>recurrent-state law</b> "
        "(a warm LSTM state carried out of a stand into a locomotion policy locks the incoming "
        "policy in the stand attractor; it marches in place). We package the handover as data. "
        "<b>⛔ THE HORIZON EXPERIMENT HAS NOT BEEN RUN ON THIS CHECKOUT.</b> This build has no "
        "results section, by construction: the figures and numbers are read from a results file "
        "that does not exist, and this document will not print a number it does not have. Run "
        "<font face='Courier'>scripts/dev/baton_horizon_experiment.sh</font> and rebuild.",
        st_body0))
A(Spacer(1, 4))

# ═══════════════════════════════════════════════════════════════════════════
A(H1("1", "The switch, not the skills"))
A(P(
    "A humanoid that must walk, stop, turn, carry and climb is usually built as one "
    "velocity-conditioned policy: a single network trained on every skill, told which one to do "
    "by a command vector. It works, and it has two costs that grow with the skill set. Every "
    "skill is trained against the interference of every other skill, so the marginal skill makes "
    "the others slightly worse; and a genuinely new skill&mdash;one that does not live on the "
    "same continuum, like a carry or a get-up&mdash;cannot be added without retraining the whole "
    "thing.", st_body0))
A(P(
    "Keeping separate specialists removes both costs and creates exactly one new problem: at the "
    "moment of the switch, a policy that has never seen this state is handed a robot in it. The "
    "literature that reports long-horizon numbers reports them degrading&mdash;a recent "
    "large-humanoid-model result falls from 90% to 18% success over five cycles of a repeated "
    "task&mdash;and the hierarchical baselines it is compared against hand over with a naive "
    "finite-state machine: swap the network, keep going. Our claim is narrow and, we think, "
    "useful: <b>most of the difficulty of policy switching is in the handover, and the handover "
    "is engineerable.</b>", st_body0))

A(H1("2", "What breaks at a handover"))
A(P(
    "Four things, and only the first is obvious.", st_body0))
A(P(
    "<b>(i) The command discontinuity.</b> The outgoing policy was tracking a 0.45&nbsp;m/s gait; "
    "the incoming one wants 0. Stepping the command in one tick is a shove. We ramp it (a "
    "<i>morph</i> blend over N ticks).", st_body0))
A(P(
    "<b>(ii) The phase discontinuity.</b> A cyclic specialist is a function of gait phase. "
    "Entering it at an arbitrary phase asks it to continue a stride it never started. We gate "
    "entry on a compatible phase.", st_body0))
A(P(
    "<b>(iii) The recurrent state.</b> This is the one that cost us the most, and it is the "
    "reason we think handover is a research object and not a plumbing detail. Our specialists are "
    "LSTMs. The hidden state is not a detail of the implementation: it <i>is</i> the policy's "
    "belief about what it is doing. Hand a walker the hidden state of a policy that has just spent "
    "three seconds standing still, and the walker does not walk&mdash;it <b>marches in place</b>, "
    "locked in the stand attractor, because its own recurrent state is telling it that it is "
    "standing. The fix is not a bigger network. It is a <i>law</i>: at a "
    "<font face='Courier'>stand &rarr; locomotion</font> edge, the hidden state is reset "
    "(<b>cold</b>); at a <font face='Courier'>locomotion &rarr; locomotion</font> edge it is "
    "carried (<b>warm</b>), because there the belief is true and worth keeping. In our library "
    "this is derived from the skills' declared attractors, not configured per demo.", st_body0))
A(P(
    "<b>(iv) The context.</b> A specialist carries more than weights: a reference (its ghost), an "
    "observation layout, a corridor, a clock. Two specialists in the same observation family can "
    "be blended element-wise. One with a different observation width cannot be blended at all, "
    "and must instead have its whole deploy context <i>swapped</i> for the duration of its "
    "segment. Both mechanisms exist in our runtime; the manifest decides which applies.", st_body0))

fig("fig_arch.png", "Figure 1: BATON. The specialists are trained independently against their own "
    "references; the engineering is in the switch between them.")

A(H1("3", "The mechanism"))
A(P(
    "A <b>skill</b> is a manifest: a reference (a phase-indexed lookup table), a policy "
    "checkpoint, an observation family, a deploy context, a BATON <i>blend</i> "
    "(<font face='Courier'>cyclic</font> or <font face='Courier'>solo_swap</font>), and an "
    "<i>attractor</i> (<font face='Courier'>stand</font> or "
    "<font face='Courier'>locomotion</font>). A <b>sequence</b> is a list of skills plus an "
    "arbiter (a schedule, or a goal-directed course of waypoints). The handover law is "
    "<i>derived</i> from the manifests&mdash;the cold/warm decision at each edge falls out of the "
    "attractors, and is not a knob a demo author sets by hand. Sequences are checked against the "
    "hand-written demo scripts they replace, key-for-key on the assembled launch environment.",
    st_body0))
A(P(
    "The specialists themselves are produced by <i>Shadowing</i> (companion report): each tracks "
    "a reference that was certified feasible before training. That matters here only in that it "
    "makes the specialists comparable&mdash;they share a training recipe, an observation family "
    "and a corridor, so a handover between them is a fair test of the handover rather than a test "
    "of two unrelated networks.", st_body0))

A(H1("4", "The turn, and why a corner is not a segment"))
A(P(
    "The hardest handover we ship is the 90&deg; corner. A turn specialist trained on a step-turn "
    "reference banks only ~60&ndash;65% of its reference yaw when replayed once on the real robot: "
    "the feet track the reference but the base under-rotates (foot slip is per-step, not "
    "per-second, so a slower clock cannot add rotation&mdash;more <i>steps</i> can). The "
    "reference is a modular staircase of mini-pivots, each beginning and ending feet-together, so "
    "we replay <i>partial passes</i>, restarting at the plateau whose remaining staircase matches "
    "the remaining angle divided by the measured gain, until the <b>actual accumulated heading</b> "
    "reaches the target. Never stopping mid-reference is a hard rule: the robot lags the reference "
    "by one to two mini-cycles, so a reference plateau says nothing about the robot's stance, and "
    "both mid-reference arrests we measured recoiled or spun.", st_body0))

A(PageBreak())
A(H1("5", "Experiments"))
A(H2("5.1", "The shipped sequences"))
A(P(
    "Four sequences ship as manifests and reproduce their hand-written demos exactly: a box "
    "delivery (walk&rarr;stand&rarr;pick&rarr;carry&rarr;90&deg; corner&rarr;place&rarr;walk), its "
    "corner-free variant, a walk&rarr;turn&rarr;walk, and a solo turn. The 90&deg; corner lands at "
    "90.6&nbsp;/&nbsp;91.5&nbsp;/&nbsp;95.6&deg; actual heading, 3/3, zero falls, with <i>zero "
    "crane yaw torque</i>&mdash;the rotation is the robot's own footwork.", st_body0))

A(H2("5.2", "Success versus horizon"))
if _r:
    e, v, es, vs, n, s = _r
    A(P(
        "A working handover is not evidence that handover is the better architecture. To test the "
        "engineering we need a task where handover error can <i>accumulate</i>, so we built one: a "
        "closed square circuit&mdash;walk a side, stand, turn 90&deg;, stand, four times&mdash;which "
        f"returns the robot to its starting pose, repeated up to {n} times. A cycle counts as a "
        "success only if every one of its eight segments completed and the pelvis never dropped "
        "below 0.45&nbsp;m; a fall is terminal. The circuit <i>must</i> close: an earlier version "
        "with two corners left the robot facing the wrong way, so cycle 1 opened by demanding a "
        "walk 90&deg; off its heading and the robot fell for a reason that had nothing to do with "
        "the handover under test.", st_body0))
    A(P(
        "Two arms, identical in every respect except the switch: the <b>engineered</b> handover as "
        "shipped, and a <b>naive</b> one (the hidden state carried warm across every edge, no morph "
        f"blend). {s} seeds per arm (the initial leg pose is perturbed, as in training; the deploy "
        "is otherwise deterministic and a single run would give a survival horizon, not a rate).",
        st_body0))
    rows = [["cycle", *[str(k) for k in range(n)], "mean survival"]]
    rows.append(["engineered", *[f"{r:.2f}" for r in e["success_rate"]], f"{es:.1f} / {n}"])
    rows.append(["naive", *[f"{r:.2f}" for r in v["success_rate"]], f"{vs:.1f} / {n}"])
    t = Table(rows, hAlign="CENTER")
    t.setStyle(TableStyle([
        ("FONT", (0, 0), (-1, 0), "Times-Bold", 8),
        ("FONT", (0, 1), (-1, -1), "Times-Roman", 8),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
        ("GRID", (0, 0), (-1, -1), 0.4, GREY),
        ("ALIGN", (1, 0), (-1, -1), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
    ]))
    A(t)
    A(P("Table 1: fraction of seeds still alive at cycle k. A run that falls is dead: survival is "
        "cumulative, not an average over independent cycles.", st_cap))
    fig("fig_horizon.png", "Figure 2: success rate versus horizon. Identical specialists, identical "
        "physics, identical course; the only difference is the handover.")
    delta = es - vs
    if delta > 0.5:
        A(P(
            f"<b>The engineering earns its keep.</b> The engineered handover survives "
            f"{es:.1f} of {n} cycles on average; the naive one {vs:.1f}. Both arms use the same "
            "specialists&mdash;the same weights, the same references&mdash;so the entire difference "
            "is the switch.", st_body0))
    elif abs(delta) <= 0.5:
        A(P(
            f"<b>An honest null.</b> On this task the two handovers are within noise of each other "
            f"({es:.1f} vs {vs:.1f} cycles of {n}). We report it because we designed the experiment "
            "to be able to say so. It does not refute the mechanism&mdash;the recurrent-state law "
            "was derived from a failure we can reproduce on demand&mdash;but it does mean this "
            "circuit is not where the naive handover is punished, and a stronger task (more "
            "cycles, or a lower harness authority) is the next thing to run.", st_body0))
    else:
        A(P(
            f"<b>Against our own hypothesis.</b> The naive handover survived <i>longer</i> "
            f"({vs:.1f} vs {es:.1f} of {n}). We report it as measured.", st_body0))
else:
    A(P(
        "<b>⛔ NOT YET RUN ON THIS CHECKOUT.</b> This section is generated from "
        "<font face='Courier'>_scratch/baton_horizon/results.json</font>, which does not exist "
        "here. The build refuses to print a horizon curve, a table or a claim it does not have. "
        "Run <font face='Courier'>scripts/dev/baton_horizon_experiment.sh 6 5 900</font> and "
        "rebuild.", st_body0))

A(H1("6", "What we do not claim"))
A(P(
    "<b>We have not beaten a monolith.</b> The third arm of the comparison&mdash;a single policy "
    "trained honestly on the same skill set with a real budget&mdash;is a training campaign, not a "
    "deploy sweep, and we have not run it. Everything above compares an engineered handover to a "
    "naive one; it says the switch is worth engineering, and it says nothing about whether "
    "specialists-plus-switching beats one big conditioned network. The literature's own "
    "hierarchical baselines are naive-FSM, which is precisely the arm we beat, so it would be easy "
    "and dishonest to slide from one claim to the other. <b>&lsquo;Switching beats a monolith&rsquo; "
    "remains an open hypothesis</b>, and the experiment that would settle it is specified in our "
    "notes.", st_body0))
A(P(
    "<b>The humanoid is on a harness.</b> Every result here runs on a weight-bearing balance "
    "crane that carries up to about twice the robot's weight and holds its attitude. The legs step "
    "and the corner is genuine footwork (zero crane yaw torque during the turn), but the robot is "
    "being <i>carried</i>. A durable free-standing humanoid walk remains an open problem in this "
    "system, and no number in this report should be read as one.", st_body0))
A(P(
    "<b>Blending requires a shared observation family.</b> Cyclic peers are blended "
    "element-wise, which requires identical observation widths; a specialist that does not share "
    "the family can only be context-swapped for its whole segment. Cross-<i>robot</i> handover is "
    "not a goal and is not supported.", st_body0))

A(H1("7", "Conclusion"))
A(P(
    "Policy switching is usually treated as plumbing between the interesting parts. We think it is "
    "the interesting part. A handover has a command discontinuity, a phase discontinuity, a "
    "context, and&mdash;if the specialists are recurrent&mdash;a <i>belief</i> that must be "
    "invalidated at exactly the right edges. Getting those right is what separates a chain of "
    "specialists that survives a long task from one that marches in place at the first corner. We "
    "have made the handover explicit, made it data, and measured it against the naive alternative. "
    "The larger question&mdash;whether this architecture beats the monolith it is meant to "
    "replace&mdash;is still open, and we would rather say so than imply otherwise.", st_body0))

doc = SimpleDocTemplate(OUT, pagesize=LETTER,
                        leftMargin=0.85 * inch, rightMargin=0.85 * inch,
                        topMargin=0.7 * inch, bottomMargin=0.7 * inch,
                        title="BATON: Runtime Handover Between Specialist Policies",
                        author="A. Fetouh")
doc.build(story)
print("BUILT", OUT, "(with results)" if RES else "(NO RESULTS -- experiment not run)")
