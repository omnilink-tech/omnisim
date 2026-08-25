# -*- coding: utf-8 -*-
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

"""Build the Shadowing research paper PDF (reportlab, single-column journal style)."""
import os
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_JUSTIFY, TA_CENTER, TA_LEFT
from reportlab.platypus import (BaseDocTemplate, PageTemplate, Frame, Paragraph,
    Spacer, Image, Table, TableStyle, KeepTogether, HRFlowable, Flowable)
from reportlab.lib.utils import ImageReader

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, "figs")
OUT = os.path.join(HERE, "Shadowing.pdf")

# ---- palette ----
INK = colors.HexColor("#1a1a1a")
ACCENT = colors.HexColor("#0B3D6B")     # deep blue
ACCENT2 = colors.HexColor("#0072B2")
RULE = colors.HexColor("#9fb3c8")
HEADER_BG = colors.HexColor("#0B3D6B")
BAND = colors.HexColor("#eef3f8")
BAND2 = colors.HexColor("#f7f9fc")

PAGE_W, PAGE_H = letter
LM = RM = 0.85 * inch
TM = 0.85 * inch
BM = 0.85 * inch
CW = PAGE_W - LM - RM   # content width

# ---------------------------------------------------------------- styles
ss = getSampleStyleSheet()
def S(name, **kw):
    base = kw.pop("parent", ss["Normal"])
    return ParagraphStyle(name, parent=base, **kw)

st_title = S("t", fontName="Times-Bold", fontSize=18.5, leading=22,
             alignment=TA_CENTER, textColor=ACCENT, spaceAfter=2)
st_sub = S("sub", fontName="Times-Italic", fontSize=11.5, leading=14,
           alignment=TA_CENTER, textColor=INK, spaceAfter=8)
st_auth = S("a", fontName="Times-Roman", fontSize=11, leading=14,
            alignment=TA_CENTER, textColor=INK, spaceAfter=1)
st_affil = S("af", fontName="Times-Italic", fontSize=9.5, leading=12,
             alignment=TA_CENTER, textColor=colors.HexColor("#444444"), spaceAfter=2)
st_abhead = S("abh", fontName="Times-Bold", fontSize=10, leading=12,
              alignment=TA_CENTER, textColor=ACCENT, spaceBefore=4, spaceAfter=3)
st_abs = S("abs", fontName="Times-Roman", fontSize=9.3, leading=12.6,
           alignment=TA_JUSTIFY, textColor=INK, leftIndent=22, rightIndent=22)
st_kw = S("kw", fontName="Times-Roman", fontSize=8.8, leading=11.5,
          alignment=TA_JUSTIFY, textColor=INK, leftIndent=22, rightIndent=22, spaceBefore=4)
st_h1 = S("h1", fontName="Times-Bold", fontSize=12, leading=14,
          textColor=ACCENT, spaceBefore=11, spaceAfter=4)
st_h2 = S("h2", fontName="Times-Bold", fontSize=10.3, leading=12.5,
          textColor=ACCENT2, spaceBefore=7, spaceAfter=3)
st_body = S("b", fontName="Times-Roman", fontSize=9.6, leading=13.1,
            alignment=TA_JUSTIFY, textColor=INK, spaceAfter=4.5, firstLineIndent=12)
st_body0 = S("b0", parent=st_body, firstLineIndent=0)
st_cap = S("cap", fontName="Times-Roman", fontSize=8.2, leading=10.4,
           alignment=TA_JUSTIFY, textColor=colors.HexColor("#333333"),
           leftIndent=10, rightIndent=10, spaceBefore=3, spaceAfter=7)
st_tcell = S("tc", fontName="Helvetica", fontSize=7.0, leading=8.4, textColor=INK)
st_tcellb = S("tcb", fontName="Helvetica-Bold", fontSize=7.0, leading=8.4, textColor=INK)
st_thead = S("th", fontName="Helvetica-Bold", fontSize=7.2, leading=8.6,
             textColor=colors.white)
st_ref = S("ref", fontName="Times-Roman", fontSize=8.3, leading=10.4,
           alignment=TA_JUSTIFY, textColor=INK, leftIndent=12, firstLineIndent=-12,
           spaceAfter=2.4)

# ---------------------------------------------------------------- helpers
FIGN = {"n": 0}
TABN = {"n": 0}

def img_flow(fname, width):
    p = os.path.join(FIGS, fname)
    iw, ih = ImageReader(p).getSize()
    h = width * ih / iw
    return Image(p, width=width, height=h)

def figure(fname, width, caption, center=True):
    FIGN["n"] += 1
    im = img_flow(fname, width)
    im.hAlign = "CENTER"
    cap = Paragraph(f'<b>Figure {FIGN["n"]}.</b> {caption}', st_cap)
    return KeepTogether([im, Spacer(1, 2), cap])

def figrow(fa, fb, width_each, caption):
    """Two figures side by side, single combined caption."""
    FIGN["n"] += 1
    ia = img_flow(fa, width_each); ib = img_flow(fb, width_each)
    t = Table([[ia, ib]], colWidths=[width_each + 6, width_each + 6])
    t.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"),
                           ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 0),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 0)]))
    cap = Paragraph(f'<b>Figure {FIGN["n"]}.</b> {caption}', st_cap)
    return KeepTogether([t, Spacer(1, 2), cap])

def figrow3(fa, fb, fc, width_each, caption):
    """Three figures side by side, single combined caption."""
    FIGN["n"] += 1
    ims = [img_flow(f, width_each) for f in (fa, fb, fc)]
    t = Table([ims], colWidths=[width_each + 4] * 3)
    t.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"),
                           ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 0),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 0)]))
    cap = Paragraph(f'<b>Figure {FIGN["n"]}.</b> {caption}', st_cap)
    return KeepTogether([t, Spacer(1, 2), cap])

def teaser(fname, width, caption):
    """A prominent figure with a thin framed border (page-1 hero image)."""
    FIGN["n"] += 1
    im = img_flow(fname, width)
    im.hAlign = "CENTER"
    inner = Table([[im]], colWidths=[width + 8])
    inner.setStyle(TableStyle([
        ("BOX", (0, 0), (-1, -1), 0.8, RULE),
        ("ALIGN", (0, 0), (-1, -1), "CENTER"),
        ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]))
    inner.hAlign = "CENTER"
    cap = Paragraph(f'<b>Figure {FIGN["n"]}.</b> {caption}', st_cap)
    return KeepTogether([inner, Spacer(1, 2), cap])

def P(text, style=st_body):
    return Paragraph(text, style)

def H1(num, text):
    return Paragraph(f"{num}&nbsp;&nbsp;{text}", st_h1)

def H2(num, text):
    return Paragraph(f"{num}&nbsp;&nbsp;{text}", st_h2)

def make_table(header, rows, col_w, caption, fontsize=7.0, band=True,
               align=None, head_bg=HEADER_BG):
    TABN["n"] += 1
    data = []
    data.append([Paragraph(h, st_thead) for h in header])
    for r in rows:
        cells = []
        for c in r:
            if isinstance(c, Paragraph):
                cells.append(c)
            else:
                cells.append(Paragraph(str(c), st_tcell))
        data.append(cells)
    t = Table(data, colWidths=col_w, repeatRows=1)
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), head_bg),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 3),
        ("RIGHTPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 2.2),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.2),
        ("LINEBELOW", (0, 0), (-1, 0), 0.7, colors.white),
        ("LINEBELOW", (0, -1), (-1, -1), 0.6, RULE),
        ("GRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#d6dee7")),
    ]
    if band:
        for i in range(1, len(data)):
            if i % 2 == 1:
                style.append(("BACKGROUND", (0, i), (-1, i), BAND))
            else:
                style.append(("BACKGROUND", (0, i), (-1, i), BAND2))
    if align:
        for col, a in align.items():
            style.append(("ALIGN", (col, 1), (col, -1), a))
    t.setStyle(TableStyle(style))
    cap = Paragraph(f'<b>Table {TABN["n"]}.</b> {caption}', st_cap)
    return [Spacer(1, 3), KeepTogether([t, Spacer(1, 2), cap])]

# ---------------------------------------------------------------- page deco
RUNNING = "Shadowing: Learning Deployable Robot Motion by Tracking Dynamically-Feasible Ghosts"
def on_page(canvas, doc):
    canvas.saveState()
    # top rule on pages after first
    if doc.page > 1:
        canvas.setStrokeColor(RULE); canvas.setLineWidth(0.5)
        canvas.line(LM, PAGE_H - TM + 14, PAGE_W - RM, PAGE_H - TM + 14)
        canvas.setFont("Times-Italic", 7.5)
        canvas.setFillColor(colors.HexColor("#555555"))
        canvas.drawString(LM, PAGE_H - TM + 18, "Shadowing — OmniSim Technical Report")
        canvas.drawRightString(PAGE_W - RM, PAGE_H - TM + 18, "")
    # footer
    canvas.setStrokeColor(RULE); canvas.setLineWidth(0.5)
    canvas.line(LM, BM - 10, PAGE_W - RM, BM - 10)
    canvas.setFont("Times-Roman", 8)
    canvas.setFillColor(colors.HexColor("#555555"))
    canvas.drawString(LM, BM - 20, "OmniSim Research · Technical Report (preprint)")
    canvas.drawRightString(PAGE_W - RM, BM - 20, f"{doc.page}")
    canvas.restoreState()

# ---------------------------------------------------------------- build
def build():
    doc = BaseDocTemplate(OUT, pagesize=letter,
                          leftMargin=LM, rightMargin=RM, topMargin=TM, bottomMargin=BM,
                          title="Shadowing: Learning Deployable Robot Motion by Tracking Dynamically-Feasible Ghosts",
                          author="OmniSim Research")
    frame = Frame(LM, BM, CW, PAGE_H - TM - BM, id="main",
                  leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
    doc.addPageTemplates([PageTemplate(id="all", frames=[frame], onPage=on_page)])

    E = []  # story
    A = E.append

    # ---------- title block ----------
    A(Spacer(1, 2))
    A(Paragraph("Shadowing: Learning Deployable Robot Motion by Tracking "
                "Dynamically-Feasible Ghosts", st_title))
    A(Paragraph("Planning describes the problem; control learns to solve it.", st_sub))
    A(Paragraph("A. Fetouh", st_auth))
    A(Paragraph("OmniSim Research", st_affil))
    A(Paragraph("Technical Report &mdash; July 2026", st_affil))
    A(Spacer(1, 5))
    A(HRFlowable(width="70%", thickness=0.8, color=RULE, spaceBefore=2, spaceAfter=6,
                 hAlign="CENTER"))

    # ---------- teaser ----------
    A(teaser("omni_g1_ghost.png", CW * 0.60,
        "<b>Shadowing, visualized in OmniSim.</b> The deployed robot (left, opaque, "
        "Unitree&nbsp;G1) reproduces the motion of the dynamically-feasible reference it was "
        "trained to follow&mdash;<i>the ghost</i> (right, translucent). A planner makes the "
        "ghost <i>physically executable by construction</i>; reinforcement learning learns "
        "only to shadow it robustly into deployment. <i>Planning describes the problem; "
        "control learns to solve it.</i> <b>NOTE &mdash; this G1 is on a balance "
        "harness.</b> The gait shown runs on a <b>weight-bearing pelvis crane</b> "
        "(&le;700&nbsp;N vertical lift &asymp; <b>2&times; the robot's body weight</b>; "
        "&plusmn;350&nbsp;N&middot;m attitude torque). <b>The legs step for real; the crane "
        "keeps the robot up.</b> This figure therefore demonstrates <i>tracking fidelity</i>, "
        "<b>not free-standing balance</b>. No G1 result in this paper is a free-standing "
        "durable walk, and none is claimed (Sections&nbsp;4 and 5.3)."))


    # ---------- abstract ----------
    A(Paragraph("Abstract", st_abhead))
    A(Paragraph(
        "We present <b>Shadowing</b>, a recipe for producing deployable whole-body "
        "robot motion that separates a hard problem into two tractable ones: a "
        "<i>planner</i> produces a dynamically-feasible reference trajectory&mdash;"
        "<i>the ghost</i>&mdash;and a reinforcement-learning (RL) <i>tracker</i> learns to "
        "<i>shadow</i> it robustly across the simulation-to-deployment gap. Our central "
        "empirical claim is that the bottleneck in learning deployable motion is "
        "<i>not the RL</i> but the <b>dynamic feasibility of the reference being imitated</b>: "
        "if a policy is asked to shadow a ghost the robot cannot physically execute, no "
        "amount of reward shaping, curriculum, or compute makes it succeed&mdash;the policy "
        "parks in a degenerate local optimum. Shadowing makes feasibility a first-class, "
        "<i>certified</i> property through three components: (1) a robot-agnostic "
        "<b>ghost generator</b> that discovers feasible motion by receding-horizon "
        "predictive-sampling control (MPPI) over the robot's full contact dynamics; "
        "(2) a <b>ghost verifier</b> that numerically certifies feasibility&mdash;via a "
        "per-step contact-wrench feasibility program&mdash;<i>before</i> any learning; and "
        "(3) an RL <b>tracker</b> that shadows the certified ghost under domain "
        "randomization. The certificate is not a new test&mdash;the contact-wrench "
        "feasibility program is classical; our contribution is its <i>architectural "
        "placement</i> as an RL-independent gate run <i>before</i> learning. On a graded "
        "sweep of eight Go2 trot references we find the certificate score predicts deploy "
        "durability (Spearman &rho;&nbsp;=&nbsp;+0.94, n=8, p=0.0006), though we show this "
        "is a <i>separation at the feasibility cliff</i>, not a graded regressor. "
        "We demonstrate the pipeline across <b>five robot morphologies</b> "
        "(the Unitree G1 humanoid, a OmniQuad, the Unitree Go2 and B2 "
        "quadrupeds, and a fixed-base 6-DOF manipulator) and six motion classes. The "
        "<b>quadrupeds cross the deployment gap durably</b>&mdash;OmniQuad walks "
        "<b>47.8&nbsp;m</b>, Go2 <b>86.7&nbsp;m</b>, and B2 <b>110.7&nbsp;m</b>, all at zero "
        "falls&mdash;and a manipulation throw lands a part <b>1.5&nbsp;cm from a target "
        "beyond the arm's kinematic reach</b>. We are explicit about what produced those "
        "three walks: they are <b>residual RL on an <i>analytic</i> foot-space trot</b>, "
        "i.e. the degenerate case of this pipeline in which the reference is written down "
        "rather than generated and certified. To test whether the full method <i>earns its "
        "cost</i>, we ran the two head to head on one robot. On the Go2, a policy shadowing "
        "a <b>certified, recorded ghost</b> beats the residual-RL incumbent it was distilled "
        "from&mdash;<b>0.429 vs 0.381&nbsp;m/s (+12.6%)</b> with <b>5&times; less lateral "
        "drift</b> and zero falls on both sides, over three interleaved 240&nbsp;s live "
        "deploy runs in the same world under the same physics. The <b>biped is where we "
        "state the limit plainly</b>: <b>no from-scratch G1 policy walks durably free-standing "
        "in our deploy engine</b>, and the best G1 gait quality we can show is produced on a "
        "<b>weight-bearing balance harness</b> that carries up to twice the robot's weight&mdash;"
        "so it is a <i>tracking-fidelity</i> result, not a balance result. A durable "
        "free-standing humanoid deploy walk remains <b>OPEN</b>. "
        "Finally, we report two <b>negative results</b> that delineate the method's "
        "boundary, because a feasible reference is <i>necessary but not sufficient</i>. "
        "The dead-seated sit-to-stand launch resists <b>both</b> 21 reward-RL runs "
        "<b>and</b> MPC-distillation/DAgger; both stall at the same statically-stable crouch "
        "because completing the rise requires a <i>predictive commitment</i> to a "
        "temporarily-unstable extension that a purely reactive learned policy will not make. "
        "And a 7&nbsp;cm stair-climb ghost <b>passes every feasibility gate we have, yet no "
        "policy climbs it</b>&mdash;the sharpest evidence we can offer against our own "
        "certificate's sufficiency.", st_abs))
    A(Paragraph("<b>Keywords:</b> legged locomotion, whole-body control, imitation "
                "learning, trajectory optimization, sim-to-real transfer, reinforcement "
                "learning, motion tracking, dynamic feasibility.", st_kw))
    A(Spacer(1, 4))
    A(HRFlowable(width="70%", thickness=0.8, color=RULE, spaceBefore=2, spaceAfter=8,
                 hAlign="CENTER"))

    # ================= 1. Introduction =================
    A(H1("1", "Introduction"))
    A(P("Getting a physical robot to perform a complex whole-body motion&mdash;walking far "
        "without falling, climbing a hill, rising from the ground, or throwing an object "
        "to a target&mdash;remains stubbornly hard, even though each of these motions has "
        "been demonstrated in isolation many times. Two broad tool families dominate. "
        "<i>Trajectory optimization</i> is excellent at <b>discovering</b> what a feasible "
        "motion looks like: it can reason about contact handoffs, centre-of-mass "
        "excursions, and torque limits as explicit constraints. But the trajectories it "
        "produces are open-loop and brittle&mdash;they assume a perfect model and shatter "
        "under the modelling error and disturbance of real deployment. <i>Reinforcement "
        "learning</i> is the opposite: it produces robust closed-loop feedback that "
        "crosses the sim-to-real gap through domain randomization, but it is notoriously "
        "bad at <b>discovering</b> a complex coordinated motion from a sparse reward.", st_body0))
    A(P("Imitation learning bridges the two by giving RL a <i>reference</i> motion to "
        "track&mdash;the DeepMimic line of work [1] and its descendants [2,3,4]. This works "
        "beautifully in simulation. Our contribution begins with an uncomfortable "
        "observation about <i>deployment</i>: in practice, the reference itself is usually "
        "<b>hand-drawn or kinematically retargeted</b>, and a kinematic reference is a "
        "<i>puppet</i>&mdash;it has no physics, cannot fall, and ignores momentum and "
        "contact. Asking a policy to “just track it” then quietly smuggles the entire "
        "dynamic-balance problem back into the RL, which is exactly the part RL is worst "
        "at. We found, repeatedly and expensively, that when the reference is dynamically "
        "infeasible the policy does not track it; it discovers a nearby <i>degenerate</i> "
        "behaviour that collects partial reward (stay seated, half-crouch, bow forward) "
        "and never deploys."))
    A(P("This paper makes that observation precise and actionable. We argue&mdash;and show "
        "with a controlled ablation&mdash;that <b>the dynamic feasibility of the reference, "
        "not the RL algorithm, is the binding constraint on learning deployable motion.</b> "
        "We then build a pipeline, <b>Shadowing</b>, that treats feasibility as a "
        "first-class, <i>certified</i> property (Figure 2). The name reflects the data "
        "flow: a planner emits a dynamically-feasible reference we call <i>the ghost</i>, "
        "and the robot learns to <i>shadow</i> it."))

    A(figure("fig_architecture.png", CW,
        "The Shadowing pipeline. An <i>intent</i> (robot model plus goal/keyframes) is "
        "turned by the <b>ghost generator</b> into a dynamically-feasible reference "
        "trajectory (the <i>ghost</i>) using receding-horizon predictive-sampling control "
        "over the robot's full contact dynamics, so the ghost is feasible <i>by "
        "construction</i>. The <b>ghost verifier</b> numerically certifies feasibility "
        "before any learning. The <b>tracker</b> is an RL policy that shadows the certified "
        "ghost under domain randomization, crossing the sim-to-deployment gap. The same "
        "machinery serves any robot and any motion: only the intent changes."))

    A(P("<b>Contributions.</b> (i) The Shadowing architecture&mdash;generator&rarr;verifier"
        "&rarr;tracker&mdash;a general recipe to make a robot perform a motion in "
        "deployment. (ii) A robot-agnostic <i>feasible-ghost generator</i> whose interface "
        "is strictly <i>(robot model, intent) &rarr; ghost</i>, with nothing "
        "robot-specific. (iii) A <i>ghost-feasibility certificate</i> used as a "
        "<b>pre-RL gate</b>. <b>We claim placement, not invention:</b> the contact-wrench "
        "feasibility program is classical [10,11,12], and we do not improve it. What is new, "
        "to our knowledge, is coupling an <i>RL-independent</i> dynamic-feasibility "
        "certificate to an imitation-RL pipeline as a gate run <i>before</i> any learning, "
        "so that a training failure is never ambiguous (&sect;2.1 positions this against "
        "the one prior RL-independent gate we found, KungFuBot/PBHC [9]). "
        "(iv) Empirical evidence that the certificate <i>score</i> predicts learnability "
        "and deploy durability (&rho;=+0.94 over a graded 8-point sweep, &sect;5.2), "
        "reported with the caveats that weaken it. (v) Generality across five morphologies "
        "and six motion classes. (vi) <b>Two negative results</b> that delineate the "
        "boundary&mdash;the sit-to-stand launch and a certified-but-unclimbable "
        "staircase&mdash;showing precisely where “planning describes, control solves” holds "
        "and where it does not."))

    # ================= 2. Related Work =================
    A(H1("2", "Related Work"))
    A(P("<b>Imitation of reference motion.</b> DeepMimic [1] trains RL policies to track "
        "mocap clips and established reference-state initialization and phase "
        "observations. AMP [2] replaces explicit tracking with an adversarial style "
        "reward; PHC [3] and MaskedMimic [4] scale tracking to whole mocap corpora and "
        "show that, <i>in simulation</i>, universal motion tracking is largely solved. "
        "None of these applies a feasibility test to the reference. Our emphasis is the "
        "orthogonal axis they hold fixed: whether the reference is <i>dynamically "
        "executable on the target robot</i>, which is what governs <i>deployment</i> "
        "rather than in-sim imitation.", st_body0))
    A(P("<b>Sim-to-real humanoid control, and the one prior pre-training gate.</b> "
        "ExBody [7] and HumanPlus [8] transfer whole-body humanoid skills to hardware and "
        "handle reference infeasibility <i>implicitly</i>&mdash;by relaxing leg tracking, "
        "or by curating the motion set by hand. H2O/OmniH2O [5,6] introduce an explicit "
        "pre-training filter, but it is <b>RL-dependent</b>: a privileged imitation policy "
        "is trained, and the motions it fails to track are discarded&mdash;which conflates "
        "<i>policy competence</i> with <i>reference feasibility</i>, the very confound we "
        "want to remove. <b>KungFuBot/PBHC [9] is the closest prior work to our central "
        "claim</b>, and to our knowledge the only other <i>RL-independent</i> pre-training "
        "feasibility gate: it filters motions by a centre-of-mass / centre-of-pressure "
        "proximity criterion. It is a binary stability threshold on a kinematic quantity, "
        "whereas our gate is a dynamic <i>torque-and-contact-wrench</i> trackability "
        "program that yields a graded score. We claim a difference in kind and in "
        "placement&mdash;not that we invented feasibility checking."))
    A(P("<b>Model-based feasibility: where our certificate comes from.</b> The certificate "
        "we use is <b>not new</b>. It is the classical contact-wrench feasibility test: "
        "Hirukawa et al. [10] give the universal contact-stability criterion; Caron, Pham "
        "and Nakamura [11] derive the closed-form contact-wrench cone (friction cone + "
        "ZMP-support + yaw-torque) for rectangular supports; Dai, Valenzuela and Tedrake "
        "[12] embed centroidal-dynamics feasibility inside whole-body trajectory "
        "optimization. Capturability analysis [22] certifies balance for planners. "
        "Classical results also establish that a walking biped is <i>open-loop unstable</i> "
        "and that <b>ZMP-feasible &ne; stable</b> [13]&mdash;which is precisely why our "
        "certificate tests <i>inverse-dynamics feasibility</i> rather than open-loop "
        "stability: the RL tracker is what closes the loop. <b>None of this line couples "
        "the certificate to a learned tracking policy as a pre-RL gate</b>, which is the "
        "gap we fill."))
    A(P("<b>Trajectory optimization, predictive control, and tracking a planned "
        "reference.</b> Contact-implicit and whole-body trajectory optimization "
        "(Crocoddyl [14], OCS2, TOWR [15]) and online predictive control (MuJoCo MPC [16], "
        "MPPI [17]) discover dynamically feasible motion subject to contact and torque "
        "constraints. We use predictive-sampling MPC as the generator, but argue its "
        "open-loop output should be <i>tracked</i> by a learned closed-loop policy rather "
        "than executed directly. The closest tracking-from-optimization work is Cassie "
        "RL [18], which learns to track a hybrid-zero-dynamics gait library that is "
        "feasible by construction&mdash;but it runs <i>no separate certificate</i>, so "
        "feasibility is inherited from the generator rather than tested, and the library "
        "is tracked only softly."))
    A(P("<b>Quadruped sim-to-real.</b> Our quadruped deployments are simulation-to-"
        "simulation, and the durable-locomotion baselines they should be read against are "
        "ANYmal [24], perceptive locomotion in the wild [25], and DreamWaQ [26], all of "
        "which cross to <i>hardware</i>. We do not, and we do not claim parity with them "
        "(&sect;2.1)."))
    A(P("<b>Composition and distillation.</b> Sequential composition of control funnels "
        "[19] and LQR-trees [20] formalize landing a transient motion in the basin of a "
        "solved primitive; we use this idea to anchor phase-ghosts in solved terminal "
        "behaviours. DAgger [21] distills an expert (here, the generator's MPC) into a "
        "reactive policy&mdash;our negative result shows where this distillation, like "
        "reward-RL, is fundamentally limited."))
    A(H2("2.1", "Positioning: placement, not invention"))
    A(P("Table 1 positions Shadowing along the single axis that governs <i>deployment</i>: "
        "whether the reference being learned is <i>dynamically executable on the target "
        "robot</i>, and whether anyone checks <i>before</i> spending the training run. "
        "Imitation methods take the reference as <i>given</i> and never certify it; "
        "feasibility is assumed, and when it fails it surfaces as a hard-to-debug training "
        "failure that is easily misattributed to the learner. Trajectory optimization "
        "produces feasible motion, but as <i>open-loop, brittle</i> trajectories that do "
        "not by themselves survive the deployment gap. H2O/OmniH2O [5,6] do filter, but "
        "with a trained policy&mdash;so the filter cannot distinguish an infeasible "
        "reference from an undertrained tracker. KungFuBot/PBHC [9] filters "
        "RL-independently, but on a kinematic CoM/CoP proximity criterion.", st_body0))
    A(P("<b>Our claim, stated narrowly.</b> We did <i>not</i> invent feasibility checking: "
        "the contact-wrench program is classical [10,11,12] and we do not improve it. The "
        "contribution is its <b>architectural placement</b>&mdash;an explicit, "
        "RL-independent, <i>dynamic</i> feasibility certificate used as a gate <i>before</i> "
        "imitation RL, whose score predicts learnability (&sect;5.2). What it buys is "
        "diagnostic: it turns an ambiguous “the policy will not learn” into a decisive "
        "“the reference is infeasible&mdash;relax the intent,” and, when the ghost passes "
        "and the policy still fails, it <i>exonerates the reference</i> and localizes the "
        "failure to the tracker or the deployment pipeline. That partition is the practical "
        "value of the gate, and it is what the prior art leaves implicit."))
    A(P("<b>An honesty note on Table 1.</b> Our deploy results are "
        "<b>simulation-to-simulation</b> (a GPU MuJoCo-warp trainer &rarr; OmniSim's Newton "
        "back-end). <b>We have not run on physical hardware.</b> H2O/OmniH2O [5,6], "
        "ExBody [7], HumanPlus [8], KungFuBot [9], Cassie RL [18] and the quadruped "
        "baselines [24,25,26] all cross to a real robot. The “deploy target” column must "
        "therefore <b>not</b> be read as a hardware comparison: those rows and ours are not "
        "on equal footing, and a sim-to-sim gap is a proxy for&mdash;not a substitute "
        "for&mdash;a sim-to-real one."))

    sh = st_tcellb
    E.extend(make_table(
        ["Approach", "Reference source", "Feasible by<br/>construction?",
         "Certified<br/>before RL?", "Discovers<br/>motion?", "Deploy<br/>target"],
        [["DeepMimic [1]", "mocap clip", "no (assumed)", "no", "no", "sim"],
         ["AMP [2]", "mocap distribution", "no (assumed)", "no", "style only", "sim"],
         ["PHC / MaskedMimic [3,4]", "mocap corpus", "no (assumed)", "no", "no", "sim"],
         ["ExBody / HumanPlus [7,8]", "retargeted human", "no (retarget&ne;feasible)",
          "no (implicit: relax /<br/>curate)", "no", "real robot"],
         ["H2O / OmniH2O [5,6]", "retargeted human", "no (retarget&ne;feasible)",
          Paragraph("filtered, but <b>RL-dependent</b> (trained policy)", st_tcell),
          "no", "real robot"],
         ["KungFuBot / PBHC [9]", "retargeted human",
          "no (filtered after)",
          Paragraph("<b>yes</b> &mdash; RL-indep., but <i>kinematic</i> CoM/CoP, binary", st_tcell),
          "no", "real robot"],
         ["Cassie RL [18]", "HZD gait library", "yes (by construction)",
          "no (inherited, not tested)", "yes", "real robot"],
         ["MuJoCo-MPC / MPPI [16,17]", "online optimization", "yes (own model)", "n/a (is the planner)", "yes", "open-loop / MPC"],
         ["Traj. opt. (TOWR,<br/>Crocoddyl) [14,15]", "offline optimization", "yes (constraints)", "implicit", "yes", "open-loop, brittle"],
         ["DAgger distillation [21]", "an expert policy", "inherits expert", "no", "no", "varies"],
         [Paragraph("<b>Shadowing (ours)</b>", sh),
          Paragraph("<b>optimized ghost</b>", sh),
          Paragraph("<b>yes (real dynamics)</b>", sh),
          Paragraph("<b>yes</b> &mdash; RL-indep., <b>dynamic</b> (contact-wrench), graded score", sh),
          Paragraph("<b>yes</b>", sh),
          Paragraph("<b>sim&rarr;sim only</b> (no hardware)", sh)]],
        [CW*0.19, CW*0.155, CW*0.155, CW*0.21, CW*0.11, CW*0.18],
        "Positioning Shadowing along the axis that governs deployment: is the learned "
        "reference guaranteed executable on the target robot, and is that checked "
        "<i>before</i> learning? The nearest prior gates are H2O/OmniH2O (a filter, but "
        "RL-dependent&mdash;it cannot separate an infeasible reference from an undertrained "
        "tracker) and KungFuBot/PBHC (RL-independent, but a binary <i>kinematic</i> CoM/CoP "
        "threshold). Ours is RL-independent and <i>dynamic</i>, and yields a graded score. "
        "<b>Note &mdash; the final column is not a like-for-like comparison: every prior row "
        "deploys to physical hardware and we do not</b> &mdash; our results are "
        "simulation-to-simulation.",
        fontsize=6.5, align={2: "CENTER", 3: "CENTER", 4: "CENTER", 5: "CENTER"}))

    # ================= 3. Method =================
    A(H1("3", "Method: The Shadowing Pipeline"))
    A(P("<b>The data contract.</b> All three components communicate through a single, "
        "robot-agnostic interface: a time-indexed <i>reference table</i> "
        "<i>r</i>[t] = {<i>q</i>[t], base_pose[t], contacts[t]} of joint angles, root "
        "pose, and a contact schedule, optionally augmented with reference velocities "
        "and torques. Any planner can produce this table; one generic tracker consumes it. "
        "A new motion is a new intent &rarr; planner &rarr; table with the <i>same</i> "
        "tracker; a new robot is its model with the <i>same</i> machinery. The pipeline is "
        "a function <i>(robot model, motion intent) &rarr; deployable policy</i>.", st_body0))

    A(H2("3.1", "Ghost Generator"))
    A(P("The generator turns an <i>intent</i> into a feasible reference. The intent is a "
        "compact, robot-agnostic specification: a total duration, a list of "
        "smoothstep-interpolated joint and base-pose keyframes, optional per-joint "
        "velocity limits, a set of <i>active</i> joints the optimizer may perturb, and an "
        "optional set of <i>prescribed</i> joints whose trajectory is forced (e.g. a "
        "hand wave the body must balance around) plus an arbitrary task cost (e.g. a "
        "release-velocity objective for a throw).", st_body0))
    A(P("Feasibility is obtained <b>by construction</b> by optimizing in the space of the "
        "robot's <i>position-target actuators</i>&mdash;the exact interface deployment "
        "uses&mdash;over the full MuJoCo contact dynamics, with a "
        "<b>receding-horizon predictive-sampling controller (MPPI)</b>. At each control "
        "step we sample <i>K</i> noisy control sequences over a short horizon, roll each "
        "forward through the real simulator (so actuator-force and velocity limits and "
        "contacts are all respected), score them by a per-step cost, take the "
        "cost-softmax-weighted mean as the new nominal, execute only its first action, and "
        "shift the warm-started horizon forward. We found this receding-horizon scheme "
        "essential: optimizing the <i>whole</i> trajectory at once went chaotic at "
        "stiff-contact knife-edges. The per-step cost is a weighted sum of joint and "
        "base-pose tracking, an upright term, a balance term (centre of mass over the "
        "foot-support centroid), an effort term, a velocity-limit penalty, and a survival "
        "term (Table 2). Because the rollouts are real dynamics, a control that violates "
        "torque or velocity limits simply cannot reduce the cost&mdash;the output is "
        "feasible w.r.t. the actuators by construction. If the optimizer cannot satisfy the "
        "intent, the intent itself is unachievable and must be relaxed."))

    E.extend(make_table(
        ["Parameter", "Symbol", "Value", "Notes"],
        [["sim timestep", "dt<sub>sim</sub>", "0.004 s", "250 Hz physics"],
         ["control timestep", "dt<sub>ctrl</sub>", "0.016&ndash;0.02 s", "50&ndash;62 Hz, deploy-matched"],
         ["planning horizon", "H", "12&ndash;45 steps", "0.3&ndash;0.9 s lookahead"],
         ["sampled rollouts", "K", "48&ndash;200", "per control step"],
         ["noise std (targets)", "&sigma;", "0.08&ndash;0.22 rad", "AR(0.6/0.4) smoothed"],
         ["temperature", "&lambda;", "0.4&ndash;0.5", "MPPI softmax"],
         [Paragraph("<b>cost: joint track</b>", st_tcellb), "w<sub>q</sub>", "3&ndash;8", "mean sq. joint error"],
         [Paragraph("<b>cost: base x/y/z</b>", st_tcellb), "w<sub>b</sub>", "2&ndash;22", "root position"],
         [Paragraph("<b>cost: upright</b>", st_tcellb), "w<sub>u</sub>", "1.5&ndash;3", "torso tilt"],
         [Paragraph("<b>cost: balance</b>", st_tcellb), "w<sub>bal</sub>", "0.3&ndash;5", "CoM over support"],
         [Paragraph("<b>cost: vel-limit</b>", st_tcellb), "w<sub>v</sub>", "50", "hinge over limit"]],
        [CW*0.30, CW*0.13, CW*0.22, CW*0.35],
        "Ghost-generator (MPPI) parameters and cost weights, spanning the motions in this "
        "paper. Ranges reflect per-motion intent files; e.g. the B2 get-up uses a strong "
        "base-height weight (22) to avoid over-extension, the Go2 trot uses a deliberately "
        "small balance weight (0.3) so a quasi-static term does not fight the dynamic gait.",
        align={1: "CENTER", 2: "CENTER"}))

    A(H2("3.2", "Ghost Verifier"))
    A(P("The generator certifies a ghost only against <i>its own model</i>. The verifier "
        "closes the loop by certifying feasibility <b>numerically and independently of the "
        "RL</b>, so that a downstream training failure is never ambiguous between "
        "“bad reference” and “bad learner.” For legged motion the principled "
        "certificate is a per-step <b>contact-wrench feasibility program</b>. At each "
        "frame we form the rigid-body dynamics <i>M(q) q<super>&middot;&middot;</super> + h(q, q<super>&middot;</super>) = "
        "S<sup>T</sup>&tau; + &Sigma;<sub>i</sub> J<sub>i</sub><sup>T</sup> f<sub>i</sub></i> "
        "and ask a small linear program whether there exist contact forces "
        "<i>f<sub>i</sub></i> (inside a friction pyramid, with non-negative normal force) "
        "and within-limit joint torques <i>&tau;</i> that reproduce the ghost's measured "
        "acceleration. The six unactuated base rows encode the Zero-Moment-Point / balance "
        "condition&mdash;they must be satisfiable by contacts alone; the actuated rows "
        "define the required torque, which must respect the actuator limits (Table 3).", st_body0))
    A(P("The verifier reports a base-wrench residual (phantom force the contacts cannot "
        "supply, as a fraction of body weight), a torque-margin ratio, and a "
        "contact-consistency check, aggregated robustly (95th percentile) into a scalar "
        "feasibility score and a pass/fail gate. The certificate cleanly <b>passes</b> "
        "planner-generated and hand-authored feasible trots (G1, Go2, B2) and "
        "<b>fails</b> a deliberately infeasible levitating-base reference (its base "
        "demands &asymp;101% of body weight from absent contacts). For fixed-base "
        "manipulators the analogous certificate checks velocity margin, torque margin "
        "(inverse dynamics on the true arm), open-loop tracking drift, and whether a "
        "ballistic release reaches the target&mdash;the throw's <i>feasibility frontier</i>. "
        "An alternative open-loop-sustainability gate (replay the ghost under the robot's "
        "own PD and check it does not fall) is a useful interim screen but is conceptually "
        "wrong for dynamic gaits, which are open-loop unstable yet perfectly trackable; "
        "and a naive inverse-dynamics certificate over-reports on contact-rich motion "
        "because contacts do not activate at zero-penetration snapshots. The contact-wrench "
        "program is what resolves both."))

    E.extend(make_table(
        ["Metric", "Definition", "Pass threshold"],
        [["base-wrench residual", "p95 phantom base force &divide; m&middot;g", "&lt; 0.08"],
         ["torque margin", "p95 max |&tau;| &divide; &tau;<sub>limit</sub>", "&lt; 1.05"],
         ["contact consistency", "fraction of frames with no valid contact", "&le; 0.05"],
         ["(arm) velocity margin", "max |q<super>&middot;</super>| &divide; q<super>&middot;</super><sub>limit</sub>", "&le; 1.05"],
         ["(arm) tracking drift", "max |q<sub>achieved</sub> &minus; q<sub>ghost</sub>|", "&lt; 0.20 rad"],
         ["(arm) reaches target", "|landing &minus; target|", "&lt; 0.05 m"]],
        [CW*0.27, CW*0.46, CW*0.27],
        "Ghost-verifier feasibility metrics and gates. Legged metrics come from the "
        "per-step contact-wrench LP (friction coefficient &mu;=1.0, friction-pyramid "
        "approximation); manipulator metrics from inverse dynamics on the true arm with the "
        "generation-time damping removed.",
        align={2: "CENTER"}))

    A(H2("3.3", "Tracker (the shadower)"))
    A(P("The tracker is an RL policy that shadows the certified ghost under domain "
        "randomization. The observation (53-dim for the G1 sit-stand; 48-dim for the "
        "quadruped walkers, plus task terms) contains base linear/angular velocity, "
        "projected gravity, the joint <i>deviation from the reference</i> "
        "<i>q&minus;q<sub>ref</sub></i>(t), joint velocities, the last action, and a small "
        "phase block carrying a short reference look-ahead (the upcoming stand fraction and "
        "base-pose errors). The action is a bounded residual on the reference joint "
        "targets, <i>q<sub>target</sub> = q<sub>ref</sub>(t) + &rho;&middot;a</i>, so a "
        "tight residual scale forces the policy to follow the ghost rather than override it "
        "to a safe compromise.", st_body0))
    A(P("The reward is deliberately simple&mdash;“all the cleverness is in the "
        "reference”&mdash;a sum of exponential tracking terms on the legs, base height, "
        "base-x and lean, an alive bonus, small effort and action-rate penalties, and a "
        "termination penalty:"))
    A(Paragraph(
        'R = &Sigma;<sub>i</sub> w<sub>i</sub> exp(&minus;k<sub>i</sub> e<sub>i</sub><super>2</super>) '
        '+ w<sub>alive</sub> + (velocity-tracking terms) &minus; penalties,',
        S("eq", parent=st_body, alignment=TA_CENTER, firstLineIndent=0, fontName="Times-Italic",
          fontSize=9.8, spaceBefore=3, spaceAfter=3)))
    A(P("where the exponential tracking errors <i>e<sub>i</sub></i> are, in turn, the "
        "leg-joint pose error <i>q&minus;q<sub>ref</sub></i>, the base height "
        "<i>z&minus;z<sub>ref</sub></i>, the base-x <i>x&minus;x<sub>ref</sub></i>, and the "
        "lean <i>&theta;&minus;&theta;<sub>ref</sub></i>. The height term dominates "
        "(sharpness <i>k</i>&nbsp;=&nbsp;20&ndash;25), which is what keeps the pelvis on the "
        "reference rather than letting the policy buy pose error back with a crouch."))
    A(P("A crucial ingredient, in the spirit of DeepMimic, is <b>velocity tracking</b>: "
        "matching the reference joint and base <i>velocities</i>, not only poses. With "
        "pose-only tracking the policy can satisfy the position terms while drifting in "
        "phase; adding velocity terms is what let the G1 shadow the feasible sit-stand "
        "<b>to 2.5&nbsp;cm</b> of pelvis-height error through the body of the motion. Two "
        "further tools matter for long or contact-rich motions: a <b>reverse curriculum</b> "
        "(reference-state initialization concentrated in a band that sweeps from near the "
        "goal back to the start, so the policy masters the receding frontier) and "
        "<b>hierarchical composition</b>&mdash;decomposing a long motion into phase-ghosts "
        "that each land in the basin of a solved primitive (e.g. sit-stand = stand-up "
        "&rarr; a solved standing balancer &rarr; sit-down), after Burridge&ndash;Tedrake "
        "funnels [19]. Domain randomization over mass, friction, actuator gains, and "
        "especially <i>contact stiffness</i> (the solver reference parameters) is what "
        "crosses the trainer&rarr;deploy physics gap."))

    # ================= 4. Experimental setup =================
    A(H1("4", "Experimental Setup"))
    A(P("All experiments run in <b>OmniSim</b>. Training uses a GPU-parallel MuJoCo-warp "
        "trainer; deployment uses OmniSim's Newton physics back-end, a deliberately "
        "different engine so that crossing the trainer&rarr;deploy gap is a genuine "
        "sim-to-sim (and a proxy for sim-to-real) test. Policies are small "
        "multilayer perceptrons (256&ndash;128 hidden units) trained with PPO "
        "(learning rate 3&times;10<sup>&minus;4</sup>, &gamma;=0.99, GAE &lambda;=0.95, "
        "clip 0.2, entropy 0.01) over 1024&ndash;4096 parallel environments. All training "
        "reported here ran on <b>local consumer GPUs</b> (laptop RTX 3060 / 5070 / 5070 Ti); "
        "we used no data-centre or cloud compute. A subtle but "
        "important throughput optimization underlies the whole campaign: the environment "
        "reset originally called a full physics <i>forward</i> (collision + constraint "
        "solve), which consumed roughly <b>60% of training time</b>; replacing it with a pure "
        "<i>kinematics</i> update&mdash;bit-identical, since link poses are a pure function "
        "of joint angles&mdash;gave a <b>2.5&times; end-to-end speedup</b> (39k &rarr; 98k "
        "environment-steps/s on the G1 walk trainer). Figure 3 summarizes the measured "
        "throughput; the quadruped trainers reach 662k environment-steps/s at 4096 "
        "environments on a laptop RTX 3060. (A later re-profiling of the same mechanism, on "
        "a <i>rented</i> RTX 4090, found the reset path still dominant at 66&ndash;81% of "
        "iteration time and reached <b>784k environment-steps/s, a 5.9&times; gain</b>, on the "
        "in-engine quadruped trainer. That measurement is the one exception to the "
        "local-compute statement above, and it postdates every result reported in this "
        "paper.)", st_body0))
    A(P("<b>A deploy-provenance note, because it nearly cost us a result.</b> Every ONNX "
        "deploy controller in our stack used to treat a policy that <i>failed to load</i> as "
        "a mode rather than an error: it printed one warning, fell back to its bare reference, "
        "and exited zero. A missing inference runtime on one machine therefore produced a "
        "complete, plausible, <i>entirely meaningless</i> head-to-head&mdash;in which the "
        "Shadowing side looked <i>better</i> than it really is, because a bare ghost replay is "
        "by construction a near-perfect match to the ghost. The tell was that the baseline "
        "under-ran its own published record. We now <b>assert that the policy loaded</b> before "
        "any run is scored, and a failed load is fatal. Every deploy number in this paper has "
        "been re-verified under that assertion (OmniQuad, Go2 and B2 each re-run with the policy "
        "confirmed loaded). We report this because the failure mode is invisible, general to "
        "any imitation-RL stack with a fallback reference, and it flatters exactly the method "
        "one is trying to evaluate.", st_body0))

    A(figure("fig_throughput.png", CW,
        "Training throughput, all first-hand on local consumer GPUs. <b>Left:</b> the "
        "reset optimization on the G1 walk trainer&mdash;replacing the per-reset full "
        "physics <i>forward</i> (collision + constraint solve, ~60% of train time) with a "
        "bit-identical <i>kinematics-only</i> refresh is a 2.5&times; end-to-end speedup. "
        "<b>Right:</b> sustained measured throughput. <b>Note &mdash; each bar is a different "
        "robot <i>and</i> a different GPU&mdash;this is not a scaling curve</b>, and the "
        "G1-stand entry is a measured range rather than a point value."))

    A(P("We evaluate on five robots: the Unitree <b>G1</b> humanoid (23 DOF, 34&nbsp;kg), a "
        "Boston-Dynamics-class <b>OmniQuad</b> quadruped, the Unitree <b>Go2</b> (16&nbsp;kg) "
        "and <b>B2</b> (60&nbsp;kg) quadrupeds, and a <b>fixed-base 6-DOF cobot arm</b>. "
        "Deployment metrics are forward distance, fall count, episode duration, "
        "and task-specific errors, measured in the Newton deploy engine."))

    A(P("<b>DISCLOSURE &mdash; the G1 runs on a balance harness.</b> "
        "<b>This is the single most important caveat in this paper and we put it before the "
        "results, not after them.</b> Our best-looking G1 gait&mdash;the ghost-shadowing "
        "walk in Figure&nbsp;1, and every G1 walk/turn/carry demonstration this project "
        "ships&mdash;is produced on a <b>weight-bearing balance harness</b>: a virtual "
        "pelvis crane applying a vertical spring/damper plus an attitude spring, clamped "
        "in code at <b>700&nbsp;N of upward force and &plusmn;350&nbsp;N&middot;m of "
        "attitude torque</b>, with a lateral catch and a yaw steer, all at 90% authority. "
        "The G1 weighs 34&nbsp;kg (&asymp;335&nbsp;N), so <b>the vertical clamp alone is "
        "&asymp;2&times; the robot's body weight: at the clamp, the robot is being "
        "carried.</b>", st_body0))
    A(P("<b>What that means, stated so it cannot be misread.</b> The legs step for real, "
        "the contact physics is real, and the tracking fidelity is real&mdash;but "
        "<b>the crane is holding the robot up, and balance is exactly the thing the "
        "deployment gap is about.</b> A harnessed walk is therefore <b>not</b> evidence of "
        "balance, and we draw no balance conclusion from it. Removing the harness "
        "(annealing its authority to zero) is an <b>open campaign, not a result</b>: "
        "<b>no G1 walk policy in this project has been shown at zero harness authority.</b> "
        "Accordingly: <b>this paper claims no durable free-standing G1 walk anywhere</b>; "
        "the G1 contributes <i>no</i> distance row to Table&nbsp;6 and <i>no</i> bar to "
        "Figure&nbsp;5a; and every G1 statement below is either a <i>trainer-side</i> "
        "learnability result or a <i>harness-supported tracking</i> result, labelled as "
        "such. <b>The quadruped and manipulator results&mdash;which are the paper's actual "
        "deploy claims&mdash;use no harness of any kind.</b> (The one G1 result that is "
        "vertically unassisted is the 3&nbsp;cm stair climb of &sect;5.5, where the lift "
        "term is set to zero and the legs do 100% of the vertical work; its lateral and "
        "attitude terms remain on, so the honest phrasing is “legs-only <i>vertically</i>,” "
        "not “unassisted.”)"))

    # ================= 5. Results =================
    A(H1("5", "Results"))

    A(H2("5.1", "Feasibility is causal (the controlled ablation)"))
    A(P("Our spine result holds the robot and the RL algorithm fixed and varies only the "
        "<i>feasibility</i> of the ghost (Table 4). A G1 with a foot-space gait model as its "
        "reference&mdash;dynamically feasible, and certified so by the verifier (base-wrench "
        "residual 4.6% of body weight)&mdash;<b>learns the motion</b>: the policy produces "
        "real stepping locomotion and tracks the reference. The <i>same</i> G1, the same "
        "learner, given a hand-drawn sit-to-stand reference that is only quasi-statically "
        "valid, <b>never learns the motion at all</b>: RL parks in a stay-seated local "
        "optimum and the motion falls. Same learner, opposite outcome&mdash;learns-to-step "
        "versus never-leaves-the-chair&mdash;because the reference's feasibility differs, "
        "isolating feasibility as the causal factor.", st_body0))
    A(P("<b>Read this ablation for exactly what it is, and no more.</b> It is a "
        "<i>learnability</i> ablation, and both arms are <b>trainer-side</b> outcomes. It "
        "establishes feasibility as <b>necessary</b>: an infeasible ghost is not merely "
        "harder to learn, it is a categorical failure that no amount of RL repairs. It does "
        "<b>not</b> establish that a feasible ghost is <b>sufficient</b>&mdash;neither for "
        "learning, nor for a durable deploy walk on this biped. It is not sufficient, and we "
        "give two independent demonstrations of that in &sect;5.4 and &sect;5.5. Nothing in "
        "this ablation should be read as a deploy result."))

    E.extend(make_table(
        ["Motion", "Ghost source", "Feasible?", "RL outcome (trainer-side)"],
        [["G1 walk", "foot-space gait model",
          Paragraph('<b>dynamically feasible</b> (certified, 4.6% mg)', st_tcellb),
          "<b>learns real stepping</b> and tracks the reference"],
         ["G1 sit-stand (step)", "hand-drawn keyframes",
          "only quasi-static",
          "<b>never learns the motion</b>; parks in stay-seated optimum; falls"]],
        [CW*0.18, CW*0.26, CW*0.20, CW*0.36],
        "The feasibility ablation. Identical robot and RL algorithm; only the ghost's "
        "dynamic feasibility differs, and it flips the outcome from <i>categorical "
        "non-learning</i> to <i>real stepping</i>. <b>Both outcomes are trainer-side:</b> "
        "this table shows feasibility is <b>necessary for learning</b>. It does not show "
        "it is sufficient, for learning or for deployment (&sect;5.4, &sect;5.5)."))

    A(H2("5.2", "Does the certificate <i>score</i> predict learnability? (a graded sweep)"))
    A(P("The ablation above is categorical: two motions, feasible versus not. If the "
        "certificate is to be useful as a <i>gate</i>, its score should track deploy "
        "outcome across a <i>graded</i> family of references. We test this directly. We "
        "generate <b>eight Go2 trot ghosts</b> at increasing commanded forward speed "
        "(vx = 0.3 &hellip; 2.5&nbsp;m/s), certify each <i>before</i> training, then train "
        "an identical residual tracker on each and measure deploy durability. Speed is the "
        "knob because it drives the reference smoothly from comfortably feasible into "
        "physically impossible, giving a feasibility gradient with everything else held "
        "fixed.", st_body0))
    A(P("The certificate score and the deploy outcome are strongly rank-correlated: "
        "<b>Spearman &rho;(score, first-fall) = +0.94</b> (n=8, p=0.0006), and "
        "&rho;(score, falls&nbsp;per&nbsp;env) = &minus;0.94. References scoring above zero "
        "survive <b>6.7&ndash;11.7&nbsp;s</b>; references scoring zero collapse to "
        "<b>1.1&ndash;1.5&nbsp;s</b> (Figure 4, Table 5). The certificate is computed "
        "before any learning, so this is a genuine <i>prediction</i>, not a post-hoc "
        "correlation."))
    A(P("<b>Three caveats, and they matter more than the correlation.</b> "
        "<b>(1) It is a cliff, not a regressor.</b> Four of the eight scores are tied at a "
        "saturated zero, so the &rho; is carried almost entirely by the <i>separation</i> "
        "between the feasible cluster and the infeasible one. We claim the certificate "
        "separates feasible from infeasible at the cliff; we do <b>not</b> claim its score "
        "is a calibrated predictor of <i>how much</i> better one feasible ghost is than "
        "another. <b>(2) Commanded speed alone nearly predicts the outcome</b> "
        "(&rho;(vx, first-fall) = &minus;0.88), so on this one-dimensional sweep the "
        "certificate is not clearly adding information beyond “faster is harder.” A "
        "sweep that varies feasibility <i>without</i> a monotone task-difficulty confound "
        "would be a stronger test, and we have not run one. <b>(3) The binary gate is too "
        "loose.</b> The pass/fail threshold rejects only the most extreme reference "
        "(vx=2.5); it <i>passes</i> vx=1.2, 1.6 and 2.0, all of which fall within "
        "1.5&nbsp;s. It is the graded <b>score</b>&mdash;which collapses to zero at the "
        "cliff as the torque margin saturates&mdash;that carries the signal, not the "
        "verdict. A practitioner should gate on the score, not the boolean."))

    A(figure("fig_learnability.png", CW,
        "<b>Certificate score vs deploy durability</b>, over eight Go2 trot ghosts of "
        "increasing commanded speed, each tracked by an identically-trained policy. "
        "<b>Left:</b> the pre-training score rank-correlates with time-to-first-fall "
        "(&rho;=+0.94, n=8, p=0.0006). <b>Note the shape:</b> the four infeasible "
        "references are <i>tied</i> at a saturated score of 0&mdash;this is a separation at "
        "the feasibility cliff, <b>not</b> a graded regressor. <b>Right:</b> the mechanism, "
        "and an honest weakness. The base-wrench residual grows smoothly with commanded "
        "speed and crosses the binary PASS threshold only at the very last point, so "
        "<b>the boolean gate passes three references that survive barely a second</b>. The "
        "score collapses to zero at vx&asymp;1.2 (the torque margin saturates) exactly where "
        "first-fall drops from 6.7&nbsp;s to 1.1&nbsp;s. Gate on the score, not the verdict."))

    E.extend(make_table(
        ["vx (m/s)", "base resid.<br/>(&times; mg)", "&tau; margin", "cert score",
         "binary gate", "first fall (s)", "falls / env"],
        [["0.3", "0.014", "0.31", "0.686", "PASS", "11.74", "3.1"],
         ["0.5", "0.020", "0.33", "0.671", "PASS", "11.17", "3.2"],
         ["0.7", "0.026", "0.46", "0.544", "PASS", "8.52", "4.3"],
         ["0.9", "0.032", "0.52", "0.476", "PASS", "6.72", "5.6"],
         ["1.2", "0.042", "1.00", Paragraph("<b>0.000</b>", st_tcellb),
          Paragraph("<b>PASS (!)</b>", st_tcellb), "1.14", "34.6"],
         ["1.6", "0.054", "1.00", Paragraph("<b>0.000</b>", st_tcellb),
          Paragraph("<b>PASS (!)</b>", st_tcellb), "1.51", "25.9"],
         ["2.0", "0.071", "1.00", Paragraph("<b>0.000</b>", st_tcellb),
          Paragraph("<b>PASS (!)</b>", st_tcellb), "1.12", "35.6"],
         ["2.5", "0.268", "1.00", "0.000", "FAIL", "1.15", "34.7"]],
        [CW*0.10, CW*0.15, CW*0.12, CW*0.13, CW*0.16, CW*0.17, CW*0.13],
        "The graded learnability sweep (n=8 Go2 trot references; committed data). The "
        "certificate <b>score</b> separates the feasible references (first fall "
        "6.7&ndash;11.7&nbsp;s) from the infeasible ones (1.1&ndash;1.5&nbsp;s) with "
        "&rho;=+0.94. <b>But the binary gate does not:</b> it passes vx = 1.2, 1.6 and "
        "2.0, each of which collapses in under 1.6&nbsp;s. Half the scores are tied at a "
        "saturated 0, and commanded speed alone reaches &rho;=&minus;0.88 against "
        "first-fall&mdash;so read this as a cliff separation, not a calibrated regressor.",
        fontsize=6.9,
        align={0: "CENTER", 1: "CENTER", 2: "CENTER", 3: "CENTER", 4: "CENTER",
               5: "CENTER", 6: "CENTER"}))

    A(H2("5.3", "Generality across robots and motions"))
    A(P("The same tracker, changing only the intent and the robot model, produces deployable "
        "policies across five morphologies and six motion classes (Table 6, Figures 5&ndash;7). "
        "The results split cleanly along one line, and we report both sides of it. We are also "
        "precise about <i>which reference</i> each result tracked, because that is the whole "
        "claim of this paper.", st_body0))
    A(P("<b>Quadrupeds cross the deployment gap.</b> OmniQuad walks <b>47.8&nbsp;m</b> "
        "dead-straight at zero falls; Go2 walks <b>86.7&nbsp;m</b> (0.38&nbsp;m/s) and B2 "
        "<b>110.7&nbsp;m</b> (0.49&nbsp;m/s), both at zero falls, in the Newton deploy "
        "engine. (These are the distances each run reached inside its measurement window "
        "with the robot still upright and advancing&mdash;lower bounds on durability, not "
        "fall points.) The recipe ports between quadrupeds cheaply: retargeting from the "
        "16&nbsp;kg Go2 to the 60&nbsp;kg B2 changes only leg constants, gait parameters, "
        "and actuator gains."))
    A(P("<b>A caveat we owe on the B2.</b> Its deploy and training stacks currently disagree "
        "about actuator stiffness (the deploy controller documents gains that look inherited "
        "from the 16&nbsp;kg Go2 and are implausible for a 60&nbsp;kg machine, while the "
        "training launcher uses far stiffer ones). The B2 walk is real&mdash;it was re-run for "
        "this report with the policy confirmed loaded, and it does not fall&mdash;but until "
        "that discrepancy is reconciled we would not defend the precise B2 <i>numbers</i> to "
        "three significant figures, and we flag them rather than quietly quote them.", st_body0))
    A(P("<b>What produced those three walks, precisely.</b> They are residual RL on an "
        "<i>analytic</i> foot-space trot: a reference written down in closed form (stance feet "
        "slide at &minus;v<sub>x</sub>, quintic swing, duty 0.6) rather than generated by the "
        "MPPI ghost generator and passed through the verifier. In the vocabulary of this paper "
        "that is the <b>degenerate case</b> of Shadowing&mdash;a reference whose feasibility is "
        "true by construction on a quadruped, where a statically-stable trot needs no "
        "certificate to be achievable. It is an honest baseline, not the method, and we label it "
        "as such in Table 6. Reporting it as evidence <i>for</i> ghost generation would be "
        "circular.", st_body0))
    A(P("<b>So we ran the two against each other on one robot.</b> On the Go2 we built a "
        "certified ghost by <i>recording the residual-RL incumbent's own achieved gait</i> "
        "(rolled deterministically on the deploy-matched model, steady segment phase-folded to "
        "64 bins) and passing it through the verifier, where the binding gate is the one that "
        "matters: the folded reference plus its declared feedforward, replayed by the bare "
        "deploy-grade PD with <i>no crane</i>, must walk&mdash;it does (0.382&nbsp;m/s, closed "
        "stance contacts, 0.976 of torque limit). Training a corridor-bounded residual against "
        "that ghost yields a policy that <b>beats the incumbent it was distilled from</b>: "
        "<b>0.429 vs 0.381&nbsp;m/s (+12.6%)</b>, <b>0.05&nbsp;m vs 0.26&nbsp;m</b> peak lateral "
        "drift (5&times; straighter), zero falls on both sides, across three interleaved "
        "240&nbsp;s live deploy runs in the same world, same physics, differing only in the "
        "controller. The student outrunning its teacher is the point: the ghost is the "
        "incumbent's <i>steady-state</i> gait, cleaned of the drift the incumbent itself "
        "accumulates, and the corridor holds the policy there.", st_body0))
    A(P("<b>Two caveats we hold ourselves to.</b> (i) That policy is <i>warm-started</i> from the "
        "incumbent's weights, so the +12.6% is an honest comparison of two <i>deployed policies</i> "
        "and a demonstration that Shadowing <i>upgrades an incumbent</i>&mdash;on its own it would "
        "not be a from-scratch method-versus-method result. So we also ran it from scratch; see "
        "below. (ii) Deploy is deterministic, so the three runs per side are reproducibility "
        "checks, not independent samples; the Shadowing policy scored 0.429&nbsp;m/s on every run.",
        st_body0))
    A(P("<b>From scratch, and the shuffle.</b> Training the same recipe with the policy weights "
        "initialized randomly&mdash;never seeing the incumbent&mdash;exposes a failure mode worth "
        "reporting on its own. On the champion's own hyperparameters (corridor 0.15, shape weight "
        "2.0) the policy converges to a <b>shuffle</b>: it tracks the ghost <i>better</i> than the "
        "champion does (<b>gmatch 0.898</b>) while barely translating (<b>0.113&nbsp;m/s</b>). The "
        "reward plateaus from iteration 200, so this is a <b>local optimum, not an undertrained "
        "run</b>. The interpretation is simple and, we think, general: <b>a reference supplies the "
        "shape of a gait, not its propulsion</b>. Ground reaction has to be discovered, and a tight "
        "corridor around a shape-dominated reward is a landscape in which marching in place is a "
        "perfectly good solution. Widening the corridor (0.30, shape weight 0.5) buys propulsion "
        "(0.321&nbsp;m/s) and throws the ghost away (gmatch 0.537). Neither end of that trade "
        "reaches the incumbent.", st_body0))
    A(P("<b>A corridor curriculum resolves it.</b> Train <i>wide</i> first (discover propulsion), "
        "then <i>tighten</i> onto the ghost once the gait already translates. From scratch, that "
        "policy deploys at <b>0.370&nbsp;m/s over 231.8&nbsp;m with zero falls</b> and gmatch "
        "0.832&mdash;<b>level with the residual-RL incumbent it never saw</b> (0.381&nbsp;m/s), "
        "which is the result that makes the head-to-head above non-circular. The method stands on "
        "its own; warm-starting it from an incumbent is then an <i>upgrade path</i> that exceeds "
        "that incumbent.", st_body0))
    A(P("<b>The honest limit of &lsquo;from scratch&rsquo;.</b> It refers to the <i>policy "
        "weights</i>, not the reference: this ghost is a <i>recording of the incumbent's own "
        "achieved gait</i>, phase-folded and certified. A pipeline independent of the incumbent "
        "end-to-end needs an <b>MPPI-generated</b> ghost, and the MPPI ghost we measured for this "
        "robot replays as a shuffle under the bare PD, so it is not yet a substitute. We state "
        "this rather than let &lsquo;from scratch&rsquo; carry more weight than it earns.", st_body0))
    A(P("<b>The biped does not.</b> This is the paper's central limitation and we state it "
        "without hedging. <b>No from-scratch G1 policy in this project walks durably "
        "free-standing in the deploy engine.</b> The from-scratch walkers we can deploy "
        "topple within seconds of launch. The one G1 gait we are proud of&mdash;the "
        "ghost-shadowing walk of Figure&nbsp;1, which tracks its reference closely and looks "
        "like walking&mdash;runs on the <b>balance harness of Section&nbsp;4</b>, a crane "
        "carrying up to twice the robot's weight. <b>It is a tracking result, not a balance "
        "result, and we do not count it as a deploy walk.</b> Consequently the G1 "
        "contributes no distance to Figure&nbsp;5a and no distance row to Table&nbsp;6. "
        "<b>A durable, free-standing, deployed humanoid walk is an OPEN problem</b>, and it "
        "is the honest headline of this paper that we have not solved it."))
    A(P("<b>What the certificate buys us here is a diagnosis.</b> The failure is <i>not</i> "
        "the reference: the verifier certifies the G1 walk ghost as feasible (base-wrench "
        "residual 4.6% of body weight), and the tracker demonstrably learns to shadow it in "
        "the trainer (&sect;5.1). The reference is therefore <b>exonerated</b>, and the "
        "residual failure is localized to the tracker and the trainer&rarr;deploy pipeline. "
        "That partition&mdash;<i>cert PASS + deploy FAIL</i>&mdash;is the practical value of "
        "a pre-RL gate, and it is what Figure&nbsp;5b plots. It is a diagnosis, not a fix, "
        "and we discuss what we think the remaining wall is in &sect;5.4 and &sect;6."))

    A(figure("fig_generality.png", CW,
        "<b>(a)</b> The durable deploy results: forward distance at zero falls. "
        "<b>The chart is quadrupeds only, and that is the honest scope of our durable "
        "deploy results.</b> The G1 biped has <i>no</i> durable free-standing deploy walk "
        "to plot, and its best gait is harness-supported (&sect;4), so it is deliberately "
        "given no bar rather than represented by a number that is not a free-standing "
        "locomotion result. <b>(b)</b> Instead, the biped appears where it honestly "
        "belongs&mdash;in the <i>partition</i>. <b>Every ghost we train on certifies; the "
        "deploy column is what splits.</b> A cert-PASS/deploy-FAIL row exonerates the "
        "reference and localizes the failure to the tracker or the deployment pipeline; "
        "that partition is the certificate's practical payoff, and the G1 walk, the "
        "sit-stand launch, the hill traverse and the 7&nbsp;cm staircase all sit on the "
        "failing side of it."))

    A(figure("fig_sitstand.png", CW*0.52,
        "<b>The generated, feasible G1 sit-to-stand ghost:</b> pelvis and CoM height "
        "through the seated&rarr;lean&rarr;rise&rarr;stand&rarr;hold motion the generator "
        "discovered (peak 0.73&nbsp;m, torso tilt 2&ndash;5&deg;)&mdash;the upright stand "
        "that hand-drawing plus fourteen from-scratch RL runs could not produce (they bowed "
        "35&ndash;47&deg;). The <i>body</i> of this motion tracks to 2.5&nbsp;cm (a "
        "previously-recorded figure; unlike the sweep of &sect;5.2 it is not regenerable "
        "from a committed artifact, and we mark it as such); the "
        "dead-seated <i>launch</i> is never learned (&sect;5.5)."))

    A(figrow3("omni_omniquad_walk.png", "omni_go2_walk.png", "omni_b2_hill.png", CW*0.315,
        "Shadowing policies and ghosts in OmniSim's Newton physics back-end (a different "
        "engine from the GPU trainer, so a deployed policy crosses a genuine sim-to-sim "
        "gap). The same generator&rarr;verifier&rarr;tracker machinery produced all three "
        "across a 4&times; mass range; only the intent and robot model changed. "
        "<b>Left:</b> a OmniQuad trotting&mdash;<i>deployed</i>, "
        "47.8&nbsp;m, zero falls. <b>Centre:</b> a 16&nbsp;kg Unitree Go2&mdash;"
        "<i>deployed</i>, 86.7&nbsp;m, zero falls. <b>Right:</b> a 60&nbsp;kg Unitree B2 on "
        "a generated hill-walk reference, riding perpendicular to the slope. <b>Note &mdash; the "
        "hill is a certified <i>ghost</i>, not a deployed policy</b>&mdash;the tracker "
        "stalls at the flat&rarr;ramp transition (&sect;5.4, Table&nbsp;6). It is shown here "
        "to illustrate what the generator discovers, not what we deploy."))

    A(P("The pipeline also spans non-legged manipulation. A fixed-base 6-DOF arm "
        "<b>throws</b> a part into a bin at 1.3&nbsp;m&mdash;<i>beyond</i> its ~1.0&nbsp;m "
        "kinematic reach, so no carry-and-place controller could ever succeed&mdash;and "
        "lands it <b>1.5&nbsp;cm from the bin centre</b> (a previously-measured deploy "
        "figure that we did not re-run for this report; the certificate below is "
        "reproducible from the committed artifacts). Here the verifier "
        "earns its keep: it certifies the 1.3&nbsp;m bin as feasible (velocity margin 0.97, "
        "torque margin 0.54) and correctly <b>rejects</b> a 1.8&nbsp;m bin, which lands "
        "41.9&nbsp;cm short&mdash;the throw's reachable-landing set is precisely the "
        "feasibility frontier. We note an honest scope limit: on a fully-actuated arm the "
        "heavy machinery degenerates&mdash;the generator becomes a designed swing and the "
        "tracker becomes classical position control&mdash;so it is the <i>feasibility "
        "certificate</i> that is indispensable here, not the full architecture."))

    A(H2("5.4", "Sim-to-deployment and the feasibility frontier"))
    A(P("Beyond walking, the generator <i>discovers</i> contact-rich motions that have no "
        "analytic reference: <b>get-ups</b> from a belly-flat collapse to a clean stand, for "
        "both OmniQuad and B2 (Figure 8), and hill traversals (Figure 9). Both get-up ghosts "
        "certify; the <b>B2 get-up deploys</b>. The hill ghosts illustrate the planner "
        "reasoning about terrain&mdash;the robot rides perpendicular to the slope and pitches "
        "parallel to it&mdash;cleanly across grades from 4&deg; to 15&deg;, and they certify; "
        "but the <i>tracker</i> stalls at the flat&rarr;ramp transition, so the hill traverse "
        "is <b>not</b> a deployed result (Table 6).", st_body0))

    A(figure("fig_getup.png", CW*0.485,
        "MPPI-discovered get-up ghosts for "
        "OmniQuad and B2: base height rising from a collapsed, belly-flat start to a stand, a "
        "motion with no analytic reference."))

    A(figure("fig_hill.png", CW,
        "Hill-walk ghosts for B2 across grades. <b>Left:</b> centre-of-mass height versus "
        "forward distance for the up&ndash;over&ndash;down traversal; the generator places "
        "the body perpendicular to the local slope so the feet land on the incline. "
        "<b>Right:</b> crest height grows monotonically with grade, as expected. These are "
        "<i>ghosts</i>, and they certify; the tracker stalls at the flat&rarr;ramp "
        "transition, so the hill traverse is not yet deployed (Section 6)."))

    A(P("A recurring, well-characterized limit is a single named wall: the "
        "<i>trainer&rarr;deploy gap</i>, and the certificate's job is to prove the wall is "
        "<i>not</i> the reference. The quadrupeds cross it (&sect;5.3). The G1 walk does "
        "not, and the partition is instructive: the verifier <b>passes</b> the G1 walk ghost "
        "(base-wrench residual 4.6% of body weight), and the tracker shadows it in the "
        "trainer&mdash;so the ghost is exonerated and the residual failure is localized "
        "entirely to the tracker/deployment pipeline.", st_body0))
    A(P("<b>And it is not a physics mismatch either, which is worth establishing because it "
        "is the obvious suspect.</b> We audited the two engines directly. A field-level diff "
        "of the compiled trainer and deploy models shows <b>zero real physics differences</b> "
        "(inertia, gains, ranges, damping, friction and solver options all identical; total "
        "mass conserved). A deterministic open-loop probe run through <i>both</i> the trainer "
        "and the actual deploy binary agrees to <b>~10<sup>&minus;5</sup>&nbsp;rad</b> when "
        "the base is welded&mdash;machine precision. Unweld the base and the two diverge "
        "anyway: trajectories match for roughly <b>0.3&nbsp;s and then separate "
        "exponentially, e-folding every ~0.27&nbsp;s</b>. That is the signature of "
        "<b>chaos, not a bug</b>: a free biped is an inverted pendulum with a positive "
        "Lyapunov exponent, so bit-level round-off is amplified into a divergent trajectory "
        "no matter how well the models match. Chaos cannot be driven to zero for a free "
        "base&mdash;the welded lane matches precisely <i>because</i> welding removes the "
        "instability. <b>The implication is that trainer&rarr;deploy parity is the wrong "
        "target for a biped:</b> the tracker must be robust to a divergence that is "
        "physically guaranteed, not trained against a mismatch that can be eliminated. "
        "That is a diagnosis, not a fix, and the fix is open."))

    A(figure("fig_negative.png", CW*0.60,
        "The headline negative result: peak pelvis height achieved on the dead-seated "
        "sit-to-stand <i>launch</i>. Both 21 reward-RL runs and the DAgger/MPC distillation "
        "stall at the &asymp;0.60&nbsp;m statically-stable crouch; the lookahead MPC reaches "
        "the full 0.74&nbsp;m stand. A feasible reference plus reactive tracking is not "
        "enough."))

    A(H2("5.5", "The boundary: a certified ghost is necessary but not sufficient"))
    A(P("Our sharpest results are where the method <i>stops</i> working&mdash;and why. We "
        "give two, because a single failure invites the reply “you under-trained it,” and "
        "two independent failures of two different <i>learning paradigms</i> on two "
        "different <i>motions</i> do not.", st_body0))
    A(P("<b>Failure 1 &mdash; the dead-seated launch.</b> On the "
        "G1 sit-to-stand we have a <i>certified feasible</i> ghost in hand, and the body of "
        "the motion tracks to 2.5&nbsp;cm. Yet the <b>dead-seated push-off-the-chair "
        "launch</b> (the first ~0.2&nbsp;s) is never learned. Across <b>21 reward-RL "
        "runs</b> spanning every lever&mdash;hand-drawn versus feasible ghost; uniform "
        "versus concentrated reverse curriculum; 4096 versus 16384 environments; pose-only "
        "versus velocity tracking; open-loop and hybrid launch assistance&mdash;the policy "
        "reaches a ~0.60&nbsp;m crouch and refuses to extend (Figure 10). Quadrupling the "
        "environment count changed nothing, so this is a <i>structural</i> optimization "
        "problem, not an exploration-amount one."))
    A(P("We then built the principled fix: distill the generator's MPC&mdash;which "
        "<i>does</i> execute the launch closed-loop to the full 0.74&nbsp;m stand&mdash;into "
        "a reactive policy via DAgger, solving two real sub-problems en route (a residual "
        "anchored on the achieved joint trajectory rather than coarse absolute targets, and "
        "a discover-at-high-noise / track-at-low-noise scheme yielding clean labels at loss "
        "&asymp;10<sup>&minus;3</sup>). It <b>hits the same wall</b>: across a full "
        "12-iteration run the distilled policy reaches a 0.62&ndash;0.67 peak then topples "
        "(tilt 60&ndash;117&deg;), with no improvement across iterations, mirroring the 21 "
        "reward-RL runs. <b>Two different learning paradigms, one identical failure.</b>"))
    A(P("The explanation sharpens the thesis. Completing the stand requires a "
        "<i>predictive commitment to a temporarily-unstable extension</i>: the centre of "
        "mass must briefly pass <i>outside</i> the foot support on the way up. A lookahead "
        "MPC makes this commitment because it sees the stable stand twenty steps ahead; a "
        "purely <b>reactive</b> learned policy will not, because small errors on the "
        "unstable extension compound faster than feedback corrects, so both "
        "BC-distillation and reward-RL retreat to the deepest <i>statically</i> stable "
        "crouch."))

    A(P("<b>Failure 2 &mdash; the staircase that certifies and cannot be climbed.</b> "
        "This is the sharpest evidence we can offer <i>against our own certificate's "
        "sufficiency</i>, and we report it because a reviewer would rightly hunt for it. "
        "We composed a walk&rarr;climb ghost for a 7&nbsp;cm-riser staircase. It "
        "<b>passes every feasibility gate we have</b>&mdash;the contact-wrench certificate, "
        "the full ghost-validator suite, and a solved-provenance check confirming the "
        "reference was produced by the planner rather than hand-drawn. By every test this "
        "paper proposes, it is a good reference. <b>And no policy climbs it.</b> In the "
        "trainer the tracker plateaus at roughly <b>two of the five steps</b> across every "
        "lever we tried; in deployment the policy <b>refuses the first riser</b> outright. "
        "The wall is <i>propulsion</i>&mdash;not balance, not timing, not reference "
        "quality. (What does work is a smaller staircase: at a <b>3&nbsp;cm</b> riser the "
        "G1 climbs a five-tread flight with real foot placements and no vertical harness "
        "assist&mdash;the legs do 100% of the lifting. At 4&nbsp;cm it manages about two "
        "steps; at 5&nbsp;cm, none. The 3&nbsp;cm figure is a <i>measured ceiling</i> for "
        "this robot's foot geometry, not a chosen setting.)"))

    A(P("<b>What these two failures jointly establish.</b> A certified ghost is "
        "<b>necessary but not sufficient</b>&mdash;and the insufficiency has (at least) two "
        "distinct causes, which the certificate cannot currently distinguish. The sit-stand "
        "shows that a <i>reactive</i> policy will not make a predictive commitment to a "
        "temporarily-unstable extension. The staircase shows that a reference can be "
        "<i>dynamically supportable at every instant</i>&mdash;which is exactly and only "
        "what a per-step contact-wrench program tests&mdash;and still lie outside what the "
        "policy can <i>discover</i> and <i>drive</i> under a bounded residual. Per-step "
        "feasibility is not the same property as <i>learnable under this tracker</i>, and "
        "our certificate tests the former. The paper-level statement: <i>a "
        "dynamically-feasible reference plus tracking is necessary but not sufficient; the "
        "missing ingredients are predictive closed-loop commitment and sufficient control "
        "authority, neither of which a per-step feasibility test can see.</i> Closing that "
        "gap&mdash;a certificate that predicts <i>trainability</i>, not merely "
        "<i>supportability</i>&mdash;is the most valuable thing we could do next."))

    E.extend(make_table(
        ["Robot", "Motion", "Deploy result", "Falls", "Status"],
        [["G1", "human-gait walk<br/><i>(free-standing)</i>",
          Paragraph("<b>no durable free-standing deploy walk.</b> From-scratch walkers "
                    "topple within seconds", st_tcell),
          "&mdash;", Paragraph('<b>OPEN</b>', st_tcellb)],
         ["G1", "human-gait walk<br/><i>(harness-supported)</i>",
          Paragraph("tracks its ghost closely, but <b>on a &asymp;2&times;-body-weight "
                    "balance crane</b> (&sect;4) &mdash; a tracking result, <b>not</b> a "
                    "balance result", st_tcell),
          "&mdash;", Paragraph('<b>not a deploy claim</b>', st_tcellb)],
         ["G1", "sit-to-stand (body)", "body tracks the ghost to 2.5 cm", "&mdash;",
          Paragraph('<b>open (launch)</b>', st_tcellb)],
         ["G1", "stair climb, 3 cm riser",
          "climbs a 5-tread flight; legs do 100% of the vertical lift", "0",
          "deployed (at 3 cm)"],
         ["G1", "stair climb, 7 cm riser",
          Paragraph("ghost <b>certifies</b>; trainer plateaus at ~2 of 5 steps; deploy "
                    "refuses the first riser", st_tcell),
          "&mdash;", Paragraph('<b>blocked (propulsion)</b>', st_tcellb)],
         ["OmniQuad", "straight walk (vel.-cond.)<br/><i>analytic trot ref</i>",
          "47.8 m, 0.32 m/s", "0", Paragraph("deployed<br/><i>(baseline, not a<br/>certified ghost)</i>", st_tcell)],
         ["Go2", "walk<br/><i>analytic trot ref</i>", "86.7 m, 0.38 m/s", "0",
          Paragraph("deployed<br/><i>(baseline, not a<br/>certified ghost)</i>", st_tcell)],
         ["B2", "walk<br/><i>analytic trot ref</i>",
          Paragraph("110.7 m, 0.49 m/s <i>(stiffness unreconciled &mdash; see &sect;5.3)</i>", st_tcell),
          "0", Paragraph("deployed<br/><i>(baseline; numbers<br/>flagged)</i>", st_tcell)],
         ["Go2", Paragraph("<b>walk, CERTIFIED GHOST</b><br/><i>(head-to-head vs the "
                           "baseline above)</i>", st_tcell),
          Paragraph("<b>0.429 m/s vs 0.381</b> (+12.6%); lateral drift <b>0.05 m vs 0.26 m</b> "
                    "(5&times;); 3&times;240 s interleaved, same world", st_tcell),
          "0", Paragraph('<b>deployed<br/>(Shadowing)</b>', st_tcellb)],
         ["B2", "get-up (lying&rarr;stand)", "rises to a stand", "0", "deployed"],
         ["B2 / OmniQuad", "hill walk", "tracker stalls at the flat&rarr;ramp transition", "&mdash;",
          "blocked (tracker)"],
         ["6-DOF arm", "toss-to-place", "1.5 cm from target @ 1.3 m (1.8 m correctly rejected)",
          "&mdash;", "deployed (PASS)"]],
        [CW*0.09, CW*0.22, CW*0.40, CW*0.09, CW*0.20],
        "Generality and honest status across robots and motions. “Deployed” means a policy "
        "crosses the trainer&rarr;Newton-deploy gap with zero falls over the stated horizon; "
        "the quadruped distances are what each run reached inside its measurement window "
        "while still upright and advancing (lower bounds, not fall points). <b>Five rows are "
        "deliberately not successes</b>: the free-standing G1 walk (<b>OPEN</b>), the "
        "harness-supported G1 walk (a <i>tracking</i> result on a &asymp;2&times;-body-weight "
        "crane&mdash;<b>we do not count it as a deploy walk</b>, &sect;4), the sit-stand "
        "launch (&sect;5.5), the 7&nbsp;cm staircase (certified yet unclimbable, &sect;5.5), "
        "and the steep hill (tracker-limited). <b>No <i>distance</i> in this table "
        "is harness-assisted, and the G1 contributes none.</b> All deploy results are "
        "simulation-to-simulation; none is on physical hardware (&sect;2.1).",
        fontsize=6.6, align={3: "CENTER", 4: "CENTER"}))

    # ================= 6. Discussion =================
    A(H1("6", "Discussion and Limitations"))
    A(P("<b>Where the architecture is indispensable&mdash;and where it is not.</b> "
        "Shadowing earns its full weight exactly when a motion (i) needs an optimizer to "
        "<i>discover</i> it and (ii) cannot be tracked open-loop because it is "
        "underactuated or contact-rich. Legged locomotion, get-ups, and dynamic "
        "manipulation that crosses a flight or release phase are the sweet spot. On a "
        "fully-actuated manipulator that can be position-controlled open-loop, the "
        "generator degenerates to designed playback and the tracker to classical control; "
        "there, the verifier&mdash;the feasibility certificate&mdash;is the component that "
        "still pays for itself, by drawing the reachable frontier <i>before</i> deployment.", st_body0))
    A(P("<b>The deployment gap is one wall, not many&mdash;and on the biped it is still "
        "standing.</b> Across every motion the residual difficulty concentrates in the "
        "trainer&rarr;deploy gap. Contact-stiffness randomization crosses its <i>stability</i> "
        "face, which is why the quadrupeds deploy durably. Its <i>durability</i> face does "
        "not yield to the same tools: on the G1, with the ghost certified and the physics "
        "audited to machine precision, the free-base trajectories still diverge chaotically "
        "(&sect;5.4) and no from-scratch policy walks durably free-standing. Reward shaping, "
        "residual-capping, and wider randomization did not close it. We name this wall "
        "explicitly rather than reporting around it&mdash;<b>the honest headline of this "
        "paper is that Shadowing deploys quadrupeds durably and does not deploy a "
        "durable free-standing biped walk</b>."))
    A(P("<b>One lever we <i>did</i> pull, and what it taught us.</b> The obvious fix for a "
        "trainer&rarr;deploy gap is to make the trainer <i>be</i> the deploy engine&mdash;one "
        "engine, one physics spec, so drift has nowhere to accumulate. We built that, and it "
        "does produce a durable, good-looking G1 walk that tracks its ghost closely. <b>But "
        "it does so on the balance harness</b> (&sect;4), and we have not been able to anneal "
        "the harness away. So the lesson is not “parity fixes the biped”: as &sect;5.4 "
        "argues, on a chaotic free base <i>exact</i> parity is not even attainable, and "
        "chasing it is the wrong target. The remaining gap is the tracker's robustness to a "
        "divergence that is physically guaranteed&mdash;and, on the evidence of the "
        "harness, its ability to produce enough <i>balance authority</i> without one."))
    A(P("<b>The certificate has real limits, and we tested it adversarially rather than "
        "trusting it.</b> A gate is only worth having if it can say <i>no</i>, so we tried "
        "to break it. Early on it was breakable: an initial get-up branch discarded every "
        "judged frame (a get-up is entirely partial-stance) and so returned a vacuous PASS "
        "on an <i>empty</i> support set&mdash;an 8&nbsp;Hz base-shake and a 3&nbsp;m/s drift "
        "grafted onto real get-up joints both sailed through. The hardened certificate "
        "rejects both, and it now <b>abstains</b> (returning INDETERMINATE, never a vacuous "
        "PASS) when too few support frames can be reconstructed. Against a battery of "
        "adversarial negative controls&mdash;levitating twins, a frozen-apex jump, a body "
        "held 0.8&nbsp;m above its own feet, a robot the certificate was never tuned "
        "on&mdash;it correctly FAILs every one with the same unmistakable signature (a base "
        "residual of ~100% of body weight: force required against nothing). Two honest "
        "weaknesses remain. <b>The scalar score is only weakly informative:</b> it pins to "
        "zero whenever a single DOF rides its torque limit, so genuinely feasible ghosts can "
        "score ~0 yet PASS&mdash;the same saturation artifact that produces the tied zeros in "
        "&sect;5.2. And it <b>over-reports on the jerkiest transients</b> (a get-up can read "
        "&gt;100% even with real contacts) because of finite-difference acceleration noise at "
        "impacts. Trust the verdict; treat the scalar and the friction/torque margins as soft "
        "evidence. Tightening the certificate on impulsive motions&mdash;and, more "
        "importantly, extending it from <i>supportability</i> to <i>trainability</i> "
        "(&sect;5.5)&mdash;is the future work we would prioritize."))
    A(P("<b>The open problems are informative.</b> The sit-stand launch tells us that the "
        "clean “planning describes, control solves” split has a boundary at unstable "
        "contact-rich <i>initiations</i>, where the controller itself must be predictive. "
        "The most promising untried lever there is a hybrid: warm-start PPO from the DAgger "
        "policy&mdash;which already <i>commits</i> to the rise to 0.62, escaping the "
        "stay-seated optimum&mdash;and let reward push it through; or run the MPC "
        "in-the-loop at deploy. Both keep the spirit of the method: let a predictive "
        "planner own the unstable commitment, and let learning own the robustness. The "
        "7&nbsp;cm staircase tells us something we find more uncomfortable and more "
        "useful&mdash;that our certificate answers the wrong question. It asks <i>“is this "
        "reference physically supportable at every instant?”</i> when what a practitioner "
        "needs to know is <i>“will my tracker learn it?”</i> Those came apart, and closing "
        "the distance between them is the research programme this paper actually opens."))

    # ================= 7. Conclusion =================
    A(H1("7", "Conclusion"))
    A(P("We presented Shadowing, a recipe that makes the dynamic feasibility of a reference "
        "motion a first-class, certified property and then learns a robust policy to "
        "shadow it into deployment. A controlled ablation isolates feasibility&mdash;not the "
        "RL&mdash;as the binding constraint on <i>learning</i>; a graded eight-point sweep "
        "shows the certificate score separates learnable references from unlearnable ones "
        "at the feasibility cliff (&rho;=+0.94, with the caveats of &sect;5.2); and a single "
        "tracker carries references across a sim-to-sim deploy gap on five robots and "
        "six motion classes&mdash;durably on the quadrupeds (OmniQuad 47.8&nbsp;m, Go2 "
        "86.7&nbsp;m, B2 110.7&nbsp;m, zero falls) and on the manipulator. Those three "
        "quadruped walks track an <i>analytic</i> trot, the degenerate case of the pipeline; "
        "where we replaced it with a <i>certified, generated</i> ghost and compared the two "
        "head to head on one robot, the certified ghost won (Go2: +12.6% speed, 5&times; less "
        "lateral drift, zero falls both sides). We claim "
        "<i>placement, not invention</i>: the contact-wrench program is classical, and our "
        "contribution is to run it, RL-independently, <i>before</i> the training "
        "run&mdash;which turns an ambiguous “the policy will not learn” into a decisive "
        "statement about the reference.", st_body0))
    A(P("We close on the boundaries, because they are the most useful part of this report. "
        "<b>A certified reference is necessary but not sufficient.</b> The dead-seated "
        "sit-to-stand launch defeats reward-RL and MPC-distillation alike, because "
        "completing it needs a predictive commitment to a temporarily-unstable extension "
        "that a reactive policy will not make. And a 7&nbsp;cm staircase <b>passes every "
        "feasibility gate we have and is still not climbed by any policy we can "
        "train</b>&mdash;so per-step supportability, which is all our certificate tests, is "
        "not the same property as trainability. <b>On the biped, we state the limit "
        "plainly: no from-scratch G1 policy walks durably free-standing in our deploy "
        "engine, and the G1 gait we are proudest of runs on a crane carrying twice the "
        "robot's weight&mdash;a tracking result, not a balance result.</b> A durable, "
        "free-standing, deployed humanoid walk remains open work. We would rather publish "
        "that sentence than a number we cannot stand behind."))

    # ================= references =================
    A(H1("", "References"))
    refs = [
        # --- [1-4] imitation / reference-tracking RL ---
        "X. B. Peng, P. Abbeel, S. Levine, and M. van de Panne. DeepMimic: "
        "Example-Guided Deep Reinforcement Learning of Physics-Based Character Skills. "
        "<i>ACM Transactions on Graphics</i>, 2018. arXiv:1804.02717.",
        "X. B. Peng, Z. Ma, P. Abbeel, S. Levine, and A. Kanazawa. AMP: Adversarial "
        "Motion Priors for Stylized Physics-Based Character Control. <i>ACM TOG</i>, 2021. "
        "arXiv:2104.02180.",
        "Z. Luo, J. Cao, K. Kitani, and W. Xu. Perpetual Humanoid Control for Real-Time "
        "Simulated Avatars (PHC). <i>ICCV</i>, 2023.",
        "C. Tessler, Y. Guo, O. Nabati, G. Chechik, and X. B. Peng. MaskedMimic: Unified "
        "Physics-Based Character Control Through Masked Motion Inpainting. <i>ACM TOG</i>, 2024.",
        # --- [5-9] sim-to-real humanoids + the prior pre-training gates ---
        "T. He et al. Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation (H2O). "
        "<i>IROS</i>, 2024. arXiv:2403.04436.",
        "T. He et al. OmniH2O: Universal and Dexterous Human-to-Humanoid Whole-Body "
        "Teleoperation and Learning. <i>CoRL</i>, 2024. arXiv:2406.08858.",
        "X. Cheng, Y. Ji, J. Chen, R. Yang, G. Yang, and X. Wang. Expressive Whole-Body "
        "Control for Humanoid Robots (ExBody). <i>RSS</i>, 2024. arXiv:2402.16796.",
        "Z. Fu, Q. Zhao, Q. Wu, G. Wetzstein, and C. Finn. HumanPlus: Humanoid Shadowing "
        "and Imitation from Humans. <i>CoRL</i>, 2024. arXiv:2406.10454.",
        "KungFuBot / PBHC: Physics-Based Humanoid Whole-Body Control for Learning "
        "Highly-Dynamic Skills. 2025. arXiv:2506.12851. "
        "<i>(The closest prior RL-independent pre-training feasibility gate; see &sect;2.1.)</i>",
        # --- [10-13] the classical feasibility literature our certificate comes from ---
        "H. Hirukawa, S. Hattori, K. Harada, S. Kajita, K. Kaneko, F. Kanehiro, K. Fujiwara, "
        "and M. Morisawa. A Universal Stability Criterion of the Foot Contact of Legged "
        "Robots &mdash; Adios ZMP. <i>ICRA</i>, 2006.",
        "S. Caron, Q.-C. Pham, and Y. Nakamura. Stability of Surface Contacts for Humanoid "
        "Robots: Closed-Form Formulae of the Contact Wrench Cone for Rectangular Support "
        "Areas. <i>ICRA</i>, 2015. arXiv:1501.04719.",
        "H. Dai, A. Valenzuela, and R. Tedrake. Whole-Body Motion Planning with Centroidal "
        "Dynamics and Full Kinematics. <i>IEEE-RAS Humanoids</i>, 2014.",
        "P.-B. Wieber. Trajectory Free Linear Model Predictive Control for Stable Walking "
        "in the Presence of Strong Perturbations. <i>IEEE-RAS Humanoids</i>, 2006. "
        "<i>(ZMP-feasible &ne; stable.)</i>",
        # --- [14-18] trajectory optimization, predictive control, tracking-from-TO ---
        "C. Mastalli et al. Crocoddyl: An Efficient and Versatile Framework for "
        "Multi-Contact Optimal Control. <i>ICRA</i>, 2020.",
        "A. W. Winkler, C. D. Bellicoso, M. Hutter, and J. Buchli. Gait and Trajectory "
        "Optimization for Legged Systems Through Phase-Based End-Effector Parameterization "
        "(TOWR). <i>IEEE RA-L</i>, 2018.",
        "T. Howell, N. Gileadi, S. Tunyasuvunakool, K. Zakka, T. Erez, and Y. Tassa. "
        "Predictive Sampling: Real-Time Behaviour Synthesis with MuJoCo MPC. 2022. "
        "arXiv:2212.00541.",
        "G. Williams, P. Drews, B. Goldfain, J. M. Rehg, and E. A. Theodorou. "
        "Information-Theoretic Model Predictive Control (MPPI). <i>IEEE T-RO</i>, 2017.",
        "Z. Li, X. Cheng, X. B. Peng, P. Abbeel, S. Levine, G. Berseth, and K. Sreenath. "
        "Reinforcement Learning for Robust Parameterized Locomotion Control of Bipedal "
        "Robots (Cassie). <i>ICRA</i>, 2021. arXiv:2103.14295.",
        # --- [19-23] composition, distillation, capturability, engine ---
        "R. R. Burridge, A. A. Rizzi, and D. E. Koditschek. Sequential Composition of "
        "Dynamically Dexterous Robot Behaviours. <i>IJRR</i>, 1999.",
        "R. Tedrake, I. R. Manchester, M. Tobenkin, and J. W. Roberts. LQR-Trees: "
        "Feedback Motion Planning via Sums-of-Squares Verification. <i>IJRR</i>, 2010.",
        "S. Ross, G. Gordon, and D. Bagnell. A Reduction of Imitation Learning and "
        "Structured Prediction to No-Regret Online Learning (DAgger). <i>AISTATS</i>, 2011.",
        "T. Koolen, T. de Boer, J. Rebula, A. Goswami, and J. Pratt. Capturability-Based "
        "Analysis and Control of Legged Locomotion. <i>IJRR</i>, 2012.",
        "E. Todorov, T. Erez, and Y. Tassa. MuJoCo: A Physics Engine for Model-Based "
        "Control. <i>IROS</i>, 2012.",
        # --- [24-26] quadruped sim-to-real baselines (they cross to hardware; we do not) ---
        "J. Hwangbo, J. Lee, A. Dosovitskiy, D. Bellicoso, V. Tsounis, V. Koltun, and "
        "M. Hutter. Learning Agile and Dynamic Motor Skills for Legged Robots (ANYmal). "
        "<i>Science Robotics</i>, 2019. arXiv:1901.08652.",
        "T. Miki, J. Lee, J. Hwangbo, L. Wellhausen, V. Koltun, and M. Hutter. Learning "
        "Robust Perceptive Locomotion for Quadrupedal Robots in the Wild. "
        "<i>Science Robotics</i>, 2022. arXiv:2201.08117.",
        "I. M. A. Nahrendra, B. Yu, and H. Myung. DreamWaQ: Learning Robust Quadrupedal "
        "Locomotion with Implicit Terrain Imagination via Deep Reinforcement Learning. "
        "<i>ICRA</i>, 2023. arXiv:2301.10602.",
    ]
    for i, r in enumerate(refs, 1):
        A(Paragraph(f"[{i}]&nbsp;&nbsp;{r}", st_ref))

    A(Spacer(1, 6))
    A(HRFlowable(width="100%", thickness=0.5, color=RULE, spaceBefore=2, spaceAfter=4))
    A(Paragraph(
        "<i>Reproducibility.</i> The pipeline, ghosts, trainers, and deploy scripts live "
        "in the OmniSim repository under <font name='Courier'>projects/policies/research/shadowing/</font> "
        "(generator, verifier, per-motion intents) and "
        "<font name='Courier'>projects/policies/research/training/</font> (the trackers). Every figure in "
        "this report is generated directly from the recorded ghost trajectories "
        "(<font name='Courier'>projects/policies/research/shadowing/ghosts/*.npz</font>) and the deploy "
        "logs.", S("repro", parent=st_ref, firstLineIndent=0, leftIndent=0)))

    doc.build(E)
    print("BUILT", OUT)

if __name__ == "__main__":
    build()
