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

"""Tier 0: parse an operator utterance into typed frames, or decline to.

WHY THIS REPLACED THE KEYWORD LADDER
------------------------------------
The ladder -- the `intent_router` module plus each bridge's own
`IntentRouter.dispatch`, all deleted on 2026-09-22 -- was a first-match-wins
chain of `re.search` calls over bare keywords. Measured on 178 unique real
operator utterances exported from production on 2026-09-19:

    a rule fired                                54%
    ...but 70% of those hits explained under a quarter of the utterance
    false positives on non-command prose       100%  (9 of 9)
    utterances matching two rules at once       20%  (ladder ORDER decides)
    net, counting only hits that explain >=25%  ~16%

The failures are not near-misses, they are confident wrong actions:

    "stop what you are doing and wait until I tell you to continue"
        -> RESUME_RE matches the word `continue`, and RESUME_RE is first,
           so a STOP order RESUMES autonomy.
    "how many times have you had to stop for something in your way?"
        -> halts the robot.
    "put the block back on the grey table"  /  "Can you pick it back up?"
        -> both drive in REVERSE, on the word `back`.
    "how much charge is left in your battery?"
        -> spins in place, on the word `left`.

Every one of those is a keyword appearing somewhere in a sentence that means
something else. The defect is structural: **matching a substring is not
interpreting an utterance**, and a ladder that only counts hits cannot tell
the two apart.

THE FIVE THINGS THIS DOES DIFFERENTLY
-------------------------------------
1. **Clause structure before keywords.** The utterance is split into clauses,
   and a clause introduced by `until / when / after / once / unless / if /
   next time` is a TRIGGER, not an order to execute now. That single rule is
   what stops `...until I tell you to continue` from resuming.

2. **Question form is checked first.** `how many carts have you delivered?`
   is a QUERY. No amount of `stop`/`left`/`back` inside a question can reach a
   motor, because questions never produce a motion frame.

3. **A consumption gate.** A frame is only accepted if its matched span
   explains most of the clause once politeness is stripped. Matching `back`
   inside `put the block back on the grey table` explains 4 characters of 38,
   so it is rejected rather than executed.

4. **Specificity, not source order.** Every rule is tried and the LONGEST
   match wins, so adding a rule cannot silently shadow another one. The
   ladder's ordering hazard disappears with the ordering.

5. **It says what it could not do.** The result carries an intent class, the
   residue it failed to explain, and a human-readable reason. Declining is a
   first-class outcome: AMBIGUOUS (`put it over there` -- which `it`?) routes
   to a clarifying question, CONVERSATION routes to the model. Neither one
   pretends to be a command.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not answer. It returns frames; the bridge executes them. It does not
ground references -- resolving "the cart in spot 3" needs live state, so it
reports AMBIGUOUS with the phrase it could not resolve and lets the caller
resolve or ask. And it has no model in it: everything here is deterministic
and runs in tens of microseconds, so an offline bridge behaves identically to
a connected one.

USAGE
-----
    from omnisim_bridges.interpret import interpret, MOBILE

    r = interpret("turn left 90 degrees, then drive forward 1 metre", MOBILE)
    r.intent            # "command"
    [f.tool for f in r.frames]      # ["turn", "drive_forward"]
    r.frames[0].args                # {"angle_rad": 1.5707963267948966}

    interpret("how much charge is left in your battery?", MOBILE).intent
    # "query"  -- not a spin
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

__all__ = [
    "interpret", "Interpretation", "Frame",
    "MOBILE", "ARM", "QUADRUPED", "DRONE", "ANY",
    "COMMAND", "QUERY", "DEFERRED", "CONSTRAINT", "MEMORY",
    "CONVERSATION", "AMBIGUOUS", "EMPTY",
]

# ── Surfaces ────────────────────────────────────────────────────────
# Which bridge is asking. A rule only fires on a surface that can do it, so
# "sit" never reaches a wheeled tug and "drive forward" never reaches an arm.
MOBILE = "mobile"
ARM = "arm"
QUADRUPED = "quadruped"
DRONE = "drone"
ANY = "any"

# ── Intent classes ──────────────────────────────────────────────────
COMMAND = "command"            # execute now; frames are filled and valid
QUERY = "query"                # read-only question about state or history
DEFERRED = "deferred"          # do X when Y -- goes to intents.schedule
CONSTRAINT = "constraint"      # standing restriction -- intents.set_constraint
MEMORY = "memory"              # a fact or policy to remember
CONVERSATION = "conversation"  # only a model can answer this
AMBIGUOUS = "ambiguous"        # understood the verb, not the referent -> ask
EMPTY = "empty"

_NUM = r"[-+]?\d+(?:\.\d+)?"
_LEN_UNIT = r"(?:m|meter|metre|meters|metres|cm|centimeter|centimetre|centimeters|centimetres)"
_ANG_UNIT = r"(?:deg|degree|degrees|rad|radian|radians)"


@dataclass
class Frame:
    """One typed, validated call the bridge can execute."""
    tool: str
    args: Dict[str, Any] = field(default_factory=dict)
    span: Tuple[int, int] = (0, 0)
    source: str = ""            # the clause this came from
    rule: str = ""              # which rule produced it, for explainability

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Frame({self.tool}, {self.args})"


@dataclass
class Interpretation:
    """What tier 0 made of one utterance, including what it could not make."""
    intent: str
    frames: List[Frame] = field(default_factory=list)
    reason: str = ""
    residue: str = ""
    tier: int = 0
    confidence: float = 0.0
    trigger: Optional[str] = None   # DEFERRED / CONSTRAINT: the condition text
    # The utterance this came from. The safety gate needs it: its rules
    # are about the SENTENCE versus the frame, and it must work on
    # frames a model produced too, where there is no parse to inspect.
    text: str = ""

    @property
    def executable(self) -> bool:
        """True only for frames that may be dispatched without asking first."""
        return self.intent == COMMAND and bool(self.frames)

    def explain(self) -> str:
        """A sentence saying what happened. Free, because the parse knows."""
        if self.intent == COMMAND:
            calls = ", ".join(f"{f.tool}({_fmt(f.args)})" for f in self.frames)
            return f"parsed {len(self.frames)} command(s): {calls}"
        if self.residue:
            return f"{self.intent}: {self.reason} (unexplained: {self.residue!r})"
        return f"{self.intent}: {self.reason}"


def _fmt(args: Dict[str, Any]) -> str:
    return ", ".join(f"{k}={v!r}" for k, v in args.items())


# ── Normalisation ───────────────────────────────────────────────────
# Production text carries smart quotes, em dashes and mojibake from three
# different clients. Normalise once so no rule has to care.
_DASHES = dict.fromkeys(map(ord, "‐‑‒–—―−"), "-")
_QUOTES = {ord("‘"): "'", ord("’"): "'",
           ord("“"): '"', ord("”"): '"'}


# Spoken quantities. Without these, "drive forward two metres" matched the
# bare `drive forward` rule with no distance, and the executor's default put
# the robot 1 m down the floor -- a SILENT GUESS, which is worse than a
# refusal because nothing in the reply says a number was invented. Measured
# by commandbench unit-3/unit-4, where "half a metre" and "two metres" both
# travelled exactly 1.000 m.
_WORD_NUM = {
    "one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
    "six": "6", "seven": "7", "eight": "8", "nine": "9", "ten": "10",
    "eleven": "11", "twelve": "12", "fifteen": "15", "twenty": "20",
    "thirty": "30", "forty": "40", "fifty": "50", "sixty": "60",
    "ninety": "90", "hundred": "100",
}
# Longest first, so "three quarters" is not eaten by "three".
_FRACTIONS = [
    ("three quarters of a", "0.75"), ("three quarters", "0.75"),
    ("a quarter of a", "0.25"), ("a quarter", "0.25"), ("quarter of a", "0.25"),
    ("a half of a", "0.5"), ("half of a", "0.5"), ("half a", "0.5"),
    ("a half", "0.5"), ("half", "0.5"),
]
_UNIT_WORDS = (r"(?=\s*(?:m|meter|metre|meters|metres|cm|centimetres?|"
               r"centimeters?|deg|degrees?|rad|radians?|turn|turns|"
               r"revolution|revolutions)\b)")


def _digits(s: str) -> str:
    for phrase, value in _FRACTIONS:
        s = re.sub(r"\b" + re.escape(phrase) + r"\b", value, s, flags=re.I)
    for word, value in _WORD_NUM.items():
        s = re.sub(r"\b" + word + r"\b", value, s, flags=re.I)
    # "a metre" / "an inch" mean one of them -- but only immediately before
    # a unit, so "a cart" is left alone.
    s = re.sub(r"\ban?\b\s+" + _UNIT_WORDS, "1 ", s, flags=re.I)
    return s


# ── Typos, conservatively ───────────────────────────────────────────
# commandbench noise-1/noise-2: "drive foward 1 metre" and "drv fwd 1m"
# were DECLINED. That is the safe failure, but it is still a failure -- an
# operator typing in a hurry is the normal case, not an edge case.
#
# Fuzzy matching is also how a keyword ladder gets its false positives
# back, so this is deliberately narrow:
#   * a closed vocabulary of command words, nothing else;
#   * a token is only corrected if it is NOT already a word we know;
#   * edit distance 1, and only when EXACTLY ONE vocabulary word is that
#     close -- an ambiguous correction is no correction.
# "lift" is in the vocabulary precisely so it is never "corrected" to
# "left", which is one edit away and means something very different.
_VOCAB = {
    "drive", "forward", "forwards", "ahead", "backward", "backwards",
    "reverse", "turn", "rotate", "spin", "left", "right", "around",
    "stop", "halt", "freeze", "pause", "wait", "resume", "continue",
    "metre", "metres", "meter", "meters", "degree", "degrees",
    "centimetre", "centimetres", "radian", "radians",
    "wave", "stand", "sit", "walk", "pick", "place", "lift", "grab",
    "open", "close", "gripper", "hover", "land", "takeoff", "climb",
    "home", "cart", "trolley", "block", "cube", "table",
}
_ABBREV = {
    "drv": "drive", "fwd": "forward", "fward": "forward", "bwd": "backward",
    "bck": "back", "rev": "reverse", "lft": "left", "rgt": "right",
    "rt": "right", "deg": "degrees", "degs": "degrees", "mtr": "metre",
    "mtrs": "metres", "cmd": "command",
}
# ⚠️ Inflections of vocabulary words, which must never be "corrected" INTO
# the base form. "drove" is one edit from "drive", and rewriting it turned
# "you already DROVE forward 2 metres, didn't you?" -- an accusation about
# the past -- into a present-tense order, which the robot obeyed.
_PROTECTED = {
    "have",  # auxiliary in "when you have finished", not a typo for wave
    "drove", "driven", "driving", "turned", "turning", "stopped", "stopping",
    "waved", "waving", "walked", "walking", "picked", "picking", "placed",
    "placing", "opened", "opening", "closed", "closing", "landed", "landing",
    "lifted", "lifting", "grabbed", "grabbing", "moved", "moving", "stood",
    "standing", "sat", "sitting", "went", "gone", "spun", "spinning",
    "climbed", "climbing", "paused", "pausing", "waited", "waiting",
    "resumed", "resuming", "halted", "hovered", "hovering", "reversed",
}


def _within_one_edit(a: str, b: str) -> bool:
    """True when a and b differ by one insert, delete or substitution."""
    la, lb = len(a), len(b)
    if abs(la - lb) > 1:
        return False
    if la == lb:
        return sum(1 for x, y in zip(a, b) if x != y) == 1
    if la > lb:
        a, b, la, lb = b, a, lb, la
    i = 0
    while i < la and a[i] == b[i]:
        i += 1
    return a[i:] == b[i + 1:]


def _despell(s: str) -> str:
    out = []
    for token in re.split(r"(\W+)", s):
        low = token.lower()
        if not low.isalpha() or low in _VOCAB or low in _PROTECTED:
            out.append(token)
            continue
        if low in _ABBREV:
            out.append(_ABBREV[low])
            continue
        if len(low) >= 4:
            near = [w for w in _VOCAB if _within_one_edit(low, w)]
            if len(near) == 1:          # ambiguous means leave it alone
                out.append(near[0])
                continue
        out.append(token)
    return "".join(out)


def _normalise(text: str) -> str:
    s = (text or "").translate(_DASHES).translate(_QUOTES)
    s = s.replace("�", " ")          # replacement char from bad decodes
    s = _digits(s)
    s = _despell(s)
    return re.sub(r"\s+", " ", s).strip()


# Politeness and discourse padding. Stripped only for the CONSUMPTION test --
# never from the text a rule matches against, so "can you stop" still parses.
#
# The adverbial tails matter as much as the politeness: "stop for a second"
# is a stop, and counting "for a second" against the gate is what made the
# first draft reject it. None of these change WHICH tool is called; they only
# stop the denominator from punishing ordinary English.
_FILLER = re.compile(
    r"\b(please|kindly|could you|can you|would you|will you|"
    r"i want you to|i need you to|i'd like you to|go ahead and|go and|"
    r"for me|right now|just now|just|now|ok|okay|alright|"
    r"actually|basically|maybe|perhaps|hey|hi|thanks|thank you|"
    r"at the moment|at this exact moment|if you can|when you can|"
    # adverbial tails
    r"for a (?:second|moment|minute|bit|while)|for now|a (?:second|minute)|"
    r"immediately|at once|straight away|in place|right there|over here|"
    r"with your work|with the work|with it|with that|as normal|as before|"
    r"and hold(?: there| position)?|steady|slowly|carefully|"
    r"again|one more time|the same|exactly|"
    # locatives that add no slot: they mean "from wherever you are"
    r"from your current location|from where you are(?: now)?|"
    r"where you are(?: now)?|from here|in front of you)\b",
    re.IGNORECASE)

# "until I tell you" / "until I say" is not a trigger to schedule -- it is an
# INDEFINITE HOLD, which intents.py already implements precisely because the
# 60 s quiet window auto-resumed and walked the tug back into a forbidden
# cell. Recognising it here is what turns the ladder's worst inversion into
# the correct pair of actions: stop NOW, and do not self-resume.
_HOLD_TRIGGER = re.compile(
    r"\b(?:until|till)\s+(?:i|you)\s+"
    r"(?:tell|say|give|release|let|signal|come back|get back)",
    re.IGNORECASE)


def _core(clause: str) -> str:
    """The clause with padding removed -- the denominator of the gate."""
    s = _FILLER.sub(" ", clause)
    s = re.sub(r"[^\w\s.-]", " ", s)
    return re.sub(r"\s+", " ", s).strip()


# ── Clause splitting ────────────────────────────────────────────────
# Coordinates contain commas ("drive to x 10.3, y -3.0"), so they are masked
# before splitting and restored after. Getting this wrong turns one command
# into two broken halves, which is how "y -3.0" becomes an unparseable clause.
_COORD = re.compile(
    r"\b(?:x\s*[=:]?\s*)?(" + _NUM + r")\s*,\s*(?:y\s*[=:]?\s*)?(" + _NUM + r")\b",
    re.IGNORECASE)
_SPLIT = re.compile(
    r"\s*(?:,|;|\.|:|-{1,2}\s|\band then\b|\bthen\b|\bafter that\b|\band\b)\s+",
    re.IGNORECASE)


def _clauses(text: str) -> List[str]:
    masked: List[str] = []

    def _mask(m: re.Match) -> str:
        masked.append(m.group(0))
        return f"\x00{len(masked) - 1}\x00"

    held = _COORD.sub(_mask, text)
    parts = [p.strip(" .") for p in _SPLIT.split(held)]

    def _restore(s: str) -> str:
        return re.sub(r"\x00(\d+)\x00", lambda m: masked[int(m.group(1))], s)

    return [_restore(p) for p in parts if p.strip()]


# ── Guards, applied before any rule ─────────────────────────────────
# Order matters here and only here, and each guard is a claim about the whole
# utterance rather than about a keyword inside it.

# A leading modal + "you" is a polite imperative ("can you stop"), NOT a
# question -- the single most common way a real operator phrases an order.
_POLITE_IMPERATIVE = re.compile(
    r"^\s*(?:so\s+)?(?:can|could|would|will|please)\s+(?:you\s+)?(?:please\s+)?", re.IGNORECASE)

_QUESTION_HEAD = re.compile(
    r"^\s*(?:so\s+|and\s+|but\s+)?"
    r"(what|where|when|why|how|which|who|whose|"
    r"is|are|was|were|do|does|did|have|has|had|am|"
    r"can|could|will|would|should|any)\b",
    re.IGNORECASE)

_AGGREGATE = re.compile(r"\b(how many|how much|how often|how long|"
                        r"count|total|in total|so far|altogether)\b", re.IGNORECASE)

# Imperative in form, question in effect. "tell me where you are" asks; it
# does not move. The ladder had no way to express the difference.
_TELL_ME = re.compile(
    r"^\s*(?:tell|show|give|let|remind)\s+(?:me|us)\b|"
    r"^\s*(?:confirm|report|describe|list|state)\b|"
    r"^\s*(?:status(?:\s+check)?|sitrep|telemetry)\b",
    re.IGNORECASE)

# Subordinators introduce a TRIGGER. Everything after one is a condition, and
# an imperative inside a condition must never be executed now. This is the
# rule that fixes the RESUME_RE inversion.
_SUBORDINATOR = re.compile(
    r"\b(until|till|when|whenever|once|after|before|unless|if|"
    r"as soon as|next time|while)\b", re.IGNORECASE)

_PROHIBITION = re.compile(
    r"\b(don't|do not|never|must not|mustn't|no longer|stay out of|"
    r"keep out of|steer clear of|avoid)\b", re.IGNORECASE)

# ── Guards added after commandbench measured where the parser ACTED ──
# when it should have held still. Every one of these was a case where the
# robot moved on a sentence that, read whole, was not an order.

# A retraction cancels what came before it. "drive forward 2 metres - no
# wait, make it 1" travelled 2 m (the head won) and would have travelled 3
# if both had run. The tail governs.
_RETRACTION = re.compile(
    r"\b(?:no wait|wait no|actually|"
    r"forget (?:that|it)|scratch that|belay that|ignore that|"
    r"i meant|make it|on second thought(?:s)?|instead)\b",
    re.IGNORECASE)

# ...and a retraction whose tail is a cancellation means DO NOTHING.
_CANCEL_TAIL = re.compile(
    r"\b(?:stay (?:where you are|put)|do nothing|never ?mind|"
    r"hold (?:still|position)|don't|do not|forget it|as you were)\b",
    re.IGNORECASE)

# A bare quantity in the tail re-fills the head's slot: "make it 1 metre".
_REQUANTIFY = re.compile(
    r"(?:make it|i meant|instead)?\s*(" + _NUM + r")\s*"
    r"(" + _LEN_UNIT + r"|" + _ANG_UNIT + r")\b", re.IGNORECASE)

# Asking whether an action is POSSIBLE is not ordering it. Only explicit
# markers count: "can you stop?" must stay an order.
_HYPOTHETICAL = re.compile(
    r"\b(?:in principle|in theory|hypothetically|theoretically|"
    r"if i (?:asked|told|said|wanted)|would you be able|are you able|"
    r"could you ever|is it possible)\b", re.IGNORECASE)

# An order that negates itself. "drive forward 1 metre without moving" is
# not a command with a caveat; it is incoherent, and guessing is wrong.
_SELF_NEGATING = re.compile(
    r"\bwithout\s+(?:moving|driving|turning|going|rotating|budging)\b|"
    r"\bbut\s+(?:don't|do not|never)\b|\bwhile\s+not\s+moving\b",
    re.IGNORECASE)

# "in ten minutes, turn left" is a schedule, not an order for now.
_DELAY = re.compile(
    r"\bin\s+(" + _NUM + r")\s*"
    r"(seconds?|secs?|minutes?|mins?|hours?|hrs?)\b", re.IGNORECASE)

# A destination the parser cannot resolve without remembering the past.
_PAST_PLACE = re.compile(
    r"\b(?:where you (?:started|began|were)|your (?:starting|original) "
    r"(?:point|position|pose|spot)|back to the start)\b", re.IGNORECASE)

# A magnitude no wheeled tug in a room-sized world was plausibly asked for.
# This is a SANITY rail, not a world bound -- the parser does not know the
# arena. commandbench bounds-1 drove 14.4 m on "500 metres" and left it.
# ⚠️ The real fix is a world-aware clamp in the bridge, which does not exist.
MAX_PLAUSIBLE_M = 50.0
MAX_PLAUSIBLE_RAD = 8 * math.pi      # four full turns

_MEMORY_CUE = re.compile(
    r"\b(remember (?:this|that)?|note that|for the (?:whole )?(?:site|shift|day)|"
    r"from now on|for future reference|keep in mind|important for)\b",
    re.IGNORECASE)

# Social pressure and honesty probes. These MUST reach the model: they are
# challenges to the robot's own account of itself, and any deterministic
# answer would be a fabrication.
_SOCIAL = re.compile(
    r"\b(admit (?:it|you)|you'?re (?:making that up|lying)|that is wrong|"
    r"that's wrong|you never |i was watching|i have (?:the )?cctv|"
    r"apologi[sz]e|your (?:log|count) is (?:a )?(?:known )?(?:bug|incorrect|wrong)|"
    # ⚠️ NO trailing \b. These alternatives end in a literal "?", and a word
    # boundary after "?" at end-of-string cannot match -- so the tag-question
    # alternatives never fired. Silent since they were written; exposed when
    # typo correction turned "you already DROVE forward" into a command and
    # the robot drove 2 m answering an accusation.
    r"are you sure|aren'?t you\?|haven'?t you\?|didn'?t you\?|wasn'?t it\?)",
    re.IGNORECASE)

# Bare pronouns with no antecedent in the utterance. The verb is understood,
# the object is not -- which is a question to ask, not a motion to guess.
_DEIXIS = {"it", "that", "this", "them", "those", "these", "there", "here",
           "the other side", "over there"}


@dataclass(frozen=True)
class Rule:
    """One parse rule: a pattern that must match, and the slots it fills."""
    tool: str
    pattern: re.Pattern
    build: Callable[[re.Match], Dict[str, Any]]
    surfaces: Tuple[str, ...] = (ANY,)
    name: str = ""

    def applies(self, surface: str) -> bool:
        # surface=ANY means "this caller can do anything" -- used by the
        # offline evaluation, and by a relay fronting several robots.
        return ANY in self.surfaces or surface == ANY or surface in self.surfaces


def _metres(value: str, unit: Optional[str]) -> float:
    v = float(value)
    if unit and unit.lower().startswith(("cm", "centi")):
        return v / 100.0
    return v


def _radians(value: str, unit: Optional[str]) -> float:
    v = float(value)
    if unit and unit.lower().startswith("rad"):
        return v
    return math.radians(v)


def _sign(verb: Optional[str], direction: Optional[str]) -> int:
    """Direction wins over verb: "drive BACK 2 m" reverses, "reverse 2 m" too."""
    token = (direction or verb or "").lower()
    return -1 if token.startswith(("back", "rev")) else 1


def _obj(raw: Optional[str], default: str = "") -> str:
    """Clean an object slot: "block back" -> "block", "the cube" -> "cube"."""
    s = (raw or default).strip().lower()
    s = re.sub(r"^(?:the|a|an|that|this)\s+", "", s)
    s = re.sub(r"\s+(?:back|again|up|down)$", "", s)
    return s.strip()


def _R(tool: str, pattern: str, build: Callable[[re.Match], Dict[str, Any]],
       surfaces: Sequence[str] = (ANY,), name: str = "") -> Rule:
    return Rule(tool, re.compile(pattern, re.IGNORECASE), build,
                tuple(surfaces), name or tool)


# ── The grammar ─────────────────────────────────────────────────────
# Every rule fills the slots its tool needs. Longest match wins, so ordering
# in this list carries no meaning -- which is the point.
RULES: List[Rule] = [
    # ---- motion, mobile --------------------------------------------
    _R("drive_to",
       r"\b(?:drive|go|move|head|navigate|take yourself|come)\s+(?:to|over to)?\s*"
       r"(?:the\s+)?(?:point\s+|position\s+|coordinates?\s+)?"
       r"x\s*[=:]?\s*(" + _NUM + r")\s*(?:,|\s+and)?\s*y\s*[=:]?\s*(" + _NUM + r")",
       lambda m: {"x": float(m.group(1)), "y": float(m.group(2))},
       [MOBILE], "drive_to_xy"),

    _R("drive_to",
       r"\b(?:drive|go|move|head|navigate)\s+to\s+(?:the\s+)?"
       r"(?:plant|point|spot|position)?\s*(?:at\s+)?"
       r"\(?\s*(" + _NUM + r")\s*,\s*(" + _NUM + r")\s*\)?",
       lambda m: {"x": float(m.group(1)), "y": float(m.group(2))},
       [MOBILE], "drive_to_pair"),

    _R("drive_forward",
       r"\b(drive|go|move|roll|reverse|back)\s*(?:up|off)?\s*"
       r"(?:(forward|forwards|ahead|back|backward|backwards|reverse)\s+)?"
       r"(?:by\s+)?(" + _NUM + r")\s*(" + _LEN_UNIT + r")\b",
       lambda m: {"distance": _metres(m.group(3), m.group(4))
                  * _sign(m.group(1), m.group(2))},
       [MOBILE], "drive_n"),

    _R("drive_forward",
       r"\b(?:drive|go|move|roll)\s+(forward|forwards|ahead)\b(?!\s*" + _NUM + r")",
       lambda m: {"distance": None},
       [MOBILE], "drive_default"),

    _R("drive_forward",
       r"\b(?:reverse|back up|back off|drive back|go back|move back)\b"
       r"(?:\s+(?:by\s+)?(" + _NUM + r")\s*(" + _LEN_UNIT + r"))?",
       lambda m: {"distance": -(_metres(m.group(1), m.group(2))
                                if m.group(1) else 1.0)},
       [MOBILE], "reverse"),

    _R("turn",
       r"\b(?:turn|rotate|spin|yaw)\s*(left|right|clockwise|anticlockwise|"
       r"counter-?clockwise)?\s*(?:by\s+|about\s+)?"
       r"(" + _NUM + r")\s*(" + _ANG_UNIT + r")?",
       lambda m: {"angle_rad": _radians(m.group(2), m.group(3))
                  * (-1 if (m.group(1) or "").lower() in
                     ("right", "clockwise") else 1)},
       [MOBILE, QUADRUPED, DRONE], "turn_n"),

    # "a quarter turn" normalises to "0.25 turn", which the degree/radian
    # rule would read as 0.25 DEGREES. A turn is a unit of its own.
    _R("turn",
       r"\b(?:turn|rotate|spin)\s*(left|right|clockwise|anticlockwise)?\s*"
       r"(?:by\s+)?(" + _NUM + r")\s*(?:turn|turns|revolution|revolutions)\b",
       lambda m: {"angle_rad": float(m.group(2)) * 2 * math.pi
                  * (-1 if (m.group(1) or "").lower() in
                     ("right", "clockwise") else 1)},
       [MOBILE, QUADRUPED, DRONE], "turn_revolutions"),

    _R("turn",
       r"\b(?:turn|rotate|spin|swing)\s+(left|right)\b(?!\s*" + _NUM + r")",
       lambda m: {"angle_rad": math.pi / 2 *
                  (1 if m.group(1).lower() == "left" else -1)},
       [MOBILE, QUADRUPED, DRONE], "turn_dir"),

    _R("turn",
       r"\b(?:turn around|u-?turn|about face|spin around|turn back around)\b",
       lambda m: {"angle_rad": math.pi},
       [MOBILE, QUADRUPED, DRONE], "turn_around"),

    _R("set_velocity",
       r"\b(?:set\s+)?velocity\s+(" + _NUM + r")[,\s]+(" + _NUM + r")",
       lambda m: {"v": float(m.group(1)), "w": float(m.group(2))},
       [MOBILE], "set_velocity"),

    # ---- carts, mobile ---------------------------------------------
    _R("attach_trolley",
       r"\b(?:attach|hook up|hitch|couple|connect|pick up|collect|fetch|grab)\s+"
       r"(?:to\s+)?(?:the\s+)?"
       # An id may be the noun itself (TROLLEY_E) or follow it (cart B).
       # Splitting the noun from its suffix is what produced trolley='_E'.
       r"(?:((?:trolley|cart|wagon)[_-][A-Za-z0-9]+)|"
       r"(?:cart|trolley|wagon)"
       # Without this lookahead the id group swallows the next ordinary word:
       # "the cart out of spot 2" became trolley='out', and "the cart parked
       # in spot 3" became trolley='PARKED'.
       r"(?:\s+(?:called\s+|named\s+|id\s+)?"
       r"(?!(?:out|from|in|into|at|to|over|on|for|with|and|that|which|is|are|"
       r"parked|sitting|standing|waiting|next|back|here|there|now)\b)"
       r"([A-Za-z][A-Za-z0-9_]*))?)"
       # "...out of park spot 2" is a slot, not noise: without capturing it
       # the clause stays half-explained and the gate rejects the whole order.
       r"(?:\s+(?:out of|from|in|at)\s+(?:the\s+)?(?:park\s+)?"
       r"(?:spot|bay|slot|row)\s*(\d+))?",
       lambda m: {"trolley": (m.group(1) or m.group(2) or "").upper() or None,
                  "spot": int(m.group(3)) if m.group(3) else None},
       [MOBILE], "attach_trolley"),

    _R("detach_trolley",
       r"\b(?:detach|unhook|uncouple|drop|release|park|leave)\s+"
       r"(?:the\s+)?(?:cart|trolley|wagon)\b",
       lambda m: {}, [MOBILE], "detach_trolley"),

    # ---- arm --------------------------------------------------------
    # Three phrasings of the same act, because English puts the particle
    # wherever it likes: "pick up the cube" / "pick the cube up" / "pick it
    # back up". The old ladder read the third as DRIVE IN REVERSE.
    _R("pick",
       r"\bpick\s+up\s+(?:the\s+|that\s+|an?\s+)?([a-z][a-z0-9_ ]{1,28}?)"
       r"(?=\s*(?:$|,|\band\b|\boff\b|\bfrom\b|\bthen\b))|"
       r"\bpick\s+(?:the\s+|that\s+|an?\s+)?([a-z][a-z0-9_ ]{1,28}?)\s+up\b|"
       r"\b(?:pick|grasp|grab|lift)\s+(?:the\s+|that\s+|an?\s+)?"
       r"([a-z][a-z0-9_ ]{1,28}?)(?=\s*(?:$|,|\band\b|\boff\b|\bfrom\b|\bthen\b))",
       lambda m: {"object": _obj(m.group(1) or m.group(2) or m.group(3))},
       [ARM], "pick_object"),

    _R("place",
       r"\b(?:place|put|set|drop|move|stack|take|bring|deliver)\s+"
       r"(?:the\s+|it\s+|that\s+|an?\s+)?"
       r"([a-z][a-z0-9_ ]{1,28}?)?\s*"
       r"(?:on top of|onto|on|into|in|at|over to|over|to)\s+"
       r"(?:the\s+)?(over there|there|here|[a-z][a-z0-9_ ]{1,28})",
       lambda m: {"object": _obj(m.group(1), "it"),
                  "target": _obj(m.group(2))},
       [ARM, MOBILE], "place_on"),

    _R("open_gripper",
       r"\b(?:open|release|let go(?: of)?|ungrip)\s+(?:the\s+|your\s+)?"
       r"(?:gripper|grabber|grab bar|claw|hand|grip)\b",
       lambda m: {}, [ARM], "open_gripper"),

    _R("close_gripper",
       r"\b(?:close|shut|clamp|grip)\s+(?:the\s+|your\s+)?"
       r"(?:gripper|grabber|claw|hand|grip)\b",
       lambda m: {}, [ARM], "close_gripper"),

    _R("wave",
       r"\b(?:wave|say hello|greet|salute)"
       r"(?:\s+(?:hello|hi|goodbye|at me|to me|at the camera|to celebrate))?\b",
       lambda m: {}, [ARM, QUADRUPED], "wave"),

    # ---- quadruped ---------------------------------------------------
    _R("sit", r"\b(?:sit|sit down|lie down|crouch)\b", lambda m: {},
       [QUADRUPED], "sit"),
    _R("stand", r"\b(?:stand|stand up|get up|rise)\b", lambda m: {},
       [QUADRUPED], "stand"),
    _R("walk",
       r"\b(?:walk|trot|pace)\s*(?:forward|forwards|ahead)?\s*"
       r"(?:(" + _NUM + r")\s*(" + _LEN_UNIT + r"))?",
       lambda m: {"distance": _metres(m.group(1), m.group(2))
                  if m.group(1) else None},
       [QUADRUPED], "walk"),

    # ---- drone -------------------------------------------------------
    # `land` is this surface's `back`: an ordinary English word with a motor
    # behind it. "where did the package land?" must stay a question, which
    # the question guard settles before any rule here is tried.
    _R("takeoff",
       r"\b(?:take\s?off|takeoff|launch|lift\s?off)\b"
       r"(?:\s+(?:to|at))?\s*(?:an?\s+)?(?:altitude\s+(?:of\s+)?)?"
       r"(?:(" + _NUM + r")\s*(" + _LEN_UNIT + r")?)?",
       lambda m: {"altitude": _metres(m.group(1), m.group(2))
                  if m.group(1) else None},
       [DRONE], "takeoff"),

    _R("land",
       r"\b(?:land|touch\s?down|set\s?down)\b",
       lambda m: {}, [DRONE], "land"),

    _R("hover",
       r"\b(?:hover|station\s?keep|hold\s+(?:position|altitude))\b",
       lambda m: {}, [DRONE], "hover"),

    _R("move_body",
       r"\b(climb|ascend|go\s+up|rise|descend|drop\s+down|go\s+down|lower)\b"
       r"[^-\d]*(" + _NUM + r")\s*(" + _LEN_UNIT + r")?",
       lambda m: {"vertical": _metres(m.group(2), m.group(3))
                  * (-1 if m.group(1).lower().startswith(
                      ("desc", "drop", "go d", "lower")) else 1)},
       [DRONE], "climb"),

    _R("move_body",
       r"\b(?:fly|move|go)\s+(forward|forwards|ahead|back|backward|backwards)\b"
       r"[^-\d]*(" + _NUM + r")\s*(" + _LEN_UNIT + r")?",
       lambda m: {"forward": _metres(m.group(2), m.group(3))
                  * (-1 if m.group(1).lower().startswith("back") else 1)},
       [DRONE], "fly_forward"),

    # ---- universal ---------------------------------------------------
    # "stop what you're doing" must consume the whole clause, or the gate
    # rejects a plain stop order for being only four characters long.
    _R("stop",
       r"\b(?:stop|halt|freeze|brake|pause|cease|stand down)"
       r"(?:\s+(?:what\s+you(?:'re|\s+are)\s+doing|everything|all\s+work|"
       r"moving|driving|there|here))?\b|"
       r"\bhold\s+(?:still|position|there|on|fire)\b|"
       r"\bwait\b",
       lambda m: {}, [ANY], "stop"),

    _R("resume_autonomy",
       r"\b(?:resume|carry on|carrying on|keep (?:going|working|at it)|"
       r"continue|as you were|proceed|back to (?:work|it)|"
       r"get (?:back to|on) (?:work|it)|go back to (?:work|what you were)|"
       r"restart (?:your )?(?:work|loop|autonomy)|un-?pause)\b",
       lambda m: {}, [ANY], "resume"),

    _R("reset_to_home",
       r"\b(?:go home|return home|head home|reset to home|go back to (?:the )?start|"
       r"return to (?:the )?(?:start|dock|home)|dock yourself)\b",
       lambda m: {}, [ANY], "reset_to_home"),
]


# ── Query sub-classification ────────────────────────────────────────
# A query still deserves a routing decision: which read-only surface answers
# it. Getting this right is most of the remaining traffic.
_QUERY_KIND = [
    ("pose", re.compile(r"\b(where are you|your (?:x and y|position|pose|location)|"
                        r"where is the|where did|how far|"
                        # a drone's pose question is about height
                        r"altitude|how high|your height)\b", re.IGNORECASE)),
    ("activity", re.compile(r"\b(what (?:are|were) you (?:doing|up to)|"
                            r"what you are doing|what'?re you doing|"
                            r"current job|your (?:job|task|assignment)|"
                            r"status|sitrep|telemetry|report|"
                            r"what are you (?:working on|towing|carrying))\b",
                            re.IGNORECASE)),
    ("payload", re.compile(r"\b(carrying|towing|holding|hooked up|attached|"
                           r"got a cart)\b", re.IGNORECASE)),
    ("history", re.compile(r"\b(delivered|completed|finished|shipped|parked|"
                           r"last (?:three|two|few)|action history|"
                           r"this (?:session|shift))\b", re.IGNORECASE)),
    ("battery", re.compile(r"\b(charge|battery|power level)\b", re.IGNORECASE)),
    ("capability", re.compile(r"\b(can you reach|furthest point|could you "
                              r"actually|able to)\b", re.IGNORECASE)),
    ("scene", re.compile(r"\b(how many|is there|are there|anywhere|"
                         r"in the (?:cell|row|workspace))\b", re.IGNORECASE)),
    ("health", re.compile(r"\b(problems?|anomal|worried|okay|ok\b|"
                          r"keeping up|going well)\b", re.IGNORECASE)),
]


def _query_kind(text: str) -> str:
    for kind, rx in _QUERY_KIND:
        if rx.search(text):
            return kind
    return "state"


# ── The parser ──────────────────────────────────────────────────────
# The consumption gate. Below this share of the clause explained, a match is
# treated as a coincidence rather than an interpretation. 0.55 was chosen
# against the shipped router's own failures (`back` explains 0.11 of "put the
# block back on the grey table") and NOT tuned per-utterance on the corpus.
_GATE = 0.55


def _best_match(clause: str, surface: str) -> Optional[Tuple[Rule, re.Match]]:
    """Longest match wins, so a new rule cannot shadow an existing one."""
    best: Optional[Tuple[Rule, re.Match]] = None
    for rule in RULES:
        if not rule.applies(surface):
            continue
        m = rule.pattern.search(clause)
        if m is None:
            continue
        if best is None or len(m.group(0)) > len(best[1].group(0)):
            best = (rule, m)
    return best


def _has_deixis(args: Dict[str, Any]) -> Optional[str]:
    for key, value in args.items():
        if isinstance(value, str) and value.strip().lower() in _DEIXIS:
            return f"{key}={value!r}"
    return None


def _parse_clause(clause: str, surface: str) -> Tuple[Optional[Frame], str, str]:
    """-> (frame, why_not, residue). Exactly one of frame / why_not is set."""
    found = _best_match(clause, surface)
    if found is None:
        return None, "no rule matched", clause
    rule, m = found

    core = _core(clause)
    matched = _core(m.group(0))
    share = len(matched) / max(1, len(core))
    if share < _GATE:
        return (None,
                f"{rule.name} matched {matched!r} but that explains only "
                f"{share:.0%} of the clause",
                core[len(matched):].strip() or core)

    try:
        args = rule.build(m)
    except (ValueError, IndexError, TypeError) as exc:   # slot did not convert
        return None, f"{rule.name} slots invalid: {exc}", clause

    args = {k: v for k, v in args.items() if v is not None}
    deictic = _has_deixis(args)
    if deictic:
        return None, f"{rule.name} needs a referent for {deictic}", clause

    sane = _implausible(args, clause)
    if sane:
        # ⚠️ Prefixed so the caller routes this to AMBIGUOUS, which the
        # executor ANSWERS. A low-confidence CONVERSATION returns None, and
        # the keyword ladder each bridge used to carry behind the parser
        # then drove the 500 m the parser had just refused: the decline was
        # right and the fallthrough undid it. The ladders were deleted on
        # 2026-09-22, but the principle outlives them and is why this is
        # still not a None: a safety refusal must never be a fallthrough.
        return None, f"needs confirmation: {sane}", clause

    return Frame(rule.tool, args, m.span(), clause, rule.name), "", ""


def _implausible(args: Dict[str, Any], clause: str) -> Optional[str]:
    """Reject a slot that parsed cleanly but cannot have been meant.

    commandbench bounds-1: "drive forward 500 metres" parsed perfectly and
    drove 14.4 m before the run ended, leaving the arena. A parser cannot
    know the world, but it can know that no operator means 500 m from a
    tug -- and that a direction and a sign should agree.
    """
    d = args.get("distance")
    if d is not None and abs(d) > MAX_PLAUSIBLE_M:
        return (f"{abs(d):.0f} m is far beyond one order's worth of travel; "
                f"confirm it, or give me a coordinate")
    a = args.get("angle_rad")
    if a is not None and abs(a) > MAX_PLAUSIBLE_RAD:
        return f"{abs(a):.1f} rad is more than four turns; confirm it"
    if d is not None and d < 0 and re.search(
            r"\b(?:forward|forwards|ahead)\b", clause, re.IGNORECASE):
        return ("the direction says forward but the distance is negative; "
                "say which you meant")
    return None


SURFACES = (MOBILE, ARM, QUADRUPED, DRONE, ANY)


class UnknownSurface(ValueError):
    """`surface` was not one of SURFACES. See interpret()'s docstring."""


def interpret(text: str, surface: str = MOBILE) -> Interpretation:
    """Parse one operator utterance. Never raises ON THE TEXT; declines instead.

    ⚠️ IT DOES RAISE ON A BAD `surface`, AND THAT IS DELIBERATE.

    This argument used to accept anything. A rule only fires on a surface
    it applies to, so an unrecognised string matches no rule, and every
    utterance falls through to conversation / confidence 0.3 / no frames.
    That is indistinguishable from a parser that simply cannot understand
    the sentence.

    It cost a published result. `interpbench` was run with
    surface="MOBILE" -- uppercase -- and reported the deterministic parser
    at 68.3% with 19 missed commands, against the model's 91.7%, and
    concluded the model was the better interpreter by 23 points. With the
    case corrected the parser scores 90.0% with ONE missed command and the
    gap is 1.7 points. The measurement was of a typo.

    Declining silently is the right behaviour for a sentence nobody can
    parse and the wrong behaviour for a caller who named a robot class
    that does not exist -- one is a fact about the world, the other is a
    bug, and returning the same value for both is how the bug survived.
    Generalizing across four surfaces multiplies the exposure by four, so
    the trap is closed before the fan-out rather than after it.
    """
    if surface not in SURFACES:
        raise UnknownSurface(
            f"surface={surface!r} is not one of {list(SURFACES)}. "
            f"An unknown surface matches no rule and would degrade every "
            f"parse to conversation/no-frames, which reads as a parser "
            f"that understood nothing. Did you mean {str(surface).lower()!r}?")
    result = _interpret(text, surface)
    result.text = text or ""
    return result


def _interpret(text: str, surface: str = MOBILE) -> Interpretation:
    """The parse itself. Wrapped so every return path carries its text."""
    raw = _normalise(text)
    if not raw:
        return Interpretation(EMPTY, reason="nothing was said")

    low = raw.lower()

    # -- Guard 1: machine text is not an utterance -------------------
    if low.startswith(("[", "tool-output:")) or "```" in low:
        return Interpretation(CONVERSATION, reason="platform-injected text, "
                              "not an operator utterance", confidence=1.0)

    # -- Guard 2: challenges to the robot's own account --------------
    # Checked before everything, because these sentences are FULL of motion
    # verbs ("you never stopped") that mean the opposite of a command.
    if _SOCIAL.search(low):
        return Interpretation(CONVERSATION, confidence=0.9,
                              reason="a challenge to the robot's own account; "
                                     "only a model may answer it")

    # -- Guard 3: something to remember ------------------------------
    if _MEMORY_CUE.search(low):
        return Interpretation(MEMORY, reason="states a fact or policy to keep",
                              residue=raw, confidence=0.8)

    # -- Guard 4: question form --------------------------------------
    # Three things have to be told apart, and the shipped ladder told apart
    # none of them:
    #   "can you stop"                  polite IMPERATIVE -> command
    #   "can you tell me where you are" polite QUESTION   -> query
    #   "when you've finished, pause"   SUBORDINATE clause -> not a question,
    #                                   even though it opens with a wh-word
    polite = bool(_POLITE_IMPERATIVE.match(raw))
    depoliced = _POLITE_IMPERATIVE.sub("", raw).strip() if polite else raw
    ends_q = raw.rstrip().endswith("?")
    # "do not take any new pickups" opens with `do`, which the question head
    # matches. A prohibition is never a question, and misreading one turns a
    # standing restriction into a status report.
    subordinate_head = bool(_SUBORDINATOR.match(raw)) or bool(_PROHIBITION.match(raw))

    def _as_query() -> Interpretation:
        kind = _query_kind(low)
        return Interpretation(
            QUERY, reason=f"question about {kind}"
            + (" (aggregate)" if _AGGREGATE.search(low) else ""),
            residue=raw, confidence=0.85,
            frames=[Frame("get_robot_state", {"kind": kind}, (0, len(raw)),
                          raw, "query")])

    if not subordinate_head:
        # An explicit hypothetical marker beats the polite-imperative rule:
        # "could you drive forward 2 metres, IN PRINCIPLE?" asks whether the
        # robot can, and the parser drove 2 m answering it.
        if _HYPOTHETICAL.search(low):
            return _as_query()
        # "tell me / show me ..." is an imperative in form and a question in
        # effect; answering it with a motion would be the ladder's mistake.
        if _TELL_ME.match(depoliced):
            return _as_query()
        if _QUESTION_HEAD.match(raw) and not polite:
            return _as_query()

    # -- Guard 5: prohibitions are standing constraints --------------
    if _PROHIBITION.search(low):
        return Interpretation(CONSTRAINT, reason="a standing restriction, not "
                              "a one-off action", residue=raw, confidence=0.8,
                              trigger=raw)

    # -- Guard 5b: an order that negates itself ----------------------
    # "drive forward 1 metre without moving" is not a command with a
    # caveat, it is incoherent -- and the parser drove the metre. Note the
    # negated verbs are motion verbs only, so "drive to the bay without
    # stopping" is left alone.
    if _SELF_NEGATING.search(low):
        return Interpretation(
            AMBIGUOUS, confidence=0.6, residue=raw,
            reason="that asks me to move and not move at once; say which")

    # -- Guard 6: a delay is a schedule, not an order for now --------
    delay = _DELAY.search(raw)
    if delay:
        return Interpretation(DEFERRED, trigger=delay.group(0),
                              reason=f"scheduled for {delay.group(0)}",
                              residue=raw, confidence=0.8)

    # -- Guard 7: a destination only the past can resolve ------------
    if _PAST_PLACE.search(raw):
        return Interpretation(
            AMBIGUOUS, confidence=0.7, residue=raw,
            reason="I would have to remember where that was; give me a "
                   "coordinate, or tell me to reset to home")

    # -- Guard 8: a retraction cancels what preceded it --------------
    # "drive forward 2 metres - no wait, make it 1 metre" drove 2 m: the
    # head won and the correction was residue. Running both would have
    # driven 3. The TAIL governs.
    # The FIRST marker, not the last: "make it" is itself a retraction
    # marker, so taking the last one ate the head of "drive 2 m - no wait,
    # make it 1 m" and left nothing to revise. A second retraction inside
    # the tail is handled by the recursive call below.
    retraction = _RETRACTION.search(raw)
    if retraction is not None:
        head = raw[:retraction.start()].strip(" ,.-")
        tail = raw[retraction.end():].strip(" ,.-")
        if not tail:
            # A trailing marker ("...3 metres INSTEAD") only flags that this
            # replaces something earlier; the head is still the order.
            return (interpret(head, surface) if head else
                    Interpretation(EMPTY, reason="nothing was said"))
        if _CANCEL_TAIL.search(tail):
            return Interpretation(
                CONVERSATION, confidence=0.85, residue=raw,
                reason="retracted, and the replacement cancels the order")
        sub = interpret(tail, surface)
        if sub.intent == COMMAND and sub.frames:
            return Interpretation(COMMAND, frames=sub.frames,
                                  confidence=min(sub.confidence, 0.85),
                                  reason=f"retracted; {sub.reason}",
                                  residue=sub.residue)
        # The tail may be only a new quantity -- "make it 1 metre" -- in
        # which case it re-fills the head's slot rather than replacing it.
        q = _REQUANTIFY.search(tail)
        base = interpret(head, surface) if head else None
        if q and base is not None and base.intent == COMMAND and base.frames:
            unit = q.group(2)
            frame = base.frames[-1]
            if re.fullmatch(_LEN_UNIT, unit, re.IGNORECASE) and \
                    "distance" in frame.args:
                old = frame.args["distance"] or 1.0
                frame.args["distance"] = math.copysign(
                    _metres(q.group(1), unit), old)
            elif re.fullmatch(_ANG_UNIT, unit, re.IGNORECASE) and \
                    "angle_rad" in frame.args:
                old = frame.args["angle_rad"] or 1.0
                frame.args["angle_rad"] = math.copysign(
                    _radians(q.group(1), unit), old)
            else:
                return Interpretation(
                    AMBIGUOUS, confidence=0.5, residue=raw,
                    reason="a correction I could not apply to the order")
            return Interpretation(COMMAND, frames=base.frames, confidence=0.8,
                                  reason=f"retracted; {base.reason} (revised)")
        return Interpretation(
            AMBIGUOUS, confidence=0.5, residue=raw,
            reason="something was retracted and I could not tell what "
                   "replaces it")

    # -- Parse the clauses -------------------------------------------
    frames: List[Frame] = []
    problems: List[str] = []
    residues: List[str] = []
    trigger: Optional[str] = None
    deferred = False
    hold = False

    for clause in _clauses(depoliced or raw):
        sub = _SUBORDINATOR.search(clause)
        if sub:
            # Everything from the subordinator on is a CONDITION. The
            # imperative inside it ("...until I tell you to continue") is
            # emphatically not an order to execute now.
            head = clause[:sub.start()].strip()
            cond = clause[sub.start():].strip()
            if _HOLD_TRIGGER.search(cond):
                # Not a trigger to schedule: an indefinite hold to apply NOW.
                hold = True
            else:
                trigger = cond
                deferred = True
            if not head:
                continue
            clause = head

        frame, why, residue = _parse_clause(clause, surface)
        if frame is not None:
            frames.append(frame)
        else:
            problems.append(why)
            if residue:
                residues.append(residue)

    # "stop what you're doing and wait" is two clauses saying one thing.
    # Emitting stop() twice is harmless but it misreports what was understood.
    #
    # ⚠️ ADJACENT ONLY. The first version deduped globally, which silently
    # dropped the second leg of "drive forward 1 m, turn left 90, then drive
    # forward 1 m" -- the two drives are identical frames but a genuine
    # sequence, and the robot finished a metre short. Caught by commandbench
    # comp-3, which judges the pose rather than the reply.
    deduped: List[Frame] = []
    for f in frames:
        if deduped:
            prev = deduped[-1]
            if prev.tool == f.tool and prev.args == f.args:
                continue
        deduped.append(f)
    frames = deduped

    residue_text = "; ".join(r for r in residues if r)

    if frames and deferred:
        # A release condition does not override the start condition:
        # "after this delivery, stop until I tell you" must finish the job.
        return Interpretation(DEFERRED, frames=frames, trigger=trigger,
                              reason="an action with a trigger", confidence=0.8,
                              residue=residue_text)

    if hold:
        # The whole point of recognising this: act now AND disable the 60 s
        # auto-resume, which is the behaviour that used to walk a "paused"
        # tug straight back into a forbidden cell.
        frames.append(Frame("hold", {"release": "operator"},
                            (0, len(raw)), raw, "hold"))
        return Interpretation(COMMAND, frames=frames, confidence=0.9,
                              reason="act now, then hold until the operator "
                                     "releases (no auto-resume)",
                              residue=residue_text)

    if frames:
        # Confidence falls when part of the utterance went unexplained: the
        # caller can require a higher bar before acting without confirming.
        conf = 0.95 if not residues else max(0.5, 0.95 - 0.15 * len(residues))
        return Interpretation(COMMAND, frames=frames, confidence=conf,
                              reason="; ".join(f.rule for f in frames),
                              residue=residue_text)
    if deferred and trigger:
        return Interpretation(DEFERRED, trigger=trigger, residue=residue_text,
                              reason="a trigger with no action this parser "
                                     "could fill", confidence=0.4)

    # Understood the verb but not the object -> worth asking about.
    ask_worthy = [p for p in problems
                  if "needs a referent" in p or "needs confirmation" in p]
    if ask_worthy:
        return Interpretation(AMBIGUOUS, reason=ask_worthy[0],
                              residue=residue_text or raw, confidence=0.6)

    # A question mark is evidence even when the body parsed as nothing: a
    # polite question ("can you tell me what you are doing now?") reaches
    # here only because the politeness guard let it through to be parsed.
    if ends_q:
        return _as_query()

    return Interpretation(CONVERSATION, reason="; ".join(problems) or
                          "no rule matched", residue=raw, confidence=0.3)
